#include "gcs_planner/planner_node.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace gcs_planner {

PlannerNode::PlannerNode() : rclcpp::Node("planner") {
    R_          = declare_parameter("region_radius",  5.0);
    v_max_      = declare_parameter("vel_limit",      2.0);
    degree_     = declare_parameter("bezier_degree",  6);
    continuity_ = declare_parameter("continuity",     2);
    max_step_   = declare_parameter("region_spacing", 2.0);  // unused by parallel corridor builder
    sample_dt_  = declare_parameter("sample_dt",      0.05);
    num_threads_  = declare_parameter("num_threads",   10);
    max_rounds_   = declare_parameter("max_rounds",    50);
    corridor_seed_ = declare_parameter<int64_t>("corridor_seed", 0);
    frame_id_   = declare_parameter<std::string>("frame_id", "map");
    pcd_file_   = declare_parameter<std::string>("pcd_file", "");

    auto flat = declare_parameter<std::vector<double>>("nominal_path", std::vector<double>{});
    for (size_t i = 0; i + 2 < flat.size(); i += 3) {
        Eigen::VectorXd p(3);
        p << flat[i], flat[i + 1], flat[i + 2];
        nominal_path_.push_back(p);
    }

    auto latched = rclcpp::QoS(1).transient_local();
    cloud_pub_        = create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", latched);
    voxels_pub_       = create_publisher<visualization_msgs::msg::MarkerArray>("map_voxels", latched);
    nominal_path_pub_ = create_publisher<nav_msgs::msg::Path>("nominal_path", latched);
    corridor_pub_     = create_publisher<visualization_msgs::msg::MarkerArray>("corridor", latched);
    traj_path_pub_    = create_publisher<nav_msgs::msg::Path>("trajectory_path", latched);
    traj_pub_         = create_publisher<planner_msgs::msg::Trajectory>("trajectory", 10);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar_points", rclcpp::SensorDataQoS(),
        std::bind(&PlannerNode::on_cloud, this, std::placeholders::_1));

    // Publish the map frame into TF so RViz can render all displays.
    // Using a dummy child frame; the parent (frame_id_) becomes the TF root.
    tf2_ros::StaticTransformBroadcaster static_tf(*this);
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = frame_id_;
    tf_msg.child_frame_id = frame_id_ + "_static_root";
    tf_msg.transform.rotation.w = 1.0;
    static_tf.sendTransform(tf_msg);

    RCLCPP_INFO(get_logger(), "PlannerNode ready (pcd_file='%s', %zu waypoints)",
                pcd_file_.c_str(), nominal_path_.size());
}

void PlannerNode::startup() {
    if (!pcd_file_.empty() && load_pcd(pcd_file_)) {
        run_pipeline();
    } else {
        RCLCPP_WARN(get_logger(), "no pcd_file (or load failed); waiting on lidar_points");
    }
}

void PlannerNode::on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(cloud_mtx_);
    last_cloud_ = msg;
}

bool PlannerNode::load_pcd(const std::string& file) {
    RCLCPP_INFO(get_logger(), "loading PCD: %s", file.c_str());
    pcl::PointCloud<pcl::PointXYZ> pc;
    if (pcl::io::loadPCDFile(file, pc) < 0) {
        RCLCPP_ERROR(get_logger(), "failed to load PCD: %s", file.c_str());
        return false;
    }
    RCLCPP_INFO(get_logger(), "loaded %u points, downsampling...", pc.size());

    pcl::PointCloud<pcl::PointXYZ> pc_ds;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(pc.makeShared());
    vg.setLeafSize(0.5f, 0.5f, 0.5f);
    vg.filter(pc_ds);
    RCLCPP_INFO(get_logger(), "downsampled to %u points (0.5 m voxels)", pc_ds.size());

    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(pc_ds, *msg);
    msg->header.frame_id = frame_id_;
    msg->header.stamp = now();
    {
        std::lock_guard<std::mutex> lk(cloud_mtx_);
        last_cloud_ = msg;
    }
    cloud_pub_->publish(*msg);

    // Build voxel cube markers — one CUBE_LIST, one cube per downsampled point.
    visualization_msgs::msg::Marker cube_marker;
    cube_marker.header.frame_id = frame_id_;
    cube_marker.header.stamp = now();
    cube_marker.ns = "map_voxels";
    cube_marker.id = 0;
    cube_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    cube_marker.action = visualization_msgs::msg::Marker::ADD;
    cube_marker.scale.x = 0.5;
    cube_marker.scale.y = 0.5;
    cube_marker.scale.z = 0.5;
    cube_marker.color.r = 0.6f;
    cube_marker.color.g = 0.6f;
    cube_marker.color.b = 0.6f;
    cube_marker.color.a = 0.7f;
    cube_marker.pose.orientation.w = 1.0;
    cube_marker.points.reserve(pc_ds.size());
    for (const auto& pt : pc_ds) {
        geometry_msgs::msg::Point p;
        p.x = pt.x; p.y = pt.y; p.z = pt.z;
        cube_marker.points.push_back(p);
    }
    last_voxels_.markers = {cube_marker};
    voxels_pub_->publish(last_voxels_);

    RCLCPP_INFO(get_logger(), "published map cloud (%u points)", pc_ds.size());
    return true;
}

void PlannerNode::run_pipeline() {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    {
        std::lock_guard<std::mutex> lk(cloud_mtx_);
        cloud = last_cloud_;
    }
    if (!cloud || nominal_path_.size() < 2) {
        RCLCPP_WARN(get_logger(), "need a cloud and >=2 nominal_path points");
        return;
    }

    nominal_path_pub_->publish(make_path_msg(nominal_path_));

    using Clock = std::chrono::steady_clock;
    using Ms = std::chrono::duration<double, std::milli>;
    auto elapsed = [](Clock::time_point t0) {
        return std::chrono::duration_cast<Ms>(Clock::now() - t0).count();
    };

    RCLCPP_INFO(get_logger(), "converting cloud to Eigen...");
    auto t0 = Clock::now();
    Eigen::MatrixXd P = cloud_to_eigen(*cloud);
    RCLCPP_INFO(get_logger(), "cloud: %ld points  [%.1f ms]; building corridor (R=%.1f, %d threads)...",
                P.rows(), elapsed(t0), R_, num_threads_);

    gcs::CorridorOptions copt;
    copt.R            = R_;
    copt.num_threads  = num_threads_;
    copt.max_rounds   = max_rounds_;
    copt.sphere_floor = true;
    copt.seed         = static_cast<uint64_t>(corridor_seed_);

    t0 = Clock::now();
    auto cr = gcs::build_corridor_parallel(nominal_path_.front(), nominal_path_.back(),
                                           nominal_path_, P, copt);
    auto regions = std::move(cr.regions);
    RCLCPP_INFO(get_logger(), "built corridor: %zu regions in %d round(s), connected=%s  [%.1f ms]",
                regions.size(), cr.rounds, cr.connected ? "true" : "false", elapsed(t0));

    if (regions.empty()) {
        RCLCPP_WARN(get_logger(), "no regions grown");
        return;
    }
    if (!cr.connected) {
        RCLCPP_WARN(get_logger(),
                    "corridor did not connect q0->qT after %d rounds; attempting plan anyway",
                    cr.rounds);
    }
    corridor_pub_->publish(make_corridor_markers(regions));

    try {
        gcs::GCSCompositeBezierPlanner planner(regions);

        t0 = Clock::now();
        planner.build_graph();
        RCLCPP_INFO(get_logger(), "graph built  [%.1f ms]", elapsed(t0));

        const int d = static_cast<int>(P.cols());
        gcs::GCSCompositeBezierPlanner::Options opt;
        opt.degree = degree_;
        opt.continuity = continuity_;
        opt.vel_limit = v_max_;
        opt.v0 = Eigen::VectorXd::Zero(d);
        opt.vT = Eigen::VectorXd::Zero(d);
        opt.rest_to_rest_accel = true;

        t0 = Clock::now();
        auto traj = planner.plan_composite(nominal_path_.front(), nominal_path_.back(), opt);
        RCLCPP_INFO(get_logger(), "trajectory optimized  [%.1f ms]", elapsed(t0));

        std_msgs::msg::Header hdr;
        hdr.stamp = now();
        hdr.frame_id = frame_id_;
        traj_pub_->publish(to_msg(traj, hdr));
        traj_path_pub_->publish(make_traj_path(traj, hdr));
        RCLCPP_INFO(get_logger(), "published trajectory: T=%.2fs", traj.total_duration());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "planning failed: %s", e.what());
    }

    // Re-publish cloud, voxels, and nominal path: the corridor takes minutes to compute,
    // so by the time we reach here RViz is definitely subscribed.
    {
        std::lock_guard<std::mutex> lk(cloud_mtx_);
        if (last_cloud_) cloud_pub_->publish(*last_cloud_);
    }
    if (!last_voxels_.markers.empty()) voxels_pub_->publish(last_voxels_);
    nominal_path_pub_->publish(make_path_msg(nominal_path_));
}

Eigen::MatrixXd PlannerNode::cloud_to_eigen(const sensor_msgs::msg::PointCloud2& msg) const {
    sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z");
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(msg.width * msg.height);
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
        if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) continue;
        pts.emplace_back(*ix, *iy, *iz);
    }
    Eigen::MatrixXd M(static_cast<int>(pts.size()), 3);
    for (size_t i = 0; i < pts.size(); ++i) {
        M.row(static_cast<int>(i)) = pts[i].transpose();
    }
    return M;
}

planner_msgs::msg::Trajectory
PlannerNode::to_msg(const gcs::CompositeTimingTrajectory& tr,
                    const std_msgs::msg::Header& hdr) const {
    planner_msgs::msg::Trajectory out;
    out.header = hdr;
    const double T = tr.total_duration();
    out.total_duration = T;
    for (double t = 0.0; t <= T + 1e-9; t += sample_dt_) {
        const double tc = std::min(t, T);
        Eigen::VectorXd p = tr.position(tc);
        Eigen::VectorXd v = tr.velocity(tc);
        Eigen::VectorXd a = tr.acceleration(tc);

        planner_msgs::msg::TrajectoryPoint pt;
        pt.time_from_start = rclcpp::Duration::from_seconds(tc);
        pt.position.x = p(0); pt.position.y = p(1); pt.position.z = p(2);
        pt.velocity.x = v(0); pt.velocity.y = v(1); pt.velocity.z = v(2);
        pt.acceleration.x = a(0); pt.acceleration.y = a(1); pt.acceleration.z = a(2);
        pt.yaw   = std::atan2(v(1), v(0));
        pt.pitch = std::atan2(v(2), std::hypot(v(0), v(1)));
        pt.speed = v.norm();
        out.points.push_back(pt);
    }
    return out;
}

nav_msgs::msg::Path PlannerNode::make_path_msg(const Path& pts) const {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = now();
    for (const auto& p : pts) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = msg.header;
        ps.pose.position.x = p(0);
        ps.pose.position.y = p(1);
        ps.pose.position.z = (p.size() > 2) ? p(2) : 0.0;
        ps.pose.orientation.w = 1.0;
        msg.poses.push_back(ps);
    }
    return msg;
}

nav_msgs::msg::Path
PlannerNode::make_traj_path(const gcs::CompositeTimingTrajectory& tr,
                            const std_msgs::msg::Header& hdr) const {
    nav_msgs::msg::Path msg;
    msg.header = hdr;
    const double T = tr.total_duration();
    for (double t = 0.0; t <= T + 1e-9; t += sample_dt_) {
        Eigen::VectorXd p = tr.position(std::min(t, T));
        geometry_msgs::msg::PoseStamped ps;
        ps.header = hdr;
        ps.pose.position.x = p(0);
        ps.pose.position.y = p(1);
        ps.pose.position.z = p(2);
        ps.pose.orientation.w = 1.0;
        msg.poses.push_back(ps);
    }
    return msg;
}

visualization_msgs::msg::MarkerArray
PlannerNode::make_corridor_markers(const std::vector<gcs::ConvexRegion>& regions) const {
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    for (const auto& reg : regions) {
        Eigen::MatrixXd V = gcs::region_vertices(reg);
        if (V.rows() == 0) continue;
        auto edges = gcs::region_edges(reg, V);

        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_id_;
        m.header.stamp = now();
        m.ns = "corridor";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::LINE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.05;
        m.color.r = 0.2f; m.color.g = 0.6f; m.color.b = 1.0f; m.color.a = 0.8f;
        m.pose.orientation.w = 1.0;

        for (const auto& e : edges) {
            geometry_msgs::msg::Point a, b;
            a.x = V(e[0], 0); a.y = V(e[0], 1); a.z = (V.cols() > 2) ? V(e[0], 2) : 0.0;
            b.x = V(e[1], 0); b.y = V(e[1], 1); b.z = (V.cols() > 2) ? V(e[1], 2) : 0.0;
            m.points.push_back(a);
            m.points.push_back(b);
        }
        arr.markers.push_back(m);
    }
    return arr;
}

}  // namespace gcs_planner