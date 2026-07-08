#include "gcs_planner/planner_node.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace gcs_planner {

namespace {

std::string vehicle_key_from_csv_path(const std::string& file) {
    std::string base = file;
    const auto slash = base.find_last_of('/');
    if (slash != std::string::npos) base = base.substr(slash + 1);
    const auto dot = base.find_last_of('.');
    if (dot != std::string::npos) base = base.substr(0, dot);
    const std::string prefix = "path_";
    if (base.rfind(prefix, 0) == 0) base = base.substr(prefix.size());
    return base;
}

constexpr double g = 9.80665; 

void attitude_from_accel_yaw(const Eigen::Vector3d& accel, double yaw,
                             double& roll, double& pitch) {
    const Eigen::Vector3d gravity(0.0, 0.0, -g);
    Eigen::Vector3d z_b = accel - gravity;
    const double z_b_norm = z_b.norm();
    if (z_b_norm < 1e-6) {
        roll = 0.0; pitch = 0.0;
        return;
    }
    z_b /= z_b_norm;

    const Eigen::Vector3d x_c(std::cos(yaw), std::sin(yaw), 0.0);
    Eigen::Vector3d y_b = z_b.cross(x_c);
    const double y_b_norm = y_b.norm();
    if (y_b_norm < 1e-6) {
        roll = 0.0; pitch = 0.0;
        return;
    }
    y_b /= y_b_norm;
    const Eigen::Vector3d x_b = y_b.cross(z_b);

    const tf2::Matrix3x3 R(x_b.x(), y_b.x(), z_b.x(),
                           x_b.y(), y_b.y(), z_b.y(),
                           x_b.z(), y_b.z(), z_b.z());
    double roll_out, pitch_out, yaw_out;
    R.getRPY(roll_out, pitch_out, yaw_out);
    roll = roll_out;
    pitch = pitch_out;
}

void trajectory_point_quaternion(const planner_msgs::msg::TrajectoryPoint& pt,
                                 double& qx, double& qy, double& qz, double& qw) {
    tf2::Quaternion q;
    q.setRPY(pt.roll, pt.pitch, pt.yaw);
    qx = q.x(); qy = q.y(); qz = q.z(); qw = q.w();
}

gcs::CompositeTimingTrajectory make_vertical_trajectory(const Eigen::VectorXd& p0,
                                                        const Eigen::VectorXd& p1,
                                                        double v_max, double a_max) {
    constexpr int N = 5;
    const int d = static_cast<int>(p0.size());
    const double dist = (p1 - p0).norm();
    constexpr double kQuinticPeakVelFactor = 15.0 / 8.0;
    const double kQuinticPeakAccelFactor = 10.0 / std::sqrt(3.0);
    const double T_vel   = kQuinticPeakVelFactor * dist / std::max(v_max, 1e-3);
    const double T_accel = std::sqrt(kQuinticPeakAccelFactor * dist / std::max(a_max, 1e-3));
    const double T = std::max({0.05, T_vel, T_accel});

    Eigen::MatrixXd Gq(N + 1, d);
    Gq.row(0) = p0.transpose(); Gq.row(1) = p0.transpose(); Gq.row(2) = p0.transpose();
    Gq.row(3) = p1.transpose(); Gq.row(4) = p1.transpose(); Gq.row(5) = p1.transpose();

    Eigen::MatrixXd Gh(N + 1, 1);
    for (int n = 0; n <= N; ++n) Gh(n, 0) = T * n / N;

    gcs::CompositeTimingTrajectory traj;
    traj.degree = N;
    traj.q_cps = {Gq};
    traj.h_cps = {Gh};
    return traj;
}

}  // namespace

PlannerNode::PlannerNode() : rclcpp::Node("planner") {
    R_          = declare_parameter("region_radius",  5.0);
    v_max_      = declare_parameter("vel_limit",      2.0);
    degree_     = declare_parameter("bezier_degree",  6);
    continuity_ = declare_parameter("continuity",     2);
    sample_dt_  = declare_parameter("sample_dt",      0.05);
    num_threads_  = declare_parameter("num_threads",   10);
    max_rounds_   = declare_parameter("max_rounds",    50);
    drop_covered_samples_ = declare_parameter("drop_covered_samples", true);
    corridor_seed_ = declare_parameter<int64_t>("corridor_seed", 0);
    frame_id_   = declare_parameter<std::string>("frame_id", "map");
    pcd_file_   = declare_parameter<std::string>("pcd_file", "");
    pcd_yaw_deg_ = declare_parameter("pcd_yaw_deg", 0.0);
    ground_band_half_width_  = declare_parameter("ground_band_half_width", 0.2);
    ground_plane_fit_radius_ = declare_parameter("ground_plane_fit_radius", R_);
    ground_plane_z_window_   = declare_parameter("ground_plane_z_window", 1.0);
    takeoff_landing_height_  = declare_parameter("takeoff_landing_height", 1.5);
    takeoff_landing_accel_limit_ = declare_parameter("takeoff_landing_accel_limit", 4.0);
    republish_period_sec_    = declare_parameter("republish_period_sec", 2.0);
    traj_output_dir_ = declare_parameter<std::string>("traj_output_dir", "");
    w_geom_accel_ = declare_parameter("w_geom_accel", 1e-2);
    w_time_accel_ = declare_parameter("w_time_accel", 1e-2);
    w_geom_jerk_  = declare_parameter("w_geom_jerk",  0.0);
    w_time_        = declare_parameter("w_time",        1.0);
    w_geom_length_ = declare_parameter("w_geom_length", 0.0);

    auto csv_files = declare_parameter<std::vector<std::string>>(
        "nominal_path_csv_files", std::vector<std::string>{});
    auto flat = declare_parameter<std::vector<double>>("nominal_path", std::vector<double>{});

    if (!csv_files.empty()) {
        if (!flat.empty()) {
            RCLCPP_WARN(get_logger(),
                        "both nominal_path_csv_files and nominal_path given; "
                        "using nominal_path_csv_files");
        }
        std::set<std::string> seen_keys;
        for (const auto& file : csv_files) {
            VehiclePlan vp;
            vp.vehicle = vehicle_key_from_csv_path(file);
            if (!seen_keys.insert(vp.vehicle).second) {
                throw std::runtime_error(
                    "duplicate vehicle key '" + vp.vehicle + "' derived from nominal_path_csv_files "
                    "entry '" + file + "'; rename the file(s) so their keys are distinct");
            }
            vp.segments = load_path_segments_csv({file});
            vehicles_.push_back(std::move(vp));
        }
    } else {
        Path pts;
        for (size_t i = 0; i + 2 < flat.size(); i += 3) {
            Eigen::VectorXd p(3);
            p << flat[i], flat[i + 1], flat[i + 2];
            pts.push_back(p);
        }
        if (!pts.empty()) {
            VehiclePlan vp;
            vp.segments.push_back({"", std::move(pts)});
            vehicles_.push_back(std::move(vp));
        }
    }

    for (auto& vp : vehicles_) {
        resegment_transitions(vp);
        for (const auto& seg : vp.segments)
            vp.nominal_path.insert(vp.nominal_path.end(), seg.waypoints.begin(), seg.waypoints.end());
    }

    auto latched = rclcpp::QoS(1).transient_local();
    cloud_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", latched);
    voxels_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("map_voxels", latched);

    for (auto& vp : vehicles_) {
        const std::string ns = vp.vehicle.empty() ? std::string() : ("/" + vp.vehicle);
        vp.nominal_path_pub  = create_publisher<nav_msgs::msg::Path>("nominal_path" + ns, latched);
        vp.corridor_pub      = create_publisher<visualization_msgs::msg::MarkerArray>("corridor" + ns, latched);
        vp.traj_path_pub     = create_publisher<nav_msgs::msg::Path>("trajectory_path" + ns, latched);
        vp.traj_segments_pub = create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_segments" + ns, latched);
        vp.traj_pub          = create_publisher<planner_msgs::msg::Trajectory>("trajectory" + ns, 10);
    }

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar_points", rclcpp::SensorDataQoS(),
        std::bind(&PlannerNode::on_cloud, this, std::placeholders::_1));

    republish_timer_ = create_wall_timer(
        std::chrono::duration<double>(republish_period_sec_),
        std::bind(&PlannerNode::publish_all, this));

    tf2_ros::StaticTransformBroadcaster static_tf(*this);
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = frame_id_;
    tf_msg.child_frame_id = frame_id_ + "_static_root";
    tf_msg.transform.rotation.w = 1.0;
    static_tf.sendTransform(tf_msg);

    size_t total_waypoints = 0, total_segments = 0;
    for (const auto& vp : vehicles_) {
        total_waypoints += vp.nominal_path.size();
        total_segments += vp.segments.size();
    }
    RCLCPP_INFO(get_logger(),
                "PlannerNode ready (pcd_file='%s', %zu vehicle(s), %zu waypoints, %zu segment(s) total)",
                pcd_file_.c_str(), vehicles_.size(), total_waypoints, total_segments);
}

void PlannerNode::startup() {
    if (!pcd_file_.empty() && load_pcd(pcd_file_)) {
        run_pipeline();
    } else {
        RCLCPP_WARN(get_logger(), "no pcd_file (or load failed); waiting on lidar_points");
    }
}

void PlannerNode::on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    last_cloud_ = msg;
}

bool PlannerNode::load_pcd(const std::string& file) {
    RCLCPP_INFO(get_logger(), "loading PCD: %s", file.c_str());
    pcl::PointCloud<pcl::PointXYZ> pc;
    if (pcl::io::loadPCDFile(file, pc) < 0) {
        RCLCPP_ERROR(get_logger(), "failed to load PCD: %s", file.c_str());
        return false;
    }
    RCLCPP_INFO(get_logger(), "loaded %u points", pc.size());

    if (pcd_yaw_deg_ != 0.0) {
        const double yaw_rad = pcd_yaw_deg_ * M_PI / 180.0;
        const float cy = static_cast<float>(std::cos(yaw_rad));
        const float sy = static_cast<float>(std::sin(yaw_rad));
        for (auto& pt : pc.points) {
            const float nx = cy * pt.x - sy * pt.y;
            const float ny = sy * pt.x + cy * pt.y;
            pt.x = nx; pt.y = ny;
        }
        RCLCPP_INFO(get_logger(), "rotated PCD %.1f deg around Z", pcd_yaw_deg_);
    }

    RCLCPP_INFO(get_logger(), "downsampling...");

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
        std::lock_guard<std::mutex> lk(state_mtx_);
        last_cloud_ = msg;
    }
    cloud_pub_->publish(*msg);

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

void PlannerNode::resegment_transitions(VehiclePlan& vp) const {
    auto& segs = vp.segments;
    for (size_t i = 0; i < segs.size(); ++i) {
        if (segs[i].tag != "TRANS") continue;
        if (i == 0 || i + 1 >= segs.size()) {
            RCLCPP_WARN(get_logger(),
                        "vehicle '%s' segment %zu (TRANS): no neighboring segment on one side, "
                        "leaving as-authored", vp.vehicle.c_str(), i);
            continue;
        }

        auto& prev = segs[i - 1];
        auto& next = segs[i + 1];
        if (prev.waypoints.empty() || next.waypoints.empty()) continue;

        if (prev.tag == "GROUND" && next.tag == "AERIAL") {
            const Eigen::VectorXd bottom = prev.waypoints.back();
            Eigen::VectorXd up = Eigen::VectorXd::Zero(bottom.size());
            up(2) = takeoff_landing_height_;
            const Eigen::VectorXd top = bottom + up;
            segs[i].waypoints = {bottom, top};
            next.waypoints.insert(next.waypoints.begin(), top);
        } else if (prev.tag == "AERIAL" && next.tag == "GROUND") {
            const Eigen::VectorXd bottom = next.waypoints.front();
            Eigen::VectorXd up = Eigen::VectorXd::Zero(bottom.size());
            up(2) = takeoff_landing_height_;
            const Eigen::VectorXd top = bottom + up;
            segs[i].waypoints = {top, bottom};
            prev.waypoints.push_back(top);
        } else {
            RCLCPP_WARN(get_logger(),
                        "vehicle '%s' segment %zu (TRANS): neighboring tags are not GROUND/AERIAL "
                        "(prev='%s', next='%s'), leaving as-authored",
                        vp.vehicle.c_str(), i, prev.tag.c_str(), next.tag.c_str());
        }
    }
}

void PlannerNode::write_trajectory_csv(const planner_msgs::msg::Trajectory& traj,
                                       const std::vector<std::string>& seg_tags,
                                       const std::string& file_path) const {
    std::ofstream out(file_path);
    if (!out.is_open()) {
        RCLCPP_ERROR(get_logger(), "failed to open trajectory CSV for writing: %s", file_path.c_str());
        return;
    }

    out << std::setprecision(9);
    out << "seg_idx,tag,x,y,z,qx,qy,qz,qw\n";

    size_t seg_idx = 0;
    for (size_t i = 0; i < traj.points.size(); ++i) {
        const auto& pt = traj.points[i];
        const std::string& tag = (seg_idx < seg_tags.size()) ? seg_tags[seg_idx] : "";

        double qx, qy, qz, qw;
        trajectory_point_quaternion(pt, qx, qy, qz, qw);

        out << seg_idx << ',' << tag << ','
            << pt.position.x << ',' << pt.position.y << ',' << pt.position.z << ','
            << qx << ',' << qy << ',' << qz << ',' << qw << '\n';

        if (i + 1 < traj.points.size()) {
            const double t_here = rclcpp::Duration(pt.time_from_start).seconds();
            const double t_next = rclcpp::Duration(traj.points[i + 1].time_from_start).seconds();
            if (t_next <= t_here + 1e-9) ++seg_idx;
        }
    }

    RCLCPP_INFO(get_logger(), "wrote %zu trajectory points (%zu segments) to %s",
                traj.points.size(), seg_tags.size(), file_path.c_str());
}

void PlannerNode::run_pipeline() {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        cloud = last_cloud_;
    }
    if (!cloud || vehicles_.empty()) {
        RCLCPP_WARN(get_logger(), "need a cloud and at least one vehicle nominal path");
        return;
    }

    using Clock = std::chrono::steady_clock;
    using Ms = std::chrono::duration<double, std::milli>;
    auto t0 = Clock::now();
    Eigen::MatrixXd P = cloud_to_eigen(*cloud);
    RCLCPP_INFO(get_logger(), "cloud: %ld points  [%.1f ms]; planning %zu vehicle(s) in parallel...",
                P.rows(),
                std::chrono::duration_cast<Ms>(Clock::now() - t0).count(),
                vehicles_.size());

    std::vector<std::thread> threads;
    threads.reserve(vehicles_.size());
    for (auto& vp : vehicles_) {
        threads.emplace_back([this, &vp, &P]() { run_pipeline_for_vehicle(vp, P); });
    }
    for (auto& t : threads) t.join();
}

void PlannerNode::run_pipeline_for_vehicle(VehiclePlan& vp, const Eigen::MatrixXd& P) {
    if (vp.nominal_path.size() < 2 || vp.segments.empty()) {
        RCLCPP_WARN(get_logger(), "vehicle '%s': need >=2 nominal_path points, skipping", vp.vehicle.c_str());
        return;
    }

    vp.nominal_path_pub->publish(make_path_msg(vp.nominal_path));

    using Clock = std::chrono::steady_clock;
    using Ms = std::chrono::duration<double, std::milli>;
    auto elapsed = [](Clock::time_point t0) {
        return std::chrono::duration_cast<Ms>(Clock::now() - t0).count();
    };
    Clock::time_point t0;

    RCLCPP_INFO(get_logger(), "vehicle '%s': planning %zu segment(s)...",
                vp.vehicle.c_str(), vp.segments.size());

    std::vector<gcs::ConvexRegion> all_regions;
    std::vector<std::string> region_tags;
    std::vector<gcs::CompositeTimingTrajectory> seg_trajs;
    std::vector<std::string> traj_tags;

    for (size_t si = 0; si < vp.segments.size(); ++si) {
        const auto& seg = vp.segments[si];
        if (seg.waypoints.size() < 2) {
            RCLCPP_WARN(get_logger(), "vehicle '%s' segment %zu (%s): fewer than 2 waypoints, skipping",
                        vp.vehicle.c_str(), si, seg.tag.c_str());
            continue;
        }

        if (seg.tag == "TRANS" && seg.waypoints.size() == 2) {
            RCLCPP_INFO(get_logger(),
                        "vehicle '%s' segment %zu (TRANS): vertical hop (%.2f m); building corridor "
                        "(R=%.1f, %d threads)...",
                        vp.vehicle.c_str(), si, (seg.waypoints.back() - seg.waypoints.front()).norm(),
                        R_, num_threads_);

            gcs::CorridorOptions copt;
            copt.R            = R_;
            copt.num_threads  = num_threads_;
            copt.max_rounds   = max_rounds_;
            copt.drop_covered_samples = drop_covered_samples_;
            copt.sphere_floor = true;
            copt.seed         = static_cast<uint64_t>(corridor_seed_);

            t0 = Clock::now();
            auto cr = gcs::build_corridor_parallel(seg.waypoints.front(), seg.waypoints.back(),
                                                   seg.waypoints, P, copt);
            RCLCPP_INFO(get_logger(),
                        "vehicle '%s' segment %zu (TRANS): built corridor: %zu regions in %d round(s), "
                        "connected=%s  [%.1f ms]",
                        vp.vehicle.c_str(), si, cr.regions.size(), cr.rounds,
                        cr.connected ? "true" : "false", elapsed(t0));

            if (cr.regions.empty()) {
                RCLCPP_WARN(get_logger(), "vehicle '%s' segment %zu (TRANS): no regions grown, skipping",
                            vp.vehicle.c_str(), si);
                continue;
            }
            for (const auto& r : cr.regions) {
                all_regions.push_back(r);
                region_tags.push_back(seg.tag);
            }

            const bool contained = std::any_of(cr.regions.begin(), cr.regions.end(),
                [&](const gcs::ConvexRegion& r) {
                    return r.contains(seg.waypoints.front()) && r.contains(seg.waypoints.back());
                });
            if (!contained) {
                RCLCPP_ERROR(get_logger(),
                            "vehicle '%s' segment %zu (TRANS): vertical hop not fully contained in any "
                            "single grown region, skipping", vp.vehicle.c_str(), si);
                continue;
            }

            seg_trajs.push_back(make_vertical_trajectory(seg.waypoints.front(), seg.waypoints.back(),
                                                         v_max_, takeoff_landing_accel_limit_));
            traj_tags.push_back(seg.tag);
            RCLCPP_INFO(get_logger(), "vehicle '%s' segment %zu (TRANS): vertical trajectory: T=%.2fs",
                        vp.vehicle.c_str(), si, seg_trajs.back().total_duration());
            continue;
        }

        gcs::CorridorOptions copt;
        copt.R            = R_;
        copt.num_threads  = num_threads_;
        copt.max_rounds   = max_rounds_;
        copt.drop_covered_samples = drop_covered_samples_;
        copt.sphere_floor = true;
        copt.seed         = static_cast<uint64_t>(corridor_seed_);
        if (seg.tag == "GROUND") {
            const double fit_radius = ground_plane_fit_radius_;
            const double z_window   = ground_plane_z_window_;
            const double half_width = ground_band_half_width_;
            copt.region_postprocess = [&P, fit_radius, z_window, half_width]
                (const Eigen::VectorXd& pq, gcs::ConvexRegion& reg) {
                auto plane = gcs::fit_local_ground_plane(P, pq, fit_radius, z_window);
                reg = gcs::clamp_region_to_ground_band(reg, plane, half_width);
            };
        }

        RCLCPP_INFO(get_logger(),
                    "vehicle '%s' segment %zu (%s): %zu waypoints; building corridor (R=%.1f, %d threads)...",
                    vp.vehicle.c_str(), si, seg.tag.c_str(), seg.waypoints.size(), R_, num_threads_);
        t0 = Clock::now();
        auto cr = gcs::build_corridor_parallel(seg.waypoints.front(), seg.waypoints.back(),
                                               seg.waypoints, P, copt);
        RCLCPP_INFO(get_logger(),
                    "vehicle '%s' segment %zu (%s): built corridor: %zu regions in %d round(s), connected=%s  [%.1f ms]",
                    vp.vehicle.c_str(), si, seg.tag.c_str(), cr.regions.size(), cr.rounds,
                    cr.connected ? "true" : "false", elapsed(t0));

        if (cr.regions.empty()) {
            RCLCPP_WARN(get_logger(), "vehicle '%s' segment %zu (%s): no regions grown, skipping",
                        vp.vehicle.c_str(), si, seg.tag.c_str());
            continue;
        }
        if (!cr.connected) {
            RCLCPP_WARN(get_logger(),
                        "vehicle '%s' segment %zu (%s): corridor did not connect q0->qT after %d rounds; "
                        "attempting plan anyway", vp.vehicle.c_str(), si, seg.tag.c_str(), cr.rounds);
        }

        for (const auto& r : cr.regions) {
            all_regions.push_back(r);
            region_tags.push_back(seg.tag);
        }

        try {
            gcs::GCSCompositeBezierPlanner planner(cr.regions);

            t0 = Clock::now();
            planner.build_graph();
            RCLCPP_INFO(get_logger(), "vehicle '%s' segment %zu (%s): graph built  [%.1f ms]",
                        vp.vehicle.c_str(), si, seg.tag.c_str(), elapsed(t0));

            const int d = static_cast<int>(P.cols());
            gcs::GCSCompositeBezierPlanner::Options opt;
            opt.degree = degree_;
            opt.continuity = continuity_;
            opt.vel_limit = v_max_;
            opt.v0 = Eigen::VectorXd::Zero(d);
            opt.vT = Eigen::VectorXd::Zero(d);
            opt.rest_to_rest_accel = true;
            opt.w_geom_accel  = w_geom_accel_;
            opt.w_time_accel  = w_time_accel_;
            opt.w_geom_jerk   = w_geom_jerk_;
            opt.w_time        = w_time_;
            opt.w_geom_length = w_geom_length_;

            t0 = Clock::now();
            auto traj = planner.plan_composite(seg.waypoints.front(), seg.waypoints.back(), opt);
            RCLCPP_INFO(get_logger(), "vehicle '%s' segment %zu (%s): trajectory optimized: T=%.2fs  [%.1f ms]",
                        vp.vehicle.c_str(), si, seg.tag.c_str(), traj.total_duration(), elapsed(t0));

            seg_trajs.push_back(std::move(traj));
            traj_tags.push_back(seg.tag);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "vehicle '%s' segment %zu (%s): planning failed: %s",
                        vp.vehicle.c_str(), si, seg.tag.c_str(), e.what());
        }
    }

    std_msgs::msg::Header hdr;
    hdr.stamp = now();
    hdr.frame_id = frame_id_;

    auto corridor_markers = make_corridor_markers(all_regions, region_tags);
    const bool has_traj = !seg_trajs.empty();
    planner_msgs::msg::Trajectory traj_msg;
    nav_msgs::msg::Path traj_path;
    visualization_msgs::msg::MarkerArray traj_segment_markers;
    if (has_traj) {
        traj_msg = to_msg(seg_trajs, traj_tags, hdr);
        traj_path = make_traj_path(seg_trajs, hdr);
        traj_segment_markers = make_traj_segment_markers(seg_trajs, traj_tags, hdr);

        if (!traj_output_dir_.empty()) {
            std::error_code ec;
            std::filesystem::create_directories(traj_output_dir_, ec);
            if (ec) {
                RCLCPP_ERROR(get_logger(), "vehicle '%s': failed to create traj_output_dir '%s': %s",
                            vp.vehicle.c_str(), traj_output_dir_.c_str(), ec.message().c_str());
            } else {
                const std::string file_path = traj_output_dir_ + "/traj_" + vp.vehicle + ".csv";
                write_trajectory_csv(traj_msg, traj_tags, file_path);
            }
        }
    }

    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        vp.last_corridor_markers = corridor_markers;
        if (has_traj) {
            vp.last_traj_msg = traj_msg;
            vp.last_traj_path = traj_path;
            vp.last_traj_segment_markers = traj_segment_markers;
            vp.has_trajectory = true;
        }
    }

    vp.corridor_pub->publish(corridor_markers);
    if (has_traj) {
        vp.traj_path_pub->publish(traj_path);
        vp.traj_segments_pub->publish(traj_segment_markers);
        vp.traj_pub->publish(traj_msg);
        RCLCPP_INFO(get_logger(), "vehicle '%s': published %zu segment trajectories",
                    vp.vehicle.c_str(), seg_trajs.size());
    } else {
        RCLCPP_WARN(get_logger(), "vehicle '%s': no segment trajectories produced", vp.vehicle.c_str());
    }
}

void PlannerNode::publish_all() {
    std::lock_guard<std::mutex> lk(state_mtx_);
    const auto stamp = now();

    if (last_cloud_) {
        last_cloud_->header.stamp = stamp;
        cloud_pub_->publish(*last_cloud_);
    }
    if (!last_voxels_.markers.empty()) {
        for (auto& m : last_voxels_.markers) m.header.stamp = stamp;
        voxels_pub_->publish(last_voxels_);
    }

    for (auto& vp : vehicles_) {
        vp.nominal_path_pub->publish(make_path_msg(vp.nominal_path));
        if (!vp.last_corridor_markers.markers.empty()) {
            for (auto& m : vp.last_corridor_markers.markers) m.header.stamp = stamp;
            vp.corridor_pub->publish(vp.last_corridor_markers);
        }
        if (vp.has_trajectory) {
            vp.last_traj_path.header.stamp = stamp;
            for (auto& m : vp.last_traj_segment_markers.markers) m.header.stamp = stamp;
            vp.last_traj_msg.header.stamp = stamp;
            vp.traj_path_pub->publish(vp.last_traj_path);
            vp.traj_segments_pub->publish(vp.last_traj_segment_markers);
            vp.traj_pub->publish(vp.last_traj_msg);
        }
    }
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

namespace {

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

std_msgs::msg::ColorRGBA tag_color(const std::string& tag) {
    std_msgs::msg::ColorRGBA c;
    c.a = 0.85f;
    if (tag == "GROUND")      { c.r = 0.2f; c.g = 0.85f; c.b = 0.2f; }
    else if (tag == "AERIAL") { c.r = 0.2f; c.g = 0.6f;  c.b = 1.0f; }
    else if (tag == "TRANS")  { c.r = 1.0f; c.g = 0.8f;  c.b = 0.1f; }
    else                      { c.r = 0.6f; c.g = 0.6f;  c.b = 0.6f; }
    return c;
}

}  // namespace

std::vector<PathSegment> PlannerNode::load_path_segments_csv(const std::vector<std::string>& files) const {
    std::vector<PathSegment> segments;
    bool have_current = false;
    long current_seg_idx = 0;

    for (const auto& file : files) {
        std::ifstream in(file);
        if (!in.is_open()) {
            throw std::runtime_error("failed to open nominal path CSV: " + file);
        }

        std::string header_line;
        if (!std::getline(in, header_line)) {
            throw std::runtime_error("empty nominal path CSV: " + file);
        }
        auto header = split_csv_line(header_line);
        int xi = -1, yi = -1, zi = -1, segi = -1, tagi = -1;
        for (size_t i = 0; i < header.size(); ++i) {
            if (header[i] == "x") xi = static_cast<int>(i);
            else if (header[i] == "y") yi = static_cast<int>(i);
            else if (header[i] == "z") zi = static_cast<int>(i);
            else if (header[i] == "seg_idx") segi = static_cast<int>(i);
            else if (header[i] == "tag") tagi = static_cast<int>(i);
        }
        if (xi < 0 || yi < 0 || zi < 0) {
            throw std::runtime_error("nominal path CSV missing x/y/z column: " + file);
        }

        std::string line;
        size_t row_count = 0;
        while (std::getline(in, line)) {
            if (line.empty()) continue;
            auto fields = split_csv_line(line);
            size_t need = static_cast<size_t>(std::max({xi, yi, zi, segi, tagi}));
            if (fields.size() <= need) {
                throw std::runtime_error("malformed row in nominal path CSV: " + file);
            }

            long seg_idx = (segi >= 0) ? std::stol(fields[segi]) : 0;
            std::string tag = (tagi >= 0) ? fields[tagi] : "";
            Eigen::VectorXd p(3);
            p << std::stod(fields[xi]), std::stod(fields[yi]), std::stod(fields[zi]);

            if (!have_current || seg_idx != current_seg_idx) {
                segments.push_back({tag, {}});
                current_seg_idx = seg_idx;
                have_current = true;
            }
            segments.back().waypoints.push_back(p);
            ++row_count;
        }
        RCLCPP_INFO(get_logger(), "loaded %zu waypoints from %s", row_count, file.c_str());
    }
    return segments;
}

planner_msgs::msg::Trajectory
PlannerNode::to_msg(const std::vector<gcs::CompositeTimingTrajectory>& trajs,
                    const std::vector<std::string>& tags,
                    const std_msgs::msg::Header& hdr) const {
    planner_msgs::msg::Trajectory out;
    out.header = hdr;
    double t_offset = 0.0;
    for (size_t si = 0; si < trajs.size(); ++si) {
        const auto& tr = trajs[si];
        const bool is_ground = (si < tags.size()) && (tags[si] == "GROUND");
        const double T = tr.total_duration();
        auto add_point = [&](double tc) {
            Eigen::VectorXd p = tr.position(tc);
            Eigen::VectorXd v = tr.velocity(tc);
            Eigen::VectorXd a = tr.acceleration(tc);

            planner_msgs::msg::TrajectoryPoint pt;
            pt.time_from_start = rclcpp::Duration::from_seconds(t_offset + tc);
            pt.position.x = p(0); pt.position.y = p(1); pt.position.z = p(2);
            pt.velocity.x = v(0); pt.velocity.y = v(1); pt.velocity.z = v(2);
            pt.acceleration.x = a(0); pt.acceleration.y = a(1); pt.acceleration.z = a(2);
            pt.yaw = std::atan2(v(1), v(0));
            if (is_ground) {
                pt.roll = 0.0;
                pt.pitch = 0.0;
            } else {
                double roll, pitch;
                attitude_from_accel_yaw(Eigen::Vector3d(a(0), a(1), a(2)), pt.yaw, roll, pitch);
                pt.roll  = roll;
                pt.pitch = pitch;
            }
            pt.speed = v.norm();
            out.points.push_back(pt);
        };
        for (double t = 0.0; t < T - 1e-9; t += sample_dt_) add_point(t);
        add_point(T);
        t_offset += T;
    }
    out.total_duration = t_offset;
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
PlannerNode::make_traj_path(const std::vector<gcs::CompositeTimingTrajectory>& trajs,
                            const std_msgs::msg::Header& hdr) const {
    nav_msgs::msg::Path msg;
    msg.header = hdr;
    for (const auto& tr : trajs) {
        const double T = tr.total_duration();
        auto add_pose = [&](double tc) {
            Eigen::VectorXd p = tr.position(tc);
            geometry_msgs::msg::PoseStamped ps;
            ps.header = hdr;
            ps.pose.position.x = p(0);
            ps.pose.position.y = p(1);
            ps.pose.position.z = p(2);
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        };
        for (double t = 0.0; t < T - 1e-9; t += sample_dt_) add_pose(t);
        add_pose(T);
    }
    return msg;
}

visualization_msgs::msg::MarkerArray
PlannerNode::make_traj_segment_markers(const std::vector<gcs::CompositeTimingTrajectory>& trajs,
                                       const std::vector<std::string>& tags,
                                       const std_msgs::msg::Header& hdr) const {
    visualization_msgs::msg::MarkerArray arr;
    for (size_t i = 0; i < trajs.size(); ++i) {
        const auto& tr = trajs[i];
        visualization_msgs::msg::Marker m;
        m.header = hdr;
        m.ns = "trajectory_segments";
        m.id = static_cast<int>(i);
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.08;
        m.color = tag_color(tags[i]);
        m.pose.orientation.w = 1.0;

        const double T = tr.total_duration();
        auto add_pt = [&](double tc) {
            Eigen::VectorXd p = tr.position(tc);
            geometry_msgs::msg::Point pt;
            pt.x = p(0); pt.y = p(1); pt.z = p(2);
            m.points.push_back(pt);
        };
        for (double t = 0.0; t < T - 1e-9; t += sample_dt_) add_pt(t);
        add_pt(T);
        arr.markers.push_back(m);
    }
    return arr;
}

visualization_msgs::msg::MarkerArray
PlannerNode::make_corridor_markers(const std::vector<gcs::ConvexRegion>& regions,
                                   const std::vector<std::string>& tags) const {
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    for (size_t ri = 0; ri < regions.size(); ++ri) {
        const auto& reg = regions[ri];
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
        m.color = tag_color(tags[ri]);
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