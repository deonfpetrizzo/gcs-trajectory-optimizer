#include "gcs_planner/planner_node.hpp"

#include "gcs_planner/trajectory_msg_builders.hpp"
#include "gcs_planner/vertical_profile.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <set>
#include <thread>

namespace gcs_planner {

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
    geom_accel_limit_   = declare_parameter("geom_accel_limit", 0.0);
    time_accel_limit_   = declare_parameter("time_accel_limit", 0.0);
    t_min_              = declare_parameter("t_min", 0.0);
    t_max_              = declare_parameter("t_max", 1e9);
    h_dot_min_          = declare_parameter("h_dot_min", 1e-2);
    soc_facets_         = declare_parameter("soc_facets", 16);
    rest_to_rest_accel_ = declare_parameter("rest_to_rest_accel", true);

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
            vp.segments = load_path_segments_csv({file}, get_logger());
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

std_msgs::msg::Header PlannerNode::make_header() const {
    std_msgs::msg::Header h;
    h.stamp = now();
    h.frame_id = frame_id_;
    return h;
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
    RCLCPP_INFO(get_logger(), "loaded %zu points", pc.size());

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
    RCLCPP_INFO(get_logger(), "downsampled to %zu points (0.5 m voxels)", pc_ds.size());

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

    RCLCPP_INFO(get_logger(), "published map cloud (%zu points)", pc_ds.size());
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

    vp.nominal_path_pub->publish(make_path_msg(vp.nominal_path, make_header()));

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
        auto traj = plan_segment(vp, si, seg, P, all_regions, region_tags);
        if (traj) {
            seg_trajs.push_back(std::move(*traj));
            traj_tags.push_back(seg.tag);
        }
    }

    publish_vehicle_output(vp, all_regions, region_tags, seg_trajs, traj_tags);
}

std::optional<gcs::CompositeTimingTrajectory> PlannerNode::plan_segment(
    const VehiclePlan& vp, size_t si, const PathSegment& seg, const Eigen::MatrixXd& P,
    std::vector<gcs::ConvexRegion>& all_regions,
    std::vector<std::string>& region_tags) const {

    using Clock = std::chrono::steady_clock;
    using Ms = std::chrono::duration<double, std::milli>;
    auto elapsed = [](Clock::time_point t0) {
        return std::chrono::duration_cast<Ms>(Clock::now() - t0).count();
    };
    Clock::time_point t0;

    gcs::CorridorOptions copt;
    copt.R            = R_;
    copt.num_threads  = num_threads_;
    copt.max_rounds   = max_rounds_;
    copt.drop_covered_samples = drop_covered_samples_;
    copt.sphere_floor = true;
    copt.seed         = static_cast<uint64_t>(corridor_seed_);

    if (seg.tag == "TRANS" && seg.waypoints.size() == 2) {
        RCLCPP_INFO(get_logger(),
                    "vehicle '%s' segment %zu (TRANS): vertical hop (%.2f m); building corridor "
                    "(R=%.1f, %d threads)...",
                    vp.vehicle.c_str(), si, (seg.waypoints.back() - seg.waypoints.front()).norm(),
                    R_, num_threads_);

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
            return std::nullopt;
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
            return std::nullopt;
        }

        auto traj = make_vertical_trajectory(seg.waypoints.front(), seg.waypoints.back(),
                                             v_max_, takeoff_landing_accel_limit_);
        RCLCPP_INFO(get_logger(), "vehicle '%s' segment %zu (TRANS): vertical trajectory: T=%.2fs",
                    vp.vehicle.c_str(), si, traj.total_duration());
        return traj;
    }

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
        return std::nullopt;
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
        if (geom_accel_limit_ > 0.0) opt.geom_accel_limit = geom_accel_limit_;
        if (time_accel_limit_ > 0.0) opt.time_accel_limit = time_accel_limit_;
        opt.v0 = Eigen::VectorXd::Zero(d);
        opt.vT = Eigen::VectorXd::Zero(d);
        opt.rest_to_rest_accel = rest_to_rest_accel_;
        opt.T_min = t_min_;
        opt.T_max = t_max_;
        opt.h_dot_min = h_dot_min_;
        opt.soc_facets = soc_facets_;
        opt.w_geom_accel  = w_geom_accel_;
        opt.w_time_accel  = w_time_accel_;
        opt.w_geom_jerk   = w_geom_jerk_;
        opt.w_time        = w_time_;
        opt.w_geom_length = w_geom_length_;

        t0 = Clock::now();
        auto traj = planner.plan_composite(seg.waypoints.front(), seg.waypoints.back(), opt);
        RCLCPP_INFO(get_logger(), "vehicle '%s' segment %zu (%s): trajectory optimized: T=%.2fs  [%.1f ms]",
                    vp.vehicle.c_str(), si, seg.tag.c_str(), traj.total_duration(), elapsed(t0));
        return traj;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "vehicle '%s' segment %zu (%s): planning failed: %s",
                    vp.vehicle.c_str(), si, seg.tag.c_str(), e.what());
        return std::nullopt;
    }
}

void PlannerNode::publish_vehicle_output(VehiclePlan& vp,
    const std::vector<gcs::ConvexRegion>& all_regions,
    const std::vector<std::string>& region_tags,
    const std::vector<gcs::CompositeTimingTrajectory>& seg_trajs,
    const std::vector<std::string>& traj_tags) {

    const std_msgs::msg::Header hdr = make_header();

    auto corridor_markers = make_corridor_markers(all_regions, region_tags, hdr);
    const bool has_traj = !seg_trajs.empty();
    planner_msgs::msg::Trajectory traj_msg;
    nav_msgs::msg::Path traj_path;
    visualization_msgs::msg::MarkerArray traj_segment_markers;
    if (has_traj) {
        traj_msg = build_trajectory_msg(seg_trajs, traj_tags, hdr, sample_dt_);
        traj_path = make_traj_path(seg_trajs, hdr, sample_dt_);
        traj_segment_markers = make_traj_segment_markers(seg_trajs, traj_tags, hdr, sample_dt_);

        if (!traj_output_dir_.empty()) {
            std::error_code ec;
            std::filesystem::create_directories(traj_output_dir_, ec);
            if (ec) {
                RCLCPP_ERROR(get_logger(), "vehicle '%s': failed to create traj_output_dir '%s': %s",
                            vp.vehicle.c_str(), traj_output_dir_.c_str(), ec.message().c_str());
            } else {
                const std::string file_path = traj_output_dir_ + "/traj_" + vp.vehicle + ".csv";
                write_trajectory_csv(traj_msg, traj_tags, file_path, get_logger());
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
        vp.nominal_path_pub->publish(make_path_msg(vp.nominal_path, make_header()));
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

}  // namespace gcs_planner
