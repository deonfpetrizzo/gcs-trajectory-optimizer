#include "gcs_planner/planner_node.hpp"

#include "gcs_planner/trajectory_msg_builders.hpp"
#include "gcs_planner/vertical_profile.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
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

namespace {

bool line_covered_by_regions(const Eigen::VectorXd& p0, const Eigen::VectorXd& p1,
                             const std::vector<gcs::ConvexRegion>& regions,
                             int samples, double tol) {
    for (int k = 0; k <= samples; ++k) {
        const double t = static_cast<double>(k) / samples;
        const Eigen::VectorXd p = p0 + t * (p1 - p0);
        bool inside = false;
        for (const auto& r : regions) {
            if (r.contains(p, tol)) { inside = true; break; }
        }
        if (!inside) return false;
    }
    return true;
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
    cloud_voxel_leaf_ = declare_parameter("cloud_voxel_leaf", 0.5);
    viz_voxel_leaf_   = declare_parameter("viz_voxel_leaf", 0.0);
    outlier_filter_enable_ = declare_parameter("outlier_filter_enable", true);
    outlier_filter_type_   = declare_parameter<std::string>("outlier_filter_type", "radius");
    outlier_radius_        = declare_parameter("outlier_radius", 0.5);
    outlier_min_neighbors_ = declare_parameter("outlier_min_neighbors", 4);
    outlier_mean_k_        = declare_parameter("outlier_mean_k", 20);
    outlier_stddev_        = declare_parameter("outlier_stddev", 1.0);
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

    trav_resolution_       = declare_parameter("trav_resolution", 0.3);
    trav_slope_fit_radius_ = declare_parameter("trav_slope_fit_radius", 0.6);
    trav_ground_z_window_  = declare_parameter("trav_ground_z_window", 0.3);
    trav_clearance_ceiling_ = declare_parameter("trav_clearance_ceiling", 2.0);
    trav_min_points_       = declare_parameter("trav_min_points", 4);
    trav_smooth_sigma_     = declare_parameter("trav_smooth_sigma", 1.0);
    ground_grid_margin_    = declare_parameter("ground_grid_margin", 2.0);
    ground_floor_offset_   = declare_parameter("ground_floor_offset", -0.5);
    robot_max_slope_deg_   = declare_parameter("robot_max_slope_deg", 25.0);
    robot_max_roughness_   = declare_parameter("robot_max_roughness", 0.05);
    robot_max_step_        = declare_parameter("robot_max_step", 0.15);
    robot_height_          = declare_parameter("robot_height", 1.0);
    robot_min_clearance_   = declare_parameter("robot_min_clearance", 0.5);
    robot_footprint_radius_ = declare_parameter("robot_footprint_radius", 0.3);
    robot_allow_stairs_    = declare_parameter("robot_allow_stairs", false);
    occ_score_threshold_   = declare_parameter("occ_score_threshold", 0.0);

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
    traversability_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("traversability", latched);
    occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", latched);

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
    const float leaf = static_cast<float>(cloud_voxel_leaf_);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(pc_ds);
    RCLCPP_INFO(get_logger(), "downsampled to %zu points (%.2f m voxels)", pc_ds.size(),
                cloud_voxel_leaf_);

    if (outlier_filter_enable_) {
        const size_t before = pc_ds.size();
        pcl::PointCloud<pcl::PointXYZ> pc_f;
        if (outlier_filter_type_ == "statistical") {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(pc_ds.makeShared());
            sor.setMeanK(outlier_mean_k_);
            sor.setStddevMulThresh(outlier_stddev_);
            sor.filter(pc_f);
        } else {
            pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
            ror.setInputCloud(pc_ds.makeShared());
            ror.setRadiusSearch(outlier_radius_);
            ror.setMinNeighborsInRadius(outlier_min_neighbors_);
            ror.filter(pc_f);
        }
        pc_ds.swap(pc_f);
        RCLCPP_INFO(get_logger(), "outlier filter (%s) removed %zu points (%zu -> %zu)",
                    outlier_filter_type_.c_str(), before - pc_ds.size(), before, pc_ds.size());
    }

    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(pc_ds, *msg);
    msg->header.frame_id = frame_id_;
    msg->header.stamp = now();

    auto viz_msg = msg;
    if (viz_voxel_leaf_ > cloud_voxel_leaf_) {
        pcl::PointCloud<pcl::PointXYZ> pc_viz;
        pcl::VoxelGrid<pcl::PointXYZ> vgv;
        vgv.setInputCloud(pc_ds.makeShared());
        const float vleaf = static_cast<float>(viz_voxel_leaf_);
        vgv.setLeafSize(vleaf, vleaf, vleaf);
        vgv.filter(pc_viz);
        viz_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(pc_viz, *viz_msg);
        viz_msg->header.frame_id = frame_id_;
        viz_msg->header.stamp = now();
        RCLCPP_INFO(get_logger(), "viz map_cloud downsampled to %zu points (%.2f m voxels)",
                    pc_viz.size(), viz_voxel_leaf_);
    }
    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        last_cloud_ = msg;
        last_map_cloud_ = viz_msg;
    }
    cloud_pub_->publish(*viz_msg);

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

    compute_traversability_map(P);

    std::vector<std::thread> threads;
    threads.reserve(vehicles_.size());
    for (auto& vp : vehicles_) {
        threads.emplace_back([this, &vp, &P]() { run_pipeline_for_vehicle(vp, P); });
    }
    for (auto& t : threads) t.join();
}

trav::RobotModel PlannerNode::robot_model() const {
    trav::RobotModel robot;
    robot.max_slope_rad    = robot_max_slope_deg_ * M_PI / 180.0;
    robot.max_roughness    = robot_max_roughness_;
    robot.max_step         = robot_max_step_;
    robot.height           = robot_height_;
    robot.min_clearance    = robot_min_clearance_;
    robot.footprint_radius = robot_footprint_radius_;
    robot.allow_stairs     = robot_allow_stairs_;
    return robot;
}

bool PlannerNode::map_ground_plane(double x, double y, gcs::GroundPlaneFit& out) const {
    int ix, iy;
    if (!trav_map_.info.world_to_cell(Eigen::Vector2d(x, y), ix, iy)) return false;
    const trav::TraversabilityCell& c = trav_map_.at(ix, iy);
    if (!c.valid) return false;
    out.a = c.plane_a;
    out.b = c.plane_b;
    out.c = c.plane_c;
    out.tilted = true;
    return true;
}

Eigen::VectorXd PlannerNode::snap_to_ground(const Eigen::VectorXd& q,
                                            const Eigen::MatrixXd& P) const {
    const Eigen::Vector2d xy = occ_grid_.nearest_free(Eigen::Vector2d(q(0), q(1)));
    gcs::GroundPlaneFit pl;
    if (!map_ground_plane(xy.x(), xy.y(), pl)) {
        Eigen::VectorXd c3(3);
        c3 << xy.x(), xy.y(), q(2);
        pl = gcs::fit_local_ground_plane(P, c3, ground_plane_fit_radius_, ground_plane_z_window_);
    }
    Eigen::VectorXd qs(3);
    qs << xy.x(), xy.y(), pl.a * xy.x() + pl.b * xy.y() + pl.c;
    return qs;
}

void PlannerNode::compute_traversability_map(const Eigen::MatrixXd& P) {
    using Clock = std::chrono::steady_clock;
    using Ms = std::chrono::duration<double, std::milli>;
    auto t0 = Clock::now();

    const trav::GridInfo info = trav::grid_from_cloud(P, trav_resolution_, ground_grid_margin_);

    trav::TraversabilityParams tp;
    tp.resolution         = trav_resolution_;
    tp.slope_fit_radius   = trav_slope_fit_radius_;
    tp.ground_z_window    = trav_ground_z_window_;
    tp.clearance_ceiling  = trav_clearance_ceiling_;
    tp.min_points_per_cell = trav_min_points_;

    trav::TraversabilityMap map = trav::compute_traversability(P, info, tp);
    if (trav_smooth_sigma_ > 0.0) trav::smooth_ground_field(map, trav_smooth_sigma_);

    const trav::RobotModel robot = robot_model();
    trav::OccupancyParams op;
    op.score_threshold = occ_score_threshold_;

    occ_grid_ = trav::make_occupancy_grid(map, robot, op);
    occupied_xy_ = occ_grid_.occupied_centers();
    trav_map_ = map;
    has_trav_ = true;

    const std_msgs::msg::Header hdr = make_header();
    auto trav_markers = make_traversability_markers(map, robot, hdr);
    auto occ_msg = make_occupancy_grid_msg(occ_grid_, hdr);

    RCLCPP_INFO(get_logger(),
                "traversability: %dx%d grid @ %.2fm, %ld occupied cells  [%.1f ms]",
                info.nx, info.ny, trav_resolution_, occupied_xy_.rows(),
                std::chrono::duration_cast<Ms>(Clock::now() - t0).count());

    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        last_trav_markers_ = trav_markers;
        last_occ_grid_ = occ_msg;
    }
    traversability_pub_->publish(trav_markers);
    occupancy_pub_->publish(occ_msg);
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

    std::vector<Eigen::VectorXd> bnd(vp.segments.size() + 1);
    bnd[0] = vp.segments.front().waypoints.front();
    for (size_t i = 0; i < vp.segments.size(); ++i)
        bnd[i + 1] = vp.segments[i].waypoints.back();
    if (has_trav_) {
        for (size_t i = 0; i < vp.segments.size(); ++i) {
            if (vp.segments[i].tag != "GROUND") continue;
            bnd[i]     = snap_to_ground(bnd[i], P);
            bnd[i + 1] = snap_to_ground(bnd[i + 1], P);
        }
    }

    for (size_t i = 0; i < vp.segments.size(); ++i) {
        if (vp.segments[i].tag != "TRANS" || vp.segments[i].waypoints.size() != 2) continue;
        int ground = -1, air = -1;
        if (i > 0 && vp.segments[i - 1].tag == "GROUND") { ground = static_cast<int>(i); air = static_cast<int>(i) + 1; }
        else if (i + 1 < vp.segments.size() && vp.segments[i + 1].tag == "GROUND") { ground = static_cast<int>(i) + 1; air = static_cast<int>(i); }
        if (ground < 0) continue;
        bnd[air] << bnd[ground](0), bnd[ground](1), bnd[air](2);
    }

    for (size_t si = 0; si < vp.segments.size(); ++si) {
        const auto& seg = vp.segments[si];
        if (seg.waypoints.size() < 2) {
            RCLCPP_WARN(get_logger(), "vehicle '%s' segment %zu (%s): fewer than 2 waypoints, skipping",
                        vp.vehicle.c_str(), si, seg.tag.c_str());
            continue;
        }
        auto traj = plan_segment(vp, si, seg, P, bnd[si], bnd[si + 1], all_regions, region_tags);
        if (traj) {
            seg_trajs.push_back(std::move(*traj));
            traj_tags.push_back(seg.tag);
        }
    }

    publish_vehicle_output(vp, all_regions, region_tags, seg_trajs, traj_tags);
}

std::optional<gcs::CompositeTimingTrajectory> PlannerNode::plan_segment(
    const VehiclePlan& vp, size_t si, const PathSegment& seg, const Eigen::MatrixXd& P,
    const Eigen::VectorXd& q0, const Eigen::VectorXd& qT,
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
                    vp.vehicle.c_str(), si, (qT - q0).norm(),
                    R_, num_threads_);

        t0 = Clock::now();
        const std::vector<Eigen::VectorXd> hop = {q0, qT};
        auto cr = gcs::build_corridor_parallel(q0, qT, hop, P, copt);
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

        const int cover_samples = std::max(8, static_cast<int>(std::ceil((qT - q0).norm() / 0.1)));
        if (!line_covered_by_regions(q0, qT, cr.regions, cover_samples, 1e-4)) {
            RCLCPP_ERROR(get_logger(),
                        "vehicle '%s' segment %zu (TRANS): vertical hop not covered by the grown "
                        "corridor, skipping", vp.vehicle.c_str(), si);
            return std::nullopt;
        }

        auto traj = make_vertical_trajectory(q0, qT, v_max_, takeoff_landing_accel_limit_);
        RCLCPP_INFO(get_logger(), "vehicle '%s' segment %zu (TRANS): vertical trajectory: T=%.2fs",
                    vp.vehicle.c_str(), si, traj.total_duration());
        return traj;
    }

    const Eigen::VectorXd& q0_used = q0;
    const Eigen::VectorXd& qT_used = qT;
    gcs::CorridorResult cr;

    if (seg.tag == "GROUND") {
        if (!has_trav_) {
            RCLCPP_WARN(get_logger(),
                        "vehicle '%s' segment %zu (GROUND): traversability map unavailable, skipping",
                        vp.vehicle.c_str(), si);
            return std::nullopt;
        }
        auto map_plane = [this](double x, double y, gcs::GroundPlaneFit& out) -> bool {
            return map_ground_plane(x, y, out);
        };

        gcs::GroundCorridorOptions gopt;
        gopt.corridor2d         = copt;
        gopt.robot_height       = robot_height_;
        gopt.floor_offset       = ground_floor_offset_;
        gopt.plane_fit_radius   = ground_plane_fit_radius_;
        gopt.plane_fit_z_window = ground_plane_z_window_;

        RCLCPP_INFO(get_logger(),
                    "vehicle '%s' segment %zu (GROUND): %zu waypoints; building ground corridor "
                    "(R=%.1f, %d threads)...",
                    vp.vehicle.c_str(), si, seg.waypoints.size(), R_, num_threads_);
        t0 = Clock::now();
        cr = gcs::build_ground_corridor(q0_used, qT_used, seg.waypoints, occupied_xy_, P, gopt,
                                        map_plane);
        RCLCPP_INFO(get_logger(),
                    "vehicle '%s' segment %zu (GROUND): built corridor: %zu regions in %d round(s), "
                    "connected=%s  [%.1f ms]",
                    vp.vehicle.c_str(), si, cr.regions.size(), cr.rounds,
                    cr.connected ? "true" : "false", elapsed(t0));
    } else {
        RCLCPP_INFO(get_logger(),
                    "vehicle '%s' segment %zu (%s): %zu waypoints; building corridor (R=%.1f, %d threads)...",
                    vp.vehicle.c_str(), si, seg.tag.c_str(), seg.waypoints.size(), R_, num_threads_);
        t0 = Clock::now();
        cr = gcs::build_corridor_parallel(q0_used, qT_used, seg.waypoints, P, copt);
        RCLCPP_INFO(get_logger(),
                    "vehicle '%s' segment %zu (%s): built corridor: %zu regions in %d round(s), connected=%s  [%.1f ms]",
                    vp.vehicle.c_str(), si, seg.tag.c_str(), cr.regions.size(), cr.rounds,
                    cr.connected ? "true" : "false", elapsed(t0));
    }

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
        auto traj = planner.plan_composite(q0_used, qT_used, opt);
        RCLCPP_INFO(get_logger(), "vehicle '%s' segment %zu (%s): trajectory optimized: T=%.2fs  [%.1f ms]",
                    vp.vehicle.c_str(), si, seg.tag.c_str(), traj.total_duration(), elapsed(t0));
        for (int idx : traj.region_sequence) {
            all_regions.push_back(cr.regions[idx]);
            region_tags.push_back(seg.tag);
        }
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

    const auto& map_cloud = last_map_cloud_ ? last_map_cloud_ : last_cloud_;
    if (map_cloud) {
        map_cloud->header.stamp = stamp;
        cloud_pub_->publish(*map_cloud);
    }
    if (!last_voxels_.markers.empty()) {
        for (auto& m : last_voxels_.markers) m.header.stamp = stamp;
        voxels_pub_->publish(last_voxels_);
    }
    if (!last_trav_markers_.markers.empty()) {
        for (auto& m : last_trav_markers_.markers) m.header.stamp = stamp;
        traversability_pub_->publish(last_trav_markers_);
    }
    if (!last_occ_grid_.data.empty()) {
        last_occ_grid_.header.stamp = stamp;
        occupancy_pub_->publish(last_occ_grid_);
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
