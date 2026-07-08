#include "gcs_planner/trajectory_executor_node.hpp"

#include <cmath>
#include <vector>

namespace gcs_planner {

namespace {

double duration_sec(const builtin_interfaces::msg::Duration& d) {
    return rclcpp::Duration(d).seconds();
}

bool same_trajectory(const planner_msgs::msg::Trajectory& a,
                     const planner_msgs::msg::Trajectory& b) {
    if (a.points.size() != b.points.size()) return false;
    if (a.total_duration != b.total_duration) return false;
    for (size_t i = 0; i < a.points.size(); ++i) {
        const auto& pa = a.points[i].position;
        const auto& pb = b.points[i].position;
        if (pa.x != pb.x || pa.y != pb.y || pa.z != pb.z) return false;
    }
    return true;
}

void sample_at(const std::vector<planner_msgs::msg::TrajectoryPoint>& pts, double t,
               geometry_msgs::msg::Point& pos, geometry_msgs::msg::Vector3& vel,
               geometry_msgs::msg::Vector3& accel) {
    if (pts.empty()) {
        pos = geometry_msgs::msg::Point();
        vel = geometry_msgs::msg::Vector3();
        accel = geometry_msgs::msg::Vector3();
        return;
    }
    if (t <= duration_sec(pts.front().time_from_start)) {
        pos = pts.front().position;
        vel = pts.front().velocity;
        accel = pts.front().acceleration;
        return;
    }
    if (t >= duration_sec(pts.back().time_from_start)) {
        pos = pts.back().position;
        vel = pts.back().velocity;
        accel = pts.back().acceleration;
        return;
    }

    size_t lo = 0, hi = pts.size() - 1;
    while (lo + 1 < hi) {
        const size_t mid = (lo + hi) / 2;
        if (duration_sec(pts[mid].time_from_start) <= t) lo = mid; else hi = mid;
    }
    const auto& p0 = pts[lo];
    const auto& p1 = pts[hi];
    const double t0 = duration_sec(p0.time_from_start);
    const double t1 = duration_sec(p1.time_from_start);
    const double f = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;

    pos.x = p0.position.x + f * (p1.position.x - p0.position.x);
    pos.y = p0.position.y + f * (p1.position.y - p0.position.y);
    pos.z = p0.position.z + f * (p1.position.z - p0.position.z);
    vel.x = p0.velocity.x + f * (p1.velocity.x - p0.velocity.x);
    vel.y = p0.velocity.y + f * (p1.velocity.y - p0.velocity.y);
    vel.z = p0.velocity.z + f * (p1.velocity.z - p0.velocity.z);
    accel.x = p0.acceleration.x + f * (p1.acceleration.x - p0.acceleration.x);
    accel.y = p0.acceleration.y + f * (p1.acceleration.y - p0.acceleration.y);
    accel.z = p0.acceleration.z + f * (p1.acceleration.z - p0.acceleration.z);
}

double anim_to_traj_time(const std::vector<std::pair<double, double>>& kf, double anim_t) {
    if (kf.empty()) return 0.0;
    if (anim_t <= kf.front().first) return kf.front().second;
    if (anim_t >= kf.back().first) return kf.back().second;

    size_t lo = 0, hi = kf.size() - 1;
    while (lo + 1 < hi) {
        const size_t mid = (lo + hi) / 2;
        if (kf[mid].first <= anim_t) lo = mid; else hi = mid;
    }
    const double a0 = kf[lo].first, a1 = kf[hi].first;
    const double t0 = kf[lo].second, t1 = kf[hi].second;
    const double f = (a1 > a0) ? (anim_t - a0) / (a1 - a0) : 0.0;
    return t0 + f * (t1 - t0);
}

}  // namespace

TrajectoryExecutorNode::TrajectoryExecutorNode() : rclcpp::Node("trajectory_executor") {
    vector_time_scale_    = declare_parameter("vector_time_scale", 0.5);
    shaft_diameter_       = declare_parameter("shaft_diameter", 0.08);
    head_diameter_        = declare_parameter("head_diameter", 0.18);
    head_length_          = declare_parameter("head_length", 0.0);
    position_marker_scale_ = declare_parameter("position_marker_scale", 0.25);
    viz_rate_hz_          = declare_parameter("viz_rate_hz", 30.0);
    segment_pause_sec_    = declare_parameter("segment_pause_sec", 2.0);
    frame_id_override_    = declare_parameter<std::string>("frame_id_override", "");

    // One playback lane per vehicle key (must match the keys planner_node
    // derives from nominal_path_csv_files, e.g. "m4", "g1", "g1m4"), each
    // subscribing to its own trajectory/<vehicle> topic. With no vehicles
    // given, falls back to a single unnamespaced lane subscribing to
    // "trajectory" -- the pre-multi-vehicle behavior.
    auto vehicle_keys = declare_parameter<std::vector<std::string>>("vehicles", std::vector<std::string>{});
    if (vehicle_keys.empty()) vehicle_keys.push_back("");

    vehicles_.resize(vehicle_keys.size());
    for (size_t i = 0; i < vehicle_keys.size(); ++i) {
        auto& ve = vehicles_[i];
        ve.vehicle = vehicle_keys[i];
        const std::string ns = ve.vehicle.empty() ? std::string() : ("/" + ve.vehicle);

        ve.marker_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
            "trajectory_exec_markers" + ns, rclcpp::QoS(1));
        ve.speed_pub = create_publisher<std_msgs::msg::Float64>(
            "trajectory_exec_speed" + ns, rclcpp::QoS(10));
        ve.accel_mag_pub = create_publisher<std_msgs::msg::Float64>(
            "trajectory_exec_accel_magnitude" + ns, rclcpp::QoS(10));

        ve.traj_sub = create_subscription<planner_msgs::msg::Trajectory>(
            "trajectory" + ns, 10,
            [this, i](const planner_msgs::msg::Trajectory::SharedPtr msg) { on_trajectory(i, msg); });

        RCLCPP_INFO(get_logger(), "trajectory_executor: lane '%s' waiting on '%s'",
                    ve.vehicle.c_str(), ("trajectory" + ns).c_str());
    }

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / viz_rate_hz_),
        std::bind(&TrajectoryExecutorNode::on_timer, this));
}

void TrajectoryExecutorNode::on_trajectory(size_t idx, const planner_msgs::msg::Trajectory::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    auto& ve = vehicles_[idx];
    const bool is_new = !ve.has_traj || !same_trajectory(ve.traj, *msg);
    ve.traj = *msg;
    ve.has_traj = true;
    if (is_new) {
        build_keyframes(ve);
        ve.playback_start = now();
        ve.last_loop_index = 0;
        RCLCPP_INFO(get_logger(), "vehicle '%s': new trajectory: T=%.2fs, %zu points, %.2fs per loop with pauses",
                    ve.vehicle.c_str(), msg->total_duration, msg->points.size(), ve.total_anim_duration);
    }
}

void TrajectoryExecutorNode::build_keyframes(VehicleExec& ve) {
    ve.keyframes.clear();
    const auto& pts = ve.traj.points;
    if (pts.empty()) {
        ve.keyframes.push_back({0.0, 0.0});
        ve.total_anim_duration = 0.0;
        return;
    }

    std::vector<double> boundaries;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const double ti = duration_sec(pts[i].time_from_start);
        const double tj = duration_sec(pts[i + 1].time_from_start);
        if (tj - ti < 1e-6) boundaries.push_back(ti);
    }
    boundaries.push_back(duration_sec(pts.back().time_from_start));

    double t_traj = 0.0, t_anim = 0.0;
    ve.keyframes.push_back({0.0, 0.0});
    for (const double b : boundaries) {
        if (b <= t_traj + 1e-9) continue;
        t_anim += (b - t_traj);
        t_traj = b;
        ve.keyframes.push_back({t_anim, t_traj});  // arrival
        t_anim += segment_pause_sec_;
        ve.keyframes.push_back({t_anim, t_traj});  // end of pause
    }
    ve.total_anim_duration = t_anim;
}

void TrajectoryExecutorNode::on_timer() {
    for (size_t i = 0; i < vehicles_.size(); ++i) {
        planner_msgs::msg::Trajectory traj_copy;
        std::vector<std::pair<double, double>> keyframes_copy;
        double total_anim_duration = 0.0;
        rclcpp::Time start;
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            auto& ve = vehicles_[i];
            if (!ve.has_traj) continue;
            traj_copy = ve.traj;
            keyframes_copy = ve.keyframes;
            total_anim_duration = ve.total_anim_duration;
            start = ve.playback_start;
        }

        const double raw_elapsed = (now() - start).seconds();
        double anim_elapsed = 0.0;
        int loop_index = 0;
        if (total_anim_duration > 1e-9) {
            loop_index = static_cast<int>(std::floor(raw_elapsed / total_anim_duration));
            anim_elapsed = raw_elapsed - loop_index * total_anim_duration;
        }
        const double traj_time = anim_to_traj_time(keyframes_copy, anim_elapsed);

        geometry_msgs::msg::Point pos;
        geometry_msgs::msg::Vector3 vel, accel;
        sample_at(traj_copy.points, traj_time, pos, vel, accel);

        const auto& ve = vehicles_[i];
        publish_markers(ve, traj_copy.header.frame_id, pos, vel);
        publish_plot_topics(ve, vel, accel);

        if (loop_index > 0) {
            std::lock_guard<std::mutex> lk(state_mtx_);
            auto& ve_mut = vehicles_[i];
            if (loop_index != ve_mut.last_loop_index) {
                RCLCPP_INFO(get_logger(), "vehicle '%s': trajectory loop %d", ve_mut.vehicle.c_str(), loop_index);
                ve_mut.last_loop_index = loop_index;
            }
        }
    }
}

void TrajectoryExecutorNode::publish_markers(const VehicleExec& ve, const std::string& frame_id,
                                             const geometry_msgs::msg::Point& pos,
                                             const geometry_msgs::msg::Vector3& vel) const {
    const std::string frame = frame_id_override_.empty() ? frame_id : frame_id_override_;
    const auto stamp = now();

    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = frame;
    arrow.header.stamp = stamp;
    arrow.ns = "trajectory_exec";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose.orientation.w = 1.0;
    arrow.points.resize(2);
    arrow.points[0] = pos;
    arrow.points[1].x = pos.x + vel.x * vector_time_scale_;
    arrow.points[1].y = pos.y + vel.y * vector_time_scale_;
    arrow.points[1].z = pos.z + vel.z * vector_time_scale_;
    arrow.scale.x = shaft_diameter_;
    arrow.scale.y = head_diameter_;
    arrow.scale.z = head_length_;
    arrow.color.r = 0.0f;
    arrow.color.g = 0.8f;
    arrow.color.b = 1.0f;
    arrow.color.a = 0.9f;
    arr.markers.push_back(arrow);

    visualization_msgs::msg::Marker sphere;
    sphere.header = arrow.header;
    sphere.ns = "trajectory_exec";
    sphere.id = 1;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position = pos;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = position_marker_scale_;
    sphere.color.r = 0.0f;
    sphere.color.g = 0.8f;
    sphere.color.b = 1.0f;
    sphere.color.a = 0.9f;
    arr.markers.push_back(sphere);

    ve.marker_pub->publish(arr);
}

void TrajectoryExecutorNode::publish_plot_topics(const VehicleExec& ve, const geometry_msgs::msg::Vector3& vel,
                                                 const geometry_msgs::msg::Vector3& accel) const {
    std_msgs::msg::Float64 speed_msg;
    speed_msg.data = std::hypot(std::hypot(vel.x, vel.y), vel.z);
    ve.speed_pub->publish(speed_msg);

    std_msgs::msg::Float64 accel_msg;
    accel_msg.data = std::hypot(std::hypot(accel.x, accel.y), accel.z);
    ve.accel_mag_pub->publish(accel_msg);
}

}  // namespace gcs_planner
