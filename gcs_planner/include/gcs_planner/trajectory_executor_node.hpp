#ifndef GCS_PLANNER__TRAJECTORY_EXECUTOR_NODE_HPP_
#define GCS_PLANNER__TRAJECTORY_EXECUTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <planner_msgs/msg/trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>

#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace gcs_planner {

// One playback lane: subscribes to one planner_msgs/Trajectory topic and
// holds that vehicle's own animation/playback state, independent of every
// other vehicle's lane.
struct VehicleExec {
    std::string vehicle;  // "" in single-vehicle (legacy, unnamespaced-topic) mode

    rclcpp::Subscription<planner_msgs::msg::Trajectory>::SharedPtr traj_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr accel_mag_pub;

    // Playback state, guarded by TrajectoryExecutorNode::state_mtx_: written
    // by on_trajectory() (subscription callback), read by on_timer() (timer
    // callback) -- both run on the same single-threaded executor, but the
    // lock keeps this safe regardless of executor type.
    planner_msgs::msg::Trajectory traj;
    bool has_traj = false;
    int  last_loop_index = 0;
    rclcpp::Time playback_start;
    // (anim_time, traj_time) keyframes; anim_to_traj_time() interpolates
    // between them, holding traj_time constant across a pause's two
    // keyframes (same traj_time, segment_pause_sec_ apart in anim_time).
    std::vector<std::pair<double, double>> keyframes;
    double total_anim_duration = 0.0;
};

// Subscribes to one or more planner_msgs/Trajectory topics (one per vehicle,
// via the `vehicles` param) and plays each back independently in wall-clock
// time, pausing for segment_pause_sec at the end of each per-PathSegment leg
// (and at the very end) before continuing, then looping back to t=0. Per
// vehicle, publishes a MarkerArray each tick: an ARROW whose tail sits at the
// vehicle's current position and whose tip is offset by velocity * scale (so
// its length/direction directly represent speed and heading), plus a SPHERE
// marking the current position (useful when velocity ~ 0, e.g. at
// rest-to-rest segment endpoints, where the arrow collapses to a point).
// Also publishes the current speed (||velocity||) and acceleration magnitude
// each tick as plain std_msgs/Float64 topics, meant to be plotted live with
// rqt_plot.
class TrajectoryExecutorNode : public rclcpp::Node {
public:
    TrajectoryExecutorNode();

private:
    void on_trajectory(size_t idx, const planner_msgs::msg::Trajectory::SharedPtr msg);
    void on_timer();
    void publish_markers(const VehicleExec& ve, const std::string& frame_id,
                         const geometry_msgs::msg::Point& pos,
                         const geometry_msgs::msg::Vector3& vel) const;
    void publish_plot_topics(const VehicleExec& ve, const geometry_msgs::msg::Vector3& vel,
                             const geometry_msgs::msg::Vector3& accel) const;

    // Rebuilds ve.keyframes/ve.total_anim_duration from ve.traj.points: an
    // animation-time -> trajectory-time piecewise-linear map that holds
    // trajectory-time flat for segment_pause_sec_ at each detected
    // per-PathSegment boundary (consecutive points with ~equal
    // time_from_start -- see to_msg() in planner_node.cpp, which always
    // samples the exact rest-to-rest endpoint of each segment) and at the
    // final endpoint. Called with state_mtx_ already held.
    void build_keyframes(VehicleExec& ve);

    // params
    double vector_time_scale_;
    double shaft_diameter_, head_diameter_, head_length_;
    double position_marker_scale_;
    double viz_rate_hz_;
    double segment_pause_sec_;
    std::string frame_id_override_;

    // ROS handles
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex state_mtx_;
    std::vector<VehicleExec> vehicles_;
};

}  // namespace gcs_planner
#endif  // GCS_PLANNER__TRAJECTORY_EXECUTOR_NODE_HPP_
