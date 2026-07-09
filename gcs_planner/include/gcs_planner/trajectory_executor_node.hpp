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

/**
 * @brief One playback lane: subscribes to one planner_msgs/Trajectory topic and holds
 * that vehicle's own animation/playback state, independent of every other vehicle's lane.
 */
struct VehicleExec {
    std::string vehicle;

    rclcpp::Subscription<planner_msgs::msg::Trajectory>::SharedPtr traj_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr accel_mag_pub;

    planner_msgs::msg::Trajectory traj;
    bool has_traj = false;
    int  last_loop_index = 0;
    rclcpp::Time playback_start;

    std::vector<std::pair<double, double>> keyframes;
    double total_anim_duration = 0.0;
};

/**
 * @brief Subscribes to one or more planner_msgs/Trajectory topics (one per vehicle,
 * via the `vehicles` param) and plays each back independently in wall-clock time.
 * @remark Pauses for segment_pause_sec at the end of each per-PathSegment leg (and at
 * the very end) before continuing, then loops back to t=0. Per vehicle, publishes a
 * MarkerArray each tick: an ARROW whose tail sits at the vehicle's current position
 * and whose tip is offset by velocity * scale (length/direction represent speed and
 * heading), plus a SPHERE marking the current position (useful when velocity ~ 0,
 * e.g. at rest-to-rest segment endpoints, where the arrow collapses to a point).
 * Also publishes speed and acceleration magnitude each tick as std_msgs/Float64
 * topics, meant to be plotted live with rqt_plot.
 */
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

    /**
     * @brief Rebuilds ve.keyframes/ve.total_anim_duration from ve.traj.points.
     * @remark Builds an animation-time -> trajectory-time piecewise-linear map that
     * holds trajectory-time flat for segment_pause_sec_ at each detected
     * per-PathSegment boundary (consecutive points with ~equal time_from_start -- see
     * build_trajectory_msg() in trajectory_msg_builders.cpp, which always samples the
     * exact rest-to-rest endpoint of each segment) and at the final endpoint. Called
     * with state_mtx_ already held.
     * @param ve The vehicle lane to rebuild keyframes for.
     */
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
