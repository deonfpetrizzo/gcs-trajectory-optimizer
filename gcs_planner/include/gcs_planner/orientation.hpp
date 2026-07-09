// orientation.hpp 
#ifndef GCS_PLANNER__ORIENTATION_HPP_
#define GCS_PLANNER__ORIENTATION_HPP_

#include <Eigen/Dense>
#include <planner_msgs/msg/trajectory_point.hpp>

namespace gcs_planner {

inline constexpr double kGravity = 9.80665;  // standard gravity, m/s^2

/**
 * @brief Derives roll/pitch from desired acceleration (yaw is passed through unchanged).
 * @remark Differential flatness (Mellinger & Kumar, ICRA 2011).
 * @param accel Desired acceleration, world frame.
 * @param yaw Heading of travel direction (passed through, not derived here).
 * @param roll Output roll.
 * @param pitch Output pitch.
 */
void attitude_from_accel_yaw(const Eigen::Vector3d& accel, double yaw,
                             double& roll, double& pitch);

/**
 * @brief Orientation quaternion for one trajectory point.
 * @remark Built via tf2::Quaternion::setRPY() from the point's roll/pitch/yaw fields.
 * @param pt Trajectory point with roll/pitch/yaw set.
 * @param qx Output quaternion x.
 * @param qy Output quaternion y.
 * @param qz Output quaternion z.
 * @param qw Output quaternion w.
 */
void trajectory_point_quaternion(const planner_msgs::msg::TrajectoryPoint& pt,
                                 double& qx, double& qy, double& qz, double& qw);

}  // namespace gcs_planner
#endif  // GCS_PLANNER__ORIENTATION_HPP_
