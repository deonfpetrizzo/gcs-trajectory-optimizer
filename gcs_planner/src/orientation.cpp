// orientation.cpp
#include "gcs_planner/orientation.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace gcs_planner {

void attitude_from_accel_yaw(const Eigen::Vector3d& accel, double yaw,
                             double& roll, double& pitch) {
    const Eigen::Vector3d gravity(0.0, 0.0, -kGravity);
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

}  // namespace gcs_planner
