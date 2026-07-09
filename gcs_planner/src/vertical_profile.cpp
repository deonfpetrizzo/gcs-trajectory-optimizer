// vertical_profile.cpp
#include "gcs_planner/vertical_profile.hpp"

#include <algorithm>
#include <cmath>

namespace gcs_planner {

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

}  // namespace gcs_planner
