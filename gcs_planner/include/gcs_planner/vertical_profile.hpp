// vertical_profile.hpp
#ifndef GCS_PLANNER__VERTICAL_PROFILE_HPP_
#define GCS_PLANNER__VERTICAL_PROFILE_HPP_

#include <Eigen/Dense>

#include "gcs_core/composite_planner.hpp"

namespace gcs_planner {

/**
 * @brief Builds a plain rest-to-rest vertical "ease" trajectory from p0 to p1 (no
 * corridor-growth/QP optimization).
 * @remark A single-segment, degree-5 Bezier q(s) with its first/last 3 control points
 * collapsed onto p0/p1 -- the standard "smootherstep" construction, which forces zero
 * 1st and 2nd derivative at s=0 and s=1 (zero velocity and acceleration at both
 * endpoints, matching the rest_to_rest boundary conditions the QP planner enforces
 * elsewhere). The time map h(s) is an exactly-linear Bezier, so it reduces to
 * h(s) = T*s. For this profile |velocity| peaks at s=0.5 with factor 15/8 and
 * |acceleration| peaks with factor 10/sqrt(3); T is the larger of the two durations
 * needed to keep each peak within v_max/a_max.
 * @param p0 Start position.
 * @param p1 End position.
 * @param v_max Velocity limit.
 * @param a_max Acceleration limit.
 * @return The vertical ease trajectory.
 */
gcs::CompositeTimingTrajectory make_vertical_trajectory(const Eigen::VectorXd& p0,
                                                        const Eigen::VectorXd& p1,
                                                        double v_max, double a_max);

}  // namespace gcs_planner
#endif  // GCS_PLANNER__VERTICAL_PROFILE_HPP_
