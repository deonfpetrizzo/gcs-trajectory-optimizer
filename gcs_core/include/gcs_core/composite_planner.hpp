// composite_planner.hpp
#pragma once

#include "gcs_core/gcs_types.hpp"
#include "gcs_core/bezier_planner.hpp"

#include <optional>
#include <utility>
#include <vector>

namespace gcs {

/**
 * @brief Joint shape + timing composite Bezier trajectory: r(t) = q_i(h_i^-1(t)),
 * a geometry curve q(s) and a monotone time map h(s) per segment (Marcucci, Sec. 11.4.1).
 */
struct CompositeTimingTrajectory {
    std::vector<MatrixXd> q_cps;
    std::vector<MatrixXd> h_cps;
    std::vector<int>      region_sequence;
    int    degree = 0;
    double cost = 0.0;

    double total_duration() const { return h_cps.back()(h_cps.back().rows() - 1, 0); }

    /**
     * @brief Position r(t).
     * @remark Locates the segment/parameter s via binary search over the monotone
     * time map h(s), then evaluates q(s).
     * @param t Query time.
     * @return Position at time t.
     */
    VectorXd position(double t) const;

    /**
     * @brief Velocity dr/dt.
     * @param t Query time.
     * @return Velocity at time t.
     */
    VectorXd velocity(double t) const;

    /**
     * @brief Acceleration d2r/dt2.
     * @param t Query time.
     * @return Acceleration at time t.
     */
    VectorXd acceleration(double t) const;

private:
    VectorXd q_eval(int i, double s, int order) const;
    double   h_eval(int i, double s, int order) const;
    std::pair<int, double> locate(double t) const;
};

/**
 * @brief Joint shape+timing GCS Bezier planner (Marcucci, Sec. 11.4.1), extending
 * GCSBezierPlanner with a monotone per-segment time map.
 * @remark The velocity constraint ||r'(t)|| <= v_max is a second-order cone; OSQP has
 * no native SOC, so a polyhedral inner approximation (soc_facets linear facets) is
 * used -- conservative (slightly tighter than the exact cone, never looser).
 */
class GCSCompositeBezierPlanner : public GCSBezierPlanner {
public:
    using GCSBezierPlanner::GCSBezierPlanner;

    struct Options {
        int    degree = 6;
        int    continuity = 2;
        std::optional<double> vel_limit;
        std::optional<double> geom_accel_limit;
        std::optional<double> time_accel_limit;
        std::optional<VectorXd> v0, vT;
        bool   rest_to_rest_accel = false;  
        double T_min = 0.0, T_max = 1e9;
        double h_dot_min = 1e-2;
        double w_time = 1.0;
        double w_geom_accel = 1e-2;
        double w_time_accel = 1e-2;
        double w_geom_jerk = 0.0;
        double w_geom_length = 0.0;
        int    soc_facets = 16;     
    };

    /**
     * @brief Solves the joint shape+timing QP for a fixed region sequence.
     * @remark Marcucci Sec. 11.4.1; velocity limit enforced via the polyhedral SOC
     * approximation described on this class.
     * @param seq Region sequence (e.g. from find_region_sequence()).
     * @param q0_pos Start position.
     * @param qT_pos Goal position.
     * @param opt Degree/continuity/limits/cost-weight options.
     * @return The optimized composite (shape + timing) trajectory.
     */
    CompositeTimingTrajectory solve_composite(const std::vector<int>& seq,
                                              const VectorXd& q0_pos,
                                              const VectorXd& qT_pos,
                                              const Options& opt);

    /**
     * @brief Routes from q0_pos to qT_pos and solves the resulting composite QP.
     * @remark Composes find_region_sequence() (Dijkstra) with solve_composite() (QP).
     * @param q0_pos Start position.
     * @param qT_pos Goal position.
     * @param opt Degree/continuity/limits/cost-weight options.
     * @return The optimized composite (shape + timing) trajectory.
     */
    CompositeTimingTrajectory plan_composite(const VectorXd& q0_pos,
                                             const VectorXd& qT_pos,
                                             const Options& opt);
};

}  // namespace gcs
