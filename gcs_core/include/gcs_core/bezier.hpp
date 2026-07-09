// bezier.hpp
#pragma once

#include "gcs_core/gcs_types.hpp"

#include <vector>

namespace gcs {

/**
 * @brief Map from control points to the control points of the k-th derivative, for a
 * degree-N Bezier over an interval of the given duration.
 * @remark Bernstein/Bezier basis algebra.
 * @param N Bezier degree.
 * @param duration Interval length.
 * @param k Derivative order.
 * @return Operator matrix of shape ((N+1-k) x (N+1)).
 */
MatrixXd derivative_operator(int N, double duration, int k);

/**
 * @brief Symmetric PSD energy matrix M (N+1 x N+1) such that
 * integral_L ||g(s)||^2 ds = L * sum_{n,m} M(n,m) g_n . g_m.
 * @remark Bernstein/Bezier basis algebra.
 * @param N Bezier degree.
 * @return The energy matrix.
 */
MatrixXd energy_matrix(int N);

/**
 * @brief Bernstein basis of degree N evaluated at tau.
 * @param N Bezier degree.
 * @param tau Evaluation parameter in [0,1].
 * @return Basis values, one per control point.
 */
VectorXd bernstein_basis(int N, double tau);

/**
 * @brief Geometry-only piecewise Bezier curve.
 */
struct BezierTrajectory {
    std::vector<MatrixXd> control_points; 
    std::vector<double>   durations;       
    std::vector<int>      region_sequence;
    int    degree = 0;
    double cost = 0.0;

    double total_duration() const;

    /**
     * @brief Evaluates the trajectory (or one of its derivatives) at time t.
     * @remark Bernstein basis evaluation of the segment located at t.
     * @param t Query time, clamped to [0, total_duration()].
     * @param order Derivative order (0 = position).
     * @return The value at time t.
     */
    VectorXd eval(double t, int order = 0) const;
};

}  // namespace gcs
