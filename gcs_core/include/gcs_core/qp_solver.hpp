// qp_solver.hpp
#pragma once

#include "gcs_core/gcs_types.hpp"

#include <string>

namespace gcs {

/**
 * @brief Result of a convex quadratic program (QP) solve.
 */
struct QPResult {
    VectorXd x;
    bool ok = false;
    std::string status;
};

/**
 * @brief Solves a convex QP: minimize 1/2 x^T P x + c^T x subject to l <= G x <= u.
 * @remark Uses OSQP; dense P/G are converted to sparse CSC internally (problem
 * sizes here are small, so dense assembly stays simple).
 * @param P Quadratic cost matrix (must be positive semi-definite).
 * @param c Linear cost vector.
 * @param G Constraint matrix.
 * @param l Lower bounds for G x.
 * @param u Upper bounds for G x.
 * @return QPResult with x populated when ok is true; status holds the OSQP solver status string.
 */
QPResult solve_qp(const MatrixXd& P, const VectorXd& c,
                  const MatrixXd& G, const VectorXd& l, const VectorXd& u);

}  // namespace gcs
