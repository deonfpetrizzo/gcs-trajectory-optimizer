// constraint_builder.hpp
#pragma once

#include "gcs_core/gcs_types.hpp"

#include <Eigen/Sparse>
#include <osqp.h> 

#include <utility>
#include <vector>

namespace gcs {
namespace detail {

/**
 * @brief Accumulates linear constraint rows (l <= G x <= u) as sparse triplets for a
 * QP, then materializes the dense G/l/u that qp_solver::solve_qp() consumes. Shared
 * by the geometry-only and composite Bezier planners.
 */
struct ConstraintBuilder {
    int nvar;
    std::vector<Eigen::Triplet<double>> trips;
    std::vector<double> lo, up;
    int row = 0;
    explicit ConstraintBuilder(int n) : nvar(n) {}

    /**
     * @brief Appends |rhs| equality rows tying block M (placed at column col0) to rhs.
     * @param M Dense constraint block.
     * @param col0 Starting column offset for M's columns in the global variable vector.
     * @param rhs Right-hand side value for each row.
     */
    void add_block_eq(const MatrixXd& M, int col0, const VectorXd& rhs) {
        for (int i = 0; i < M.rows(); ++i) {
            for (int j = 0; j < M.cols(); ++j)
                if (M(i, j) != 0.0) trips.emplace_back(row + i, col0 + j, M(i, j));
            lo.push_back(rhs(i)); up.push_back(rhs(i));
        }
        row += M.rows();
    }

    /**
     * @brief Appends inequality rows M x <= ub (block placed at column col0); lower
     * bound is -infinity.
     * @param M Dense constraint block.
     * @param col0 Starting column offset for M's columns in the global variable vector.
     * @param ub Upper bound for each row.
     */
    void add_block_le(const MatrixXd& M, int col0, const VectorXd& ub) {
        for (int i = 0; i < M.rows(); ++i) {
            for (int j = 0; j < M.cols(); ++j)
                if (M(i, j) != 0.0) trips.emplace_back(row + i, col0 + j, M(i, j));
            lo.push_back(-OSQP_INFTY); up.push_back(ub(i));
        }
        row += M.rows();
    }

    /**
     * @brief Appends one constraint row l <= sum(terms) <= u.
     * @param terms Sparse (column, coefficient) pairs for this row.
     * @param l Lower bound.
     * @param u Upper bound.
     */
    void add_row(const std::vector<std::pair<int, double>>& terms, double l, double u) {
        for (auto& [c, v] : terms) trips.emplace_back(row, c, v);
        lo.push_back(l); up.push_back(u); ++row;
    }

    MatrixXd dense() const {
        MatrixXd G = MatrixXd::Zero(row, nvar);
        for (auto& t : trips) G(t.row(), t.col()) += t.value();
        return G;
    }
    VectorXd lvec() const { return Eigen::Map<const VectorXd>(lo.data(), lo.size()); }
    VectorXd uvec() const { return Eigen::Map<const VectorXd>(up.data(), up.size()); }
};

}  // namespace detail
}  // namespace gcs
