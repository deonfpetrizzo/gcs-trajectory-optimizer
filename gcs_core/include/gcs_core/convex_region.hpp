// convex_region.hpp
#pragma once

#include "gcs_core/gcs_types.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace gcs {

/**
 * @brief H-representation convex polytope { x : A x <= b }.
 */
struct ConvexRegion {
    MatrixXd A;
    VectorXd b;
    std::string name;

    ConvexRegion() = default;
    ConvexRegion(MatrixXd A_, VectorXd b_, std::string name_ = "")
        : A(std::move(A_)), b(std::move(b_)), name(std::move(name_)) {
        if (A.rows() != b.size())
            throw std::invalid_argument("A and b have incompatible shapes");
    }
    int dim() const { return static_cast<int>(A.cols()); }
    bool contains(const VectorXd& x, double tol = 1e-7) const {
        return ((A * x).array() <= b.array() + tol).all();
    }

    /**
     * @brief Largest inscribed ball (Chebyshev center) of this region.
     * @remark Solved as a linear program formulated as a degenerate QP (via solve_qp).
     * @return Pair of (center, radius).
     */
    std::pair<VectorXd, double> chebyshev_center() const;

    /**
     * @brief Tests whether this region and another share at least one point.
     * @remark Solved as an LP feasibility check formulated as a degenerate QP (via solve_qp).
     * @param other The region to test intersection against.
     * @param tol Numerical tolerance for the feasibility check.
     * @return True if the two polytopes intersect (or nearly do, within tol).
     */
    bool intersects(const ConvexRegion& other, double tol = 1e-7) const;
};

}  // namespace gcs
