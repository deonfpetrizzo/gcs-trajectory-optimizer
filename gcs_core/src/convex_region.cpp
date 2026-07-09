// convex_region.cpp
#include "gcs_core/convex_region.hpp"

#include "gcs_core/qp_solver.hpp"

#include <osqp.h> 

namespace gcs {

std::pair<VectorXd, double> ConvexRegion::chebyshev_center() const {
    const int d = dim();
    const int m = A.rows();
    const int nv = d + 1;
    VectorXd norms = A.rowwise().norm();

    MatrixXd P = MatrixXd::Zero(nv, nv);
    P(d, d) = 1e-9;
    VectorXd c = VectorXd::Zero(nv);
    c(d) = -1.0;                          

    MatrixXd G(m + 1, nv);
    VectorXd lo(m + 1), up(m + 1);
    G.setZero();
    G.block(0, 0, m, d) = A;
    G.block(0, d, m, 1) = norms;
    lo.head(m).setConstant(-OSQP_INFTY);
    up.head(m) = b;
    G.row(m).setZero(); G(m, d) = 1.0;
    lo(m) = 0.0; up(m) = OSQP_INFTY;

    QPResult r = solve_qp(P, c, G, lo, up);
    if (!r.ok) return {VectorXd::Zero(d), 0.0};
    return {r.x.head(d), r.x(d)};
}

bool ConvexRegion::intersects(const ConvexRegion& other, double tol) const {
    const int d = dim();
    const int m1 = A.rows(), m2 = other.A.rows();
    MatrixXd P = 1e-9 * MatrixXd::Identity(d, d);
    VectorXd c = VectorXd::Zero(d);
    MatrixXd G(m1 + m2, d);
    VectorXd lo(m1 + m2), up(m1 + m2);
    G.topRows(m1) = A;
    up.head(m1) = b.array() + tol;
    G.bottomRows(m2) = other.A;
    up.tail(m2) = other.b.array() + tol;
    lo.setConstant(-OSQP_INFTY);
    QPResult r = solve_qp(P, c, G, lo, up);
    return r.ok;
}

}  // namespace gcs
