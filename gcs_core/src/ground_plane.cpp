// ground_plane.cpp
#include "gcs_core/ground_plane.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace gcs {

GroundPlaneFit fit_local_ground_plane(const MatrixXd& cloud, const VectorXd& center,
                                      double xy_radius, double z_window) {
    GroundPlaneFit fit;
    fit.c = center(2);

    std::vector<int> near;
    for (int i = 0; i < cloud.rows(); ++i) {
        double dx = cloud(i, 0) - center(0), dy = cloud(i, 1) - center(1);
        if (dx * dx + dy * dy > xy_radius * xy_radius) continue;
        if (std::abs(cloud(i, 2) - center(2)) > z_window) continue;
        near.push_back(i);
    }
    if (near.size() < 8) return fit;

    MatrixXd M(near.size(), 3);
    VectorXd zc(near.size());
    for (size_t k = 0; k < near.size(); ++k) {
        M(k, 0) = cloud(near[k], 0);
        M(k, 1) = cloud(near[k], 1);
        M(k, 2) = 1.0;
        zc(k) = cloud(near[k], 2);
    }

    Eigen::JacobiSVD<MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    VectorXd sv = svd.singularValues();
    if (sv(sv.size() - 1) < 1e-6 * sv(0)) return fit;

    VectorXd coeffs = svd.solve(zc);
    fit.a = coeffs(0);
    fit.b = coeffs(1);
    fit.c = coeffs(2);
    fit.tilted = true;
    return fit;
}

ConvexRegion clamp_region_to_ground_band(const ConvexRegion& reg, const GroundPlaneFit& plane,
                                         double half_width) {
    const int d = reg.dim();
    if (d != 3) throw std::invalid_argument("clamp_region_to_ground_band requires a 3D region");

    MatrixXd A2(reg.A.rows() + 2, d);
    VectorXd b2(reg.b.size() + 2);
    A2.topRows(reg.A.rows()) = reg.A;
    b2.head(reg.b.size()) = reg.b;

    A2.row(reg.A.rows()) << -plane.a, -plane.b, 1.0;
    b2(reg.b.size()) = plane.c + half_width;
    A2.row(reg.A.rows() + 1) << plane.a, plane.b, -1.0;
    b2(reg.b.size() + 1) = -plane.c + half_width;

    return ConvexRegion(A2, b2, reg.name);
}

}  // namespace gcs
