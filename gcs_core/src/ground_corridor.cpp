// ground_corridor.cpp
#include "gcs_core/ground_corridor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gcs {

namespace {

VectorXd project_2d(const VectorXd& p) {
    VectorXd q(2);
    q << p(0), p(1);
    return q;
}

double estimate_ground_z(const MatrixXd& cloud, double cx, double cy, double radius) {
    const double r2 = radius * radius;
    std::vector<double> zs;
    for (int i = 0; i < cloud.rows(); ++i) {
        const double dx = cloud(i, 0) - cx, dy = cloud(i, 1) - cy;
        if (dx * dx + dy * dy > r2) continue;
        zs.push_back(cloud(i, 2));
    }
    if (zs.empty()) return std::numeric_limits<double>::infinity();

    const size_t k = static_cast<size_t>(0.1 * (zs.size() - 1));
    std::nth_element(zs.begin(), zs.begin() + k, zs.end());
    return zs[k];
}

}  // namespace

CorridorResult build_ground_corridor(const VectorXd& q0, const VectorXd& qT,
                                     const std::vector<VectorXd>& nominal_path,
                                     const MatrixXd& occupied_xy,
                                     const MatrixXd& cloud3d,
                                     const GroundCorridorOptions& opt,
                                     const GroundPlaneSampler& ground_plane) {
    std::vector<VectorXd> path2d;
    path2d.reserve(nominal_path.size());
    for (const auto& p : nominal_path) path2d.push_back(project_2d(p));

    CorridorResult cr2d = build_corridor_parallel(project_2d(q0), project_2d(qT), path2d,
                                                  occupied_xy, opt.corridor2d);

    CorridorResult out;
    out.connected = cr2d.connected;
    out.rounds = cr2d.rounds;
    out.regions.reserve(cr2d.regions.size());

    for (const auto& poly2d : cr2d.regions) {
        const VectorXd c2 = poly2d.chebyshev_center().first;
        const double cx = c2(0), cy = c2(1);

        GroundPlaneFit plane;
        if (!ground_plane || !ground_plane(cx, cy, plane)) {
            double z0 = estimate_ground_z(cloud3d, cx, cy, opt.plane_fit_radius);
            if (!std::isfinite(z0)) z0 = q0(2);

            VectorXd center3d(3);
            center3d << cx, cy, z0;
            plane = fit_local_ground_plane(cloud3d, center3d, opt.plane_fit_radius,
                                           opt.plane_fit_z_window);
        }
        out.regions.push_back(
            lift_polygon_to_prism(poly2d, plane, opt.floor_offset, opt.robot_height));
    }
    return out;
}

}  // namespace gcs
