// traversability_map.cpp
#include "traversability_core/traversability_map.hpp"
#include "traversability_core/voxel_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace trav {

TraversabilityMap compute_traversability(const MatrixXd& cloud, const GridInfo& info,
                                         const TraversabilityParams& params) {
    TraversabilityMap map;
    map.info = info;
    map.cells.assign(static_cast<size_t>(info.nx) * info.ny, TraversabilityCell{});

    const ColumnBins bins = bin_into_columns(cloud, info);
    const int cell_radius = std::max(1, static_cast<int>(std::ceil(params.slope_fit_radius / info.resolution)));
    const double r2 = params.slope_fit_radius * params.slope_fit_radius;
    const double eps_clear = std::max(0.05, params.ground_z_window);

    for (int iy = 0; iy < info.ny; ++iy) {
        for (int ix = 0; ix < info.nx; ++ix) {
            TraversabilityCell& cell = map.at(ix, iy);
            const std::vector<int>& own = bins.cells[info.index(ix, iy)];
            const Eigen::Vector2d ctr = info.cell_center(ix, iy);

            std::vector<Eigen::Vector3d> near;
            for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
                const int jy = iy + dy;
                if (jy < 0 || jy >= info.ny) continue;
                for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
                    const int jx = ix + dx;
                    if (jx < 0 || jx >= info.nx) continue;
                    for (int idx : bins.cells[info.index(jx, jy)]) {
                        const double ddx = cloud(idx, 0) - ctr.x();
                        const double ddy = cloud(idx, 1) - ctr.y();
                        if (ddx * ddx + ddy * ddy > r2) continue;
                        near.emplace_back(cloud(idx, 0), cloud(idx, 1), cloud(idx, 2));
                    }
                }
            }
            if (near.empty()) continue;

            double g0 = std::numeric_limits<double>::infinity();
            for (const auto& p : near) g0 = std::min(g0, p.z());
            std::vector<Eigen::Vector3d> pts;
            for (const auto& p : near)
                if (std::abs(p.z() - g0) <= params.ground_z_window) pts.push_back(p);

            cell.point_count = static_cast<int>(pts.size());
            if (cell.point_count < params.min_points_per_cell) continue;

            Eigen::Vector3d c = Eigen::Vector3d::Zero();
            for (const auto& p : pts) c += p;
            c /= static_cast<double>(pts.size());
            Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
            for (const auto& p : pts) {
                const Eigen::Vector3d d = p - c;
                cov += d * d.transpose();
            }
            cov /= static_cast<double>(pts.size());

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
            Eigen::Vector3d n = es.eigenvectors().col(0);
            const double lambda_min = es.eigenvalues()(0);
            if (n.z() < 0.0) n = -n;

            cell.roughness = std::sqrt(std::max(lambda_min, 0.0));
            const double nz = n.z();
            if (nz < 1e-6) {
                cell.slope_rad = M_PI / 2.0;
                cell.plane_a = 0.0;
                cell.plane_b = 0.0;
                cell.plane_c = c.z();
            } else {
                cell.slope_rad = std::acos(std::min(1.0, nz));
                cell.plane_a = -n.x() / nz;
                cell.plane_b = -n.y() / nz;
                cell.plane_c = c.z() - cell.plane_a * c.x() - cell.plane_b * c.y();
            }
            cell.ground_z = cell.plane_a * ctr.x() + cell.plane_b * ctr.y() + cell.plane_c;

            double clear = std::numeric_limits<double>::infinity();
            for (int idx : own) {
                const double dz = cloud(idx, 2) - cell.ground_z;
                if (dz > eps_clear && dz <= params.clearance_ceiling) clear = std::min(clear, dz);
            }
            cell.min_overhead_clearance = clear;
            cell.valid = true;
        }
    }

    for (int iy = 0; iy < info.ny; ++iy) {
        for (int ix = 0; ix < info.nx; ++ix) {
            TraversabilityCell& cell = map.at(ix, iy);
            if (!cell.valid) continue;
            double step = 0.0;
            for (int dy = -1; dy <= 1; ++dy) {
                const int jy = iy + dy;
                if (jy < 0 || jy >= info.ny) continue;
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    const int jx = ix + dx;
                    if (jx < 0 || jx >= info.nx) continue;
                    const TraversabilityCell& nb = map.at(jx, jy);
                    if (!nb.valid) continue;
                    step = std::max(step, std::abs(cell.ground_z - nb.ground_z));
                }
            }
            cell.step_height = step;
        }
    }
    return map;
}

void smooth_ground_field(TraversabilityMap& map, double sigma_cells) {
    if (sigma_cells <= 0.0) return;
    const GridInfo& info = map.info;
    const int rad = std::max(1, static_cast<int>(std::ceil(3.0 * sigma_cells)));
    const double s2 = 2.0 * sigma_cells * sigma_cells;

    std::vector<double> smoothed(map.cells.size(), 0.0);
    for (int iy = 0; iy < info.ny; ++iy) {
        for (int ix = 0; ix < info.nx; ++ix) {
            const TraversabilityCell& cell = map.at(ix, iy);
            if (!cell.valid) continue;
            double wsum = 0.0, acc = 0.0;
            for (int dy = -rad; dy <= rad; ++dy) {
                const int jy = iy + dy;
                if (jy < 0 || jy >= info.ny) continue;
                for (int dx = -rad; dx <= rad; ++dx) {
                    const int jx = ix + dx;
                    if (jx < 0 || jx >= info.nx) continue;
                    const TraversabilityCell& nb = map.at(jx, jy);
                    if (!nb.valid) continue;
                    const double w = std::exp(-static_cast<double>(dx * dx + dy * dy) / s2);
                    acc += w * nb.ground_z;
                    wsum += w;
                }
            }
            smoothed[info.index(ix, iy)] = (wsum > 0.0) ? acc / wsum : cell.ground_z;
        }
    }
    for (int iy = 0; iy < info.ny; ++iy)
        for (int ix = 0; ix < info.nx; ++ix)
            if (map.at(ix, iy).valid) map.at(ix, iy).ground_z = smoothed[info.index(ix, iy)];

    const double h = info.resolution;
    for (int iy = 0; iy < info.ny; ++iy) {
        for (int ix = 0; ix < info.nx; ++ix) {
            TraversabilityCell& cell = map.at(ix, iy);
            if (!cell.valid) continue;
            auto gz = [&](int jx, int jy, double fallback) {
                if (jx < 0 || jx >= info.nx || jy < 0 || jy >= info.ny) return fallback;
                const TraversabilityCell& c = map.at(jx, jy);
                return c.valid ? c.ground_z : fallback;
            };
            const double gzc = cell.ground_z;
            const double a = (gz(ix + 1, iy, gzc) - gz(ix - 1, iy, gzc)) / (2.0 * h);
            const double b = (gz(ix, iy + 1, gzc) - gz(ix, iy - 1, gzc)) / (2.0 * h);
            const Eigen::Vector2d ctr = info.cell_center(ix, iy);
            cell.plane_a = a;
            cell.plane_b = b;
            cell.plane_c = gzc - a * ctr.x() - b * ctr.y();
            cell.slope_rad = std::atan(std::sqrt(a * a + b * b));
        }
    }
}

}  // namespace trav
