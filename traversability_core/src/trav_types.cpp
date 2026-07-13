// trav_types.cpp
#include "traversability_core/trav_types.hpp"

#include <algorithm>
#include <cmath>

namespace trav {

int GridInfo::index(int ix, int iy) const { return iy * nx + ix; }

Eigen::Vector2d GridInfo::cell_center(int ix, int iy) const {
    return origin + resolution * Eigen::Vector2d(ix + 0.5, iy + 0.5);
}

bool GridInfo::world_to_cell(const Eigen::Vector2d& p, int& ix, int& iy) const {
    const Eigen::Vector2d local = (p - origin) / resolution;
    ix = static_cast<int>(std::floor(local.x()));
    iy = static_cast<int>(std::floor(local.y()));
    return ix >= 0 && ix < nx && iy >= 0 && iy < ny;
}

const TraversabilityCell& TraversabilityMap::at(int ix, int iy) const {
    return cells[info.index(ix, iy)];
}

TraversabilityCell& TraversabilityMap::at(int ix, int iy) {
    return cells[info.index(ix, iy)];
}

bool OccupancyGrid2D::occupied(int ix, int iy) const {
    if (ix < 0 || ix >= info.nx || iy < 0 || iy >= info.ny) return true;
    return occ[info.index(ix, iy)] != 0;
}

MatrixXd OccupancyGrid2D::occupied_centers() const {
    int count = 0;
    for (int iy = 0; iy < info.ny; ++iy)
        for (int ix = 0; ix < info.nx; ++ix)
            if (occupied(ix, iy)) ++count;

    MatrixXd out(count, 2);
    int r = 0;
    for (int iy = 0; iy < info.ny; ++iy)
        for (int ix = 0; ix < info.nx; ++ix)
            if (occupied(ix, iy)) out.row(r++) = info.cell_center(ix, iy).transpose();
    return out;
}

Eigen::Vector2d OccupancyGrid2D::nearest_free(const Eigen::Vector2d& p) const {
    double best = std::numeric_limits<double>::infinity();
    Eigen::Vector2d best_pt = p;
    for (int iy = 0; iy < info.ny; ++iy)
        for (int ix = 0; ix < info.nx; ++ix) {
            if (occupied(ix, iy)) continue;
            const Eigen::Vector2d c = info.cell_center(ix, iy);
            const double d = (c - p).squaredNorm();
            if (d < best) { best = d; best_pt = c; }
        }
    return best_pt;
}

GridInfo grid_from_cloud(const MatrixXd& cloud, double resolution, double margin) {
    GridInfo info;
    info.resolution = resolution;
    if (cloud.rows() == 0) return info;

    const double minx = cloud.col(0).minCoeff() - margin;
    const double miny = cloud.col(1).minCoeff() - margin;
    const double maxx = cloud.col(0).maxCoeff() + margin;
    const double maxy = cloud.col(1).maxCoeff() + margin;
    info.origin = Eigen::Vector2d(minx, miny);
    info.nx = std::max(1, static_cast<int>(std::ceil((maxx - minx) / resolution)));
    info.ny = std::max(1, static_cast<int>(std::ceil((maxy - miny) / resolution)));
    return info;
}

}  // namespace trav
