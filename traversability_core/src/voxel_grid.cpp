// voxel_grid.cpp
#include "traversability_core/voxel_grid.hpp"

#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace trav {

MatrixXd voxel_downsample(const MatrixXd& cloud, double leaf) {
    if (cloud.rows() == 0 || leaf <= 0.0) return cloud;

    struct Acc { Eigen::Vector3d sum = Eigen::Vector3d::Zero(); int n = 0; };
    std::unordered_map<uint64_t, Acc> vox;
    vox.reserve(static_cast<size_t>(cloud.rows()));

    auto key = [](int i, int j, int k) -> uint64_t {
        const uint64_t xi = static_cast<uint64_t>(i + (1 << 20)) & 0x1FFFFF;
        const uint64_t yj = static_cast<uint64_t>(j + (1 << 20)) & 0x1FFFFF;
        const uint64_t zk = static_cast<uint64_t>(k + (1 << 20)) & 0x1FFFFF;
        return (xi << 42) | (yj << 21) | zk;
    };

    for (int r = 0; r < cloud.rows(); ++r) {
        const int i = static_cast<int>(std::floor(cloud(r, 0) / leaf));
        const int j = static_cast<int>(std::floor(cloud(r, 1) / leaf));
        const int k = static_cast<int>(std::floor(cloud(r, 2) / leaf));
        Acc& a = vox[key(i, j, k)];
        a.sum += cloud.row(r).head<3>().transpose();
        a.n += 1;
    }

    MatrixXd out(static_cast<int>(vox.size()), 3);
    int r = 0;
    for (const auto& kv : vox) out.row(r++) = (kv.second.sum / kv.second.n).transpose();
    return out;
}

ColumnBins bin_into_columns(const MatrixXd& cloud, const GridInfo& info) {
    ColumnBins bins;
    bins.info = info;
    bins.cells.assign(static_cast<size_t>(info.nx) * info.ny, {});

    for (int r = 0; r < cloud.rows(); ++r) {
        int ix, iy;
        const Eigen::Vector2d p(cloud(r, 0), cloud(r, 1));
        if (info.world_to_cell(p, ix, iy)) bins.cells[info.index(ix, iy)].push_back(r);
    }
    return bins;
}

}  // namespace trav
