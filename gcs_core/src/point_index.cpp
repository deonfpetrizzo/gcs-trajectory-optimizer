// point_index.cpp
#include "gcs_core/point_index.hpp"

#include <cmath>

namespace gcs {
namespace {

std::array<int, 3> cell_of(const VectorXd& p, int dim, double cell) {
    std::array<int, 3> key{0, 0, 0};
    for (int j = 0; j < dim; ++j)
        key[j] = static_cast<int>(std::floor(p(j) / cell));
    return key;
}

}  // namespace

PointIndex::PointIndex(const MatrixXd& cloud, double cell)
    : cloud_(cloud), dim_(static_cast<int>(cloud.cols())), cell_(cell) {
    if (dim_ > 3) dim_ = 3;
    for (int i = 0; i < cloud_.rows(); ++i) {
        const CellKey key = cell_of(cloud_.row(i).transpose(), dim_, cell_);
        buckets_[key].push_back(i);
    }
}

void PointIndex::radius_query(const VectorXd& pq, double R, double eps,
                              std::vector<int>& out) const {
    if (buckets_.empty()) return;

    const int reach = std::max(1, static_cast<int>(std::ceil(R / cell_)));
    const CellKey base = cell_of(pq, dim_, cell_);

    std::array<int, 3> lo{0, 0, 0}, hi{0, 0, 0};
    for (int j = 0; j < dim_; ++j) {
        lo[j] = base[j] - reach;
        hi[j] = base[j] + reach;
    }

    CellKey key{0, 0, 0};
    for (int cx = lo[0]; cx <= hi[0]; ++cx) {
        key[0] = cx;
        for (int cy = lo[1]; cy <= hi[1]; ++cy) {
            key[1] = cy;
            for (int cz = lo[2]; cz <= hi[2]; ++cz) {
                key[2] = cz;
                auto it = buckets_.find(key);
                if (it == buckets_.end()) continue;
                for (int idx : it->second) {
                    const double dd = (cloud_.row(idx).transpose() - pq).norm();
                    if (dd > eps && dd < R) out.push_back(idx);
                }
            }
        }
    }
}

}  // namespace gcs
