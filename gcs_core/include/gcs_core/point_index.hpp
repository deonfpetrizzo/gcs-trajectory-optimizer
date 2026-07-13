// point_index.hpp
#pragma once

#include "gcs_core/gcs_types.hpp"

#include <array>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace gcs {

/**
 * @brief One-time uniform-grid spatial index for fixed-radius neighbor queries.
 * @remark Built once over a static obstacle cloud and reused for every region
 * grown during corridor building, replacing the per-region full-cloud scan.
 * Dimension-generic (2D ground polygons and 3D aerial clouds). Immutable after
 * construction: radius_query() is const and reads only shared buckets, so it is
 * safe to call concurrently from the corridor-growth worker threads.
 */
class PointIndex {
public:
    /**
     * @brief Buckets the cloud into a uniform grid of the given cell size.
     * @param cloud Obstacle point cloud (rows = points). Borrowed; must outlive this index.
     * @param cell Grid cell edge length. Choosing cell == query radius R makes each
     * query touch only the 3^d cells around the query point.
     */
    PointIndex(const MatrixXd& cloud, double cell);

    /**
     * @brief Appends the row indices of cloud points within (eps, R) of pq.
     * @param pq Query point.
     * @param R Query radius (exclusive upper bound on distance).
     * @param eps Exclusive lower bound on distance (excludes points at pq itself).
     * @param out Result indices are appended (not cleared) to this vector.
     */
    void radius_query(const VectorXd& pq, double R, double eps,
                      std::vector<int>& out) const;

    int dim() const { return dim_; }
    bool empty() const { return buckets_.empty(); }
    const MatrixXd& cloud() const { return cloud_; }

private:
    using CellKey = std::array<int, 3>;
    struct CellHash {
        std::size_t operator()(const CellKey& k) const {
            std::size_t h = 1469598103934665603ull;
            for (int v : k) {
                h ^= static_cast<std::size_t>(static_cast<uint32_t>(v));
                h *= 1099511628211ull;
            }
            return h;
        }
    };

    const MatrixXd& cloud_;
    int dim_;
    double cell_;
    std::unordered_map<CellKey, std::vector<int>, CellHash> buckets_;
};

}  // namespace gcs
