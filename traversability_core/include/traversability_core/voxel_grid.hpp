// voxel_grid.hpp
#pragma once

#include "traversability_core/trav_types.hpp"

#include <vector>

namespace trav {

/**
 * @brief Column binning of a point cloud over a 2D grid.
 * @remark cells[GridInfo::index(ix,iy)] holds the row indices (into the source
 * cloud) of points whose xy falls in that cell.
 */
struct ColumnBins {
    GridInfo info;
    std::vector<std::vector<int>> cells;
};

/**
 * @brief Voxel-grid downsample of a point cloud.
 * @remark One representative point (centroid) per occupied voxel.
 * @param cloud Input cloud (rows = points, cols = x,y,z).
 * @param leaf Voxel edge length.
 * @return Downsampled cloud (rows = points).
 */
MatrixXd voxel_downsample(const MatrixXd& cloud, double leaf);

/**
 * @brief Bins a point cloud into 2D columns over the given grid.
 * @param cloud Input cloud (rows = points, cols = x,y,z).
 * @param info Target grid geometry.
 * @return Per-cell lists of contributing point indices.
 */
ColumnBins bin_into_columns(const MatrixXd& cloud, const GridInfo& info);

}  // namespace trav
