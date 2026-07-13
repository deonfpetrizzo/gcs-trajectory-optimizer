// trav_types.hpp
#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <limits>
#include <vector>

namespace trav {

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * @brief 2D grid geometry: origin, resolution, dimensions, and index/world mapping.
 */
struct GridInfo {
    Eigen::Vector2d origin = Eigen::Vector2d::Zero();
    double resolution = 0.1;
    int nx = 0;
    int ny = 0;

    /**
     * @brief Row-major flat index of cell (ix, iy).
     * @param ix Column index.
     * @param iy Row index.
     * @return Flat index into a cells/occ vector.
     */
    int index(int ix, int iy) const;

    /**
     * @brief World-space xy of a cell's center.
     * @param ix Column index.
     * @param iy Row index.
     * @return Cell-center coordinates.
     */
    Eigen::Vector2d cell_center(int ix, int iy) const;

    /**
     * @brief Maps a world point to the cell containing it.
     * @param p World-space xy point.
     * @param ix Output column index.
     * @param iy Output row index.
     * @return True if the point falls within grid bounds.
     */
    bool world_to_cell(const Eigen::Vector2d& p, int& ix, int& iy) const;
};

/**
 * @brief Per-column terrain features used to decide traversability.
 * @remark plane_{a,b,c} are the locally-fit ground plane z ~= a*x + b*y + c;
 * slope/roughness derive from that fit, step_height from neighbour discontinuity,
 * and min_overhead_clearance from a column scan above ground_z.
 */
struct TraversabilityCell {
    double ground_z = 0.0;
    double plane_a = 0.0;
    double plane_b = 0.0;
    double plane_c = 0.0;
    double slope_rad = 0.0;
    double roughness = 0.0;
    double step_height = 0.0;
    double min_overhead_clearance = std::numeric_limits<double>::infinity();
    int    point_count = 0;
    bool   valid = false;
};

/**
 * @brief Dense grid of TraversabilityCell over a GridInfo.
 */
struct TraversabilityMap {
    GridInfo info;
    std::vector<TraversabilityCell> cells;

    /**
     * @brief Const cell accessor by grid indices.
     * @param ix Column index.
     * @param iy Row index.
     * @return Const reference to the cell at (ix, iy).
     */
    const TraversabilityCell& at(int ix, int iy) const;

    /**
     * @brief Mutable cell accessor by grid indices.
     * @param ix Column index.
     * @param iy Row index.
     * @return Mutable reference to the cell at (ix, iy).
     */
    TraversabilityCell& at(int ix, int iy);
};

/**
 * @brief Binary 2D occupancy grid (1 = occupied/untraversable).
 */
struct OccupancyGrid2D {
    GridInfo info;
    std::vector<uint8_t> occ;

    /**
     * @brief Occupancy state of a cell.
     * @param ix Column index.
     * @param iy Row index.
     * @return True if the cell is occupied (out-of-bounds cells count as occupied).
     */
    bool occupied(int ix, int iy) const;

    /**
     * @brief World-space centers of all occupied cells.
     * @remark The 2D obstacle cloud consumed by the ground-corridor pipeline.
     * @return Matrix of occupied cell centers (rows = cells, cols = x,y).
     */
    MatrixXd occupied_centers() const;

    /**
     * @brief Nearest free-cell center to a query point.
     * @remark Used to snap q0/qT onto traversable space before corridor growth.
     * @param p Query point (world xy).
     * @return Center of the closest free cell (p itself if none are free).
     */
    Eigen::Vector2d nearest_free(const Eigen::Vector2d& p) const;
};

/**
 * @brief Builds a GridInfo spanning a cloud's xy bounds.
 * @param cloud Cloud to bound (rows = points, cols = x,y,z).
 * @param resolution Cell edge length.
 * @param margin Extra padding added around the xy bounds.
 * @return Grid geometry covering the padded bounds.
 */
GridInfo grid_from_cloud(const MatrixXd& cloud, double resolution, double margin);

}  // namespace trav
