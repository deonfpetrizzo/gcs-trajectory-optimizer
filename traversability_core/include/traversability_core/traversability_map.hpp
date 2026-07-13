// traversability_map.hpp
#pragma once

#include "traversability_core/trav_types.hpp"

namespace trav {

/**
 * @brief Parameters for terrain-feature extraction.
 */
struct TraversabilityParams {
    double resolution         = 0.1;
    double slope_fit_radius   = 0.4;
    double ground_z_window    = 0.3;
    double clearance_ceiling  = 2.0;
    int    min_points_per_cell = 4;
};

/**
 * @brief Builds a TraversabilityMap from a point cloud.
 * @remark Per cell: local least-squares ground plane (slope, roughness, ground_z),
 * step height from neighbour ground_z differences, and min overhead clearance from
 * a column scan up to clearance_ceiling above ground. Cells below
 * min_points_per_cell are left invalid.
 * @param cloud Terrain/obstacle cloud (rows = points, cols = x,y,z).
 * @param info Target grid geometry.
 * @param params Feature-extraction parameters.
 * @return The populated traversability map.
 */
TraversabilityMap compute_traversability(const MatrixXd& cloud, const GridInfo& info,
                                         const TraversabilityParams& params);

/**
 * @brief Smooths the ground-height / plane field across cells in place.
 * @remark Produces a C^1-ish terrain field so slope has a well-defined gradient,
 * enabling gradient-based "prefer low-slope terrain" objectives downstream.
 * @param map Map whose ground_z / plane coefficients are smoothed.
 * @param sigma_cells Gaussian smoothing radius in cells.
 */
void smooth_ground_field(TraversabilityMap& map, double sigma_cells);

}  // namespace trav
