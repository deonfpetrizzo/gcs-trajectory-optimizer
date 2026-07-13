// occupancy_grid.hpp
#pragma once

#include "traversability_core/trav_types.hpp"
#include "traversability_core/robot_model.hpp"

namespace trav {

/**
 * @brief Parameters for occupancy-grid generation.
 */
struct OccupancyParams {
    double score_threshold = 0.0;  // optional soft-score floor; 0 = hard gate only
};

/**
 * @brief Thresholds a traversability map into a binary occupancy grid.
 * @remark A cell is occupied when it fails the robot's hard passability gate
 * (passable()) or its overhead clearance is below robot height (folding the 2.5D
 * clearance check into the 2D grid, so extrusion is honest by construction). When
 * score_threshold > 0 the soft traversability score is applied as an extra floor.
 * @param map Terrain features.
 * @param robot Robot capability model.
 * @param params Occupancy thresholds / options.
 * @return The binary occupancy grid.
 */
OccupancyGrid2D make_occupancy_grid(const TraversabilityMap& map, const RobotModel& robot,
                                    const OccupancyParams& params);

}  // namespace trav
