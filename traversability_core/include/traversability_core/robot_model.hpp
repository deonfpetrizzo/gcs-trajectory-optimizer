// robot_model.hpp
#pragma once

#include "traversability_core/trav_types.hpp"

namespace trav {

/**
 * @brief Robot capability model gating which cells are traversable.
 * @remark The same TraversabilityMap yields different occupancy per model
 * (e.g. a wheeled robot rejects a stair run a legged robot accepts).
 */
struct RobotModel {
    double max_slope_rad    = 0.35;
    double max_roughness    = 0.05;
    double max_step         = 0.15;
    double height           = 1.0;
    double min_clearance    = 0.5;
    double footprint_radius = 0.3;
    bool   allow_stairs     = false;
};

/**
 * @brief Continuous traversability score of a cell under a robot model.
 * @remark Combines slope, roughness, and step-height margins into [0, 1]
 * (0 = untraversable, 1 = ideal). Invalid cells score 0.
 * @param cell Terrain features for the cell.
 * @param robot Robot capability model.
 * @return Traversability score in [0, 1].
 */
double traversability_score(const TraversabilityCell& cell, const RobotModel& robot);

/**
 * @brief Hard traversability gate for a cell under a robot model.
 * @param cell Terrain features for the cell.
 * @param robot Robot capability model.
 * @return True if the robot can traverse the cell.
 */
bool passable(const TraversabilityCell& cell, const RobotModel& robot);

}  // namespace trav
