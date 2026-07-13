/**
 * traversability_core.hpp
 *
 *   voxel-grid terrain features -> per-robot traversability -> 2D occupancy grid
 *   for 2.5D ground-corridor generation.
 *
 * Dependencies: Eigen (linear algebra).
 * Include this to pull in the whole library, or include the individual
 * component headers directly for finer-grained compilation dependencies.
 */
#pragma once

#include "traversability_core/trav_types.hpp"
#include "traversability_core/robot_model.hpp"
#include "traversability_core/voxel_grid.hpp"
#include "traversability_core/traversability_map.hpp"
#include "traversability_core/occupancy_grid.hpp"
