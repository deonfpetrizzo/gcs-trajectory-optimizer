// ground_corridor.hpp
#pragma once

#include "gcs_core/corridor_builder.hpp"
#include "gcs_core/ground_plane.hpp"

#include <functional>

namespace gcs {

/**
 * @brief Callback yielding the local ground plane at a query (x, y).
 * @remark Lets an integrator (e.g. gcs_planner) latch prisms to an already-built,
 * globally-consistent terrain field (a traversability map's ground_z) instead of
 * re-deriving z per region from the raw cloud. Return true and fill `out` when a
 * plane is available; return false to fall back to the raw-cloud fit.
 */
using GroundPlaneSampler = std::function<bool(double x, double y, GroundPlaneFit& out)>;

/**
 * @brief Options for 2.5D ground-corridor generation.
 */
struct GroundCorridorOptions {
    CorridorOptions corridor2d;              // 2D free-space growth in the ground plane
    double robot_height        = 1.0;        // prism extrusion height above the plane
    double floor_offset        = 0.0;        // prism floor height above the plane
    double plane_fit_radius    = 1.0;        // horizontal radius for local plane fit
    double plane_fit_z_window  = 0.5;        // vertical window for local plane fit
};

/**
 * @brief Builds a 2.5D ground corridor: overlapping sloped prisms latched to terrain.
 * @remark Grows overlapping 2D free-space polygons from `occupied_xy` (occupied-cell
 * centers of a traversability occupancy grid) via build_corridor_parallel in the
 * ground plane, then lifts each polygon onto a locally-fit ground plane
 * (fit_local_ground_plane on `cloud3d`) and extrudes it to robot_height
 * (lift_polygon_to_prism). q0/qT are expected already snapped to free space.
 * @param q0 Start point (3D).
 * @param qT Goal point (3D).
 * @param nominal_path Ordered 3D waypoints to sample query points along.
 * @param occupied_xy 2D obstacle cloud (occupied cell centers; rows = cells, cols = x,y).
 * @param cloud3d Full 3D cloud, used to fit the ground plane under any polygon the
 * sampler cannot supply (or for all polygons when no sampler is given).
 * @param opt Ground-corridor options.
 * @param ground_plane Optional per-(x,y) ground-plane sampler (see GroundPlaneSampler);
 * when it returns false for a polygon's center, the raw-cloud fit is used instead.
 * @return The grown 3D prism regions plus whether a q0..qT chain connected.
 */
CorridorResult build_ground_corridor(const VectorXd& q0, const VectorXd& qT,
                                     const std::vector<VectorXd>& nominal_path,
                                     const MatrixXd& occupied_xy,
                                     const MatrixXd& cloud3d,
                                     const GroundCorridorOptions& opt = {},
                                     const GroundPlaneSampler& ground_plane = {});

}  // namespace gcs
