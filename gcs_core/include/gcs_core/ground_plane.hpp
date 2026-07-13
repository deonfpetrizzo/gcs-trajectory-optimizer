// ground_plane.hpp
#pragma once

#include "gcs_core/gcs_types.hpp"
#include "gcs_core/convex_region.hpp"

namespace gcs {

/**
 * @brief Locally-fit ground plane z ~= a*x + b*y + c, for latching ground-vehicle
 * regions to terrain (which may be sloped).
 */
struct GroundPlaneFit {
    double a = 0.0, b = 0.0, c = 0.0;
    bool   tilted = false;
};

/**
 * @brief Fits a GroundPlaneFit to the point cloud near `center`.
 * @remark Least-squares fit via SVD. Falls back to a flat plane through center's z
 * (tilted = false) when there are too few candidate points or the fit is
 * ill-conditioned (e.g. a narrow, nearly-collinear neighborhood).
 * @param cloud Point cloud (rows = points).
 * @param center Fit center.
 * @param xy_radius Horizontal search radius around center.
 * @param z_window Vertical search window around center's z.
 * @return The fitted (or fallback) plane.
 */
GroundPlaneFit fit_local_ground_plane(const MatrixXd& cloud, const VectorXd& center,
                                      double xy_radius, double z_window);

/**
 * @brief Lifts a 2D polygon onto a sloped ground plane and extrudes it to a 3D prism.
 * @remark Vertical walls from the polygon's halfspaces (z-free), a floor at
 * plane + floor_offset, and a ceiling at plane + ceil_height -- a 2.5D sloped prism
 * for ground-corridor generation.
 * @param poly2d 2D convex polygon { x : A x <= b } (must be 2-dimensional).
 * @param plane Ground plane to project the footprint onto.
 * @param floor_offset Floor height above the plane.
 * @param ceil_height Ceiling height above the plane.
 * @return The 3D prism region.
 */
ConvexRegion lift_polygon_to_prism(const ConvexRegion& poly2d, const GroundPlaneFit& plane,
                                   double floor_offset, double ceil_height);

}  // namespace gcs
