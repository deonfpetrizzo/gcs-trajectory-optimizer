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
 * @brief Intersects a 3D region with a thin band around the fitted ground plane:
 * { x : |z - (a*x + b*y + c)| <= half_width }, appended as two halfspaces.
 * @param reg Region to clamp.
 * @param plane Ground plane to clamp against.
 * @param half_width Band half-width.
 * @return The clamped region.
 */
ConvexRegion clamp_region_to_ground_band(const ConvexRegion& reg,
                                         const GroundPlaneFit& plane,
                                         double half_width);

}  // namespace gcs
