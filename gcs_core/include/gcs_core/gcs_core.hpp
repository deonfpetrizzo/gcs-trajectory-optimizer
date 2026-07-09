/**
 * gcs_core.hpp
 *
 *   sphere-flipping star-convex free-space regions (Zhong et al.)
 *   GCS Bezier trajectory optimization (Marcucci, Ch. 11.2 / 11.4)
 *   joint shape+timing  r(t) = q(h^{-1}(t))           (Ch. 11.4.1)
 * 
 * Dependencies: Eigen (linear algebra), Qhull (convex hull), OSQP (convex QP).
 * Include this to pull in the whole library, or include the individual
 * component headers directly for finer-grained compilation dependencies.
 */
#pragma once

#include "gcs_core/qp_solver.hpp"
#include "gcs_core/convex_region.hpp"
#include "gcs_core/free_space_regions.hpp"
#include "gcs_core/ground_plane.hpp"
#include "gcs_core/bezier.hpp"
#include "gcs_core/bezier_planner.hpp"
#include "gcs_core/composite_planner.hpp"
