#pragma once
#include "gcs_core/convex_region.hpp"
#include <Eigen/Dense>
#include <array>
#include <vector>

namespace gcs {

/**
 * @brief Enumerates the vertices of a convex region { x : A x <= b }.
 * @remark Intersects every d-subset of facets, keeping the feasible solutions
 * (combinatorial vertex enumeration).
 * @param region The region to enumerate.
 * @param tol Numerical tolerance for feasibility.
 * @return One vertex per row.
 */
Eigen::MatrixXd region_vertices(const ConvexRegion& region, double tol = 1e-6);

/**
 * @brief Enumerates wireframe edges of a convex region for visualization.
 * @remark Two vertices form an edge if they share at least d-1 tight facets.
 * @param region The region the vertices belong to.
 * @param verts Vertices, e.g. from region_vertices().
 * @param tol Numerical tolerance for tight-facet membership.
 * @return Pairs of vertex indices, one pair per edge.
 */
std::vector<std::array<int, 2>>
region_edges(const ConvexRegion& region, const Eigen::MatrixXd& verts,
             double tol = 1e-6);

}  // namespace gcs
