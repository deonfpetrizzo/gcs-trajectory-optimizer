#pragma once
#include "gcs_pipeline.hpp"
#include <Eigen/Dense>
#include <array>
#include <vector>

namespace gcs {
// Enumerate vertices of {x : A x <= b}: intersect every d-subset of facets,
// keep feasible solutions.
Eigen::MatrixXd region_vertices(const ConvexRegion& region, double tol = 1e-6);

// Wireframe edges (pairs of vertex indices): two vertices form an edge if they
// share at least d-1 tight facets.
std::vector<std::array<int, 2>>
region_edges(const ConvexRegion& region, const Eigen::MatrixXd& verts,
             double tol = 1e-6);
}  // namespace gcs