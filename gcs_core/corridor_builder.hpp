// corridor_builder.hpp
// ============================================================================
//   Parallel, sampling-based safe-corridor construction.
//
//   Instead of growing one convex region per densified waypoint sequentially,
//   this builder grows regions in parallel (one per thread per round), with each
//   thread sampling its query point uniformly by arc-length along the nominal
//   path. After each round the regions are intersected into a connectivity graph
//   (disjoint-set union); rounds repeat until a connected chain links a region
//   containing q0 to a region containing qT -- exactly the precondition the GCS
//   planner's find_region_sequence() needs.
//
//   ROS-agnostic: depends only on gcs_pipeline (Eigen / Qhull / OSQP) + <thread>.
// ============================================================================
#pragma once

#include "gcs_pipeline.hpp"

#include <cstdint>
#include <functional>
#include <vector>

namespace gcs {

struct CorridorOptions {
    double   R            = 5.0;    // region radius (sphere-flip radius)
    int      num_threads  = 10;    // regions grown per round
    int      max_rounds   = 50;    // safety cap to avoid infinite loops
    bool     tighten      = true;  // forward to convex_region_from_pointcloud
    bool     sphere_floor = true;  // use the sphere-floor variant (min-size guarantee)
    bool     drop_covered_samples = true; // skip a sample whose pq already lies in a pool region
    uint64_t seed         = 0;     // 0 -> seed from std::random_device (non-deterministic)

    // Optional hook applied to each freshly grown region, before the
    // containment/connectivity checks -- e.g. to latch a region to a ground
    // band. Since it runs before those checks, `connected` and the region
    // pool correctly reflect the post-processed (e.g. shrunk) region.
    std::function<void(const VectorXd& pq, ConvexRegion& reg)> region_postprocess;
};

struct CorridorResult {
    std::vector<ConvexRegion> regions;
    bool connected = false;   // did a q0..qT chain form?
    int  rounds    = 0;       // rounds actually executed
};

// Build a safe corridor connecting q0 to qT by parallel sampling along
// `nominal_path` (ordered waypoints). The returned regions feed directly into
// GCSBezierPlanner / GCSCompositeBezierPlanner.
CorridorResult build_corridor_parallel(const VectorXd& q0, const VectorXd& qT,
                                       const std::vector<VectorXd>& nominal_path,
                                       const MatrixXd& cloud,
                                       const CorridorOptions& opt = {});

}  // namespace gcs
