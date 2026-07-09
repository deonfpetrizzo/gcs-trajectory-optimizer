// corridor_builder.hpp
#pragma once

#include "gcs_core/free_space_regions.hpp"

#include <cstdint>
#include <functional>
#include <vector>

namespace gcs {

struct CorridorOptions {
    double   R            = 5.0;    
    int      num_threads  = 10;   
    int      max_rounds   = 50;   
    bool     tighten      = true; 
    bool     sphere_floor = true;
    bool     drop_covered_samples = true;
    uint64_t seed         = 0;    

    std::function<void(const VectorXd& pq, ConvexRegion& reg)> region_postprocess;
};

struct CorridorResult {
    std::vector<ConvexRegion> regions;
    bool connected = false;  
    int  rounds    = 0;      
};

/**
 * @brief Builds a safe corridor connecting q0 to qT by parallel sampling along
 * `nominal_path`.
 * @remark Grows regions in parallel (one per thread per round), each thread sampling
 * its query point uniformly by arc-length along the nominal path; after each round the
 * regions are merged into a connectivity graph via disjoint-set union (DSU). Rounds
 * repeat until a connected chain links a region containing q0 to one containing qT --
 * the precondition GCSBezierPlanner::find_region_sequence() needs.
 * @param q0 Start point.
 * @param qT Goal point.
 * @param nominal_path Ordered waypoints to sample query points along.
 * @param cloud Obstacle point cloud (rows = points).
 * @param opt Corridor growth options.
 * @return The grown regions plus whether a q0..qT chain connected.
 */
CorridorResult build_corridor_parallel(const VectorXd& q0, const VectorXd& qT,
                                       const std::vector<VectorXd>& nominal_path,
                                       const MatrixXd& cloud,
                                       const CorridorOptions& opt = {});

}  // namespace gcs
