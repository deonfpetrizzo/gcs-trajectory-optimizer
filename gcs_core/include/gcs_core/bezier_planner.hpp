// bezier_planner.hpp
#pragma once

#include "gcs_core/gcs_types.hpp"
#include "gcs_core/convex_region.hpp"
#include "gcs_core/bezier.hpp"

#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace gcs {

/**
 * @brief Boundary conditions for a Bezier trajectory: derivative order -> value.
 */
struct BoundaryConditions {
    std::vector<std::pair<int, VectorXd>> conds;  // order -> value (0 = position, 1 = velocity, ...)
    static BoundaryConditions position(const VectorXd& p) { return {{{0, p}}}; }
    void set(int order, const VectorXd& v) { conds.push_back({order, v}); }
};

/**
 * @brief Geometry-only GCS Bezier planner (Marcucci, Ch. 11.2): builds a
 * region-adjacency graph, routes through it, and solves a convex-restriction QP
 * minimizing integral squared derivatives.
 */
class GCSBezierPlanner {
public:
    explicit GCSBezierPlanner(std::vector<ConvexRegion> regions);

    /**
     * @brief Builds the region-adjacency graph.
     * @remark Pairwise polytope intersection tested as an LP feasibility check.
     * @return Adjacency matrix (1 where two regions intersect).
     */
    const Eigen::MatrixXi& build_graph();

    /**
     * @brief Finds a sequence of intersecting regions connecting q0 to qT.
     * @remark Dijkstra shortest path over the region-adjacency graph, edge-weighted
     * by Chebyshev-center distance.
     * @param q0 Start point.
     * @param qT Goal point.
     * @return Ordered region indices from the region containing q0 to the one containing qT.
     */
    std::vector<int> find_region_sequence(const VectorXd& q0, const VectorXd& qT);

    struct Options {
        int    degree = 5;
        int    continuity = 2;
        double total_time = 1.0;
        std::vector<std::pair<int, double>> cost_derivatives = {{2, 1.0}}; 
        std::vector<std::tuple<int, double, double>> derivative_box;    
        std::optional<std::vector<double>> durations; 
    };

    /**
     * @brief Solves the convex-restriction QP for a fixed region sequence.
     * @remark Marcucci Ch. 11.2 convex restriction: minimizes weighted derivative
     * energy subject to per-region containment and C^k continuity at junctions.
     * @param seq Region sequence (e.g. from find_region_sequence()).
     * @param q0 Boundary conditions at the start.
     * @param qT Boundary conditions at the end.
     * @param opt Degree/continuity/cost/duration options.
     * @return The optimized piecewise Bezier trajectory.
     */
    BezierTrajectory solve_convex_restriction(const std::vector<int>& seq,
                                              const BoundaryConditions& q0,
                                              const BoundaryConditions& qT,
                                              const Options& opt);

    /**
     * @brief Routes from q0 to qT and solves the resulting convex restriction.
     * @remark Composes find_region_sequence() (Dijkstra) with solve_convex_restriction() (QP).
     * @param q0 Boundary conditions at the start.
     * @param qT Boundary conditions at the end.
     * @param opt Degree/continuity/cost/duration options.
     * @return The optimized piecewise Bezier trajectory.
     */
    BezierTrajectory plan(const BoundaryConditions& q0,
                          const BoundaryConditions& qT, const Options& opt);

    const std::vector<ConvexRegion>& regions() const { return regions_; }
    int dim() const { return dim_; }

protected:
    std::vector<ConvexRegion> regions_;
    int dim_ = 0;
    Eigen::MatrixXi adj_;
    std::vector<VectorXd> centers_;
    const std::vector<VectorXd>& centers();
};

}  // namespace gcs
