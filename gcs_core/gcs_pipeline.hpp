// gcs_pipeline.hpp
// ============================================================================
//   PART 1   sphere-flipping star-convex free-space regions (Zhong et al.)
//   PART 2   GCS Bezier trajectory optimization (Marcucci, Ch. 11.2 / 11.4)
//   PART 2b  joint shape+timing  r(t) = q(h^{-1}(t))           (Ch. 11.4.1)
//
// Dependencies:
//   * Eigen          (header-only linear algebra)
//   * Qhull (C++)    (convex hull + halfspace intersection)
//   * OSQP           (convex QP solver)
//
// Solver note: OSQP solves quadratic programs (quadratic objective, linear
// constraints). The geometry-only planner is a pure QP. The composite
// planner's velocity limit ||q'|| <= v_max h' is a second-order cone; OSQP
// has no native SOC, so we use a polyhedral INNER approximation of the cone
// (a set of linear facets). This is a standard, conservative technique: the
// resulting velocity bound is slightly tighter than the exact cone, never
// looser. Increase the facet count for a closer approximation.
// ============================================================================
#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <optional>
#include <stdexcept>

namespace gcs {

using Eigen::MatrixXd;
using Eigen::VectorXd;


// Result of the internal convex QP solve
struct QPResult {
    VectorXd x;
    bool ok = false;
    std::string status;
};
QPResult solve_qp_extern(const MatrixXd& P, const VectorXd& c,
                         const MatrixXd& G, const VectorXd& l, const VectorXd& u);


//  Convex region:  { x : A x <= b }
struct ConvexRegion {
    MatrixXd A;
    VectorXd b;         
    std::string name;

    ConvexRegion() = default;
    ConvexRegion(MatrixXd A_, VectorXd b_, std::string name_ = "")
        : A(std::move(A_)), b(std::move(b_)), name(std::move(name_)) {
        if (A.rows() != b.size())
            throw std::invalid_argument("A and b have incompatible shapes");
    }
    int dim() const { return static_cast<int>(A.cols()); }
    bool contains(const VectorXd& x, double tol = 1e-7) const {
        return ((A * x).array() <= b.array() + tol).all();
    }
    // Largest inscribed ball (Chebyshev center) via a small LP solved as a QP.
    std::pair<VectorXd, double> chebyshev_center() const;
    // Feasibility test: do the two polytopes share a point? (LP via QP)
    bool intersects(const ConvexRegion& other, double tol = 1e-7) const;
};



// PART 1.  Sphere-flipping
// Sphere flip p -> p * (2R - ||p||)/||p|| about `center`. Returns flipped
// points; `valid` marks rows with nonzero radius.
MatrixXd sphere_flip(const MatrixXd& points, const VectorXd& center, double R);

// Inverse sphere flip (an involution: same functional form as the forward map).
MatrixXd inv_sphere_flip(const MatrixXd& flipped, const VectorXd& center, double R);

// Star-convex polytope vertices (in original space) for query point 'pq'.
// If sphere_floor is true, sphere-surface samples (2d cardinal + 2^d diagonal
// directions) are added before Qhull so the result is always at least as large
// as the sphere-inscribed polytope, even when obstacles are absent or sparse.
MatrixXd generate_star_convex(const VectorXd& pq, const MatrixXd& cloud, double R,
                               bool sphere_floor = false);

// Trim star-convex vertices to a strict convex H-polytope.
void star_to_convex(const MatrixXd& star_verts, const VectorXd& pq,
                    MatrixXd& A_out, VectorXd& b_out);

// One-shot: query point + cloud -> obstacle-free convex region. With `tighten`
// a safety pass guarantees no in-R cloud point lies inside the region.
// Set sphere_floor=true to enable the minimum-size guarantee (see generate_star_convex).
ConvexRegion convex_region_from_pointcloud(const VectorXd& pq,
                                           const MatrixXd& cloud, double R,
                                           bool tighten = true,
                                           const std::string& name = "",
                                           bool sphere_floor = false);




// Map from control points to the control points of the k-th derivative,
// for a degree-N Bezier over an interval of the given duration. Shape
// ((N+1-k) x (N+1)).
MatrixXd derivative_operator(int N, double duration, int k);

// Symmetric PSD energy matrix M (N+1 x N+1):
// integral_L ||g(s)||^2 ds = L * sum_{n,m} M(n,m) g_n . g_m
MatrixXd energy_matrix(int N);

// Bernstein basis of degree N evaluated at tau in [0,1].
VectorXd bernstein_basis(int N, double tau);


// PART 2.  Geometry-only Bezier curve
struct BezierTrajectory {
    std::vector<MatrixXd> control_points;   // each (N+1 x d)
    std::vector<double>   durations;        // per segment
    std::vector<int>      region_sequence;
    int    degree = 0;
    double cost = 0.0;

    double total_duration() const;
    VectorXd eval(double t, int order = 0) const;   // position / derivatives
};

struct BoundaryConditions {
    // order -> value. order 0 is position; 1 velocity; etc.
    std::vector<std::pair<int, VectorXd>> conds;
    static BoundaryConditions position(const VectorXd& p) { return {{{0, p}}}; }
    void set(int order, const VectorXd& v) { conds.push_back({order, v}); }
};

class GCSBezierPlanner {
public:
    explicit GCSBezierPlanner(std::vector<ConvexRegion> regions);

    const Eigen::MatrixXi& build_graph();           // adjacency (intersection)
    std::vector<int> find_region_sequence(const VectorXd& q0, const VectorXd& qT);

    struct Options {
        int    degree = 5;
        int    continuity = 2;
        double total_time = 1.0;
        // cost: order -> weight (minimize w * integral ||q^(order)||^2)
        std::vector<std::pair<int, double>> cost_derivatives = {{2, 1.0}};
        // optional box on a derivative's control points: order -> (lo, hi)
        std::vector<std::tuple<int, double, double>> derivative_box;
        std::optional<std::vector<double>> durations;  // else from centers
    };

    BezierTrajectory solve_convex_restriction(const std::vector<int>& seq,
                                              const BoundaryConditions& q0,
                                              const BoundaryConditions& qT,
                                              const Options& opt);

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




//  PART 2b.  Joint shape + timing:  r(t) = q_i( h_i^{-1}(t) )   (Sec. 11.4.1)
struct CompositeTimingTrajectory {
    std::vector<MatrixXd> q_cps;  
    std::vector<MatrixXd> h_cps; 
    std::vector<int>      region_sequence;
    int    degree = 0;
    double cost = 0.0;

    double total_duration() const { return h_cps.back()(h_cps.back().rows() - 1, 0); }

    VectorXd position(double t) const;
    VectorXd velocity(double t) const;
    VectorXd acceleration(double t) const;

private:
    VectorXd q_eval(int i, double s, int order) const;
    double   h_eval(int i, double s, int order) const;
    std::pair<int, double> locate(double t) const;   // (segment, s)
};

class GCSCompositeBezierPlanner : public GCSBezierPlanner {
public:
    using GCSBezierPlanner::GCSBezierPlanner;

    struct Options {
        int    degree = 6;
        int    continuity = 2;
        std::optional<double> vel_limit;     // ||r'(t)|| <= vel_limit (m/s)
        std::optional<double> accel_limit;   // conservative cap on ||q''_n||
        std::optional<VectorXd> v0, vT;      // boundary velocities
        bool   rest_to_rest_accel = false;   // zero accel at rest endpoints
        double T_min = 0.0, T_max = 1e9;
        double h_dot_min = 1e-2;
        double w_time = 1.0;
        double w_geom_accel = 1e-2;
        double w_time_accel = 1e-2;
        double w_geom_jerk = 0.0;
        int    soc_facets = 16;              // polyhedral SOC approximation
    };

    CompositeTimingTrajectory solve_composite(const std::vector<int>& seq,
                                              const VectorXd& q0_pos,
                                              const VectorXd& qT_pos,
                                              const Options& opt);

    CompositeTimingTrajectory plan_composite(const VectorXd& q0_pos,
                                             const VectorXd& qT_pos,
                                             const Options& opt);
};

}  // namespace gcs
