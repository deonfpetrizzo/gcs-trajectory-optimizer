// gcs_pipeline_planners.cpp  (graph + planners)
#include "gcs_pipeline.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <osqp.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace gcs {


// ===========================================================================
//  BezierTrajectory evaluation
// ===========================================================================
double BezierTrajectory::total_duration() const {
    double s = 0; for (double d : durations) s += d; return s;
}

VectorXd BezierTrajectory::eval(double t, int order) const {
    std::vector<double> bp(durations.size() + 1, 0.0);
    for (size_t i = 0; i < durations.size(); ++i) bp[i + 1] = bp[i] + durations[i];
    double T = bp.back();
    t = std::clamp(t, 0.0, T);
    int i = 0;
    while (i + 1 < (int)durations.size() && t > bp[i + 1]) ++i;
    double tau = (t - bp[i]) / durations[i];
    tau = std::clamp(tau, 0.0, 1.0);
    MatrixXd G = control_points[i];
    int N = degree;
    if (order > 0) { G = derivative_operator(N, durations[i], order) * G; N -= order; }
    return (bernstein_basis(N, tau).transpose() * G).transpose();
}

// ===========================================================================
//  GCSBezierPlanner: graph + routing
// ===========================================================================
GCSBezierPlanner::GCSBezierPlanner(std::vector<ConvexRegion> regions)
    : regions_(std::move(regions)) {
    if (regions_.empty()) throw std::invalid_argument("need at least one region");
    dim_ = regions_[0].dim();
    for (auto& r : regions_)
        if (r.dim() != dim_) throw std::invalid_argument("dimension mismatch");
}

const std::vector<VectorXd>& GCSBezierPlanner::centers() {
    if (centers_.empty())
        for (auto& r : regions_) centers_.push_back(r.chebyshev_center().first);
    return centers_;
}

const Eigen::MatrixXi& GCSBezierPlanner::build_graph() {
    int n = regions_.size();
    adj_ = Eigen::MatrixXi::Zero(n, n);
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            if (regions_[i].intersects(regions_[j])) { adj_(i, j) = adj_(j, i) = 1; }
    return adj_;
}

std::vector<int> GCSBezierPlanner::find_region_sequence(const VectorXd& q0, const VectorXd& qT) {
    if (adj_.size() == 0) build_graph();
    const auto& C = centers();
    int n = regions_.size();
    std::vector<int> src, tgt;
    for (int i = 0; i < n; ++i) if (regions_[i].contains(q0)) src.push_back(i);
    for (int i = 0; i < n; ++i) if (regions_[i].contains(qT)) tgt.push_back(i);
    if (src.empty()) throw std::runtime_error("start q0 not inside any region");
    if (tgt.empty()) throw std::runtime_error("goal qT not inside any region");

    const int S = n, T = n + 1, NV = n + 2;
    std::vector<std::vector<std::pair<int, double>>> g(NV);
    auto add = [&](int a, int b, double w) {
        g[a].push_back({b, w}); g[b].push_back({a, w});
    };
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            if (adj_(i, j)) g[i].push_back({j, (C[i] - C[j]).norm()});
    for (int i : src) add(S, i, (q0 - C[i]).norm() + 1e-6);
    for (int i : tgt) add(i, T, (qT - C[i]).norm() + 1e-6);

    std::vector<double> dist(NV, std::numeric_limits<double>::infinity());
    std::vector<int> pred(NV, -1);
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>,
                        std::greater<>> pq;
    dist[S] = 0; pq.push({0, S});
    while (!pq.empty()) {
        auto [du, u] = pq.top(); pq.pop();
        if (du > dist[u]) continue;
        for (auto [v, w] : g[u])
            if (dist[u] + w < dist[v]) { dist[v] = dist[u] + w; pred[v] = u; pq.push({dist[v], v}); }
    }
    if (!std::isfinite(dist[T])) throw std::runtime_error("no overlapping-region path");
    std::vector<int> path; for (int node = T; node != S; node = pred[node]) path.push_back(node);
    std::reverse(path.begin(), path.end());
    std::vector<int> out;
    for (int p : path) if (p < n) out.push_back(p);
    return out;
}

// ===========================================================================
//  Helper: place a small dense block into a global constraint triplet builder.
// ===========================================================================
namespace {
struct ConstraintBuilder {
    int nvar;
    std::vector<Eigen::Triplet<double>> trips;
    std::vector<double> lo, up;
    int row = 0;
    explicit ConstraintBuilder(int n) : nvar(n) {}
    // add rows:  lvec <= M (block, cols [col0..]) x_block <= uvec
    void add_block_eq(const MatrixXd& M, int col0, const VectorXd& rhs) {
        for (int i = 0; i < M.rows(); ++i) {
            for (int j = 0; j < M.cols(); ++j)
                if (M(i, j) != 0.0) trips.emplace_back(row + i, col0 + j, M(i, j));
            lo.push_back(rhs(i)); up.push_back(rhs(i));
        }
        row += M.rows();
    }
    void add_block_le(const MatrixXd& M, int col0, const VectorXd& ub) {
        for (int i = 0; i < M.rows(); ++i) {
            for (int j = 0; j < M.cols(); ++j)
                if (M(i, j) != 0.0) trips.emplace_back(row + i, col0 + j, M(i, j));
            lo.push_back(-OSQP_INFTY); up.push_back(ub(i));
        }
        row += M.rows();
    }
    // single scalar row: lo <= sum(coeffs at cols) <= up
    void add_row(const std::vector<std::pair<int, double>>& terms, double l, double u) {
        for (auto& [c, v] : terms) trips.emplace_back(row, c, v);
        lo.push_back(l); up.push_back(u); ++row;
    }
    MatrixXd dense() const {
        MatrixXd G = MatrixXd::Zero(row, nvar);
        for (auto& t : trips) G(t.row(), t.col()) += t.value();
        return G;
    }
    VectorXd lvec() const { return Eigen::Map<const VectorXd>(lo.data(), lo.size()); }
    VectorXd uvec() const { return Eigen::Map<const VectorXd>(up.data(), up.size()); }
};
}  // namespace

// ===========================================================================
//  PART 2.  Geometry-only convex restriction (pure QP)
//
//  Variable layout: per segment k, control points stacked row-major as
//    x[ base_k + n*d + j ]  for control point n (0..N), dimension j (0..d-1).
// ===========================================================================
BezierTrajectory GCSBezierPlanner::solve_convex_restriction(
    const std::vector<int>& seq, const BoundaryConditions& q0,
    const BoundaryConditions& qT, const Options& opt) {
    const int d = dim_, N = opt.degree, nseg = seq.size();
    if (N < opt.continuity + 1) throw std::invalid_argument("degree >= continuity+1");
    const int per = (N + 1) * d;              // vars per segment
    const int nvar = per * nseg;
    auto base = [&](int k) { return k * per; };
    auto cpcol = [&](int k, int n, int j) { return base(k) + n * d + j; };

    // durations
    std::vector<double> dur;
    if (opt.durations) dur = *opt.durations;
    else {
        const auto& C = centers();
        VectorXd prev = q0.conds.front().second;
        std::vector<double> segs;
        for (int i : seq) { segs.push_back((C[i] - prev).norm() + 1e-3); prev = C[i]; }
        double mean = 0; for (double s : segs) mean += s; mean /= segs.size();
        double sum = 0; for (double& s : segs) { s = std::max(s, 0.4 * mean); sum += s; }
        for (double s : segs) dur.push_back(s / sum * opt.total_time);
    }

    // ---- objective P (block diagonal over segments / dims) ----
    MatrixXd P = MatrixXd::Zero(nvar, nvar);
    for (int k = 0; k < nseg; ++k) {
        double T = dur[k];
        for (auto& [order, w] : opt.cost_derivatives) {
            MatrixXd D = derivative_operator(N, T, order);    // (N+1-order x N+1)
            MatrixXd M = energy_matrix(N - order);            // (N+1-order sq)
            MatrixXd H = D.transpose() * M * D;               // (N+1 x N+1), on cps
            H *= 2.0 * w * T;                                  // 1/2 x'Px convention
            for (int a = 0; a <= N; ++a)
                for (int bb = 0; bb <= N; ++bb)
                    for (int j = 0; j < d; ++j)
                        P(cpcol(k, a, j), cpcol(k, bb, j)) += H(a, bb);
        }
    }
    P += 1e-9 * MatrixXd::Identity(nvar, nvar);   // conditioning
    VectorXd cvec = VectorXd::Zero(nvar);

    // ---- constraints ----
    ConstraintBuilder cb(nvar);
    // region containment
    for (int k = 0; k < nseg; ++k) {
        const ConvexRegion& reg = regions_[seq[k]];
        for (int n = 0; n <= N; ++n) {
            // A * cp_n <= b
            for (int i = 0; i < reg.A.rows(); ++i) {
                std::vector<std::pair<int, double>> terms;
                for (int j = 0; j < d; ++j) terms.push_back({cpcol(k, n, j), reg.A(i, j)});
                cb.add_row(terms, -OSQP_INFTY, reg.b(i));
            }
        }
    }
    // optional derivative box
    for (auto& [order, lo, hi] : opt.derivative_box) {
        for (int k = 0; k < nseg; ++k) {
            MatrixXd D = derivative_operator(N, dur[k], order);   // (m x N+1)
            int m = D.rows();
            for (int r = 0; r < m; ++r)
                for (int j = 0; j < d; ++j) {
                    std::vector<std::pair<int, double>> terms;
                    for (int n = 0; n <= N; ++n)
                        if (D(r, n) != 0.0) terms.push_back({cpcol(k, n, j), D(r, n)});
                    cb.add_row(terms, lo, hi);
                }
        }
    }
    // continuity at junctions
    for (int k = 0; k + 1 < nseg; ++k) {
        for (int order = 0; order <= opt.continuity; ++order) {
            MatrixXd Dk = derivative_operator(N, dur[k], order);
            MatrixXd Dk1 = derivative_operator(N, dur[k + 1], order);
            VectorXd last = Dk.row(Dk.rows() - 1);   // coeffs on cps of seg k
            VectorXd first = Dk1.row(0);
            for (int j = 0; j < d; ++j) {
                std::vector<std::pair<int, double>> terms;
                for (int n = 0; n <= N; ++n) {
                    if (last(n) != 0.0) terms.push_back({cpcol(k, n, j), last(n)});
                    if (first(n) != 0.0) terms.push_back({cpcol(k + 1, n, j), -first(n)});
                }
                cb.add_row(terms, 0.0, 0.0);
            }
        }
    }
    // boundary conditions
    auto add_bc = [&](int k, bool at_start, const BoundaryConditions& bc) {
        double T = dur[k];
        for (auto& [order, val] : bc.conds) {
            MatrixXd D = derivative_operator(N, T, order);
            VectorXd coef = at_start ? D.row(0) : D.row(D.rows() - 1);
            for (int j = 0; j < d; ++j) {
                std::vector<std::pair<int, double>> terms;
                for (int n = 0; n <= N; ++n)
                    if (coef(n) != 0.0) terms.push_back({cpcol(k, n, j), coef(n)});
                cb.add_row(terms, val(j), val(j));
            }
        }
    };
    add_bc(0, true, q0);
    add_bc(nseg - 1, false, qT);

    QPResult r = solve_qp_extern(P, cvec, cb.dense(), cb.lvec(), cb.uvec());
    if (!r.ok) throw std::runtime_error("convex restriction infeasible: " + r.status);

    BezierTrajectory traj;
    traj.degree = N; traj.durations = dur; traj.region_sequence = seq;
    for (int k = 0; k < nseg; ++k) {
        MatrixXd G(N + 1, d);
        for (int n = 0; n <= N; ++n)
            for (int j = 0; j < d; ++j) G(n, j) = r.x(cpcol(k, n, j));
        traj.control_points.push_back(G);
    }
    // recompute objective value
    traj.cost = 0.5 * r.x.transpose() * P * r.x;
    return traj;
}

BezierTrajectory GCSBezierPlanner::plan(const BoundaryConditions& q0,
                                        const BoundaryConditions& qT,
                                        const Options& opt) {
    VectorXd p0 = q0.conds.front().second, pT = qT.conds.front().second;
    auto seq = find_region_sequence(p0, pT);
    return solve_convex_restriction(seq, q0, qT, opt);
}

// ===========================================================================
//  PART 2b.  Composite shape + timing
//
//  Variables per segment k: q control points ((N+1)*d) then h control points
//  (N+1). Velocity SOC ||q'_n|| <= v_max h'_n is approximated by a polyhedral
//  inner cone: for K unit directions u_m in R^d,  u_m . q'_n <= v_max h'_n.
// ===========================================================================
VectorXd CompositeTimingTrajectory::q_eval(int i, double s, int order) const {
    MatrixXd G = q_cps[i]; int N = degree;
    if (order) { G = derivative_operator(N, 1.0, order) * G; N -= order; }
    return (bernstein_basis(N, s).transpose() * G).transpose();
}
double CompositeTimingTrajectory::h_eval(int i, double s, int order) const {
    MatrixXd G = h_cps[i]; int N = degree;
    if (order) { G = derivative_operator(N, 1.0, order) * G; N -= order; }
    return (bernstein_basis(N, s).transpose() * G)(0, 0);
}
std::pair<int, double> CompositeTimingTrajectory::locate(double t) const {
    double T = total_duration();
    t = std::clamp(t, 0.0, T);
    for (size_t i = 0; i < q_cps.size(); ++i) {
        double t1 = h_cps[i](h_cps[i].rows() - 1, 0);
        if (t <= t1 + 1e-9 || i + 1 == q_cps.size()) {
            double lo = 0, hi = 1;
            for (int it = 0; it < 60; ++it) {
                double m = 0.5 * (lo + hi);
                if (h_eval(i, m, 0) < t) lo = m; else hi = m;
            }
            return {(int)i, 0.5 * (lo + hi)};
        }
    }
    return {(int)q_cps.size() - 1, 1.0};
}
VectorXd CompositeTimingTrajectory::position(double t) const {
    auto [i, s] = locate(t); return q_eval(i, s, 0);
}
VectorXd CompositeTimingTrajectory::velocity(double t) const {
    auto [i, s] = locate(t); return q_eval(i, s, 1) / h_eval(i, s, 1);
}
VectorXd CompositeTimingTrajectory::acceleration(double t) const {
    auto [i, s] = locate(t);
    VectorXd qd = q_eval(i, s, 1), qdd = q_eval(i, s, 2);
    double hd = h_eval(i, s, 1), hdd = h_eval(i, s, 2);
    return (qdd * hd - qd * hdd) / (hd * hd * hd);
}

CompositeTimingTrajectory GCSCompositeBezierPlanner::solve_composite(
    const std::vector<int>& seq, const VectorXd& q0_pos, const VectorXd& qT_pos,
    const Options& opt) {
    const int d = dim_, N = opt.degree, nseg = seq.size();
    if (N < opt.continuity + 1) throw std::invalid_argument("degree >= continuity+1");
    if (N < 2) throw std::invalid_argument("degree >= 2");

    // variable layout
    const int qper = (N + 1) * d, hper = (N + 1), per = qper + hper;
    const int nvar = per * nseg;
    auto qcol = [&](int k, int n, int j) { return k * per + n * d + j; };
    auto hcol = [&](int k, int n) { return k * per + qper + n; };

    MatrixXd D1 = derivative_operator(N, 1.0, 1);   // (N x N+1)
    MatrixXd D2 = derivative_operator(N, 1.0, 2);   // (N-1 x N+1)
    MatrixXd D3 = (N >= 3) ? derivative_operator(N, 1.0, 3) : MatrixXd();

    // ---- objective ----
    MatrixXd P = MatrixXd::Zero(nvar, nvar);
    VectorXd cvec = VectorXd::Zero(nvar);
    auto add_quad_on_q = [&](int k, const MatrixXd& D, double w) {
        MatrixXd M = energy_matrix(D.rows() - 1);
        MatrixXd H = 2.0 * w * (D.transpose() * M * D);   // (N+1 sq), L=1
        for (int a = 0; a <= N; ++a) for (int bb = 0; bb <= N; ++bb)
            for (int j = 0; j < d; ++j) P(qcol(k, a, j), qcol(k, bb, j)) += H(a, bb);
    };
    auto add_quad_on_h = [&](int k, const MatrixXd& D, double w) {
        MatrixXd M = energy_matrix(D.rows() - 1);
        MatrixXd H = 2.0 * w * (D.transpose() * M * D);
        for (int a = 0; a <= N; ++a) for (int bb = 0; bb <= N; ++bb)
            P(hcol(k, a), hcol(k, bb)) += H(a, bb);
    };
    for (int k = 0; k < nseg; ++k) {
        if (opt.w_geom_accel) add_quad_on_q(k, D2, opt.w_geom_accel);
        if (opt.w_time_accel) add_quad_on_h(k, D2, opt.w_time_accel);
        if (opt.w_geom_jerk && N >= 3) add_quad_on_q(k, D3, opt.w_geom_jerk);
    }
    P += 1e-9 * MatrixXd::Identity(nvar, nvar);
    // duration cost  w_time * T  (T = last h control point of last segment)
    cvec(hcol(nseg - 1, N)) += opt.w_time;

    // ---- constraints ----
    ConstraintBuilder cb(nvar);
    // polyhedral SOC directions (d-dimensional unit vectors)
    std::vector<VectorXd> dirs;
    if (opt.vel_limit) {
        int K = std::max(opt.soc_facets, 2 * d);
        if (d == 2) {
            for (int m = 0; m < K; ++m) {
                double a = 2 * M_PI * m / K;
                VectorXd u(2); u << std::cos(a), std::sin(a); dirs.push_back(u);
            }
        } else {  // d==3 (or higher): axes + a Fibonacci sphere
            for (int m = 0; m < K; ++m) {
                double y = 1 - 2.0 * (m + 0.5) / K;
                double r = std::sqrt(std::max(0.0, 1 - y * y));
                double phi = M_PI * (3 - std::sqrt(5.0)) * m;
                VectorXd u(3); u << r * std::cos(phi), y, r * std::sin(phi);
                dirs.push_back(u.normalized());
            }
        }
    }

    for (int k = 0; k < nseg; ++k) {
        const ConvexRegion& reg = regions_[seq[k]];
        // (1) shape containment
        for (int n = 0; n <= N; ++n)
            for (int i = 0; i < reg.A.rows(); ++i) {
                std::vector<std::pair<int, double>> terms;
                for (int j = 0; j < d; ++j) terms.push_back({qcol(k, n, j), reg.A(i, j)});
                cb.add_row(terms, -OSQP_INFTY, reg.b(i));
            }
        // (2) monotone timing h'_n >= h_dot_min
        for (int n = 0; n < N; ++n) {
            std::vector<std::pair<int, double>> terms;
            for (int p = 0; p <= N; ++p) if (D1(n, p) != 0.0) terms.push_back({hcol(k, p), D1(n, p)});
            cb.add_row(terms, opt.h_dot_min, OSQP_INFTY);
        }
        // (3) velocity limit: u_m . q'_n <= v_max h'_n  (polyhedral SOC)
        if (opt.vel_limit) {
            double vmax = *opt.vel_limit;
            for (int n = 0; n < N; ++n) {        // q'/h' have N control points
                for (auto& u : dirs) {
                    std::vector<std::pair<int, double>> terms;
                    // sum_j u_j * (D1 q)_n,j  -  vmax * (D1 h)_n  <= 0
                    for (int j = 0; j < d; ++j)
                        for (int p = 0; p <= N; ++p)
                            if (D1(n, p) != 0.0) terms.push_back({qcol(k, p, j), u(j) * D1(n, p)});
                    for (int p = 0; p <= N; ++p)
                        if (D1(n, p) != 0.0) terms.push_back({hcol(k, p), -vmax * D1(n, p)});
                    cb.add_row(terms, -OSQP_INFTY, 0.0);
                }
            }
        }
        // (4) conservative geometric accel cap (box on q'' control points)
        if (opt.accel_limit) {
            double am = *opt.accel_limit;
            for (int n = 0; n < D2.rows(); ++n)
                for (int j = 0; j < d; ++j) {
                    std::vector<std::pair<int, double>> terms;
                    for (int p = 0; p <= N; ++p) if (D2(n, p) != 0.0) terms.push_back({qcol(k, p, j), D2(n, p)});
                    cb.add_row(terms, -am, am);
                }
        }
    }
    // (6) continuity (q and h independently)
    for (int k = 0; k + 1 < nseg; ++k)
        for (int order = 0; order <= opt.continuity; ++order) {
            MatrixXd D = derivative_operator(N, 1.0, order);
            VectorXd last = D.row(D.rows() - 1), first = D.row(0);
            for (int j = 0; j < d; ++j) {     // q
                std::vector<std::pair<int, double>> t;
                for (int n = 0; n <= N; ++n) {
                    if (last(n) != 0) t.push_back({qcol(k, n, j), last(n)});
                    if (first(n) != 0) t.push_back({qcol(k + 1, n, j), -first(n)});
                }
                cb.add_row(t, 0, 0);
            }
            {                                  // h
                std::vector<std::pair<int, double>> t;
                for (int n = 0; n <= N; ++n) {
                    if (last(n) != 0) t.push_back({hcol(k, n), last(n)});
                    if (first(n) != 0) t.push_back({hcol(k + 1, n), -first(n)});
                }
                cb.add_row(t, 0, 0);
            }
        }
    // (7) boundary conditions
    for (int j = 0; j < d; ++j) { cb.add_row({{qcol(0, 0, j), 1.0}}, q0_pos(j), q0_pos(j)); }
    for (int j = 0; j < d; ++j) { cb.add_row({{qcol(nseg - 1, N, j), 1.0}}, qT_pos(j), qT_pos(j)); }
    cb.add_row({{hcol(0, 0), 1.0}}, 0.0, 0.0);                 // time starts at 0
    cb.add_row({{hcol(nseg - 1, N), 1.0}}, opt.T_min, opt.T_max);
    // velocity BC: q'(end) = v * h'(end)   (linear; v constant)
    auto vel_bc = [&](int k, bool start, const VectorXd& v) {
        VectorXd coef = start ? D1.row(0) : D1.row(D1.rows() - 1);
        for (int j = 0; j < d; ++j) {
            std::vector<std::pair<int, double>> t;
            for (int n = 0; n <= N; ++n) if (coef(n) != 0) t.push_back({qcol(k, n, j), coef(n)});
            for (int n = 0; n <= N; ++n) if (coef(n) != 0) t.push_back({hcol(k, n), -v(j) * coef(n)});
            cb.add_row(t, 0, 0);
        }
    };
    if (opt.v0) vel_bc(0, true, *opt.v0);
    if (opt.vT) vel_bc(nseg - 1, false, *opt.vT);
    // zero accel at rest endpoints: q''(end) = 0
    if (opt.rest_to_rest_accel) {
        VectorXd c0 = D2.row(0), c1 = D2.row(D2.rows() - 1);
        for (int j = 0; j < d; ++j) {
            std::vector<std::pair<int, double>> t0, t1;
            for (int n = 0; n <= N; ++n) {
                if (c0(n) != 0) t0.push_back({qcol(0, n, j), c0(n)});
                if (c1(n) != 0) t1.push_back({qcol(nseg - 1, n, j), c1(n)});
            }
            cb.add_row(t0, 0, 0); cb.add_row(t1, 0, 0);
        }
    }

    QPResult r = solve_qp_extern(P, cvec, cb.dense(), cb.lvec(), cb.uvec());
    if (!r.ok) throw std::runtime_error("composite program infeasible: " + r.status);

    CompositeTimingTrajectory traj;
    traj.degree = N; traj.region_sequence = seq;
    for (int k = 0; k < nseg; ++k) {
        MatrixXd Gq(N + 1, d), Gh(N + 1, 1);
        for (int n = 0; n <= N; ++n) {
            for (int j = 0; j < d; ++j) Gq(n, j) = r.x(qcol(k, n, j));
            Gh(n, 0) = r.x(hcol(k, n));
        }
        traj.q_cps.push_back(Gq); traj.h_cps.push_back(Gh);
    }
    traj.cost = 0.5 * r.x.transpose() * P * r.x + cvec.dot(r.x);
    return traj;
}

CompositeTimingTrajectory GCSCompositeBezierPlanner::plan_composite(
    const VectorXd& q0_pos, const VectorXd& qT_pos, const Options& opt) {
    auto seq = find_region_sequence(q0_pos, qT_pos);
    return solve_composite(seq, q0_pos, qT_pos, opt);
}

}  // namespace gcs
