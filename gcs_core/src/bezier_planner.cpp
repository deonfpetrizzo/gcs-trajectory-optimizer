// bezier_planner.cpp
#include "gcs_core/bezier_planner.hpp"

#include "gcs_core/qp_solver.hpp"
#include "gcs_core/constraint_builder.hpp"

#include <osqp.h> 

#include <algorithm>
#include <limits>
#include <queue>

namespace gcs {

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

BezierTrajectory GCSBezierPlanner::solve_convex_restriction(
    const std::vector<int>& seq, const BoundaryConditions& q0,
    const BoundaryConditions& qT, const Options& opt) {
    const int d = dim_, N = opt.degree, nseg = seq.size();
    if (N < opt.continuity + 1) throw std::invalid_argument("degree >= continuity+1");
    const int per = (N + 1) * d;              
    const int nvar = per * nseg;
    auto base = [&](int k) { return k * per; };
    auto cpcol = [&](int k, int n, int j) { return base(k) + n * d + j; };

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

    MatrixXd P = MatrixXd::Zero(nvar, nvar);
    for (int k = 0; k < nseg; ++k) {
        double T = dur[k];
        for (auto& [order, w] : opt.cost_derivatives) {
            MatrixXd D = derivative_operator(N, T, order);
            MatrixXd M = energy_matrix(N - order);
            MatrixXd H = D.transpose() * M * D;
            H *= 2.0 * w * T;
            for (int a = 0; a <= N; ++a)
                for (int bb = 0; bb <= N; ++bb)
                    for (int j = 0; j < d; ++j)
                        P(cpcol(k, a, j), cpcol(k, bb, j)) += H(a, bb);
        }
    }
    P += 1e-9 * MatrixXd::Identity(nvar, nvar);
    VectorXd cvec = VectorXd::Zero(nvar);

    detail::ConstraintBuilder cb(nvar);
    for (int k = 0; k < nseg; ++k) {
        const ConvexRegion& reg = regions_[seq[k]];
        for (int n = 0; n <= N; ++n) {
            for (int i = 0; i < reg.A.rows(); ++i) {
                std::vector<std::pair<int, double>> terms;
                for (int j = 0; j < d; ++j) terms.push_back({cpcol(k, n, j), reg.A(i, j)});
                cb.add_row(terms, -OSQP_INFTY, reg.b(i));
            }
        }
    }
    for (auto& [order, lo, hi] : opt.derivative_box) {
        for (int k = 0; k < nseg; ++k) {
            MatrixXd D = derivative_operator(N, dur[k], order);
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
    for (int k = 0; k + 1 < nseg; ++k) {
        for (int order = 0; order <= opt.continuity; ++order) {
            MatrixXd Dk = derivative_operator(N, dur[k], order);
            MatrixXd Dk1 = derivative_operator(N, dur[k + 1], order);
            VectorXd last = Dk.row(Dk.rows() - 1);
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

    QPResult r = solve_qp(P, cvec, cb.dense(), cb.lvec(), cb.uvec());
    if (!r.ok) throw std::runtime_error("convex restriction infeasible: " + r.status);

    BezierTrajectory traj;
    traj.degree = N; traj.durations = dur; traj.region_sequence = seq;
    for (int k = 0; k < nseg; ++k) {
        MatrixXd G(N + 1, d);
        for (int n = 0; n <= N; ++n)
            for (int j = 0; j < d; ++j) G(n, j) = r.x(cpcol(k, n, j));
        traj.control_points.push_back(G);
    }
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

}  // namespace gcs
