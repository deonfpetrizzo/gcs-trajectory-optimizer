// composite_planner.cpp
#include "gcs_core/composite_planner.hpp"

#include "gcs_core/bezier.hpp"
#include "gcs_core/qp_solver.hpp"
#include "gcs_core/constraint_builder.hpp"

#include <osqp.h> 

#include <algorithm>
#include <cmath>

namespace gcs {

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

    const int qper = (N + 1) * d, hper = (N + 1), per = qper + hper;
    const int nvar = per * nseg;
    auto qcol = [&](int k, int n, int j) { return k * per + n * d + j; };
    auto hcol = [&](int k, int n) { return k * per + qper + n; };

    MatrixXd D1 = derivative_operator(N, 1.0, 1);   
    MatrixXd D2 = derivative_operator(N, 1.0, 2);   
    MatrixXd D3 = (N >= 3) ? derivative_operator(N, 1.0, 3) : MatrixXd();

    MatrixXd P = MatrixXd::Zero(nvar, nvar);
    VectorXd cvec = VectorXd::Zero(nvar);
    auto add_quad_on_q = [&](int k, const MatrixXd& D, double w) {
        MatrixXd M = energy_matrix(D.rows() - 1);
        MatrixXd H = 2.0 * w * (D.transpose() * M * D);  
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
        if (opt.w_geom_length) add_quad_on_q(k, D1, opt.w_geom_length);
        if (opt.w_geom_accel) add_quad_on_q(k, D2, opt.w_geom_accel);
        if (opt.w_time_accel) add_quad_on_h(k, D2, opt.w_time_accel);
        if (opt.w_geom_jerk && N >= 3) add_quad_on_q(k, D3, opt.w_geom_jerk);
    }
    P += 1e-9 * MatrixXd::Identity(nvar, nvar);
    cvec(hcol(nseg - 1, N)) += opt.w_time;

    detail::ConstraintBuilder cb(nvar);
    std::vector<VectorXd> dirs;
    if (opt.vel_limit) {
        int K = std::max(opt.soc_facets, 2 * d);
        if (d == 2) {
            for (int m = 0; m < K; ++m) {
                double a = 2 * M_PI * m / K;
                VectorXd u(2); u << std::cos(a), std::sin(a); dirs.push_back(u);
            }
        } else {
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
        for (int n = 0; n <= N; ++n)
            for (int i = 0; i < reg.A.rows(); ++i) {
                std::vector<std::pair<int, double>> terms;
                for (int j = 0; j < d; ++j) terms.push_back({qcol(k, n, j), reg.A(i, j)});
                cb.add_row(terms, -OSQP_INFTY, reg.b(i));
            }
        for (int n = 0; n < N; ++n) {
            std::vector<std::pair<int, double>> terms;
            for (int p = 0; p <= N; ++p) if (D1(n, p) != 0.0) terms.push_back({hcol(k, p), D1(n, p)});
            cb.add_row(terms, opt.h_dot_min, OSQP_INFTY);
        }
        if (opt.vel_limit) {
            double vmax = *opt.vel_limit;
            for (int n = 0; n < N; ++n) {      
                for (auto& u : dirs) {
                    std::vector<std::pair<int, double>> terms;
                    for (int j = 0; j < d; ++j)
                        for (int p = 0; p <= N; ++p)
                            if (D1(n, p) != 0.0) terms.push_back({qcol(k, p, j), u(j) * D1(n, p)});
                    for (int p = 0; p <= N; ++p)
                        if (D1(n, p) != 0.0) terms.push_back({hcol(k, p), -vmax * D1(n, p)});
                    cb.add_row(terms, -OSQP_INFTY, 0.0);
                }
            }
        }
        if (opt.geom_accel_limit) {
            double am = *opt.geom_accel_limit;
            for (int n = 0; n < D2.rows(); ++n)
                for (int j = 0; j < d; ++j) {
                    std::vector<std::pair<int, double>> terms;
                    for (int p = 0; p <= N; ++p) if (D2(n, p) != 0.0) terms.push_back({qcol(k, p, j), D2(n, p)});
                    cb.add_row(terms, -am, am);
                }
        }
        if (opt.time_accel_limit) {
            double amh = *opt.time_accel_limit;
            for (int n = 0; n < D2.rows(); ++n) {
                std::vector<std::pair<int, double>> terms;
                for (int p = 0; p <= N; ++p) if (D2(n, p) != 0.0) terms.push_back({hcol(k, p), D2(n, p)});
                cb.add_row(terms, -amh, amh);
            }
        }
    }
    for (int k = 0; k + 1 < nseg; ++k)
        for (int order = 0; order <= opt.continuity; ++order) {
            MatrixXd D = derivative_operator(N, 1.0, order);
            VectorXd last = D.row(D.rows() - 1), first = D.row(0);
            for (int j = 0; j < d; ++j) {     
                std::vector<std::pair<int, double>> t;
                for (int n = 0; n <= N; ++n) {
                    if (last(n) != 0) t.push_back({qcol(k, n, j), last(n)});
                    if (first(n) != 0) t.push_back({qcol(k + 1, n, j), -first(n)});
                }
                cb.add_row(t, 0, 0);
            }
            {                                  
                std::vector<std::pair<int, double>> t;
                for (int n = 0; n <= N; ++n) {
                    if (last(n) != 0) t.push_back({hcol(k, n), last(n)});
                    if (first(n) != 0) t.push_back({hcol(k + 1, n), -first(n)});
                }
                cb.add_row(t, 0, 0);
            }
        }
    for (int j = 0; j < d; ++j) { cb.add_row({{qcol(0, 0, j), 1.0}}, q0_pos(j), q0_pos(j)); }
    for (int j = 0; j < d; ++j) { cb.add_row({{qcol(nseg - 1, N, j), 1.0}}, qT_pos(j), qT_pos(j)); }
    cb.add_row({{hcol(0, 0), 1.0}}, 0.0, 0.0);                
    cb.add_row({{hcol(nseg - 1, N), 1.0}}, opt.T_min, opt.T_max);
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

    QPResult r = solve_qp(P, cvec, cb.dense(), cb.lvec(), cb.uvec());
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
