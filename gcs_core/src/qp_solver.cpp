// qp_solver.cpp
#include "gcs_core/qp_solver.hpp"

#include <Eigen/Sparse>
#include <osqp.h>

#include <vector>

namespace gcs {

namespace {

struct CSC {
    std::vector<c_float> data;
    std::vector<c_int>   rows;
    std::vector<c_int>   cols;
    c_int m = 0, n = 0, nnz = 0;
};

CSC to_csc(const MatrixXd& M, bool upper_triangular_only) {
    CSC c;
    c.m = M.rows();
    c.n = M.cols();
    c.cols.push_back(0);
    for (int j = 0; j < M.cols(); ++j) {
        for (int i = 0; i < M.rows(); ++i) {
            if (upper_triangular_only && i > j) continue;
            double v = M(i, j);
            if (v != 0.0 || (upper_triangular_only && i == j)) {
                c.data.push_back(v);
                c.rows.push_back(i);
            }
        }
        c.cols.push_back(static_cast<c_int>(c.data.size()));
    }
    c.nnz = static_cast<c_int>(c.data.size());
    return c;
}

}  // namespace

QPResult solve_qp(const MatrixXd& P, const VectorXd& cvec,
                  const MatrixXd& G, const VectorXd& l, const VectorXd& u) {
    QPResult res;
    const c_int n = P.rows();
    const c_int m = G.rows();

    CSC Pc = to_csc(P, /*upper*/ true);
    CSC Gc = to_csc(G, /*upper*/ false);

    std::vector<c_float> q(cvec.data(), cvec.data() + n);
    std::vector<c_float> lo(l.data(), l.data() + m);
    std::vector<c_float> up(u.data(), u.data() + m);

    OSQPData* data = static_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
    data->n = n;
    data->m = m;
    data->P = csc_matrix(n, n, Pc.nnz, Pc.data.data(), Pc.rows.data(), Pc.cols.data());
    data->A = csc_matrix(m, n, Gc.nnz, Gc.data.data(), Gc.rows.data(), Gc.cols.data());
    data->q = q.data();
    data->l = lo.data();
    data->u = up.data();

    OSQPSettings* settings = static_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
    osqp_set_default_settings(settings);
    settings->verbose = 0;
    settings->eps_abs = 1e-6;
    settings->eps_rel = 1e-6;
    settings->max_iter = 200000;
    settings->scaling = 15;
    settings->adaptive_rho = 1;
    settings->polish = 1;
    settings->polish_refine_iter = 5;

    OSQPWorkspace* work = nullptr;
    c_int err = osqp_setup(&work, data, settings);
    if (err) { res.status = "osqp_setup_failed"; }
    else {
        osqp_solve(work);
        int st = work->info->status_val;
        if (st == OSQP_SOLVED || st == OSQP_SOLVED_INACCURATE) {
            res.ok = true;
            res.x = Eigen::Map<VectorXd>(work->solution->x, n);
        }
        res.status = work->info->status;
    }
    if (work) osqp_cleanup(work);
    if (data) { c_free(data->P); c_free(data->A); c_free(data); }
    if (settings) c_free(settings);
    return res;
}

}  // namespace gcs
