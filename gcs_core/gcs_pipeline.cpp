// gcs_pipeline.cpp  (solver wrapper, regions, Bezier algebra)
#include "gcs_pipeline.hpp"

#include <Eigen/Sparse>
#include <osqp.h>

#include "libqhullcpp/Qhull.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullVertexSet.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>

namespace gcs {

// ===========================================================================
//  A small dense->sparse OSQP wrapper.
//
//  Solves:   minimize  1/2 x^T P x + c^T x
//            s.t.       l <= Gx <= u
//
//  We assemble constraints in dense Eigen and convert to CSC for OSQP. The
//  problems here are small (dozens of segments x a handful of control points),
//  so dense assembly is fine and keeps the translation readable.
// ===========================================================================
namespace {

// Convert a dense matrix to OSQP CSC (upper-triangular for P; full for A).
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

}  // namespace

QPResult solve_qp_extern(const MatrixXd& P, const VectorXd& c,
                         const MatrixXd& G, const VectorXd& l, const VectorXd& u) {
    return solve_qp(P, c, G, l, u);
}

// ===========================================================================
//  ConvexRegion LP utilities (solved as degenerate QPs with P = 0).
// ===========================================================================
std::pair<VectorXd, double> ConvexRegion::chebyshev_center() const {
    // maximize r  s.t.  a_i^T x + ||a_i|| r <= b_i,  r >= 0
    // variables: [x (d), r];  minimize -r  (P=0 -> small reg added below)
    const int d = dim();
    const int m = A.rows();
    const int nv = d + 1;
    VectorXd norms = A.rowwise().norm();

    MatrixXd P = MatrixXd::Zero(nv, nv);
    P(d, d) = 1e-9;                       
    VectorXd c = VectorXd::Zero(nv);
    c(d) = -1.0;                          // minimize -r

    // Constraints: A x + norms r <= b   (m rows),  and r >= 0 (1 row).
    MatrixXd G(m + 1, nv);
    VectorXd lo(m + 1), up(m + 1);
    G.setZero();
    G.block(0, 0, m, d) = A;
    G.block(0, d, m, 1) = norms;
    lo.head(m).setConstant(-OSQP_INFTY);
    up.head(m) = b;
    G.row(m).setZero(); G(m, d) = 1.0;
    lo(m) = 0.0; up(m) = OSQP_INFTY;

    QPResult r = solve_qp(P, c, G, lo, up);
    if (!r.ok) return {VectorXd::Zero(d), 0.0};
    return {r.x.head(d), r.x(d)};
}

bool ConvexRegion::intersects(const ConvexRegion& other, double tol) const {
    const int d = dim();
    const int m1 = A.rows(), m2 = other.A.rows();
    MatrixXd P = 1e-9 * MatrixXd::Identity(d, d);
    VectorXd c = VectorXd::Zero(d);
    MatrixXd G(m1 + m2, d);
    VectorXd lo(m1 + m2), up(m1 + m2);
    G.topRows(m1) = A;      
    up.head(m1) = b.array() + tol;
    G.bottomRows(m2) = other.A; 
    up.tail(m2) = other.b.array() + tol;
    lo.setConstant(-OSQP_INFTY);
    QPResult r = solve_qp(P, c, G, lo, up);
    return r.ok;
}

// ===========================================================================
//  PART 1.  Region generation
// ===========================================================================
MatrixXd sphere_flip(const MatrixXd& points, const VectorXd& center, double R) {
    MatrixXd local = points.rowwise() - center.transpose();
    VectorXd norms = local.rowwise().norm();
    MatrixXd flipped = MatrixXd::Zero(points.rows(), points.cols());
    for (int i = 0; i < points.rows(); ++i) {
        if (norms(i) > 1e-10)
            flipped.row(i) = local.row(i) * ((2.0 * R - norms(i)) / norms(i));
    }
    return flipped.rowwise() + center.transpose();
}

MatrixXd inv_sphere_flip(const MatrixXd& flipped, const VectorXd& center, double R) {
    return sphere_flip(flipped, center, R);
}

// Run Qhull on `pts` (rows = points) and return vertices and facet (A,b) of the
// convex hull, with hull expressed as A x <= b.
namespace {
struct Hull {
    MatrixXd vertices;       
    MatrixXd A;              
    VectorXd b;           
    std::vector<std::vector<int>> facet_vertex_ids; 
};

Hull qhull(const MatrixXd& pts) {
    using namespace orgQhull;
    const int n = pts.rows(), d = pts.cols();
    std::vector<double> flat(n * d);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < d; ++j) flat[i * d + j] = pts(i, j);

    Qhull q;
    q.runQhull("", d, n, flat.data(), "QJ");

    Hull h;
    std::map<countT, int> id2idx;
    std::vector<VectorXd> verts;
    for (QhullVertex v : q.vertexList()) {
        VectorXd p(d);
        const double* c = v.point().coordinates();
        for (int j = 0; j < d; ++j) p(j) = c[j];
        id2idx[v.point().id()] = static_cast<int>(verts.size());
        verts.push_back(p);
    }
    h.vertices.resize(verts.size(), d);
    for (size_t i = 0; i < verts.size(); ++i) h.vertices.row(i) = verts[i].transpose();

    std::vector<VectorXd> Arows;
    std::vector<double> brows;
    for (QhullFacet f : q.facetList()) {
        if (!f.isGood() && f.isUpperDelaunay()) continue;
        auto hp = f.hyperplane();
        const double* coords = hp.coordinates();
        VectorXd nrm(d);
        for (int j = 0; j < d; ++j) nrm(j) = coords[j];
        Arows.push_back(nrm);
        brows.push_back(-hp.offset());
        std::vector<int> ids;
        for (QhullVertex v : f.vertices())
            ids.push_back(id2idx[v.point().id()]);
        h.facet_vertex_ids.push_back(ids);
    }
    h.A.resize(Arows.size(), d);
    h.b.resize(brows.size());
    for (size_t i = 0; i < Arows.size(); ++i) {
        h.A.row(i) = Arows[i].transpose();
        h.b(i) = brows[i];
    }
    return h;
}
}  // namespace

MatrixXd generate_star_convex(const VectorXd& pq, const MatrixXd& cloud, double R,
                               bool sphere_floor) {
    const int d = cloud.cols();

    std::vector<int> in;
    for (int i = 0; i < cloud.rows(); ++i) {
        double dd = (cloud.row(i).transpose() - pq).norm();
        if (dd > 1e-10 && dd < R) in.push_back(i);
    }

    if (!sphere_floor) {
        MatrixXd pts(in.size(), d);
        for (size_t i = 0; i < in.size(); ++i) pts.row(i) = cloud.row(in[i]);
        MatrixXd flipped = sphere_flip(pts, pq, R);
        Hull h = qhull(flipped);
        return inv_sphere_flip(h.vertices, pq, R);
    }

    // sphere_floor=true: add sphere-surface samples (2d cardinal + 2^d diagonal).
    // Points at d=R flip to themselves (2R-R=R), so they pass through sphere_flip
    // unchanged. This ensures:
    //   - Zero-obstacle case: Qhull always has input; result is the sphere-inscribed
    //     polytope, a valid obstacle-free region with the full sphere's extent.
    //   - Sparse-obstacle case: asymmetric obstacle coverage can never collapse the
    //     hull to a sliver; the region is always >= the sphere-inscribed polytope.
    std::vector<VectorXd> sphere_samples;
    for (int j = 0; j < d; ++j)
        for (int s : {-1, 1}) {
            VectorXd v = VectorXd::Zero(d);
            v(j) = s * R;
            sphere_samples.push_back(pq + v);
        }
    if (d <= 6) {
        const int ncorners = 1 << d;
        for (int mask = 0; mask < ncorners; ++mask) {
            VectorXd dir(d);
            for (int j = 0; j < d; ++j) dir(j) = ((mask >> j) & 1) ? 1.0 : -1.0;
            sphere_samples.push_back(pq + R * dir.normalized());
        }
    }

    const int n_obs = static_cast<int>(in.size());
    const int n_sph = static_cast<int>(sphere_samples.size());
    MatrixXd pts(n_obs + n_sph, d);
    for (int i = 0; i < n_obs; ++i) pts.row(i) = cloud.row(in[i]);
    for (int i = 0; i < n_sph; ++i) pts.row(n_obs + i) = sphere_samples[i].transpose();

    MatrixXd flipped = sphere_flip(pts, pq, R);
    Hull h = qhull(flipped);
    return inv_sphere_flip(h.vertices, pq, R);
}

namespace {
// barycentric test: is p inside the simplex with the given (d+1) vertices?
bool in_simplex(const VectorXd& p, const MatrixXd& verts, double tol = 1e-8) {
    const int d = p.size();
    MatrixXd A(d, d);
    for (int j = 0; j < d; ++j) A.col(j) = verts.row(j + 1).transpose() - verts.row(0).transpose();
    VectorXd rhs = p - verts.row(0).transpose();
    Eigen::FullPivLU<MatrixXd> lu(A);
    if (!lu.isInvertible()) return false;
    VectorXd bary = lu.solve(rhs);
    return (bary.array() >= -tol).all() && bary.sum() <= 1.0 + tol;
}
}  // namespace

void star_to_convex(const MatrixXd& star_verts, const VectorXd& pq,
                    MatrixXd& A_out, VectorXd& b_out) {
    const int d = star_verts.cols();
    MatrixXd local = star_verts.rowwise() - pq.transpose();
    VectorXd origin = VectorXd::Zero(d);

    Hull h = qhull(local);
    const int nf = h.A.rows();
    A_out.resize(nf, d);
    b_out.resize(nf);
    for (int f = 0; f < nf; ++f) {
        VectorXd normal = h.A.row(f).transpose();
        double face_d = h.b(f);
        // simplex = origin + facet vertices
        const auto& ids = h.facet_vertex_ids[f];
        MatrixXd simplex(d + 1, d);
        simplex.row(0) = origin.transpose();
        for (int k = 0; k < d; ++k) simplex.row(k + 1) = h.vertices.row(ids[k]);

        double b_val = face_d;
        bool any = false;
        for (int i = 0; i < local.rows(); ++i) {
            VectorXd v = local.row(i).transpose();
            if (in_simplex(v, simplex)) {
                double proj = normal.dot(v);
                b_val = any ? std::max(b_val, proj) : proj;
                any = true;
            }
        }
        A_out.row(f) = normal.transpose();
        b_out(f) = b_val;
    }
    // shift to world coordinates: b_world = b_local + A local @ pq
    b_out += A_out * pq;
}

namespace {
VectorXd tighten_against_cloud(const MatrixXd& A, const VectorXd& b,
                               const VectorXd& pq, const MatrixXd& cloud, double R) {
    std::vector<int> near;
    for (int i = 0; i < cloud.rows(); ++i) {
        double dd = (cloud.row(i).transpose() - pq).norm();
        if (dd > 1e-9 && dd < R) near.push_back(i);
    }
    if (near.empty()) return b;
    MatrixXd normals = A;
    for (int i = 0; i < A.rows(); ++i) normals.row(i) /= A.row(i).norm();

    VectorXd out = b;
    std::vector<bool> seen(A.rows(), false);
    for (int idx : near) {
        VectorXd p = cloud.row(idx).transpose();
        VectorXd dir = (p - pq).normalized();
        // owner facet: most aligned outward normal
        int best = 0; double bestv = -1e18;
        for (int f = 0; f < A.rows(); ++f) {
            double a = normals.row(f).dot(dir.transpose());
            if (a > bestv) { bestv = a; best = f; }
        }
        double proj = A.row(best).dot(p);
        if (!seen[best]) { out(best) = std::min(out(best), proj); seen[best] = true; }
        else out(best) = std::min(out(best), proj);
    }
    return out;
}
}  // namespace

ConvexRegion convex_region_from_pointcloud(const VectorXd& pq, const MatrixXd& cloud,
                                           double R, bool tighten,
                                           const std::string& name,
                                           bool sphere_floor) {
    MatrixXd sv = generate_star_convex(pq, cloud, R, sphere_floor);
    MatrixXd A; VectorXd b;
    star_to_convex(sv, pq, A, b);
    if (tighten) b = tighten_against_cloud(A, b, pq, cloud, R);
    return ConvexRegion(A, b, name);
}

// ===========================================================================
//  Bezier algebra (Sec. 11.3)
// ===========================================================================
namespace {
double binom(int n, int k) {
    if (k < 0 || k > n) return 0.0;
    double r = 1.0;
    for (int i = 0; i < k; ++i) r = r * (n - i) / (i + 1);
    return r;
}
}  // namespace

MatrixXd derivative_operator(int N, double duration, int k) {
    MatrixXd op = MatrixXd::Identity(N + 1, N + 1);
    int deg = N;
    for (int it = 0; it < k; ++it) {
        MatrixXd D = MatrixXd::Zero(deg, deg + 1);
        for (int n = 0; n < deg; ++n) {
            D(n, n) = -deg / duration;
            D(n, n + 1) = deg / duration;
        }
        op = D * op;
        --deg;
    }
    return op;
}

MatrixXd energy_matrix(int N) {
    MatrixXd M(N + 1, N + 1);
    for (int n = 0; n <= N; ++n)
        for (int m = 0; m <= N; ++m)
            M(n, m) = binom(N, n) * binom(N, m) / ((2.0 * N + 1) * binom(2 * N, n + m));
    return M;
}

VectorXd bernstein_basis(int N, double tau) {
    VectorXd v(N + 1);
    for (int n = 0; n <= N; ++n)
        v(n) = binom(N, n) * std::pow(tau, n) * std::pow(1.0 - tau, N - n);
    return v;
}

}  // namespace gcs
