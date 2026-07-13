// free_space_regions.cpp 
#include "gcs_core/free_space_regions.hpp"

#include "libqhullcpp/Qhull.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullVertexSet.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

namespace gcs {

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

MatrixXd generate_star_convex(const VectorXd& pq, const PointIndex& index, double R,
                               bool sphere_floor) {
    const MatrixXd& cloud = index.cloud();
    const int d = index.dim();

    std::vector<int> in;
    index.radius_query(pq, R, 1e-10, in);

    if (!sphere_floor) {
        MatrixXd pts(in.size(), d);
        for (size_t i = 0; i < in.size(); ++i) pts.row(i) = cloud.row(in[i]);
        MatrixXd flipped = sphere_flip(pts, pq, R);
        Hull h = qhull(flipped);
        return inv_sphere_flip(h.vertices, pq, R);
    }

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

MatrixXd generate_star_convex(const VectorXd& pq, const MatrixXd& cloud, double R,
                               bool sphere_floor) {
    PointIndex index(cloud, R);
    return generate_star_convex(pq, index, R, sphere_floor);
}

namespace {
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
    b_out += A_out * pq;
}

namespace {
VectorXd tighten_against_cloud(const MatrixXd& A, const VectorXd& b,
                               const VectorXd& pq, const PointIndex& index, double R) {
    const MatrixXd& cloud = index.cloud();
    std::vector<int> near;
    index.radius_query(pq, R, 1e-9, near);
    if (near.empty()) return b;
    MatrixXd normals = A;
    for (int i = 0; i < A.rows(); ++i) normals.row(i) /= A.row(i).norm();

    VectorXd out = b;
    std::vector<bool> seen(A.rows(), false);
    for (int idx : near) {
        VectorXd p = cloud.row(idx).transpose();
        VectorXd dir = (p - pq).normalized();
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

ConvexRegion convex_region_from_pointcloud(const VectorXd& pq, const PointIndex& index,
                                           double R, bool tighten,
                                           const std::string& name,
                                           bool sphere_floor) {
    MatrixXd sv = generate_star_convex(pq, index, R, sphere_floor);
    MatrixXd A; VectorXd b;
    star_to_convex(sv, pq, A, b);
    if (tighten) b = tighten_against_cloud(A, b, pq, index, R);
    return ConvexRegion(A, b, name);
}

ConvexRegion convex_region_from_pointcloud(const VectorXd& pq, const MatrixXd& cloud,
                                           double R, bool tighten,
                                           const std::string& name,
                                           bool sphere_floor) {
    PointIndex index(cloud, R);
    return convex_region_from_pointcloud(pq, index, R, tighten, name, sphere_floor);
}

}  // namespace gcs
