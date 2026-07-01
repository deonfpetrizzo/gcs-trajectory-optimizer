#include "region_viz.hpp"
#include <cmath>

namespace gcs {
using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd region_vertices(const ConvexRegion& region, double tol) {
    const int m = static_cast<int>(region.A.rows());
    const int d = static_cast<int>(region.A.cols());
    std::vector<VectorXd> verts;
    if (m < d) return MatrixXd(0, d);

    std::vector<int> c(d);
    for (int i = 0; i < d; ++i) c[i] = i;
    auto feasible = [&](const VectorXd& x) {
        return ((region.A * x).array() <= region.b.array() + tol).all();
    };
    while (true) {
        MatrixXd As(d, d);
        VectorXd bs(d);
        for (int i = 0; i < d; ++i) { As.row(i) = region.A.row(c[i]); bs(i) = region.b(c[i]); }
        Eigen::FullPivLU<MatrixXd> lu(As);
        if (lu.isInvertible()) {
            VectorXd x = lu.solve(bs);
            if (feasible(x)) {
                bool dup = false;
                for (const auto& v : verts) if ((v - x).norm() < 1e-6) { dup = true; break; }
                if (!dup) verts.push_back(x);
            }
        }
        int i = d - 1;
        while (i >= 0 && c[i] == m - d + i) --i;
        if (i < 0) break;
        ++c[i];
        for (int j = i + 1; j < d; ++j) c[j] = c[j - 1] + 1;
    }
    MatrixXd V(static_cast<int>(verts.size()), d);
    for (size_t i = 0; i < verts.size(); ++i) V.row(static_cast<int>(i)) = verts[i].transpose();
    return V;
}

std::vector<std::array<int, 2>>
region_edges(const ConvexRegion& region, const MatrixXd& V, double tol) {
    const int m = static_cast<int>(region.A.rows());
    const int d = static_cast<int>(region.A.cols());
    const int n = static_cast<int>(V.rows());
    std::vector<std::vector<char>> tight(n, std::vector<char>(m, 0));
    for (int i = 0; i < n; ++i) {
        VectorXd r = region.A * V.row(i).transpose() - region.b;
        for (int k = 0; k < m; ++k) tight[i][k] = (std::abs(r(k)) <= tol) ? 1 : 0;
    }
    std::vector<std::array<int, 2>> edges;
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j) {
            int common = 0;
            for (int k = 0; k < m; ++k) if (tight[i][k] && tight[j][k]) ++common;
            if (common >= d - 1) edges.push_back({i, j});
        }
    return edges;
}
}  // namespace gcs