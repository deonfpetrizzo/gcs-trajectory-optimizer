// bezier.cpp
#include "gcs_core/bezier.hpp"

#include <algorithm>
#include <cmath>

namespace gcs {

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

}  // namespace gcs
