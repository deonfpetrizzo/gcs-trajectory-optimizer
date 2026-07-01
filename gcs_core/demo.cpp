// demo.cpp -- mirrors the Python run_demo / run_timing_demo.
// Builds the same 2D environment, grows star-convex regions along a nominal
// path, runs the geometry-only and the composite shape+timing planners, prints
// summary stats, and writes trajectory samples to CSV (no plotting in C++).
// ============================================================================
#include "gcs_pipeline.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

using namespace gcs;

// dense perimeter samples of an axis-aligned rectangle (an "obstacle surface")
static void rect_perimeter(double x0, double y0, double x1, double y1,
                           double spacing, std::vector<Eigen::Vector2d>& out) {
    int nx = std::max(2, (int)((x1 - x0) / spacing));
    int ny = std::max(2, (int)((y1 - y0) / spacing));
    for (int i = 0; i < nx; ++i) {
        double x = x0 + (x1 - x0) * i / (nx - 1);
        out.push_back({x, y0}); out.push_back({x, y1});
    }
    for (int i = 0; i < ny; ++i) {
        double y = y0 + (y1 - y0) * i / (ny - 1);
        out.push_back({x0, y}); out.push_back({x1, y});
    }
}

int main() {
    // ---- environment (same as the Python demo) ----
    std::vector<std::array<double, 4>> obstacles = {
        {4, 0, 6, 9}, {10, 5, 12, 14}, {15, 0, 17, 8},
        {7.5, 11, 9, 12.5}, {18.5, 9.5, 20, 11}};
    std::vector<Eigen::Vector2d> pts;
    for (auto& o : obstacles) rect_perimeter(o[0], o[1], o[2], o[3], 0.25, pts);
    rect_perimeter(0, 0, 22, 14, 0.4, pts);
    MatrixXd cloud(pts.size(), 2);
    for (size_t i = 0; i < pts.size(); ++i) cloud.row(i) = pts[i].transpose();

    std::vector<Eigen::Vector2d> wp = {
        {1.5, 2}, {3, 6.5}, {5, 11}, {8.5, 8.5}, {11, 2.5},
        {13.5, 6}, {16, 11}, {19, 6.5}, {21, 7}};

    // densify so neighbouring regions overlap
    std::vector<Eigen::Vector2d> path;
    double max_step = 2.0;
    path.push_back(wp[0]);
    for (size_t i = 0; i + 1 < wp.size(); ++i) {
        Eigen::Vector2d a = wp[i], b = wp[i + 1];
        int n = std::max(1, (int)std::ceil((b - a).norm() / max_step));
        for (int k = 1; k <= n; ++k) path.push_back(a + (b - a) * double(k) / n);
    }

    // ---- grow regions ----
    double R = 5.0;
    std::vector<ConvexRegion> regions;
    for (auto& pq : path) {
        try {
            ConvexRegion reg = convex_region_from_pointcloud(pq, cloud, R);
            if (reg.contains(pq)) regions.push_back(reg);
        } catch (...) { /* skip degenerate */ }
    }
    std::cout << "grew " << regions.size() << " convex regions along the nominal path\n";

    Eigen::Vector2d q0 = wp.front(), qT = wp.back();

    // ===== geometry-only planner =====
    {
        GCSBezierPlanner planner(regions);
        planner.build_graph();
        auto seq = planner.find_region_sequence(q0, qT);
        std::cout << "[geometry] region sequence length " << seq.size() << "\n";

        BoundaryConditions bc0, bcT;
        bc0.set(0, q0); bc0.set(1, Eigen::Vector2d::Zero()); bc0.set(2, Eigen::Vector2d::Zero());
        bcT.set(0, qT); bcT.set(1, Eigen::Vector2d::Zero()); bcT.set(2, Eigen::Vector2d::Zero());

        GCSBezierPlanner::Options opt;
        opt.degree = 6; opt.continuity = 3; opt.total_time = 20.0;
        opt.cost_derivatives = {{2, 1.0}, {1, 1e-3}};

        auto traj = planner.solve_convex_restriction(seq, bc0, bcT, opt);
        double T = traj.total_duration();
        double pkspeed = 0; bool in_union = true;
        std::ofstream f("traj_geometry.csv"); f << "t,x,y,speed\n";
        for (int i = 0; i <= 400; ++i) {
            double t = T * i / 400.0;
            VectorXd p = traj.eval(t, 0), v = traj.eval(t, 1);
            pkspeed = std::max(pkspeed, v.norm());
            bool ok = false; for (auto& r : regions) if (r.contains(p, 1e-4)) { ok = true; break; }
            in_union = in_union && ok;
            f << t << "," << p(0) << "," << p(1) << "," << v.norm() << "\n";
        }
        std::cout << "[geometry] cost=" << traj.cost << " segments=" << traj.control_points.size()
                  << " in_union=" << in_union << " peak_speed=" << pkspeed << "\n";
        std::cout << "[geometry] start_err=" << (traj.eval(0, 0) - q0).norm()
                  << " end_err=" << (traj.eval(T, 0) - qT).norm() << "\n";
    }

    // ===== composite shape + timing planner =====
    {
        GCSCompositeBezierPlanner planner(regions);
        planner.build_graph();
        auto seq = planner.find_region_sequence(q0, qT);

        GCSCompositeBezierPlanner::Options opt;
        opt.degree = 6; opt.continuity = 2;
        opt.vel_limit = 2.0;
        opt.v0 = Eigen::Vector2d::Zero(); opt.vT = Eigen::Vector2d::Zero();
        opt.rest_to_rest_accel = true;
        opt.T_min = 0.0; opt.T_max = 80.0;
        opt.w_time = 1.0; opt.w_geom_accel = 8e-2; opt.w_time_accel = 8e-2;
        opt.soc_facets = 24;

        auto traj = planner.solve_composite(seq, q0, qT, opt);
        double T = traj.total_duration();
        double pkspeed = 0, pkacc = 0; bool in_union = true;
        std::ofstream f("traj_timing.csv"); f << "t,x,y,speed,accel\n";
        for (int i = 0; i <= 800; ++i) {
            double t = T * i / 800.0;
            VectorXd p = traj.position(t), v = traj.velocity(t), a = traj.acceleration(t);
            pkspeed = std::max(pkspeed, v.norm()); pkacc = std::max(pkacc, a.norm());
            bool ok = false; for (auto& r : regions) if (r.contains(p, 1e-4)) { ok = true; break; }
            in_union = in_union && ok;
            f << t << "," << p(0) << "," << p(1) << "," << v.norm() << "," << a.norm() << "\n";
        }
        std::cout << "[timing] total_time=" << T << "s cost=" << traj.cost
                  << " segments=" << traj.q_cps.size() << "\n";
        std::cout << "[timing] in_union=" << in_union << " peak_speed=" << pkspeed
                  << " (limit 2.0) peak_accel=" << pkacc << "\n";
        std::cout << "[timing] start_speed=" << traj.velocity(0).norm()
                  << " end_speed=" << traj.velocity(T).norm() << "\n";
        std::cout << "wrote traj_geometry.csv and traj_timing.csv\n";
    }
    return 0;
}
