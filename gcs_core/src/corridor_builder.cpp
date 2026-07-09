// corridor_builder.cpp
#include "gcs_core/corridor_builder.hpp"

#include <atomic>
#include <random>
#include <thread>

namespace gcs {
namespace {

struct DSU {
    std::vector<int> parent;
    int make_set() { 
        parent.push_back(static_cast<int>(parent.size())); 
        return static_cast<int>(parent.size()) - 1; 
    }
    int find(int x) { 
        while (parent[x] != x) { 
            parent[x] = parent[parent[x]]; 
            x = parent[x]; 
        } 
        return x; 
    }
    void unite(int a, int b) { 
        parent[find(a)] = find(b); 
    }
};

struct PathSampler {
    const std::vector<VectorXd>& wp;
    std::vector<double> cum;  
    double total = 0.0;

    explicit PathSampler(const std::vector<VectorXd>& waypoints) : wp(waypoints) {
        cum.resize(wp.size(), 0.0);
        for (size_t i = 1; i < wp.size(); ++i)
            cum[i] = cum[i - 1] + (wp[i] - wp[i - 1]).norm();
        total = cum.empty() ? 0.0 : cum.back();
    }

    VectorXd at(double u) const {
        if (wp.size() == 1 || total <= 0.0) return wp.front();
        size_t i = 0;
        while (i + 2 < wp.size() && cum[i + 1] < u) ++i;
        double seg = cum[i + 1] - cum[i];
        double t = (seg > 1e-12) ? (u - cum[i]) / seg : 0.0;
        return wp[i] + t * (wp[i + 1] - wp[i]);
    }
};

bool grow_one(const VectorXd& pq, const MatrixXd& cloud, const CorridorOptions& opt, ConvexRegion& out) {
    try {
        ConvexRegion reg = convex_region_from_pointcloud(pq, cloud, opt.R, opt.tighten, "", opt.sphere_floor);
        if (opt.region_postprocess) opt.region_postprocess(pq, reg);
        if (reg.contains(pq)) {
            out = std::move(reg);
            return true;
        }
    } catch (const std::exception&) {}
    return false;
}

}  // namespace

CorridorResult build_corridor_parallel(const VectorXd& q0, const VectorXd& qT,
                                       const std::vector<VectorXd>& nominal_path,
                                       const MatrixXd& cloud,
                                       const CorridorOptions& opt) {
    CorridorResult result;
    if (nominal_path.empty()) return result;

    PathSampler sampler(nominal_path);
    DSU dsu;

    std::vector<ConvexRegion> pool;
    std::vector<int> src, tgt;  

    auto add_region = [&](ConvexRegion&& reg) {
        int idx = static_cast<int>(pool.size());
        dsu.make_set();
        for (int j = 0; j < idx; ++j)
            if (reg.intersects(pool[j])) dsu.unite(idx, j);
        if (reg.contains(q0)) src.push_back(idx);
        if (reg.contains(qT)) tgt.push_back(idx);
        pool.push_back(std::move(reg));
    };

    auto connected = [&]() {
        for (int s : src)
            for (int t : tgt)
                if (dsu.find(s) == dsu.find(t)) return true;
        return false;
    };

    for (const VectorXd& endpt : {q0, qT}) {
        ConvexRegion reg;
        if (grow_one(endpt, cloud, opt, reg)) add_region(std::move(reg));
    }
    if (connected()) { 
        result.regions = std::move(pool); 
        result.connected = true; 
        return result; 
    }

    const uint64_t base_seed = opt.seed != 0 ? opt.seed : std::random_device{}();

    std::uniform_real_distribution<double> U(0.0, sampler.total);

    for (int round = 0; round < opt.max_rounds; ++round) {
        result.rounds = round + 1;

        const int N = std::max(1, opt.num_threads);
        std::vector<ConvexRegion> grown(N);
        std::vector<char> ok(N, 0);

        auto worker = [&](int tid) {
            std::mt19937_64 rng(base_seed ^ (uint64_t(round) << 32) ^ uint64_t(tid * 2654435761u));
            std::uniform_real_distribution<double> u = U;  
            VectorXd pq = sampler.at(u(rng));
            if (opt.drop_covered_samples) {
                for (const auto& r : pool)
                    if (r.contains(pq)) return;
            }
            ConvexRegion reg;
            if (grow_one(pq, cloud, opt, reg)) { 
                grown[tid] = std::move(reg); 
                ok[tid] = 1; 
            }
        };

        std::vector<std::thread> threads;
        threads.reserve(N);
        for (int tid = 0; tid < N; ++tid) threads.emplace_back(worker, tid);
        for (auto& th : threads) th.join();

        for (int tid = 0; tid < N; ++tid)
            if (ok[tid]) add_region(std::move(grown[tid]));

        if (connected()) { result.connected = true; break; }
    }

    result.regions = std::move(pool);
    return result;
}

}  // namespace gcs
