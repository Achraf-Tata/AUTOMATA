// =============================================================================
// reachability_cpu.cpp — Member 4
// =============================================================================
#include "reachability_cpu.hpp"
#include "unicycle_model.hpp"
#include <omp.h>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <cstring>
#include <limits>

namespace solver {

// ---------------------------------------------------------------------------
// Internal helper: worst-case value over all successors of (cell, u, q)
// ---------------------------------------------------------------------------

/// Given a source cell index and a control, finds all grid cells that can be
/// reached (using the footprint table as a bounding box, then checking each
/// candidate), evaluates V at each successor under the DFA transition,
/// and returns the MAXIMUM (worst-case) V.
static float worst_case_value(
    int c1, int c2, int c3,
    int q,
    int ui,
    const grid::FootprintTable& footprint,
    const std::vector<float>&   V,
    const grid::RegionConfig&   regions)
{
    const auto& fp  = footprint[ui][c3];
    const auto& u   = unicycle::ALL_CONTROLS[ui];
    float worst = 0.0f;

    // Iterate over the bounding box of reachable cells
    int x_lo = std::max(0,            c1 + fp.dc1_min);
    int x_hi = std::min(grid::NX - 1, c1 + fp.dc1_max);
    int y_lo = std::max(0,            c2 + fp.dc2_min);
    int y_hi = std::min(grid::NY - 1, c2 + fp.dc2_max);

    for (int nc1 = x_lo; nc1 <= x_hi; ++nc1) {
        for (int nc2 = y_lo; nc2 <= y_hi; ++nc2) {
            // For each candidate theta offset
            for (int nc3 = 0; nc3 < grid::NTHETA; ++nc3) {
                // Confirm reachability: check if cell centre of (nc1,nc2,nc3)
                // is inside the reachable box of the source cell under u.
                unicycle::State src_ctr = grid::cell_center({c1, c2, c3});
                unicycle::Box src_box {
                    src_ctr.x - grid::DX / 2,  src_ctr.x + grid::DX / 2,
                    src_ctr.y - grid::DY / 2,  src_ctr.y + grid::DY / 2,
                    src_ctr.theta - grid::DTHETA / 2, src_ctr.theta + grid::DTHETA / 2
                };
                auto rb = unicycle::compute_reachable_box(src_box, u, 0.05, 1.0);
                unicycle::State dst_ctr = grid::cell_center({nc1, nc2, nc3});
                bool in_box = (dst_ctr.x >= rb.x_min && dst_ctr.x <= rb.x_max &&
                               dst_ctr.y >= rb.y_min && dst_ctr.y <= rb.y_max);
                if (!in_box) continue;

                int label = grid::get_region_label(dst_ctr.x, dst_ctr.y, regions);
                int q_next = automaton::dfa_transition(q, label);
                float v_next = V[automaton::product_index(q_next, grid::flat_index(nc1, nc2, nc3))];
                worst = std::max(worst, v_next);
            }
        }
    }
    return worst;
}

// ---------------------------------------------------------------------------
// Main solver
// ---------------------------------------------------------------------------

int solve_reachability_openmp(
    std::vector<float>&         V,
    std::vector<uint8_t>&       ctrl,
    const grid::RegionConfig&   regions,
    const grid::FootprintTable& footprint,
    ProgressCallback            callback)
{
    int iter = 0;
    float delta = automaton::INF;

    while (delta > CONVERGENCE_EPS && iter < MAX_ITERATIONS) {
        delta = 0.0f;
        // Atomic max for delta across threads
        // We use a local per-thread delta, then reduce.
        std::vector<float> local_deltas(omp_get_max_threads(), 0.0f);

        // Parallelise over DFA states and x-cells (5 * 100 = 500 tasks)
        #pragma omp parallel for schedule(dynamic, 1)
        for (int qc1 = 0; qc1 < automaton::NDFA * grid::NX; ++qc1) {
            int q  = (qc1 / grid::NX) + 1;
            int c1 = qc1 % grid::NX;
            int tid = omp_get_thread_num();
                for (int c2 = 0; c2 < grid::NY; ++c2) {
                    for (int c3 = 0; c3 < grid::NTHETA; ++c3) {
                        // Goal and sink states do not change
                        if (q == 4 || q == 5) continue;

                        int pidx = automaton::product_index(q, grid::flat_index(c1, c2, c3));
                        float best_v   = automaton::INF;
                        uint8_t best_u = 255;

                        for (int ui = 0; ui < 15; ++ui) {
                            float wv = worst_case_value(c1, c2, c3, q, ui,
                                                        footprint, V, regions);
                            if (wv < best_v) {
                                best_v = wv;
                                best_u = static_cast<uint8_t>(ui);
                            }
                        }

                        float old_v = V[pidx];
                        if (best_v < old_v - 1e-6f) {
                            V[pidx]    = best_v;
                            ctrl[pidx] = best_u;
                            local_deltas[tid] = std::max(local_deltas[tid],
                                                         old_v - best_v);
                        }
                    }
                }
            }

        for (float ld : local_deltas) delta = std::max(delta, ld);
        ++iter;
        if (callback) callback(iter, delta);
    }
    return iter;
}

// ---------------------------------------------------------------------------
// File I/O
// ---------------------------------------------------------------------------

bool save_controller_binary(const char*                  path,
                             const std::vector<float>&    V,
                             const std::vector<uint8_t>&  ctrl)
{
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f.write("CTRL1.00", 8);
    uint32_t n = static_cast<uint32_t>(automaton::NTOTAL);
    f.write(reinterpret_cast<const char*>(&n), 4);
    f.write(reinterpret_cast<const char*>(V.data()),    n * sizeof(float));
    f.write(reinterpret_cast<const char*>(ctrl.data()), n * sizeof(uint8_t));
    return f.good();
}

bool load_controller_binary(const char*            path,
                              std::vector<float>&    V,
                              std::vector<uint8_t>&  ctrl)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    char magic[8];
    f.read(magic, 8);
    if (std::memcmp(magic, "CTRL1.00", 8) != 0) return false;
    uint32_t n = 0;
    f.read(reinterpret_cast<char*>(&n), 4);
    if (n != static_cast<uint32_t>(automaton::NTOTAL)) return false;
    V.resize(n);
    ctrl.resize(n);
    f.read(reinterpret_cast<char*>(V.data()),    n * sizeof(float));
    f.read(reinterpret_cast<char*>(ctrl.data()), n * sizeof(uint8_t));
    return f.good();
}

} // namespace solver
