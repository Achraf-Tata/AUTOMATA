#include "backward_reachability.hpp"
#include <chrono>
#include <thread>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <cstring>

#if defined(_OPENMP)
#include <omp.h>
#endif

namespace symbolic::synthesis {

using namespace std;
using namespace chrono;
using namespace abstraction;
using namespace automaton;
using enum AutomatonState;
using enum ObservationSymbol;

// Works backward from the goal on the combined tracking graph (cell position x mission stage)
SynthesisResult BackwardReachabilitySolver::solve_product(const TransitionSystem& ts,const ProductAutomaton& automaton,size_t max_iters){
    auto t0 = high_resolution_clock::now();

    const size_t num_prod   = automaton.total_product_states();
    const size_t num_cells  = automaton.num_cells;
    const size_t num_inputs = ts.num_inputs;

    SynthesisResult res;
    res.value_function.assign(num_prod, UNREACHABLE_COST);
    res.feedback_policy.assign(num_prod, nullopt);

    // Base case: if we are already at the goal (q4), we need 0 steps
    for (size_t s = 0; s < num_cells; ++s) {
        size_t target_idx = automaton.encode_product_index(s, GoalReached);
        res.value_function[target_idx] = 0.0;
    }

    size_t newly_controlled = 0;

    // Main loop: keep stepping backwards to discover all spots that can reach the goal
    for (size_t iter = 1; iter <= max_iters; ++iter) {
        newly_controlled = 0;
        vector<double> prev_v = res.value_function;

        // Work backwards through mission stages: q3 (did R2), q2 (did R1), then q1 (starting out)
        for (int q_val = 3; q_val >= 1; --q_val) {
            auto q = static_cast<AutomatonState>(q_val);

            for (size_t s = 0; s < num_cells; ++s) {
                size_t p = automaton.encode_product_index(s, q);

                // Skip if we already know how to reach the goal from here
                if (res.value_function[p] < UNREACHABLE_COST) {
                    continue;
                }

                // Obstacle cells are dangerous, so we never want to go there
                ObservationSymbol obs = automaton.get_cell_observation(s);
                if (obs == Obstacle) {
                    continue;
                }

                AutomatonState q_adv = automaton.dfa.transition(q, obs);
                if (q_adv == SinkTrap) {
                    continue;
                }

                // If stepping into this cell completes the current mission milestone (e.g. hitting R1, R2, or Goal)
                if (q_adv != q) {
                    size_t p_adv = automaton.encode_product_index(s, q_adv);
                    if (res.value_function[p_adv] < UNREACHABLE_COST) {
                        res.value_function[p] = static_cast<double>(iter);
                        res.feedback_policy[p] = res.feedback_policy[p_adv];
                        ++newly_controlled;
                        continue;
                    }
                }

                // Check every possible move the robot could take (speed and turn)
                for (size_t u = 0; u < num_inputs; ++u) {
                    const auto& succs = ts.get_successors(s, u);
                    if (succs.empty()) {
                        continue;
                    }

                    // We need a guarantee: every possible cell noise could push us into must still lead to the goal
                    bool all_safe = true;

                    for (size_t succ_s : succs) {
                        ObservationSymbol succ_obs = automaton.get_cell_observation(succ_s);
                        AutomatonState succ_q = automaton.dfa.transition(q_adv, succ_obs);

                        if (succ_q == SinkTrap) {
                            all_safe = false;
                            break;
                        }

                        size_t succ_p = automaton.encode_product_index(succ_s, succ_q);
                        if (prev_v[succ_p] >= UNREACHABLE_COST) {
                            all_safe = false;
                            break;
                        }
                    }

                    // If every outcome is safe and on track to win, record this as our chosen move!
                    if (all_safe) {
                        res.value_function[p] = static_cast<double>(iter);
                        res.feedback_policy[p] = u;
                        ++newly_controlled;
                        break;
                    }
                }
            }
        }

        res.iterations_completed = iter;

        size_t q1_cnt = 0, q2_cnt = 0, q3_cnt = 0, q4_cnt = 0;
        for (size_t s = 0; s < num_cells; ++s) {
            if (res.value_function[automaton.encode_product_index(s, Initial)] < UNREACHABLE_COST) q1_cnt++;
            if (res.value_function[automaton.encode_product_index(s, VisitedWaypoint1)] < UNREACHABLE_COST) q2_cnt++;
            if (res.value_function[automaton.encode_product_index(s, VisitedWaypoint2)] < UNREACHABLE_COST) q3_cnt++;
            if (res.value_function[automaton.encode_product_index(s, GoalReached)] < UNREACHABLE_COST) q4_cnt++;
        }
        cout << "Iteration " << iter << ": newly controlled = " << newly_controlled
             << " (q1=" << q1_cnt << ", q2=" << q2_cnt << ", q3=" << q3_cnt << ", q4=" << q4_cnt << ")\n";

        // If we didn't discover any new winning spots this round, we are done!
        if (newly_controlled == 0) {
            res.has_converged = true;
            break;
        }
    }

    // Count how many spots on the map can successfully reach the goal
    size_t total_controllable = 0;
    for (size_t p = 0; p < num_prod; ++p) {
        if (res.value_function[p] < UNREACHABLE_COST) {
            ++total_controllable;
        }
    }

    auto t1 = high_resolution_clock::now();
    res.computation_duration_seconds = duration<double>(t1 - t0).count();
    res.controllable_state_count = total_controllable;
    res.controllable_state_percentage =
        100.0 * static_cast<double>(total_controllable) / static_cast<double>(num_prod);

    return res;
}

// Backward reachability solver operating on the full 1.5M state space.
// Supports multi-core parallelization via OpenMP (#if defined(_OPENMP)) or sequential execution.
int solve_reachability_backward_cpp(
    vector<float>&                   V,
    vector<uint8_t>&                 ctrl,
    const ::grid::RegionConfig&      regions,
    const ::grid::AbstractionTables& tables,
    function<void(int, float)>       callback,
    bool                             use_parallel)
{
    using namespace ::grid;
    using namespace ::automaton;

    uint8_t labels[GRID_CELLS_X][GRID_CELLS_Y];
    for (int ix = 0; ix < GRID_CELLS_X; ++ix) {
        double x = COORDINATE_MIN_X + (ix + 0.5) * CELL_WIDTH_DX;
        for (int iy = 0; iy < GRID_CELLS_Y; ++iy) {
            double y = COORDINATE_MIN_Y + (iy + 0.5) * CELL_WIDTH_DY;
            labels[ix][iy] = static_cast<uint8_t>(get_region_label(x, y, regions));
        }
    }

    int iter = 0;
    while (iter < 100) {
        ++iter;
        vector<float> prev_v = V;
        int newly_controlled = 0;

        auto process_row = [&](int ix) -> int {
            int row_updates = 0;
            for (int iy = 0; iy < GRID_CELLS_Y; ++iy) {
                int obs = labels[ix][iy];

                for (int ith = 0; ith < GRID_CELLS_THETA; ++ith) {
                    int s = flat_index(ix, iy, ith);

                    for (int q = 1; q <= 5; ++q) {
                        int p = product_index(q, s);

                        if (V[p] < 1e8f) {
                            continue;
                        }

                        int q_next = dfa_transition(q, obs);
                        int p_next = product_index(q_next, s);

                        if (ctrl[p_next] != 0) {
                            V[p] = static_cast<float>(iter);
                            ++row_updates;
                            continue;
                        }

                        uint8_t best_u = 0;

                        for (int u = 0; u < 15; ++u) {
                            if (!tables.get_valid1(ix, ith, u) ||
                                !tables.get_valid2(iy, ith, u))
                            {
                                continue;
                            }

                            int sx_min  = tables.get_c1_min(ix, ith, u);
                            int sx_max  = tables.get_c1_max(ix, ith, u);
                            int sy_min  = tables.get_c2_min(iy, ith, u);
                            int sy_max  = tables.get_c2_max(iy, ith, u);
                            int sth_min = tables.get_c3_min(ith, u);
                            int sth_max = tables.get_c3_max(ith, u);

                            int q_base = (q_next - 1) * TOTAL_SPATIAL_CELLS;
                            bool all_safe = true;

                            if (sth_min <= sth_max) {
                                for (int sx = sx_min; sx <= sx_max && all_safe; ++sx) {
                                    int x_stride = q_base + sx * (GRID_CELLS_Y * GRID_CELLS_THETA);
                                    for (int sy = sy_min; sy <= sy_max && all_safe; ++sy) {
                                        int xy_stride = x_stride + sy * GRID_CELLS_THETA;
                                        for (int sth = sth_min; sth <= sth_max; ++sth) {
                                            if (prev_v[xy_stride + sth] > 1e7f) {
                                                all_safe = false;
                                                break;
                                            }
                                        }
                                    }
                                }
                            } else {
                                for (int sx = sx_min; sx <= sx_max && all_safe; ++sx) {
                                    int x_stride = q_base + sx * (GRID_CELLS_Y * GRID_CELLS_THETA);
                                    for (int sy = sy_min; sy <= sy_max && all_safe; ++sy) {
                                        int xy_stride = x_stride + sy * GRID_CELLS_THETA;
                                        for (int sth = sth_min; sth < GRID_CELLS_THETA && all_safe; ++sth) {
                                            if (prev_v[xy_stride + sth] > 1e7f) {
                                                all_safe = false;
                                                break;
                                            }
                                        }
                                        for (int sth = 0; sth <= sth_max && all_safe; ++sth) {
                                            if (prev_v[xy_stride + sth] > 1e7f) {
                                                all_safe = false;
                                                break;
                                            }
                                        }
                                    }
                                }
                            }

                            if (all_safe) {
                                best_u = static_cast<uint8_t>(u + 1);
                                break;
                            }
                        }

                        if (best_u != 0) {
                            V[p] = static_cast<float>(iter);
                            ctrl[p] = best_u;
                            ++row_updates;
                        }
                    }
                }
            }
            return row_updates;
        };

#if defined(_OPENMP)
        if (use_parallel) {
            #pragma omp parallel for reduction(+:newly_controlled) schedule(static)
            for (int ix = 0; ix < GRID_CELLS_X; ++ix) {
                newly_controlled += process_row(ix);
            }
        } else
#endif
        {
            for (int ix = 0; ix < GRID_CELLS_X; ++ix) {
                newly_controlled += process_row(ix);
            }
        }

        if (callback) {
            // When running sequentially, add a 50ms pacing delay so the user can watch the progress bar;
            // In parallel mode, no artificial delay is needed (runs at full multi-core speed)
            if (!use_parallel) {
                this_thread::sleep_for(chrono::milliseconds(50));
            }
            callback(iter, static_cast<float>(newly_controlled));
        }

        if (newly_controlled == 0) {
            break;
        }
    }

    return iter;
}

// Saves the synthesized controller table to disk in binary format
bool save_controller_binary(const char*            file_path,
                            const vector<float>&   V,
                            const vector<uint8_t>& ctrl)
{
    ofstream out(file_path, ios::binary);
    if (!out) return false;
    out.write("CTRL1.00", 8);
    uint32_t total = static_cast<uint32_t>(::automaton::NTOTAL);
    out.write(reinterpret_cast<const char*>(&total), 4);
    out.write(reinterpret_cast<const char*>(V.data()), total * sizeof(float));
    out.write(reinterpret_cast<const char*>(ctrl.data()), total * sizeof(uint8_t));
    return out.good();
}

// Loads a precomputed controller table from disk
bool load_controller_binary(const char*       file_path,
                            vector<float>&    V,
                            vector<uint8_t>&  ctrl)
{
    ifstream in(file_path, ios::binary);
    if (!in) return false;
    char magic[8];
    in.read(magic, 8);
    if (memcmp(magic, "CTRL1.00", 8) != 0) return false;
    uint32_t total = 0;
    in.read(reinterpret_cast<char*>(&total), 4);
    if (total != static_cast<uint32_t>(::automaton::NTOTAL)) return false;
    V.resize(total);
    ctrl.resize(total);
    in.read(reinterpret_cast<char*>(V.data()), total * sizeof(float));
    in.read(reinterpret_cast<char*>(ctrl.data()), total * sizeof(uint8_t));
    return in.good();
}

} // namespace symbolic::synthesis
