#include "backward_reachability.hpp"
#include <chrono>
#include <thread>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <cstring>
#include <filesystem>
#include <cmath>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

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

                if (automaton.cell_observation_masks[s] & (1u << 4)) {
                    continue;
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
                        for (int label = 0; label < 5; ++label) {
                            if (!(automaton.cell_observation_masks[succ_s] & (1u << label))) continue;
                            const auto next_q = automaton.dfa.transition(q, static_cast<ObservationSymbol>(label));
                            const size_t next = automaton.encode_product_index(succ_s, next_q);
                            if (prev_v[next] >= UNREACHABLE_COST) {
                                all_safe = false;
                                break;
                            }
                        }
                        if (!all_safe) break;
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
    vector<float>& V,
    vector<uint8_t>& ctrl,
    const ::grid::RegionConfig& regions,
    const ::grid::AbstractionTables& tables,
    function<void(int, float)> callback,
    bool use_parallel,
    function<bool()> cancelled)
{
    using namespace ::grid;
    uint8_t observations[NX][NY];
    for (int ix = 0; ix < NX; ++ix) {
        for (int iy = 0; iy < NY; ++iy) {
            const double x = X_MIN + ix * DX;
            const double y = Y_MIN + iy * DY;
            observations[ix][iy] = get_cell_observation_mask(x, x + DX, y, y + DY, regions);
        }
    }

    for (int iteration = 1; iteration <= 100; ++iteration) {
        if (cancelled && cancelled()) return -1;
        const vector<float> previous = V;
        int updates = 0;
        auto process_row = [&](int ix) {
            int row_updates = 0;
            for (int iy = 0; iy < NY; ++iy) {
                if (observations[ix][iy] & (1u << 4)) continue;
                for (int heading = 0; heading < NTHETA; ++heading) {
                    const int cell = flat_index(ix, iy, heading);
                    // Memory already includes the current observation, as in equation (14).
                    for (int memory = 1; memory <= 3; ++memory) {
                        const int state = ::automaton::product_index(memory, cell);
                        if (previous[state] < ::automaton::INF) continue;
                        for (int input = 0; input < 15; ++input) {
                            if (!tables.get_valid1(ix, heading, input) ||
                                !tables.get_valid2(iy, heading, input)) continue;
                            const int first_heading = tables.get_c3_min(heading, input);
                            const int last_heading = tables.get_c3_max(heading, input);
                            const int heading_count = (last_heading - first_heading + NTHETA) % NTHETA + 1;
                            bool winning = true;
                            for (int sx = tables.get_c1_min(ix, heading, input);
                                 sx <= tables.get_c1_max(ix, heading, input) && winning; ++sx) {
                                for (int sy = tables.get_c2_min(iy, heading, input);
                                     sy <= tables.get_c2_max(iy, heading, input) && winning; ++sy) {
                                    for (int label = 1; label <= 5 && winning; ++label) {
                                        if (!(observations[sx][sy] & (1u << (label - 1)))) continue;
                                        const int next_memory = ::automaton::dfa_transition(memory, label);
                                        for (int offset = 0; offset < heading_count; ++offset) {
                                            const int next_heading = (first_heading + offset) % NTHETA;
                                            const int next = ::automaton::product_index(
                                                next_memory, flat_index(sx, sy, next_heading));
                                            if (previous[next] >= ::automaton::INF) {
                                                winning = false;
                                                break;
                                            }
                                        }
                                    }
                                }
                            }
                            if (winning) {
                                V[state] = static_cast<float>(iteration);
                                ctrl[state] = static_cast<uint8_t>(input + 1);
                                ++row_updates;
                                break;
                            }
                        }
                    }
                }
            }
            return row_updates;
        };
#if defined(_OPENMP)
        #pragma omp parallel for reduction(+:updates) schedule(static) if(use_parallel)
#endif
        for (int ix = 0; ix < NX; ++ix) updates += process_row(ix);
        if (callback) callback(iteration, static_cast<float>(updates));
        if (updates == 0) return iteration;
    }
    return 100;
}

namespace {
vector<double> controller_configuration(const ::grid::RegionConfig& r) {
    vector<double> configuration = {
        ::grid::NX, ::grid::NY, ::grid::NTHETA,
        ::grid::X_MIN, ::grid::X_MAX, ::grid::Y_MIN, ::grid::Y_MAX,
        ::grid::TH_MIN, ::grid::TH_MAX, 1.0, 0.05,
        r.r1_x0, r.r1_x1, r.r1_y0, r.r1_y1,
        r.r2_x0, r.r2_x1, r.r2_y0, r.r2_y1,
        r.r3_x0, r.r3_x1, r.r3_y0, r.r3_y1,
        r.r4_x0, r.r4_x1, r.r4_y0, r.r4_y1
    };
    for (const auto& input : ::unicycle::ALL_CONTROLS) {
        configuration.push_back(input.v);
        configuration.push_back(input.w);
    }
    return configuration;
}

uint64_t controller_checksum(const vector<float>& values, const vector<uint8_t>& controls) {
    uint64_t hash = 14695981039346656037ull;
    auto add_bytes = [&hash](const unsigned char* bytes, size_t count) {
        for (size_t i = 0; i < count; ++i) {
            hash ^= bytes[i];
            hash *= 1099511628211ull;
        }
    };
    add_bytes(reinterpret_cast<const unsigned char*>(values.data()), values.size() * sizeof(float));
    add_bytes(controls.data(), controls.size());
    return hash;
}
}

bool save_controller_binary(const char* file_path, const vector<float>& values,
                            const vector<uint8_t>& controls, const ::grid::RegionConfig& regions) {
    if (values.size() != ::automaton::NTOTAL || controls.size() != values.size()) return false;
    const auto configuration = controller_configuration(regions);
    const uint32_t count = static_cast<uint32_t>(values.size());
    const uint64_t checksum = controller_checksum(values, controls);
    const filesystem::path destination(file_path);
    auto temporary = destination;
    temporary += ".tmp." + to_string(steady_clock::now().time_since_epoch().count());
    ofstream output(temporary, ios::binary | ios::trunc);
    if (!output) return false;
    output.write("CTRL2.00", 8);
    output.write(reinterpret_cast<const char*>(&count), sizeof(count));
    output.write(reinterpret_cast<const char*>(configuration.data()), configuration.size() * sizeof(double));
    output.write(reinterpret_cast<const char*>(&checksum), sizeof(checksum));
    output.write(reinterpret_cast<const char*>(values.data()), values.size() * sizeof(float));
    output.write(reinterpret_cast<const char*>(controls.data()), controls.size());
    output.close();
    bool replaced = false;
    if (output.good()) {
#ifdef _WIN32
        replaced = MoveFileExW(temporary.c_str(), destination.c_str(),
                               MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0;
#else
        error_code error;
        filesystem::rename(temporary, destination, error);
        replaced = !error;
#endif
    }
    if (!replaced) {
        error_code ignored;
        filesystem::remove(temporary, ignored);
    }
    return replaced;
}

bool load_controller_binary(const char* file_path, vector<float>& values,
                            vector<uint8_t>& controls, const ::grid::RegionConfig& regions) {
    ifstream input(file_path, ios::binary);
    char magic[8]{};
    uint32_t count = 0;
    input.read(magic, sizeof(magic));
    input.read(reinterpret_cast<char*>(&count), sizeof(count));
    if (!input || memcmp(magic, "CTRL2.00", 8) != 0 || count != ::automaton::NTOTAL) return false;
    const auto expected = controller_configuration(regions);
    vector<double> configuration(expected.size());
    uint64_t checksum = 0;
    input.read(reinterpret_cast<char*>(configuration.data()), configuration.size() * sizeof(double));
    input.read(reinterpret_cast<char*>(&checksum), sizeof(checksum));
    if (!input || configuration != expected) return false;
    vector<float> loaded_values(count);
    vector<uint8_t> loaded_controls(count);
    input.read(reinterpret_cast<char*>(loaded_values.data()), count * sizeof(float));
    input.read(reinterpret_cast<char*>(loaded_controls.data()), count);
    if (!input || input.peek() != char_traits<char>::eof()) return false;
    if (checksum != controller_checksum(loaded_values, loaded_controls)) return false;
    for (size_t i = 0; i < count; ++i) {
        if (!isfinite(loaded_values[i]) || loaded_values[i] < 0 ||
            loaded_values[i] > ::automaton::INF || loaded_controls[i] > 15) return false;
    }
    values.swap(loaded_values);
    controls.swap(loaded_controls);
    return true;
}

} // namespace symbolic::synthesis
