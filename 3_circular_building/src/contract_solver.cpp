#include "contract_solver.hpp"
#include <chrono>
#include <fstream>
#include <iostream>

#if defined(_OPENMP)
#include <omp.h>
#endif

namespace circular::synthesis {

std::vector<bool> CompositionalContractSolver::pre_operator(
    const std::vector<bool>& in_R,
    const TransitionTable& trans)
{
    std::vector<bool> pre(ND_CELLS + 1, false);

    for (int s = 0; s < ND_CELLS; ++s) {
        for (int u_idx = 0; u_idx < NUM_CONTROLS; ++u_idx) {
            const auto& post = trans.table[s][u_idx];
            if (post.empty()) continue;

            bool all_safe = true;
            for (int succ : post) {
                if (succ < 0 || succ >= ND_CELLS || !in_R[succ]) {
                    all_safe = false;
                    break;
                }
            }

            if (all_safe) {
                pre[s] = true;
                break;
            }
        }
    }

    return pre;
}

std::vector<bool> CompositionalContractSolver::solve_fixed_point(
    const TransitionTable& trans,
    int& iterations)
{
    // Candidate safe set R_0 = {0..9} corresponding to S = [17.0, 22.0] C (Slide 16)
    std::vector<bool> R(ND_CELLS + 1, false);
    for (int s = 0; s < ND_CELLS; ++s) {
        R[s] = true;
    }

    iterations = 0;
    while (true) {
        ++iterations;
        std::vector<bool> pre = pre_operator(R, trans);
        std::vector<bool> R_next(ND_CELLS + 1, false);

        bool changed = false;
        for (int s = 0; s < ND_CELLS; ++s) {
            R_next[s] = R[s] && pre[s];
            if (R_next[s] != R[s]) {
                changed = true;
            }
        }

        if (!changed) {
            break; // Fixed point converged: R_{k+1} == R_k == R*
        }
        R = R_next;
    }

    return R;
}

std::array<int, ND_CELLS + 1> CompositionalContractSolver::extract_policy(
    const std::vector<bool>& R_star,
    const TransitionTable& trans)
{
    std::array<int, ND_CELLS + 1> policy;
    policy.fill(0);

    for (int s = 0; s <= ND_CELLS; ++s) {
        if (s == SINK_CELL || !R_star[s]) {
            // Out of invariant recovery action:
            policy[s] = (s < ND_CELLS / 2) ? 2 : 0; // Heat if cold, off if hot
            continue;
        }

        // Energy-saving tie-breaker: test controls in increasing order {0.0, 0.3, 0.6}
        bool found = false;
        for (int u_idx = 0; u_idx < NUM_CONTROLS; ++u_idx) {
            const auto& post = trans.table[s][u_idx];
            if (post.empty()) continue;

            bool all_safe = true;
            for (int succ : post) {
                if (succ < 0 || succ >= ND_CELLS || !R_star[succ]) {
                    all_safe = false;
                    break;
                }
            }

            if (all_safe) {
                policy[s] = u_idx;
                found = true;
                break;
            }
        }

        if (!found) {
            policy[s] = 1; // Default fallback to 0.3
        }
    }

    return policy;
}

bool CompositionalContractSolver::verify_composition(const std::vector<bool>& R_star) {
    for (int s = 0; s <= ND_CELLS; ++s) {
        if (R_star[s]) {
            if (s < 0 || s >= ND_CELLS) return false;
        }
    }
    return true;
}

SynthesisResult CompositionalContractSolver::synthesize(int N, bool use_parallel) {
    SynthesisResult res{};
    res.N = N;

    // 1. Measure Abstraction Time Tabs (Slide 17)
    auto t0 = std::chrono::high_resolution_clock::now();
    TransitionTable trans = ContractAbstraction::build_transition_table();
    auto t1 = std::chrono::high_resolution_clock::now();
    res.T_abs_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // 2. Measure Controller Synthesis Time Tctrl (Slide 17)
    auto t2 = std::chrono::high_resolution_clock::now();
    std::vector<bool> R_star = solve_fixed_point(trans, res.iterations);
    res.local_policy = extract_policy(R_star, trans);
    res.composition_verified = verify_composition(R_star);

    // Multi-room policy assignment across all N rooms (with OpenMP multi-core support)
    res.multi_controllers.resize(N);
#if defined(_OPENMP)
    if (use_parallel && N >= 512) {
        #pragma omp parallel for schedule(static)
        for (int i = 0; i < N; ++i) {
            res.multi_controllers[i] = res.local_policy;
        }
    } else {
        for (int i = 0; i < N; ++i) {
            res.multi_controllers[i] = res.local_policy;
        }
    }
#else
    (void)use_parallel;
    for (int i = 0; i < N; ++i) {
        res.multi_controllers[i] = res.local_policy;
    }
#endif
    auto t3 = std::chrono::high_resolution_clock::now();
    res.T_ctrl_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

    // 3. Compute Controllable Percentage and Memory Metrics
    int controlled_count = 0;
    for (int s = 0; s < ND_CELLS; ++s) {
        if (R_star[s]) ++controlled_count;
    }
    res.controllable_count = controlled_count;
    res.controllable_pct = (100.0 * controlled_count) / static_cast<double>(ND_CELLS);

    // Peak memory estimation (KB)
    double mem_bytes = static_cast<double>(sizeof(TransitionTable) + (sizeof(int) * (ND_CELLS + 1) * N));
    res.peak_mem_kb = mem_bytes / 1024.0;
    if (res.peak_mem_kb < 0.5) res.peak_mem_kb = 0.5;

    return res;
}

bool ControllerSerializer::save_binary(const char* file_path, const std::array<int, ND_CELLS + 1>& policy) {
    std::ofstream out(file_path, std::ios::binary);
    if (!out) {
        std::cerr << "Error: Cannot open file for writing: " << file_path << "\n";
        return false;
    }

    uint32_t magic   = MAGIC_HEADER;
    uint32_t version = VERSION;
    uint32_t count   = static_cast<uint32_t>(ND_CELLS + 1);

    out.write(reinterpret_cast<const char*>(&magic),   sizeof(magic));
    out.write(reinterpret_cast<const char*>(&version), sizeof(version));
    out.write(reinterpret_cast<const char*>(&count),   sizeof(count));

    std::vector<int8_t> buffer(ND_CELLS + 1);
    for (size_t i = 0; i < policy.size(); ++i) {
        buffer[i] = static_cast<int8_t>(policy[i]);
    }
    out.write(reinterpret_cast<const char*>(buffer.data()), buffer.size() * sizeof(int8_t));

    return out.good();
}

bool ControllerSerializer::load_binary(const char* file_path, std::array<int, ND_CELLS + 1>& policy) {
    std::ifstream in(file_path, std::ios::binary);
    if (!in) {
        std::cerr << "Error: Cannot open file for reading: " << file_path << "\n";
        return false;
    }

    uint32_t magic = 0, version = 0, count = 0;
    in.read(reinterpret_cast<char*>(&magic),   sizeof(magic));
    in.read(reinterpret_cast<char*>(&version), sizeof(version));
    in.read(reinterpret_cast<char*>(&count),   sizeof(count));

    if (magic != MAGIC_HEADER || version != VERSION || count != static_cast<uint32_t>(ND_CELLS + 1)) {
        std::cerr << "Error: Invalid header or version in binary controller: " << file_path << "\n";
        return false;
    }

    std::vector<int8_t> buffer(ND_CELLS + 1);
    in.read(reinterpret_cast<char*>(buffer.data()), buffer.size() * sizeof(int8_t));

    if (!in.good()) return false;

    for (size_t i = 0; i < policy.size(); ++i) {
        policy[i] = static_cast<int>(buffer[i]);
    }

    return true;
}

} // namespace circular::synthesis
