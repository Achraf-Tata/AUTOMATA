#pragma once

#include "contract_abstraction.hpp"
#include <vector>
#include <array>
#include <memory>
#include <cstdint>

namespace circular::synthesis {

using namespace abstraction;

// Results container matching Slide 17 of 42717210-s7467.pdf
struct SynthesisResult {
    int    N                    = 0;
    double T_abs_ms             = 0.0;    // Abstraction time Tabs (ms)
    double T_ctrl_ms            = 0.0;    // Synthesis time Tctrl (ms)
    double peak_mem_kb          = 0.0;    // Peak memory consumption (KB)
    double controllable_pct     = 0.0;    // Percentage of controllable states in S (100.0%)
    int    iterations           = 0;      // Fixed-point iterations
    bool   composition_verified = false;  // Saoud-Girard-Fribourg composition theorem
    int    controllable_count   = 0;

    std::array<int, ND_CELLS + 1>              local_policy;
    std::vector<std::array<int, ND_CELLS + 1>> multi_controllers;
};

// Abstract strategy interface for safety synthesis
class ISynthesisSolver {
public:
    virtual ~ISynthesisSolver() = default;
    [[nodiscard]] virtual SynthesisResult synthesize(int N, bool use_parallel) = 0;
};

// Compositional Assume-Guarantee contract solver (Slides 15-17)
class CompositionalContractSolver : public ISynthesisSolver {
public:
    [[nodiscard]] SynthesisResult synthesize(int N, bool use_parallel) override;

    [[nodiscard]] static std::vector<bool> solve_fixed_point(
        const TransitionTable& trans,
        int& iterations);

    [[nodiscard]] static std::vector<bool> pre_operator(
        const std::vector<bool>& in_R,
        const TransitionTable& trans);

    [[nodiscard]] static std::array<int, ND_CELLS + 1> extract_policy(
        const std::vector<bool>& R_star,
        const TransitionTable& trans);

    [[nodiscard]] static bool verify_composition(const std::vector<bool>& R_star);
};

// Evaluator for monolithic state-space explosion (Slide 16)
class MonolithicCurseEvaluator {
public:
    static void print_curse_benchmark(int max_N);
};

// Binary persistence for synthesized controller
class ControllerSerializer {
public:
    static constexpr uint32_t MAGIC_HEADER = 0x434D4243; // 'C' 'B' 'M' 'C'
    static constexpr uint32_t VERSION      = 1;

    static bool save_binary(const char* file_path, const std::array<int, ND_CELLS + 1>& policy);
    static bool load_binary(const char* file_path, std::array<int, ND_CELLS + 1>& policy);
};

} // namespace circular::synthesis
