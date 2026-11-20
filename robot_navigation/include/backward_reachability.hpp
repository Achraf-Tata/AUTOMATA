#pragma once

#include "transition_system.hpp"
#include "product_automaton.hpp"
#include "grid_quantizer.hpp"
#include <vector>
#include <optional>
#include <functional>
#include <cstddef>

namespace symbolic::synthesis {

using namespace std;
using namespace abstraction;
using namespace automaton;

// Results holding the step counts to goal, chosen moves for each cell, and search stats
struct SynthesisResult {
    bool has_converged = false;
    size_t iterations_completed = 0;
    size_t controllable_state_count = 0;
    double controllable_state_percentage = 0.0;
    double computation_duration_seconds = 0.0;

    // The move to pick for each state (stores the chosen action index 1..15)
    vector<optional<size_t>> feedback_policy;
    // Minimum number of steps needed to reach the goal from here (or unreachable)
    vector<double> value_function;
};

// Works backwards step-by-step from the goal to find winning moves for the robot
class BackwardReachabilitySolver {
public:
    static constexpr double UNREACHABLE_COST = 1e9;

    // Default constructor
    BackwardReachabilitySolver() = default;

    // Computes the best moves across the combined tracking graph so the robot completes the mission
    [[nodiscard]] SynthesisResult solve_product(
        const TransitionSystem& ts,
        const ProductAutomaton& automaton,
        size_t max_iters = 100);
};

// Backward reachability solver operating on the full 1.5M state space.
// Supports multi-core parallelization via OpenMP (#if defined(_OPENMP)) or sequential execution.
int solve_reachability_backward_cpp(
    std::vector<float>&              V,
    std::vector<uint8_t>&            ctrl,
    const ::grid::RegionConfig&      regions,
    const ::grid::AbstractionTables& tables,
    std::function<void(int, float)>  callback = nullptr,
    bool                             use_parallel = true);

// Saves the synthesized value function and controller policy to a binary file.
bool save_controller_binary(const char*                  file_path,
                            const std::vector<float>&   V,
                            const std::vector<uint8_t>& ctrl);

// Loads a previously computed controller from a binary file.
bool load_controller_binary(const char*             file_path,
                            std::vector<float>&    V,
                            std::vector<uint8_t>&  ctrl);

} // namespace symbolic::synthesis

namespace solver {
    using symbolic::synthesis::solve_reachability_backward_cpp;
    using symbolic::synthesis::save_controller_binary;
    using symbolic::synthesis::load_controller_binary;
}

namespace symbolic {
    using synthesis::SynthesisResult;
    using synthesis::BackwardReachabilitySolver;
    using synthesis::solve_reachability_backward_cpp;
    using synthesis::save_controller_binary;
    using synthesis::load_controller_binary;
}
