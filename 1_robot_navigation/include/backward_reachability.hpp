#pragma once

#include "transition_system.hpp"
#include "product_automaton.hpp"
#include <vector>
#include <unordered_set>
#include <optional>
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

    // Direct navigation on the spatial grid: reach target cells while avoiding obstacle cells
    [[nodiscard]] SynthesisResult solve_spatial(
        const TransitionSystem& ts,
        const unordered_set<size_t>& targets,
        const unordered_set<size_t>& obstacles,
        size_t max_iters = 100);
};

} // namespace symbolic::synthesis

namespace symbolic {
    using synthesis::SynthesisResult;
    using synthesis::BackwardReachabilitySolver;
}
