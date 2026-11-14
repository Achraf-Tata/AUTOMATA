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

// synthesis output containing the value function, feedback policy, and solve stats
struct SynthesisResult {
    bool has_converged = false;
    size_t iterations_completed = 0;
    size_t controllable_state_count = 0;
    double controllable_state_percentage = 0.0;
    double computation_duration_seconds = 0.0;

    // policy[product_state] = optimal control action index (1..15)
    vector<optional<size_t>> feedback_policy;
    // value[product_state] = minimum steps to reach goal
    vector<double> value_function;
};

// backward reachability fixed-point solver: works backwards from goal to find winning policy
class BackwardReachabilitySolver {
public:
    static constexpr double UNREACHABLE_COST = 1e9;

    // default constructor
    BackwardReachabilitySolver() = default;

    // synthesize feedback policy over product automaton: find all winning states satisfying the mission
    [[nodiscard]] SynthesisResult solve_product(
        const TransitionSystem& transition_system,
        const ProductAutomaton& product_automaton,
        size_t maximum_iterations = 100);

    // baseline reachability on spatial grid alone (simple reach-avoid without DFA specification)
    [[nodiscard]] SynthesisResult solve_spatial(
        const TransitionSystem& transition_system,
        const unordered_set<size_t>& target_spatial_states,
        const unordered_set<size_t>& obstacle_spatial_states,
        size_t maximum_iterations = 100);
};

} // namespace symbolic::synthesis

namespace symbolic {
    using synthesis::SynthesisResult;
    using synthesis::BackwardReachabilitySolver;
}
