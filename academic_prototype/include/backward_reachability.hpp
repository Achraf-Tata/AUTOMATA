#pragma once

#include "transition_system.hpp"
#include "product_automaton.hpp"
#include <vector>
#include <unordered_set>
#include <optional>
#include <cstddef>

namespace symbolic::synthesis {

struct SynthesisResult {
    bool has_converged = false;
    std::size_t iterations_completed = 0;
    std::size_t controllable_state_count = 0;
    double controllable_state_percentage = 0.0;
    double computation_duration_seconds = 0.0;

    // feedback_policy[product_state_index] = optimal control action index
    std::vector<std::optional<std::size_t>> feedback_policy;
    // value_function[product_state_index] = minimum steps to reach goal
    std::vector<double> value_function;
};

class BackwardReachabilitySolver {
public:
    static constexpr double UNREACHABLE_COST = 1e9;

    BackwardReachabilitySolver() = default;

    // Backward reachability synthesis over the Product Automaton (S x A)
    [[nodiscard]] SynthesisResult solve_product(
        const abstraction::TransitionSystem& transition_system,
        const automaton::ProductAutomaton& product_automaton,
        std::size_t maximum_iterations = 100);

    // Baseline reachability over standard spatial partition
    [[nodiscard]] SynthesisResult solve_spatial(
        const abstraction::TransitionSystem& transition_system,
        const std::unordered_set<std::size_t>& target_spatial_states,
        const std::unordered_set<std::size_t>& obstacle_spatial_states,
        std::size_t maximum_iterations = 100);
};

} // namespace symbolic::synthesis
