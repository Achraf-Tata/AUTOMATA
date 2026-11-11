#pragma once

#include "transition_system.hpp"
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

    // feedback_policy[state_index] = optimal discrete control index, or empty if uncontrollable
    std::vector<std::optional<std::size_t>> feedback_policy;
    // value_function[state_index] = minimum steps to reach target, or infinity
    std::vector<double> value_function;
};

class BackwardReachabilitySolver {
public:
    static constexpr double UNREACHABLE_COST = 1e9;

    BackwardReachabilitySolver() = default;

    [[nodiscard]] SynthesisResult solve(
        const abstraction::TransitionSystem& transition_system,
        const std::unordered_set<std::size_t>& target_state_indices,
        const std::unordered_set<std::size_t>& obstacle_state_indices,
        std::size_t maximum_iterations = 100);
};

} // namespace symbolic::synthesis
