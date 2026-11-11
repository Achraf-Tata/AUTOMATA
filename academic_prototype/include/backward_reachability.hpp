#pragma once

#include "transition_system.hpp"
#include <vector>
#include <unordered_set>
#include <optional>
#include <cstddef>
#include <chrono>

namespace symbolic::synthesis {

struct SynthesisResult {
    bool converged = false;
    std::size_t iterations = 0;
    std::size_t winning_set_size = 0;
    double controllable_percentage = 0.0;
    double duration_seconds = 0.0;

    // controller[state] = optimal discrete input index, or empty if uncontrollable
    std::vector<std::optional<std::size_t>> controller;
    // value_function[state] = minimum steps to reach target, or infinity
    std::vector<double> value_function;
};

class BackwardReachabilitySolver {
public:
    static constexpr double INF_COST = 1e9;

    BackwardReachabilitySolver() = default;

    [[nodiscard]] SynthesisResult solve(
        const abstraction::TransitionSystem& ts,
        const std::unordered_set<std::size_t>& target_states,
        const std::unordered_set<std::size_t>& obstacle_states,
        std::size_t max_iterations = 100);
};

} // namespace symbolic::synthesis
