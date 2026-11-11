#include "backward_reachability.hpp"
#include <algorithm>
#include <chrono>

namespace symbolic::synthesis {

SynthesisResult BackwardReachabilitySolver::solve(
    const abstraction::TransitionSystem& transition_system,
    const std::unordered_set<std::size_t>& target_state_indices,
    const std::unordered_set<std::size_t>& obstacle_state_indices,
    std::size_t maximum_iterations)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    SynthesisResult result;
    result.feedback_policy.assign(transition_system.state_count, std::nullopt);
    result.value_function.assign(transition_system.state_count, UNREACHABLE_COST);

    // Initialize target states with 0 steps
    for (std::size_t target_state : target_state_indices) {
        if (obstacle_state_indices.find(target_state) == obstacle_state_indices.end() &&
            target_state < transition_system.state_count)
        {
            result.value_function[target_state] = 0.0;
        }
    }

    std::size_t current_iteration = 0;
    while (current_iteration < maximum_iterations) {
        ++current_iteration;
        bool has_policy_changed = false;
        std::vector<double> previous_iteration_values = result.value_function;

        for (std::size_t state_index = 0; state_index < transition_system.state_count; ++state_index) {
            // Skip already converged states
            if (result.value_function[state_index] < UNREACHABLE_COST * 0.5) {
                continue;
            }
            // Skip obstacle states
            if (obstacle_state_indices.find(state_index) != obstacle_state_indices.end()) {
                continue;
            }

            // Check if any discrete control action guarantees reaching the winning set
            for (std::size_t control_index = 0; control_index < transition_system.input_count; ++control_index) {
                const auto& successor_states = transition_system.get_successors(state_index, control_index);
                if (successor_states.empty()) {
                    continue;
                }

                bool is_control_action_safe_and_progressing = true;
                double maximum_successor_cost = 0.0;

                for (std::size_t successor_state : successor_states) {
                    if (obstacle_state_indices.find(successor_state) != obstacle_state_indices.end() ||
                        previous_iteration_values[successor_state] >= UNREACHABLE_COST * 0.5)
                    {
                        is_control_action_safe_and_progressing = false;
                        break;
                    }
                    maximum_successor_cost = std::max(maximum_successor_cost, previous_iteration_values[successor_state]);
                }

                if (is_control_action_safe_and_progressing) {
                    result.value_function[state_index] = maximum_successor_cost + 1.0;
                    result.feedback_policy[state_index] = control_index;
                    has_policy_changed = true;
                    break;
                }
            }
        }

        if (!has_policy_changed) {
            result.has_converged = true;
            break;
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    result.computation_duration_seconds = std::chrono::duration<double>(end_time - start_time).count();
    result.iterations_completed = current_iteration;

    std::size_t total_controllable = 0;
    for (std::size_t state_index = 0; state_index < transition_system.state_count; ++state_index) {
        if (result.value_function[state_index] < UNREACHABLE_COST * 0.5) {
            ++total_controllable;
        }
    }
    result.controllable_state_count = total_controllable;
    result.controllable_state_percentage = (transition_system.state_count > 0)
        ? (100.0 * static_cast<double>(total_controllable) / static_cast<double>(transition_system.state_count))
        : 0.0;

    return result;
}

} // namespace symbolic::synthesis
