#include "backward_reachability.hpp"
#include <algorithm>
#include <chrono>

namespace symbolic::synthesis {

SynthesisResult BackwardReachabilitySolver::solve(
    const abstraction::TransitionSystem& ts,
    const std::unordered_set<std::size_t>& target_states,
    const std::unordered_set<std::size_t>& obstacle_states,
    std::size_t max_iterations)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    SynthesisResult result;
    result.controller.assign(ts.num_states, std::nullopt);
    result.value_function.assign(ts.num_states, INF_COST);

    // Initialize target states with 0 steps
    for (std::size_t target : target_states) {
        if (obstacle_states.find(target) == obstacle_states.end() && target < ts.num_states) {
            result.value_function[target] = 0.0;
        }
    }

    std::size_t iter = 0;
    while (iter < max_iterations) {
        ++iter;
        bool has_changes = false;
        std::vector<double> prev_values = result.value_function;

        for (std::size_t q = 0; q < ts.num_states; ++q) {
            // Skip already converged states
            if (result.value_function[q] < INF_COST * 0.5) {
                continue;
            }
            // Skip obstacle states
            if (obstacle_states.find(q) != obstacle_states.end()) {
                continue;
            }

            // Check if any discrete control action guarantees reaching the winning set
            for (std::size_t u = 0; u < ts.num_inputs; ++u) {
                const auto& post = ts.get_post(q, u);
                if (post.empty()) {
                    continue;
                }

                bool valid_action = true;
                double max_post_value = 0.0;

                for (std::size_t next_q : post) {
                    if (obstacle_states.find(next_q) != obstacle_states.end() ||
                        prev_values[next_q] >= INF_COST * 0.5) {
                        valid_action = false;
                        break;
                    }
                    max_post_value = std::max(max_post_value, prev_values[next_q]);
                }

                if (valid_action) {
                    result.value_function[q] = max_post_value + 1.0;
                    result.controller[q] = u;
                    has_changes = true;
                    break;
                }
            }
        }

        if (!has_changes) {
            result.converged = true;
            break;
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    result.duration_seconds = std::chrono::duration<double>(end_time - start_time).count();
    result.iterations = iter;

    std::size_t win_count = 0;
    for (std::size_t q = 0; q < ts.num_states; ++q) {
        if (result.value_function[q] < INF_COST * 0.5) {
            ++win_count;
        }
    }
    result.winning_set_size = win_count;
    result.controllable_percentage = (ts.num_states > 0)
        ? (100.0 * static_cast<double>(win_count) / static_cast<double>(ts.num_states))
        : 0.0;

    return result;
}

} // namespace symbolic::synthesis
