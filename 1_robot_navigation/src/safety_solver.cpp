#include "safety_solver.hpp"
#include <chrono>

namespace symbolic::synthesis {

using namespace std;
using namespace chrono;
using namespace abstraction;
using namespace automaton;

// Implements Algorithm 1: Maximal Controlled Invariant Set Computation (Safety Game)
// Goal: find the set of states S* from which there exists a feedback controller
// that keeps the robot inside the safe set forever (square Safe), despite any bounded disturbance.
SafetyResult SafetySolver::solve_safety(
    const TransitionSystem& transition_system,
    const unordered_set<size_t>& obstacle_states,
    size_t maximum_iterations)
{
    auto start_time = high_resolution_clock::now();
    SafetyResult result;

    size_t total_states = transition_system.state_count;
    size_t input_count = transition_system.input_count;

    // Step 1: Initialize S_0 = all states that are not inside obstacles
    vector<bool> current_safe(total_states, true);
    for (size_t obs_idx : obstacle_states) {
        if (obs_idx < total_states) {
            current_safe[obs_idx] = false;
        }
    }

    vector<optional<size_t>> current_policy(total_states, nullopt);

    // Step 2: Fixed-point pruning loop (Greatest Fixed Point: nu X. (Safe cap CPre(X)))
    size_t iteration = 0;
    while (iteration < maximum_iterations) {
        ++iteration;
        bool any_state_pruned = false;
        vector<bool> next_safe = current_safe;

        for (size_t state_index = 0; state_index < total_states; ++state_index) {
            if (!current_safe[state_index]) {
                continue; // Already pruned as unsafe
            }

            // Check if there exists at least one control u such that
            // ALL successors under disturbance are inside current_safe
            bool found_safe_control = false;
            for (size_t control_index = 0; control_index < input_count; ++control_index) {
                const auto& successors = transition_system.get_successors(state_index, control_index);
                if (successors.empty()) {
                    continue; // Control leaves domain bounds or has no valid transitions
                }

                bool all_successors_safe = true;
                for (size_t succ_index : successors) {
                    if (!current_safe[succ_index]) {
                        all_successors_safe = false;
                        break;
                    }
                }

                if (all_successors_safe) {
                    found_safe_control = true;
                    current_policy[state_index] = control_index;
                    break;
                }
            }

            // If no control can guarantee staying in current_safe under all disturbances, prune it!
            if (!found_safe_control) {
                next_safe[state_index] = false;
                current_policy[state_index] = nullopt;
                any_state_pruned = true;
            }
        }

        current_safe = move(next_safe);

        // Fixed-point convergence: no states were pruned in this iteration!
        if (!any_state_pruned) {
            result.has_converged = true;
            break;
        }
    }

    auto end_time = high_resolution_clock::now();
    result.computation_duration_seconds = duration<double>(end_time - start_time).count();
    result.iterations_completed = iteration;

    // Count remaining invariant states
    size_t safe_count = 0;
    for (size_t s = 0; s < total_states; ++s) {
        if (current_safe[s]) {
            ++safe_count;
        }
    }

    result.safe_state_count = safe_count;
    result.safe_state_percentage = total_states > 0 ? (100.0 * static_cast<double>(safe_count) / static_cast<double>(total_states)) : 0.0;
    result.invariant_set = move(current_safe);
    result.safe_policy = move(current_policy);

    return result;
}

// Computes safety invariance over the synchronous product automaton (avoiding DFA sink trap q5)
SafetyResult SafetySolver::solve_safety_product(
    const TransitionSystem& transition_system,
    const ProductAutomaton& product_automaton,
    size_t maximum_iterations)
{
    auto start_time = high_resolution_clock::now();
    SafetyResult result;

    size_t total_product_states = product_automaton.total_product_states();
    size_t spatial_count = product_automaton.spatial_cell_count;
    size_t input_count = transition_system.input_count;

    // Step 1: Initialize S_0: all product states whose DFA state is not the sink trap (q5)
    vector<bool> current_safe(total_product_states, true);
    for (size_t pidx = 0; pidx < total_product_states; ++pidx) {
        auto decoded = product_automaton.decode_product_index(pidx);
        if (product_automaton.specification_dfa.is_sink_state(decoded.automaton_state)) {
            current_safe[pidx] = false;
        }
    }

    vector<optional<size_t>> current_policy(total_product_states, nullopt);

    // Step 2: Fixed-point pruning
    size_t iteration = 0;
    while (iteration < maximum_iterations) {
        ++iteration;
        bool any_state_pruned = false;
        vector<bool> next_safe = current_safe;

        for (size_t pidx = 0; pidx < total_product_states; ++pidx) {
            if (!current_safe[pidx]) {
                continue;
            }

            auto decoded = product_automaton.decode_product_index(pidx);
            size_t cell_index = decoded.spatial_cell_index;
            auto current_dfa_state = decoded.automaton_state;

            bool found_safe_control = false;
            for (size_t control_index = 0; control_index < input_count; ++control_index) {
                const auto& successors = transition_system.get_successors(cell_index, control_index);
                if (successors.empty()) {
                    continue;
                }

                bool all_successors_safe = true;
                for (size_t succ_cell : successors) {
                    if (succ_cell >= spatial_count) {
                        all_successors_safe = false;
                        break;
                    }

                    ObservationSymbol obs = product_automaton.get_cell_observation(succ_cell);
                    auto next_dfa_state = product_automaton.specification_dfa.transition(current_dfa_state, obs);

                    size_t succ_pidx = product_automaton.encode_product_index(succ_cell, next_dfa_state);
                    if (succ_pidx >= total_product_states || !current_safe[succ_pidx]) {
                        all_successors_safe = false;
                        break;
                    }
                }

                if (all_successors_safe) {
                    found_safe_control = true;
                    current_policy[pidx] = control_index;
                    break;
                }
            }

            if (!found_safe_control) {
                next_safe[pidx] = false;
                current_policy[pidx] = nullopt;
                any_state_pruned = true;
            }
        }

        current_safe = move(next_safe);

        if (!any_state_pruned) {
            result.has_converged = true;
            break;
        }
    }

    auto end_time = high_resolution_clock::now();
    result.computation_duration_seconds = duration<double>(end_time - start_time).count();
    result.iterations_completed = iteration;

    size_t safe_count = 0;
    for (size_t p = 0; p < total_product_states; ++p) {
        if (current_safe[p]) {
            ++safe_count;
        }
    }

    result.safe_state_count = safe_count;
    result.safe_state_percentage = total_product_states > 0 ? (100.0 * static_cast<double>(safe_count) / static_cast<double>(total_product_states)) : 0.0;
    result.invariant_set = move(current_safe);
    result.safe_policy = move(current_policy);

    return result;
}

} // namespace symbolic::synthesis
