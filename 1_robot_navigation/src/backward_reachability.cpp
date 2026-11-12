#include "backward_reachability.hpp"
#include <chrono>
#include <algorithm>

namespace symbolic::synthesis {

// Solves fixed-point reachability game over the product automaton (S x A)
SynthesisResult BackwardReachabilitySolver::solve_product(
    const abstraction::TransitionSystem& transition_system,
    const automaton::ProductAutomaton& product_automaton,
    std::size_t maximum_iterations)
{
    auto start_timestamp = std::chrono::high_resolution_clock::now();

    const std::size_t total_product_states = product_automaton.total_product_states();
    const std::size_t total_spatial_cells  = product_automaton.spatial_cell_count;
    const std::size_t total_control_inputs = transition_system.input_count;

    SynthesisResult synthesis_result;
    synthesis_result.value_function.assign(total_product_states, UNREACHABLE_COST);
    synthesis_result.feedback_policy.assign(total_product_states, std::nullopt);

    // Initialize accepting target states: V(c, q4) = 0 for all spatial cells
    for (std::size_t spatial_cell_index = 0; spatial_cell_index < total_spatial_cells; ++spatial_cell_index) {
        std::size_t target_product_index = product_automaton.encode_product_index(
            spatial_cell_index, automaton::AutomatonState::GoalReached);
        synthesis_result.value_function[target_product_index] = 0.0;
    }

    std::size_t newly_controlled_states = 0;

    for (std::size_t iteration = 1; iteration <= maximum_iterations; ++iteration) {
        newly_controlled_states = 0;
        std::vector<double> previous_value_function = synthesis_result.value_function;

        // Iterate backward over all non-terminal DFA states:
        // q3 (Visited R2), q2 (Visited R1), then q1 (Initial)
        for (int dfa_state_value = 3; dfa_state_value >= 1; --dfa_state_value) {
            auto current_automaton_state = static_cast<automaton::AutomatonState>(dfa_state_value);

            for (std::size_t spatial_cell_index = 0; spatial_cell_index < total_spatial_cells; ++spatial_cell_index) {
                std::size_t current_product_index = product_automaton.encode_product_index(
                    spatial_cell_index, current_automaton_state);

                // Skip if this product state has already converged
                if (synthesis_result.value_function[current_product_index] < UNREACHABLE_COST) {
                    continue;
                }

                // If the current cell is inside an obstacle, it is permanently uncontrollable
                automaton::ObservationSymbol current_cell_observation =
                    product_automaton.get_cell_observation(spatial_cell_index);
                if (current_cell_observation == automaton::ObservationSymbol::Obstacle) {
                    continue;
                }

                // Check all discrete control inputs
                for (std::size_t control_index = 0; control_index < total_control_inputs; ++control_index) {
                    const auto& successor_spatial_cells = transition_system.get_successors(
                        spatial_cell_index, control_index);

                    if (successor_spatial_cells.empty()) {
                        continue;
                    }

                    bool all_successors_controllable = true;

                    for (std::size_t successor_cell_index : successor_spatial_cells) {
                        // The automaton transitions based on the observation of the successor state
                        automaton::ObservationSymbol successor_observation =
                            product_automaton.get_cell_observation(successor_cell_index);

                        automaton::AutomatonState successor_automaton_state =
                            product_automaton.specification_dfa.transition(
                                current_automaton_state, successor_observation);

                        // If any successor hits an obstacle or violates exclusivity, the action is unsafe
                        if (successor_automaton_state == automaton::AutomatonState::SinkTrap) {
                            all_successors_controllable = false;
                            break;
                        }

                        std::size_t successor_product_index = product_automaton.encode_product_index(
                            successor_cell_index, successor_automaton_state);

                        if (previous_value_function[successor_product_index] >= UNREACHABLE_COST) {
                            all_successors_controllable = false;
                            break;
                        }
                    }

                    if (all_successors_controllable) {
                        synthesis_result.value_function[current_product_index] = static_cast<double>(iteration);
                        synthesis_result.feedback_policy[current_product_index] = control_index;
                        ++newly_controlled_states;
                        break; // Greedily select the first winning control action
                    }
                }
            }
        }

        synthesis_result.iterations_completed = iteration;

        if (newly_controlled_states == 0) {
            synthesis_result.has_converged = true;
            break;
        }
    }

    // Compute metrics
    std::size_t total_controllable = 0;
    for (std::size_t product_index = 0; product_index < total_product_states; ++product_index) {
        if (synthesis_result.value_function[product_index] < UNREACHABLE_COST) {
            ++total_controllable;
        }
    }

    auto end_timestamp = std::chrono::high_resolution_clock::now();
    synthesis_result.computation_duration_seconds =
        std::chrono::duration<double>(end_timestamp - start_timestamp).count();
    synthesis_result.controllable_state_count = total_controllable;
    synthesis_result.controllable_state_percentage =
        100.0 * static_cast<double>(total_controllable) / static_cast<double>(total_product_states);

    return synthesis_result;
}

// Solves reachability game over spatial states to target set avoiding obstacles
SynthesisResult BackwardReachabilitySolver::solve_spatial(
    const abstraction::TransitionSystem& transition_system,
    const std::unordered_set<std::size_t>& target_spatial_states,
    const std::unordered_set<std::size_t>& obstacle_spatial_states,
    std::size_t maximum_iterations)
{
    auto start_timestamp = std::chrono::high_resolution_clock::now();

    const std::size_t total_spatial_states = transition_system.state_count;
    const std::size_t total_control_inputs = transition_system.input_count;

    SynthesisResult synthesis_result;
    synthesis_result.value_function.assign(total_spatial_states, UNREACHABLE_COST);
    synthesis_result.feedback_policy.assign(total_spatial_states, std::nullopt);

    for (std::size_t target_state : target_spatial_states) {
        if (target_state < total_spatial_states &&
            obstacle_spatial_states.find(target_state) == obstacle_spatial_states.end())
        {
            synthesis_result.value_function[target_state] = 0.0;
        }
    }

    std::size_t newly_controlled_states = 0;

    for (std::size_t iteration = 1; iteration <= maximum_iterations; ++iteration) {
        newly_controlled_states = 0;
        std::vector<double> previous_value_function = synthesis_result.value_function;

        for (std::size_t state_index = 0; state_index < total_spatial_states; ++state_index) {
            if (synthesis_result.value_function[state_index] < UNREACHABLE_COST ||
                obstacle_spatial_states.find(state_index) != obstacle_spatial_states.end())
            {
                continue;
            }

            for (std::size_t control_index = 0; control_index < total_control_inputs; ++control_index) {
                const auto& successor_states = transition_system.get_successors(state_index, control_index);

                if (successor_states.empty()) {
                    continue;
                }

                bool all_successors_controllable = true;
                for (std::size_t successor : successor_states) {
                    if (previous_value_function[successor] >= UNREACHABLE_COST) {
                        all_successors_controllable = false;
                        break;
                    }
                }

                if (all_successors_controllable) {
                    synthesis_result.value_function[state_index] = static_cast<double>(iteration);
                    synthesis_result.feedback_policy[state_index] = control_index;
                    ++newly_controlled_states;
                    break;
                }
            }
        }

        synthesis_result.iterations_completed = iteration;

        if (newly_controlled_states == 0) {
            synthesis_result.has_converged = true;
            break;
        }
    }

    std::size_t total_controllable = 0;
    for (std::size_t state_index = 0; state_index < total_spatial_states; ++state_index) {
        if (synthesis_result.value_function[state_index] < UNREACHABLE_COST) {
            ++total_controllable;
        }
    }

    auto end_timestamp = std::chrono::high_resolution_clock::now();
    synthesis_result.computation_duration_seconds =
        std::chrono::duration<double>(end_timestamp - start_timestamp).count();
    synthesis_result.controllable_state_count = total_controllable;
    synthesis_result.controllable_state_percentage =
        100.0 * static_cast<double>(total_controllable) / static_cast<double>(total_spatial_states);

    return synthesis_result;
}

} // namespace symbolic::synthesis
