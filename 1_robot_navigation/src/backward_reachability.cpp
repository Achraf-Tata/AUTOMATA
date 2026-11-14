#include "backward_reachability.hpp"
#include <chrono>
#include <algorithm>
#include <iostream>

namespace symbolic::synthesis {

using namespace std;
using namespace chrono;
using namespace abstraction;
using namespace automaton;
using enum AutomatonState;
using enum ObservationSymbol;

// backward reachability algorithm on product automaton (position x DFA state)
SynthesisResult BackwardReachabilitySolver::solve_product(
    const TransitionSystem& transition_system,
    const ProductAutomaton& product_automaton,
    size_t maximum_iterations)
{
    auto start_timestamp = high_resolution_clock::now();

    const size_t total_product_states = product_automaton.total_product_states();
    const size_t total_spatial_cells  = product_automaton.spatial_cell_count;
    const size_t total_control_inputs = transition_system.input_count;

    SynthesisResult synthesis_result;
    synthesis_result.value_function.assign(total_product_states, UNREACHABLE_COST);
    synthesis_result.feedback_policy.assign(total_product_states, nullopt);

    // base step: all states where DFA is in goal state (q4) have cost = 0
    for (size_t spatial_cell_index = 0; spatial_cell_index < total_spatial_cells; ++spatial_cell_index) {
        size_t target_product_index = product_automaton.encode_product_index(
            spatial_cell_index, GoalReached);
        synthesis_result.value_function[target_product_index] = 0.0;
    }

    size_t newly_controlled_states = 0;

    // fixed-point loop: expand the controllable set backwards until convergence
    for (size_t iteration = 1; iteration <= maximum_iterations; ++iteration) {
        newly_controlled_states = 0;
        vector<double> previous_value_function = synthesis_result.value_function;

        // iterate backwards through DFA states: q3 (visited R2), q2 (visited R1), then q1 (start)
        for (int dfa_state_value = 3; dfa_state_value >= 1; --dfa_state_value) {
            auto current_automaton_state = static_cast<AutomatonState>(dfa_state_value);

            for (size_t spatial_cell_index = 0; spatial_cell_index < total_spatial_cells; ++spatial_cell_index) {
                size_t current_product_index = product_automaton.encode_product_index(
                    spatial_cell_index, current_automaton_state);

                // skip if this state is already controlled
                if (synthesis_result.value_function[current_product_index] < UNREACHABLE_COST) {
                    continue;
                }

                // obstacle cells are unsafe and cannot be part of the winning set
                ObservationSymbol current_cell_observation =
                    product_automaton.get_cell_observation(spatial_cell_index);
                if (current_cell_observation == Obstacle) {
                    continue;
                }

                AutomatonState advanced_automaton_state =
                    product_automaton.specification_dfa.transition(
                        current_automaton_state, current_cell_observation);

                if (advanced_automaton_state == SinkTrap) {
                    continue;
                }

                // if entering this cell advances the DFA (e.g. stepping into R1/R2 or Goal)
                if (advanced_automaton_state != current_automaton_state) {
                    size_t advanced_product_index = product_automaton.encode_product_index(
                        spatial_cell_index, advanced_automaton_state);
                    if (synthesis_result.value_function[advanced_product_index] < UNREACHABLE_COST) {
                        synthesis_result.value_function[current_product_index] = static_cast<double>(iteration);
                        synthesis_result.feedback_policy[current_product_index] = synthesis_result.feedback_policy[advanced_product_index];
                        ++newly_controlled_states;
                        continue;
                    }
                }

                // test all discrete control inputs (v, w)
                for (size_t control_index = 0; control_index < total_control_inputs; ++control_index) {
                    const auto& successor_spatial_cells = transition_system.get_successors(
                        spatial_cell_index, control_index);

                    if (successor_spatial_cells.empty()) {
                        continue;
                    }

                    // must guarantee that ALL reachable next cells are in the winning set
                    bool all_successors_controllable = true;

                    for (size_t successor_cell_index : successor_spatial_cells) {
                        ObservationSymbol succ_obs =
                            product_automaton.get_cell_observation(successor_cell_index);
                        AutomatonState succ_dfa_state =
                            product_automaton.specification_dfa.transition(
                                advanced_automaton_state, succ_obs);

                        if (succ_dfa_state == SinkTrap) {
                            all_successors_controllable = false;
                            break;
                        }

                        size_t successor_product_index = product_automaton.encode_product_index(
                            successor_cell_index, succ_dfa_state);

                        if (previous_value_function[successor_product_index] >= UNREACHABLE_COST) {
                            all_successors_controllable = false;
                            break;
                        }
                    }

                    // if all successors are safe and winning, pick this control action
                    if (all_successors_controllable) {
                        synthesis_result.value_function[current_product_index] = static_cast<double>(iteration);
                        synthesis_result.feedback_policy[current_product_index] = control_index;
                        ++newly_controlled_states;
                        break;
                    }
                }
            }
        }

        synthesis_result.iterations_completed = iteration;

        size_t q1_cnt = 0, q2_cnt = 0, q3_cnt = 0, q4_cnt = 0;
        for (size_t s = 0; s < total_spatial_cells; ++s) {
            if (synthesis_result.value_function[product_automaton.encode_product_index(s, Initial)] < UNREACHABLE_COST) q1_cnt++;
            if (synthesis_result.value_function[product_automaton.encode_product_index(s, VisitedWaypoint1)] < UNREACHABLE_COST) q2_cnt++;
            if (synthesis_result.value_function[product_automaton.encode_product_index(s, VisitedWaypoint2)] < UNREACHABLE_COST) q3_cnt++;
            if (synthesis_result.value_function[product_automaton.encode_product_index(s, GoalReached)] < UNREACHABLE_COST) q4_cnt++;
        }
        cout << "Iteration " << iteration << ": newly controlled = " << newly_controlled_states
             << " (q1=" << q1_cnt << ", q2=" << q2_cnt << ", q3=" << q3_cnt << ", q4=" << q4_cnt << ")\n";

        // if no new states became controlled, we reached the fixed-point
        if (newly_controlled_states == 0) {
            synthesis_result.has_converged = true;
            break;
        }
    }

    // count total controllable states
    size_t total_controllable = 0;
    for (size_t product_index = 0; product_index < total_product_states; ++product_index) {
        if (synthesis_result.value_function[product_index] < UNREACHABLE_COST) {
            ++total_controllable;
        }
    }

    auto end_timestamp = high_resolution_clock::now();
    synthesis_result.computation_duration_seconds =
        duration<double>(end_timestamp - start_timestamp).count();
    synthesis_result.controllable_state_count = total_controllable;
    synthesis_result.controllable_state_percentage =
        100.0 * static_cast<double>(total_controllable) / static_cast<double>(total_product_states);

    return synthesis_result;
}

// baseline reachability algorithm without DFA (direct reach-avoid on the spatial grid)
SynthesisResult BackwardReachabilitySolver::solve_spatial(
    const TransitionSystem& transition_system,
    const unordered_set<size_t>& target_spatial_states,
    const unordered_set<size_t>& obstacle_spatial_states,
    size_t maximum_iterations)
{
    auto start_timestamp = high_resolution_clock::now();

    const size_t total_spatial_states = transition_system.state_count;
    const size_t total_control_inputs = transition_system.input_count;

    SynthesisResult synthesis_result;
    synthesis_result.value_function.assign(total_spatial_states, UNREACHABLE_COST);
    synthesis_result.feedback_policy.assign(total_spatial_states, nullopt);

    for (size_t target_state : target_spatial_states) {
        if (target_state < total_spatial_states &&
            obstacle_spatial_states.find(target_state) == obstacle_spatial_states.end())
        {
            synthesis_result.value_function[target_state] = 0.0;
        }
    }

    size_t newly_controlled_states = 0;

    for (size_t iteration = 1; iteration <= maximum_iterations; ++iteration) {
        newly_controlled_states = 0;
        vector<double> previous_value_function = synthesis_result.value_function;

        for (size_t state_index = 0; state_index < total_spatial_states; ++state_index) {
            if (synthesis_result.value_function[state_index] < UNREACHABLE_COST ||
                obstacle_spatial_states.find(state_index) != obstacle_spatial_states.end())
            {
                continue;
            }

            for (size_t control_index = 0; control_index < total_control_inputs; ++control_index) {
                const auto& successor_states = transition_system.get_successors(state_index, control_index);

                if (successor_states.empty()) {
                    continue;
                }

                bool all_successors_controllable = true;
                for (size_t successor : successor_states) {
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

    size_t total_controllable = 0;
    for (size_t state_index = 0; state_index < total_spatial_states; ++state_index) {
        if (synthesis_result.value_function[state_index] < UNREACHABLE_COST) {
            ++total_controllable;
        }
    }

    auto end_timestamp = high_resolution_clock::now();
    synthesis_result.computation_duration_seconds =
        duration<double>(end_timestamp - start_timestamp).count();
    synthesis_result.controllable_state_count = total_controllable;
    synthesis_result.controllable_state_percentage =
        100.0 * static_cast<double>(total_controllable) / static_cast<double>(total_spatial_states);

    return synthesis_result;
}

} // namespace symbolic::synthesis
