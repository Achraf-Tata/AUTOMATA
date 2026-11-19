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

// Works backward from the goal on the combined tracking graph (cell position x mission stage)
SynthesisResult BackwardReachabilitySolver::solve_product(
    const TransitionSystem& ts,
    const ProductAutomaton& automaton,
    size_t max_iters)
{
    auto t0 = high_resolution_clock::now();

    const size_t num_prod   = automaton.total_product_states();
    const size_t num_cells  = automaton.num_cells;
    const size_t num_inputs = ts.num_inputs;

    SynthesisResult res;
    res.value_function.assign(num_prod, UNREACHABLE_COST);
    res.feedback_policy.assign(num_prod, nullopt);

    // Base case: if we are already at the goal (q4), we need 0 steps
    for (size_t s = 0; s < num_cells; ++s) {
        size_t target_idx = automaton.encode_product_index(s, GoalReached);
        res.value_function[target_idx] = 0.0;
    }

    size_t newly_controlled = 0;

    // Main loop: keep stepping backwards to discover all spots that can reach the goal
    for (size_t iter = 1; iter <= max_iters; ++iter) {
        newly_controlled = 0;
        vector<double> prev_v = res.value_function;

        // Work backwards through mission stages: q3 (did R2), q2 (did R1), then q1 (starting out)
        for (int q_val = 3; q_val >= 1; --q_val) {
            auto q = static_cast<AutomatonState>(q_val);

            for (size_t s = 0; s < num_cells; ++s) {
                size_t p = automaton.encode_product_index(s, q);

                // Skip if we already know how to reach the goal from here
                if (res.value_function[p] < UNREACHABLE_COST) {
                    continue;
                }

                // Obstacle cells are dangerous, so we never want to go there
                ObservationSymbol obs = automaton.get_cell_observation(s);
                if (obs == Obstacle) {
                    continue;
                }

                AutomatonState q_adv = automaton.dfa.transition(q, obs);
                if (q_adv == SinkTrap) {
                    continue;
                }

                // If stepping into this cell completes the current mission milestone (e.g. hitting R1, R2, or Goal)
                if (q_adv != q) {
                    size_t p_adv = automaton.encode_product_index(s, q_adv);
                    if (res.value_function[p_adv] < UNREACHABLE_COST) {
                        res.value_function[p] = static_cast<double>(iter);
                        res.feedback_policy[p] = res.feedback_policy[p_adv];
                        ++newly_controlled;
                        continue;
                    }
                }

                // Check every possible move the robot could take (speed and turn)
                for (size_t u = 0; u < num_inputs; ++u) {
                    const auto& succs = ts.get_successors(s, u);
                    if (succs.empty()) {
                        continue;
                    }

                    // We need a guarantee: every possible cell noise could push us into must still lead to the goal
                    bool all_safe = true;

                    for (size_t succ_s : succs) {
                        ObservationSymbol succ_obs = automaton.get_cell_observation(succ_s);
                        AutomatonState succ_q = automaton.dfa.transition(q_adv, succ_obs);

                        if (succ_q == SinkTrap) {
                            all_safe = false;
                            break;
                        }

                        size_t succ_p = automaton.encode_product_index(succ_s, succ_q);
                        if (prev_v[succ_p] >= UNREACHABLE_COST) {
                            all_safe = false;
                            break;
                        }
                    }

                    // If every outcome is safe and on track to win, record this as our chosen move!
                    if (all_safe) {
                        res.value_function[p] = static_cast<double>(iter);
                        res.feedback_policy[p] = u;
                        ++newly_controlled;
                        break;
                    }
                }
            }
        }

        res.iterations_completed = iter;

        size_t q1_cnt = 0, q2_cnt = 0, q3_cnt = 0, q4_cnt = 0;
        for (size_t s = 0; s < num_cells; ++s) {
            if (res.value_function[automaton.encode_product_index(s, Initial)] < UNREACHABLE_COST) q1_cnt++;
            if (res.value_function[automaton.encode_product_index(s, VisitedWaypoint1)] < UNREACHABLE_COST) q2_cnt++;
            if (res.value_function[automaton.encode_product_index(s, VisitedWaypoint2)] < UNREACHABLE_COST) q3_cnt++;
            if (res.value_function[automaton.encode_product_index(s, GoalReached)] < UNREACHABLE_COST) q4_cnt++;
        }
        cout << "Iteration " << iter << ": newly controlled = " << newly_controlled
             << " (q1=" << q1_cnt << ", q2=" << q2_cnt << ", q3=" << q3_cnt << ", q4=" << q4_cnt << ")\n";

        // If we didn't discover any new winning spots this round, we are done!
        if (newly_controlled == 0) {
            res.has_converged = true;
            break;
        }
    }

    // Count how many spots on the map can successfully reach the goal
    size_t total_controllable = 0;
    for (size_t p = 0; p < num_prod; ++p) {
        if (res.value_function[p] < UNREACHABLE_COST) {
            ++total_controllable;
        }
    }

    auto t1 = high_resolution_clock::now();
    res.computation_duration_seconds = duration<double>(t1 - t0).count();
    res.controllable_state_count = total_controllable;
    res.controllable_state_percentage =
        100.0 * static_cast<double>(total_controllable) / static_cast<double>(num_prod);

    return res;
}

// Simple direct search without stages: just reach target cells while avoiding obstacles
SynthesisResult BackwardReachabilitySolver::solve_spatial(
    const TransitionSystem& ts,
    const unordered_set<size_t>& targets,
    const unordered_set<size_t>& obstacles,
    size_t max_iters)
{
    auto t0 = high_resolution_clock::now();

    const size_t num_states = ts.num_states;
    const size_t num_inputs = ts.num_inputs;

    SynthesisResult res;
    res.value_function.assign(num_states, UNREACHABLE_COST);
    res.feedback_policy.assign(num_states, nullopt);

    for (size_t target : targets) {
        if (target < num_states && obstacles.find(target) == obstacles.end()) {
            res.value_function[target] = 0.0;
        }
    }

    size_t newly_controlled = 0;

    for (size_t iter = 1; iter <= max_iters; ++iter) {
        newly_controlled = 0;
        vector<double> prev_v = res.value_function;

        for (size_t s = 0; s < num_states; ++s) {
            if (res.value_function[s] < UNREACHABLE_COST || obstacles.find(s) != obstacles.end()) {
                continue;
            }

            for (size_t u = 0; u < num_inputs; ++u) {
                const auto& succs = ts.get_successors(s, u);
                if (succs.empty()) {
                    continue;
                }

                bool all_safe = true;
                for (size_t succ : succs) {
                    if (prev_v[succ] >= UNREACHABLE_COST) {
                        all_safe = false;
                        break;
                    }
                }

                if (all_safe) {
                    res.value_function[s] = static_cast<double>(iter);
                    res.feedback_policy[s] = u;
                    ++newly_controlled;
                    break;
                }
            }
        }

        res.iterations_completed = iter;

        if (newly_controlled == 0) {
            res.has_converged = true;
            break;
        }
    }

    size_t total_controllable = 0;
    for (size_t s = 0; s < num_states; ++s) {
        if (res.value_function[s] < UNREACHABLE_COST) {
            ++total_controllable;
        }
    }

    auto t1 = high_resolution_clock::now();
    res.computation_duration_seconds = duration<double>(t1 - t0).count();
    res.controllable_state_count = total_controllable;
    res.controllable_state_percentage =
        100.0 * static_cast<double>(total_controllable) / static_cast<double>(num_states);

    return res;
}

} // namespace symbolic::synthesis
