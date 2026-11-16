#include "safety_solver.hpp"
#include <chrono>

namespace symbolic::synthesis {

using namespace std;
using namespace chrono;
using namespace abstraction;
using namespace automaton;

// Algorithm 1: Finding everywhere the robot can drive safely forever.
// Goal: Find all cells where the robot always has a move available that avoids obstacles,
// even when random motion noise tries to blow it off course.
SafetyResult SafetySolver::solve_safety(
    const TransitionSystem& ts,
    const unordered_set<size_t>& obstacles,
    size_t max_iters)
{
    auto t0 = high_resolution_clock::now();
    SafetyResult res;

    size_t num_states = ts.state_count;
    size_t num_inputs = ts.input_count;

    // Step 1: Start out assuming all cells outside the obstacles are safe
    vector<bool> safe(num_states, true);
    for (size_t obs_idx : obstacles) {
        if (obs_idx < num_states) {
            safe[obs_idx] = false;
        }
    }

    vector<optional<size_t>> policy(num_states, nullopt);

    // Step 2: Pruning loop: keep checking each cell and drop any that could lead to trouble
    size_t iter = 0;
    while (iter < max_iters) {
        ++iter;
        bool pruned = false;
        vector<bool> next_safe = safe;

        for (size_t s = 0; s < num_states; ++s) {
            if (!safe[s]) {
                continue; // Already marked as dangerous
            }

            // Check if there is at least one move where EVERY possible landing spot is safe
            bool found_safe = false;
            for (size_t u = 0; u < num_inputs; ++u) {
                const auto& succs = ts.get_successors(s, u);
                if (succs.empty()) {
                    continue; // Move would drive out of bounds or isn't possible
                }

                bool all_safe = true;
                for (size_t succ : succs) {
                    if (!safe[succ]) {
                        all_safe = false;
                        break;
                    }
                }

                if (all_safe) {
                    found_safe = true;
                    policy[s] = u;
                    break;
                }
            }

            // If none of our moves can guarantee safety against bad noise bumps, this cell is a trap!
            if (!found_safe) {
                next_safe[s] = false;
                policy[s] = nullopt;
                pruned = true;
            }
        }

        safe = move(next_safe);

        // Done! We didn't have to remove any cells this round, so the safe zone is locked in.
        if (!pruned) {
            res.has_converged = true;
            break;
        }
    }

    auto t1 = high_resolution_clock::now();
    res.computation_duration_seconds = duration<double>(t1 - t0).count();
    res.iterations_completed = iter;

    // Count how many guaranteed safe cells we ended up with
    size_t safe_count = 0;
    for (size_t s = 0; s < num_states; ++s) {
        if (safe[s]) {
            ++safe_count;
        }
    }

    res.safe_state_count = safe_count;
    res.safe_state_percentage = num_states > 0 ? (100.0 * static_cast<double>(safe_count) / static_cast<double>(num_states)) : 0.0;
    res.invariant_set = move(safe);
    res.safe_policy = move(policy);

    return res;
}

// Calculates safe moves across the combined tracking graph (avoiding the failed mission state q5)
SafetyResult SafetySolver::solve_safety_product(
    const TransitionSystem& ts,
    const ProductAutomaton& automaton,
    size_t max_iters)
{
    auto t0 = high_resolution_clock::now();
    SafetyResult res;

    size_t num_prod   = automaton.total_product_states();
    size_t num_cells  = automaton.spatial_cell_count;
    size_t num_inputs = ts.input_count;

    // Step 1: Start by keeping every state that hasn't failed the mission (not q5)
    vector<bool> safe(num_prod, true);
    for (size_t p = 0; p < num_prod; ++p) {
        auto decoded = automaton.decode_product_index(p);
        if (automaton.specification_dfa.is_sink_state(decoded.automaton_state)) {
            safe[p] = false;
        }
    }

    vector<optional<size_t>> policy(num_prod, nullopt);

    // Step 2: Drop any states where bad luck with noise could force us into the fail state
    size_t iter = 0;
    while (iter < max_iters) {
        ++iter;
        bool pruned = false;
        vector<bool> next_safe = safe;

        for (size_t p = 0; p < num_prod; ++p) {
            if (!safe[p]) {
                continue;
            }

            auto decoded = automaton.decode_product_index(p);
            size_t s = decoded.spatial_cell_index;
            auto q   = decoded.automaton_state;

            bool found_safe = false;
            for (size_t u = 0; u < num_inputs; ++u) {
                const auto& succs = ts.get_successors(s, u);
                if (succs.empty()) {
                    continue;
                }

                bool all_safe = true;
                for (size_t succ_s : succs) {
                    if (succ_s >= num_cells) {
                        all_safe = false;
                        break;
                    }

                    ObservationSymbol obs = automaton.get_cell_observation(succ_s);
                    auto q_next = automaton.specification_dfa.transition(q, obs);

                    size_t p_next = automaton.encode_product_index(succ_s, q_next);
                    if (p_next >= num_prod || !safe[p_next]) {
                        all_safe = false;
                        break;
                    }
                }

                if (all_safe) {
                    found_safe = true;
                    policy[p] = u;
                    break;
                }
            }

            if (!found_safe) {
                next_safe[p] = false;
                policy[p] = nullopt;
                pruned = true;
            }
        }

        safe = move(next_safe);

        if (!pruned) {
            res.has_converged = true;
            break;
        }
    }

    auto t1 = high_resolution_clock::now();
    res.computation_duration_seconds = duration<double>(t1 - t0).count();
    res.iterations_completed = iter;

    size_t safe_count = 0;
    for (size_t p = 0; p < num_prod; ++p) {
        if (safe[p]) {
            ++safe_count;
        }
    }

    res.safe_state_count = safe_count;
    res.safe_state_percentage = num_prod > 0 ? (100.0 * static_cast<double>(safe_count) / static_cast<double>(num_prod)) : 0.0;
    res.invariant_set = move(safe);
    res.safe_policy = move(policy);

    return res;
}

} // namespace symbolic::synthesis
