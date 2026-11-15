#pragma once

#include "transition_system.hpp"
#include "product_automaton.hpp"
#include <vector>
#include <unordered_set>
#include <optional>
#include <cstddef>

namespace symbolic::synthesis {

using namespace std;
using namespace abstraction;
using namespace automaton;

// Result of Algorithm 1 (Safety Game / Maximal Controlled Invariant Set)
struct SafetyResult {
    bool has_converged = false;
    size_t iterations_completed = 0;
    size_t safe_state_count = 0;
    double safe_state_percentage = 0.0;
    double computation_duration_seconds = 0.0;

    // invariant_set[s] = true if state s can remain safe for an infinite horizon (square Safe)
    vector<bool> invariant_set;

    // safe_policy[s] = a control action index u that guarantees all successors stay in the safe set
    vector<optional<size_t>> safe_policy;
};

// Implements Algorithm 1 (Safety Game / Invariance) from Antoine Girard's paper.
// Unlike BackwardReachabilitySolver (Algorithm 2: Reach-Avoid, which grows winning states from the goal),
// SafetySolver computes the GREATEST FIXED POINT by top-down pruning:
// 1. Start with S_0 = Safe = StateSpace \ Obstacles.
// 2. In each iteration, prune any state s from which disturbances can force the robot into unsafe states.
// 3. Terminate when no more states are pruned: S_k = S_{k-1} = Inv(Safe).
class SafetySolver {
public:
    SafetySolver() = default;

    // Computes the maximal controlled invariant set on the spatial transition system
    // given a set of unsafe obstacle state indices
    [[nodiscard]] SafetyResult solve_safety(
        const TransitionSystem& transition_system,
        const unordered_set<size_t>& obstacle_states,
        size_t maximum_iterations = 100);

    // Computes safety over the product automaton (avoiding DFA sink trap state q5)
    [[nodiscard]] SafetyResult solve_safety_product(
        const TransitionSystem& transition_system,
        const ProductAutomaton& product_automaton,
        size_t maximum_iterations = 100);
};

} // namespace symbolic::synthesis

namespace symbolic {
    using synthesis::SafetyResult;
    using synthesis::SafetySolver;
}
