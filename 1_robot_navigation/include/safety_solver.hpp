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

// Result of Algorithm 1: finding safe spots where the robot can keep driving forever without crashing
struct SafetyResult {
    bool has_converged = false;
    size_t iterations_completed = 0;
    size_t safe_state_count = 0;
    double safe_state_percentage = 0.0;
    double computation_duration_seconds = 0.0;

    // invariant_set[s] = true if the robot can stay in safe territory forever starting from state s
    vector<bool> invariant_set;

    // safe_policy[s] = a safe move the robot can take so every possible next position remains safe
    vector<optional<size_t>> safe_policy;
};

// Implements Algorithm 1 (Staying Safe Forever) from Antoine Girard's paper.
// While reach-avoid starts at the goal and grows outward, this safety algorithm works by elimination:
// 1. Start by assuming all open cells (outside obstacles) are safe.
// 2. In each round, remove any cell where noise or turning could push the robot into danger.
// 3. Stop when no more cells need to be removed. What's left is the guaranteed safe zone!
class SafetySolver {
public:
    SafetySolver() = default;

    // Finds all spots where the robot can drive around forever without hitting obstacles
    [[nodiscard]] SafetyResult solve_safety(
        const TransitionSystem& ts,
        const unordered_set<size_t>& obstacles,
        size_t max_iters = 100);

    // Finds safe spots across the combined tracking graph (staying clear of the fail state q5)
    [[nodiscard]] SafetyResult solve_safety_product(
        const TransitionSystem& ts,
        const ProductAutomaton& automaton,
        size_t max_iters = 100);
};

} // namespace symbolic::synthesis

namespace symbolic {
    using synthesis::SafetyResult;
    using synthesis::SafetySolver;
}
