// =============================================================================
// product_automaton.cpp — Member 3
// =============================================================================
#include "product_automaton.hpp"

namespace automaton {

int dfa_transition(int q, int label) {
    // Hardcoded 5×5 transition table: row = q (1-indexed), col = label (0-4).
    // Labels: 0=Free, 1=R1, 2=R2, 3=R3 (goal), 4=R4 (obstacle).
    // Values are the next DFA state.
    static constexpr int TABLE[5][5] = {
    //  label: 0  1  2  3  4
        {      1, 2, 3, 1, 5 },  // q=1: searching -> R1 goes to 2, R2 goes to 3
        {      2, 2, 5, 4, 5 },  // q=2: R1 visited -> R2 forbidden (5), R3 reaches goal (4)
        {      3, 5, 3, 4, 5 },  // q=3: R2 visited -> R1 forbidden (5), R3 reaches goal (4)
        {      4, 4, 4, 4, 4 },  // q=4: goal (absorbing)
        {      5, 5, 5, 5, 5 },  // q=5: sink (absorbing)
    };
    if (q < 1 || q > 5 || label < 0 || label > 4) return q;
    return TABLE[q - 1][label];
}

std::vector<float> init_value_function() {
    std::vector<float> V(NTOTAL, INF);
    // q=4 (goal): cost = 0 everywhere — the robot has completed the task.
    int base_goal = (4 - 1) * grid::NCELLS;
    for (int i = 0; i < grid::NCELLS; ++i)
        V[base_goal + i] = 0.0f;
    return V;
}

std::vector<uint8_t> init_controller() {
    return std::vector<uint8_t>(NTOTAL, 255u);
}

} // namespace automaton
