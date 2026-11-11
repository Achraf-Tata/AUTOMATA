// =============================================================================
// product_automaton.cpp — Member 3
// =============================================================================
#include "product_automaton.hpp"

namespace automaton {

int dfa_transition(int q, int label) noexcept {
    // 5x5 transition table:
    // Rows: q in 1..5
    // Columns: label in 1..5 (1:Free, 2:R1, 3:R2, 4:R3, 5:R4)
    static constexpr int TABLE[5][5] = {
    //  lab: 1  2  3  4  5
        {    1, 2, 3, 1, 5 },  // q=1: Start -> R1 visits 2, R2 visits 3, R4 hits 5
        {    2, 2, 5, 4, 5 },  // q=2: R1 visited -> R2 forbidden (5), R3 reaches goal (4)
        {    3, 5, 3, 4, 5 },  // q=3: R2 visited -> R1 forbidden (5), R3 reaches goal (4)
        {    4, 4, 4, 4, 4 },  // q=4: Goal reached (absorbing)
        {    5, 5, 5, 5, 5 }   // q=5: Sink trap (absorbing)
    };
    if (q < 1 || q > 5 || label < 1 || label > 5) return q;
    return TABLE[q - 1][label - 1];
}

std::vector<float> init_value_function() {
    std::vector<float> V(NTOTAL, INF);
    // q=4 (goal): cost = 0 everywhere
    int base_goal = (4 - 1) * grid::NCELLS;
    for (int i = 0; i < grid::NCELLS; ++i) {
        V[base_goal + i] = 0.0f;
    }
    return V;
}

std::vector<uint8_t> init_controller() {
    return std::vector<uint8_t>(NTOTAL, 0u);
}

} // namespace automaton
