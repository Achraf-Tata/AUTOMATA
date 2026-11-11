#include "product_automaton.hpp"

namespace automaton {

int dfa_transition(int q, int label) noexcept {
    static constexpr int TABLE[5][5] = {
        { 1, 2, 3, 1, 5 },
        { 2, 2, 5, 4, 5 },
        { 3, 5, 3, 4, 5 },
        { 4, 4, 4, 4, 4 },
        { 5, 5, 5, 5, 5 }
    };
    if (q < 1 || q > 5 || label < 1 || label > 5) return q;
    return TABLE[q - 1][label - 1];
}

std::vector<float> init_value_function() {
    std::vector<float> V(NTOTAL, INF);
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
