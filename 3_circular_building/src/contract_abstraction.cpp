#include "contract_abstraction.hpp"

namespace circular::abstraction {

std::vector<int> ContractAbstraction::compute_post(int cell_id, int u_idx) {
    if (cell_id == SINK_CELL) {
        return {SINK_CELL}; // Absorbing sink state
    }

    auto [lo, hi] = UniformGrid1D::cell_bounds(cell_id);
    auto [next_lo, next_hi] = step_interval(lo, hi, u_idx);

    std::vector<int> post;
    bool hits_sink = false;

    if (next_lo < SAFE_MIN || next_hi > SAFE_MAX) {
        hits_sink = true;
    }

    int k0 = static_cast<int>(std::floor((next_lo - SAFE_MIN) / CELL_W));
    int k1 = static_cast<int>(std::floor((next_hi - SAFE_MIN) / CELL_W));

    int k_start = std::max(0, k0);
    int k_end   = std::min(ND_CELLS - 1, k1);

    if (next_hi >= SAFE_MIN + (k_end + 1) * CELL_W && k_end < ND_CELLS - 1) {
        k_end++;
    }

    for (int k = k_start; k <= k_end; ++k) {
        post.push_back(k);
    }

    if (hits_sink) {
        post.push_back(SINK_CELL);
    }

    return post;
}

TransitionTable ContractAbstraction::build_transition_table() {
    TransitionTable trans;
    for (int s = 0; s <= ND_CELLS; ++s) {
        for (int u_idx = 0; u_idx < NUM_CONTROLS; ++u_idx) {
            trans.table[s][u_idx] = compute_post(s, u_idx);
        }
    }
    return trans;
}

} // namespace circular::abstraction
