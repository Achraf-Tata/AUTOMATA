#include "contract_solver.hpp"

namespace circular {

LocalTransitionTable build_local_transition_table() {
    LocalTransitionTable trans;
    for (int s = 0; s <= NUM_CELLS; ++s) {
        for (int u = 0; u < 2; ++u) {
            trans.table[s][u] = compute_local_post(s, u);
        }
    }
    return trans;
}

std::vector<bool> pre_operator(const std::vector<bool>& in_R, const LocalTransitionTable& trans) {
    std::vector<bool> pre(NUM_CELLS + 1, false);

    for (int s = 0; s <= NUM_CELLS; ++s) {
        for (int u = 0; u < 2; ++u) {
            const auto& post = trans.table[s][u];
            bool all_safe = !post.empty();
            for (int succ : post) {
                if (!in_R[succ]) {
                    all_safe = false;
                    break;
                }
            }
            if (all_safe) {
                pre[s] = true;
                break;
            }
        }
    }

    return pre;
}

std::vector<bool> solve_local_invariant_contract(const LocalTransitionTable& trans, int& iterations) {
    std::vector<bool> R(NUM_CELLS + 1, false);
    for (int k = SAFE_CELL_START; k <= SAFE_CELL_END; ++k) {
        R[k] = true;
    }

    iterations = 0;
    while (true) {
        ++iterations;
        std::vector<bool> pre = pre_operator(R, trans);
        std::vector<bool> R_next(NUM_CELLS + 1, false);

        bool changed = false;
        for (int s = 0; s <= NUM_CELLS; ++s) {
            R_next[s] = R[s] && pre[s];
            if (R_next[s] != R[s]) {
                changed = true;
            }
        }

        if (!changed) {
            break;
        }
        R = R_next;
    }

    return R;
}

std::array<int, NUM_CELLS + 1> extract_local_controller(
    const std::vector<bool>& R_star,
    const LocalTransitionTable& trans)
{
    std::array<int, NUM_CELLS + 1> ctrl;
    ctrl.fill(-1);

    for (int s = 0; s <= NUM_CELLS; ++s) {
        if (!R_star[s]) {
            if (s < SAFE_CELL_START) ctrl[s] = 1;
            else if (s > SAFE_CELL_END) ctrl[s] = 0;
            continue;
        }

        // Test u = 0 (energy-saving tie-breaker)
        bool u0_ok = true;
        for (int succ : trans.table[s][0]) {
            if (!R_star[succ]) {
                u0_ok = false;
                break;
            }
        }

        if (u0_ok) {
            ctrl[s] = 0;
            continue;
        }

        // Test u = 1
        bool u1_ok = true;
        for (int succ : trans.table[s][1]) {
            if (!R_star[succ]) {
                u1_ok = false;
                break;
            }
        }

        if (u1_ok) {
            ctrl[s] = 1;
        }
    }

    return ctrl;
}

bool verify_contract_composition(const std::vector<bool>& R_star) {
    // Check that every cell in R_star is strictly within [SAFE_CELL_START, SAFE_CELL_END]
    for (int s = 0; s <= NUM_CELLS; ++s) {
        if (R_star[s]) {
            if (s < SAFE_CELL_START || s > SAFE_CELL_END) {
                return false;
            }
        }
    }
    return true;
}

} // namespace circular
