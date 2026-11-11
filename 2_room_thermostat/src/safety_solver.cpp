#include "safety_solver.hpp"

namespace thermostat {

TransitionTable build_transition_table() {
    TransitionTable trans;
    for (int s = 0; s <= NUM_CELLS; ++s) {
        for (int u = 0; u < 2; ++u) {
            trans.table[s][u] = compute_post_cells(s, u);
        }
    }
    return trans;
}

std::vector<bool> pre_operator(const std::vector<bool>& in_R, const TransitionTable& trans) {
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
                break; // Found at least one control guaranteeing safety
            }
        }
    }

    return pre;
}

std::vector<bool> solve_maximal_invariant_set(const TransitionTable& trans, int& iterations) {
    // Initial safe set S_safe = {20..34}
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

std::array<int, NUM_CELLS + 1> extract_controller(const std::vector<bool>& R_star, const TransitionTable& trans) {
    std::array<int, NUM_CELLS + 1> ctrl;
    ctrl.fill(-1);

    for (int s = 0; s <= NUM_CELLS; ++s) {
        if (!R_star[s]) {
            // Fallback for states outside invariant set: heat if below safe range, cool if above
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

} // namespace thermostat
