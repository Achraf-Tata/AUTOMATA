#pragma once
#include "interval_abstraction.hpp"
#include <cstdint>
#include <array>

namespace thermostat::bitset {

inline constexpr uint64_t SAFE_MASK = ((1ULL << 15) - 1ULL) << 20; // Bits 20 through 34 set

struct BitsetTransitionTable {
    uint64_t masks[NUM_CELLS + 1][2];
};

inline BitsetTransitionTable build_bitset_table() {
    BitsetTransitionTable btable{};
    for (int s = 0; s <= NUM_CELLS; ++s) {
        for (int u = 0; u < 2; ++u) {
            uint64_t mask = 0ULL;
            auto post = compute_post_cells(s, u);
            for (int succ : post) {
                if (succ <= NUM_CELLS) {
                    mask |= (1ULL << succ);
                }
            }
            btable.masks[s][u] = mask;
        }
    }
    return btable;
}

inline uint64_t compute_pre_bitset(uint64_t R_mask, const BitsetTransitionTable& btable) noexcept {
    uint64_t pre = 0ULL;
    uint64_t not_R = ~R_mask;

    for (int s = 0; s <= NUM_CELLS; ++s) {
        bool u0_ok = ((btable.masks[s][0] & not_R) == 0ULL);
        bool u1_ok = ((btable.masks[s][1] & not_R) == 0ULL);
        if (u0_ok || u1_ok) {
            pre |= (1ULL << s);
        }
    }
    return pre;
}

inline uint64_t solve_fixed_point_bitset(const BitsetTransitionTable& btable, int& iterations) noexcept {
    uint64_t R = SAFE_MASK;
    iterations = 0;

    while (true) {
        ++iterations;
        uint64_t pre = compute_pre_bitset(R, btable);
        uint64_t R_next = R & pre;
        if (R_next == R) {
            break;
        }
        R = R_next;
    }

    return R;
}

} // namespace thermostat::bitset
