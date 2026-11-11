// =============================================================================
// product_automaton.hpp — Member 3
//
// Product automaton: couples the continuous grid with a 5-state DFA that
// tracks whether the robot has visited R1 or R2 (exclusive), then reached R3,
// while avoiding R4.
//
// DFA states:
//   q=1  searching  — neither R1 nor R2 visited yet
//   q=2  R1 visited — seek R3 (avoid R2 and R4)
//   q=3  R2 visited — seek R3 (avoid R1 and R4)
//   q=4  goal       — target reached (SUCCESS!)
//   q=5  sink       — obstacle hit or rule violated (VIOLATION!)
//
// Atomic propositions (labels):
//   1 = Free, 2 = R1, 3 = R2, 4 = R3, 5 = R4
// =============================================================================
#pragma once
#include "grid_quantizer.hpp"
#include <vector>
#include <cstdint>
#include <limits>

namespace automaton {

inline constexpr int   NDFA   = 5;
inline constexpr int   NTOTAL = NDFA * grid::NCELLS; // 1 500 000
inline constexpr float INF    = 1e9f;

/// Transitions DFA state q (1..5) with region label (1..5)
int dfa_transition(int q, int label) noexcept;

/// Allocates value function array: V[q=4] = 0.0f, all others = INF
std::vector<float> init_value_function();

/// Allocates controller array: 0 = unassigned/uncontrollable, 1..15 = control index
std::vector<uint8_t> init_controller();

inline int product_index(int q, int cell_flat) noexcept {
    return (q - 1) * grid::NCELLS + cell_flat;
}

} // namespace automaton
