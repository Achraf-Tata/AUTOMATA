// =============================================================================
// product_automaton.hpp — Member 3
//
// Product automaton: couples the continuous grid with a 5-state DFA that
// tracks whether the robot has visited R1, then R2, then reached R3,
// while avoiding R4.
//
// DFA states (from paper Section 3):
//   q=1  searching   — neither R1 nor R2 visited yet
//   q=2  R1 visited  — robot visited R1 (R2 now forbidden, head to R3)
//   q=3  R2 visited  — robot visited R2 (R1 now forbidden, head to R3)
//   q=4  goal        — robot entered R3 after R1 or R2: TASK COMPLETE
//   q=5  sink        — robot hit R4 or violated exclusivity: TASK FAILED
//
// The DFA transition function δ(q, label) is a hardcoded 5×5 table.
//
// The value function V[dfa_state][cell_flat] stores the minimum worst-case
// cost to reach the goal from each (DFA state, grid cell) pair.
// V = 0 at the goal state (q=4), V = INF everywhere else initially.
// =============================================================================
#pragma once
#include "grid_quantizer.hpp"
#include <vector>
#include <cstdint>
#include <limits>
#include <string>

namespace automaton {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

inline constexpr int    NDFA   = 5;         ///< number of DFA states (1..5)
inline constexpr int    NTOTAL = NDFA * grid::NCELLS; ///< total value-function entries
inline constexpr float  INF    = 1e7f;      ///< "unreachable" sentinel value

// ---------------------------------------------------------------------------
// DFA transition
// ---------------------------------------------------------------------------

/// Returns the next DFA state given current state q (1-indexed) and
/// atomic-proposition label (0=none, 1=R1, 2=R2, 3=R3, 4=R4).
///
/// Transition rules for mission: (Visit R1 xor R2) then R3, avoiding R4:
///   q=1: label=1 → 2,  label=2 → 3,  label=4 → 5,  else → 1
///   q=2: label=2 → 5,  label=3 → 4,  label=4 → 5,  else → 2
///   q=3: label=1 → 5,  label=3 → 4,  label=4 → 5,  else → 3
///   q=4: (absorbing goal)   → 4
///   q=5: (absorbing sink)   → 5
int dfa_transition(int q, int label);

// ---------------------------------------------------------------------------
// Value function and controller storage
// ---------------------------------------------------------------------------

/// Allocates and initialises the value function:
///   V[dfa_state - 1][cell_flat] = INF  for all states except q=4
///   V[3][*]                     = 0.0  (q=4 is the goal — cost is 0)
///   V[4][*]                     = INF  (q=5 is the sink — unreachable)
///
/// Returns a flat vector of length NTOTAL = 5 × 300000 = 1 500 000 entries.
/// Layout: index = (q-1) * NCELLS + cell_flat
std::vector<float> init_value_function();

/// Allocates the controller array: for each (q, cell) pair, stores the
/// index (0..14) of the best control action, or 255 if no action is defined.
/// Length = NTOTAL.
std::vector<uint8_t> init_controller();

// ---------------------------------------------------------------------------
// Flat index helper (product state → scalar index)
// ---------------------------------------------------------------------------

/// Maps (DFA state q ∈ [1..5], flat cell index ∈ [0..NCELLS-1]) to a
/// single index into the V / ctrl arrays.
inline int product_index(int q, int cell_flat) noexcept {
    return (q - 1) * grid::NCELLS + cell_flat;
}

} // namespace automaton
