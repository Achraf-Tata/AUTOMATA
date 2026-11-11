#pragma once
#include "interval_abstraction.hpp"
#include <vector>
#include <array>

namespace thermostat {

struct TransitionTable {
    // 51 cells (0..49 real, 50 sink), 2 control inputs (0, 1)
    std::vector<int> table[NUM_CELLS + 1][2];
};

TransitionTable build_transition_table();

// Predecessor operator: returns bitmask or vector of cells in Pre(R)
std::vector<bool> pre_operator(const std::vector<bool>& in_R, const TransitionTable& trans);

// Algorithm 1: Computes the maximal robustly invariant set R*
std::vector<bool> solve_maximal_invariant_set(const TransitionTable& trans, int& iterations);

// Extracts deterministic feedback controller with energy-saving tie-breaker (u=0 preferred)
// Returns array of control actions: 0 = OFF, 1 = ON, -1 = uncontrollable
std::array<int, NUM_CELLS + 1> extract_controller(const std::vector<bool>& R_star, const TransitionTable& trans);

} // namespace thermostat
