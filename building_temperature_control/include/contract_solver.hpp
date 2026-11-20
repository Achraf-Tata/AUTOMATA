#pragma once
#include "contract_abstraction.hpp"
#include <vector>
#include <array>

namespace circular {

struct LocalTransitionTable {
    std::vector<int> table[NUM_CELLS + 1][2];
};

LocalTransitionTable build_local_transition_table();

// Predecessor operator for local contract
std::vector<bool> pre_operator(const std::vector<bool>& in_R, const LocalTransitionTable& trans);

// Fixed-point synthesis of local robust invariant contract
std::vector<bool> solve_local_invariant_contract(const LocalTransitionTable& trans, int& iterations);

// Extracts local feedback controller with energy-saving tie-breaker (u=0 preferred)
std::array<int, NUM_CELLS + 1> extract_local_controller(
    const std::vector<bool>& R_star,
    const LocalTransitionTable& trans);

// Verifies Assume-Guarantee contract composition condition: G_i subseteq A_{i+-1}
bool verify_contract_composition(const std::vector<bool>& R_star);

} // namespace circular
