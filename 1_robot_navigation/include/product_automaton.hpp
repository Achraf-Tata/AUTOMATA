#pragma once
#include "grid_quantizer.hpp"
#include <vector>
#include <cstdint>
#include <limits>

namespace automaton {

inline constexpr int   NDFA   = 5;
inline constexpr int   NTOTAL = NDFA * grid::NCELLS;
inline constexpr float INF    = 1e9f;

int dfa_transition(int q, int label) noexcept;
std::vector<float> init_value_function();
std::vector<uint8_t> init_controller();

inline int product_index(int q, int cell_flat) noexcept {
    return (q - 1) * grid::NCELLS + cell_flat;
}

} // namespace automaton
