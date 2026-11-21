#pragma once

#include "thermal_network.hpp"
#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>

namespace circular::abstraction {

using namespace model;

// Discretization parameters from Slide 16 of 42717210-s7467.pdf:
// State space: uniform partition, nd = 10 cells per dimension (cell width 0.5 C)
// Input space: nu = 3 controls {0.0, 0.3, 0.6}
inline constexpr int    ND_CELLS  = 10;
inline constexpr int    SINK_CELL = ND_CELLS; // Index 10 is the absorbing sink cell
inline constexpr double CELL_W    = (SAFE_MAX - SAFE_MIN) / ND_CELLS; // 0.5 deg C

// Assume-Guarantee contract disturbance bounds:
// Room i assumes its neighbors stay in comfort safe set: T_{i-1}, T_{i+1} in [17.0, 22.0] C (Slide 16)
// Disturbance W_i = alpha * (T_{i-1} + T_{i+1}) + beta * Te
inline constexpr double W_MIN = ALPHA * (2.0 * SAFE_MIN) + BETA * TE_NOM; // 0.45 * 34 - 0.045 = 15.255
inline constexpr double W_MAX = ALPHA * (2.0 * SAFE_MAX) + BETA * TE_NOM; // 0.45 * 44 - 0.045 = 19.755

// 1D Uniform Grid Quantizer (Single Responsibility: coordinate to discrete index mapping)
class UniformGrid1D {
public:
    [[nodiscard]] static int quantize(double T) noexcept {
        if (T < SAFE_MIN || T > SAFE_MAX) {
            return SINK_CELL;
        }
        int k = static_cast<int>(std::floor((T - SAFE_MIN) / CELL_W));
        if (k >= ND_CELLS) k = ND_CELLS - 1;
        return k;
    }

    [[nodiscard]] static std::pair<double, double> cell_bounds(int k) noexcept {
        if (k < 0 || k >= ND_CELLS) {
            return {SAFE_MIN, SAFE_MAX};
        }
        double lo = SAFE_MIN + k * CELL_W;
        double hi = lo + CELL_W;
        return {lo, hi};
    }
};

// Transition table storing discrete successor cells: table[cell][u_idx]
struct TransitionTable {
    std::vector<int> table[ND_CELLS + 1][NUM_CONTROLS];
};

// Contract-Based Symbolic Abstraction Generator
class ContractAbstraction {
public:
    // Computes continuous next-interval reachable set under disturbance bounds [W_MIN, W_MAX]
    [[nodiscard]] static std::pair<double, double> step_interval(
        double t_min,
        double t_max,
        int u_idx) noexcept
    {
        double u_val = CONTROLS[u_idx];
        double a_self = 1.0 - 2.0 * ALPHA - BETA - GAMMA * u_val; // 0.055 - 0.09 * u

        double next_lo = a_self * t_min + W_MIN + GAMMA * TH_NOM * u_val;
        double next_hi = a_self * t_max + W_MAX + GAMMA * TH_NOM * u_val;
        return {next_lo, next_hi};
    }

    // Computes the set of successor cells Post(s, u) for a given discrete state s and control action
    [[nodiscard]] static std::vector<int> compute_post(int cell_id, int u_idx);

    // Constructs the complete discrete transition system across all 11 cells and 3 controls
    [[nodiscard]] static TransitionTable build_transition_table();
};

} // namespace circular::abstraction
