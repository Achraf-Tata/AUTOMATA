#pragma once
#include "circular_network.hpp"
#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>

namespace circular {

inline constexpr int    NUM_CELLS = 50;
inline constexpr int    SINK_CELL = NUM_CELLS; // Index 50 is sink
inline constexpr double T_MIN     = 15.0;
inline constexpr double T_MAX     = 25.0;
inline constexpr double CELL_W    = (T_MAX - T_MIN) / NUM_CELLS; // 0.2 deg C

inline constexpr int SAFE_CELL_START = 20; // 15.0 + 20 * 0.2 = 19.0 deg C
inline constexpr int SAFE_CELL_END   = 34; // 15.0 + 35 * 0.2 = 22.0 deg C (15 cells)

// Assume-Guarantee contract disturbance bounds:
// Room i assumes neighbors T_{i-1}, T_{i+1} in [19.0, 22.0] C
inline constexpr double W_COUPLING_MIN = BETA * TAU * (2.0 * SAFE_MIN); // 0.02 * 38 = 0.76
inline constexpr double W_COUPLING_MAX = BETA * TAU * (2.0 * SAFE_MAX); // 0.02 * 44 = 0.88
inline constexpr double W_WEATHER_MIN  = ALPHA * TAU * TE_MIN;          // 0.05 * 5  = 0.25
inline constexpr double W_WEATHER_MAX  = ALPHA * TAU * TE_MAX;          // 0.05 * 12 = 0.60

inline constexpr double W_MIN = W_WEATHER_MIN + W_COUPLING_MIN; // 1.01
inline constexpr double W_MAX = W_WEATHER_MAX + W_COUPLING_MAX; // 1.48

inline constexpr double COEFF_SELF = 1.0 - (ALPHA + 2.0 * BETA) * TAU; // 0.91

inline int quantize(double T) noexcept {
    if (T < T_MIN || T > T_MAX) {
        return SINK_CELL;
    }
    int k = static_cast<int>(std::floor((T - T_MIN) / CELL_W));
    if (k >= NUM_CELLS) k = NUM_CELLS - 1;
    return k;
}

inline std::pair<double, double> cell_bounds(int k) noexcept {
    if (k < 0 || k >= NUM_CELLS) {
        return {T_MIN, T_MAX};
    }
    double lo = T_MIN + k * CELL_W;
    double hi = lo + CELL_W;
    return {lo, hi};
}

inline std::pair<double, double> step_local_interval(double t_min, double t_max, int u) noexcept {
    double next_lo = COEFF_SELF * t_min + W_MIN + GAMMA * TAU * static_cast<double>(u);
    double next_hi = COEFF_SELF * t_max + W_MAX + GAMMA * TAU * static_cast<double>(u);
    return {next_lo, next_hi};
}

inline std::vector<int> compute_local_post(int cell_id, int u) {
    if (cell_id == SINK_CELL) {
        return {SINK_CELL};
    }

    auto [lo, hi] = cell_bounds(cell_id);
    auto [next_lo, next_hi] = step_local_interval(lo, hi, u);

    std::vector<int> post;
    bool hits_sink = false;

    if (next_lo < T_MIN || next_hi > T_MAX) {
        hits_sink = true;
    }

    int k_start = std::max(0, static_cast<int>(std::floor((next_lo - T_MIN) / CELL_W)));
    int k_end   = std::min(NUM_CELLS - 1, static_cast<int>(std::floor((next_hi - T_MIN) / CELL_W)));

    if (next_hi >= T_MIN + (k_end + 1) * CELL_W && k_end < NUM_CELLS - 1) {
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

} // namespace circular
