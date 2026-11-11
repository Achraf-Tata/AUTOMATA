#pragma once
#include "thermostat_model.hpp"
#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>

namespace thermostat {

inline constexpr int    NUM_CELLS = 50;
inline constexpr int    SINK_CELL = NUM_CELLS; // Index 50 is sink
inline constexpr double T_MIN     = 15.0;
inline constexpr double T_MAX     = 25.0;
inline constexpr double CELL_W    = (T_MAX - T_MIN) / NUM_CELLS; // 0.2 deg C

inline constexpr int SAFE_CELL_START = 20; // 15.0 + 20 * 0.2 = 19.0 deg C
inline constexpr int SAFE_CELL_END   = 34; // 15.0 + 35 * 0.2 = 22.0 deg C (cells 20..34 inclusive, 15 cells)

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

inline std::vector<int> compute_post_cells(int cell_id, int u) {
    if (cell_id == SINK_CELL) {
        return {SINK_CELL};
    }

    auto [lo, hi] = cell_bounds(cell_id);
    auto [next_lo, next_hi] = step_interval(lo, hi, u);

    std::vector<int> post;
    bool hits_sink = false;

    if (next_lo < T_MIN || next_hi > T_MAX) {
        hits_sink = true;
    }

    int k_start = std::max(0, static_cast<int>(std::floor((next_lo - T_MIN) / CELL_W)));
    int k_end   = std::min(NUM_CELLS - 1, static_cast<int>(std::floor((next_hi - T_MIN) / CELL_W)));

    // Handle floating point exact upper boundary
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

} // namespace thermostat
