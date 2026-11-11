// =============================================================================
// grid_quantizer.hpp — Member 2
//
// Maps continuous (x, y, θ) states to integer grid indices and back.
// Also determines which of the four labelled regions the robot is in,
// and precomputes per-action reachable-cell offsets for fast value iteration.
//
// Grid dimensions (from paper Section 3):
//   x  : 100 cells over [0, 10],  cell width  DX = 0.1
//   y  : 100 cells over [0, 10],  cell height DY = 0.1
//   θ  :  30 cells over [-π, π],  cell width  DTHETA = 2π/30
//
// Flat cell index: c1 * NY * NTHETA + c2 * NTHETA + c3
// =============================================================================
#pragma once
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include "unicycle_model.hpp"
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace grid {

// ---------------------------------------------------------------------------
// Grid constants
// ---------------------------------------------------------------------------

inline constexpr int    NX     = 100;
inline constexpr int    NY     = 100;
inline constexpr int    NTHETA = 30;
inline constexpr int    NCELLS = NX * NY * NTHETA;  ///< 300 000 cells

inline constexpr double X_MIN  =  0.0,  X_MAX  = 10.0;
inline constexpr double Y_MIN  =  0.0,  Y_MAX  = 10.0;
inline constexpr double TH_MIN = -M_PI, TH_MAX =  M_PI;

inline constexpr double DX     = (X_MAX  - X_MIN)  / NX;      ///< 0.1
inline constexpr double DY     = (Y_MAX  - Y_MIN)  / NY;      ///< 0.1
inline constexpr double DTHETA = (TH_MAX - TH_MIN) / NTHETA;  ///< ≈ 0.2094 rad

// ---------------------------------------------------------------------------
// Region configuration — user-editable from the GUI
// ---------------------------------------------------------------------------

/// Each region is an axis-aligned rectangle in the (x,y) plane.
/// Default values are taken from Section 3 of the paper.
struct RegionConfig {
    // R1: target visited first  — default [4,5] × [8.5,9.5]
    double r1_x0 = 4.0,  r1_x1 = 5.0,  r1_y0 = 8.5,  r1_y1 = 9.5;
    // R2: target visited second — default [8.5,9.5] × [2,3]
    double r2_x0 = 8.5,  r2_x1 = 9.5,  r2_y0 = 2.0,  r2_y1 = 3.0;
    // R3: target (goal)         — default [2,3] × [0.5,1.5]
    double r3_x0 = 2.0,  r3_x1 = 3.0,  r3_y0 = 0.5,  r3_y1 = 1.5;
    // R4: obstacle to avoid     — default [3,7] × [3,7]
    double r4_x0 = 3.0,  r4_x1 = 7.0,  r4_y0 = 3.0,  r4_y1 = 7.0;
};

// ---------------------------------------------------------------------------
// Integer grid coordinate
// ---------------------------------------------------------------------------

/// Three-component grid index (all clamped to valid ranges).
struct GridCoord {
    int c1;  ///< x  index ∈ [0, NX-1]
    int c2;  ///< y  index ∈ [0, NY-1]
    int c3;  ///< θ  index ∈ [0, NTHETA-1]
};

/// Converts a continuous state to the nearest grid cell using floor division.
/// Values outside the domain are clamped to the boundary cell.
inline GridCoord quantize(const unicycle::State& s) noexcept {
    int c1 = static_cast<int>(std::floor((s.x     - X_MIN)  / DX));
    int c2 = static_cast<int>(std::floor((s.y     - Y_MIN)  / DY));
    int c3 = static_cast<int>(std::floor((s.theta - TH_MIN) / DTHETA));
    return {
        std::clamp(c1, 0, NX     - 1),
        std::clamp(c2, 0, NY     - 1),
        std::clamp(c3, 0, NTHETA - 1)
    };
}

/// Returns the centre of cell (c1, c2, c3) in continuous state space.
/// Used when simulating a trajectory: we start the robot at the cell centre.
inline unicycle::State cell_center(const GridCoord& c) noexcept {
    return {
        X_MIN  + (c.c1 + 0.5) * DX,
        Y_MIN  + (c.c2 + 0.5) * DY,
        TH_MIN + (c.c3 + 0.5) * DTHETA
    };
}

/// Converts (c1, c2, c3) to a flat array index.
inline int flat_index(int c1, int c2, int c3) noexcept {
    return c1 * NY * NTHETA + c2 * NTHETA + c3;
}

/// Overload taking a GridCoord.
inline int flat_index(const GridCoord& c) noexcept {
    return flat_index(c.c1, c.c2, c.c3);
}

// ---------------------------------------------------------------------------
// Region labelling
// ---------------------------------------------------------------------------

/// Returns the atomic proposition label for position (x, y):
///   1  → inside R1
///   2  → inside R2
///   3  → inside R3
///   4  → inside R4 (obstacle)
///   0  → none of the above
///
/// If the robot is simultaneously inside R4 and another region,
/// R4 wins (obstacle takes priority).
inline int get_region_label(double x, double y,
                             const RegionConfig& cfg) noexcept {
    bool in_r4 = (x >= cfg.r4_x0 && x <= cfg.r4_x1 &&
                  y >= cfg.r4_y0 && y <= cfg.r4_y1);
    if (in_r4) return 4;

    if (x >= cfg.r1_x0 && x <= cfg.r1_x1 &&
        y >= cfg.r1_y0 && y <= cfg.r1_y1) return 1;
    if (x >= cfg.r2_x0 && x <= cfg.r2_x1 &&
        y >= cfg.r2_y0 && y <= cfg.r2_y1) return 2;
    if (x >= cfg.r3_x0 && x <= cfg.r3_x1 &&
        y >= cfg.r3_y0 && y <= cfg.r3_y1) return 3;
    return 0;
}

// ---------------------------------------------------------------------------
// Action footprint precomputation
// ---------------------------------------------------------------------------

/// For each of the 15 controls and each theta cell, stores the range of
/// (dc1, dc2) offsets that could be reached.  This lets the backward
/// reachability loop iterate only over relevant predecessor cells instead
/// of scanning all 300 000 cells.
///
/// dc1_min..dc1_max, dc2_min..dc2_max are the bounding box of integer
/// offsets in the x/y grid dimensions.
struct ActionFootprint {
    int dc1_min, dc1_max;  ///< range of x-cell offsets
    int dc2_min, dc2_max;  ///< range of y-cell offsets
    int dc3_min, dc3_max;  ///< range of θ-cell offsets
};

/// Table indexed [control_index][theta_cell] → ActionFootprint.
using FootprintTable = std::array<std::array<ActionFootprint, NTHETA>,
                                   unicycle::ALL_CONTROLS.size()>;

/// Fills the footprint table using compute_reachable_box from unicycle_model.
/// Called once at program start (takes < 1 ms).
inline FootprintTable precompute_action_footprints(double noise_bound = 0.05,
                                                    double dt = 1.0) {
    FootprintTable table;
    for (int ui = 0; ui < static_cast<int>(unicycle::ALL_CONTROLS.size()); ++ui) {
        const auto& u = unicycle::ALL_CONTROLS[ui];
        for (int c3 = 0; c3 < NTHETA; ++c3) {
            // Build the box for this single theta cell
            double th0 = TH_MIN + c3 * DTHETA;
            unicycle::Box cell_box {
                X_MIN, X_MAX,          // x — full range (conservative)
                Y_MIN, Y_MAX,          // y — full range
                th0, th0 + DTHETA
            };
            auto rb = unicycle::compute_reachable_box(cell_box, u,
                                                       noise_bound, dt);
            // Convert box to grid offset ranges
            int x_lo = static_cast<int>(std::floor((rb.x_min - X_MIN) / DX));
            int x_hi = static_cast<int>(std::ceil ((rb.x_max - X_MIN) / DX));
            int y_lo = static_cast<int>(std::floor((rb.y_min - Y_MIN) / DY));
            int y_hi = static_cast<int>(std::ceil ((rb.y_max - Y_MIN) / DY));
            // theta offsets
            int t_lo = static_cast<int>(std::floor((rb.th_min - TH_MIN) / DTHETA)) - c3;
            int t_hi = static_cast<int>(std::ceil ((rb.th_max - TH_MIN) / DTHETA)) - c3;

            table[ui][c3] = { x_lo, x_hi, y_lo, y_hi, t_lo, t_hi };
        }
    }
    return table;
}

} // namespace grid
