#pragma once

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "unicycle_model.hpp"
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace grid {

using namespace std;
using namespace unicycle;

// Grid discretization resolution for the robot navigation benchmark:
// 100 cells along X, 100 cells along Y, 30 angle bins along theta.
// Total spatial cells = 100 * 100 * 30 = 300,000 cells.
inline constexpr int GRID_CELLS_X        = 100;
inline constexpr int GRID_CELLS_Y        = 100;
inline constexpr int GRID_CELLS_THETA    = 30;
inline constexpr int TOTAL_SPATIAL_CELLS = GRID_CELLS_X * GRID_CELLS_Y * GRID_CELLS_THETA;

// Short aliases matching the course paper notation
inline constexpr int NX     = GRID_CELLS_X;
inline constexpr int NY     = GRID_CELLS_Y;
inline constexpr int NTHETA = GRID_CELLS_THETA;
inline constexpr int NCELLS = TOTAL_SPATIAL_CELLS;

// Continuous state space boundaries: [0, 10] m x [0, 10] m x [-pi, pi] rad
inline constexpr double COORDINATE_MIN_X =  0.0;
inline constexpr double COORDINATE_MAX_X = 10.0;
inline constexpr double COORDINATE_MIN_Y =  0.0;
inline constexpr double COORDINATE_MAX_Y = 10.0;
inline constexpr double ANGLE_MIN_THETA  = -M_PI;
inline constexpr double ANGLE_MAX_THETA  =  M_PI;

inline constexpr double X_MIN  = COORDINATE_MIN_X;
inline constexpr double X_MAX  = COORDINATE_MAX_X;
inline constexpr double Y_MIN  = COORDINATE_MIN_Y;
inline constexpr double Y_MAX  = COORDINATE_MAX_Y;
inline constexpr double TH_MIN = ANGLE_MIN_THETA;
inline constexpr double TH_MAX = ANGLE_MAX_THETA;

// Cell dimensions: dx = 0.1 m, dy = 0.1 m, dtheta = 2*pi/30 = 0.2094 rad
inline constexpr double CELL_WIDTH_DX     = (COORDINATE_MAX_X - COORDINATE_MIN_X) / GRID_CELLS_X;
inline constexpr double CELL_WIDTH_DY     = (COORDINATE_MAX_Y - COORDINATE_MIN_Y) / GRID_CELLS_Y;
inline constexpr double CELL_WIDTH_DTHETA = (ANGLE_MAX_THETA - ANGLE_MIN_THETA)   / GRID_CELLS_THETA;

inline constexpr double DX     = CELL_WIDTH_DX;
inline constexpr double DY     = CELL_WIDTH_DY;
inline constexpr double DTHETA = CELL_WIDTH_DTHETA;

// Rectangular regions defined in the benchmark arena:
// R1: Waypoint 1 (top center)
// R2: Waypoint 2 (bottom right)
// R3: Goal region (bottom left)
// R4: Obstacle region (large center square [3, 7] x [3, 7])
struct RegionConfig {
    double r1_x0 = 4.0; double r1_x1 = 5.0; double r1_y0 = 8.5; double r1_y1 = 9.5;
    double r2_x0 = 8.5; double r2_x1 = 9.5; double r2_y0 = 2.0; double r2_y1 = 3.0;
    double r3_x0 = 2.0; double r3_x1 = 3.0; double r3_y0 = 0.5; double r3_y1 = 1.5;
    double r4_x0 = 3.0; double r4_x1 = 7.0; double r4_y0 = 3.0; double r4_y1 = 7.0;
};

// 3D discrete grid coordinates (cell_x, cell_y, cell_theta)
struct GridCoord {
    int cell_x = 0;
    int cell_y = 0;
    int cell_theta = 0;
};

// Quantizes continuous robot state (x, y, theta) into discrete integer cell coordinates
GridCoord quantize(const State& continuous_state) noexcept;

inline bool is_in_workspace(const State& state) noexcept {
    return std::isfinite(state.x) && std::isfinite(state.y) && std::isfinite(state.theta)
        && state.x >= X_MIN && state.x <= X_MAX
        && state.y >= Y_MIN && state.y <= Y_MAX;
}

// Computes the continuous center position of a discrete grid cell
State cell_center(const GridCoord& grid_coordinate) noexcept;

// Converts 3D grid cell coordinates to a flat 1D array index (row-major order)
int flat_index(int cell_x, int cell_y, int cell_theta) noexcept;
int flat_index(const GridCoord& grid_coordinate) noexcept;

// Classifies continuous position (x, y) into its region observation label:
// 5 = Obstacle R4 (highest priority for safety)
// 2 = Waypoint R1
// 3 = Waypoint R2
// 4 = Goal R3
// 1 = Free space
int get_region_label(double x, double y, const RegionConfig& config) noexcept;

// Precomputed range tables using the coordinate trick from Antoine Girard's paper.
// Because the robot's motion equations can be split across X, Y, and angle theta,
// we can calculate the possible next boxes along each dimension independently:
//   X-axis possible range: [c1_min, c1_max]
//   Y-axis possible range: [c2_min, c2_max]
//   Theta-axis possible range: [c3_min, c3_max]
struct AbstractionTables {
    vector<int>     c1_min;
    vector<int>     c1_max;
    vector<uint8_t> valid1; // true if successor interval lies entirely within arena bounds

    vector<int>     c2_min;
    vector<int>     c2_max;
    vector<uint8_t> valid2; // true if successor interval lies entirely within arena bounds

    vector<int>     c3_min;
    vector<int>     c3_max;

    [[nodiscard]] inline int get_c1_min(int i1, int i3, int s) const noexcept {
        return c1_min[(i1 * NTHETA + i3) * 15 + s];
    }
    [[nodiscard]] inline int get_c1_max(int i1, int i3, int s) const noexcept {
        return c1_max[(i1 * NTHETA + i3) * 15 + s];
    }
    [[nodiscard]] inline bool get_valid1(int i1, int i3, int s) const noexcept {
        return valid1[(i1 * NTHETA + i3) * 15 + s] != 0;
    }

    [[nodiscard]] inline int get_c2_min(int i2, int i3, int s) const noexcept {
        return c2_min[(i2 * NTHETA + i3) * 15 + s];
    }
    [[nodiscard]] inline int get_c2_max(int i2, int i3, int s) const noexcept {
        return c2_max[(i2 * NTHETA + i3) * 15 + s];
    }
    [[nodiscard]] inline bool get_valid2(int i2, int i3, int s) const noexcept {
        return valid2[(i2 * NTHETA + i3) * 15 + s] != 0;
    }

    [[nodiscard]] inline int get_c3_min(int i3, int s) const noexcept {
        return c3_min[i3 * 15 + s];
    }
    [[nodiscard]] inline int get_c3_max(int i3, int s) const noexcept {
        return c3_max[i3 * 15 + s];
    }
};

// Precomputes the 1D interval abstraction tables for a given disturbance bound and sampling period
AbstractionTables precompute_abstraction_tables(double noise_bound = 0.05, double dt = 1.0);

} // namespace grid
