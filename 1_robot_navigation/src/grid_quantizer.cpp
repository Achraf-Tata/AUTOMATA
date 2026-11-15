#include "grid_quantizer.hpp"
#include <cmath>
#include <algorithm>

namespace grid {

using namespace std;
using namespace unicycle;

// Quantizes continuous robot state (x, y, theta) into discrete integer cell coordinates
GridCoord quantize(const State& continuous_state) noexcept {
    int cell_x = static_cast<int>(floor((continuous_state.x - COORDINATE_MIN_X) / CELL_WIDTH_DX));
    int cell_y = static_cast<int>(floor((continuous_state.y - COORDINATE_MIN_Y) / CELL_WIDTH_DY));

    // Normalize heading angle into [-pi, pi] before quantizing
    double normalized_theta = continuous_state.theta;
    while (normalized_theta >  M_PI) normalized_theta -= 2.0 * M_PI;
    while (normalized_theta < -M_PI) normalized_theta += 2.0 * M_PI;
    int cell_theta = static_cast<int>(floor((normalized_theta - ANGLE_MIN_THETA) / CELL_WIDTH_DTHETA));

    return {
        clamp(cell_x,     0, GRID_CELLS_X     - 1),
        clamp(cell_y,     0, GRID_CELLS_Y     - 1),
        clamp(cell_theta, 0, GRID_CELLS_THETA - 1)
    };
}

// Computes the continuous center position of a discrete grid cell
State cell_center(const GridCoord& grid_coordinate) noexcept {
    return {
        COORDINATE_MIN_X + (grid_coordinate.cell_x     + 0.5) * CELL_WIDTH_DX,
        COORDINATE_MIN_Y + (grid_coordinate.cell_y     + 0.5) * CELL_WIDTH_DY,
        ANGLE_MIN_THETA  + (grid_coordinate.cell_theta + 0.5) * CELL_WIDTH_DTHETA
    };
}

// Converts 3D grid cell coordinates to a flat 1D array index (row-major order)
int flat_index(int cell_x, int cell_y, int cell_theta) noexcept {
    return cell_x * (GRID_CELLS_Y * GRID_CELLS_THETA) + cell_y * GRID_CELLS_THETA + cell_theta;
}

int flat_index(const GridCoord& grid_coordinate) noexcept {
    return flat_index(grid_coordinate.cell_x, grid_coordinate.cell_y, grid_coordinate.cell_theta);
}

// Classifies continuous position (x, y) into its region observation label
int get_region_label(double x, double y, const RegionConfig& config) noexcept {
    // Obstacle R4 has highest priority for safety
    if (x >= config.r4_x0 && x <= config.r4_x1 && y >= config.r4_y0 && y <= config.r4_y1) {
        return 5;
    }
    // Waypoint R1
    if (x >= config.r1_x0 && x <= config.r1_x1 && y >= config.r1_y0 && y <= config.r1_y1) {
        return 2;
    }
    // Waypoint R2
    if (x >= config.r2_x0 && x <= config.r2_x1 && y >= config.r2_y0 && y <= config.r2_y1) {
        return 3;
    }
    // Goal R3
    if (x >= config.r3_x0 && x <= config.r3_x1 && y >= config.r3_y0 && y <= config.r3_y1) {
        return 4;
    }
    return 1; // Free space
}

// Precomputes the 1D interval abstraction tables for a given disturbance bound and sampling period
AbstractionTables precompute_abstraction_tables(double noise_bound, double dt) {
    AbstractionTables tab;
    tab.c1_min.resize(NX * NTHETA * 15);
    tab.c1_max.resize(NX * NTHETA * 15);
    tab.valid1.resize(NX * NTHETA * 15);

    tab.c2_min.resize(NY * NTHETA * 15);
    tab.c2_max.resize(NY * NTHETA * 15);
    tab.valid2.resize(NY * NTHETA * 15);

    tab.c3_min.resize(NTHETA * 15);
    tab.c3_max.resize(NTHETA * 15);

    // Initial cell diameter and disturbance radius
    double d_x[3] = { DX, DY, DTHETA };
    double d_w[3] = { 2.0 * noise_bound, 2.0 * noise_bound, 2.0 * noise_bound };

    // Growth bounds for each of the 15 control inputs
    double d_x_succ[3][15];
    for (int s = 0; s < 15; ++s) {
        double u0 = ALL_CONTROLS[s].v;
        d_x_succ[0][s] = 0.5 * (d_x[0] + dt * abs(u0) * d_x[2]) + 0.5 * dt * d_w[0];
        d_x_succ[1][s] = 0.5 * (d_x[1] + dt * abs(u0) * d_x[2]) + 0.5 * dt * d_w[1];
        d_x_succ[2][s] = 0.5 * d_x[2] + 0.5 * dt * d_w[2];
    }

    // Precompute theta successors: theta' = theta + dt * omega +- d_x_succ[2]
    for (int s = 0; s < 15; ++s) {
        double u1 = ALL_CONTROLS[s].w;
        for (int i3 = 0; i3 < NTHETA; ++i3) {
            double x3_c = TH_MIN + (i3 + 0.5) * DTHETA;
            double r_min = x3_c + dt * u1 - d_x_succ[2][s];
            double r_max = x3_c + dt * u1 + d_x_succ[2][s];

            // Handle wrap-around across [-pi, pi]
            if (r_min < -M_PI && r_max >= -M_PI) {
                r_min += 2.0 * M_PI;
            } else if (r_min < -M_PI && r_max < -M_PI) {
                r_min += 2.0 * M_PI;
                r_max += 2.0 * M_PI;
            } else if (r_max > M_PI && r_min <= M_PI) {
                r_max -= 2.0 * M_PI;
            } else if (r_max > M_PI && r_min > M_PI) {
                r_max -= 2.0 * M_PI;
                r_min -= 2.0 * M_PI;
            }

            int lo = static_cast<int>(floor((r_min - TH_MIN) / DTHETA));
            int hi = static_cast<int>(ceil((r_max - TH_MIN) / DTHETA)) - 1;
            tab.c3_min[i3 * 15 + s] = lo;
            tab.c3_max[i3 * 15 + s] = hi;
        }
    }

    // Precompute X successors: x' = x + dt * v * cos(theta) +- d_x_succ[0]
    for (int s = 0; s < 15; ++s) {
        double u0 = ALL_CONTROLS[s].v;
        for (int i3 = 0; i3 < NTHETA; ++i3) {
            double x3_c = TH_MIN + (i3 + 0.5) * DTHETA;
            double delta = dt * u0 * cos(x3_c);
            for (int i1 = 0; i1 < NX; ++i1) {
                double x1_c = X_MIN + (i1 + 0.5) * DX;
                double r_min = x1_c + delta - d_x_succ[0][s];
                double r_max = x1_c + delta + d_x_succ[0][s];

                int idx = (i1 * NTHETA + i3) * 15 + s;
                tab.valid1[idx] = (r_min >= X_MIN && r_max <= X_MAX) ? 1 : 0;
                int lo = static_cast<int>(floor((r_min - X_MIN) / DX));
                int hi = static_cast<int>(ceil((r_max - X_MIN) / DX)) - 1;
                tab.c1_min[idx] = clamp(lo, 0, NX - 1);
                tab.c1_max[idx] = clamp(hi, 0, NX - 1);
            }
        }
    }

    // Precompute Y successors: y' = y + dt * v * sin(theta) +- d_x_succ[1]
    for (int s = 0; s < 15; ++s) {
        double u0 = ALL_CONTROLS[s].v;
        for (int i3 = 0; i3 < NTHETA; ++i3) {
            double x3_c = TH_MIN + (i3 + 0.5) * DTHETA;
            double delta = dt * u0 * sin(x3_c);
            for (int i2 = 0; i2 < NY; ++i2) {
                double x2_c = Y_MIN + (i2 + 0.5) * DY;
                double r_min = x2_c + delta - d_x_succ[1][s];
                double r_max = x2_c + delta + d_x_succ[1][s];

                int idx = (i2 * NTHETA + i3) * 15 + s;
                tab.valid2[idx] = (r_min >= Y_MIN && r_max <= Y_MAX) ? 1 : 0;
                int lo = static_cast<int>(floor((r_min - Y_MIN) / DY));
                int hi = static_cast<int>(ceil((r_max - Y_MIN) / DY)) - 1;
                tab.c2_min[idx] = clamp(lo, 0, NY - 1);
                tab.c2_max[idx] = clamp(hi, 0, NY - 1);
            }
        }
    }

    return tab;
}

} // namespace grid
