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

inline constexpr int    NX     = 100;
inline constexpr int    NY     = 100;
inline constexpr int    NTHETA = 30;
inline constexpr int    NCELLS = NX * NY * NTHETA;

inline constexpr double X_MIN  =  0.0,  X_MAX  = 10.0;
inline constexpr double Y_MIN  =  0.0,  Y_MAX  = 10.0;
inline constexpr double TH_MIN = -M_PI, TH_MAX =  M_PI;

inline constexpr double DX     = (X_MAX  - X_MIN)  / NX;
inline constexpr double DY     = (Y_MAX  - Y_MIN)  / NY;
inline constexpr double DTHETA = (TH_MAX - TH_MIN) / NTHETA;

struct RegionConfig {
    double r1_x0 = 4.0,  r1_x1 = 5.0,  r1_y0 = 8.5,  r1_y1 = 9.5;
    double r2_x0 = 8.5,  r2_x1 = 9.5,  r2_y0 = 2.0,  r2_y1 = 3.0;
    double r3_x0 = 2.0,  r3_x1 = 3.0,  r3_y0 = 0.5,  r3_y1 = 1.5;
    double r4_x0 = 3.0,  r4_x1 = 7.0,  r4_y0 = 3.0,  r4_y1 = 7.0;
};

struct GridCoord {
    int c1;
    int c2;
    int c3;
};

inline GridCoord quantize(const unicycle::State& s) noexcept {
    int c1 = static_cast<int>(std::floor((s.x - X_MIN) / DX));
    int c2 = static_cast<int>(std::floor((s.y - Y_MIN) / DY));
    double th = s.theta;
    while (th >  M_PI) th -= 2.0 * M_PI;
    while (th < -M_PI) th += 2.0 * M_PI;
    int c3 = static_cast<int>(std::floor((th - TH_MIN) / DTHETA));
    return {
        std::clamp(c1, 0, NX     - 1),
        std::clamp(c2, 0, NY     - 1),
        std::clamp(c3, 0, NTHETA - 1)
    };
}

inline unicycle::State cell_center(const GridCoord& c) noexcept {
    return {
        X_MIN  + (c.c1 + 0.5) * DX,
        Y_MIN  + (c.c2 + 0.5) * DY,
        TH_MIN + (c.c3 + 0.5) * DTHETA
    };
}

inline int flat_index(int c1, int c2, int c3) noexcept {
    return c1 * (NY * NTHETA) + c2 * NTHETA + c3;
}

inline int flat_index(const GridCoord& c) noexcept {
    return flat_index(c.c1, c.c2, c.c3);
}

inline int get_region_label(double x, double y, const RegionConfig& cfg) noexcept {
    if (x >= cfg.r4_x0 && x <= cfg.r4_x1 && y >= cfg.r4_y0 && y <= cfg.r4_y1) {
        return 5;
    }
    if (x >= cfg.r1_x0 && x <= cfg.r1_x1 && y >= cfg.r1_y0 && y <= cfg.r1_y1) {
        return 2;
    }
    if (x >= cfg.r2_x0 && x <= cfg.r2_x1 && y >= cfg.r2_y0 && y <= cfg.r2_y1) {
        return 3;
    }
    if (x >= cfg.r3_x0 && x <= cfg.r3_x1 && y >= cfg.r3_y0 && y <= cfg.r3_y1) {
        return 4;
    }
    return 1;
}

struct AbstractionTables {
    std::vector<int>     c1_min;
    std::vector<int>     c1_max;
    std::vector<uint8_t> valid1;

    std::vector<int>     c2_min;
    std::vector<int>     c2_max;
    std::vector<uint8_t> valid2;

    std::vector<int>     c3_min;
    std::vector<int>     c3_max;

    inline int get_c1_min(int i1, int i3, int s) const noexcept {
        return c1_min[(i1 * NTHETA + i3) * 15 + s];
    }
    inline int get_c1_max(int i1, int i3, int s) const noexcept {
        return c1_max[(i1 * NTHETA + i3) * 15 + s];
    }
    inline bool get_valid1(int i1, int i3, int s) const noexcept {
        return valid1[(i1 * NTHETA + i3) * 15 + s] != 0;
    }

    inline int get_c2_min(int i2, int i3, int s) const noexcept {
        return c2_min[(i2 * NTHETA + i3) * 15 + s];
    }
    inline int get_c2_max(int i2, int i3, int s) const noexcept {
        return c2_max[(i2 * NTHETA + i3) * 15 + s];
    }
    inline bool get_valid2(int i2, int i3, int s) const noexcept {
        return valid2[(i2 * NTHETA + i3) * 15 + s] != 0;
    }

    inline int get_c3_min(int i3, int s) const noexcept {
        return c3_min[i3 * 15 + s];
    }
    inline int get_c3_max(int i3, int s) const noexcept {
        return c3_max[i3 * 15 + s];
    }
};

inline AbstractionTables precompute_abstraction_tables(double noise_bound = 0.05, double dt = 1.0) {
    AbstractionTables tab;
    tab.c1_min.resize(NX * NTHETA * 15);
    tab.c1_max.resize(NX * NTHETA * 15);
    tab.valid1.resize(NX * NTHETA * 15);

    tab.c2_min.resize(NY * NTHETA * 15);
    tab.c2_max.resize(NY * NTHETA * 15);
    tab.valid2.resize(NY * NTHETA * 15);

    tab.c3_min.resize(NTHETA * 15);
    tab.c3_max.resize(NTHETA * 15);

    double d_x[3] = { DX, DY, DTHETA };
    double d_w[3] = { 2.0 * noise_bound, 2.0 * noise_bound, 2.0 * noise_bound };

    double d_x_succ[3][15];
    for (int s = 0; s < 15; ++s) {
        double u0 = unicycle::ALL_CONTROLS[s].v;
        d_x_succ[0][s] = 0.5 * (d_x[0] + dt * std::abs(u0) * d_x[2]) + 0.5 * dt * d_w[0];
        d_x_succ[1][s] = 0.5 * (d_x[1] + dt * std::abs(u0) * d_x[2]) + 0.5 * dt * d_w[1];
        d_x_succ[2][s] = 0.5 * d_x[2] + 0.5 * dt * d_w[2];
    }

    for (int s = 0; s < 15; ++s) {
        double u1 = unicycle::ALL_CONTROLS[s].w;
        for (int i3 = 0; i3 < NTHETA; ++i3) {
            double x3_c = TH_MIN + (i3 + 0.5) * DTHETA;
            double r_min = x3_c + dt * u1 - d_x_succ[2][s];
            double r_max = x3_c + dt * u1 + d_x_succ[2][s];

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

            int lo = static_cast<int>(std::floor((r_min - TH_MIN) / DTHETA));
            int hi = static_cast<int>(std::ceil((r_max - TH_MIN) / DTHETA)) - 1;
            tab.c3_min[i3 * 15 + s] = lo;
            tab.c3_max[i3 * 15 + s] = hi;
        }
    }

    for (int s = 0; s < 15; ++s) {
        double u0 = unicycle::ALL_CONTROLS[s].v;
        for (int i3 = 0; i3 < NTHETA; ++i3) {
            double x3_c = TH_MIN + (i3 + 0.5) * DTHETA;
            double delta = dt * u0 * std::cos(x3_c);
            for (int i1 = 0; i1 < NX; ++i1) {
                double x1_c = X_MIN + (i1 + 0.5) * DX;
                double r_min = x1_c + delta - d_x_succ[0][s];
                double r_max = x1_c + delta + d_x_succ[0][s];

                int idx = (i1 * NTHETA + i3) * 15 + s;
                tab.valid1[idx] = (r_min >= X_MIN && r_max <= X_MAX) ? 1 : 0;
                int lo = static_cast<int>(std::floor((r_min - X_MIN) / DX));
                int hi = static_cast<int>(std::ceil((r_max - X_MIN) / DX)) - 1;
                tab.c1_min[idx] = std::clamp(lo, 0, NX - 1);
                tab.c1_max[idx] = std::clamp(hi, 0, NX - 1);
            }
        }
    }

    for (int s = 0; s < 15; ++s) {
        double u0 = unicycle::ALL_CONTROLS[s].v;
        for (int i3 = 0; i3 < NTHETA; ++i3) {
            double x3_c = TH_MIN + (i3 + 0.5) * DTHETA;
            double delta = dt * u0 * std::sin(x3_c);
            for (int i2 = 0; i2 < NY; ++i2) {
                double x2_c = Y_MIN + (i2 + 0.5) * DY;
                double r_min = x2_c + delta - d_x_succ[1][s];
                double r_max = x2_c + delta + d_x_succ[1][s];

                int idx = (i2 * NTHETA + i3) * 15 + s;
                tab.valid2[idx] = (r_min >= Y_MIN && r_max <= Y_MAX) ? 1 : 0;
                int lo = static_cast<int>(std::floor((r_min - Y_MIN) / DY));
                int hi = static_cast<int>(std::ceil((r_max - Y_MIN) / DY)) - 1;
                tab.c2_min[idx] = std::clamp(lo, 0, NY - 1);
                tab.c2_max[idx] = std::clamp(hi, 0, NY - 1);
            }
        }
    }

    return tab;
}

} // namespace grid
