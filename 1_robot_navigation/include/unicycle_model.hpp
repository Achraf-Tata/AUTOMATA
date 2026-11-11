#pragma once
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <array>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace unicycle {

struct State {
    double x;
    double y;
    double theta;
};

struct Control {
    double v;
    double w;
};

struct Box {
    double x_min, x_max;
    double y_min, y_max;
    double th_min, th_max;
};

inline constexpr std::array<Control, 15> ALL_CONTROLS = {{
    {0.25,  -1.0}, {0.625, -1.0}, {1.0,   -1.0},
    {0.25,  -0.5}, {0.625, -0.5}, {1.0,   -0.5},
    {0.25,   0.0}, {0.625,  0.0}, {1.0,    0.0},
    {0.25,   0.5}, {0.625,  0.5}, {1.0,    0.5},
    {0.25,   1.0}, {0.625,  1.0}, {1.0,    1.0}
}};

inline double wrap_angle(double a) noexcept {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

inline State step(const State& s, const Control& u,
                  const std::array<double, 3>& noise,
                  double dt = 1.0) noexcept {
    return {
        s.x     + dt * u.v * std::cos(s.theta) + dt * noise[0],
        s.y     + dt * u.v * std::sin(s.theta) + dt * noise[1],
        wrap_angle(s.theta + dt * u.w          + dt * noise[2])
    };
}

inline Box compute_reachable_box(const Box& cell, const Control& u,
                                  double noise_bound = 0.05,
                                  double dt = 1.0) noexcept {
    double x_lo = 1e18, x_hi = -1e18;
    double y_lo = 1e18, y_hi = -1e18;
    double th_lo = cell.th_min + dt * u.w;
    double th_hi = cell.th_max + dt * u.w;

    const double thetas[7] = {
        cell.th_min, cell.th_max,
        (cell.th_min + cell.th_max) * 0.5,
        -M_PI / 2.0, 0.0, M_PI / 2.0, M_PI
    };
    for (double th : thetas) {
        if (th < cell.th_min || th > cell.th_max) continue;
        double dx = dt * u.v * std::cos(th);
        double dy = dt * u.v * std::sin(th);
        x_lo = std::min(x_lo, cell.x_min + dx);
        x_hi = std::max(x_hi, cell.x_max + dx);
        y_lo = std::min(y_lo, cell.y_min + dy);
        y_hi = std::max(y_hi, cell.y_max + dy);
    }

    return {
        x_lo  - dt * noise_bound, x_hi  + dt * noise_bound,
        y_lo  - dt * noise_bound, y_hi  + dt * noise_bound,
        wrap_angle(th_lo - dt * noise_bound), wrap_angle(th_hi + dt * noise_bound)
    };
}

} // namespace unicycle
