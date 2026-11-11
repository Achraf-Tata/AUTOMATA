// =============================================================================
// unicycle_model.hpp — Member 1
//
// Unicycle robot dynamics and interval-arithmetic reachable-set computation.
//
// The robot's state is (x, y, θ):
//   x     — horizontal position in [0, 10]
//   y     — vertical position  in [0, 10]
//   theta — heading angle      in [-π, π]
//
// Controls are:
//   v  — forward speed  chosen from {0.25, 0.625, 1.0}
//   w  — angular speed  chosen from {-1.0, -0.5, 0.0, 0.5, 1.0}
//
// Additive noise w_i ∈ [-0.05, 0.05] affects each state component.
// =============================================================================
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

// ---------------------------------------------------------------------------
// Data types
// ---------------------------------------------------------------------------

/// One point in state space (x, y, theta).
struct State {
    double x;      ///< horizontal position, metres
    double y;      ///< vertical position,   metres
    double theta;  ///< heading, radians in [-π, π]
};

/// One control action chosen from the discrete set.
struct Control {
    double v;  ///< forward speed  ∈ {0.25, 0.625, 1.0}
    double w;  ///< angular speed  ∈ {-1.0, -0.5, 0.0, 0.5, 1.0}
};

/// An axis-aligned box in state space: [x_min,x_max]×[y_min,y_max]×[th_min,th_max].
struct Box {
    double x_min, x_max;
    double y_min, y_max;
    double th_min, th_max;
};

// All 15 (v, w) pairs from the paper (Section 3).
inline constexpr std::array<Control, 15> ALL_CONTROLS = {{
    {0.25,  -1.0}, {0.25,  -0.5}, {0.25,  0.0}, {0.25,  0.5}, {0.25,  1.0},
    {0.625, -1.0}, {0.625, -0.5}, {0.625, 0.0}, {0.625, 0.5}, {0.625, 1.0},
    {1.0,   -1.0}, {1.0,   -0.5}, {1.0,   0.0}, {1.0,   0.5}, {1.0,   1.0}
}};

// ---------------------------------------------------------------------------
// Dynamics
// ---------------------------------------------------------------------------

/// Wraps angle to [-π, π].
/// Used after every theta update to keep the state in the valid range.
inline double wrap_angle(double a) noexcept {
    // shift to [0, 2π), subtract π to get [-π, π)
    constexpr double TWO_PI = 2.0 * M_PI;
    a = std::fmod(a, TWO_PI);
    if (a >  M_PI) a -= TWO_PI;
    if (a < -M_PI) a += TWO_PI;
    return a;
}

/// Advances the state by one time step using Euler integration.
///
/// Dynamics (from paper eq. 1):
///   x_next     = x     + v * cos(theta) + w_x
///   y_next     = y     + v * sin(theta) + w_y
///   theta_next = theta + w_ctrl + w_theta
///
/// @param s    Current state.
/// @param u    Control to apply (v, w).
/// @param noise Additive noise vector [w_x, w_y, w_theta], each in [-0.05, 0.05].
/// @param dt   Time step (default 1.0 — the paper uses unit time).
/// @return     Next state with theta wrapped to [-π, π].
inline State step(const State& s, const Control& u,
                  const std::array<double, 3>& noise,
                  double dt = 1.0) noexcept {
    return {
        s.x     + u.v * std::cos(s.theta) * dt + noise[0],
        s.y     + u.v * std::sin(s.theta) * dt + noise[1],
        wrap_angle(s.theta + u.w * dt + noise[2])
    };
}

// ---------------------------------------------------------------------------
// Interval arithmetic for reachable-set over-approximation
// ---------------------------------------------------------------------------

/// Computes an axis-aligned bounding box that contains ALL next states
/// reachable from any point inside `cell` under control `u` with
/// noise magnitude `noise_bound` on each axis.
///
/// Method: evaluate the nonlinear dynamics at the 8 corners of the
/// angle interval cross x/y interval, take the min/max, then add the
/// noise envelope.  For theta we also check the midpoint to handle
/// cos/sin extrema inside the interval.
///
/// This over-approximation is used by the grid quantizer to find which
/// grid cells a given (cell, control) pair can reach, without having to
/// sample dense trajectories.
///
/// @param cell        The source cell box in state space.
/// @param u           The control applied.
/// @param noise_bound Scalar noise bound (0.05 from paper).
/// @param dt          Time step.
/// @return            Bounding box of all reachable next states.
inline Box compute_reachable_box(const Box& cell, const Control& u,
                                  double noise_bound = 0.05,
                                  double dt = 1.0) noexcept {
    // We sample theta at min, max, and the angle where cos or sin is extremal
    // within the interval to avoid missing interior maxima.
    double x_lo = 1e18, x_hi = -1e18;
    double y_lo = 1e18, y_hi = -1e18;
    double th_lo = cell.th_min + u.w * dt;
    double th_hi = cell.th_max + u.w * dt;

    // Probe a few theta samples for nonlinear cos/sin
    const double thetas[7] = {
        cell.th_min, cell.th_max,
        (cell.th_min + cell.th_max) * 0.5,
        // angles where cos=0 or sin=0 if inside interval
        -M_PI / 2.0, 0.0, M_PI / 2.0, M_PI
    };
    for (double th : thetas) {
        if (th < cell.th_min || th > cell.th_max) continue;
        double dx = u.v * std::cos(th) * dt;
        double dy = u.v * std::sin(th) * dt;
        // x range from [x_min+dx, x_max+dx]
        x_lo = std::min(x_lo, cell.x_min + dx);
        x_hi = std::max(x_hi, cell.x_max + dx);
        y_lo = std::min(y_lo, cell.y_min + dy);
        y_hi = std::max(y_hi, cell.y_max + dy);
    }

    return {
        x_lo  - noise_bound, x_hi  + noise_bound,
        y_lo  - noise_bound, y_hi  + noise_bound,
        wrap_angle(th_lo - noise_bound), wrap_angle(th_hi + noise_bound)
    };
}

} // namespace unicycle
