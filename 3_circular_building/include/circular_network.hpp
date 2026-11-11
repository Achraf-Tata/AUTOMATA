#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <cassert>

namespace circular {

inline constexpr double ALPHA    = 0.05;   // Wall heat loss coefficient (1/s)
inline constexpr double BETA     = 0.02;   // Inter-room wall conduction (1/s)
inline constexpr double GAMMA    = 1.0;    // Heater power coefficient (K/s)
inline constexpr double TAU      = 1.0;    // Sampling period (s)
inline constexpr double TE_MIN   = 5.0;    // Minimum external weather (deg C)
inline constexpr double TE_MAX   = 12.0;   // Maximum external weather (deg C)
inline constexpr double SAFE_MIN = 19.0;   // Minimum safe temperature (deg C)
inline constexpr double SAFE_MAX = 22.0;   // Maximum safe temperature (deg C)

inline void step_network_euler(
    std::vector<double>& T,
    const std::vector<int>& u,
    double Te,
    double dt = TAU)
{
    const int N = static_cast<int>(T.size());
    std::vector<double> T_next(N);

    for (int i = 0; i < N; ++i) {
        int left_idx  = (i - 1 + N) % N;
        int right_idx = (i + 1) % N;

        double t_i     = T[i];
        double t_left  = T[left_idx];
        double t_right = T[right_idx];
        double u_i     = static_cast<double>(u[i]);

        double tdot = -ALPHA * (t_i - Te)
                      - BETA * ((t_i - t_left) + (t_i - t_right))
                      + GAMMA * u_i;

        T_next[i] = t_i + dt * tdot;
    }

    T = std::move(T_next);
}

// Propagates extreme trajectories by Kamke-Muller Metzler monotonicity
inline void propagate_extreme_trajectories(
    std::vector<double>& T_lower,
    std::vector<double>& T_upper,
    const std::vector<int>& u,
    double dt = TAU)
{
    // Lower bound trajectory under coldest weather (5 C)
    step_network_euler(T_lower, u, TE_MIN, dt);
    // Upper bound trajectory under warmest weather (12 C)
    step_network_euler(T_upper, u, TE_MAX, dt);
}

inline bool verify_metzler_property() noexcept {
    // Off-diagonal partial derivatives d(T_dot_i) / d(T_j) must be strictly non-negative
    // For neighbors j = i +- 1, d(T_dot_i) / d(T_j) = BETA = 0.02 > 0
    return (BETA > 0.0) && (ALPHA > 0.0) && (1.0 - (ALPHA + 2.0 * BETA) * TAU > 0.0);
}

} // namespace circular
