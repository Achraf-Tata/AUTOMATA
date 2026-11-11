#pragma once
#include <utility>
#include <cmath>

namespace thermostat {

inline constexpr double ALPHA    = 0.05;   // Heat transfer coefficient (1/s)
inline constexpr double BETA     = 1.0;    // Heater power coefficient (K/s)
inline constexpr double TAU      = 1.0;    // Sampling time (s)
inline constexpr double TE_MIN   = 5.0;    // Minimum external temperature (deg C)
inline constexpr double TE_MAX   = 12.0;   // Maximum external temperature (deg C)
inline constexpr double SAFE_MIN = 19.0;   // Minimum safe temperature (deg C)
inline constexpr double SAFE_MAX = 22.0;   // Maximum safe temperature (deg C)

inline double step_euler(double T, int u, double Te, double dt = TAU) noexcept {
    return (1.0 - ALPHA * dt) * T + ALPHA * dt * Te + BETA * dt * static_cast<double>(u);
}

inline std::pair<double, double> step_interval(
    double T_min, double T_max,
    int u,
    double Te_min = TE_MIN, double Te_max = TE_MAX,
    double dt = TAU) noexcept
{
    // By monotonicity, minimum reachable T occurs at (T_min, Te_min) and maximum at (T_max, Te_max)
    double next_min = step_euler(T_min, u, Te_min, dt);
    double next_max = step_euler(T_max, u, Te_max, dt);
    return {next_min, next_max};
}

inline bool verify_continuous_boundary_invariance() noexcept {
    // At lower boundary (19 deg C) with heater ON (u=1), T_dot must be strictly positive for all Te in [5, 12]
    double tdot_lower = -ALPHA * (SAFE_MIN - TE_MIN) + BETA * 1.0; // Worst case: coldest Te = 5
    if (tdot_lower <= 0.0) return false;

    // At upper boundary (22 deg C) with heater OFF (u=0), T_dot must be strictly negative for all Te in [5, 12]
    double tdot_upper = -ALPHA * (SAFE_MAX - TE_MAX) + BETA * 0.0; // Worst case: warmest Te = 12
    if (tdot_upper >= 0.0) return false;

    return true;
}

} // namespace thermostat
