#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <cassert>

namespace circular::model {

// Exact physical constants from Slide 15 of 42717210-s7467.pdf (Prof. Adnane Saoud):
// T_i(t+1) = T_i(t) + alpha * (T_{i+1}(t) + T_{i-1}(t) - 2 * T_i(t))
//                   + beta  * (T_e - T_i(t))
//                   + gamma * (T_h - T_i(t)) * u_i(t)
inline constexpr double ALPHA    = 0.45;   // Inter-room wall conduction coefficient
inline constexpr double BETA     = 0.045;  // Exterior wall heat loss coefficient
inline constexpr double GAMMA    = 0.09;   // Heater heat transfer efficiency
inline constexpr double TE_NOM   = -1.0;   // Outside temperature (deg C)
inline constexpr double TH_NOM   = 50.0;   // Heater radiator temperature (deg C)

// Comfort specification from Slide 16: S = [17.0, 22.0]^N deg C
inline constexpr double SAFE_MIN = 17.0;   // Minimum comfort temperature (deg C)
inline constexpr double SAFE_MAX = 22.0;   // Maximum comfort temperature (deg C)

// Quantized input controls from Slide 16: nu = 3 symbolic inputs {0.0, 0.3, 0.6}
inline constexpr int    NUM_CONTROLS = 3;
inline constexpr double CONTROLS[NUM_CONTROLS] = {0.0, 0.3, 0.6};

// Thermal network physics engine implementing continuous/discrete coupled room dynamics
class ThermalNetworkModel {
public:
    double alpha = ALPHA;
    double beta  = BETA;
    double gamma = GAMMA;
    double Te    = TE_NOM;
    double Th    = TH_NOM;

    constexpr ThermalNetworkModel() = default;

    // Evaluates the single-step discrete difference equation for the entire circular ring
    // with periodic boundary conditions T_0 = T_N and T_{N+1} = T_1
    void step(std::vector<double>& T, const std::vector<double>& u, double current_Te) const;

    // Analytically and numerically verifies the Metzler monotonicity property (Kamke-Muller theorem)
    // Ensures all off-diagonal and diagonal partial derivatives are strictly positive:
    // d(T_i(t+1)) / d(T_{i+-1}) = alpha = 0.45 > 0
    // d(T_i(t+1)) / d(T_i) = 1 - 2*alpha - beta - gamma*u_i = 0.055 - 0.09*u_i > 0 for all u in {0, 0.3, 0.6}
    [[nodiscard]] bool verify_metzler_monotonicity() const noexcept;

    // Propagates tight upper and lower bounds trajectories using Kamke-Muller corner trajectories
    void propagate_extreme_bounds(
        std::vector<double>& T_lower,
        std::vector<double>& T_upper,
        double Te_cold,
        double Te_warm) const;
};

} // namespace circular::model
