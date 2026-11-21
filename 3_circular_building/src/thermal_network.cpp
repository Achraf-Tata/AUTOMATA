#include "thermal_network.hpp"
#include <algorithm>

namespace circular::model {

void ThermalNetworkModel::step(std::vector<double>& T, const std::vector<double>& u, double current_Te) const {
    const int N = static_cast<int>(T.size());
    std::vector<double> T_next(N);

    for (int i = 0; i < N; ++i) {
        // Circular periodic indexing: T_0 = T_N, T_{N+1} = T_1
        int left_idx  = (i - 1 + N) % N;
        int right_idx = (i + 1) % N;

        double t_i     = T[i];
        double t_left  = T[left_idx];
        double t_right = T[right_idx];
        double u_i     = (i < static_cast<int>(u.size())) ? u[i] : 0.0;

        // Discrete thermal difference equation from Slide 15:
        // T_i(t+1) = T_i(t) + alpha * (T_{i+1}(t) + T_{i-1}(t) - 2*T_i(t))
        //                   + beta  * (Te - T_i(t))
        //                   + gamma * (Th - T_i(t)) * u_i(t)
        double delta = alpha * (t_left + t_right - 2.0 * t_i)
                     + beta  * (current_Te - t_i)
                     + gamma * (Th - t_i) * u_i;

        T_next[i] = t_i + delta;
    }

    T = std::move(T_next);
}

bool ThermalNetworkModel::verify_metzler_monotonicity() const noexcept {
    // 1. Off-diagonal coupling terms: d(T_i(t+1)) / d(T_{i+-1}) = alpha
    if (alpha <= 0.0) return false;

    // 2. Diagonal self terms: d(T_i(t+1)) / d(T_i) = 1 - 2*alpha - beta - gamma*u_i
    // Must remain strictly positive for all admissible controls u_i in [0, 0.6]
    for (int idx = 0; idx < NUM_CONTROLS; ++idx) {
        double u_val = CONTROLS[idx];
        double diag = 1.0 - 2.0 * alpha - beta - gamma * u_val;
        if (diag <= 0.0) {
            return false;
        }
    }

    return true;
}

void ThermalNetworkModel::propagate_extreme_bounds(
    std::vector<double>& T_lower,
    std::vector<double>& T_upper,
    double Te_cold,
    double Te_warm) const
{
    const int N = static_cast<int>(T_lower.size());
    std::vector<double> u_min(N, CONTROLS[0]);                  // 0.0 (heater OFF for lower bound)
    std::vector<double> u_max(N, CONTROLS[NUM_CONTROLS - 1]);  // 0.6 (maximum heating for upper bound)

    step(T_lower, u_min, Te_cold);
    step(T_upper, u_max, Te_warm);
}

} // namespace circular::model
