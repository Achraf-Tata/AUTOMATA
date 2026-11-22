#include "thermostat.hpp"

#include <algorithm>

namespace thermostat {
namespace {

// ============================================================================
// Fixed-Point Integer Scaling (Why multiply temperatures by 2000?)
// ============================================================================
// In computers, floating-point numbers (double) can have tiny rounding errors,
// like 17.000000000000004 or 17.499999999999996. Near bin edges (17.5°C),
// this could cause a temperature to be assigned to the wrong bin.
//
// To make the math 100% exact and reproducible on any computer:
//   1. Our heat equation weights are given in thousandths (scale = 1000).
//   2. Our temperature bins are half-degrees (bins_per_degree = 2).
//
// Multiplying them together gives:
//   temperature_scale = 1000 * 2 = 2000.
//
// In this scale:
//   17.0 °C becomes exactly 17 * 2000 = 34,000
//   17.5 °C becomes exactly 35,000 (each bin spans exactly 1,000 units)
//   22.0 °C becomes exactly 22 * 2000 = 44,000
//
// All calculations are done with exact integer arithmetic. No rounding errors!
constexpr int temperature_scale = coefficient_scale * bins_per_degree; // 2000
constexpr int safe_lower = min_temperature * temperature_scale;        // 34,000
constexpr int safe_upper = max_temperature * temperature_scale;        // 44,000

// Given the lowest and highest reachable next temperature (in 1/2000 °C units),
// finds all discrete bins [0..9] that overlap with that temperature range.
BinMask successor_bins(int lower, int upper) {
    // If the next temperature can drop below 17°C or rise above 22°C,
    // we mark this transition with the 'unsafe' flag bit.
    const BinMask sink = lower < safe_lower || upper > safe_upper ? unsafe : 0;

    // If the entire reachable range is completely outside [17, 22] °C,
    // no valid bins are reachable; return just the unsafe flag.
    if (upper < safe_lower || lower > safe_upper) {
        return sink;
    }

    // Clip the reachable range to the safe boundary [safe_lower, safe_upper].
    // Each bin spans 1000 integer units, so dividing by 1000 gives the bin index.
    const int first = std::min(bin_count - 1,
                               (std::max(lower, safe_lower) - safe_lower) / coefficient_scale);
    const int last = std::min(bin_count - 1,
                              (std::min(upper, safe_upper) - safe_lower) / coefficient_scale);

    // Turn on all bits from 'first' up to 'last'.
    // Example: if first=2 and last=4, run = 0b111 (length 3), shifted by 2 -> bits 2, 3, 4.
    const unsigned run = (1u << (last - first + 1)) - 1u;
    return static_cast<BinMask>(sink | (run << first));
}

} // namespace

// Maps a continuous temperature to a discrete bin index (0 to 9).
// Example:
//   17.0°C to 17.499°C -> bin 0
//   17.5°C to 17.999°C -> bin 1
//   ...
//   21.5°C to 22.000°C -> bin 9
// Returns std::nullopt if the temperature is outside [17, 22] °C.
std::optional<int> quantize(double temperature) {
    if (!(min_temperature <= temperature && temperature <= max_temperature)) {
        return std::nullopt;
    }
    const int bin = static_cast<int>((temperature - min_temperature) * bins_per_degree);
    return std::min(bin_count - 1, bin);
}

// ============================================================================
// Discrete Abstraction Construction (Contract Assumption & System Monotonicity)
// ============================================================================
//
// The physical temperature update for room i at step k is:
//   T_i(k+1) = T_i + alpha*(T_{i-1} + T_{i+1} - 2*T_i) + beta*(T_e - T_i) + gamma*(T_h - T_i)*u_i
//
// By regrouping the terms that multiply T_i:
//   T_i(k+1) = self * T_i + alpha * (T_{i-1} + T_{i+1}) + offset
//   where self = 1 - 2*alpha - beta - gamma*u_i
//
// WHY CAN WE JUST TEST THE CORNERS? (System Monotonicity):
//   1. The neighbor coefficient alpha = 0.450 is POSITIVE.
//      Warmer neighbors ALWAYS make room i warmer; colder neighbors make it colder.
//   2. The self coefficient is POSITIVE for all heater settings (0.055, 0.028, 0.001).
//      A warmer initial room temperature ALWAYS leads to a warmer next temperature.
//
// Because the system is strictly monotonic (a cooperative dynamical system):
//   - The COLDEST possible next temperature happens when:
//       * Room i is at the coldest edge of its bin
//       * Both neighbors are at their coldest allowed temperature (17°C, from contract)
//   - The WARMEST possible next temperature happens when:
//       * Room i is at the warmest edge of its bin
//       * Both neighbors are at their warmest allowed temperature (22°C, from contract)
//
// Testing just these two extreme corner points mathematically guarantees that ALL
// continuous trajectories in between are fully enclosed.
Transitions build_abstraction() {
    Transitions transitions{};

    for (int action = 0; action < action_count; ++action) {
        // Radiator heating contribution for this action:
        // action 0 (0.0): heating = 0
        // action 1 (0.3): heating = 90 * 3 / 10 = 27
        // action 2 (0.6): heating = 90 * 6 / 10 = 54
        const int heating = heater_weight * heater_tenths[action] / 10;

        // self weight = 1000 - 2*450 - 45 - heating = 55 - heating
        const int self = coefficient_scale - 2 * neighbor_weight - outside_weight - heating;

        // Constant terms: outside cold air (-1°C) and radiator heat (+50°C)
        const int offset = outside_weight * outside_temperature + heating * heater_temperature;

        // Contract Assumption: both neighbors are guaranteed to stay in [17, 22] °C.
        // We scale by bins_per_degree (2) to match our half-degree units.
        const int cold_offset = (2 * neighbor_weight * min_temperature + offset) * bins_per_degree;
        const int warm_offset = (2 * neighbor_weight * max_temperature + offset) * bins_per_degree;

        for (int bin = 0; bin < bin_count; ++bin) {
            // Left edge of the current bin in half-degrees (e.g. for bin 0: 17 * 2 = 34)
            const int left = min_temperature * bins_per_degree + bin;

            // Coldest possible next state (room at left edge, neighbors at 17°C)
            const int lower = self * left + cold_offset;

            // Warmest possible next state (room at right edge, neighbors at 22°C)
            const int upper = self * (left + 1) + warm_offset;

            // Find all bins that this range covers and store them in the table
            transitions[bin][action] = successor_bins(lower, upper);
        }
    }
    return transitions;
}

} // namespace thermostat
