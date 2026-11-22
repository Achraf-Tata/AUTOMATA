#include "thermostat.hpp"

#include <algorithm>

namespace thermostat {
namespace {

// Bin endpoints are in half degrees, so affine corner values are exact in 1/2000 C.
constexpr int temperature_scale = coefficient_scale * bins_per_degree;
constexpr int safe_lower = min_temperature * temperature_scale;
constexpr int safe_upper = max_temperature * temperature_scale;

BinMask successor_bins(int lower, int upper) {
    const BinMask sink = lower < safe_lower || upper > safe_upper ? unsafe : 0;
    if (upper < safe_lower || lower > safe_upper) {
        return sink;
    }

    // After clipping, a closed reachable interval intersects a consecutive run of bins.
    const int first = std::min(bin_count - 1,
                               (std::max(lower, safe_lower) - safe_lower) / coefficient_scale);
    const int last = std::min(bin_count - 1,
                              (std::min(upper, safe_upper) - safe_lower) / coefficient_scale);
    const unsigned run = (1u << (last - first + 1)) - 1u;
    return static_cast<BinMask>(sink | (run << first));
}

} // namespace

std::optional<int> quantize(double temperature) {
    if (!(min_temperature <= temperature && temperature <= max_temperature)) {
        return std::nullopt;
    }
    const int bin = static_cast<int>((temperature - min_temperature) * bins_per_degree);
    return std::min(bin_count - 1, bin);
}

Transitions build_abstraction() {
    Transitions transitions{};
    for (int action = 0; action < action_count; ++action) {
        const int heating = heater_weight * heater_tenths[action] / 10;
        const int self = coefficient_scale - 2 * neighbor_weight - outside_weight - heating;
        const int offset = outside_weight * outside_temperature + heating * heater_temperature;

        // Under the contract, the two neighbors independently range over [17, 22].
        // Use their two coldest values for the lower bound, and two warmest for the upper.
        const int cold_offset = (2 * neighbor_weight * min_temperature + offset) * bins_per_degree;
        const int warm_offset = (2 * neighbor_weight * max_temperature + offset) * bins_per_degree;
        for (int bin = 0; bin < bin_count; ++bin) {
            const int left = min_temperature * bins_per_degree + bin;

            // The self coefficient is positive for all three actions, so these
            // bin endpoints and disturbance extremes enclose every successor.
            const int lower = self * left + cold_offset;
            const int upper = self * (left + 1) + warm_offset;
            transitions[bin][action] = successor_bins(lower, upper);
        }
    }
    return transitions;
}

} // namespace thermostat
