#include "thermostat.hpp"

#include <bit>
#include <chrono>
#include <cmath>

namespace thermostat {

Policy synthesize(const Transitions& transitions) {
    BinMask candidates = all_bins;
    while (true) {
        Policy next{};
        for (int bin = 0; bin < bin_count; ++bin) {
            if ((candidates & bin_bit(bin)) == 0) {
                continue;
            }

            for (int action = 0; action < action_count; ++action) {
                const BinMask successors = transitions[bin][action];
                // Every successor must remain in the candidate set. A missing
                // transition would block execution, so it cannot certify an action.
                if (successors != 0 && (successors & candidates) == successors) {
                    next.allowed[bin] |= action_bit(action);
                }
            }
            if (next.allowed[bin] != 0) {
                next.winning |= bin_bit(bin);
            }
        }
        if (next.winning == candidates) {
            return next;
        }
        candidates = next.winning;
    }
}

Controller build_controller() {
    using Clock = std::chrono::steady_clock;
    using Micros = std::chrono::duration<double, std::micro>;

    const auto start = Clock::now();
    const auto transitions = build_abstraction();
    const auto abstracted = Clock::now();
    const auto policy = synthesize(transitions);
    const auto synthesized = Clock::now();

    return {transitions, policy, Micros(abstracted - start).count(),
            Micros(synthesized - abstracted).count()};
}

std::optional<int> choose_action(const Policy& policy, double temperature) {
    const auto bin = quantize(temperature);
    if (!bin || policy.allowed[*bin] == 0) {
        return std::nullopt;
    }
    return std::countr_zero(policy.allowed[*bin]);
}

double certified_percentage(const Policy& policy, std::size_t rooms) {
    // The certified initial cells are the Cartesian product of the local winning sets.
    const double local_fraction = static_cast<double>(std::popcount(policy.winning)) / bin_count;
    return 100.0 * std::pow(local_fraction, static_cast<double>(rooms));
}

} // namespace thermostat
