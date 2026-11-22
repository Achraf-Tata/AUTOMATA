#include "thermostat.hpp"

#include <bit>
#include <chrono>
#include <cmath>

namespace thermostat {

// ============================================================================
// Controller Synthesis (Maximal Controlled Invariant Fixed-Point)
// ============================================================================
//
// Goal: Find the largest set of bins R* where we can guarantee that the room
// stays within the comfort zone [17, 22] °C forever.
//
// In formal control theory, this is the maximal controlled invariant set:
//   R* = nu X . (S intersect Pre(X))
//
// How this algorithm works in plain terms:
//   1. Start with the candidate set equal to ALL 10 bins [0..9].
//   2. In each round, test every remaining candidate bin:
//      - Look at each of the 3 heater actions.
//      - Check where that action could lead: are ALL possible next bins still
//        inside our safe candidate set?
//      - If YES, this action is safe to use in this bin!
//      - If an action could lead outside the candidates (or hit 'unsafe'),
//        that action cannot be trusted.
//   3. If a bin has NO safe actions left, it cannot survive: eliminate it.
//   4. If some bins were eliminated, repeat the check! (Because other bins might
//      now lead into the freshly eliminated bins).
//   5. When a pass finishes without eliminating any bins, we have reached the
//      "fixed point". All surviving bins are guaranteed safe indefinitely.
Policy synthesize(const Transitions& transitions) {
    BinMask candidates = all_bins; // Start assuming all 10 bins are safe

    while (true) {
        Policy next{};

        for (int bin = 0; bin < bin_count; ++bin) {
            // If this bin was already eliminated in a previous round, skip it
            if ((candidates & bin_bit(bin)) == 0) {
                continue;
            }

            for (int action = 0; action < action_count; ++action) {
                const BinMask successors = transitions[bin][action];

                // Are all reachable successors contained within the current candidate set?
                // Notice the bitwise trick:
                //   (successors & candidates) == successors
                // In a single CPU cycle, this checks if 'successors' is a subset of 'candidates'.
                // If 'successors' has the 'unsafe' bit (bit 10) set, it will not match,
                // so unsafe transitions are immediately rejected.
                if (successors != 0 && (successors & candidates) == successors) {
                    next.allowed[bin] |= action_bit(action); // Action is certified safe!
                }
            }

            // If this bin has at least one safe action, it survives this round
            if (next.allowed[bin] != 0) {
                next.winning |= bin_bit(bin);
            }
        }

        // Fixed point reached: candidate set did not shrink during this pass
        if (next.winning == candidates) {
            return next;
        }

        // Otherwise, shrink candidates to the survivors and run another round
        candidates = next.winning;
    }
}

// Builds both the abstraction and controller, measuring their execution time in microseconds.
Controller build_controller() {
    using Clock = std::chrono::steady_clock;
    using Micros = std::chrono::duration<double, std::micro>;

    const auto start = Clock::now();
    const auto transitions = build_abstraction();
    const auto abstracted = Clock::now();
    const auto policy = synthesize(transitions);
    const auto synthesized = Clock::now();

    return {transitions, policy,
            Micros(abstracted - start).count(),
            Micros(synthesized - abstracted).count()};
}

// Given a continuous temperature measurement, looks up its bin and selects a heater setting.
// If multiple heater actions are safe, it selects the lowest index (lowest energy consumption)
// using std::countr_zero (count trailing zeros).
std::optional<int> choose_action(const Policy& policy, double temperature) {
    const auto bin = quantize(temperature);
    if (!bin || policy.allowed[*bin] == 0) {
        return std::nullopt; // Out of bounds or no safe action available
    }

    // std::countr_zero returns the index of the first '1' bit in the mask.
    // For example, if allowed is 0b010 (action 1), countr_zero returns 1.
    return std::countr_zero(policy.allowed[*bin]);
}

// Computes what percentage of the global N-room building state-space is certified safe.
//
// Under the Contract-Based framework (Saoud et al., IEEE TAC 2020):
//   - Each room independently satisfies its local contract:
//       Assumption:  Both neighbors stay in [17, 22] °C
//       Guarantee:   This room stays in [17, 22] °C
//   - Because all rooms satisfy their contract, the global winning set is simply
//     the Cartesian product of the local winning sets:
//       Global_Winning = Winning_Room_1 x Winning_Room_2 x ... x Winning_Room_N
//   - Therefore:
//       Certified_Fraction = (local_winning_bins / total_local_bins) ^ N
//   - Here, local_winning = 10 out of 10 bins, so (10/10)^N = 100% for ANY N!
double certified_percentage(const Policy& policy, std::size_t rooms) {
    const double local_fraction = static_cast<double>(std::popcount(policy.winning)) / bin_count;
    return 100.0 * std::pow(local_fraction, static_cast<double>(rooms));
}

} // namespace thermostat
