#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace thermostat {

// ============================================================================
// Physical Parameters (from the temperature regulation model)
// ============================================================================

// The comfort temperature zone is [17°C, 22°C].
// We discretize this continuous range into small intervals called "bins".
// Each degree is split into 2 bins (0.5°C per bin):
//   Bin 0: [17.0, 17.5) °C
//   Bin 1: [17.5, 18.0) °C
//   ...
//   Bin 9: [21.5, 22.0] °C  (10 bins total)
inline constexpr int min_temperature = 17;
inline constexpr int max_temperature = 22;
inline constexpr int bins_per_degree = 2;
inline constexpr int bin_count = (max_temperature - min_temperature) * bins_per_degree; // 10

// Available heater control actions: off (0.0), medium (0.3), high (0.6).
// Stored as tenths (0, 3, 6) so we can do exact integer math when computing transitions.
inline constexpr std::array heater_tenths{0, 3, 6};
inline constexpr int action_count = static_cast<int>(heater_tenths.size()); // 3

// Heat transfer weights (scaled by 1000 to use whole integers):
//   neighbor_weight (alpha = 0.450) -> heat exchanged through walls with each neighbor
//   outside_weight  (beta  = 0.045) -> heat lost through windows to the outside
//   heater_weight   (gamma = 0.090) -> heat added by the radiator
inline constexpr int coefficient_scale = 1000;
inline constexpr int neighbor_weight = 450;
inline constexpr int outside_weight = 45;
inline constexpr int heater_weight = 90;

// Environmental temperatures:
//   outside_temperature: cold outdoor air at -1°C
//   heater_temperature:  radiator hot water surface at 50°C
inline constexpr int outside_temperature = -1;
inline constexpr int heater_temperature = 50;

// Helper: converts a heater action index (0, 1, 2) to its real value (0.0, 0.3, 0.6).
constexpr double heater_setting(int action) {
    return heater_tenths[action] / 10.0;
}

// ============================================================================
// Bitmask Representation (Fast Set Operations in CPU Registers)
// ============================================================================

// Instead of heavy data structures (like std::set<int>), we represent sets of
// bins using bits in a 16-bit integer (uint16_t).
//
// How it works:
//   - Bit 0 is 1 if Bin 0 is in the set
//   - Bit 1 is 1 if Bin 1 is in the set
//   ...
//   - Bit 9 is 1 if Bin 9 is in the set
//
// Checking if a bin is in the set, adding a bin, or checking if all next states
// are safe takes just one bitwise CPU instruction (AND &, OR |, XOR ^).
using BinMask = std::uint16_t;
using ActionMask = std::uint8_t;

// Creates a mask with only the bit for 'bin' turned on (1 << bin).
constexpr BinMask bin_bit(int bin) { return static_cast<BinMask>(1u << bin); }

// Creates a mask with only the bit for 'action' turned on (1 << action).
constexpr ActionMask action_bit(int action) { return static_cast<ActionMask>(1u << action); }

// Mask with all 10 comfort bins active (bits 0 to 9 set = 0x03FF = 1023 in decimal).
inline constexpr BinMask all_bins = bin_bit(bin_count) - 1;

// Special flag bit (bit 10): set if a transition can leave the safe range [17, 22] °C.
inline constexpr BinMask unsafe = bin_bit(bin_count);

// Transition table:
//   transitions[current_bin][action] -> BinMask
// For each bin and each heater setting, stores a bitmask of ALL bins the room
// could possibly land in on the next step.
using Transitions = std::array<std::array<BinMask, action_count>, bin_count>;

// The synthesized control policy:
//   - winning: mask of bins from which the room can be kept safe forever
//   - allowed[bin]: mask of which heater actions are safe to use in that bin
struct Policy {
    BinMask winning{};
    std::array<ActionMask, bin_count> allowed{};
};

// Bundle storing the abstraction table, the policy, and synthesis timing measurements.
struct Controller {
    Transitions transitions;
    Policy policy;
    double abstraction_us; // Time to build the abstraction table in microseconds
    double synthesis_us;   // Time to run the safety fixed-point loop in microseconds
};

// ============================================================================
// Function Declarations
// ============================================================================

// Converts a continuous temperature in °C to its discrete bin index (0 to 9).
// Returns std::nullopt if the temperature is outside the comfort zone [17, 22] °C.
[[nodiscard]] std::optional<int> quantize(double temperature);

// Builds the discrete abstraction table by evaluating worst-case neighbor bounds.
[[nodiscard]] Transitions build_abstraction();

// Synthesizes the safe control policy using maximal controlled invariant fixed-point iteration.
[[nodiscard]] Policy synthesize(const Transitions& transitions);

// Convenience function: builds abstraction, synthesizes policy, and records timings.
[[nodiscard]] Controller build_controller();

// Picks the safest heater setting (lowest energy among safe actions) for a measured temperature.
[[nodiscard]] std::optional<int> choose_action(const Policy& policy, double temperature);

// Calculates what percentage of the N-room building state-space is certified safe.
// Under contract theory, this is simply (local_winning_bins / 10)^N * 100%.
[[nodiscard]] double certified_percentage(const Policy& policy, std::size_t rooms);

// ============================================================================
// Building Simulation Class
// ============================================================================

// Simulates the physical circular building with N rooms in continuous temperature.
// Uses double-buffering so that all rooms update simultaneously based on the same snapshot.
class Building {
public:
    explicit Building(std::vector<double> initial);

    // Current temperatures of all rooms.
    [[nodiscard]] std::span<const double> temperatures() const { return current_; }

    // Number of simulation steps completed.
    [[nodiscard]] std::size_t tick() const { return tick_; }

    // Advances the building by one simulation step using the policy.
    void step(const Policy& policy);

private:
    std::vector<double> current_; // Temperatures at time k
    std::vector<double> next_;    // Temperatures at time k + 1
    std::size_t tick_{};
};

} // namespace thermostat
