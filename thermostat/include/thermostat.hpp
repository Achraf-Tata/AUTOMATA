#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace thermostat {

inline constexpr int min_temperature = 17;
inline constexpr int max_temperature = 22;
inline constexpr int bins_per_degree = 2;
inline constexpr int bin_count = (max_temperature - min_temperature) * bins_per_degree;
inline constexpr std::array heater_tenths{0, 3, 6};
inline constexpr int action_count = static_cast<int>(heater_tenths.size());

// The slide's coefficients, in thousandths, shared by abstraction and simulation.
inline constexpr int coefficient_scale = 1000;
inline constexpr int neighbor_weight = 450;
inline constexpr int outside_weight = 45;
inline constexpr int heater_weight = 90;
inline constexpr int outside_temperature = -1;
inline constexpr int heater_temperature = 50;

constexpr double heater_setting(int action) {
    return heater_tenths[action] / 10.0;
}

using BinMask = std::uint16_t;
using ActionMask = std::uint8_t;
constexpr BinMask bin_bit(int bin) { return static_cast<BinMask>(1u << bin); }
constexpr ActionMask action_bit(int action) { return static_cast<ActionMask>(1u << action); }
inline constexpr BinMask all_bins = bin_bit(bin_count) - 1;
inline constexpr BinMask unsafe = bin_bit(bin_count);

// Rows are current bins, columns are heater indices. Each entry gives all successors.
using Transitions = std::array<std::array<BinMask, action_count>, bin_count>;

struct Policy {
    BinMask winning{};
    std::array<ActionMask, bin_count> allowed{};
};

struct Controller {
    Transitions transitions;
    Policy policy;
    double abstraction_us;
    double synthesis_us;
};

// Cells are half-open on the right, except the final cell, which includes 22.
[[nodiscard]] std::optional<int> quantize(double temperature);
[[nodiscard]] Transitions build_abstraction();
[[nodiscard]] Policy synthesize(const Transitions& transitions);
[[nodiscard]] Controller build_controller();

// Return the lowest allowed heater index. A missing action disables the update.
[[nodiscard]] std::optional<int> choose_action(const Policy& policy, double temperature);
[[nodiscard]] double certified_percentage(const Policy& policy, std::size_t rooms);

class Building {
public:
    explicit Building(std::vector<double> initial);
    [[nodiscard]] std::span<const double> temperatures() const { return current_; }
    [[nodiscard]] std::size_t tick() const { return tick_; }

    // Apply a simultaneous plant update, using the neighbors in the current snapshot.
    void step(const Policy& policy);

private:
    std::vector<double> current_;
    std::vector<double> next_;
    std::size_t tick_{};
};

} // namespace thermostat
