#include "thermostat.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace thermostat {

// Constructs the physical building simulator.
// Checks the mathematical hypotheses required by the contract framework:
//   1. Ring topology requires at least 3 rooms (so each room has 2 distinct neighbors).
//   2. All initial temperatures must start inside the comfort zone [17, 22] °C (Hypothesis 4).
Building::Building(std::vector<double> initial) : current_(std::move(initial)) {
    if (current_.size() < 3) {
        throw std::invalid_argument("A circular building needs at least three rooms.");
    }
    if (!std::all_of(current_.begin(), current_.end(), [](double value) {
            return quantize(value).has_value();
        })) {
        throw std::invalid_argument("Initial temperatures must be in [17, 22] C.");
    }
    next_.resize(current_.size());
}

// Advances the entire building by one simulation step.
//
// Implementation Technique: Double-Buffering ("Ping-Pong" Buffer)
//   - When computing room i's new temperature, it needs the temperatures of room i-1 and i+1.
//   - If we updated temperatures in place in a single vector, room 1 would read room 0's
//     NEW temperature instead of its OLD temperature!
//   - To ensure that ALL rooms update simultaneously based on the EXACT SAME time-step snapshot:
//       * We read from 'current_'
//       * We write newly calculated values into 'next_'
//       * At the end of the loop, we swap 'current_' and 'next_' in O(1) time (pointer swap).
//   - This is completely thread-safe, deterministic, and allocates zero memory at each step.
void Building::step(const Policy& policy) {
    const std::size_t count = current_.size();
    constexpr double alpha = static_cast<double>(neighbor_weight) / coefficient_scale; // 0.450
    constexpr double beta = static_cast<double>(outside_weight) / coefficient_scale;   // 0.045
    constexpr double gamma = static_cast<double>(heater_weight) / coefficient_scale;  // 0.090

    for (std::size_t room = 0; room < count; ++room) {
        // Decentralized control: each room chooses its heater based ONLY on its OWN temperature!
        const auto action = choose_action(policy, current_[room]);
        if (!action) {
            throw std::runtime_error("The controller has no action for the current cell.");
        }

        // Circular ring boundary wrapping:
        // Room 0's left neighbor is the last room (count - 1).
        // The last room's right neighbor is Room 0.
        const std::size_t left = (room == 0) ? count - 1 : room - 1;
        const std::size_t right = (room + 1 == count) ? 0 : room + 1;
        const double own = current_[room];

        // Discrete-time physical heat transfer equation (Slide 15):
        //   - alpha * (left + right - 2*own) : conduction through walls with adjacent rooms
        //   - beta * (outside - own)         : heat loss to cold outside air (-1°C)
        //   - gamma * (heater - own) * u     : heat added by radiator (50°C) with power setting u
        next_[room] = own + alpha * (current_[left] + current_[right] - 2.0 * own)
                      + beta * (outside_temperature - own)
                      + gamma * (heater_temperature - own) * heater_setting(*action);

        // Safety invariant check: confirm the room stayed in comfort range [17, 22] °C
        if (!(min_temperature <= next_[room] && next_[room] <= max_temperature)) {
            throw std::runtime_error("A room left the comfort zone.");
        }
    }

    // Ping-pong buffer swap (instantaneous O(1) pointer swap, zero memory allocation)
    current_.swap(next_);
    ++tick_;
}

} // namespace thermostat
