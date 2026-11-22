#include "thermostat.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace thermostat {

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

void Building::step(const Policy& policy) {
    const std::size_t count = current_.size();
    constexpr double alpha = static_cast<double>(neighbor_weight) / coefficient_scale;
    constexpr double beta = static_cast<double>(outside_weight) / coefficient_scale;
    constexpr double gamma = static_cast<double>(heater_weight) / coefficient_scale;

    for (std::size_t room = 0; room < count; ++room) {
        const auto action = choose_action(policy, current_[room]);
        if (!action) {
            throw std::runtime_error("The controller has no action for the current cell.");
        }

        const std::size_t left = room == 0 ? count - 1 : room - 1;
        const std::size_t right = room + 1 == count ? 0 : room + 1;
        const double own = current_[room];

        // The policy gets only the room's measurement. The plant still exchanges
        // heat with its actual neighbors, and all room updates read the same snapshot.
        next_[room] = own + alpha * (current_[left] + current_[right] - 2.0 * own)
                      + beta * (outside_temperature - own)
                      + gamma * (heater_temperature - own) * heater_setting(*action);
        if (!(min_temperature <= next_[room] && next_[room] <= max_temperature)) {
            throw std::runtime_error("A room left the comfort zone.");
        }
    }

    current_.swap(next_);
    ++tick_;
}

} // namespace thermostat
