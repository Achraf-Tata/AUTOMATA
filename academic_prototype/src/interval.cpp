#include "interval.hpp"
#include <algorithm>

namespace symbolic::domain {

Interval::Interval(double initial_lower_bound, double initial_upper_bound)
    : lower_bound(std::min(initial_lower_bound, initial_upper_bound)),
      upper_bound(std::max(initial_lower_bound, initial_upper_bound)) {}

double Interval::width() const noexcept {
    return upper_bound - lower_bound;
}

double Interval::radius() const noexcept {
    return 0.5 * (upper_bound - lower_bound);
}

double Interval::midpoint() const noexcept {
    return 0.5 * (lower_bound + upper_bound);
}

bool Interval::contains(double value) const noexcept {
    return value >= lower_bound && value <= upper_bound;
}

bool Interval::overlaps(const Interval& other_interval) const noexcept {
    return lower_bound <= other_interval.upper_bound && upper_bound >= other_interval.lower_bound;
}

bool Interval::is_empty() const noexcept {
    return lower_bound > upper_bound;
}

Interval Interval::intersect_with(const Interval& other_interval) const noexcept {
    double intersected_lower = std::max(lower_bound, other_interval.lower_bound);
    double intersected_upper = std::min(upper_bound, other_interval.upper_bound);
    return Interval(intersected_lower, intersected_upper);
}

Interval Interval::inflate(double inflation_radius) const noexcept {
    return Interval(lower_bound - inflation_radius, upper_bound + inflation_radius);
}

Interval Interval::operator+(const Interval& other_interval) const noexcept {
    return Interval(lower_bound + other_interval.lower_bound, upper_bound + other_interval.upper_bound);
}

Interval Interval::operator-(const Interval& other_interval) const noexcept {
    return Interval(lower_bound - other_interval.upper_bound, upper_bound - other_interval.lower_bound);
}

Interval Interval::operator*(double scalar_multiplier) const noexcept {
    if (scalar_multiplier >= 0.0) {
        return Interval(lower_bound * scalar_multiplier, upper_bound * scalar_multiplier);
    }
    return Interval(upper_bound * scalar_multiplier, lower_bound * scalar_multiplier);
}

Interval operator*(double scalar_multiplier, const Interval& interval) noexcept {
    return interval * scalar_multiplier;
}

} // namespace symbolic::domain
