#include "interval.hpp"
#include <algorithm>

namespace symbolic::domain {

Interval::Interval(double lower_bound, double upper_bound)
    : lower(std::min(lower_bound, upper_bound)),
      upper(std::max(lower_bound, upper_bound)) {}

double Interval::width() const noexcept {
    return upper - lower;
}

double Interval::radius() const noexcept {
    return 0.5 * (upper - lower);
}

double Interval::midpoint() const noexcept {
    return 0.5 * (lower + upper);
}

bool Interval::contains(double value) const noexcept {
    return value >= lower && value <= upper;
}

bool Interval::overlaps(const Interval& other) const noexcept {
    return lower <= other.upper && upper >= other.lower;
}

bool Interval::is_empty() const noexcept {
    return lower > upper;
}

Interval Interval::intersect_with(const Interval& other) const noexcept {
    double new_lower = std::max(lower, other.lower);
    double new_upper = std::min(upper, other.upper);
    return Interval(new_lower, new_upper);
}

Interval Interval::inflate(double r) const noexcept {
    return Interval(lower - r, upper + r);
}

Interval Interval::operator+(const Interval& other) const noexcept {
    return Interval(lower + other.lower, upper + other.upper);
}

Interval Interval::operator-(const Interval& other) const noexcept {
    return Interval(lower - other.upper, upper - other.lower);
}

Interval Interval::operator*(double scalar) const noexcept {
    if (scalar >= 0.0) {
        return Interval(lower * scalar, upper * scalar);
    }
    return Interval(upper * scalar, lower * scalar);
}

Interval operator*(double scalar, const Interval& interval) noexcept {
    return interval * scalar;
}

} // namespace symbolic::domain
