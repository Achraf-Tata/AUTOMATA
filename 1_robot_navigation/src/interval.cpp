#include "interval.hpp"
#include <algorithm>

namespace symbolic::domain {

using namespace std;

// construct interval and make sure lower is always <= upper
Interval::Interval(double initial_lower_bound, double initial_upper_bound)
    : lower_bound(min(initial_lower_bound, initial_upper_bound)),
      upper_bound(max(initial_lower_bound, initial_upper_bound)) {}

// interval length: upper - lower
double Interval::width() const noexcept {
    return upper_bound - lower_bound;
}

// half-width: (upper - lower) / 2
double Interval::radius() const noexcept {
    return 0.5 * (upper_bound - lower_bound);
}

// center position: (lower + upper) / 2
double Interval::midpoint() const noexcept {
    return 0.5 * (lower_bound + upper_bound);
}

// test if point x is inside [lower, upper]
bool Interval::contains(double value) const noexcept {
    return value >= lower_bound && value <= upper_bound;
}

// test if two intervals share at least one point
bool Interval::overlaps(const Interval& other_interval) const noexcept {
    return lower_bound <= other_interval.upper_bound && upper_bound >= other_interval.lower_bound;
}

// empty check: true if lower > upper
bool Interval::is_empty() const noexcept {
    return lower_bound > upper_bound;
}

// find overlapping segment between two intervals
Interval Interval::intersect_with(const Interval& other_interval) const noexcept {
    double intersected_lower = max(lower_bound, other_interval.lower_bound);
    double intersected_upper = min(upper_bound, other_interval.upper_bound);
    return Interval(intersected_lower, intersected_upper);
}

// expand bounds outward by radius on both sides
Interval Interval::inflate(double inflation_radius) const noexcept {
    return Interval(lower_bound - inflation_radius, upper_bound + inflation_radius);
}

// interval addition: [a, b] + [c, d] = [a + c, b + d]
Interval Interval::operator+(const Interval& other_interval) const noexcept {
    return Interval(lower_bound + other_interval.lower_bound, upper_bound + other_interval.upper_bound);
}

// interval subtraction: [a, b] - [c, d] = [a - d, b - c]
Interval Interval::operator-(const Interval& other_interval) const noexcept {
    return Interval(lower_bound - other_interval.upper_bound, upper_bound - other_interval.lower_bound);
}

// multiply by scalar: if negative, swap lower and upper bounds
Interval Interval::operator*(double scalar_multiplier) const noexcept {
    if (scalar_multiplier >= 0.0) {
        return Interval(lower_bound * scalar_multiplier, upper_bound * scalar_multiplier);
    }
    return Interval(upper_bound * scalar_multiplier, lower_bound * scalar_multiplier);
}

// commutative multiplication: k * interval
Interval operator*(double scalar_multiplier, const Interval& interval) noexcept {
    return interval * scalar_multiplier;
}

} // namespace symbolic::domain
