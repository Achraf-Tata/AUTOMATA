#pragma once

#include <algorithm>
#include <cmath>

namespace symbolic::domain {

class Interval {
public:
    double lower_bound = 0.0;
    double upper_bound = 0.0;

    // default constructor: creates interval [0, 0]
    Interval() = default;

    // initialize with bounds, automatically ensures lower <= upper
    Interval(double initial_lower_bound, double initial_upper_bound);

    // width of the interval: (upper - lower)
    [[nodiscard]] double width() const noexcept;

    // half-width radius: (upper - lower) / 2
    [[nodiscard]] double radius() const noexcept;

    // midpoint center: (lower + upper) / 2
    [[nodiscard]] double midpoint() const noexcept;

    // check if a value falls inside [lower, upper]
    [[nodiscard]] bool contains(double value) const noexcept;

    // check if this interval overlaps with another one
    [[nodiscard]] bool overlaps(const Interval& other_interval) const noexcept;

    // check if the interval is invalid (lower > upper)
    [[nodiscard]] bool is_empty() const noexcept;

    // intersection: find the common overlapping sub-interval
    [[nodiscard]] Interval intersect_with(const Interval& other_interval) const noexcept;

    // expand the interval outward on both sides by radius: [lower - r, upper + r]
    [[nodiscard]] Interval inflate(double inflation_radius) const noexcept;

    // interval addition: [a, b] + [c, d] = [a + c, b + d]
    Interval operator+(const Interval& other_interval) const noexcept;

    // interval subtraction: [a, b] - [c, d] = [a - d, b - c]
    Interval operator-(const Interval& other_interval) const noexcept;

    // multiply interval by a scalar, handles negative numbers by flipping bounds
    Interval operator*(double scalar_multiplier) const noexcept;
};

// scalar * interval (same as interval * scalar)
Interval operator*(double scalar_multiplier, const Interval& interval) noexcept;

} // namespace symbolic::domain
