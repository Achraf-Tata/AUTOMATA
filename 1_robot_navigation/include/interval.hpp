#pragma once

#include <algorithm>
#include <cmath>

namespace symbolic::domain {

class Interval {
public:
    double lower_bound = 0.0;
    double upper_bound = 0.0;

    // Default constructor: creates [0.0, 0.0]
    Interval() = default;

    // Construct interval with given lower and upper bounds
    Interval(double initial_lower_bound, double initial_upper_bound);

    // Compute the width of the interval: upper_bound - lower_bound
    [[nodiscard]] double width() const noexcept;

    // Compute the radius (half-width) of the interval
    [[nodiscard]] double radius() const noexcept;

    // Compute the center midpoint of the interval
    [[nodiscard]] double midpoint() const noexcept;

    // Check whether a scalar value lies within [lower_bound, upper_bound]
    [[nodiscard]] bool contains(double value) const noexcept;

    // Check whether this interval overlaps with another interval
    [[nodiscard]] bool overlaps(const Interval& other_interval) const noexcept;

    // Check whether the interval is empty/degenerate (lower > upper)
    [[nodiscard]] bool is_empty() const noexcept;

    // Compute the intersection interval with another interval
    [[nodiscard]] Interval intersect_with(const Interval& other_interval) const noexcept;

    // Inflate both bounds outward by the given inflation radius
    [[nodiscard]] Interval inflate(double inflation_radius) const noexcept;

    // Minkowski sum of two intervals
    Interval operator+(const Interval& other_interval) const noexcept;

    // Minkowski difference of two intervals
    Interval operator-(const Interval& other_interval) const noexcept;

    // Scalar multiplication of an interval
    Interval operator*(double scalar_multiplier) const noexcept;
};

// Commutative scalar multiplication with an interval
Interval operator*(double scalar_multiplier, const Interval& interval) noexcept;

} // namespace symbolic::domain
