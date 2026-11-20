#pragma once

#include <algorithm>
#include <cmath>

namespace symbolic::domain {

class Interval {
public:
    double low = 0.0;
    double high = 0.0;

    // default constructor: creates interval [0, 0]
    Interval() = default;

    // initialize with bounds, automatically ensures low <= high
    Interval(double low, double high);

    // width of the interval: (high - low)
    [[nodiscard]] double width() const noexcept;

    // half-width radius: (high - low) / 2
    [[nodiscard]] double radius() const noexcept;

    // midpoint center: (low + high) / 2
    [[nodiscard]] double midpoint() const noexcept;

    // check if a value falls inside [low, high]
    [[nodiscard]] bool contains(double x) const noexcept;

    // check if this interval overlaps with another one
    [[nodiscard]] bool overlaps(const Interval& other) const noexcept;

    // check if the interval is invalid (low > high)
    [[nodiscard]] bool is_empty() const noexcept;

    // intersection: find the common overlapping sub-interval
    [[nodiscard]] Interval intersect_with(const Interval& other) const noexcept;

    // expand the interval outward on both sides by radius: [low - r, high + r]
    [[nodiscard]] Interval inflate(double r) const noexcept;

    // interval addition: [a, b] + [c, d] = [a + c, b + d]
    Interval operator+(const Interval& other) const noexcept;

    // interval subtraction: [a, b] - [c, d] = [a - d, b - c]
    Interval operator-(const Interval& other) const noexcept;

    // multiply interval by a scalar, handles negative numbers by flipping bounds
    Interval operator*(double k) const noexcept;
};

// scalar * interval (same as interval * scalar)
Interval operator*(double k, const Interval& iv) noexcept;

} // namespace symbolic::domain
