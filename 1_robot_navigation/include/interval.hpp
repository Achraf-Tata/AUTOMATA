#pragma once

#include <algorithm>
#include <cmath>

namespace symbolic::domain {

class Interval {
public:
    double lower_bound = 0.0;
    double upper_bound = 0.0;

    Interval() = default;
    Interval(double initial_lower_bound, double initial_upper_bound);

    [[nodiscard]] double width() const noexcept;
    [[nodiscard]] double radius() const noexcept;
    [[nodiscard]] double midpoint() const noexcept;

    [[nodiscard]] bool contains(double value) const noexcept;
    [[nodiscard]] bool overlaps(const Interval& other_interval) const noexcept;
    [[nodiscard]] bool is_empty() const noexcept;

    [[nodiscard]] Interval intersect_with(const Interval& other_interval) const noexcept;
    [[nodiscard]] Interval inflate(double inflation_radius) const noexcept;

    Interval operator+(const Interval& other_interval) const noexcept;
    Interval operator-(const Interval& other_interval) const noexcept;
    Interval operator*(double scalar_multiplier) const noexcept;
};

Interval operator*(double scalar_multiplier, const Interval& interval) noexcept;

} // namespace symbolic::domain
