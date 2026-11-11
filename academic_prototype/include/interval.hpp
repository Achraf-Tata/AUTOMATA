#pragma once

#include <algorithm>
#include <cmath>
#include <string>

namespace symbolic::domain {

class Interval {
public:
    double lower = 0.0;
    double upper = 0.0;

    Interval() = default;
    Interval(double lower_bound, double upper_bound);

    [[nodiscard]] double width() const noexcept;
    [[nodiscard]] double radius() const noexcept;
    [[nodiscard]] double midpoint() const noexcept;

    [[nodiscard]] bool contains(double value) const noexcept;
    [[nodiscard]] bool overlaps(const Interval& other) const noexcept;
    [[nodiscard]] bool is_empty() const noexcept;

    [[nodiscard]] Interval intersect_with(const Interval& other) const noexcept;
    [[nodiscard]] Interval inflate(double radius) const noexcept;

    Interval operator+(const Interval& other) const noexcept;
    Interval operator-(const Interval& other) const noexcept;
    Interval operator*(double scalar) const noexcept;
};

Interval operator*(double scalar, const Interval& interval) noexcept;

} // namespace symbolic::domain
