#pragma once

#include "interval.hpp"
#include <array>

namespace symbolic::domain {

class BoundingBox {
public:
    Interval x_range;
    Interval y_range;
    Interval theta_range;

    BoundingBox() = default;
    BoundingBox(const Interval& x, const Interval& y, const Interval& theta);
    BoundingBox(double x_min, double x_max, double y_min, double y_max, double theta_min, double theta_max);

    [[nodiscard]] std::array<double, 3> center() const noexcept;
    [[nodiscard]] std::array<double, 3> radii() const noexcept;

    [[nodiscard]] bool contains(double x, double y, double theta) const noexcept;
    [[nodiscard]] bool intersects(const BoundingBox& other) const noexcept;
    [[nodiscard]] BoundingBox intersect_with(const BoundingBox& other) const noexcept;
    [[nodiscard]] bool is_empty() const noexcept;
};

} // namespace symbolic::domain
