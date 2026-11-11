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
    BoundingBox(const Interval& x_interval, const Interval& y_interval, const Interval& theta_interval);
    BoundingBox(double x_minimum, double x_maximum,
                double y_minimum, double y_maximum,
                double theta_minimum, double theta_maximum);

    [[nodiscard]] std::array<double, 3> center() const noexcept;
    [[nodiscard]] std::array<double, 3> radii() const noexcept;

    [[nodiscard]] bool contains(double x_coordinate, double y_coordinate, double theta_angle) const noexcept;
    [[nodiscard]] bool intersects(const BoundingBox& other_box) const noexcept;
    [[nodiscard]] BoundingBox intersect_with(const BoundingBox& other_box) const noexcept;
    [[nodiscard]] bool is_empty() const noexcept;
};

} // namespace symbolic::domain
