#pragma once

#include "interval.hpp"
#include <array>

namespace symbolic::domain {

class BoundingBox {
public:
    Interval x_range;
    Interval y_range;
    Interval theta_range;

    // Default constructor
    BoundingBox() = default;

    // Construct 3D bounding box from three 1D intervals
    BoundingBox(const Interval& x_interval, const Interval& y_interval, const Interval& theta_interval);

    // Construct 3D bounding box from explicit coordinate bounds
    BoundingBox(double x_minimum, double x_maximum,
                double y_minimum, double y_maximum,
                double theta_minimum, double theta_maximum);

    // Returns the 3D center point (x, y, theta)
    [[nodiscard]] std::array<double, 3> center() const noexcept;

    // Returns the 3D half-width radii (rx, ry, rtheta)
    [[nodiscard]] std::array<double, 3> radii() const noexcept;

    // Tests whether a continuous point (x, y, theta) lies inside this box
    [[nodiscard]] bool contains(double x_coordinate, double y_coordinate, double theta_angle) const noexcept;

    // Tests whether this box intersects another 3D bounding box
    [[nodiscard]] bool intersects(const BoundingBox& other_box) const noexcept;

    // Computes the intersection box with another bounding box
    [[nodiscard]] BoundingBox intersect_with(const BoundingBox& other_box) const noexcept;

    // Checks if any dimension is empty or degenerate
    [[nodiscard]] bool is_empty() const noexcept;
};

} // namespace symbolic::domain
