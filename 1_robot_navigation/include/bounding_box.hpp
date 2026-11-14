#pragma once

#include "interval.hpp"
#include <array>

namespace symbolic::domain {

using namespace std;

// 3D bounding box representing a region in robot state space (x, y, theta)
class BoundingBox {
public:
    Interval x_range;
    Interval y_range;
    Interval theta_range;

    // default constructor: creates zero-sized box at origin
    BoundingBox() = default;

    // build box from three 1D intervals: [x_min, x_max], [y_min, y_max], [th_min, th_max]
    BoundingBox(const Interval& x_interval, const Interval& y_interval, const Interval& theta_interval);

    // build box from explicit min and max boundary coordinates
    BoundingBox(double x_minimum, double x_maximum,double y_minimum, double y_maximum,double theta_minimum, double theta_maximum);

    // 3D center point: {x_mid, y_mid, theta_mid}
    [[nodiscard]] array<double, 3> center() const noexcept;

    // 3D half-widths: {radius_x, radius_y, radius_theta}
    [[nodiscard]] array<double, 3> radii() const noexcept;

    // check if a robot pose (x, y, theta) is inside this box
    [[nodiscard]] bool contains(double x_coordinate, double y_coordinate, double theta_angle) const noexcept;

    // check if this box overlaps with another 3D box (all 3 dimensions must overlap)
    [[nodiscard]] bool intersects(const BoundingBox& other_box) const noexcept;

    // compute the common 3D overlapping region with another box
    [[nodiscard]] BoundingBox intersect_with(const BoundingBox& other_box) const noexcept;

    // returns true if any coordinate interval is empty or invalid
    [[nodiscard]] bool is_empty() const noexcept;
};

} // namespace symbolic::domain

namespace symbolic {
    using domain::BoundingBox;
}
