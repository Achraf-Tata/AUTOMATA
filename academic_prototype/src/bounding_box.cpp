#include "bounding_box.hpp"

namespace symbolic::domain {

BoundingBox::BoundingBox(const Interval& x_interval, const Interval& y_interval, const Interval& theta_interval)
    : x_range(x_interval), y_range(y_interval), theta_range(theta_interval) {}

BoundingBox::BoundingBox(double x_minimum, double x_maximum,
                         double y_minimum, double y_maximum,
                         double theta_minimum, double theta_maximum)
    : x_range(x_minimum, x_maximum),
      y_range(y_minimum, y_maximum),
      theta_range(theta_minimum, theta_maximum) {}

std::array<double, 3> BoundingBox::center() const noexcept {
    return { x_range.midpoint(), y_range.midpoint(), theta_range.midpoint() };
}

std::array<double, 3> BoundingBox::radii() const noexcept {
    return { x_range.radius(), y_range.radius(), theta_range.radius() };
}

bool BoundingBox::contains(double x_coordinate, double y_coordinate, double theta_angle) const noexcept {
    return x_range.contains(x_coordinate) &&
           y_range.contains(y_coordinate) &&
           theta_range.contains(theta_angle);
}

bool BoundingBox::intersects(const BoundingBox& other_box) const noexcept {
    return x_range.overlaps(other_box.x_range) &&
           y_range.overlaps(other_box.y_range) &&
           theta_range.overlaps(other_box.theta_range);
}

BoundingBox BoundingBox::intersect_with(const BoundingBox& other_box) const noexcept {
    return BoundingBox(
        x_range.intersect_with(other_box.x_range),
        y_range.intersect_with(other_box.y_range),
        theta_range.intersect_with(other_box.theta_range)
    );
}

bool BoundingBox::is_empty() const noexcept {
    return x_range.is_empty() || y_range.is_empty() || theta_range.is_empty();
}

} // namespace symbolic::domain
