#include "bounding_box.hpp"

namespace symbolic::domain {

BoundingBox::BoundingBox(const Interval& x, const Interval& y, const Interval& theta)
    : x_range(x), y_range(y), theta_range(theta) {}

BoundingBox::BoundingBox(double x_min, double x_max, double y_min, double y_max, double theta_min, double theta_max)
    : x_range(x_min, x_max), y_range(y_min, y_max), theta_range(theta_min, theta_max) {}

std::array<double, 3> BoundingBox::center() const noexcept {
    return { x_range.midpoint(), y_range.midpoint(), theta_range.midpoint() };
}

std::array<double, 3> BoundingBox::radii() const noexcept {
    return { x_range.radius(), y_range.radius(), theta_range.radius() };
}

bool BoundingBox::contains(double x, double y, double theta) const noexcept {
    return x_range.contains(x) && y_range.contains(y) && theta_range.contains(theta);
}

bool BoundingBox::intersects(const BoundingBox& other) const noexcept {
    return x_range.overlaps(other.x_range) &&
           y_range.overlaps(other.y_range) &&
           theta_range.overlaps(other.theta_range);
}

BoundingBox BoundingBox::intersect_with(const BoundingBox& other) const noexcept {
    return BoundingBox(
        x_range.intersect_with(other.x_range),
        y_range.intersect_with(other.y_range),
        theta_range.intersect_with(other.theta_range)
    );
}

bool BoundingBox::is_empty() const noexcept {
    return x_range.is_empty() || y_range.is_empty() || theta_range.is_empty();
}

} // namespace symbolic::domain
