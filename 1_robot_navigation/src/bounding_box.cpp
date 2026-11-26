#include "bounding_box.hpp"

namespace symbolic::domain {

using namespace std;

// construct box from 1D intervals for x, y, and theta
BoundingBox::BoundingBox(const Interval& x_int, const Interval& y_int, const Interval& th_int)
    : x_range(x_int), y_range(y_int), theta_range(th_int) {}

// construct box from explicit minimum and maximum coordinates
BoundingBox::BoundingBox(double x_min, double x_max, double y_min, double y_max, double th_min, double th_max)
    : x_range(x_min, x_max), y_range(y_min, y_max), theta_range(th_min, th_max) {}

// center coordinate: {midpoint(x), midpoint(y), midpoint(theta)}
array<double, 3> BoundingBox::center() const noexcept {
    return { x_range.midpoint(), y_range.midpoint(), theta_range.midpoint() };
}

// half-widths: {radius(x), radius(y), radius(theta)}
array<double, 3> BoundingBox::radii() const noexcept {
    return { x_range.radius(), y_range.radius(), theta_range.radius() };
}

// test if point (x, y, theta) is inside the box bounds
bool BoundingBox::contains(double x, double y, double theta) const noexcept {
    return x_range.contains(x) && y_range.contains(y) && theta_range.contains(theta);
}

// test if this box and another overlap in all 3 coordinates
bool BoundingBox::intersects(const BoundingBox& other) const noexcept {
    return x_range.overlaps(other.x_range) &&
           y_range.overlaps(other.y_range) &&
           theta_range.overlaps(other.theta_range);
}

// find overlapping 3D region between two boxes
BoundingBox BoundingBox::intersect_with(const BoundingBox& other) const noexcept {
    return BoundingBox(
        x_range.intersect_with(other.x_range),
        y_range.intersect_with(other.y_range),
        theta_range.intersect_with(other.theta_range)
    );
}

// true if any dimension has an empty or invalid interval
bool BoundingBox::is_empty() const noexcept {
    return x_range.is_empty() || y_range.is_empty() || theta_range.is_empty();
}

} // namespace symbolic::domain
