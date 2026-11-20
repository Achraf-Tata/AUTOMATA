#include "interval.hpp"
#include <algorithm>

namespace symbolic::domain {

using namespace std;

// construct interval and make sure low is always <= high
Interval::Interval(double low_val, double high_val)
    : low(min(low_val, high_val)),
      high(max(low_val, high_val)) {}

// interval length: high - low
double Interval::width() const noexcept {
    return high - low;
}

// half-width: (high - low) / 2
double Interval::radius() const noexcept {
    return 0.5 * (high - low);
}

// center position: (low + high) / 2
double Interval::midpoint() const noexcept {
    return 0.5 * (low + high);
}

// test if point x is inside [low, high]
bool Interval::contains(double x) const noexcept {
    return x >= low && x <= high;
}

// test if two intervals share at least one point
bool Interval::overlaps(const Interval& other) const noexcept {
    return low <= other.high && high >= other.low;
}

// empty check: true if low > high
bool Interval::is_empty() const noexcept {
    return low > high;
}

// find overlapping segment between two intervals
Interval Interval::intersect_with(const Interval& other) const noexcept {
    double lo = max(low, other.low);
    double hi = min(high, other.high);
    return Interval(lo, hi);
}

// expand bounds outward by radius on both sides
Interval Interval::inflate(double r) const noexcept {
    return Interval(low - r, high + r);
}

// interval addition: [a, b] + [c, d] = [a + c, b + d]
Interval Interval::operator+(const Interval& other) const noexcept {
    return Interval(low + other.low, high + other.high);
}

// interval subtraction: [a, b] - [c, d] = [a - d, b - c]
Interval Interval::operator-(const Interval& other) const noexcept {
    return Interval(low - other.high, high - other.low);
}

// multiply by scalar: if negative, swap low and high bounds
Interval Interval::operator*(double k) const noexcept {
    if (k >= 0.0) {
        return Interval(low * k, high * k);
    }
    return Interval(high * k, low * k);
}

// commutative multiplication: k * interval
Interval operator*(double k, const Interval& iv) noexcept {
    return iv * k;
}

} // namespace symbolic::domain
