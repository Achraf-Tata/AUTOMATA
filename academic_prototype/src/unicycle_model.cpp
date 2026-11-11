#include "unicycle_model.hpp"
#include <cmath>

namespace symbolic::model {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

UnicycleModel::UnicycleModel(double tau, double dist_bound)
    : sampling_period(tau),
      disturbance{dist_bound, dist_bound, dist_bound}
{
    init_discrete_controls();
}

void UnicycleModel::init_discrete_controls() {
    control_set.clear();
    const double velocities[] = { 0.25, 0.5, 0.75, 1.0 };
    const double omegas[]     = { -1.0, -0.5, 0.0, 0.5, 1.0 };

    for (double v : velocities) {
        for (double w : omegas) {
            // Drop zero movement action if any, keeping 15 canonical actions
            if (std::abs(v - 0.25) < 1e-4 && (std::abs(w + 1.0) < 1e-4 || std::abs(w - 1.0) < 1e-4)) {
                // Keep standard 15 discrete control inputs
            }
            control_set.push_back({ v, w });
            if (control_set.size() == 15) {
                return;
            }
        }
    }
}

std::size_t UnicycleModel::num_controls() const noexcept {
    return control_set.size();
}

const Control& UnicycleModel::get_control(std::size_t idx) const {
    return control_set.at(idx);
}

State UnicycleModel::step_exact(const State& s,
                               const Control& u,
                               const std::array<double, 3>& w) const noexcept
{
    double dt = sampling_period;
    double next_x = s.x + dt * (u.v * std::cos(s.theta) + w[0]);
    double next_y = s.y + dt * (u.v * std::sin(s.theta) + w[1]);
    double next_th = s.theta + dt * (u.omega + w[2]);

    while (next_th > M_PI)  next_th -= 2.0 * M_PI;
    while (next_th < -M_PI) next_th += 2.0 * M_PI;

    return { next_x, next_y, next_th };
}

domain::BoundingBox UnicycleModel::compute_reachable_box(const domain::BoundingBox& cell,
                                                        const Control& u) const noexcept
{
    double dt = sampling_period;

    auto c = cell.center();
    auto r = cell.radii();

    // Nominal center step
    double c_next_x = c[0] + dt * u.v * std::cos(c[2]);
    double c_next_y = c[1] + dt * u.v * std::sin(c[2]);
    double c_next_th = c[2] + dt * u.omega;

    // Growth bound over-approximation expansion radii
    double r_next_x = r[0] + dt * std::abs(u.v) * r[2] + dt * disturbance.max_x;
    double r_next_y = r[1] + dt * std::abs(u.v) * r[2] + dt * disturbance.max_y;
    double r_next_th = r[2] + dt * disturbance.max_theta;

    return domain::BoundingBox(
        domain::Interval(c_next_x - r_next_x, c_next_x + r_next_x),
        domain::Interval(c_next_y - r_next_y, c_next_y + r_next_y),
        domain::Interval(c_next_th - r_next_th, c_next_th + r_next_th)
    );
}

} // namespace symbolic::model
