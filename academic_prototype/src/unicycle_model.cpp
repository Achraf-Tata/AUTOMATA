#include "unicycle_model.hpp"
#include <cmath>

namespace symbolic::model {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

UnicycleModel::UnicycleModel(double initial_sampling_period, double initial_disturbance_bound)
    : sampling_period_seconds(initial_sampling_period),
      disturbance_bounds{initial_disturbance_bound, initial_disturbance_bound, initial_disturbance_bound}
{
    initialize_discrete_controls();
}

void UnicycleModel::initialize_discrete_controls() {
    discrete_controls.clear();
    const double velocities[] = { 0.25, 0.50, 0.75 };
    const double turn_rates[] = { -1.0, -0.5, 0.0, 0.5, 1.0 };

    for (double velocity : velocities) {
        for (double turn_rate : turn_rates) {
            discrete_controls.push_back({ velocity, turn_rate });
        }
    }
}

std::size_t UnicycleModel::num_controls() const noexcept {
    return discrete_controls.size();
}

const Control& UnicycleModel::get_control(std::size_t control_index) const {
    return discrete_controls.at(control_index);
}

State UnicycleModel::step_exact(const State& current_state,
                               const Control& control_input,
                               const std::array<double, 3>& disturbance_vector) const noexcept
{
    double time_step = sampling_period_seconds;
    double next_x = current_state.x_position + time_step * (control_input.linear_velocity * std::cos(current_state.heading_angle) + disturbance_vector[0]);
    double next_y = current_state.y_position + time_step * (control_input.linear_velocity * std::sin(current_state.heading_angle) + disturbance_vector[1]);
    double next_theta = current_state.heading_angle + time_step * (control_input.angular_velocity + disturbance_vector[2]);

    while (next_theta > M_PI)  next_theta -= 2.0 * M_PI;
    while (next_theta < -M_PI) next_theta += 2.0 * M_PI;

    return { next_x, next_y, next_theta };
}

domain::BoundingBox UnicycleModel::compute_reachable_box(const domain::BoundingBox& cell_box,
                                                        const Control& control_input) const noexcept
{
    double time_step = sampling_period_seconds;

    auto cell_center = cell_box.center();
    auto cell_radii  = cell_box.radii();

    // Nominal center propagation
    double predicted_center_x = cell_center[0] + time_step * control_input.linear_velocity * std::cos(cell_center[2]);
    double predicted_center_y = cell_center[1] + time_step * control_input.linear_velocity * std::sin(cell_center[2]);
    double predicted_center_theta = cell_center[2] + time_step * control_input.angular_velocity;

    // Growth bound over-approximation expansion radii
    double expansion_radius_x = cell_radii[0] + time_step * std::abs(control_input.linear_velocity) * cell_radii[2] + time_step * disturbance_bounds.max_disturbance_x;
    double expansion_radius_y = cell_radii[1] + time_step * std::abs(control_input.linear_velocity) * cell_radii[2] + time_step * disturbance_bounds.max_disturbance_y;
    double expansion_radius_theta = cell_radii[2] + time_step * disturbance_bounds.max_disturbance_theta;

    return domain::BoundingBox(
        domain::Interval(predicted_center_x - expansion_radius_x, predicted_center_x + expansion_radius_x),
        domain::Interval(predicted_center_y - expansion_radius_y, predicted_center_y + expansion_radius_y),
        domain::Interval(predicted_center_theta - expansion_radius_theta, predicted_center_theta + expansion_radius_theta)
    );
}

} // namespace symbolic::model
