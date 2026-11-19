#include "unicycle_model.hpp"
#include <cmath>

namespace symbolic::model {

using namespace std;
using namespace domain;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// initialize sampling period dt and noise bounds, then build discrete control list
UnicycleModel::UnicycleModel(double initial_sampling_period, double initial_disturbance_bound)
    : sampling_period_seconds(initial_sampling_period),
      disturbance_bounds{initial_disturbance_bound, initial_disturbance_bound, initial_disturbance_bound}
{
    initialize_discrete_controls();
}

// create the discrete set of (v, w) controls: 3 velocities x 5 steering rates = 15 pairs
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

// total number of available discrete controls (15)
size_t UnicycleModel::num_controls() const noexcept {
    return discrete_controls.size();
}

// get control pair (v, w) by index
const Control& UnicycleModel::get_control(size_t control_index) const {
    return discrete_controls.at(control_index);
}

// simulate one discrete time-step forward using unicycle kinematics plus noise
State UnicycleModel::step_exact(const State& current_state,
                               const Control& control_input,
                               const array<double, 3>& disturbance_vector) const noexcept
{
    double time_step = sampling_period_seconds;
    double next_x = current_state.x_position + time_step * (control_input.linear_velocity * cos(current_state.heading_angle) + disturbance_vector[0]);
    double next_y = current_state.y_position + time_step * (control_input.linear_velocity * sin(current_state.heading_angle) + disturbance_vector[1]);
    double next_theta = current_state.heading_angle + time_step * (control_input.angular_velocity + disturbance_vector[2]);

    while (next_theta > M_PI)  next_theta -= 2.0 * M_PI;
    while (next_theta < -M_PI) next_theta += 2.0 * M_PI;

    return { next_x, next_y, next_theta };
}

// compute a bounding box that encloses all points reachable from this cell under control u
BoundingBox UnicycleModel::compute_reachable_box(const BoundingBox& cell_box,
                                                const Control& control_input) const noexcept
{
    double time_step = sampling_period_seconds;
    double noise_bound = disturbance_bounds.max_disturbance_x;

    double th_min = cell_box.theta_range.lower_bound;
    double th_max = cell_box.theta_range.upper_bound;

    double min_dx = 1e18, max_dx = -1e18;
    double min_dy = 1e18, max_dy = -1e18;

    const double sample_thetas[7] = {
        th_min, th_max,
        0.5 * (th_min + th_max),
        -M_PI * 0.5, 0.0, M_PI * 0.5, M_PI
    };

    for (double th : sample_thetas) {
        if (th < th_min || th > th_max) continue;
        double dx = time_step * control_input.linear_velocity * cos(th);
        double dy = time_step * control_input.linear_velocity * sin(th);
        min_dx = min(min_dx, dx);
        max_dx = max(max_dx, dx);
        min_dy = min(min_dy, dy);
        max_dy = max(max_dy, dy);
    }

    double next_x_min = cell_box.x_range.lower_bound + min_dx - time_step * noise_bound;
    double next_x_max = cell_box.x_range.upper_bound + max_dx + time_step * noise_bound;
    double next_y_min = cell_box.y_range.lower_bound + min_dy - time_step * noise_bound;
    double next_y_max = cell_box.y_range.upper_bound + max_dy + time_step * noise_bound;

    double next_th_min = th_min + time_step * control_input.angular_velocity - time_step * disturbance_bounds.max_disturbance_theta;
    double next_th_max = th_max + time_step * control_input.angular_velocity + time_step * disturbance_bounds.max_disturbance_theta;

    return BoundingBox(
        Interval(next_x_min, next_x_max),
        Interval(next_y_min, next_y_max),
        Interval(next_th_min, next_th_max)
    );
}

} // namespace symbolic::model