#include "unicycle_model.hpp"
#include <cmath>

namespace symbolic::model {

using namespace std;
using namespace domain;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// initialize sampling period dt and noise bounds, then build discrete control list
UnicycleModel::UnicycleModel(double dt_sec, double noise)
    : dt(dt_sec),
      noise_bounds{noise, noise, noise}
{
    initialize_discrete_controls();
}

// create the discrete set of (v, w) controls: 3 velocities x 5 steering rates = 15 pairs
void UnicycleModel::initialize_discrete_controls() {
    controls.clear();
    for (const auto& control : ::unicycle::ALL_CONTROLS) {
        controls.push_back({control.v, control.w});
    }
}

// total number of available discrete controls (15)
size_t UnicycleModel::num_controls() const noexcept {
    return controls.size();
}

// get control pair (v, w) by index
const Control& UnicycleModel::get_control(size_t u_idx) const {
    return controls.at(u_idx);
}

// simulate one discrete time-step forward using unicycle kinematics plus noise
State UnicycleModel::step_exact(const State& state,
                               const Control& u,
                               const array<double, 3>& w) const noexcept
{
    double nx  = state.x + dt * (u.v * cos(state.theta) + w[0]);
    double ny  = state.y + dt * (u.v * sin(state.theta) + w[1]);
    double nth = state.theta + dt * (u.w + w[2]);

    while (nth > M_PI)  nth -= 2.0 * M_PI;
    while (nth < -M_PI) nth += 2.0 * M_PI;

    return { nx, ny, nth };
}

// compute a bounding box that encloses all points reachable from this cell under control u
BoundingBox UnicycleModel::compute_reachable_box(const BoundingBox& cell_box,
                                                const Control& u) const noexcept
{
    double noise = noise_bounds.max_x;

    double th_min = cell_box.theta_range.low;
    double th_max = cell_box.theta_range.high;

    double min_dx = 1e18, max_dx = -1e18;
    double min_dy = 1e18, max_dy = -1e18;

    const double thetas[7] = {
        th_min, th_max,
        0.5 * (th_min + th_max),
        -M_PI * 0.5, 0.0, M_PI * 0.5, M_PI
    };

    for (double th : thetas) {
        if (th < th_min || th > th_max) continue;
        double dx = dt * u.v * cos(th);
        double dy = dt * u.v * sin(th);
        min_dx = min(min_dx, dx);
        max_dx = max(max_dx, dx);
        min_dy = min(min_dy, dy);
        max_dy = max(max_dy, dy);
    }

    double nx_min = cell_box.x_range.low + min_dx - dt * noise;
    double nx_max = cell_box.x_range.high + max_dx + dt * noise;
    double ny_min = cell_box.y_range.low + min_dy - dt * noise;
    double ny_max = cell_box.y_range.high + max_dy + dt * noise;

    double nth_min = th_min + dt * u.w - dt * noise_bounds.max_theta;
    double nth_max = th_max + dt * u.w + dt * noise_bounds.max_theta;

    return BoundingBox(
        Interval(nx_min, nx_max),
        Interval(ny_min, ny_max),
        Interval(nth_min, nth_max)
    );
}

} // namespace symbolic::model
