#pragma once

#include "bounding_box.hpp"
#include <vector>
#include <array>
#include <cmath>
#include <cstddef>

namespace symbolic::model {

using namespace std;
using namespace domain;

// continuous robot state: position (x, y) in meters, heading theta in radians
struct State {
    double x_position = 0.0;
    double y_position = 0.0;
    double heading_angle = 0.0;
};

// control action: linear velocity v (m/s) and steering rate w (rad/s)
struct Control {
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;
};

// maximum bounded disturbance (noise) on each state dimension
struct DisturbanceBounds {
    double max_disturbance_x = 0.05;
    double max_disturbance_y = 0.05;
    double max_disturbance_theta = 0.05;
};

// discrete-time kinematic unicycle robot model with bounded noise
class UnicycleModel {
public:
    double sampling_period_seconds = 1.0;
    DisturbanceBounds disturbance_bounds;
    vector<Control> discrete_controls;

    // set sampling period dt (default 1.0 s) and noise bound (default 0.05)
    UnicycleModel(double initial_sampling_period = 1.0,double initial_disturbance_bound = 0.05);

    // number of discrete control choices available
    [[nodiscard]] size_t num_controls() const noexcept;

    // get control action (v, w) by index
    [[nodiscard]] const Control& get_control(size_t control_index) const;

    // simulate one step forward: x_{k+1} = x_k + dt * (v*cos(theta) + w_x), etc.
    [[nodiscard]] State step_exact(
        const State& current_state,
        const Control& control_input,
        const array<double, 3>& disturbance_vector = {0.0, 0.0, 0.0}) const noexcept;

    // compute a bounding box that encloses all points reachable from this cell under control u
    [[nodiscard]] BoundingBox compute_reachable_box(
        const BoundingBox& cell_box,
        const Control& control_input) const noexcept;

private:
    // generate the grid of discrete (v, w) control pairs
    void initialize_discrete_controls();
};

} // namespace symbolic::model

namespace symbolic {
    using model::State;
    using model::Control;
    using model::DisturbanceBounds;
    using model::UnicycleModel;
}

namespace unicycle {

using namespace std;

struct State {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

struct Control {
    double v = 0.0;
    double w = 0.0;
};

struct Box {
    double x_min, x_max;
    double y_min, y_max;
    double th_min, th_max;
};

inline constexpr array<Control, 15> ALL_CONTROLS = {{
    {0.25,  -1.0}, {0.625, -1.0}, {1.0,   -1.0},
    {0.25,  -0.5}, {0.625, -0.5}, {1.0,   -0.5},
    {0.25,   0.0}, {0.625,  0.0}, {1.0,    0.0},
    {0.25,   0.5}, {0.625,  0.5}, {1.0,    0.5},
    {0.25,   1.0}, {0.625,  1.0}, {1.0,    1.0}
}};

inline double wrap_angle(double a) noexcept {
    while (a >  3.14159265358979323846) a -= 2.0 * 3.14159265358979323846;
    while (a < -3.14159265358979323846) a += 2.0 * 3.14159265358979323846;
    return a;
}

inline State step(const State& s, const Control& u,
                  const array<double, 3>& noise,
                  double dt = 1.0) noexcept {
    return {
        s.x     + dt * u.v * cos(s.theta) + dt * noise[0],
        s.y     + dt * u.v * sin(s.theta) + dt * noise[1],
        wrap_angle(s.theta + dt * u.w     + dt * noise[2])
    };
}

} // namespace unicycle
