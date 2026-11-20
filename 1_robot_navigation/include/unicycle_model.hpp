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
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

// control action: linear velocity v (m/s) and steering rate w (rad/s)
struct Control {
    double v = 0.0;
    double w = 0.0;
};

// maximum bounded disturbance (noise) on each state dimension
struct DisturbanceBounds {
    double max_x = 0.05;
    double max_y = 0.05;
    double max_theta = 0.05;
};

// discrete-time kinematic unicycle robot model with bounded noise
class UnicycleModel {
public:
    double dt = 1.0;
    DisturbanceBounds noise_bounds;
    vector<Control> controls;

    // set sampling period dt (default 1.0 s) and noise bound (default 0.05)
    UnicycleModel(double dt_sec = 1.0, double noise = 0.05);

    // number of discrete control choices available
    [[nodiscard]] size_t num_controls() const noexcept;

    // get control action (v, w) by index
    [[nodiscard]] const Control& get_control(size_t u_idx) const;

    // simulate one step forward: x_{k+1} = x_k + dt * (v*cos(theta) + w_x), etc.
    [[nodiscard]] State step_exact(
        const State& state,
        const Control& u,
        const array<double, 3>& w = {0.0, 0.0, 0.0}) const noexcept;

    // compute a bounding box that encloses all points reachable from this cell under control u
    [[nodiscard]] BoundingBox compute_reachable_box(
        const BoundingBox& cell_box,
        const Control& u) const noexcept;

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
