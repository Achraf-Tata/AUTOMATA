#pragma once

#include "bounding_box.hpp"
#include <vector>
#include <array>
#include <cstddef>

namespace symbolic::model {

struct State {
    double x_position = 0.0;
    double y_position = 0.0;
    double heading_angle = 0.0;
};

struct Control {
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;
};

struct DisturbanceBounds {
    double max_disturbance_x = 0.05;
    double max_disturbance_y = 0.05;
    double max_disturbance_theta = 0.05;
};

class UnicycleModel {
public:
    double sampling_period_seconds = 1.0;
    DisturbanceBounds disturbance_bounds;
    std::vector<Control> discrete_controls;

    // Constructor initializing sampling period and disturbance bounds
    UnicycleModel(double initial_sampling_period = 1.0,
                  double initial_disturbance_bound = 0.05);

    // Returns the total number of discrete control inputs in U
    [[nodiscard]] std::size_t num_controls() const noexcept;

    // Retrieves discrete control action by index
    [[nodiscard]] const Control& get_control(std::size_t control_index) const;

    // Computes single-step continuous state integration with disturbances
    [[nodiscard]] State step_exact(
        const State& current_state,
        const Control& control_input,
        const std::array<double, 3>& disturbance_vector = {0.0, 0.0, 0.0}) const noexcept;

    // Computes over-approximation bounding box of reachable set from a cell under control input
    [[nodiscard]] domain::BoundingBox compute_reachable_box(
        const domain::BoundingBox& cell_box,
        const Control& control_input) const noexcept;

private:
    // Populates admissible discrete forward velocities and steering rates
    void initialize_discrete_controls();
};

} // namespace symbolic::model
