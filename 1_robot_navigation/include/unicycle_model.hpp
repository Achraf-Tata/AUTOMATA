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

    UnicycleModel(double initial_sampling_period = 1.0,
                  double initial_disturbance_bound = 0.05);

    [[nodiscard]] std::size_t num_controls() const noexcept;
    [[nodiscard]] const Control& get_control(std::size_t control_index) const;

    [[nodiscard]] State step_exact(
        const State& current_state,
        const Control& control_input,
        const std::array<double, 3>& disturbance_vector = {0.0, 0.0, 0.0}) const noexcept;

    // Over-approximation of reachable set from a cell bounding box under control input
    [[nodiscard]] domain::BoundingBox compute_reachable_box(
        const domain::BoundingBox& cell_box,
        const Control& control_input) const noexcept;

private:
    void initialize_discrete_controls();
};

} // namespace symbolic::model
