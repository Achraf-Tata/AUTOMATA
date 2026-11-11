#pragma once

#include "bounding_box.hpp"
#include <vector>
#include <array>
#include <cstddef>

namespace symbolic::model {

struct State {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

struct Control {
    double v = 0.0;     // Linear velocity
    double omega = 0.0; // Angular velocity
};

struct DisturbanceBounds {
    double max_x = 0.05;
    double max_y = 0.05;
    double max_theta = 0.05;
};

class UnicycleModel {
public:
    double sampling_period = 1.0;
    DisturbanceBounds disturbance;
    std::vector<Control> control_set;

    UnicycleModel(double tau = 1.0, double dist_bound = 0.05);

    [[nodiscard]] std::size_t num_controls() const noexcept;
    [[nodiscard]] const Control& get_control(std::size_t idx) const;

    [[nodiscard]] State step_exact(const State& current,
                                  const Control& u,
                                  const std::array<double, 3>& w = {0.0, 0.0, 0.0}) const noexcept;

    // Over-approximation of reachable set from a cell bounding box under control u
    [[nodiscard]] domain::BoundingBox compute_reachable_box(const domain::BoundingBox& cell,
                                                            const Control& u) const noexcept;

private:
    void init_discrete_controls();
};

} // namespace symbolic::model
