#pragma once

#include "bounding_box.hpp"
#include <vector>
#include <cstddef>

namespace symbolic::grid {

struct GridIndex {
    std::size_t index_x = 0;
    std::size_t index_y = 0;
    std::size_t index_theta = 0;
};

class GridPartition {
public:
    domain::BoundingBox state_domain;
    std::size_t cell_count_x = 0;
    std::size_t cell_count_y = 0;
    std::size_t cell_count_theta = 0;

    double cell_width_x = 0.0;
    double cell_width_y = 0.0;
    double cell_width_theta = 0.0;

    GridPartition(const domain::BoundingBox& total_state_domain,
                  std::size_t initial_cell_count_x,
                  std::size_t initial_cell_count_y,
                  std::size_t initial_cell_count_theta);

    [[nodiscard]] std::size_t total_cells() const noexcept;

    [[nodiscard]] std::size_t coord_to_flat(std::size_t index_x,
                                           std::size_t index_y,
                                           std::size_t index_theta) const noexcept;
    [[nodiscard]] GridIndex flat_to_coord(std::size_t cell_id) const noexcept;

    [[nodiscard]] GridIndex quantize_point(double x_coordinate,
                                          double y_coordinate,
                                          double theta_angle) const noexcept;
    [[nodiscard]] std::size_t point_to_flat(double x_coordinate,
                                            double y_coordinate,
                                            double theta_angle) const noexcept;

    [[nodiscard]] domain::BoundingBox get_cell_box(std::size_t cell_id) const noexcept;
    [[nodiscard]] domain::BoundingBox get_cell_box(const GridIndex& grid_index) const noexcept;

    [[nodiscard]] std::vector<std::size_t> query_overlapping_cells(const domain::BoundingBox& query_box) const;

private:
    static double normalize_angle(double angle_radians) noexcept;
};

} // namespace symbolic::grid
