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

    // Construct partition given bounding box domain and resolution along each axis
    GridPartition(const domain::BoundingBox& total_state_domain,
                  std::size_t initial_cell_count_x,
                  std::size_t initial_cell_count_y,
                  std::size_t initial_cell_count_theta);

    // Total number of discrete cells: Nx * Ny * Ntheta
    [[nodiscard]] std::size_t total_cells() const noexcept;

    // Convert 3D discrete grid indices (ix, iy, itheta) to a 1D flat cell index
    [[nodiscard]] std::size_t coord_to_flat(std::size_t index_x,
                                           std::size_t index_y,
                                           std::size_t index_theta) const noexcept;

    // Convert a 1D flat cell index back into 3D discrete grid indices
    [[nodiscard]] GridIndex flat_to_coord(std::size_t cell_id) const noexcept;

    // Quantize continuous coordinates (x, y, theta) to discrete 3D grid index
    [[nodiscard]] GridIndex quantize_point(double x_coordinate,
                                          double y_coordinate,
                                          double theta_angle) const noexcept;

    // Quantize continuous coordinates directly to a 1D flat cell index
    [[nodiscard]] std::size_t point_to_flat(double x_coordinate,
                                            double y_coordinate,
                                            double theta_angle) const noexcept;

    // Compute the 3D continuous bounding box of a cell from its flat index
    [[nodiscard]] domain::BoundingBox get_cell_box(std::size_t cell_id) const noexcept;

    // Compute the 3D continuous bounding box of a cell from its 3D grid index
    [[nodiscard]] domain::BoundingBox get_cell_box(const GridIndex& grid_index) const noexcept;

    // Query all cell indices whose continuous bounding boxes intersect the query box
    [[nodiscard]] std::vector<std::size_t> query_overlapping_cells(const domain::BoundingBox& query_box) const;

private:
    // Normalize an angle in radians into the interval [-pi, pi]
    static double normalize_angle(double angle_radians) noexcept;
};

} // namespace symbolic::grid
