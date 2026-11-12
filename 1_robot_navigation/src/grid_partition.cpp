#include "grid_partition.hpp"
#include <cmath>
#include <algorithm>

namespace symbolic::grid {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Initializes partition geometry and computes per-axis cell widths
GridPartition::GridPartition(const domain::BoundingBox& total_state_domain,
                             std::size_t initial_cell_count_x,
                             std::size_t initial_cell_count_y,
                             std::size_t initial_cell_count_theta)
    : state_domain(total_state_domain),
      cell_count_x(std::max<std::size_t>(1, initial_cell_count_x)),
      cell_count_y(std::max<std::size_t>(1, initial_cell_count_y)),
      cell_count_theta(std::max<std::size_t>(1, initial_cell_count_theta))
{
    cell_width_x     = state_domain.x_range.width() / static_cast<double>(cell_count_x);
    cell_width_y     = state_domain.y_range.width() / static_cast<double>(cell_count_y);
    cell_width_theta = state_domain.theta_range.width() / static_cast<double>(cell_count_theta);
}

// Returns the total number of discrete states
std::size_t GridPartition::total_cells() const noexcept {
    return cell_count_x * cell_count_y * cell_count_theta;
}

// Maps 3D indices (ix, iy, itheta) to a unique flat cell index
std::size_t GridPartition::coord_to_flat(std::size_t index_x,
                                         std::size_t index_y,
                                         std::size_t index_theta) const noexcept
{
    return index_x * (cell_count_y * cell_count_theta) + index_y * cell_count_theta + index_theta;
}

// Decodes a flat cell index into 3D discrete grid indices
GridIndex GridPartition::flat_to_coord(std::size_t cell_id) const noexcept {
    std::size_t index_theta = cell_id % cell_count_theta;
    std::size_t remainder = cell_id / cell_count_theta;
    std::size_t index_y = remainder % cell_count_y;
    std::size_t index_x = remainder / cell_count_y;
    return { index_x, index_y, index_theta };
}

// Normalizes an angle into [-pi, pi]
double GridPartition::normalize_angle(double angle_radians) noexcept {
    while (angle_radians > M_PI)  angle_radians -= 2.0 * M_PI;
    while (angle_radians < -M_PI) angle_radians += 2.0 * M_PI;
    return angle_radians;
}

// Quantizes continuous coordinates to 3D grid indices with clamping
GridIndex GridPartition::quantize_point(double x_coordinate,double y_coordinate,double theta_angle) const noexcept
{
    theta_angle = normalize_angle(theta_angle);

    double relative_x = (x_coordinate - state_domain.x_range.lower_bound) / cell_width_x;
    double relative_y = (y_coordinate - state_domain.y_range.lower_bound) / cell_width_y;
    double relative_theta = (theta_angle - state_domain.theta_range.lower_bound) / cell_width_theta;

    long long discrete_x = static_cast<long long>(std::floor(relative_x));
    long long discrete_y = static_cast<long long>(std::floor(relative_y));
    long long discrete_theta = static_cast<long long>(std::floor(relative_theta));

    discrete_x = std::clamp<long long>(discrete_x, 0, static_cast<long long>(cell_count_x - 1));
    discrete_y = std::clamp<long long>(discrete_y, 0, static_cast<long long>(cell_count_y - 1));
    discrete_theta = std::clamp<long long>(discrete_theta, 0, static_cast<long long>(cell_count_theta - 1));

    return { static_cast<std::size_t>(discrete_x),
             static_cast<std::size_t>(discrete_y),
             static_cast<std::size_t>(discrete_theta) };
}

// Quantizes continuous coordinates directly to a flat cell index
std::size_t GridPartition::point_to_flat(double x_coordinate,
                                         double y_coordinate,
                                         double theta_angle) const noexcept
{
    GridIndex grid_index = quantize_point(x_coordinate, y_coordinate, theta_angle);
    return coord_to_flat(grid_index.index_x, grid_index.index_y, grid_index.index_theta);
}

// Computes the continuous bounding box for a given 3D grid index
domain::BoundingBox GridPartition::get_cell_box(const GridIndex& grid_index) const noexcept {
    double cell_origin_x = state_domain.x_range.lower_bound + static_cast<double>(grid_index.index_x) * cell_width_x;
    double cell_origin_y = state_domain.y_range.lower_bound + static_cast<double>(grid_index.index_y) * cell_width_y;
    double cell_origin_theta = state_domain.theta_range.lower_bound + static_cast<double>(grid_index.index_theta) * cell_width_theta;

    return domain::BoundingBox(
        domain::Interval(cell_origin_x, cell_origin_x + cell_width_x),
        domain::Interval(cell_origin_y, cell_origin_y + cell_width_y),
        domain::Interval(cell_origin_theta, cell_origin_theta + cell_width_theta)
    );
}

// Computes the continuous bounding box for a flat cell index
domain::BoundingBox GridPartition::get_cell_box(std::size_t cell_id) const noexcept {
    return get_cell_box(flat_to_coord(cell_id));
}

// Finds all cell indices overlapping a given query bounding box
std::vector<std::size_t> GridPartition::query_overlapping_cells(const domain::BoundingBox& query_box) const {
    std::vector<std::size_t> overlapping_cell_indices;

    if (!state_domain.intersects(query_box)) {
        return overlapping_cell_indices;
    }

    long long min_cell_x = static_cast<long long>(std::floor((query_box.x_range.lower_bound - state_domain.x_range.lower_bound) / cell_width_x));
    long long max_cell_x = static_cast<long long>(std::ceil((query_box.x_range.upper_bound - state_domain.x_range.lower_bound) / cell_width_x)) - 1;

    long long min_cell_y = static_cast<long long>(std::floor((query_box.y_range.lower_bound - state_domain.y_range.lower_bound) / cell_width_y));
    long long max_cell_y = static_cast<long long>(std::ceil((query_box.y_range.upper_bound - state_domain.y_range.lower_bound) / cell_width_y)) - 1;

    min_cell_x = std::clamp<long long>(min_cell_x, 0, static_cast<long long>(cell_count_x - 1));
    max_cell_x = std::clamp<long long>(max_cell_x, 0, static_cast<long long>(cell_count_x - 1));
    if (max_cell_x < min_cell_x) max_cell_x = min_cell_x;

    min_cell_y = std::clamp<long long>(min_cell_y, 0, static_cast<long long>(cell_count_y - 1));
    max_cell_y = std::clamp<long long>(max_cell_y, 0, static_cast<long long>(cell_count_y - 1));
    if (max_cell_y < min_cell_y) max_cell_y = min_cell_y;

    // Handle angular interval with periodic wrap-around at +/- PI
    double theta_range_min = query_box.theta_range.lower_bound;
    double theta_range_max = query_box.theta_range.upper_bound;

    std::vector<std::pair<std::size_t, std::size_t>> theta_segment_ranges;

    if (query_box.theta_range.width() >= 2.0 * M_PI - 1e-6) {
        theta_segment_ranges.emplace_back(0, cell_count_theta - 1);
    } else {
        if (theta_range_min < -M_PI && theta_range_max >= -M_PI) {
            theta_range_min += 2.0 * M_PI;
        } else if (theta_range_min < -M_PI && theta_range_max < -M_PI) {
            theta_range_min += 2.0 * M_PI;
            theta_range_max += 2.0 * M_PI;
        } else if (theta_range_max > M_PI && theta_range_min <= M_PI) {
            theta_range_max -= 2.0 * M_PI;
        } else if (theta_range_max > M_PI && theta_range_min > M_PI) {
            theta_range_max -= 2.0 * M_PI;
            theta_range_min -= 2.0 * M_PI;
        }

        long long min_cell_theta = static_cast<long long>(std::floor((theta_range_min - state_domain.theta_range.lower_bound) / cell_width_theta));
        long long max_cell_theta = static_cast<long long>(std::ceil((theta_range_max - state_domain.theta_range.lower_bound) / cell_width_theta)) - 1;

        min_cell_theta = std::clamp<long long>(min_cell_theta, 0, static_cast<long long>(cell_count_theta - 1));
        max_cell_theta = std::clamp<long long>(max_cell_theta, 0, static_cast<long long>(cell_count_theta - 1));

        if (min_cell_theta <= max_cell_theta) {
            theta_segment_ranges.emplace_back(static_cast<std::size_t>(min_cell_theta),
                                              static_cast<std::size_t>(max_cell_theta));
        } else {
            theta_segment_ranges.emplace_back(static_cast<std::size_t>(min_cell_theta), cell_count_theta - 1);
            theta_segment_ranges.emplace_back(0, static_cast<std::size_t>(max_cell_theta));
        }
    }

    for (long long cell_x = min_cell_x; cell_x <= max_cell_x; ++cell_x) {
        for (long long cell_y = min_cell_y; cell_y <= max_cell_y; ++cell_y) {
            for (const auto& [start_theta_idx, end_theta_idx] : theta_segment_ranges) {
                for (std::size_t cell_theta = start_theta_idx; cell_theta <= end_theta_idx; ++cell_theta) {
                    overlapping_cell_indices.push_back(coord_to_flat(static_cast<std::size_t>(cell_x),
                                                                     static_cast<std::size_t>(cell_y),
                                                                     cell_theta));
                }
            }
        }
    }

    return overlapping_cell_indices;
}

} // namespace symbolic::grid
