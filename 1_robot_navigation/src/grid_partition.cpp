#include "grid_partition.hpp"
#include <cmath>
#include <algorithm>

namespace symbolic::grid {

using namespace std;
using namespace domain;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// set up grid dimensions and calculate step sizes: dx, dy, dtheta
GridPartition::GridPartition(const BoundingBox& total_state_domain,
                             size_t initial_cell_count_x,
                             size_t initial_cell_count_y,
                             size_t initial_cell_count_theta)
    : state_domain(total_state_domain),
      cell_count_x(max<size_t>(1, initial_cell_count_x)),
      cell_count_y(max<size_t>(1, initial_cell_count_y)),
      cell_count_theta(max<size_t>(1, initial_cell_count_theta))
{
    cell_width_x     = state_domain.x_range.width() / static_cast<double>(cell_count_x);
    cell_width_y     = state_domain.y_range.width() / static_cast<double>(cell_count_y);
    cell_width_theta = state_domain.theta_range.width() / static_cast<double>(cell_count_theta);
}

// total number of cells in the grid: Nx * Ny * Ntheta
size_t GridPartition::total_cells() const noexcept {
    return cell_count_x * cell_count_y * cell_count_theta;
}

// pack 3D cell coordinates (ix, iy, itheta) into a single 1D array index
size_t GridPartition::coord_to_flat(size_t index_x,
                                    size_t index_y,
                                    size_t index_theta) const noexcept
{
    return index_x * (cell_count_y * cell_count_theta) + index_y * cell_count_theta + index_theta;
}

// unpack a 1D flat index back into 3D cell coordinates (ix, iy, itheta)
GridIndex GridPartition::flat_to_coord(size_t cell_id) const noexcept {
    size_t index_theta = cell_id % cell_count_theta;
    size_t remainder = cell_id / cell_count_theta;
    size_t index_y = remainder % cell_count_y;
    size_t index_x = remainder / cell_count_y;
    return { index_x, index_y, index_theta };
}

// keep angle wrapped inside [-pi, pi]
double GridPartition::normalize_angle(double angle_radians) noexcept {
    while (angle_radians > M_PI)  angle_radians -= 2.0 * M_PI;
    while (angle_radians < -M_PI) angle_radians += 2.0 * M_PI;
    return angle_radians;
}

// convert continuous position (x, y, theta) to cell indices, clamped to stay inside the grid
GridIndex GridPartition::quantize_point(double x_coordinate, double y_coordinate, double theta_angle) const noexcept
{
    theta_angle = normalize_angle(theta_angle);

    double relative_x = (x_coordinate - state_domain.x_range.lower_bound) / cell_width_x;
    double relative_y = (y_coordinate - state_domain.y_range.lower_bound) / cell_width_y;
    double relative_theta = (theta_angle - state_domain.theta_range.lower_bound) / cell_width_theta;

    long long discrete_x = static_cast<long long>(floor(relative_x));
    long long discrete_y = static_cast<long long>(floor(relative_y));
    long long discrete_theta = static_cast<long long>(floor(relative_theta));

    discrete_x = clamp<long long>(discrete_x, 0, static_cast<long long>(cell_count_x - 1));
    discrete_y = clamp<long long>(discrete_y, 0, static_cast<long long>(cell_count_y - 1));
    discrete_theta = clamp<long long>(discrete_theta, 0, static_cast<long long>(cell_count_theta - 1));

    return { static_cast<size_t>(discrete_x),
             static_cast<size_t>(discrete_y),
             static_cast<size_t>(discrete_theta) };
}

// convert continuous position directly to a flat 1D cell index
size_t GridPartition::point_to_flat(double x_coordinate,
                                    double y_coordinate,
                                    double theta_angle) const noexcept
{
    GridIndex grid_index = quantize_point(x_coordinate, y_coordinate, theta_angle);
    return coord_to_flat(grid_index.index_x, grid_index.index_y, grid_index.index_theta);
}

// get the continuous bounding box for a given cell (ix, iy, itheta)
BoundingBox GridPartition::get_cell_box(const GridIndex& grid_index) const noexcept {
    double cell_origin_x = state_domain.x_range.lower_bound + static_cast<double>(grid_index.index_x) * cell_width_x;
    double cell_origin_y = state_domain.y_range.lower_bound + static_cast<double>(grid_index.index_y) * cell_width_y;
    double cell_origin_theta = state_domain.theta_range.lower_bound + static_cast<double>(grid_index.index_theta) * cell_width_theta;

    return BoundingBox(
        Interval(cell_origin_x, cell_origin_x + cell_width_x),
        Interval(cell_origin_y, cell_origin_y + cell_width_y),
        Interval(cell_origin_theta, cell_origin_theta + cell_width_theta)
    );
}

// get the continuous bounding box for a flat 1D cell index
BoundingBox GridPartition::get_cell_box(size_t cell_id) const noexcept {
    return get_cell_box(flat_to_coord(cell_id));
}

// find all grid cells that overlap with a continuous query box (used for computing reachable transitions)
vector<size_t> GridPartition::query_overlapping_cells(const BoundingBox& query_box) const {
    vector<size_t> overlapping_cell_indices;

    if (!state_domain.intersects(query_box)) {
        return overlapping_cell_indices;
    }

    long long min_cell_x = static_cast<long long>(floor((query_box.x_range.lower_bound - state_domain.x_range.lower_bound) / cell_width_x));
    long long max_cell_x = static_cast<long long>(ceil((query_box.x_range.upper_bound - state_domain.x_range.lower_bound) / cell_width_x)) - 1;

    long long min_cell_y = static_cast<long long>(floor((query_box.y_range.lower_bound - state_domain.y_range.lower_bound) / cell_width_y));
    long long max_cell_y = static_cast<long long>(ceil((query_box.y_range.upper_bound - state_domain.y_range.lower_bound) / cell_width_y)) - 1;

    min_cell_x = clamp<long long>(min_cell_x, 0, static_cast<long long>(cell_count_x - 1));
    max_cell_x = clamp<long long>(max_cell_x, 0, static_cast<long long>(cell_count_x - 1));
    if (max_cell_x < min_cell_x) max_cell_x = min_cell_x;

    min_cell_y = clamp<long long>(min_cell_y, 0, static_cast<long long>(cell_count_y - 1));
    max_cell_y = clamp<long long>(max_cell_y, 0, static_cast<long long>(cell_count_y - 1));
    if (max_cell_y < min_cell_y) max_cell_y = min_cell_y;

    // handle angular interval with periodic wrap-around at +/- pi
    double theta_range_min = query_box.theta_range.lower_bound;
    double theta_range_max = query_box.theta_range.upper_bound;

    vector<pair<size_t, size_t>> theta_segment_ranges;

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

        long long min_cell_theta = static_cast<long long>(floor((theta_range_min - state_domain.theta_range.lower_bound) / cell_width_theta));
        long long max_cell_theta = static_cast<long long>(ceil((theta_range_max - state_domain.theta_range.lower_bound) / cell_width_theta)) - 1;

        min_cell_theta = clamp<long long>(min_cell_theta, 0, static_cast<long long>(cell_count_theta - 1));
        max_cell_theta = clamp<long long>(max_cell_theta, 0, static_cast<long long>(cell_count_theta - 1));

        if (min_cell_theta <= max_cell_theta) {
            theta_segment_ranges.emplace_back(static_cast<size_t>(min_cell_theta),
                                              static_cast<size_t>(max_cell_theta));
        } else {
            theta_segment_ranges.emplace_back(static_cast<size_t>(min_cell_theta), cell_count_theta - 1);
            theta_segment_ranges.emplace_back(0, static_cast<size_t>(max_cell_theta));
        }
    }

    for (long long cell_x = min_cell_x; cell_x <= max_cell_x; ++cell_x) {
        for (long long cell_y = min_cell_y; cell_y <= max_cell_y; ++cell_y) {
            for (const auto& [start_theta_idx, end_theta_idx] : theta_segment_ranges) {
                for (size_t cell_theta = start_theta_idx; cell_theta <= end_theta_idx; ++cell_theta) {
                    overlapping_cell_indices.push_back(coord_to_flat(static_cast<size_t>(cell_x),
                                                                     static_cast<size_t>(cell_y),
                                                                     cell_theta));
                }
            }
        }
    }

    return overlapping_cell_indices;
}

} // namespace symbolic::grid
