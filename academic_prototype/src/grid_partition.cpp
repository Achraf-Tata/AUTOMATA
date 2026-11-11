#include "grid_partition.hpp"
#include <cmath>
#include <algorithm>

namespace symbolic::grid {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GridPartition::GridPartition(const domain::BoundingBox& state_domain,
                             std::size_t num_x,
                             std::size_t num_y,
                             std::size_t num_theta)
    : domain(state_domain),
      nx(std::max<std::size_t>(1, num_x)),
      ny(std::max<std::size_t>(1, num_y)),
      ntheta(std::max<std::size_t>(1, num_theta))
{
    dx     = domain.x_range.width() / static_cast<double>(nx);
    dy     = domain.y_range.width() / static_cast<double>(ny);
    dtheta = domain.theta_range.width() / static_cast<double>(ntheta);
}

std::size_t GridPartition::total_cells() const noexcept {
    return nx * ny * ntheta;
}

std::size_t GridPartition::coord_to_flat(std::size_t ix, std::size_t iy, std::size_t itheta) const noexcept {
    return ix * (ny * ntheta) + iy * ntheta + itheta;
}

GridIndex GridPartition::flat_to_coord(std::size_t cell_id) const noexcept {
    std::size_t itheta = cell_id % ntheta;
    std::size_t rem = cell_id / ntheta;
    std::size_t iy = rem % ny;
    std::size_t ix = rem / ny;
    return { ix, iy, itheta };
}

double GridPartition::normalize_angle(double theta) noexcept {
    while (theta > M_PI)  theta -= 2.0 * M_PI;
    while (theta < -M_PI) theta += 2.0 * M_PI;
    return theta;
}

GridIndex GridPartition::quantize_point(double x, double y, double theta) const noexcept {
    theta = normalize_angle(theta);

    double rel_x = (x - domain.x_range.lower) / dx;
    double rel_y = (y - domain.y_range.lower) / dy;
    double rel_th = (theta - domain.theta_range.lower) / dtheta;

    long long ix = static_cast<long long>(std::floor(rel_x));
    long long iy = static_cast<long long>(std::floor(rel_y));
    long long ith = static_cast<long long>(std::floor(rel_th));

    ix = std::clamp<long long>(ix, 0, static_cast<long long>(nx - 1));
    iy = std::clamp<long long>(iy, 0, static_cast<long long>(ny - 1));
    ith = std::clamp<long long>(ith, 0, static_cast<long long>(ntheta - 1));

    return { static_cast<std::size_t>(ix),
             static_cast<std::size_t>(iy),
             static_cast<std::size_t>(ith) };
}

std::size_t GridPartition::point_to_flat(double x, double y, double theta) const noexcept {
    GridIndex idx = quantize_point(x, y, theta);
    return coord_to_flat(idx.ix, idx.iy, idx.itheta);
}

domain::BoundingBox GridPartition::get_cell_box(const GridIndex& idx) const noexcept {
    double x0 = domain.x_range.lower + static_cast<double>(idx.ix) * dx;
    double y0 = domain.y_range.lower + static_cast<double>(idx.iy) * dy;
    double th0 = domain.theta_range.lower + static_cast<double>(idx.itheta) * dtheta;

    return domain::BoundingBox(
        domain::Interval(x0, x0 + dx),
        domain::Interval(y0, y0 + dy),
        domain::Interval(th0, th0 + dtheta)
    );
}

domain::BoundingBox GridPartition::get_cell_box(std::size_t cell_id) const noexcept {
    return get_cell_box(flat_to_coord(cell_id));
}

std::vector<std::size_t> GridPartition::query_overlapping_cells(const domain::BoundingBox& box) const {
    std::vector<std::size_t> result;

    if (!domain.intersects(box)) {
        return result;
    }

    long long ix_min = static_cast<long long>(std::floor((box.x_range.lower - domain.x_range.lower) / dx));
    long long ix_max = static_cast<long long>(std::ceil((box.x_range.upper - domain.x_range.lower) / dx)) - 1;

    long long iy_min = static_cast<long long>(std::floor((box.y_range.lower - domain.y_range.lower) / dy));
    long long iy_max = static_cast<long long>(std::ceil((box.y_range.upper - domain.y_range.lower) / dy)) - 1;

    ix_min = std::clamp<long long>(ix_min, 0, static_cast<long long>(nx - 1));
    ix_max = std::clamp<long long>(ix_max, 0, static_cast<long long>(nx - 1));
    if (ix_max < ix_min) ix_max = ix_min;

    iy_min = std::clamp<long long>(iy_min, 0, static_cast<long long>(ny - 1));
    iy_max = std::clamp<long long>(iy_max, 0, static_cast<long long>(ny - 1));
    if (iy_max < iy_min) iy_max = iy_min;

    // Handle angular interval with periodic wrap-around
    double r_min = box.theta_range.lower;
    double r_max = box.theta_range.upper;

    std::vector<std::pair<std::size_t, std::size_t>> theta_ranges;

    if (box.theta_range.width() >= 2.0 * M_PI - 1e-6) {
        theta_ranges.emplace_back(0, ntheta - 1);
    } else {
        if (r_min < -M_PI && r_max >= -M_PI) {
            r_min += 2.0 * M_PI;
        } else if (r_min < -M_PI && r_max < -M_PI) {
            r_min += 2.0 * M_PI;
            r_max += 2.0 * M_PI;
        } else if (r_max > M_PI && r_min <= M_PI) {
            r_max -= 2.0 * M_PI;
        } else if (r_max > M_PI && r_min > M_PI) {
            r_max -= 2.0 * M_PI;
            r_min -= 2.0 * M_PI;
        }

        long long lo = static_cast<long long>(std::floor((r_min - domain.theta_range.lower) / dtheta));
        long long hi = static_cast<long long>(std::ceil((r_max - domain.theta_range.lower) / dtheta)) - 1;

        lo = std::clamp<long long>(lo, 0, static_cast<long long>(ntheta - 1));
        hi = std::clamp<long long>(hi, 0, static_cast<long long>(ntheta - 1));

        if (lo <= hi) {
            theta_ranges.emplace_back(static_cast<std::size_t>(lo), static_cast<std::size_t>(hi));
        } else {
            theta_ranges.emplace_back(static_cast<std::size_t>(lo), ntheta - 1);
            theta_ranges.emplace_back(0, static_cast<std::size_t>(hi));
        }
    }

    for (long long ix = ix_min; ix <= ix_max; ++ix) {
        for (long long iy = iy_min; iy <= iy_max; ++iy) {
            for (const auto& [th_a, th_b] : theta_ranges) {
                for (std::size_t ith = th_a; ith <= th_b; ++ith) {
                    result.push_back(coord_to_flat(static_cast<std::size_t>(ix),
                                                   static_cast<std::size_t>(iy),
                                                   ith));
                }
            }
        }
    }

    return result;
}

} // namespace symbolic::grid
