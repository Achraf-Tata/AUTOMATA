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
GridPartition::GridPartition(const BoundingBox& dom,
                             size_t nx_cells,
                             size_t ny_cells,
                             size_t nth_cells)
    : domain(dom),
      nx(max<size_t>(1, nx_cells)),
      ny(max<size_t>(1, ny_cells)),
      ntheta(max<size_t>(1, nth_cells))
{
    dx     = domain.x_range.width() / static_cast<double>(nx);
    dy     = domain.y_range.width() / static_cast<double>(ny);
    dtheta = domain.theta_range.width() / static_cast<double>(ntheta);
}

// total number of cells in the grid: nx * ny * ntheta
size_t GridPartition::total_cells() const noexcept {
    return nx * ny * ntheta;
}

// pack 3D cell coordinates (ix, iy, itheta) into a single 1D array index
size_t GridPartition::coord_to_flat(size_t ix, size_t iy, size_t ith) const noexcept {
    return ix * (ny * ntheta) + iy * ntheta + ith;
}

// unpack a 1D flat index back into 3D cell coordinates (ix, iy, itheta)
GridIndex GridPartition::flat_to_coord(size_t cell_id) const noexcept {
    size_t ith = cell_id % ntheta;
    size_t rem = cell_id / ntheta;
    size_t iy  = rem % ny;
    size_t ix  = rem / ny;
    return { ix, iy, ith };
}

// keep angle wrapped inside [-pi, pi]
double GridPartition::normalize_angle(double angle) noexcept {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// convert continuous position (x, y, theta) to cell indices, clamped to stay inside the grid
GridIndex GridPartition::quantize_point(double x, double y, double theta) const noexcept {
    theta = normalize_angle(theta);

    double rx  = (x - domain.x_range.low) / dx;
    double ry  = (y - domain.y_range.low) / dy;
    double rth = (theta - domain.theta_range.low) / dtheta;

    long long cx  = static_cast<long long>(floor(rx));
    long long cy  = static_cast<long long>(floor(ry));
    long long cth = static_cast<long long>(floor(rth));

    cx  = clamp<long long>(cx,  0, static_cast<long long>(nx - 1));
    cy  = clamp<long long>(cy,  0, static_cast<long long>(ny - 1));
    cth = clamp<long long>(cth, 0, static_cast<long long>(ntheta - 1));

    return { static_cast<size_t>(cx), static_cast<size_t>(cy), static_cast<size_t>(cth) };
}

// convert continuous position directly to a flat 1D cell index
size_t GridPartition::point_to_flat(double x, double y, double theta) const noexcept {
    GridIndex idx = quantize_point(x, y, theta);
    return coord_to_flat(idx.ix, idx.iy, idx.ith);
}

// get the continuous bounding box for a given cell (ix, iy, itheta)
BoundingBox GridPartition::get_cell_box(const GridIndex& idx) const noexcept {
    double ox  = domain.x_range.low + static_cast<double>(idx.ix) * dx;
    double oy  = domain.y_range.low + static_cast<double>(idx.iy) * dy;
    double oth = domain.theta_range.low + static_cast<double>(idx.ith) * dtheta;

    return BoundingBox(
        Interval(ox, ox + dx),
        Interval(oy, oy + dy),
        Interval(oth, oth + dtheta)
    );
}

// get the continuous bounding box for a flat 1D cell index
BoundingBox GridPartition::get_cell_box(size_t cell_id) const noexcept {
    return get_cell_box(flat_to_coord(cell_id));
}

// find all grid cells that overlap with a continuous query box (used for computing reachable transitions)
vector<size_t> GridPartition::query_overlapping_cells(const BoundingBox& query_box) const {
    vector<size_t> cells;

    if (!domain.intersects(query_box)) {
        return cells;
    }

    long long x_min = static_cast<long long>(floor((query_box.x_range.low - domain.x_range.low) / dx));
    long long x_max = static_cast<long long>(ceil((query_box.x_range.high - domain.x_range.low) / dx)) - 1;

    long long y_min = static_cast<long long>(floor((query_box.y_range.low - domain.y_range.low) / dy));
    long long y_max = static_cast<long long>(ceil((query_box.y_range.high - domain.y_range.low) / dy)) - 1;

    x_min = clamp<long long>(x_min, 0, static_cast<long long>(nx - 1));
    x_max = clamp<long long>(x_max, 0, static_cast<long long>(nx - 1));
    if (x_max < x_min) x_max = x_min;

    y_min = clamp<long long>(y_min, 0, static_cast<long long>(ny - 1));
    y_max = clamp<long long>(y_max, 0, static_cast<long long>(ny - 1));
    if (y_max < y_min) y_max = y_min;

    // handle angular interval with periodic wrap-around at +/- pi
    double th_min = query_box.theta_range.low;
    double th_max = query_box.theta_range.high;

    vector<pair<size_t, size_t>> th_ranges;

    if (query_box.theta_range.width() >= 2.0 * M_PI - 1e-6) {
        th_ranges.emplace_back(0, ntheta - 1);
    } else {
        if (th_min < -M_PI && th_max >= -M_PI) {
            th_min += 2.0 * M_PI;
        } else if (th_min < -M_PI && th_max < -M_PI) {
            th_min += 2.0 * M_PI;
            th_max += 2.0 * M_PI;
        } else if (th_max > M_PI && th_min <= M_PI) {
            th_max -= 2.0 * M_PI;
        } else if (th_max > M_PI && th_min > M_PI) {
            th_max -= 2.0 * M_PI;
            th_min -= 2.0 * M_PI;
        }

        long long cth_min = static_cast<long long>(floor((th_min - domain.theta_range.low) / dtheta));
        long long cth_max = static_cast<long long>(ceil((th_max - domain.theta_range.low) / dtheta)) - 1;

        cth_min = clamp<long long>(cth_min, 0, static_cast<long long>(ntheta - 1));
        cth_max = clamp<long long>(cth_max, 0, static_cast<long long>(ntheta - 1));

        if (cth_min <= cth_max) {
            th_ranges.emplace_back(static_cast<size_t>(cth_min), static_cast<size_t>(cth_max));
        } else {
            th_ranges.emplace_back(static_cast<size_t>(cth_min), ntheta - 1);
            th_ranges.emplace_back(0, static_cast<size_t>(cth_max));
        }
    }

    for (long long cx = x_min; cx <= x_max; ++cx) {
        for (long long cy = y_min; cy <= y_max; ++cy) {
            for (const auto& [start_th, end_th] : th_ranges) {
                for (size_t cth = start_th; cth <= end_th; ++cth) {
                    cells.push_back(coord_to_flat(static_cast<size_t>(cx),
                                                  static_cast<size_t>(cy),
                                                  cth));
                }
            }
        }
    }

    return cells;
}

} // namespace symbolic::grid
