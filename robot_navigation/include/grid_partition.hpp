#pragma once

#include "bounding_box.hpp"
#include <vector>
#include <cstddef>

namespace symbolic::grid {

using namespace std;
using namespace domain;

// discrete 3D cell coordinates: (ix, iy, itheta)
struct GridIndex {
    size_t ix = 0;
    size_t iy = 0;
    size_t ith = 0;
};

// uniform grid partition of robot state space (x, y, theta)
class GridPartition {
public:
    BoundingBox domain;
    size_t nx = 0;
    size_t ny = 0;
    size_t ntheta = 0;

    double dx = 0.0;
    double dy = 0.0;
    double dtheta = 0.0;

    // initialize grid with continuous domain bounds and number of cells along each axis
    GridPartition(const BoundingBox& dom,
                  size_t nx_cells,
                  size_t ny_cells,
                  size_t nth_cells);

    // total number of grid cells: nx * ny * ntheta
    [[nodiscard]] size_t total_cells() const noexcept;

    // convert 3D grid coordinate (ix, iy, itheta) into a flat 1D array index
    [[nodiscard]] size_t coord_to_flat(size_t ix, size_t iy, size_t ith) const noexcept;

    // unpack a flat 1D index back into 3D grid coordinates (ix, iy, itheta)
    [[nodiscard]] GridIndex flat_to_coord(size_t cell_id) const noexcept;

    // find which cell (ix, iy, itheta) contains continuous point (x, y, theta)
    [[nodiscard]] GridIndex quantize_point(double x, double y, double theta) const noexcept;

    // find flat cell index directly from continuous position (x, y, theta)
    [[nodiscard]] size_t point_to_flat(double x, double y, double theta) const noexcept;

    // get the continuous bounding box for a cell given its flat index
    [[nodiscard]] BoundingBox get_cell_box(size_t cell_id) const noexcept;

    // get the continuous bounding box for a cell given its (ix, iy, itheta) index
    [[nodiscard]] BoundingBox get_cell_box(const GridIndex& idx) const noexcept;

    // find all grid cells that overlap with a continuous box (used for computing transitions)
    [[nodiscard]] vector<size_t> query_overlapping_cells(const BoundingBox& query_box) const;

private:
    // wrap angle into [-pi, pi]
    static double normalize_angle(double angle) noexcept;
};

} // namespace symbolic::grid

namespace symbolic {
    using grid::GridIndex;
    using grid::GridPartition;
}
