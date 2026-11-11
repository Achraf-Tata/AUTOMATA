#pragma once

#include "bounding_box.hpp"
#include <vector>
#include <cstddef>
#include <tuple>

namespace symbolic::grid {

struct GridIndex {
    std::size_t ix = 0;
    std::size_t iy = 0;
    std::size_t itheta = 0;
};

class GridPartition {
public:
    domain::BoundingBox domain;
    std::size_t nx = 0;
    std::size_t ny = 0;
    std::size_t ntheta = 0;

    double dx = 0.0;
    double dy = 0.0;
    double dtheta = 0.0;

    GridPartition(const domain::BoundingBox& state_domain,
                  std::size_t num_x,
                  std::size_t num_y,
                  std::size_t num_theta);

    [[nodiscard]] std::size_t total_cells() const noexcept;

    [[nodiscard]] std::size_t coord_to_flat(std::size_t ix, std::size_t iy, std::size_t itheta) const noexcept;
    [[nodiscard]] GridIndex flat_to_coord(std::size_t cell_id) const noexcept;

    [[nodiscard]] GridIndex quantize_point(double x, double y, double theta) const noexcept;
    [[nodiscard]] std::size_t point_to_flat(double x, double y, double theta) const noexcept;

    [[nodiscard]] domain::BoundingBox get_cell_box(std::size_t cell_id) const noexcept;
    [[nodiscard]] domain::BoundingBox get_cell_box(const GridIndex& idx) const noexcept;

    [[nodiscard]] std::vector<std::size_t> query_overlapping_cells(const domain::BoundingBox& box) const;

private:
    static double normalize_angle(double theta) noexcept;
};

} // namespace symbolic::grid
