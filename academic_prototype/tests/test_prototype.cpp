#include "interval.hpp"
#include "bounding_box.hpp"
#include "grid_partition.hpp"
#include "unicycle_model.hpp"
#include "transition_system.hpp"
#include "backward_reachability.hpp"

#include <cassert>
#include <iostream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void test_interval_arithmetic() {
    symbolic::domain::Interval i1(2.0, 5.0);
    symbolic::domain::Interval i2(4.0, 7.0);

    assert(i1.width() == 3.0);
    assert(i1.radius() == 1.5);
    assert(i1.midpoint() == 3.5);
    assert(i1.contains(3.0));
    assert(!i1.contains(6.0));
    assert(i1.overlaps(i2));

    auto inter = i1.intersect_with(i2);
    assert(inter.lower == 4.0 && inter.upper == 5.0);

    auto sum = i1 + i2;
    assert(sum.lower == 6.0 && sum.upper == 12.0);

    auto scaled = i1 * 2.0;
    assert(scaled.lower == 4.0 && scaled.upper == 10.0);

    std::cout << "[Test 1] Interval arithmetic passed.\n";
}

void test_bounding_box() {
    symbolic::domain::BoundingBox b1(0.0, 2.0, 0.0, 2.0, -1.0, 1.0);
    symbolic::domain::BoundingBox b2(1.0, 3.0, 1.0, 3.0, 0.0, 2.0);

    assert(b1.contains(1.0, 1.0, 0.0));
    assert(!b1.contains(3.0, 1.0, 0.0));
    assert(b1.intersects(b2));

    auto inter = b1.intersect_with(b2);
    assert(inter.x_range.lower == 1.0 && inter.x_range.upper == 2.0);
    assert(inter.y_range.lower == 1.0 && inter.y_range.upper == 2.0);
    assert(inter.theta_range.lower == 0.0 && inter.theta_range.upper == 1.0);

    std::cout << "[Test 2] Bounding box operations passed.\n";
}

void test_grid_partition() {
    symbolic::domain::BoundingBox domain(0.0, 10.0, 0.0, 10.0, -M_PI, M_PI);
    symbolic::grid::GridPartition grid(domain, 20, 20, 8);

    assert(grid.total_cells() == 20 * 20 * 8);

    // Test flat coordinate round-trip
    for (std::size_t ix = 0; ix < 20; ix += 5) {
        for (std::size_t iy = 0; iy < 20; iy += 5) {
            for (std::size_t ith = 0; ith < 8; ith += 2) {
                std::size_t flat = grid.coord_to_flat(ix, iy, ith);
                auto coord = grid.flat_to_coord(flat);
                assert(coord.ix == ix);
                assert(coord.iy == iy);
                assert(coord.itheta == ith);
            }
        }
    }

    // Test point quantization
    auto idx = grid.quantize_point(0.25, 0.25, 0.0);
    assert(idx.ix == 0);
    assert(idx.iy == 0);

    std::cout << "[Test 3] Grid partition and indexing passed.\n";
}

void test_reachability_solver_mini() {
    symbolic::domain::BoundingBox domain(0.0, 4.0, 0.0, 4.0, -M_PI, M_PI);
    symbolic::grid::GridPartition grid(domain, 8, 8, 4);
    symbolic::model::UnicycleModel model(1.0, 0.02);

    symbolic::abstraction::TransitionSystem ts;
    ts.build_abstraction(grid, model);

    // Goal at (3, 3)
    symbolic::domain::BoundingBox target_box(3.0, 4.0, 3.0, 4.0, -M_PI, M_PI);
    auto target_cells = grid.query_overlapping_cells(target_box);
    std::unordered_set<std::size_t> targets(target_cells.begin(), target_cells.end());
    std::unordered_set<std::size_t> obstacles;

    symbolic::synthesis::BackwardReachabilitySolver solver;
    auto res = solver.solve(ts, targets, obstacles, 20);

    assert(res.winning_set_size >= targets.size());
    std::cout << "[Test 4] Miniature reachability synthesis passed (Winning states: "
              << res.winning_set_size << ").\n";
}

int main() {
    std::cout << "Running academic prototype test suite...\n";
    test_interval_arithmetic();
    test_bounding_box();
    test_grid_partition();
    test_reachability_solver_mini();
    std::cout << "All academic prototype tests passed successfully.\n";
    return 0;
}
