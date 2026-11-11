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
    symbolic::domain::Interval first_interval(2.0, 5.0);
    symbolic::domain::Interval second_interval(4.0, 7.0);

    assert(first_interval.width() == 3.0);
    assert(first_interval.radius() == 1.5);
    assert(first_interval.midpoint() == 3.5);
    assert(first_interval.contains(3.0));
    assert(!first_interval.contains(6.0));
    assert(first_interval.overlaps(second_interval));

    auto intersection = first_interval.intersect_with(second_interval);
    assert(intersection.lower_bound == 4.0 && intersection.upper_bound == 5.0);

    auto sum_interval = first_interval + second_interval;
    assert(sum_interval.lower_bound == 6.0 && sum_interval.upper_bound == 12.0);

    auto scaled_interval = first_interval * 2.0;
    assert(scaled_interval.lower_bound == 4.0 && scaled_interval.upper_bound == 10.0);

    std::cout << "[Test 1] Interval arithmetic passed.\n";
}

void test_bounding_box() {
    symbolic::domain::BoundingBox first_box(0.0, 2.0, 0.0, 2.0, -1.0, 1.0);
    symbolic::domain::BoundingBox second_box(1.0, 3.0, 1.0, 3.0, 0.0, 2.0);

    assert(first_box.contains(1.0, 1.0, 0.0));
    assert(!first_box.contains(3.0, 1.0, 0.0));
    assert(first_box.intersects(second_box));

    auto intersection_box = first_box.intersect_with(second_box);
    assert(intersection_box.x_range.lower_bound == 1.0 && intersection_box.x_range.upper_bound == 2.0);
    assert(intersection_box.y_range.lower_bound == 1.0 && intersection_box.y_range.upper_bound == 2.0);
    assert(intersection_box.theta_range.lower_bound == 0.0 && intersection_box.theta_range.upper_bound == 1.0);

    std::cout << "[Test 2] Bounding box operations passed.\n";
}

void test_grid_partition() {
    symbolic::domain::BoundingBox continuous_domain(0.0, 10.0, 0.0, 10.0, -M_PI, M_PI);
    symbolic::grid::GridPartition grid_partition(continuous_domain, 20, 20, 8);

    assert(grid_partition.total_cells() == 20 * 20 * 8);

    // Test flat coordinate round-trip
    for (std::size_t index_x = 0; index_x < 20; index_x += 5) {
        for (std::size_t index_y = 0; index_y < 20; index_y += 5) {
            for (std::size_t index_theta = 0; index_theta < 8; index_theta += 2) {
                std::size_t flat_index = grid_partition.coord_to_flat(index_x, index_y, index_theta);
                auto recovered_coord = grid_partition.flat_to_coord(flat_index);
                assert(recovered_coord.index_x == index_x);
                assert(recovered_coord.index_y == index_y);
                assert(recovered_coord.index_theta == index_theta);
            }
        }
    }

    // Test point quantization
    auto discrete_index = grid_partition.quantize_point(0.25, 0.25, 0.0);
    assert(discrete_index.index_x == 0);
    assert(discrete_index.index_y == 0);

    std::cout << "[Test 3] Grid partition and indexing passed.\n";
}

void test_reachability_solver_miniature() {
    symbolic::domain::BoundingBox miniature_domain(0.0, 4.0, 0.0, 4.0, -M_PI, M_PI);
    symbolic::grid::GridPartition miniature_grid(miniature_domain, 8, 8, 4);
    symbolic::model::UnicycleModel unicycle_model(1.0, 0.02);

    symbolic::abstraction::TransitionSystem transition_system;
    transition_system.build_abstraction(miniature_grid, unicycle_model);

    // Target box at (3, 3)
    symbolic::domain::BoundingBox target_box(3.0, 4.0, 3.0, 4.0, -M_PI, M_PI);
    auto target_cell_indices = miniature_grid.query_overlapping_cells(target_box);
    std::unordered_set<std::size_t> target_states(target_cell_indices.begin(), target_cell_indices.end());
    std::unordered_set<std::size_t> obstacle_states;

    symbolic::synthesis::BackwardReachabilitySolver reachability_solver;
    auto synthesis_result = reachability_solver.solve(transition_system, target_states, obstacle_states, 20);

    assert(synthesis_result.controllable_state_count >= target_states.size());
    std::cout << "[Test 4] Miniature reachability synthesis passed (Winning states: "
              << synthesis_result.controllable_state_count << ").\n";
}

int main() {
    std::cout << "Running academic prototype test suite...\n";
    test_interval_arithmetic();
    test_bounding_box();
    test_grid_partition();
    test_reachability_solver_miniature();
    std::cout << "All academic prototype tests passed successfully.\n";
    return 0;
}
