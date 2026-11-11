#include "interval.hpp"
#include "bounding_box.hpp"
#include "grid_partition.hpp"
#include "dfa.hpp"
#include "product_automaton.hpp"
#include "unicycle_model.hpp"
#include "transition_system.hpp"
#include "backward_reachability.hpp"
#include "simulator.hpp"

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

    auto discrete_index = grid_partition.quantize_point(0.25, 0.25, 0.0);
    assert(discrete_index.index_x == 0);
    assert(discrete_index.index_y == 0);

    std::cout << "[Test 3] Grid partition and indexing passed.\n";
}

void test_dfa_and_language_algorithms() {
    auto dfa = symbolic::automaton::create_navigation_specification_dfa();

    // q1 -> Waypoint1 -> q2
    auto state_after_r1 = dfa.transition(symbolic::automaton::AutomatonState::Initial,
                                         symbolic::automaton::ObservationSymbol::Waypoint1);
    assert(state_after_r1 == symbolic::automaton::AutomatonState::VisitedWaypoint1);

    // q2 -> Goal -> q4 (accepting)
    auto state_after_goal = dfa.transition(state_after_r1, symbolic::automaton::ObservationSymbol::Goal);
    assert(state_after_goal == symbolic::automaton::AutomatonState::GoalReached);
    assert(dfa.is_accepting_state(state_after_goal));
    (void)state_after_goal;

    // Exclusivity test: q2 -> Waypoint2 -> q5 (sink)
    auto state_exclusivity_violation = dfa.transition(state_after_r1, symbolic::automaton::ObservationSymbol::Waypoint2);
    assert(state_exclusivity_violation == symbolic::automaton::AutomatonState::SinkTrap);
    assert(dfa.is_sink_state(state_exclusivity_violation));
    (void)state_exclusivity_violation;

    // Obstacle test: q1 -> Obstacle -> q5 (sink)
    auto state_obstacle = dfa.transition(symbolic::automaton::AutomatonState::Initial,
                                         symbolic::automaton::ObservationSymbol::Obstacle);
    assert(state_obstacle == symbolic::automaton::AutomatonState::SinkTrap);
    (void)state_obstacle;

    // Language word acceptance
    std::vector<symbolic::automaton::ObservationSymbol> accepted_trace_1 = {
        symbolic::automaton::ObservationSymbol::FreeSpace,
        symbolic::automaton::ObservationSymbol::Waypoint1,
        symbolic::automaton::ObservationSymbol::FreeSpace,
        symbolic::automaton::ObservationSymbol::Goal
    };
    assert(dfa.accepts_word(accepted_trace_1));

    std::vector<symbolic::automaton::ObservationSymbol> accepted_trace_2 = {
        symbolic::automaton::ObservationSymbol::Waypoint2,
        symbolic::automaton::ObservationSymbol::Goal
    };
    assert(dfa.accepts_word(accepted_trace_2));

    std::vector<symbolic::automaton::ObservationSymbol> rejected_trace = {
        symbolic::automaton::ObservationSymbol::Waypoint1,
        symbolic::automaton::ObservationSymbol::Waypoint2,
        symbolic::automaton::ObservationSymbol::Goal
    };
    assert(!dfa.accepts_word(rejected_trace));

    std::cout << "[Test 4] DFA definition and language algorithms passed.\n";
}

void test_product_automaton_and_reachability_miniature() {
    symbolic::domain::BoundingBox miniature_domain(0.0, 4.0, 0.0, 4.0, -M_PI, M_PI);
    symbolic::grid::GridPartition miniature_grid(miniature_domain, 8, 8, 4);
    symbolic::model::UnicycleModel unicycle_model(1.0, 0.02);

    symbolic::abstraction::TransitionSystem transition_system;
    transition_system.build_abstraction(miniature_grid, unicycle_model);

    symbolic::automaton::ArenaRegions test_regions;
    test_regions.waypoint_1 = symbolic::domain::BoundingBox(1.0, 2.0, 1.0, 2.0, -M_PI, M_PI);
    test_regions.waypoint_2 = symbolic::domain::BoundingBox(2.5, 3.5, 1.0, 2.0, -M_PI, M_PI);
    test_regions.goal       = symbolic::domain::BoundingBox(1.0, 2.0, 3.0, 4.0, -M_PI, M_PI);
    test_regions.obstacle   = symbolic::domain::BoundingBox(2.0, 3.0, 2.0, 3.0, -M_PI, M_PI);

    symbolic::automaton::ProductAutomaton product_automaton(
        miniature_grid.total_cells(), miniature_grid, test_regions);

    assert(product_automaton.total_product_states() == miniature_grid.total_cells() * 5);

    // Test product index round-trip
    for (std::size_t spatial_idx = 0; spatial_idx < miniature_grid.total_cells(); spatial_idx += 20) {
        for (std::uint8_t dfa_idx = 1; dfa_idx <= 5; ++dfa_idx) {
            auto dfa_st = static_cast<symbolic::automaton::AutomatonState>(dfa_idx);
            std::size_t prod_idx = product_automaton.encode_product_index(spatial_idx, dfa_st);
            auto decoded = product_automaton.decode_product_index(prod_idx);
            assert(decoded.spatial_cell_index == spatial_idx);
            assert(decoded.automaton_state == dfa_st);
        }
    }

    symbolic::synthesis::BackwardReachabilitySolver solver;
    auto synthesis_result = solver.solve_product(transition_system, product_automaton, 25);

    assert(synthesis_result.controllable_state_count > 0);
    std::cout << "[Test 5] Product automaton reachability synthesis passed (Controllable product states: "
              << synthesis_result.controllable_state_count << ").\n";
}

int main() {
    std::cout << "Running Academic Prototype Unit Test Suite...\n\n";

    test_interval_arithmetic();
    test_bounding_box();
    test_grid_partition();
    test_dfa_and_language_algorithms();
    test_product_automaton_and_reachability_miniature();

    std::cout << "\nAll Academic Prototype tests passed successfully.\n";
    return 0;
}
