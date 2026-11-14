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

using namespace std;
using namespace symbolic::domain;
using namespace symbolic::grid;
using namespace symbolic::automaton;
using namespace symbolic::model;
using namespace symbolic::abstraction;
using namespace symbolic::synthesis;
using namespace symbolic::simulation;
using enum ObservationSymbol;
using AutomatonState::Initial;
using AutomatonState::VisitedWaypoint1;
using AutomatonState::VisitedWaypoint2;
using AutomatonState::GoalReached;
using AutomatonState::SinkTrap;

// Test 1: Verifies 1D closed interval arithmetic operations [a, b].
// We check interval width, midpoint, radius, point containment,
// intersection [2, 5] cap [4, 7] = [4, 5], Minkowski sum [2, 5] + [4, 7] = [6, 12],
// and positive scalar scaling [2, 5] * 2 = [4, 10].
void test_interval_arithmetic() {
    Interval first_interval(2.0, 5.0);
    Interval second_interval(4.0, 7.0);

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

    cout << "[Test 1] Interval arithmetic passed.\n";
}

// Test 2: Verifies 3D continuous bounding boxes [xmin, xmax] x [ymin, ymax] x [thetamin, thetamax].
// Ensures containment checks, intersection overlap tests, and coordinate-wise
// box intersection calculations work correctly across all three dimensions.
void test_bounding_box() {
    BoundingBox first_box(0.0, 2.0, 0.0, 2.0, -1.0, 1.0);
    BoundingBox second_box(1.0, 3.0, 1.0, 3.0, 0.0, 2.0);

    assert(first_box.contains(1.0, 1.0, 0.0));
    assert(!first_box.contains(3.0, 1.0, 0.0));
    assert(first_box.intersects(second_box));

    auto intersection_box = first_box.intersect_with(second_box);
    assert(intersection_box.x_range.lower_bound == 1.0 && intersection_box.x_range.upper_bound == 2.0);
    assert(intersection_box.y_range.lower_bound == 1.0 && intersection_box.y_range.upper_bound == 2.0);
    assert(intersection_box.theta_range.lower_bound == 0.0 && intersection_box.theta_range.upper_bound == 1.0);

    cout << "[Test 2] Bounding box operations passed.\n";
}

// Test 3: Verifies uniform state space discretization, cell coordinates, and bijective 1D flat indexing.
// Confirms that coord_to_flat and flat_to_coord are exact mathematical inverses of each other,
// and tests continuous point quantization into discrete cell coordinate bins.
void test_grid_partition() {
    BoundingBox continuous_domain(0.0, 10.0, 0.0, 10.0, -M_PI, M_PI);
    GridPartition grid_partition(continuous_domain, 20, 20, 8);

    assert(grid_partition.total_cells() == 20 * 20 * 8);

    for (size_t index_x = 0; index_x < 20; index_x += 5) {
        for (size_t index_y = 0; index_y < 20; index_y += 5) {
            for (size_t index_theta = 0; index_theta < 8; index_theta += 2) {
                size_t flat_index = grid_partition.coord_to_flat(index_x, index_y, index_theta);
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

    cout << "[Test 3] Grid partition and indexing passed.\n";
}

// Test 4: Verifies deterministic finite automaton (DFA) transitions, exclusivity enforcement, and word acceptance.
// Tests the mission specification: visit either Waypoint R1 or R2, then Goal R3, avoiding Obstacle R4.
void test_dfa_and_language_algorithms() {
    auto dfa = create_navigation_specification_dfa();

    // q1 -> Waypoint1 -> q2
    auto state_after_r1 = dfa.transition(Initial, Waypoint1);
    assert(state_after_r1 == VisitedWaypoint1);

    // q2 -> Goal -> q4 (accepting)
    auto state_after_goal = dfa.transition(state_after_r1, Goal);
    assert(state_after_goal == GoalReached);
    assert(dfa.is_accepting_state(state_after_goal));
    (void)state_after_goal;

    // Exclusivity test: q2 -> Waypoint2 -> q5 (sink)
    auto state_exclusivity_violation = dfa.transition(state_after_r1, Waypoint2);
    assert(state_exclusivity_violation == SinkTrap);
    assert(dfa.is_sink_state(state_exclusivity_violation));
    (void)state_exclusivity_violation;

    // Obstacle test: q1 -> Obstacle -> q5 (sink)
    auto state_obstacle = dfa.transition(Initial, Obstacle);
    assert(state_obstacle == SinkTrap);
    (void)state_obstacle;

    // Language word acceptance
    vector<ObservationSymbol> accepted_trace_1 = {
        FreeSpace,
        Waypoint1,
        FreeSpace,
        Goal
    };
    assert(dfa.accepts_word(accepted_trace_1));

    vector<ObservationSymbol> accepted_trace_2 = {
        Waypoint2,
        Goal
    };
    assert(dfa.accepts_word(accepted_trace_2));

    vector<ObservationSymbol> rejected_trace = {
        Waypoint1,
        Waypoint2,
        Goal
    };
    assert(!dfa.accepts_word(rejected_trace));

    cout << "[Test 4] DFA definition and language algorithms passed.\n";
}

// Test 5: Verifies product automaton state encoding and backward reachability synthesis on a miniature problem.
// Tests the bijective index formula: prod_idx = (q - 1) * total_cells + spatial_cell.
// Solves a small 8x8x4 grid to verify that backward reachability successfully propagates
// winning states backward from the goal.
void test_product_automaton_and_reachability_miniature() {
    BoundingBox miniature_domain(0.0, 4.0, 0.0, 4.0, -M_PI, M_PI);
    GridPartition miniature_grid(miniature_domain, 8, 8, 4);
    UnicycleModel unicycle_model(1.0, 0.02);

    TransitionSystem transition_system;
    transition_system.build_abstraction(miniature_grid, unicycle_model);

    ArenaRegions test_regions;
    test_regions.waypoint_1 = BoundingBox(1.0, 2.0, 1.0, 2.0, -M_PI, M_PI);
    test_regions.waypoint_2 = BoundingBox(2.5, 3.5, 1.0, 2.0, -M_PI, M_PI);
    test_regions.goal       = BoundingBox(1.0, 2.0, 3.0, 4.0, -M_PI, M_PI);
    test_regions.obstacle   = BoundingBox(2.0, 3.0, 2.0, 3.0, -M_PI, M_PI);

    ProductAutomaton product_automaton(
        miniature_grid.total_cells(), miniature_grid, test_regions);

    assert(product_automaton.total_product_states() == miniature_grid.total_cells() * 5);

    // Test product index round-trip
    for (size_t spatial_idx = 0; spatial_idx < miniature_grid.total_cells(); spatial_idx += 20) {
        for (uint8_t dfa_idx = 1; dfa_idx <= 5; ++dfa_idx) {
            auto dfa_st = static_cast<AutomatonState>(dfa_idx);
            size_t prod_idx = product_automaton.encode_product_index(spatial_idx, dfa_st);
            auto decoded = product_automaton.decode_product_index(prod_idx);
            assert(decoded.spatial_cell_index == spatial_idx);
            assert(decoded.automaton_state == dfa_st);
        }
    }

    BackwardReachabilitySolver solver;
    auto synthesis_result = solver.solve_product(transition_system, product_automaton, 25);

    assert(synthesis_result.controllable_state_count > 0);
    cout << "[Test 5] Product automaton reachability synthesis passed (Controllable product states: "
         << synthesis_result.controllable_state_count << ").\n";
}

int main() {
    cout << "Running Academic Prototype Unit Test Suite...\n\n";

    test_interval_arithmetic();
    test_bounding_box();
    test_grid_partition();
    test_dfa_and_language_algorithms();
    test_product_automaton_and_reachability_miniature();

    cout << "\nAll Academic Prototype tests passed successfully.\n";
    return 0;
}
