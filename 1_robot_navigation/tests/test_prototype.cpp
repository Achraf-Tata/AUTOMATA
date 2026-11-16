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
    Interval a(2.0, 5.0);
    Interval b(4.0, 7.0);

    assert(a.width() == 3.0);
    assert(a.radius() == 1.5);
    assert(a.midpoint() == 3.5);
    assert(a.contains(3.0));
    assert(!a.contains(6.0));
    assert(a.overlaps(b));

    auto isect = a.intersect_with(b);
    assert(isect.lower_bound == 4.0 && isect.upper_bound == 5.0);

    auto sum = a + b;
    assert(sum.lower_bound == 6.0 && sum.upper_bound == 12.0);

    auto scaled = a * 2.0;
    assert(scaled.lower_bound == 4.0 && scaled.upper_bound == 10.0);

    cout << "[Test 1] Interval arithmetic passed.\n";
}

// Test 2: Verifies 3D continuous bounding boxes [xmin, xmax] x [ymin, ymax] x [thetamin, thetamax].
// Ensures containment checks, intersection overlap tests, and coordinate-wise
// box intersection calculations work correctly across all three dimensions.
void test_bounding_box() {
    BoundingBox b1(0.0, 2.0, 0.0, 2.0, -1.0, 1.0);
    BoundingBox b2(1.0, 3.0, 1.0, 3.0, 0.0, 2.0);

    assert(b1.contains(1.0, 1.0, 0.0));
    assert(!b1.contains(3.0, 1.0, 0.0));
    assert(b1.intersects(b2));

    auto isect = b1.intersect_with(b2);
    assert(isect.x_range.lower_bound == 1.0 && isect.x_range.upper_bound == 2.0);
    assert(isect.y_range.lower_bound == 1.0 && isect.y_range.upper_bound == 2.0);
    assert(isect.theta_range.lower_bound == 0.0 && isect.theta_range.upper_bound == 1.0);

    cout << "[Test 2] Bounding box operations passed.\n";
}

// Test 3: Verifies uniform state space discretization, cell coordinates, and bijective 1D flat indexing.
// Confirms that coord_to_flat and flat_to_coord are exact mathematical inverses of each other,
// and tests continuous point quantization into discrete cell coordinate bins.
void test_grid_partition() {
    BoundingBox domain(0.0, 10.0, 0.0, 10.0, -M_PI, M_PI);
    GridPartition grid(domain, 20, 20, 8);

    assert(grid.total_cells() == 20 * 20 * 8);

    for (size_t ix = 0; ix < 20; ix += 5) {
        for (size_t iy = 0; iy < 20; iy += 5) {
            for (size_t ith = 0; ith < 8; ith += 2) {
                size_t idx = grid.coord_to_flat(ix, iy, ith);
                auto c = grid.flat_to_coord(idx);
                assert(c.index_x == ix);
                assert(c.index_y == iy);
                assert(c.index_theta == ith);
            }
        }
    }

    auto pt = grid.quantize_point(0.25, 0.25, 0.0);
    assert(pt.index_x == 0);
    assert(pt.index_y == 0);

    cout << "[Test 3] Grid partition and indexing passed.\n";
}

// Test 4: Verifies the 5-state mission DFA transition table and formal language word acceptance.
// Validates:
// - Visiting R1 advances from q1 to q2.
// - Reaching Goal R3 from q2 transitions to accepting state q4.
// - Mutual exclusivity: visiting R2 while already in q2 violates the rule and enters sink trap q5.
// - Hitting Obstacle R4 from any state immediately drops into sink trap q5.
// - Words (sequences of observations) are correctly accepted or rejected according to the language L(A).
void test_dfa_and_language_algorithms() {
    auto dfa = create_navigation_specification_dfa();

    // q1 -> Waypoint1 -> q2
    auto s_r1 = dfa.transition(Initial, Waypoint1);
    assert(s_r1 == VisitedWaypoint1);

    // q2 -> Goal -> q4 (accepting goal state)
    auto s_goal = dfa.transition(s_r1, Goal);
    assert(s_goal == GoalReached);
    assert(dfa.is_accepting_state(s_goal));
    (void)s_goal;

    // Mutual exclusivity check: q2 -> Waypoint2 -> q5 (sink trap)
    auto s_bad = dfa.transition(s_r1, Waypoint2);
    assert(s_bad == SinkTrap);
    assert(dfa.is_sink_state(s_bad));
    (void)s_bad;

    // Obstacle safety check: q1 -> Obstacle -> q5 (sink trap)
    auto s_obs = dfa.transition(Initial, Obstacle);
    assert(s_obs == SinkTrap);
    (void)s_obs;

    // Word acceptance tests
    vector<ObservationSymbol> trace1 = {
        FreeSpace,
        Waypoint1,
        FreeSpace,
        Goal
    };
    assert(dfa.accepts_word(trace1));

    vector<ObservationSymbol> trace2 = {
        Waypoint2,
        Goal
    };
    assert(dfa.accepts_word(trace2));

    vector<ObservationSymbol> bad_trace = {
        Waypoint1,
        Waypoint2,
        Goal
    };
    assert(!dfa.accepts_word(bad_trace));

    cout << "[Test 4] DFA definition and language algorithms passed.\n";
}

// Test 5: Checks combining the grid with mission stages, and searches backward from the goal on a small test grid.
// Uses formula: prod_idx = (stage - 1) * total_cells + spatial_cell.
// Solves a small 8x8x4 test room to verify that working backwards finds the winning moves.
void test_product_automaton_and_reachability_miniature() {
    BoundingBox domain(0.0, 4.0, 0.0, 4.0, -M_PI, M_PI);
    GridPartition grid(domain, 8, 8, 4);
    UnicycleModel model(1.0, 0.02);

    TransitionSystem ts;
    ts.build_abstraction(grid, model);

    ArenaRegions regions;
    regions.waypoint_1 = BoundingBox(1.0, 2.0, 1.0, 2.0, -M_PI, M_PI);
    regions.waypoint_2 = BoundingBox(2.5, 3.5, 1.0, 2.0, -M_PI, M_PI);
    regions.goal       = BoundingBox(1.0, 2.0, 3.0, 4.0, -M_PI, M_PI);
    regions.obstacle   = BoundingBox(2.0, 3.0, 2.0, 3.0, -M_PI, M_PI);

    ProductAutomaton automaton(grid.total_cells(), grid, regions);

    assert(automaton.total_product_states() == grid.total_cells() * 5);

    // Verify product index encoding and decoding round-trip
    for (size_t s = 0; s < grid.total_cells(); s += 20) {
        for (uint8_t q_idx = 1; q_idx <= 5; ++q_idx) {
            auto q = static_cast<AutomatonState>(q_idx);
            size_t p = automaton.encode_product_index(s, q);
            auto dec = automaton.decode_product_index(p);
            assert(dec.spatial_cell_index == s);
            assert(dec.automaton_state == q);
        }
    }

    BackwardReachabilitySolver solver;
    auto res = solver.solve_product(ts, automaton, 25);

    assert(res.controllable_state_count > 0);
    cout << "[Test 5] Combined tracker and backward reachability passed (Winning combinations found: "
         << res.controllable_state_count << ").\n";
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
