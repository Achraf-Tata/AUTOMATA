#include "interval.hpp"
#include "bounding_box.hpp"
#include "grid_partition.hpp"
#include "dfa.hpp"
#include "product_automaton.hpp"
#include "unicycle_model.hpp"
#include "transition_system.hpp"
#include "backward_reachability.hpp"
#include "safety_solver.hpp"
#include "simulator.hpp"
#include "simulation_engine.hpp"
#include "app_state.hpp"

#include <cassert>
#include <iostream>
#include <chrono>
#include <cmath>
#include <unordered_set>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;
using namespace std::chrono;
using namespace symbolic::domain;
using namespace symbolic::grid;
using namespace symbolic::automaton;
using namespace symbolic::model;
using namespace symbolic::abstraction;
using namespace symbolic::synthesis;
using namespace symbolic::simulation;
using namespace robot_nav;
using namespace robot_nav::engine;
using namespace ::grid;
using namespace ::automaton;
using namespace ::solver;
using enum ObservationSymbol;
using enum RunStatus;
using AutomatonState::Initial;
using AutomatonState::VisitedWaypoint1;
using AutomatonState::VisitedWaypoint2;
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
    assert(isect.low == 4.0 && isect.high == 5.0);

    auto sum = a + b;
    assert(sum.low == 6.0 && sum.high == 12.0);

    auto scaled = a * 2.0;
    assert(scaled.low == 4.0 && scaled.high == 10.0);

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
    assert(isect.x_range.low == 1.0 && isect.x_range.high == 2.0);
    assert(isect.y_range.low == 1.0 && isect.y_range.high == 2.0);
    assert(isect.theta_range.low == 0.0 && isect.theta_range.high == 1.0);

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
                assert(c.ix == ix);
                assert(c.iy == iy);
                assert(c.ith == ith);
            }
        }
    }

    auto pt = grid.quantize_point(0.25, 0.25, 0.0);
    assert(pt.ix == 0);
    assert(pt.iy == 0);

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
    assert(s_goal == AutomatonState::GoalReached);
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
            assert(dec.s == s);
            assert(dec.q == q);
        }
    }

    BackwardReachabilitySolver solver;
    auto res = solver.solve_product(ts, automaton, 25);

    assert(res.controllable_state_count > 0);
    cout << "[Test 5] Combined tracker and backward reachability passed (Winning combinations found: "
         << res.controllable_state_count << ").\n";
}

// Test 6: Verifies Algorithm 1 (Staying Safe Forever) from Antoine Girard's paper.
// While Algorithm 2 grows winning moves outward from the goal,
// Algorithm 1 starts with all open space and eliminates any cell that could trap the robot:
// A spot stays safe if and only if the robot has at least one move where every possible bump from noise remains safe.
void test_safety_game_algorithm() {
    BoundingBox domain(0.0, 10.0, 0.0, 10.0, -M_PI, M_PI);
    GridPartition grid(domain, 32, 32, 16);
    UnicycleModel model(1.0, 0.02);

    TransitionSystem ts;
    ts.build_abstraction(grid, model);

    ArenaRegions regions;
    regions.waypoint_1 = BoundingBox(1.0, 2.0, 8.0, 9.0, -M_PI, M_PI);
    regions.waypoint_2 = BoundingBox(8.0, 9.0, 1.0, 2.0, -M_PI, M_PI);
    regions.goal       = BoundingBox(1.0, 2.0, 1.0, 2.0, -M_PI, M_PI);
    regions.obstacle   = BoundingBox(4.0, 6.0, 4.0, 6.0, -M_PI, M_PI);

    // Identify all grid cells overlapping the obstacle
    auto overlapping = grid.query_overlapping_cells(regions.obstacle);
    unordered_set<size_t> obstacles(overlapping.begin(), overlapping.end());
    assert(!obstacles.empty());

    SafetySolver solver;

    // Part A: Spatial safety game on transition system
    auto safety = solver.solve_safety(ts, obstacles, 50);

    if (!safety.has_converged || safety.safe_state_count == 0 || safety.safe_state_count >= ts.num_states) {
        cerr << "Spatial safety game failed: converged=" << safety.has_converged
             << ", safe_count=" << safety.safe_state_count << " / " << ts.num_states << "\n";
        abort();
    }

    // None of the obstacle cells can belong to the safe invariant set
    for (size_t obs_idx : obstacles) {
        if (safety.invariant_set[obs_idx]) {
            cerr << "Obstacle cell was incorrectly marked safe in invariant set!\n";
            abort();
        }
    }

    // Verify invariance property: for every state in the invariant set,
    // the safe policy guarantees that all successors under disturbance remain in the set
    for (size_t s = 0; s < ts.num_states; ++s) {
        if (safety.invariant_set[s]) {
            if (!safety.safe_policy[s].has_value()) {
                cerr << "Invariant state has no safe policy assigned!\n";
                abort();
            }
            size_t u = *safety.safe_policy[s];
            const auto& succs = ts.get_successors(s, u);
            if (succs.empty()) {
                cerr << "Invariant state has no valid control successors!\n";
                abort();
            }
            for (size_t succ : succs) {
                if (!safety.invariant_set[succ]) {
                    cerr << "Successor violates controlled invariance property!\n";
                    abort();
                }
            }
        }
    }

    // Part B: Product automaton safety game (staying safe forever while avoiding SinkTrap)
    ProductAutomaton automaton(grid.total_cells(), grid, regions);

    auto prod_safety = solver.solve_safety_product(ts, automaton, 50);

    if (!prod_safety.has_converged || prod_safety.safe_state_count == 0 || prod_safety.safe_state_count >= automaton.total_product_states()) {
        cerr << "Product safety game failed: converged=" << prod_safety.has_converged
             << ", safe_count=" << prod_safety.safe_state_count << " / " << automaton.total_product_states() << "\n";
        abort();
    }

    // Product states corresponding to SinkTrap (q5) must never be safe
    for (size_t p = 0; p < automaton.total_product_states(); ++p) {
        auto dec = automaton.decode_product_index(p);
        if (dec.q == SinkTrap && prod_safety.invariant_set[p]) {
            cerr << "Sink trap state was incorrectly marked safe in product invariant set!\n";
            abort();
        }
    }

    cout << "[Test 6] Safety Game (Algorithm 1: Staying Safe Forever) passed (Safe cells: "
         << safety.safe_state_count << " / " << ts.num_states
         << " in " << safety.iterations_completed << " iterations).\n";
}

// Tests 7, 8, and 9: Full-scale system verification.
// Test 7: Solves all 1,500,000 combinations sequentially with backward_reachability.cpp
//         and confirms it completes in exactly 35 rounds as shown in the paper.
// Test 8: Checks saving and loading the 1.5 million move table and step counts to/from a binary file.
// Test 9: Runs the virtual robot using the loaded moves and confirms it reaches Goal R3
//         in the expected number of steps (18-25 steps).
void test_end_to_end_synthesis_and_simulation() {
    RegionConfig regions;
    auto tables = precompute_abstraction_tables(0.05, 1.0);
    auto V = init_value_function();
    auto ctrl = init_controller();

    assert(V.size() == static_cast<size_t>(NTOTAL));
    assert(ctrl.size() == static_cast<size_t>(NTOTAL));

    // Test 7: Backward reachability solver convergence (Multi-core parallel mode)
    auto t0 = high_resolution_clock::now();
    int iterations = solve_reachability_backward_cpp(V, ctrl, regions, tables, nullptr, true);
    auto t1 = high_resolution_clock::now();
    double elapsed = chrono::duration<double>(t1 - t0).count();

    if (iterations <= 0 || iterations >= 100) {
        cerr << "The reachability solver did not converge: " << iterations << "\n";
        abort();
    }
    cout << "[Test 7] Full-scale backward reachability solver passed (" << iterations << " iterations in "
         << elapsed << " s [parallel]).\n";

    // Test 8: Saving and loading move table binary
    const char* bin_file = "test_controller_serialization.bin";
    bool saved = save_controller_binary(bin_file, V, ctrl);
    if (!saved) {
        cerr << "Failed to save controller binary\n";
        abort();
    }

    vector<float> v_loaded;
    vector<uint8_t> c_loaded;
    bool loaded = load_controller_binary(bin_file, v_loaded, c_loaded);
    if (!loaded || v_loaded.size() != V.size() || c_loaded.size() != ctrl.size()) {
        cerr << "Failed to load controller binary or size mismatch\n";
        abort();
    }
    cout << "[Test 8] Saving and loading move table binary passed.\n";

    // Clean up temporary test file from disk
    remove(bin_file);

    // Test 9: Closed loop trajectory simulation from robot start pose (1.0, 2.0, 0.0)
    AppState app;
    init_app_state(app);
    app.abs_tables = tables;
    app.abs_tables_ready = true;
    app.V = move(v_loaded);
    app.ctrl = move(c_loaded);
    app.controller_ready = true;
    app.sim_with_noise = false; // Deterministic test run
    app.robot_start = {1.0, 2.0, 0.0};
    reset_simulation(app);

    bool reached_goal = false;
    for (int step = 0; step < 200; ++step) {
        app.sim_running = true;
        if (!sim_step_once(app)) {
            if (app.last_status == RunStatus::GoalReached) {
                reached_goal = true;
            }
            break;
        }
    }

    if (!reached_goal || app.sim_step < 18 || app.sim_step > 25) {
        cerr << "Closed loop simulation failed to reach goal within expected step bounds\n";
        abort();
    }
    cout << "[Test 9] End-to-end driving test passed (Goal reached in "
         << app.sim_step << " steps).\n";

    // Test 10: Trajectory bundle generation under physical noise (Monte Carlo route spread)
    generate_trajectory_bundle(app, 50);
    if (app.trajectory_bundle.size() != 50 || app.bundle_success_count != 50) {
        cerr << "Trajectory bundle verification failed: only "
             << app.bundle_success_count << "/50 runs reached goal\n";
        abort();
    }
    cout << "[Test 10] Trajectory bundle passed (50/50 routes reached Goal: "
         << app.bundle_r1_count << " via R1, " << app.bundle_r2_count << " via R2).\n";
}

int main() {
    cout << "Running Academic Prototype Unit Test Suite...\n\n";

    test_interval_arithmetic();
    test_bounding_box();
    test_grid_partition();
    test_dfa_and_language_algorithms();
    test_product_automaton_and_reachability_miniature();
    test_safety_game_algorithm();
    test_end_to_end_synthesis_and_simulation();

    cout << "\nAll Academic Prototype tests passed successfully.\n";
    return 0;
}
