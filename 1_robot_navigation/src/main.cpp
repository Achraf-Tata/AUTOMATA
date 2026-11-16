#include "grid_partition.hpp"
#include "unicycle_model.hpp"
#include "transition_system.hpp"
#include "dfa.hpp"
#include "product_automaton.hpp"
#include "backward_reachability.hpp"
#include "safety_solver.hpp"
#include "simulator.hpp"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;
using namespace symbolic;
using namespace symbolic::domain;
using namespace symbolic::grid;
using namespace symbolic::automaton;
using namespace symbolic::model;
using namespace symbolic::abstraction;
using namespace symbolic::synthesis;
using namespace symbolic::simulation;
using enum ObservationSymbol;
using enum SimulationOutcome;

// Main walk-through demonstration of the robot navigation project.
// Here are the steps we follow to guide our robot safely to the goal:
// Step 1: Slice the 3D room [0, 10] x [0, 10] x [-pi, pi] into little grid boxes.
// Step 2: Set up the mission checklist (start -> visit R1 or R2 -> reach goal R3, dodge R4).
// Step 3: Define the robot's wheels and steering options (1-second time steps).
// Step 4: Build the move map: test where each move could land us under real-world noise.
// Step 5: Combine the grid boxes with our mission checklist to keep track of both together.
// Step 6: Work backward from the goal to find winning moves for each spot on the map.
// Step 6b: Also find safe driving zones where the robot can cruise forever without crashing.
// Step 7: Put the robot on the virtual floor and let it drive itself to the finish line!
int main(int argc, char* argv[]) {
    // Default partition resolution (can be overridden via command-line arguments)
    size_t nx     = 30;
    size_t ny     = 30;
    size_t ntheta = 12;

    if (argc >= 4) {
        nx     = static_cast<size_t>(max(10, atoi(argv[1])));
        ny     = static_cast<size_t>(max(10, atoi(argv[2])));
        ntheta = static_cast<size_t>(max(4,  atoi(argv[3])));
    }

    // Step 1: Uniform grid partition of the robot's 3D state space
    BoundingBox domain(0.0, 10.0, 0.0, 10.0, -M_PI, M_PI);
    GridPartition grid(domain, nx, ny, ntheta);

    cout << "Grid partition: " << grid.cell_count_x << " x "
                               << grid.cell_count_y << " x "
                               << grid.cell_count_theta
                               << " (" << grid.total_cells() << " cells)\n";
    cout << "Cell resolution: dx = " << fixed << setprecision(3) << grid.cell_width_x
         << " m, dy = " << grid.cell_width_y
         << " m, dtheta = " << grid.cell_width_theta << " rad\n";

    // Step 2: Mission DFA specification: visit either Waypoint R1 or Waypoint R2, then Goal R3, avoiding Obstacle R4
    auto dfa = create_navigation_specification_dfa();

    // Trace 1: FreeSpace -> R1 -> FreeSpace -> Goal R3 (should be Accepted)
    vector<ObservationSymbol> trace_valid = {
        FreeSpace,
        Waypoint1,
        FreeSpace,
        Goal
    };

    // Trace 2: R1 -> R2 -> Goal (violates mutual exclusivity, should be Rejected)
    vector<ObservationSymbol> trace_exclusive = {
        Waypoint1,
        Waypoint2,
        Goal
    };

    // Trace 3: FreeSpace -> Obstacle (hits obstacle, should be Rejected)
    vector<ObservationSymbol> trace_obstacle = {
        FreeSpace,
        Obstacle
    };

    cout << "DFA evaluation (R1 -> R3): "
         << (dfa.accepts_word(trace_valid) ? "Accepted" : "Rejected") << "\n";
    cout << "DFA evaluation (R1 -> R2): "
         << (dfa.accepts_word(trace_exclusive) ? "Accepted" : "Rejected") << "\n";
    cout << "DFA evaluation (Obstacle):  "
         << (dfa.accepts_word(trace_obstacle) ? "Accepted" : "Rejected") << "\n";

    // Step 3: Unicycle kinematics model with bounded disturbances w in [-0.05, 0.05] m
    const double dt = 1.0;
    const double noise = 0.05;
    UnicycleModel model(dt, noise);

    // Step 4: Build the move map: find all possible next boxes for every move from every box
    TransitionSystem ts;
    ts.build_abstraction(grid, model);

    cout << "Abstraction transitions: " << ts.total_transitions() << "\n";

    // Step 5: Combine the grid boxes with the 5 mission stages into one tracking map
    ArenaRegions regions;
    ProductAutomaton automaton(grid.total_cells(), grid, regions);

    cout << "Product automaton states: " << automaton.total_product_states() << "\n";

    // Step 6: Work backward from the goal to calculate the best moves (Algorithm 2: Reach-Avoid)
    BackwardReachabilitySolver solver;
    auto reach_res = solver.solve_product(ts, automaton, 40);

    cout << "Reach-Avoid Search (Alg 2): " << reach_res.iterations_completed << " iterations in "
         << fixed << setprecision(3) << reach_res.computation_duration_seconds << " s, "
         << reach_res.controllable_state_count << " controllable states ("
         << fixed << setprecision(1) << reach_res.controllable_state_percentage << "%)\n";

    // Step 6b: Find all spots where the robot can drive around safely forever (Algorithm 1: Safety Game)
    // Filters out any trap spots until only permanently safe cells remain
    SafetySolver safety;
    auto safety_res = safety.solve_safety_product(ts, automaton, 40);

    cout << "Safety Invariance (Alg 1): " << safety_res.iterations_completed << " iterations in "
         << fixed << setprecision(3) << safety_res.computation_duration_seconds << " s, "
         << safety_res.safe_state_count << " invariant states ("
         << fixed << setprecision(1) << safety_res.safe_state_percentage << "%)\n";

    // Step 7: Test-drive the robot starting from position (1.0, 2.0, 0.0)
    ClosedLoopSimulator simulator;
    State x0{1.0, 2.0, 0.0};

    auto trace = simulator.run_product_closed_loop(
        x0,
        grid,
        model,
        automaton,
        reach_res,
        100,
        true
    );

    cout << "Simulation: " << trace.total_steps << " steps, outcome = ";
    switch (trace.outcome) {
        case GoalReached:
            cout << "GoalReached";
            break;
        case ObstacleCollision:
            cout << "ObstacleCollision";
            break;
        case SpecificationViolation:
            cout << "SpecificationViolation";
            break;
        case OutOfBounds:
            cout << "OutOfBounds";
            break;
        case UncontrollableState:
            cout << "UncontrollableState";
            break;
        case StepLimitReached:
            cout << "StepLimitReached";
            break;
    }
    cout << ", language_satisfied = " << (trace.satisfies_specification_language ? "true" : "false") << "\n";

    return 0;
}
