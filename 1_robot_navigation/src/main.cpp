#include "grid_partition.hpp"
#include "unicycle_model.hpp"
#include "transition_system.hpp"
#include "dfa.hpp"
#include "product_automaton.hpp"
#include "backward_reachability.hpp"
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

// Academic prototype demonstration program.
// This main function walks step-by-step through the entire symbolic control pipeline:
// Step 1: Discretize the continuous 3D state space [0, 10] x [0, 10] x [-pi, pi] into uniform grid cells.
// Step 2: Build the mission DFA (q1 -> R1/R2 -> R3) and verify test traces.
// Step 3: Define unicycle kinematics and discrete control inputs (sampling period dt = 1.0 s).
// Step 4: Construct the symbolic transition system (over-approximating cell-to-cell transitions).
// Step 5: Compute the synchronous product automaton (S x Q) combining spatial cells with DFA states.
// Step 6: Run backward reachability synthesis to find the winning set and controller policy.
// Step 7: Simulate the closed-loop robot in continuous time under bounded disturbances.
int main(int argc, char* argv[]) {
    // Default partition resolution (can be overridden via command-line arguments)
    size_t cell_count_x     = 30;
    size_t cell_count_y     = 30;
    size_t cell_count_theta = 12;

    if (argc >= 4) {
        cell_count_x     = static_cast<size_t>(max(10, atoi(argv[1])));
        cell_count_y     = static_cast<size_t>(max(10, atoi(argv[2])));
        cell_count_theta = static_cast<size_t>(max(4,  atoi(argv[3])));
    }

    // Step 1: Uniform grid partition of the robot's 3D state space
    BoundingBox continuous_state_domain(0.0, 10.0, 0.0, 10.0, -M_PI, M_PI);
    GridPartition grid_partition(
        continuous_state_domain, cell_count_x, cell_count_y, cell_count_theta);

    cout << "Grid partition: " << grid_partition.cell_count_x << " x "
                               << grid_partition.cell_count_y << " x "
                               << grid_partition.cell_count_theta
                               << " (" << grid_partition.total_cells() << " cells)\n";
    cout << "Cell resolution: dx = " << fixed << setprecision(3) << grid_partition.cell_width_x
         << " m, dy = " << grid_partition.cell_width_y
         << " m, dtheta = " << grid_partition.cell_width_theta << " rad\n";

    // Step 2: Mission DFA specification: visit either Waypoint R1 or Waypoint R2, then Goal R3, avoiding Obstacle R4
    auto specification_dfa = create_navigation_specification_dfa();

    // Trace 1: FreeSpace -> R1 -> FreeSpace -> Goal R3 (should be Accepted)
    vector<ObservationSymbol> valid_word = {
        FreeSpace,
        Waypoint1,
        FreeSpace,
        Goal
    };

    // Trace 2: R1 -> R2 -> Goal (violates mutual exclusivity, should be Rejected)
    vector<ObservationSymbol> invalid_word_exclusive = {
        Waypoint1,
        Waypoint2,
        Goal
    };

    // Trace 3: FreeSpace -> Obstacle (hits obstacle, should be Rejected)
    vector<ObservationSymbol> invalid_word_obstacle = {
        FreeSpace,
        Obstacle
    };

    cout << "DFA evaluation (R1 -> R3): "
         << (specification_dfa.accepts_word(valid_word) ? "Accepted" : "Rejected") << "\n";
    cout << "DFA evaluation (R1 -> R2): "
         << (specification_dfa.accepts_word(invalid_word_exclusive) ? "Accepted" : "Rejected") << "\n";
    cout << "DFA evaluation (Obstacle):  "
         << (specification_dfa.accepts_word(invalid_word_obstacle) ? "Accepted" : "Rejected") << "\n";

    // Step 3: Unicycle kinematics model with bounded disturbances w in [-0.05, 0.05] m
    const double sampling_period_seconds = 1.0;
    const double disturbance_bound_meters = 0.05;
    UnicycleModel unicycle_model(sampling_period_seconds, disturbance_bound_meters);

    // Step 4: Build symbolic abstraction by computing reachable set over-approximations for every cell and control
    TransitionSystem transition_system;
    transition_system.build_abstraction(grid_partition, unicycle_model);

    cout << "Abstraction transitions: " << transition_system.total_transitions() << "\n";

    // Step 5: Product automaton combining the discrete spatial cells with the 5 DFA states
    ArenaRegions arena_regions;
    ProductAutomaton product_automaton(
        grid_partition.total_cells(), grid_partition, arena_regions);

    cout << "Product automaton states: " << product_automaton.total_product_states() << "\n";

    // Step 6: Backward reachability fixed-point solver working backward from the goal state
    BackwardReachabilitySolver reachability_solver;
    auto synthesis_result = reachability_solver.solve_product(transition_system, product_automaton, 40);

    cout << "Synthesis: " << synthesis_result.iterations_completed << " iterations in "
         << fixed << setprecision(3) << synthesis_result.computation_duration_seconds << " s, "
         << synthesis_result.controllable_state_count << " controllable states ("
         << fixed << setprecision(1) << synthesis_result.controllable_state_percentage << "%)\n";

    // Step 7: Closed-loop continuous simulation of the unicycle from start pose (1.0, 2.0, 0.0)
    ClosedLoopSimulator simulator;
    State initial_robot_state{1.0, 2.0, 0.0};

    auto simulation_trace = simulator.run_product_closed_loop(
        initial_robot_state,
        grid_partition,
        unicycle_model,
        product_automaton,
        synthesis_result,
        100,
        true
    );

    cout << "Simulation: " << simulation_trace.total_steps << " steps, outcome = ";
    switch (simulation_trace.outcome) {
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
    cout << ", language_satisfied = " << (simulation_trace.satisfies_specification_language ? "true" : "false") << "\n";

    return 0;
}
