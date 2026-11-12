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

// Academic prototype driver demonstrating the complete symbolic control pipeline
int main(int argc, char* argv[]) {
    size_t cell_count_x     = 30;
    size_t cell_count_y     = 30;
    size_t cell_count_theta = 12;

    if (argc >= 4) {
        cell_count_x     = static_cast<size_t>(max(10, atoi(argv[1])));
        cell_count_y     = static_cast<size_t>(max(10, atoi(argv[2])));
        cell_count_theta = static_cast<size_t>(max(4,  atoi(argv[3])));
    }

    symbolic::domain::BoundingBox continuous_state_domain(0.0, 10.0, 0.0, 10.0, -M_PI, M_PI);
    symbolic::grid::GridPartition grid_partition(
        continuous_state_domain, cell_count_x, cell_count_y, cell_count_theta);

    cout << "Grid partition: " << grid_partition.cell_count_x << " x "
                               << grid_partition.cell_count_y << " x "
                               << grid_partition.cell_count_theta
                               << " (" << grid_partition.total_cells() << " cells)\n";
    cout << "Cell resolution: dx = " << fixed << setprecision(3) << grid_partition.cell_width_x
         << " m, dy = " << grid_partition.cell_width_y
         << " m, dtheta = " << grid_partition.cell_width_theta << " rad\n";

    auto specification_dfa = symbolic::automaton::create_navigation_specification_dfa();

    vector<symbolic::automaton::ObservationSymbol> valid_word = {
        symbolic::automaton::ObservationSymbol::FreeSpace,
        symbolic::automaton::ObservationSymbol::Waypoint1,
        symbolic::automaton::ObservationSymbol::FreeSpace,
        symbolic::automaton::ObservationSymbol::Goal
    };

    vector<symbolic::automaton::ObservationSymbol> invalid_word_exclusive = {
        symbolic::automaton::ObservationSymbol::Waypoint1,
        symbolic::automaton::ObservationSymbol::Waypoint2,
        symbolic::automaton::ObservationSymbol::Goal
    };

    vector<symbolic::automaton::ObservationSymbol> invalid_word_obstacle = {
        symbolic::automaton::ObservationSymbol::FreeSpace,
        symbolic::automaton::ObservationSymbol::Obstacle
    };

    cout << "DFA evaluation (R1 -> R3): "
         << (specification_dfa.accepts_word(valid_word) ? "Accepted" : "Rejected") << "\n";
    cout << "DFA evaluation (R1 -> R2): "
         << (specification_dfa.accepts_word(invalid_word_exclusive) ? "Accepted" : "Rejected") << "\n";
    cout << "DFA evaluation (Obstacle):  "
         << (specification_dfa.accepts_word(invalid_word_obstacle) ? "Accepted" : "Rejected") << "\n";

    const double sampling_period_seconds = 1.0;
    const double disturbance_bound_meters = 0.05;
    symbolic::model::UnicycleModel unicycle_model(sampling_period_seconds, disturbance_bound_meters);

    symbolic::abstraction::TransitionSystem transition_system;
    transition_system.build_abstraction(grid_partition, unicycle_model);

    cout << "Abstraction transitions: " << transition_system.total_transitions() << "\n";

    symbolic::automaton::ArenaRegions arena_regions;
    symbolic::automaton::ProductAutomaton product_automaton(
        grid_partition.total_cells(), grid_partition, arena_regions);

    cout << "Product automaton states: " << product_automaton.total_product_states() << "\n";

    symbolic::synthesis::BackwardReachabilitySolver reachability_solver;
    auto synthesis_result = reachability_solver.solve_product(transition_system, product_automaton, 40);

    cout << "Synthesis: " << synthesis_result.iterations_completed << " iterations in "
         << fixed << setprecision(3) << synthesis_result.computation_duration_seconds << " s, "
         << synthesis_result.controllable_state_count << " controllable states ("
         << fixed << setprecision(1) << synthesis_result.controllable_state_percentage << "%)\n";

    symbolic::simulation::ClosedLoopSimulator simulator;
    symbolic::model::State initial_robot_state{1.0, 2.0, 0.0};

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
        case symbolic::simulation::SimulationOutcome::GoalReached:
            cout << "GoalReached";
            break;
        case symbolic::simulation::SimulationOutcome::ObstacleCollision:
            cout << "ObstacleCollision";
            break;
        case symbolic::simulation::SimulationOutcome::SpecificationViolation:
            cout << "SpecificationViolation";
            break;
        case symbolic::simulation::SimulationOutcome::OutOfBounds:
            cout << "OutOfBounds";
            break;
        case symbolic::simulation::SimulationOutcome::UncontrollableState:
            cout << "UncontrollableState";
            break;
        case symbolic::simulation::SimulationOutcome::StepLimitReached:
            cout << "StepLimitReached";
            break;
    }
    cout << ", language_satisfied = " << (simulation_trace.satisfies_specification_language ? "true" : "false") << "\n";

    return 0;
}
