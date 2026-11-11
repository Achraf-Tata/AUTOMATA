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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(int argument_count, char* argument_values[]) {
    std::cout << "====================================================================\n";
    std::cout << "  Symbolic Control - Standard Academic Prototype (C++20)\n";
    std::cout << "  Milestone: State Discretization, DFA Language & Reachability\n";
    std::cout << "====================================================================\n\n";

    // -------------------------------------------------------------------------
    // 1. State Space Discretization
    // -------------------------------------------------------------------------
    std::size_t cell_count_x     = 30;
    std::size_t cell_count_y     = 30;
    std::size_t cell_count_theta = 12;

    if (argument_count >= 4) {
        cell_count_x     = static_cast<std::size_t>(std::max(10, std::atoi(argument_values[1])));
        cell_count_y     = static_cast<std::size_t>(std::max(10, std::atoi(argument_values[2])));
        cell_count_theta = static_cast<std::size_t>(std::max(4,  std::atoi(argument_values[3])));
    }

    symbolic::domain::BoundingBox continuous_state_domain(0.0, 10.0, 0.0, 10.0, -M_PI, M_PI);
    symbolic::grid::GridPartition grid_partition(
        continuous_state_domain, cell_count_x, cell_count_y, cell_count_theta);

    std::cout << "[Step 1] State Space Discretization:\n";
    std::cout << "  Continuous domain: [0.0, 10.0] x [0.0, 10.0] x [-pi, pi]\n";
    std::cout << "  Grid resolution:   " << grid_partition.cell_count_x << " x "
                                        << grid_partition.cell_count_y << " x "
                                        << grid_partition.cell_count_theta << "\n";
    std::cout << "  Total grid cells:  " << grid_partition.total_cells() << "\n";
    std::cout << "  Cell width dx:     " << std::fixed << std::setprecision(3) << grid_partition.cell_width_x << " m\n";
    std::cout << "  Cell width dy:     " << grid_partition.cell_width_y << " m\n";
    std::cout << "  Cell width dtheta: " << grid_partition.cell_width_theta << " rad ("
              << (grid_partition.cell_width_theta * 180.0 / M_PI) << " deg)\n\n";

    // -------------------------------------------------------------------------
    // 2. Definition of the DFA and Language-Related Algorithms
    // -------------------------------------------------------------------------
    std::cout << "[Step 2] Specification Automaton (DFA) & Language Verification:\n";
    auto specification_dfa = symbolic::automaton::create_navigation_specification_dfa();

    std::cout << "  Alphabet Sigma:    { FreeSpace, Waypoint1, Waypoint2, Goal, Obstacle }\n";
    std::cout << "  States Q:          { q1 (Init), q2 (Visited R1), q3 (Visited R2), q4 (Goal), q5 (Sink) }\n";
    std::cout << "  Accepting F:       { q4 }\n";
    std::cout << "  Specification:     \"Visit R1 or R2 exclusively, then reach R3, avoiding R4 all the way.\"\n\n";

    // Test Word A: Valid word visiting Waypoint 1 then Goal
    std::vector<symbolic::automaton::ObservationSymbol> valid_word_a = {
        symbolic::automaton::ObservationSymbol::FreeSpace,
        symbolic::automaton::ObservationSymbol::Waypoint1,
        symbolic::automaton::ObservationSymbol::FreeSpace,
        symbolic::automaton::ObservationSymbol::Goal
    };
    bool is_word_a_accepted = specification_dfa.accepts_word(valid_word_a);
    std::cout << "  Language Test A (R1 -> Goal):   "
              << (is_word_a_accepted ? "ACCEPTED (In L(A))" : "REJECTED") << "\n";

    // Test Word B: Exclusivity violation (visiting both Waypoint 1 and Waypoint 2)
    std::vector<symbolic::automaton::ObservationSymbol> invalid_word_b = {
        symbolic::automaton::ObservationSymbol::Waypoint1,
        symbolic::automaton::ObservationSymbol::Waypoint2,
        symbolic::automaton::ObservationSymbol::Goal
    };
    bool is_word_b_accepted = specification_dfa.accepts_word(invalid_word_b);
    std::cout << "  Language Test B (R1 -> R2):     "
              << (is_word_b_accepted ? "ACCEPTED" : "REJECTED (Exclusivity violation -> Sink q5)") << "\n";

    // Test Word C: Obstacle collision
    std::vector<symbolic::automaton::ObservationSymbol> invalid_word_c = {
        symbolic::automaton::ObservationSymbol::FreeSpace,
        symbolic::automaton::ObservationSymbol::Obstacle
    };
    bool is_word_c_accepted = specification_dfa.accepts_word(invalid_word_c);
    std::cout << "  Language Test C (Hits R4):      "
              << (is_word_c_accepted ? "ACCEPTED" : "REJECTED (Obstacle collision -> Sink q5)") << "\n\n";

    // -------------------------------------------------------------------------
    // 3. Unicycle System Dynamics & Disturbance Bounds
    // -------------------------------------------------------------------------
    const double sampling_period_seconds = 1.0;
    const double disturbance_bound_meters = 0.05;
    symbolic::model::UnicycleModel unicycle_model(sampling_period_seconds, disturbance_bound_meters);

    std::cout << "[Step 3] System Dynamics & Disturbance Bounds:\n";
    std::cout << "  Sampling period:   tau = " << unicycle_model.sampling_period_seconds << " s\n";
    std::cout << "  Disturbances W:    [-0.05, 0.05]^3\n";
    std::cout << "  Discrete controls: " << unicycle_model.num_controls() << " actions in U\n\n";

    // -------------------------------------------------------------------------
    // 4. Symbolic Transition Abstraction
    // -------------------------------------------------------------------------
    std::cout << "[Step 4] Computing Symbolic Abstraction Post Operators...\n";
    symbolic::abstraction::TransitionSystem transition_system;
    transition_system.build_abstraction(
        grid_partition,
        unicycle_model,
        [](std::size_t processed_cells, std::size_t total_cells) {
            if (processed_cells % (total_cells / 4) == 0 || processed_cells == total_cells) {
                double completion = 100.0 * static_cast<double>(processed_cells) / static_cast<double>(total_cells);
                std::cout << "  Abstraction progress: " << std::fixed << std::setprecision(1) << completion
                          << "% (" << processed_cells << " / " << total_cells << " cells)\n";
            }
        }
    );

    std::cout << "  Total transition edges: " << transition_system.total_transitions() << "\n\n";

    // -------------------------------------------------------------------------
    // 5. Product Automaton Construction
    // -------------------------------------------------------------------------
    std::cout << "[Step 5] Composing Product Automaton (S x A)...\n";
    symbolic::automaton::ArenaRegions arena_regions;
    symbolic::automaton::ProductAutomaton product_automaton(
        grid_partition.total_cells(), grid_partition, arena_regions);

    std::cout << "  Spatial cells:     " << product_automaton.spatial_cell_count << "\n";
    std::cout << "  DFA states:        " << product_automaton.DFA_STATE_COUNT << "\n";
    std::cout << "  Product states:    " << product_automaton.total_product_states() << "\n\n";

    // -------------------------------------------------------------------------
    // 6. Backward Reachability Synthesis over Product Automaton
    // -------------------------------------------------------------------------
    std::cout << "[Step 6] Running Backward Reachability Fixed-Point Synthesis...\n";
    symbolic::synthesis::BackwardReachabilitySolver reachability_solver;
    auto synthesis_result = reachability_solver.solve_product(
        transition_system, product_automaton, 40);

    std::cout << "  Synthesis duration:    " << std::fixed << std::setprecision(3)
              << synthesis_result.computation_duration_seconds << " s\n";
    std::cout << "  Iterations to converge:" << synthesis_result.iterations_completed << "\n";
    std::cout << "  Controllable states:   " << synthesis_result.controllable_state_count << " / "
              << product_automaton.total_product_states() << " ("
              << std::fixed << std::setprecision(1) << synthesis_result.controllable_state_percentage << "%)\n\n";

    // -------------------------------------------------------------------------
    // 7. Closed-Loop Simulation with Verification against DFA Language
    // -------------------------------------------------------------------------
    std::cout << "[Step 7] Closed-Loop Continuous Simulation & Formal Verification:\n";
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

    std::cout << "  Initial robot pose:    (" << initial_robot_state.x_position << ", "
              << initial_robot_state.y_position << ", "
              << initial_robot_state.heading_angle << ")\n";
    std::cout << "  Steps taken to goal:   " << simulation_trace.total_steps << "\n";
    std::cout << "  Simulation outcome:    ";
    switch (simulation_trace.outcome) {
        case symbolic::simulation::SimulationOutcome::GoalReached:
            std::cout << "Goal reached successfully in Region 3\n";
            break;
        case symbolic::simulation::SimulationOutcome::ObstacleCollision:
            std::cout << "Collision with obstacle in Region 4\n";
            break;
        case symbolic::simulation::SimulationOutcome::SpecificationViolation:
            std::cout << "Specification violated (transition to sink trap)\n";
            break;
        case symbolic::simulation::SimulationOutcome::OutOfBounds:
            std::cout << "Traversed out of arena boundaries\n";
            break;
        case symbolic::simulation::SimulationOutcome::UncontrollableState:
            std::cout << "Encountered uncontrollable state\n";
            break;
        case symbolic::simulation::SimulationOutcome::StepLimitReached:
            std::cout << "Step limit reached without reaching goal\n";
            break;
    }

    std::cout << "  DFA Language check:    "
              << (simulation_trace.satisfies_specification_language
                      ? "VERIFIED (Trace belongs to L(A))"
                      : "FAILED (Trace not in L(A))") << "\n\n";

    std::cout << "====================================================================\n";
    std::cout << "  Academic Prototype Milestone execution completed successfully.\n";
    std::cout << "====================================================================\n";

    return 0;
}
