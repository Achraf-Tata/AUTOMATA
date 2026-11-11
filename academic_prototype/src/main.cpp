#include "grid_partition.hpp"
#include "unicycle_model.hpp"
#include "transition_system.hpp"
#include "backward_reachability.hpp"
#include "simulator.hpp"

#include <iostream>
#include <iomanip>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(int argument_count, char* argument_values[]) {
    std::cout << "========================================================\n";
    std::cout << "  Symbolic Control - Standard Academic Prototype (C++20)\n";
    std::cout << "  Milestone: State Discretization & Reachability Synthesis\n";
    std::cout << "========================================================\n\n";

    // -------------------------------------------------------------------------
    // 1. State Space Discretization & Grid Configuration
    // -------------------------------------------------------------------------
    std::size_t cell_count_x = 50;
    std::size_t cell_count_y = 50;
    std::size_t cell_count_theta = 20;

    if (argument_count >= 4) {
        cell_count_x     = static_cast<std::size_t>(std::max(10, std::atoi(argument_values[1])));
        cell_count_y     = static_cast<std::size_t>(std::max(10, std::atoi(argument_values[2])));
        cell_count_theta = static_cast<std::size_t>(std::max(4, std::atoi(argument_values[3])));
    }

    symbolic::domain::BoundingBox continuous_state_domain(0.0, 10.0, 0.0, 10.0, -M_PI, M_PI);
    symbolic::grid::GridPartition grid_partition(continuous_state_domain, cell_count_x, cell_count_y, cell_count_theta);

    std::cout << "[Step 1] State Space Discretization:\n";
    std::cout << "  Domain X:          [0.0, 10.0] x [0.0, 10.0] x [-pi, pi]\n";
    std::cout << "  Grid resolution:   " << grid_partition.cell_count_x << " x "
                                        << grid_partition.cell_count_y << " x "
                                        << grid_partition.cell_count_theta << "\n";
    std::cout << "  Total grid cells:  " << grid_partition.total_cells() << "\n";
    std::cout << "  Cell width dx:     " << std::fixed << std::setprecision(3) << grid_partition.cell_width_x << " m\n";
    std::cout << "  Cell width dy:     " << grid_partition.cell_width_y << " m\n";
    std::cout << "  Cell width dtheta: " << grid_partition.cell_width_theta << " rad ("
              << (grid_partition.cell_width_theta * 180.0 / M_PI) << " deg)\n\n";

    // -------------------------------------------------------------------------
    // 2. System Model & Specification Setup
    // -------------------------------------------------------------------------
    const double sampling_period_seconds = 1.0;
    const double disturbance_bound = 0.05;
    symbolic::model::UnicycleModel unicycle_model(sampling_period_seconds, disturbance_bound);

    std::cout << "[Step 2] System Dynamics & Disturbance Bounds:\n";
    std::cout << "  Sampling period:   tau = " << unicycle_model.sampling_period_seconds << " s\n";
    std::cout << "  Disturbance W:     [-0.05, 0.05]^3\n";
    std::cout << "  Discrete inputs:   " << unicycle_model.num_controls() << " actions\n";

    // Target Region R3: [2.0, 3.0] x [0.5, 1.5]
    symbolic::domain::BoundingBox target_region_box(2.0, 3.0, 0.5, 1.5, -M_PI, M_PI);
    auto target_cell_indices = grid_partition.query_overlapping_cells(target_region_box);
    std::unordered_set<std::size_t> target_state_indices(target_cell_indices.begin(), target_cell_indices.end());

    // Obstacle Region R4: [3.0, 7.0] x [3.0, 7.0]
    symbolic::domain::BoundingBox obstacle_region_box(3.0, 7.0, 3.0, 7.0, -M_PI, M_PI);
    auto obstacle_cell_indices = grid_partition.query_overlapping_cells(obstacle_region_box);
    std::unordered_set<std::size_t> obstacle_state_indices(obstacle_cell_indices.begin(), obstacle_cell_indices.end());

    std::cout << "  Target cells (R3):   " << target_state_indices.size() << " cells\n";
    std::cout << "  Obstacle cells (R4): " << obstacle_state_indices.size() << " cells\n\n";

    // -------------------------------------------------------------------------
    // 3. Symbolic Transition Abstraction
    // -------------------------------------------------------------------------
    std::cout << "[Step 3] Computing Symbolic Transition Abstraction...\n";
    symbolic::abstraction::TransitionSystem transition_system;
    transition_system.build_abstraction(
        grid_partition,
        unicycle_model,
        [](std::size_t processed_cells, std::size_t total_cells) {
            if (processed_cells % (total_cells / 5) == 0 || processed_cells == total_cells) {
                double completion_percentage = 100.0 * static_cast<double>(processed_cells) / static_cast<double>(total_cells);
                std::cout << "  Progress: " << std::fixed << std::setprecision(1) << completion_percentage
                          << "% (" << processed_cells << " / " << total_cells << " cells)\n";
            }
        }
    );

    std::cout << "  Abstraction complete.\n";
    std::cout << "  Total transition edges: " << transition_system.total_transitions() << "\n";
    std::cout << "  Average successors per action: "
              << std::fixed << std::setprecision(2)
              << (static_cast<double>(transition_system.total_transitions()) /
                  (transition_system.state_count * transition_system.input_count)) << "\n\n";

    // -------------------------------------------------------------------------
    // 4. Backward Reachability Fixed-Point Synthesis
    // -------------------------------------------------------------------------
    std::cout << "[Step 4] Running Backward Reachability Synthesis...\n";
    symbolic::synthesis::BackwardReachabilitySolver reachability_solver;
    const std::size_t maximum_synthesis_iterations = 100;
    auto synthesis_result = reachability_solver.solve(
        transition_system,
        target_state_indices,
        obstacle_state_indices,
        maximum_synthesis_iterations);

    std::cout << "  Synthesis status:      " << (synthesis_result.has_converged ? "Converged" : "Horizon reached") << "\n";
    std::cout << "  Iterations completed:  " << synthesis_result.iterations_completed << "\n";
    std::cout << "  Computation time:      " << std::fixed << std::setprecision(4)
              << synthesis_result.computation_duration_seconds << " s\n";
    std::cout << "  Winning set size:      " << synthesis_result.controllable_state_count << " cells\n";
    std::cout << "  Controllable coverage: " << std::fixed << std::setprecision(2)
              << synthesis_result.controllable_state_percentage << " %\n\n";

    // -------------------------------------------------------------------------
    // 5. Closed-Loop Continuous Simulation
    // -------------------------------------------------------------------------
    std::cout << "[Step 5] Simulating Closed-Loop Trajectory under Disturbances...\n";

    // Select a controllable initial pose outside target
    symbolic::model::State initial_robot_state{ 2.5, 1.8, -M_PI * 0.5 };
    std::size_t initial_cell_index = grid_partition.point_to_flat(
        initial_robot_state.x_position,
        initial_robot_state.y_position,
        initial_robot_state.heading_angle);

    if (initial_cell_index >= synthesis_result.feedback_policy.size() ||
        !synthesis_result.feedback_policy[initial_cell_index].has_value() ||
        target_state_indices.find(initial_cell_index) != target_state_indices.end())
    {
        for (std::size_t cell_candidate = 0; cell_candidate < grid_partition.total_cells(); ++cell_candidate) {
            if (synthesis_result.feedback_policy[cell_candidate].has_value() &&
                target_state_indices.find(cell_candidate) == target_state_indices.end() &&
                obstacle_state_indices.find(cell_candidate) == obstacle_state_indices.end())
            {
                auto cell_box = grid_partition.get_cell_box(cell_candidate);
                auto cell_center = cell_box.center();
                initial_robot_state = { cell_center[0], cell_center[1], cell_center[2] };
                initial_cell_index = cell_candidate;
                break;
            }
        }
    }

    std::cout << "  Initial state x0:  (x=" << std::fixed << std::setprecision(3) << initial_robot_state.x_position
              << " m, y=" << initial_robot_state.y_position
              << " m, theta=" << (initial_robot_state.heading_angle * 180.0 / M_PI) << " deg)\n";
    std::cout << "  Initial cell ID:   " << initial_cell_index << "\n";
    std::cout << "  Steps to target:   " << static_cast<int>(synthesis_result.value_function[initial_cell_index]) << "\n";

    symbolic::simulation::ClosedLoopSimulator closed_loop_simulator;
    const std::size_t maximum_simulation_steps = 60;
    const bool apply_disturbances = true;
    auto simulation_trace = closed_loop_simulator.run(
        initial_robot_state,
        grid_partition,
        unicycle_model,
        synthesis_result,
        target_state_indices,
        obstacle_state_indices,
        maximum_simulation_steps,
        apply_disturbances);

    std::cout << "  Simulation outcome: ";
    switch (simulation_trace.outcome) {
        case symbolic::simulation::SimulationOutcome::GoalReached:
            std::cout << "Target Reached (Success)\n";
            break;
        case symbolic::simulation::SimulationOutcome::ObstacleCollision:
            std::cout << "Obstacle Collision\n";
            break;
        case symbolic::simulation::SimulationOutcome::OutOfBounds:
            std::cout << "Out of Bounds\n";
            break;
        case symbolic::simulation::SimulationOutcome::UncontrollableState:
            std::cout << "Uncontrollable State\n";
            break;
        case symbolic::simulation::SimulationOutcome::StepLimitReached:
            std::cout << "Step Limit Reached\n";
            break;
    }

    std::cout << "  Trajectory steps:   " << simulation_trace.total_steps << "\n\n";
    std::cout << "  Sample Trajectory Trace (first 10 steps):\n";
    std::cout << "  Step |      x (m) |      y (m) | theta (deg) | Cell ID |  v (m/s) | omega (rad/s)\n";
    std::cout << "  -----+------------+------------+-------------+---------+----------+--------------\n";

    std::size_t display_limit = std::min<std::size_t>(10, simulation_trace.recorded_steps.size());
    for (std::size_t step_index = 0; step_index < display_limit; ++step_index) {
        const auto& step_entry = simulation_trace.recorded_steps[step_index];
        std::cout << "  " << std::setw(4) << step_index << " | "
                  << std::setw(10) << std::fixed << std::setprecision(3) << step_entry.continuous_state.x_position << " | "
                  << std::setw(10) << step_entry.continuous_state.y_position << " | "
                  << std::setw(11) << (step_entry.continuous_state.heading_angle * 180.0 / M_PI) << " | "
                  << std::setw(7) << step_entry.cell_index << " | "
                  << std::setw(8) << step_entry.applied_control.linear_velocity << " | "
                  << std::setw(12) << step_entry.applied_control.angular_velocity << "\n";
    }

    if (simulation_trace.recorded_steps.size() > 10) {
        std::cout << "  ... (" << (simulation_trace.recorded_steps.size() - 10) << " additional steps omitted) ...\n";
        const auto& final_step_entry = simulation_trace.recorded_steps.back();
        std::cout << "  " << std::setw(4) << (simulation_trace.recorded_steps.size() - 1) << " | "
                  << std::setw(10) << std::fixed << std::setprecision(3) << final_step_entry.continuous_state.x_position << " | "
                  << std::setw(10) << final_step_entry.continuous_state.y_position << " | "
                  << std::setw(11) << (final_step_entry.continuous_state.heading_angle * 180.0 / M_PI) << " | "
                  << std::setw(7) << final_step_entry.cell_index << " | "
                  << std::setw(8) << final_step_entry.applied_control.linear_velocity << " | "
                  << std::setw(12) << final_step_entry.applied_control.angular_velocity << "\n";
    }

    std::cout << "\n========================================================\n";
    std::cout << "  Milestone Execution Completed Successfully.\n";
    std::cout << "========================================================\n";

    return 0;
}
