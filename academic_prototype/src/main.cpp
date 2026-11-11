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

int main(int argc, char* argv[]) {
    std::cout << "========================================================\n";
    std::cout << "  Symbolic Control - Standard Academic Prototype (C++20)\n";
    std::cout << "  Milestone: State Discretization & Reachability Synthesis\n";
    std::cout << "========================================================\n\n";

    // -------------------------------------------------------------------------
    // 1. State Space Discretization & Grid Configuration
    // -------------------------------------------------------------------------
    std::size_t nx = 50;
    std::size_t ny = 50;
    std::size_t ntheta = 20;

    if (argc >= 4) {
        nx = static_cast<std::size_t>(std::max(10, std::atoi(argv[1])));
        ny = static_cast<std::size_t>(std::max(10, std::atoi(argv[2])));
        ntheta = static_cast<std::size_t>(std::max(4, std::atoi(argv[3])));
    }

    symbolic::domain::BoundingBox state_domain(0.0, 10.0, 0.0, 10.0, -M_PI, M_PI);
    symbolic::grid::GridPartition partition(state_domain, nx, ny, ntheta);

    std::cout << "[Step 1] State Space Discretization:\n";
    std::cout << "  Domain X:          [0.0, 10.0] x [0.0, 10.0] x [-pi, pi]\n";
    std::cout << "  Grid resolution:   " << partition.nx << " x " << partition.ny << " x " << partition.ntheta << "\n";
    std::cout << "  Total grid cells:  " << partition.total_cells() << "\n";
    std::cout << "  Cell width dx:     " << std::fixed << std::setprecision(3) << partition.dx << " m\n";
    std::cout << "  Cell width dy:     " << partition.dy << " m\n";
    std::cout << "  Cell width dtheta: " << partition.dtheta << " rad ("
              << (partition.dtheta * 180.0 / M_PI) << " deg)\n\n";

    // -------------------------------------------------------------------------
    // 2. System Model & Specification Setup
    // -------------------------------------------------------------------------
    symbolic::model::UnicycleModel model(1.0, 0.05);

    std::cout << "[Step 2] System Dynamics & Disturbance Bounds:\n";
    std::cout << "  Sampling period:   tau = " << model.sampling_period << " s\n";
    std::cout << "  Disturbance W:     [-0.05, 0.05]^3\n";
    std::cout << "  Discrete inputs:   " << model.num_controls() << " actions\n";

    // Regions matching project specification:
    // Target Region R3: [2.0, 3.0] x [0.5, 1.5]
    symbolic::domain::BoundingBox target_box(2.0, 3.0, 0.5, 1.5, -M_PI, M_PI);
    auto target_cells_vec = partition.query_overlapping_cells(target_box);
    std::unordered_set<std::size_t> target_states(target_cells_vec.begin(), target_cells_vec.end());

    // Obstacle Region R4: [3.0, 7.0] x [3.0, 7.0]
    symbolic::domain::BoundingBox obstacle_box(3.0, 7.0, 3.0, 7.0, -M_PI, M_PI);
    auto obstacle_cells_vec = partition.query_overlapping_cells(obstacle_box);
    std::unordered_set<std::size_t> obstacle_states(obstacle_cells_vec.begin(), obstacle_cells_vec.end());

    std::cout << "  Target cells (R3): " << target_states.size() << " cells\n";
    std::cout << "  Obstacle cells (R4): " << obstacle_states.size() << " cells\n\n";

    // -------------------------------------------------------------------------
    // 3. Symbolic Transition Abstraction
    // -------------------------------------------------------------------------
    std::cout << "[Step 3] Computing Symbolic Transition Abstraction...\n";
    symbolic::abstraction::TransitionSystem ts;
    ts.build_abstraction(partition, model, [](std::size_t current, std::size_t total) {
        if (current % (total / 5) == 0 || current == total) {
            double pct = 100.0 * static_cast<double>(current) / static_cast<double>(total);
            std::cout << "  Progress: " << std::fixed << std::setprecision(1) << pct
                      << "% (" << current << " / " << total << " cells)\n";
        }
    });

    std::cout << "  Abstraction complete.\n";
    std::cout << "  Total transition edges: " << ts.total_transitions() << "\n";
    std::cout << "  Average successors/action: "
              << std::fixed << std::setprecision(2)
              << (static_cast<double>(ts.total_transitions()) / (ts.num_states * ts.num_inputs)) << "\n\n";

    // -------------------------------------------------------------------------
    // 4. Backward Reachability Fixed-Point Synthesis
    // -------------------------------------------------------------------------
    std::cout << "[Step 4] Running Backward Reachability Synthesis...\n";
    symbolic::synthesis::BackwardReachabilitySolver solver;
    auto synthesis_result = solver.solve(ts, target_states, obstacle_states, 100);

    std::cout << "  Synthesis status:      " << (synthesis_result.converged ? "Converged" : "Horizon reached") << "\n";
    std::cout << "  Iterations:            " << synthesis_result.iterations << "\n";
    std::cout << "  Computation time:      " << std::fixed << std::setprecision(4) << synthesis_result.duration_seconds << " s\n";
    std::cout << "  Winning set size:      " << synthesis_result.winning_set_size << " cells\n";
    std::cout << "  Controllable coverage: " << std::fixed << std::setprecision(2)
              << synthesis_result.controllable_percentage << " %\n\n";

    // -------------------------------------------------------------------------
    // 5. Closed-Loop Continuous Simulation
    // -------------------------------------------------------------------------
    std::cout << "[Step 5] Simulating Closed-Loop Trajectory under Disturbances...\n";

    // Select a controllable initial pose outside target
    symbolic::model::State initial_state{ 2.5, 1.8, -M_PI * 0.5 };
    std::size_t init_cell = partition.point_to_flat(initial_state.x, initial_state.y, initial_state.theta);

    if (init_cell >= synthesis_result.controller.size() ||
        !synthesis_result.controller[init_cell].has_value() ||
        target_states.find(init_cell) != target_states.end())
    {
        for (std::size_t q = 0; q < partition.total_cells(); ++q) {
            if (synthesis_result.controller[q].has_value() &&
                target_states.find(q) == target_states.end() &&
                obstacle_states.find(q) == obstacle_states.end())
            {
                auto box = partition.get_cell_box(q);
                auto c = box.center();
                initial_state = { c[0], c[1], c[2] };
                init_cell = q;
                break;
            }
        }
    }

    std::cout << "  Initial state x0:  (x=" << std::fixed << std::setprecision(3) << initial_state.x
              << " m, y=" << initial_state.y
              << " m, theta=" << (initial_state.theta * 180.0 / M_PI) << " deg)\n";
    std::cout << "  Initial cell ID:   " << init_cell << "\n";
    std::cout << "  Steps to target:   " << static_cast<int>(synthesis_result.value_function[init_cell]) << "\n";

    symbolic::simulation::ClosedLoopSimulator simulator;
    auto trace = simulator.run(initial_state, partition, model, synthesis_result, target_states, obstacle_states, 60, true);

    std::cout << "  Simulation outcome: ";
    switch (trace.outcome) {
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

    std::cout << "  Trajectory steps:   " << trace.total_steps << "\n\n";
    std::cout << "  Sample Trajectory Trace (first 10 steps):\n";
    std::cout << "  Step |      x (m) |      y (m) | theta (deg) | Cell ID |  v (m/s) | omega (rad/s)\n";
    std::cout << "  -----+------------+------------+-------------+---------+----------+--------------\n";

    std::size_t print_limit = std::min<std::size_t>(10, trace.steps.size());
    for (std::size_t i = 0; i < print_limit; ++i) {
        const auto& s = trace.steps[i];
        std::cout << "  " << std::setw(4) << i << " | "
                  << std::setw(10) << std::fixed << std::setprecision(3) << s.continuous_state.x << " | "
                  << std::setw(10) << s.continuous_state.y << " | "
                  << std::setw(11) << (s.continuous_state.theta * 180.0 / M_PI) << " | "
                  << std::setw(7) << s.cell_index << " | "
                  << std::setw(8) << s.applied_control.v << " | "
                  << std::setw(12) << s.applied_control.omega << "\n";
    }

    if (trace.steps.size() > 10) {
        std::cout << "  ... (" << (trace.steps.size() - 10) << " additional steps omitted) ...\n";
        const auto& final_step = trace.steps.back();
        std::cout << "  " << std::setw(4) << (trace.steps.size() - 1) << " | "
                  << std::setw(10) << std::fixed << std::setprecision(3) << final_step.continuous_state.x << " | "
                  << std::setw(10) << final_step.continuous_state.y << " | "
                  << std::setw(11) << (final_step.continuous_state.theta * 180.0 / M_PI) << " | "
                  << std::setw(7) << final_step.cell_index << " | "
                  << std::setw(8) << final_step.applied_control.v << " | "
                  << std::setw(12) << final_step.applied_control.omega << "\n";
    }

    std::cout << "\n========================================================\n";
    std::cout << "  Milestone Execution Completed Successfully.\n";
    std::cout << "========================================================\n";

    return 0;
}
