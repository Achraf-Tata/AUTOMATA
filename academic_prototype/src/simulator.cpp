#include "simulator.hpp"
#include <random>

namespace symbolic::simulation {

SimulationTrace ClosedLoopSimulator::run(
    const model::State& initial_state,
    const grid::GridPartition& partition,
    const model::UnicycleModel& model,
    const synthesis::SynthesisResult& synthesis_result,
    const std::unordered_set<std::size_t>& target_state_indices,
    const std::unordered_set<std::size_t>& obstacle_state_indices,
    std::size_t maximum_steps,
    bool apply_bounded_disturbances)
{
    SimulationTrace simulation_trace;
    model::State current_continuous_state = initial_state;

    std::mt19937 random_generator(12345);
    std::uniform_real_distribution<double> disturbance_distribution(-0.045, 0.045);

    for (std::size_t step_index = 0; step_index < maximum_steps; ++step_index) {
        // Arena boundary verification
        if (!partition.state_domain.contains(current_continuous_state.x_position,
                                             current_continuous_state.y_position,
                                             current_continuous_state.heading_angle))
        {
            simulation_trace.outcome = SimulationOutcome::OutOfBounds;
            simulation_trace.total_steps = step_index;
            return simulation_trace;
        }

        std::size_t current_cell_index = partition.point_to_flat(
            current_continuous_state.x_position,
            current_continuous_state.y_position,
            current_continuous_state.heading_angle);

        // Target reachability verification
        if (target_state_indices.find(current_cell_index) != target_state_indices.end()) {
            simulation_trace.outcome = SimulationOutcome::GoalReached;
            simulation_trace.total_steps = step_index;
            simulation_trace.recorded_steps.push_back({ current_continuous_state, current_cell_index, {0.0, 0.0} });
            return simulation_trace;
        }

        // Obstacle avoidance verification
        if (obstacle_state_indices.find(current_cell_index) != obstacle_state_indices.end()) {
            simulation_trace.outcome = SimulationOutcome::ObstacleCollision;
            simulation_trace.total_steps = step_index;
            simulation_trace.recorded_steps.push_back({ current_continuous_state, current_cell_index, {0.0, 0.0} });
            return simulation_trace;
        }

        // Feedback controller policy query
        if (current_cell_index >= synthesis_result.feedback_policy.size() ||
            !synthesis_result.feedback_policy[current_cell_index].has_value())
        {
            simulation_trace.outcome = SimulationOutcome::UncontrollableState;
            simulation_trace.total_steps = step_index;
            simulation_trace.recorded_steps.push_back({ current_continuous_state, current_cell_index, {0.0, 0.0} });
            return simulation_trace;
        }

        std::size_t optimal_control_index = synthesis_result.feedback_policy[current_cell_index].value();
        const auto& selected_control = model.get_control(optimal_control_index);

        simulation_trace.recorded_steps.push_back({ current_continuous_state, current_cell_index, selected_control });

        // Apply continuous dynamics with disturbance
        std::array<double, 3> active_disturbance_vector = {0.0, 0.0, 0.0};
        if (apply_bounded_disturbances) {
            active_disturbance_vector = {
                disturbance_distribution(random_generator),
                disturbance_distribution(random_generator),
                disturbance_distribution(random_generator)
            };
        }

        current_continuous_state = model.step_exact(current_continuous_state, selected_control, active_disturbance_vector);
    }

    simulation_trace.outcome = SimulationOutcome::StepLimitReached;
    simulation_trace.total_steps = maximum_steps;
    return simulation_trace;
}

} // namespace symbolic::simulation
