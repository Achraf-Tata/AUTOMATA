#include "simulator.hpp"
#include <random>
#include <cmath>

namespace symbolic::simulation {

// Executes closed-loop simulation under synthesized product automaton feedback policy
SimulationTrace ClosedLoopSimulator::run_product_closed_loop(
    const model::State& initial_state,
    const grid::GridPartition& partition,
    const model::UnicycleModel& model,
    const automaton::ProductAutomaton& product_automaton,
    const synthesis::SynthesisResult& synthesis_result,
    std::size_t maximum_steps,
    bool apply_bounded_disturbances)
{
    SimulationTrace simulation_trace;
    model::State current_continuous_state = initial_state;
    automaton::AutomatonState current_automaton_state = automaton::AutomatonState::Initial;

    std::mt19937 random_generator(1337);
    std::uniform_real_distribution<double> disturbance_distribution(
        -model.disturbance_bounds.max_disturbance_x,
        model.disturbance_bounds.max_disturbance_x
    );

    for (std::size_t step_index = 0; step_index < maximum_steps; ++step_index) {
        // Check continuous domain bounds
        if (!partition.state_domain.contains(
                current_continuous_state.x_position,
                current_continuous_state.y_position,
                current_continuous_state.heading_angle))
        {
            simulation_trace.outcome = SimulationOutcome::OutOfBounds;
            break;
        }

        std::size_t current_cell_index = partition.point_to_flat(
            current_continuous_state.x_position,
            current_continuous_state.y_position,
            current_continuous_state.heading_angle
        );

        automaton::ObservationSymbol current_observation = product_automaton.evaluate_point_observation(
            current_continuous_state.x_position,
            current_continuous_state.y_position
        );
        simulation_trace.observation_word.push_back(current_observation);

        // Update DFA state
        current_automaton_state = product_automaton.specification_dfa.transition(
            current_automaton_state, current_observation);

        if (current_automaton_state == automaton::AutomatonState::GoalReached) {
            simulation_trace.outcome = SimulationOutcome::GoalReached;
            simulation_trace.satisfies_specification_language = true;
            simulation_trace.total_steps = step_index;
            return simulation_trace;
        }

        if (current_automaton_state == automaton::AutomatonState::SinkTrap) {
            if (current_observation == automaton::ObservationSymbol::Obstacle) {
                simulation_trace.outcome = SimulationOutcome::ObstacleCollision;
            } else {
                simulation_trace.outcome = SimulationOutcome::SpecificationViolation;
            }
            simulation_trace.satisfies_specification_language = false;
            simulation_trace.total_steps = step_index;
            return simulation_trace;
        }

        std::size_t current_product_index = product_automaton.encode_product_index(
            current_cell_index, current_automaton_state);

        if (current_product_index >= synthesis_result.feedback_policy.size() ||
            !synthesis_result.feedback_policy[current_product_index].has_value())
        {
            simulation_trace.outcome = SimulationOutcome::UncontrollableState;
            break;
        }

        std::size_t optimal_control_index = synthesis_result.feedback_policy[current_product_index].value();
        model::Control optimal_control = model.get_control(optimal_control_index);

        TrajectoryStep recorded_step;
        recorded_step.continuous_state = current_continuous_state;
        recorded_step.spatial_cell_index = current_cell_index;
        recorded_step.observation_symbol = current_observation;
        recorded_step.automaton_state = current_automaton_state;
        recorded_step.applied_control = optimal_control;
        simulation_trace.recorded_steps.push_back(recorded_step);

        std::array<double, 3> disturbance_vector = {0.0, 0.0, 0.0};
        if (apply_bounded_disturbances) {
            disturbance_vector = {
                disturbance_distribution(random_generator),
                disturbance_distribution(random_generator),
                disturbance_distribution(random_generator)
            };
        }

        current_continuous_state = model.step_exact(
            current_continuous_state, optimal_control, disturbance_vector);
    }

    if (simulation_trace.outcome == SimulationOutcome::UncontrollableState &&
        simulation_trace.recorded_steps.size() >= maximum_steps)
    {
        simulation_trace.outcome = SimulationOutcome::StepLimitReached;
    }

    simulation_trace.total_steps = simulation_trace.recorded_steps.size();
    simulation_trace.satisfies_specification_language =
        product_automaton.specification_dfa.accepts_word(simulation_trace.observation_word);

    return simulation_trace;
}

// Executes closed-loop simulation under baseline spatial reachability feedback policy
SimulationTrace ClosedLoopSimulator::run_spatial(
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

    std::mt19937 random_generator(1337);
    std::uniform_real_distribution<double> disturbance_distribution(
        -model.disturbance_bounds.max_disturbance_x,
        model.disturbance_bounds.max_disturbance_x
    );

    for (std::size_t step_index = 0; step_index < maximum_steps; ++step_index) {
        if (!partition.state_domain.contains(
                current_continuous_state.x_position,
                current_continuous_state.y_position,
                current_continuous_state.heading_angle))
        {
            simulation_trace.outcome = SimulationOutcome::OutOfBounds;
            break;
        }

        std::size_t current_cell_index = partition.point_to_flat(
            current_continuous_state.x_position,
            current_continuous_state.y_position,
            current_continuous_state.heading_angle
        );

        if (target_state_indices.find(current_cell_index) != target_state_indices.end()) {
            simulation_trace.outcome = SimulationOutcome::GoalReached;
            simulation_trace.satisfies_specification_language = true;
            simulation_trace.total_steps = step_index;
            return simulation_trace;
        }

        if (obstacle_state_indices.find(current_cell_index) != obstacle_state_indices.end()) {
            simulation_trace.outcome = SimulationOutcome::ObstacleCollision;
            simulation_trace.satisfies_specification_language = false;
            simulation_trace.total_steps = step_index;
            return simulation_trace;
        }

        if (current_cell_index >= synthesis_result.feedback_policy.size() ||
            !synthesis_result.feedback_policy[current_cell_index].has_value())
        {
            simulation_trace.outcome = SimulationOutcome::UncontrollableState;
            break;
        }

        std::size_t optimal_control_index = synthesis_result.feedback_policy[current_cell_index].value();
        model::Control optimal_control = model.get_control(optimal_control_index);

        TrajectoryStep recorded_step;
        recorded_step.continuous_state = current_continuous_state;
        recorded_step.spatial_cell_index = current_cell_index;
        recorded_step.applied_control = optimal_control;
        simulation_trace.recorded_steps.push_back(recorded_step);

        std::array<double, 3> disturbance_vector = {0.0, 0.0, 0.0};
        if (apply_bounded_disturbances) {
            disturbance_vector = {
                disturbance_distribution(random_generator),
                disturbance_distribution(random_generator),
                disturbance_distribution(random_generator)
            };
        }

        current_continuous_state = model.step_exact(
            current_continuous_state, optimal_control, disturbance_vector);
    }

    if (simulation_trace.outcome == SimulationOutcome::UncontrollableState &&
        simulation_trace.recorded_steps.size() >= maximum_steps)
    {
        simulation_trace.outcome = SimulationOutcome::StepLimitReached;
    }

    simulation_trace.total_steps = simulation_trace.recorded_steps.size();
    return simulation_trace;
}

} // namespace symbolic::simulation
