#include "simulator.hpp"
#include <random>
#include <cmath>

namespace symbolic::simulation {

using namespace std;
using namespace model;
using namespace grid;
using namespace automaton;
using namespace synthesis;
using enum SimulationOutcome;
using enum ObservationSymbol;
using AutomatonState::Initial;
using AutomatonState::SinkTrap;

// Simulates the robot executing the synthesized feedback policy on the product automaton.
// At each discrete time step (dt = 1.0 s):
// 1. Boundary check: ensure robot (x, y, theta) is within continuous state domain.
// 2. Quantization & observation: map continuous coordinates to cell index and lookup region label.
// 3. DFA state update: transition current DFA state to next state delta(q, sigma).
//    - If goal reached (q4), simulation ends with success!
//    - If sink trap reached (q5), simulation ends with obstacle collision or exclusivity violation.
// 4. Policy lookup: find the optimal control input u from feedback_policy[product_index(cell, q')].
// 5. Kinematic integration: calculate exact continuous motion x(k+1) = f(x(k), u(k)) + w(k)
//    with bounded uniform disturbance w in [-max_disturbance, max_disturbance].
// 6. Record step into trajectory log.
SimulationTrace ClosedLoopSimulator::run_product_closed_loop(
    const State& initial_state,
    const GridPartition& partition,
    const UnicycleModel& model,
    const ProductAutomaton& product_automaton,
    const SynthesisResult& synthesis_result,
    size_t maximum_steps,
    bool apply_bounded_disturbances)
{
    SimulationTrace simulation_trace;
    State current_continuous_state = initial_state;
    AutomatonState current_automaton_state = Initial;

    // Use a fixed seed for reproducible simulation runs
    mt19937 random_generator(1337);
    uniform_real_distribution<double> disturbance_distribution(
        -model.disturbance_bounds.max_disturbance_x,
        model.disturbance_bounds.max_disturbance_x
    );

    for (size_t step_index = 0; step_index < maximum_steps; ++step_index) {
        // Check continuous domain bounds
        if (!partition.state_domain.contains(
                current_continuous_state.x_position,
                current_continuous_state.y_position,
                current_continuous_state.heading_angle))
        {
            simulation_trace.outcome = OutOfBounds;
            break;
        }

        size_t current_cell_index = partition.point_to_flat(
            current_continuous_state.x_position,
            current_continuous_state.y_position,
            current_continuous_state.heading_angle
        );

        ObservationSymbol current_observation = product_automaton.evaluate_point_observation(
            current_continuous_state.x_position,
            current_continuous_state.y_position
        );
        simulation_trace.observation_word.push_back(current_observation);

        // Update DFA state: q' = delta(q, sigma)
        current_automaton_state = product_automaton.specification_dfa.transition(
            current_automaton_state, current_observation);

        // Accepting goal reached
        if (current_automaton_state == AutomatonState::GoalReached) {
            simulation_trace.outcome = GoalReached;
            simulation_trace.satisfies_specification_language = true;
            simulation_trace.total_steps = step_index;
            return simulation_trace;
        }

        // Sink trap reached: check whether it was an obstacle or exclusivity violation
        if (current_automaton_state == SinkTrap) {
            if (current_observation == Obstacle) {
                simulation_trace.outcome = ObstacleCollision;
            } else {
                simulation_trace.outcome = SpecificationViolation;
            }
            simulation_trace.satisfies_specification_language = false;
            simulation_trace.total_steps = step_index;
            return simulation_trace;
        }

        // Look up the feedback control in the synthesized product policy table
        size_t current_product_index = product_automaton.encode_product_index(
            current_cell_index, current_automaton_state);

        if (current_product_index >= synthesis_result.feedback_policy.size() ||
            !synthesis_result.feedback_policy[current_product_index].has_value())
        {
            simulation_trace.outcome = UncontrollableState;
            break;
        }

        size_t optimal_control_index = synthesis_result.feedback_policy[current_product_index].value();
        Control optimal_control = model.get_control(optimal_control_index);

        TrajectoryStep recorded_step;
        recorded_step.continuous_state = current_continuous_state;
        recorded_step.spatial_cell_index = current_cell_index;
        recorded_step.observation_symbol = current_observation;
        recorded_step.automaton_state = current_automaton_state;
        recorded_step.applied_control = optimal_control;
        simulation_trace.recorded_steps.push_back(recorded_step);

        // Apply bounded disturbance vector if requested
        array<double, 3> disturbance_vector = {0.0, 0.0, 0.0};
        if (apply_bounded_disturbances) {
            disturbance_vector = {
                disturbance_distribution(random_generator),
                disturbance_distribution(random_generator),
                disturbance_distribution(random_generator)
            };
        }

        // Forward integration of continuous unicycle dynamics: x(k+1) = f(x(k), u(k)) + w(k)
        current_continuous_state = model.step_exact(
            current_continuous_state, optimal_control, disturbance_vector);
    }

    if (simulation_trace.outcome == UncontrollableState &&
        simulation_trace.recorded_steps.size() >= maximum_steps)
    {
        simulation_trace.outcome = StepLimitReached;
    }

    simulation_trace.total_steps = simulation_trace.recorded_steps.size();
    simulation_trace.satisfies_specification_language =
        product_automaton.specification_dfa.accepts_word(simulation_trace.observation_word);

    return simulation_trace;
}

// Simulates unicycle under a baseline spatial reachability feedback policy (without DFA)
SimulationTrace ClosedLoopSimulator::run_spatial(
    const State& initial_state,
    const GridPartition& partition,
    const UnicycleModel& model,
    const SynthesisResult& synthesis_result,
    const unordered_set<size_t>& target_state_indices,
    const unordered_set<size_t>& obstacle_state_indices,
    size_t maximum_steps,
    bool apply_bounded_disturbances)
{
    SimulationTrace simulation_trace;
    State current_continuous_state = initial_state;

    mt19937 random_generator(1337);
    uniform_real_distribution<double> disturbance_distribution(
        -model.disturbance_bounds.max_disturbance_x,
        model.disturbance_bounds.max_disturbance_x
    );

    for (size_t step_index = 0; step_index < maximum_steps; ++step_index) {
        if (!partition.state_domain.contains(
                current_continuous_state.x_position,
                current_continuous_state.y_position,
                current_continuous_state.heading_angle))
        {
            simulation_trace.outcome = OutOfBounds;
            break;
        }

        size_t current_cell_index = partition.point_to_flat(
            current_continuous_state.x_position,
            current_continuous_state.y_position,
            current_continuous_state.heading_angle
        );

        if (target_state_indices.find(current_cell_index) != target_state_indices.end()) {
            simulation_trace.outcome = GoalReached;
            simulation_trace.satisfies_specification_language = true;
            simulation_trace.total_steps = step_index;
            return simulation_trace;
        }

        if (obstacle_state_indices.find(current_cell_index) != obstacle_state_indices.end()) {
            simulation_trace.outcome = ObstacleCollision;
            simulation_trace.satisfies_specification_language = false;
            simulation_trace.total_steps = step_index;
            return simulation_trace;
        }

        if (current_cell_index >= synthesis_result.feedback_policy.size() ||
            !synthesis_result.feedback_policy[current_cell_index].has_value())
        {
            simulation_trace.outcome = UncontrollableState;
            break;
        }

        size_t optimal_control_index = synthesis_result.feedback_policy[current_cell_index].value();
        Control optimal_control = model.get_control(optimal_control_index);

        TrajectoryStep recorded_step;
        recorded_step.continuous_state = current_continuous_state;
        recorded_step.spatial_cell_index = current_cell_index;
        recorded_step.applied_control = optimal_control;
        simulation_trace.recorded_steps.push_back(recorded_step);

        array<double, 3> disturbance_vector = {0.0, 0.0, 0.0};
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

    if (simulation_trace.outcome == UncontrollableState &&
        simulation_trace.recorded_steps.size() >= maximum_steps)
    {
        simulation_trace.outcome = StepLimitReached;
    }

    simulation_trace.total_steps = simulation_trace.recorded_steps.size();
    return simulation_trace;
}

} // namespace symbolic::simulation
