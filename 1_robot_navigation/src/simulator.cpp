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

// Simulates the robot driving in real-time using our computed move lookup table.
// Every 1 second:
// 1. Check bounds: make sure the robot is still inside the playground area.
// 2. Locate: find which grid box the robot is in and check what zone it just hit.
// 3. Update stage: update our mission progress.
//    - If we hit the goal (q4), celebrate victory!
//    - If we hit the trap state (q5), we crashed or broke a mission rule.
// 4. Choose move: look up the best steering action from our table.
// 5. Move robot: update position using the wheel equations, plus a little random wobble.
// 6. Save step: record the move and position into our trip log.
SimulationTrace ClosedLoopSimulator::run_product_closed_loop(
    const State& x0,
    const GridPartition& grid,
    const UnicycleModel& model,
    const ProductAutomaton& automaton,
    const SynthesisResult& result,
    size_t max_steps,
    bool apply_noise)
{
    SimulationTrace trace;
    State pose = x0;
    AutomatonState q = Initial;

    // Use a fixed seed for reproducible simulation runs
    mt19937 rng(1337);
    uniform_real_distribution<double> noise_dist(
        -model.disturbance_bounds.max_disturbance_x,
        model.disturbance_bounds.max_disturbance_x
    );

    for (size_t step = 0; step < max_steps; ++step) {
        // Check continuous domain bounds
        if (!grid.state_domain.contains(pose.x_position, pose.y_position, pose.heading_angle)) {
            trace.outcome = OutOfBounds;
            break;
        }

        size_t s = grid.point_to_flat(pose.x_position, pose.y_position, pose.heading_angle);
        ObservationSymbol obs = automaton.evaluate_point_observation(pose.x_position, pose.y_position);
        trace.observation_word.push_back(obs);

        // Update DFA state: q' = delta(q, sigma)
        q = automaton.specification_dfa.transition(q, obs);

        // Accepting goal reached
        if (q == AutomatonState::GoalReached) {
            trace.outcome = GoalReached;
            trace.satisfies_specification_language = true;
            trace.total_steps = step;
            return trace;
        }

        // Sink trap reached: check whether it was an obstacle or exclusivity violation
        if (q == SinkTrap) {
            trace.outcome = (obs == Obstacle) ? ObstacleCollision : SpecificationViolation;
            trace.satisfies_specification_language = false;
            trace.total_steps = step;
            return trace;
        }

        // Look up which steering action to take from our table for this grid box and mission stage
        size_t p = automaton.encode_product_index(s, q);

        if (p >= result.feedback_policy.size() || !result.feedback_policy[p].has_value()) {
            trace.outcome = UncontrollableState;
            break;
        }

        size_t u_idx = result.feedback_policy[p].value();
        Control u = model.get_control(u_idx);

        TrajectoryStep rec;
        rec.continuous_state = pose;
        rec.spatial_cell_index = s;
        rec.observation_symbol = obs;
        rec.automaton_state = q;
        rec.applied_control = u;
        trace.recorded_steps.push_back(rec);

        // Add slight random wobble to simulate real-world noise
        array<double, 3> w = {0.0, 0.0, 0.0};
        if (apply_noise) {
            w = { noise_dist(rng), noise_dist(rng), noise_dist(rng) };
        }

        // Move the robot forward using the unicycle motion formula with noise
        pose = model.step_exact(pose, u, w);
    }

    if (trace.outcome == UncontrollableState && trace.recorded_steps.size() >= max_steps) {
        trace.outcome = StepLimitReached;
    }

    trace.total_steps = trace.recorded_steps.size();
    trace.satisfies_specification_language =
        automaton.specification_dfa.accepts_word(trace.observation_word);

    return trace;
}

// Drives the robot straight toward the goal without mission stages, just dodging obstacles
SimulationTrace ClosedLoopSimulator::run_spatial(
    const State& x0,
    const GridPartition& grid,
    const UnicycleModel& model,
    const SynthesisResult& result,
    const unordered_set<size_t>& targets,
    const unordered_set<size_t>& obstacles,
    size_t max_steps,
    bool apply_noise)
{
    SimulationTrace trace;
    State pose = x0;

    mt19937 rng(1337);
    uniform_real_distribution<double> noise_dist(
        -model.disturbance_bounds.max_disturbance_x,
        model.disturbance_bounds.max_disturbance_x
    );

    for (size_t step = 0; step < max_steps; ++step) {
        if (!grid.state_domain.contains(pose.x_position, pose.y_position, pose.heading_angle)) {
            trace.outcome = OutOfBounds;
            break;
        }

        size_t s = grid.point_to_flat(pose.x_position, pose.y_position, pose.heading_angle);

        if (targets.find(s) != targets.end()) {
            trace.outcome = GoalReached;
            trace.satisfies_specification_language = true;
            trace.total_steps = step;
            return trace;
        }

        if (obstacles.find(s) != obstacles.end()) {
            trace.outcome = ObstacleCollision;
            trace.satisfies_specification_language = false;
            trace.total_steps = step;
            return trace;
        }

        if (s >= result.feedback_policy.size() || !result.feedback_policy[s].has_value()) {
            trace.outcome = UncontrollableState;
            break;
        }

        size_t u_idx = result.feedback_policy[s].value();
        Control u = model.get_control(u_idx);

        TrajectoryStep rec;
        rec.continuous_state = pose;
        rec.spatial_cell_index = s;
        rec.applied_control = u;
        trace.recorded_steps.push_back(rec);

        array<double, 3> w = {0.0, 0.0, 0.0};
        if (apply_noise) {
            w = { noise_dist(rng), noise_dist(rng), noise_dist(rng) };
        }

        pose = model.step_exact(pose, u, w);
    }

    if (trace.outcome == UncontrollableState && trace.recorded_steps.size() >= max_steps) {
        trace.outcome = StepLimitReached;
    }

    trace.total_steps = trace.recorded_steps.size();
    return trace;
}

} // namespace symbolic::simulation
