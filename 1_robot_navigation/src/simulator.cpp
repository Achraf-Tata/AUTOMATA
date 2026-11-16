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
        -model.noise_bounds.max_x,
        model.noise_bounds.max_x
    );

    for (size_t step = 0; step < max_steps; ++step) {
        // Check continuous domain bounds
        if (!grid.domain.contains(pose.x, pose.y, pose.theta)) {
            trace.outcome = OutOfBounds;
            break;
        }

        size_t s = grid.point_to_flat(pose.x, pose.y, pose.theta);
        ObservationSymbol obs = automaton.evaluate_point_observation(pose.x, pose.y);
        trace.word.push_back(obs);

        // Update DFA state: q' = delta(q, sigma)
        q = automaton.dfa.transition(q, obs);

        // Accepting goal reached
        if (q == AutomatonState::GoalReached) {
            trace.outcome = GoalReached;
            trace.accepted = true;
            trace.steps = step;
            return trace;
        }

        // Sink trap reached: check whether it was an obstacle or exclusivity violation
        if (q == SinkTrap) {
            trace.outcome = (obs == Obstacle) ? ObstacleCollision : SpecificationViolation;
            trace.accepted = false;
            trace.steps = step;
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
        rec.pose = pose;
        rec.s    = s;
        rec.obs  = obs;
        rec.q    = q;
        rec.u    = u;
        trace.history.push_back(rec);

        // Add slight random wobble to simulate real-world noise
        array<double, 3> w = {0.0, 0.0, 0.0};
        if (apply_noise) {
            w = { noise_dist(rng), noise_dist(rng), noise_dist(rng) };
        }

        // Move the robot forward using the unicycle motion formula with noise
        pose = model.step_exact(pose, u, w);
    }

    if (trace.outcome == UncontrollableState && trace.history.size() >= max_steps) {
        trace.outcome = StepLimitReached;
    }

    trace.steps = trace.history.size();
    trace.accepted = automaton.dfa.accepts_word(trace.word);

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
        -model.noise_bounds.max_x,
        model.noise_bounds.max_x
    );

    for (size_t step = 0; step < max_steps; ++step) {
        if (!grid.domain.contains(pose.x, pose.y, pose.theta)) {
            trace.outcome = OutOfBounds;
            break;
        }

        size_t s = grid.point_to_flat(pose.x, pose.y, pose.theta);

        if (targets.find(s) != targets.end()) {
            trace.outcome = GoalReached;
            trace.accepted = true;
            trace.steps = step;
            return trace;
        }

        if (obstacles.find(s) != obstacles.end()) {
            trace.outcome = ObstacleCollision;
            trace.accepted = false;
            trace.steps = step;
            return trace;
        }

        if (s >= result.feedback_policy.size() || !result.feedback_policy[s].has_value()) {
            trace.outcome = UncontrollableState;
            break;
        }

        size_t u_idx = result.feedback_policy[s].value();
        Control u = model.get_control(u_idx);

        TrajectoryStep rec;
        rec.pose = pose;
        rec.s    = s;
        rec.u    = u;
        trace.history.push_back(rec);

        array<double, 3> w = {0.0, 0.0, 0.0};
        if (apply_noise) {
            w = { noise_dist(rng), noise_dist(rng), noise_dist(rng) };
        }

        pose = model.step_exact(pose, u, w);
    }

    if (trace.outcome == UncontrollableState && trace.history.size() >= max_steps) {
        trace.outcome = StepLimitReached;
    }

    trace.steps = trace.history.size();
    return trace;
}

} // namespace symbolic::simulation
