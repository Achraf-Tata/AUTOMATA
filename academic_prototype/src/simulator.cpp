#include "simulator.hpp"
#include <random>

namespace symbolic::simulation {

SimulationTrace ClosedLoopSimulator::run(
    const model::State& initial_state,
    const grid::GridPartition& partition,
    const model::UnicycleModel& model,
    const synthesis::SynthesisResult& synthesis,
    const std::unordered_set<std::size_t>& target_states,
    const std::unordered_set<std::size_t>& obstacle_states,
    std::size_t max_steps,
    bool apply_disturbances)
{
    SimulationTrace trace;
    model::State current = initial_state;

    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> dist_w(-0.045, 0.045);

    for (std::size_t step = 0; step < max_steps; ++step) {
        // Boundary check
        if (!partition.domain.contains(current.x, current.y, current.theta)) {
            trace.outcome = SimulationOutcome::OutOfBounds;
            trace.total_steps = step;
            return trace;
        }

        std::size_t q = partition.point_to_flat(current.x, current.y, current.theta);

        // Check target condition
        if (target_states.find(q) != target_states.end()) {
            trace.outcome = SimulationOutcome::GoalReached;
            trace.total_steps = step;
            trace.steps.push_back({ current, q, {0.0, 0.0} });
            return trace;
        }

        // Check obstacle collision
        if (obstacle_states.find(q) != obstacle_states.end()) {
            trace.outcome = SimulationOutcome::ObstacleCollision;
            trace.total_steps = step;
            trace.steps.push_back({ current, q, {0.0, 0.0} });
            return trace;
        }

        // Check controller policy
        if (q >= synthesis.controller.size() || !synthesis.controller[q].has_value()) {
            trace.outcome = SimulationOutcome::UncontrollableState;
            trace.total_steps = step;
            trace.steps.push_back({ current, q, {0.0, 0.0} });
            return trace;
        }

        std::size_t u_idx = synthesis.controller[q].value();
        const auto& u = model.get_control(u_idx);

        trace.steps.push_back({ current, q, u });

        // Continuous unicycle state transition under bounded disturbance
        std::array<double, 3> w = {0.0, 0.0, 0.0};
        if (apply_disturbances) {
            w = { dist_w(rng), dist_w(rng), dist_w(rng) };
        }

        current = model.step_exact(current, u, w);
    }

    trace.outcome = SimulationOutcome::StepLimitReached;
    trace.total_steps = max_steps;
    return trace;
}

} // namespace symbolic::simulation
