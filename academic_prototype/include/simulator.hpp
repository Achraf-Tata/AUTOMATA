#pragma once

#include "grid_partition.hpp"
#include "unicycle_model.hpp"
#include "backward_reachability.hpp"
#include <vector>
#include <unordered_set>

namespace symbolic::simulation {

enum class SimulationOutcome {
    GoalReached,
    ObstacleCollision,
    OutOfBounds,
    UncontrollableState,
    StepLimitReached
};

struct TrajectoryStep {
    model::State continuous_state;
    std::size_t cell_index = 0;
    model::Control applied_control;
};

struct SimulationTrace {
    SimulationOutcome outcome = SimulationOutcome::UncontrollableState;
    std::size_t total_steps = 0;
    std::vector<TrajectoryStep> steps;
};

class ClosedLoopSimulator {
public:
    ClosedLoopSimulator() = default;

    [[nodiscard]] SimulationTrace run(
        const model::State& initial_state,
        const grid::GridPartition& partition,
        const model::UnicycleModel& model,
        const synthesis::SynthesisResult& synthesis,
        const std::unordered_set<std::size_t>& target_states,
        const std::unordered_set<std::size_t>& obstacle_states,
        std::size_t max_steps = 100,
        bool apply_disturbances = true);
};

} // namespace symbolic::simulation
