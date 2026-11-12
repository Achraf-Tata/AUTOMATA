#pragma once

#include "grid_partition.hpp"
#include "unicycle_model.hpp"
#include "backward_reachability.hpp"
#include "product_automaton.hpp"
#include "dfa.hpp"
#include <vector>
#include <unordered_set>

namespace symbolic::simulation {

enum class SimulationOutcome {
    GoalReached,
    ObstacleCollision,
    SpecificationViolation,
    OutOfBounds,
    UncontrollableState,
    StepLimitReached
};

struct TrajectoryStep {
    model::State continuous_state;
    std::size_t spatial_cell_index = 0;
    automaton::ObservationSymbol observation_symbol = automaton::ObservationSymbol::FreeSpace;
    automaton::AutomatonState automaton_state = automaton::AutomatonState::Initial;
    model::Control applied_control;
};

struct SimulationTrace {
    SimulationOutcome outcome = SimulationOutcome::UncontrollableState;
    bool satisfies_specification_language = false;
    std::size_t total_steps = 0;
    std::vector<TrajectoryStep> recorded_steps;
    std::vector<automaton::ObservationSymbol> observation_word;
};

class ClosedLoopSimulator {
public:
    // Default constructor
    ClosedLoopSimulator() = default;

    // Simulates unicycle under feedback policy synthesized over product automaton (S x A)
    [[nodiscard]] SimulationTrace run_product_closed_loop(
        const model::State& initial_state,
        const grid::GridPartition& partition,
        const model::UnicycleModel& model,
        const automaton::ProductAutomaton& product_automaton,
        const synthesis::SynthesisResult& synthesis_result,
        std::size_t maximum_steps = 100,
        bool apply_bounded_disturbances = true);

    // Simulates unicycle under baseline spatial feedback policy
    [[nodiscard]] SimulationTrace run_spatial(
        const model::State& initial_state,
        const grid::GridPartition& partition,
        const model::UnicycleModel& model,
        const synthesis::SynthesisResult& synthesis_result,
        const std::unordered_set<std::size_t>& target_state_indices,
        const std::unordered_set<std::size_t>& obstacle_state_indices,
        std::size_t maximum_steps = 100,
        bool apply_bounded_disturbances = true);
};

} // namespace symbolic::simulation
