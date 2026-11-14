#pragma once

#include "grid_partition.hpp"
#include "unicycle_model.hpp"
#include "backward_reachability.hpp"
#include "product_automaton.hpp"
#include "dfa.hpp"
#include <vector>
#include <unordered_set>

namespace symbolic::simulation {

using namespace std;
using namespace model;
using namespace grid;
using namespace automaton;
using namespace synthesis;
using enum ObservationSymbol;

// Possible terminal outcomes for a closed-loop simulation run
enum class SimulationOutcome {
    GoalReached,            // Reached accepting DFA state (q4)
    ObstacleCollision,      // Entered the unsafe obstacle region
    SpecificationViolation, // Violated the temporal mission (e.g. visited both R1 and R2, transitioning to q5)
    OutOfBounds,            // Wandered outside the continuous bounding box domain
    UncontrollableState,     // Reached a state where no valid control action exists in the feedback policy
    StepLimitReached        // Exceeded maximum allowable steps before reaching a terminal condition
};
using enum SimulationOutcome;

// Snapshot of a single discrete time step in the robot's trajectory
struct TrajectoryStep {
    State continuous_state;                                  // Continuous pose (x, y, theta)
    size_t spatial_cell_index = 0;                           // Discretized grid cell index
    ObservationSymbol observation_symbol = FreeSpace;        // Region label at this point
    AutomatonState automaton_state = AutomatonState::Initial;// Mission DFA state at this step
    Control applied_control;                                 // Selected control input (v, omega)
};

// Complete simulation record containing outcome, recorded path, and observation trace
struct SimulationTrace {
    SimulationOutcome outcome = UncontrollableState;
    bool satisfies_specification_language = false;          // True if the observation word is accepted by the DFA
    size_t total_steps = 0;                                  // Number of steps executed
    vector<TrajectoryStep> recorded_steps;                   // Full trajectory log
    vector<ObservationSymbol> observation_word;              // Sequence of symbols visited by the robot
};

// Closed-loop simulation engine for testing synthesized controllers on continuous unicycle dynamics
class ClosedLoopSimulator {
public:
    ClosedLoopSimulator() = default;

    // Runs a closed-loop simulation using the product automaton feedback policy (S x Q).
    // At each time step:
    // 1. Checks if continuous state is within domain bounds.
    // 2. Quantizes continuous state to cell index and observes region label.
    // 3. Updates DFA state: q' = delta(q, sigma).
    // 4. Looks up control u = policy(cell, q').
    // 5. Steps unicycle continuous kinematics with bounded random disturbance.
    // 6. Continues until goal is reached, collision occurs, or step limit is hit.
    [[nodiscard]] SimulationTrace run_product_closed_loop(
        const State& initial_state,
        const GridPartition& partition,
        const UnicycleModel& model,
        const ProductAutomaton& product_automaton,
        const SynthesisResult& synthesis_result,
        size_t maximum_steps = 100,
        bool apply_bounded_disturbances = true);

    // Runs a baseline spatial feedback simulation (pure reach-avoid without DFA tracking)
    [[nodiscard]] SimulationTrace run_spatial(
        const State& initial_state,
        const GridPartition& partition,
        const UnicycleModel& model,
        const SynthesisResult& synthesis_result,
        const unordered_set<size_t>& target_state_indices,
        const unordered_set<size_t>& obstacle_state_indices,
        size_t maximum_steps = 100,
        bool apply_bounded_disturbances = true);
};

} // namespace symbolic::simulation

namespace symbolic {
    using simulation::SimulationOutcome;
    using simulation::TrajectoryStep;
    using simulation::SimulationTrace;
    using simulation::ClosedLoopSimulator;
}
