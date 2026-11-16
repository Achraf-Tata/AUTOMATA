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

// Possible ways a test run can end
enum class SimulationOutcome {
    GoalReached,            // Reached accepting DFA state (q4)
    ObstacleCollision,      // Entered the unsafe obstacle region
    SpecificationViolation, // Violated the temporal mission (e.g. visited both R1 and R2, transitioning to q5)
    OutOfBounds,            // Wandered outside the continuous bounding box domain
    UncontrollableState,     // Reached a cell where we don't have a computed move
    StepLimitReached        // Exceeded maximum allowable steps before reaching a terminal condition
};
using enum SimulationOutcome;

// A single step in the robot's recorded journey
struct TrajectoryStep {
    State continuous_state;                                  // Continuous pose (x, y, theta)
    size_t spatial_cell_index = 0;                           // Discretized grid cell index
    ObservationSymbol observation_symbol = FreeSpace;        // Region label at this point
    AutomatonState automaton_state = AutomatonState::Initial;// Mission DFA state at this step
    Control applied_control;                                 // Selected control input (v, omega)
};

// Complete log of a test run: how it ended, the path taken, and the markers visited
struct SimulationTrace {
    SimulationOutcome outcome = UncontrollableState;
    bool satisfies_specification_language = false;          // True if the observation word is accepted by the DFA
    size_t total_steps = 0;                                  // Number of steps executed
    vector<TrajectoryStep> recorded_steps;                   // Full trajectory log
    vector<ObservationSymbol> observation_word;              // Sequence of symbols visited by the robot
};

// Simulator to test the robot driving along its chosen moves under real physical movement
class ClosedLoopSimulator {
public:
    ClosedLoopSimulator() = default;

    // Simulates the robot driving in real-time using our move lookup table.
    // At each tick:
    // 1. Checks if the robot is still inside the playground area.
    // 2. Figures out which grid box the robot is in and sees what zone it's touching.
    // 3. Updates the mission stage tracker based on what was just visited.
    // 4. Looks up what steering move the robot should take from this box and stage.
    // 5. Moves the robot forward with its physical wheels, adding slight real-world wobble.
    // 6. Keeps going until the robot wins, hits an obstacle, or runs out of steps.
    [[nodiscard]] SimulationTrace run_product_closed_loop(
        const State& x0,
        const GridPartition& grid,
        const UnicycleModel& model,
        const ProductAutomaton& automaton,
        const SynthesisResult& result,
        size_t max_steps = 100,
        bool apply_noise = true);

    // Simple direct test run: drive to the target boxes while steering clear of obstacle boxes
    [[nodiscard]] SimulationTrace run_spatial(
        const State& x0,
        const GridPartition& grid,
        const UnicycleModel& model,
        const SynthesisResult& result,
        const unordered_set<size_t>& targets,
        const unordered_set<size_t>& obstacles,
        size_t max_steps = 100,
        bool apply_noise = true);
};

} // namespace symbolic::simulation

namespace symbolic {
    using simulation::SimulationOutcome;
    using simulation::TrajectoryStep;
    using simulation::SimulationTrace;
    using simulation::ClosedLoopSimulator;
}
