#pragma once
#include "app_state.hpp"

namespace robot_nav::engine {

// Advances the simulation by exactly one discrete time step (dt = 1.0 s).
// Checks arena boundaries, detects obstacle collisions (R4), advances the DFA state,
// queries the synthesized feedback controller table, and applies unicycle kinematics with noise.
// Returns true while the robot is actively moving, or false once a terminal state is reached
// (goal reached, collision, out of bounds, exclusivity violation, or timeout).
bool sim_step_once(AppState& app);

// Executes the entire simulation loop from the current starting pose until termination.
// Useful for instant batch evaluation or headless trajectory generation.
void run_simulation(AppState& app);

// Simulates a bundle of runs with random disturbances from the starting pose
// to discover and visualize all possible routes the robot can take to the goal.
void generate_trajectory_bundle(AppState& app, int num_runs = 50);

// Spawns a background worker thread to run the backward reachability solver.
// This prevents freezing the ImGui render loop during synthesis,
// updating iteration count and progress bar in real time until the controller is saved.
void trigger_synthesis(AppState& app);

// Publish completed results on the calling (GUI) thread only.
bool poll_synthesis(AppState& app, bool wait = false);
void cancel_synthesis(AppState& app);

} // namespace robot_nav::engine
