#pragma once
#include "app_state.hpp"

namespace robot_nav::engine {

// Step simulation once per frame
bool sim_step_once(AppState& app);

// Run full simulation from current start pose
void run_simulation(AppState& app);

// Run Monte Carlo analysis
void run_monte_carlo_analysis(AppState& app);

// Trigger OpenMP backward reachability solver
void trigger_synthesis(AppState& app);

} // namespace robot_nav::engine
