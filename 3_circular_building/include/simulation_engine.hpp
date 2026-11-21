#pragma once

#include "app_state.hpp"

namespace circular::engine {

// Triggers Assume-Guarantee contract synthesis across all N rooms using the configured solver
void trigger_synthesis(AppState& state);

// Advances the continuous/discrete coupled multi-room network simulation by 1 step
void step_simulation(AppState& state);

// Resets room temperatures, simulation clock, and performance counters
void reset_simulation(AppState& state);

// Precomputes the Kamke-Muller extreme trajectory envelope (coldest vs warmest bounds)
void generate_extreme_envelope(AppState& state, int horizon_steps = 7200);

// Generates a bundle of 50 Monte Carlo simulation runs with perturbed weather and initial states
// to visually verify that all possible closed-loop trajectories remain safely bounded within [17.0, 22.0] C
void generate_trajectory_bundle(AppState& state, int num_runs = 50);

} // namespace circular::engine
