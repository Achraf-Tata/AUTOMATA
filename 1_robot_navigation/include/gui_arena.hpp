// =============================================================================
// gui_arena.hpp — Member 5
//
// Dear ImGui + OpenGL arena renderer and simulation runner.
//
// The GUI has two panels:
//   Left panel  — the arena canvas: shows the 10×10 grid world, the four
//                 coloured regions R1-R4, the robot (arrow), and trajectory.
//                 The user can drag regions to move them, drag their corners
//                 to resize them, and drag the robot to reposition it.
//   Right panel — controls: run/pause simulation, run synthesis, progress bar,
//                 Monte Carlo stats, and numeric readouts.
// =============================================================================
#pragma once
#include "grid_quantizer.hpp"
#include "unicycle_model.hpp"
#include "reachability_cpu.hpp"
#include <vector>
#include <string>
#include <array>
#include <functional>

// Forward-declare ImVec2 to avoid including imgui.h in this header.
struct ImVec2;

namespace gui {

// ---------------------------------------------------------------------------
// Simulation result for one run
// ---------------------------------------------------------------------------

/// Records the trajectory and outcome of one closed-loop simulation run.
struct SimulationResult {
    std::vector<unicycle::State> trajectory; ///< states visited, length ≤ max_steps
    bool reached_goal;   ///< robot reached DFA state q=4
    bool hit_obstacle;   ///< robot entered R4 (DFA state q=5)
    int  steps_taken;    ///< number of steps before terminal or max_steps
};

// ---------------------------------------------------------------------------
// Monte Carlo summary
// ---------------------------------------------------------------------------

struct MonteCarloStats {
    int   total_runs;
    int   successful;      ///< reached goal
    int   failed;          ///< entered obstacle
    int   timeout;         ///< neither within max_steps
    float success_rate;    ///< successful / total_runs
};

// ---------------------------------------------------------------------------
// Application state (all GUI state lives here)
// ---------------------------------------------------------------------------

struct AppState {
    // --- region config (editable by mouse) ---
    grid::RegionConfig regions;

    // --- robot initial pose (editable by mouse) ---
    unicycle::State robot_start = {0.5, 0.5, 0.0};

    // --- value function and controller (computed by solver) ---
    std::vector<float>   V;
    std::vector<uint8_t> ctrl;
    bool controller_ready = false;

    // --- solver progress ---
    int   solve_iteration = 0;
    float solve_delta     = 1e7f;
    bool  solving         = false;
    bool  solve_done      = false;
    float solve_progress  = 0.0f; ///< 0..1 for the progress bar

    // --- simulation state ---
    bool  sim_running    = false;
    bool  sim_paused     = false;
    int   sim_step       = 0;
    int   sim_max_steps  = 200;
    bool  sim_with_noise = true;
    int   sim_dfa_state  = 1;
    unicycle::State sim_robot_pos = {0.5, 0.5, 0.0};
    std::vector<unicycle::State> sim_trajectory;

    // --- Monte Carlo ---
    bool           mc_running = false;
    int            mc_runs    = 50;
    MonteCarloStats mc_stats  = {};
    std::vector<std::vector<unicycle::State>> mc_trajectories; ///< up to 50 trajectories

    // --- footprint table (precomputed once) ---
    grid::FootprintTable footprint;
    bool footprint_ready = false;

    // --- file paths ---
    std::string controller_path = "controller.bin";
    char        controller_path_buf[256] = "controller.bin";

    // --- UI state ---
    bool  show_grid        = true;
    bool  show_all_mc      = true;  ///< draw all 50 Monte Carlo trajectories
    int   dragging_region  = 0;     ///< 1-4 = dragging region, 0 = none
    bool  dragging_robot   = false;
    int   resize_corner    = 0;     ///< 1-4 corner index, 0 = not resizing
    float arena_scale      = 60.0f; ///< pixels per world unit (zoom)
    ImVec2* arena_origin   = nullptr; ///< top-left of arena in screen coords
};

// ---------------------------------------------------------------------------
// Core functions (implemented in main_robot.cpp)
// ---------------------------------------------------------------------------

/// Runs one closed-loop simulation step: looks up the controller for
/// (sim_dfa_state, current_cell), applies the control, optionally adds noise,
/// advances the DFA, records the new state.
/// Returns true while the simulation should continue (not terminal, not max_steps).
bool sim_step_once(AppState& app);

/// Runs a full closed-loop simulation from x0 for up to max_steps steps.
/// Used by run_monte_carlo to generate each trajectory.
SimulationResult run_closed_loop(
    const unicycle::State&       x0,
    int                          dfa_start,
    int                          max_steps,
    bool                         with_noise,
    const std::vector<float>&    V,
    const std::vector<uint8_t>&  ctrl,
    const grid::RegionConfig&    regions);

/// Runs mc_runs independent simulations with random starting noise,
/// fills app.mc_stats and app.mc_trajectories.
void run_monte_carlo(AppState& app);

/// Renders the arena canvas (R1-R4 boxes, robot arrow, trajectory polylines)
/// inside a Dear ImGui window using ImDrawList primitives.
/// Also handles mouse interactions (drag/resize regions, drag robot).
void render_arena(AppState& app, float canvas_size);

/// Renders the right-side control panel: buttons, sliders, stats readouts.
void render_control_panel(AppState& app);

/// One-time setup: initialises GLFW + OpenGL + ImGui.
/// Returns false on failure.
bool init_graphics(int window_w, int window_h, const char* title);

/// Cleans up GLFW + ImGui resources.
void shutdown_graphics();

} // namespace gui
