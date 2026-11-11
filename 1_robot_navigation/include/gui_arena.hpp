// =============================================================================
// gui_arena.hpp — Member 5
//
// Dear ImGui + OpenGL interactive arena renderer and simulation runner.
// =============================================================================
#pragma once
#include "grid_quantizer.hpp"
#include "unicycle_model.hpp"
#include "reachability_cpu.hpp"
#include <vector>
#include <string>
#include <array>

namespace gui {

struct SimulationResult {
    std::vector<unicycle::State> trajectory;
    bool reached_goal = false;
    bool hit_obstacle = false;
    int  steps_taken  = 0;
};

struct MonteCarloStats {
    int   total_runs   = 0;
    int   successful   = 0;
    int   failed       = 0;
    int   timeout      = 0;
    float success_rate = 0.0f;
};

struct AppState {
    // --- Region layout (editable by mouse) ---
    grid::RegionConfig regions;

    // --- Robot initial pose ---
    unicycle::State robot_start = {1.0, 2.0, 0.0};

    // --- Value function and controller ---
    std::vector<float>   V;
    std::vector<uint8_t> ctrl;
    bool controller_ready = false;

    // --- Abstraction lookup tables ---
    grid::AbstractionTables abs_tables;
    bool abs_tables_ready = false;

    // --- Solver state ---
    int   solve_iteration = 0;
    int   solve_changes   = 0;
    bool  solving         = false;
    float solve_progress  = 0.0f;
    float solve_duration  = 0.0f;

    // --- Simulation state ---
    bool  sim_running    = false;
    bool  sim_paused     = false;
    int   sim_step       = 0;
    int   sim_max_steps  = 200;
    bool  sim_with_noise = true;
    int   sim_dfa_state  = 1;
    unicycle::State sim_robot_pos = {1.0, 2.0, 0.0};
    unicycle::Control sim_last_u  = {0.0, 0.0};
    std::vector<unicycle::State> sim_trajectory;

    // --- Monte Carlo state ---
    bool           mc_running = false;
    int            mc_runs    = 50;
    MonteCarloStats mc_stats;
    std::vector<std::vector<unicycle::State>> mc_trajectories;

    // --- File paths ---
    char controller_path_buf[256] = "controller.bin";

    // --- UI interaction state ---
    bool  show_grid       = true;
    bool  show_all_mc     = true;
    int   dragging_region = 0; // 1=R1, 2=R2, 3=R3, 4=R4
    bool  dragging_robot  = false;
    int   resize_region   = 0;
    int   resize_corner   = 0;
};

bool init_graphics(int window_w, int window_h, const char* title);
void shutdown_graphics();

void render_arena(AppState& app, float canvas_size);
void render_control_panel(AppState& app);

bool sim_step_once(AppState& app);
void trigger_synthesis(AppState& app);
void run_monte_carlo(AppState& app);

} // namespace gui
