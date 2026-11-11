#pragma once
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include "unicycle_model.hpp"
#include "grid_quantizer.hpp"
#include "product_automaton.hpp"

#include <vector>
#include <string>
#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace robot_nav {

enum class RunStatus {
    Idle,
    Running,
    GoalReached,
    ObstacleCollision,
    OutOfBounds,
    ExclusivityViolation,
    Timeout,
    Uncontrollable
};

struct MonteCarloStats {
    int   total_runs   = 0;
    int   successful   = 0;
    int   failed       = 0;
    int   timeout      = 0;
    float success_rate = 0.0f;
};

struct AppState {
    // --- Arena configuration ---
    grid::RegionConfig regions;
    grid::RegionConfig initial_regions;
    bool arena_modified = false;

    // --- Robot initial configuration ---
    unicycle::State robot_start = {1.23, 9.29, -29.7 * M_PI / 180.0};

    // --- Precomputed abstraction tables ---
    grid::AbstractionTables abs_tables;
    bool abs_tables_ready = false;

    // --- Controller synthesis state ---
    std::vector<float>   V;
    std::vector<uint8_t> ctrl;
    bool  controller_ready          = false;
    bool  solving                   = false;
    int   solve_iterations          = 25;
    float solve_duration            = 1.206f;
    float solve_progress            = 0.0f;
    int   solve_changes             = 0;
    bool  auto_simulate_after_solve = false;
    char  controller_filename[128]  = "controller.bin";

    // --- Simulation execution state ---
    bool              sim_running    = false;
    bool              sim_paused     = false;
    int               sim_step       = 0;
    int               sim_max_steps  = 200;
    bool              sim_with_noise = true;
    int               sim_dfa_state  = 1;
    unicycle::State   sim_robot_pos  = {7.04, 3.10, -58.4 * M_PI / 180.0};
    unicycle::Control sim_last_u     = {0.0, 0.0};
    std::vector<unicycle::State> active_trajectory;

    // --- Run result display state ---
    bool            has_run_result = true;
    RunStatus       last_status    = RunStatus::GoalReached;
    unicycle::State final_state    = {7.04, 3.10, -58.4 * M_PI / 180.0};
    int             final_step     = 11;

    // --- Monte Carlo analysis ---
    bool  mc_running   = false;
    int   mc_samples   = 200;
    float mc_progress  = 1.0f;
    MonteCarloStats mc_stats;
    std::vector<std::vector<unicycle::State>> mc_trajectories;

    // --- UI toggles & interaction ---
    bool show_grid         = true;
    bool show_trajectories = true;
    int  dragging_region   = 0; // 1..4
    bool dragging_robot    = false;
    int  resize_region     = 0;
    int  resize_corner     = 0;

    // --- Modal dialog states ---
    bool show_arena_modified_modal = false;
    bool show_alert_modal          = false;
    std::string alert_title;
    std::string alert_what;
    std::string alert_why;
    std::string alert_recommendation;

    void trigger_alert(const std::string& title,
                       const std::string& what,
                       const std::string& why,
                       const std::string& recommendation);
};

void init_app_state(AppState& app);
void mark_arena_modified(AppState& app);

} // namespace robot_nav
