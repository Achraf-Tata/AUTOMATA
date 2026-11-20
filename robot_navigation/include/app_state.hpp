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

using namespace std;
using namespace grid;
using namespace unicycle;

// Possible terminal outcomes for a robot navigation trajectory run
enum class RunStatus {
    Idle,
    Running,
    GoalReached,          // Successfully visited an exclusive waypoint (R1 or R2) then reached Goal R3
    ObstacleCollision,    // Hit unsafe Obstacle region R4
    OutOfBounds,          // Wandered outside the [0, 10] x [0, 10] workspace
    ExclusivityViolation, // Violated DFA specification by entering both R1 and R2
    Timeout,              // Reached step limit without reaching the goal
    Uncontrollable        // Robot entered a state where no valid control action exists in the winning set
};
using enum RunStatus;

// Global application state holding configuration, synthesis tables, and interactive UI states
struct AppState {
    // --- Arena configuration ---
    RegionConfig regions;
    RegionConfig initial_regions;
    bool arena_modified = false; // Set to true if the user dragged or resized any region

    // --- Robot initial configuration ---
    State robot_start = {1.0, 2.0, 0.0}; // Default starting pose: x = 1.0 m, y = 2.0 m, theta = 0 rad

    // --- Precomputed abstraction tables ---
    AbstractionTables abs_tables;
    bool abs_tables_ready = false;

    // --- Controller synthesis state ---
    bool  use_parallel              = true;   // Multi-core CPU parallelization via OpenMP (default: active)
    vector<float>   V;                        // Value function array: iterations to goal (size: 1,500,000)
    vector<uint8_t> ctrl;                     // Controller lookup table: optimal control 1..15 (size: 1,500,000)
    bool  controller_ready          = false;  // True when policy tables are loaded or synthesized
    bool  solving                   = false;  // True while background solver thread is running
    int   solve_iterations          = 0;      // Number of iterations computed so far
    float solve_duration            = 0.0f;   // Total computation wall-clock time in seconds
    float solve_progress            = 0.0f;   // Fractional progress 0.0 .. 1.0 for progress bar
    int   solve_changes             = 0;      // Count of newly conquered states in current iteration
    bool  auto_simulate_after_solve = false;
    char  controller_filename[128]  = "controller.bin";

    // --- Simulation execution state ---
    bool            sim_running    = false;   // True while animated simulation is active
    bool            sim_paused     = false;   // True if animation is paused
    int             sim_step       = 0;       // Current step index
    int             sim_max_steps  = 200;     // Horizon limit
    bool            sim_with_noise = true;    // If true, injects random disturbances w in [-0.05, 0.05]
    int             sim_dfa_state  = 1;       // Current mission DFA state (1 = start)
    State           sim_robot_pos  = {1.0, 2.0, 0.0};
    Control         sim_last_u     = {0.0, 0.0};
    vector<State>   active_trajectory;        // History of robot poses for drawing the trail
    float           anim_speed     = 15.0f;   // Animation speed in steps per second (1..60)
    bool            instant_sim    = false;   // If true, computes entire run instantly without animation

    // --- Trajectory bundle (visualizing all possible routes under random physical disturbance) ---
    struct BundleRoute {
        vector<State> path;
        RunStatus     status = Idle;
        int           waypoint_visited = 0; // 1 = via R1, 2 = via R2, 0 = direct/other
    };
    vector<BundleRoute> trajectory_bundle;
    bool                show_bundle          = true;
    int                 bundle_runs          = 50;
    int                 bundle_r1_count      = 0;
    int                 bundle_r2_count      = 0;
    int                 bundle_success_count = 0;

    // --- Run result display state ---
    bool      has_run_result = false;
    RunStatus last_status    = Idle;
    State     final_state    = {1.0, 2.0, 0.0};
    int       final_step     = 0;

    // --- UI toggles & mouse interaction ---
    bool show_grid            = true;
    bool show_trajectories    = true;
    int  dragging_region      = 0; // 1..4 (which region is being moved)
    bool dragging_robot_pos   = false;
    bool dragging_robot_rot   = false;
    int  resize_region        = 0;
    int  resize_corner        = 0;

    // --- Modal dialog states ---
    bool show_arena_modified_modal = false;
    bool show_alert_modal          = false;
    string alert_title;
    string alert_what;
    string alert_why;
    string alert_recommendation;

    // Triggers an alert popup with clear, structured explanations
    void trigger_alert(const string& title,
                       const string& what,
                       const string& why,
                       const string& recommendation);
};

// Initializes default application parameters and region coordinates
void init_app_state(AppState& app);

// Flags that the arena regions have changed and the controller needs re-synthesis
void mark_arena_modified(AppState& app);

// Resets the robot back to the starting pose and clears trajectory trails
void reset_simulation(AppState& app);

} // namespace robot_nav
