#include "app_state.hpp"

namespace robot_nav {

using namespace std;
using enum RunStatus;

// Displays an alert modal with details on why simulation stopped or warning occurred
void AppState::trigger_alert(const string& title,
                             const string& what,
                             const string& why,
                             const string& recommendation)
{
    alert_title = title;
    alert_what = what;
    alert_why = why;
    alert_recommendation = recommendation;
    show_alert_modal = true;
}

// Marks the arena geometry as changed by the user (dragging/resizing regions)
void mark_arena_modified(AppState& app) {
    app.arena_modified = true;
}

// Resets the robot state back to its initial start pose and clears the drawn trajectory
void reset_simulation(AppState& app) {
    app.sim_running = false;
    app.sim_paused = false;
    app.sim_step = 0;
    app.sim_dfa_state = 1; // Return DFA to initial state q1
    app.sim_robot_pos = app.robot_start;
    app.sim_last_u = {0.0, 0.0};
    app.active_trajectory.clear();
    app.active_trajectory.push_back(app.robot_start);
    app.has_run_result = false;
    app.last_status = Idle;
    app.final_state = app.robot_start;
    app.final_step = 0;
}

// Sets up default arena configuration, benchmark coordinates, and initial robot pose
void init_app_state(AppState& app) {
    // Default benchmark regions from Antoine Girard's paper:
    // R1 (Waypoint 1): [4.0, 5.0] x [8.5, 9.5]
    app.regions.r1_x0 = 4.0;
    app.regions.r1_x1 = 5.0;
    app.regions.r1_y0 = 8.5;
    app.regions.r1_y1 = 9.5;

    // R2 (Waypoint 2): [8.5, 9.5] x [2.0, 3.0]
    app.regions.r2_x0 = 8.5;
    app.regions.r2_x1 = 9.5;
    app.regions.r2_y0 = 2.0;
    app.regions.r2_y1 = 3.0;

    // R3 (Goal region): [2.0, 3.0] x [0.5, 1.5]
    app.regions.r3_x0 = 2.0;
    app.regions.r3_x1 = 3.0;
    app.regions.r3_y0 = 0.5;
    app.regions.r3_y1 = 1.5;

    // R4 (Obstacle region): [3.0, 7.0] x [3.0, 7.0]
    app.regions.r4_x0 = 3.0;
    app.regions.r4_x1 = 7.0;
    app.regions.r4_y0 = 3.0;
    app.regions.r4_y1 = 7.0;

    app.initial_regions = app.regions;
    app.arena_modified = false;

    // Default robot starting pose: (x = 1.0 m, y = 2.0 m, theta = 0 rad)
    app.robot_start = {1.0, 2.0, 0.0};
    app.sim_robot_pos = app.robot_start;
    app.active_trajectory.clear();
    app.active_trajectory.push_back(app.robot_start);

    // Initial simulation telemetry
    app.sim_running = false;
    app.sim_paused = false;
    app.sim_step = 0;
    app.sim_max_steps = 200;
    app.sim_with_noise = true;
    app.sim_dfa_state = 1;
    app.anim_speed = 15.0f;
    app.instant_sim = false;

    // Clean idle results
    app.has_run_result = false;
    app.last_status = Idle;
    app.final_state = app.robot_start;
    app.final_step = 0;

    // Synthesis clean state
    app.solving = false;
    app.solve_iterations = 0;
    app.solve_duration = 0.0f;
    app.solve_progress = 0.0f;
    app.solve_changes = 0;
    app.controller_ready = false;
}

} // namespace robot_nav
