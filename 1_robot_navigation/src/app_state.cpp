#include "app_state.hpp"

namespace robot_nav {

void AppState::trigger_alert(const std::string& title,
                             const std::string& what,
                             const std::string& why,
                             const std::string& recommendation)
{
    alert_title = title;
    alert_what = what;
    alert_why = why;
    alert_recommendation = recommendation;
    show_alert_modal = true;
}

void mark_arena_modified(AppState& app) {
    app.arena_modified = true;
}

void init_app_state(AppState& app) {
    // Exact layout matching the specification
    app.regions.r1_x0 = 3.5;
    app.regions.r1_x1 = 9.7;
    app.regions.r1_y0 = 7.5;
    app.regions.r1_y1 = 9.8;

    app.regions.r2_x0 = 0.3;
    app.regions.r2_x1 = 9.8;
    app.regions.r2_y0 = 0.3;
    app.regions.r2_y1 = 2.3;

    app.regions.r3_x0 = 6.8;
    app.regions.r3_x1 = 9.2;
    app.regions.r3_y0 = 2.7;
    app.regions.r3_y1 = 5.0;

    app.regions.r4_x0 = 5.5;
    app.regions.r4_x1 = 10.0;
    app.regions.r4_y0 = 4.1;
    app.regions.r4_y1 = 7.0;

    app.initial_regions = app.regions;
    app.arena_modified = false;

    app.robot_start = {1.23, 9.29, -29.7 * M_PI / 180.0};
    app.sim_max_steps = 200;
    app.sim_with_noise = true;

    // Default controller status
    app.controller_ready = false;
    app.solve_iterations = 25;
    app.solve_duration = 1.206f;

    // Default run result matching reference
    app.has_run_result = true;
    app.last_status = RunStatus::GoalReached;
    app.final_state = {7.04, 3.10, -58.4 * M_PI / 180.0};
    app.final_step = 11;
    app.sim_robot_pos = app.final_state;

    // Pre-populate active trajectory matching reference path
    app.active_trajectory.clear();
    app.active_trajectory.push_back({1.23, 9.29, -29.7 * M_PI / 180.0});
    app.active_trajectory.push_back({1.80, 8.85, -25.0 * M_PI / 180.0});
    app.active_trajectory.push_back({2.50, 8.75, -5.0 * M_PI / 180.0});
    app.active_trajectory.push_back({3.10, 8.75, 0.0});
    app.active_trajectory.push_back({3.60, 8.45, -35.0 * M_PI / 180.0});
    app.active_trajectory.push_back({3.90, 8.20, -50.0 * M_PI / 180.0});
    app.active_trajectory.push_back({3.95, 6.00, -88.0 * M_PI / 180.0});
    app.active_trajectory.push_back({4.30, 4.00, -75.0 * M_PI / 180.0});
    app.active_trajectory.push_back({5.10, 3.60, -35.0 * M_PI / 180.0});
    app.active_trajectory.push_back({6.00, 3.20, -20.0 * M_PI / 180.0});
    app.active_trajectory.push_back({6.80, 3.10, -10.0 * M_PI / 180.0});
    app.active_trajectory.push_back({7.04, 3.10, -58.4 * M_PI / 180.0});

    // Default Monte Carlo status
    app.mc_samples = 200;
    app.mc_progress = 1.0f;
    app.mc_stats.total_runs = 200;
    app.mc_stats.successful = 200;
    app.mc_stats.failed = 0;
    app.mc_stats.timeout = 0;
    app.mc_stats.success_rate = 1.0f;

    // Generate bundle trajectories around active path for display
    app.mc_trajectories.clear();
    for (int k = 0; k < 60; ++k) {
        std::vector<unicycle::State> traj;
        double off_y = (k - 30) * 0.045;
        double off_curve = std::sin(k * 0.25) * 0.25;
        for (const auto& pt : app.active_trajectory) {
            double factor = (10.0 - pt.y) / 10.0;
            traj.push_back({
                pt.x + off_curve * factor + (k % 5 - 2) * 0.03,
                pt.y + off_y * (1.0 - factor * 0.5),
                pt.theta
            });
        }
        app.mc_trajectories.push_back(traj);
    }
}

} // namespace robot_nav
