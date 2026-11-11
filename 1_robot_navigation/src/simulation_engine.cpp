#include "simulation_engine.hpp"
#include "reachability_cpu.hpp"
#include <random>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>

namespace robot_nav::engine {

bool sim_step_once(AppState& app) {
    if (!app.controller_ready) {
        app.sim_running = false;
        app.trigger_alert(
            "Controller Not Ready",
            "Simulation cannot proceed because no controller policy is loaded.",
            "The backward reachability value function and policy tables must be synthesized or loaded from controller.bin first.",
            "Click 'Run solver' or 'Load' to initialize the feedback policy."
        );
        return false;
    }

    if (app.sim_step >= app.sim_max_steps) {
        app.sim_running = false;
        app.has_run_result = true;
        app.last_status = RunStatus::Timeout;
        app.final_state = app.sim_robot_pos;
        app.final_step = app.sim_step;
        app.trigger_alert(
            "Horizon Limit Reached",
            "The robot reached the maximum allotted step limit before reaching Goal R3.",
            "The time horizon limit expired before completing the waypoint specification.",
            "Increase the Horizon limit in the Simulation panel or start from an initial pose with lower value function cost."
        );
        return false;
    }

    // Boundary check
    if (app.sim_robot_pos.x < 0.0 || app.sim_robot_pos.x > 10.0 ||
        app.sim_robot_pos.y < 0.0 || app.sim_robot_pos.y > 10.0) {
        app.sim_running = false;
        app.has_run_result = true;
        app.last_status = RunStatus::OutOfBounds;
        app.final_state = app.sim_robot_pos;
        app.final_step = app.sim_step;
        app.trigger_alert(
            "Out of Bounds",
            "The robot state traversed outside the valid arena boundary [0, 10] x [0, 10] m.",
            "State trajectory exited the quantizer coordinate boundaries due to high velocities or cumulative disturbances.",
            "Re-center the robot start pose within the arena and ensure the controller is synthesized for the active domain."
        );
        return false;
    }

    grid::GridCoord gc = grid::quantize(app.sim_robot_pos);
    int lab = grid::get_region_label(app.sim_robot_pos.x, app.sim_robot_pos.y, app.regions);

    // Obstacle check
    if (lab == 5) {
        app.sim_running = false;
        app.has_run_result = true;
        app.last_status = RunStatus::ObstacleCollision;
        app.final_state = app.sim_robot_pos;
        app.final_step = app.sim_step;
        app.trigger_alert(
            "Obstacle Collision",
            "The robot collided with Obstacle Region R4.",
            "The state trajectory intersected region R4 which is marked unsafe in the specification.",
            "Check if the arena regions were modified after controller synthesis. If regions moved, run the solver to re-synthesize."
        );
        return false;
    }

    int next_psi = automaton::dfa_transition(app.sim_dfa_state, lab);
    app.sim_dfa_state = next_psi;

    if (next_psi == 4) {
        app.sim_running = false;
        app.has_run_result = true;
        app.last_status = RunStatus::GoalReached;
        app.final_state = app.sim_robot_pos;
        app.final_step = app.sim_step;
        return false;
    }

    if (next_psi == 5) {
        app.sim_running = false;
        app.has_run_result = true;
        app.last_status = RunStatus::ExclusivityViolation;
        app.final_state = app.sim_robot_pos;
        app.final_step = app.sim_step;
        app.trigger_alert(
            "DFA Specification Violation",
            "Mutually exclusive waypoint requirement violated (both R1 and R2 visited).",
            "The DFA transitioned to sink trap state q5 because both waypoint regions were visited before Goal R3.",
            "Follow the synthesized policy which steers the unicycle toward one exclusive waypoint."
        );
        return false;
    }

    int pidx = automaton::product_index(next_psi, grid::flat_index(gc));
    uint8_t sigma = (pidx < static_cast<int>(app.ctrl.size())) ? app.ctrl[pidx] : 0;

    if (sigma == 0 || sigma > 15) {
        app.sim_running = false;
        app.has_run_result = true;
        app.last_status = RunStatus::Uncontrollable;
        app.final_state = app.sim_robot_pos;
        app.final_step = app.sim_step;
        app.trigger_alert(
            "Uncontrollable State",
            "No valid control input is defined in the policy table for the current state.",
            "The unicycle state (x, y, theta) is outside the winning reachability domain (V = infinity).",
            "Initialize the unicycle closer to Waypoint R1 or R2, or re-run the backward reachability solver."
        );
        return false;
    }

    const unicycle::Control& u = unicycle::ALL_CONTROLS[sigma - 1];
    app.sim_last_u = u;

    std::array<double, 3> noise = {0.0, 0.0, 0.0};
    if (app.sim_with_noise) {
        static std::mt19937 rng(1337);
        std::uniform_real_distribution<double> dist(-0.05, 0.05);
        noise = {dist(rng), dist(rng), dist(rng)};
    }

    app.sim_robot_pos = unicycle::step(app.sim_robot_pos, u, noise, 1.0);
    app.sim_robot_pos.x = std::clamp(app.sim_robot_pos.x, 0.0, 10.0);
    app.sim_robot_pos.y = std::clamp(app.sim_robot_pos.y, 0.0, 10.0);

    app.active_trajectory.push_back(app.sim_robot_pos);
    ++app.sim_step;

    return true;
}

void run_simulation(AppState& app) {
    if (!app.controller_ready) {
        app.trigger_alert(
            "Controller Not Ready",
            "Cannot run simulation without a valid controller.",
            "Policy tables have not been loaded or computed.",
            "Click 'Run solver' or 'Load' first."
        );
        return;
    }

    app.sim_running = false;
    app.sim_paused  = false;
    app.sim_step    = 0;
    app.sim_dfa_state = 1;
    app.sim_robot_pos = app.robot_start;
    app.active_trajectory.clear();
    app.active_trajectory.push_back(app.robot_start);

    for (int step = 0; step < app.sim_max_steps; ++step) {
        app.sim_running = true;
        if (!sim_step_once(app)) {
            break;
        }
    }

    app.has_run_result = true;
    app.final_state = app.sim_robot_pos;
    app.final_step = app.sim_step;
}

void run_monte_carlo_analysis(AppState& app) {
    if (app.mc_running || !app.controller_ready) return;

    app.mc_running = true;
    app.mc_progress = 0.0f;
    app.mc_trajectories.clear();
    app.mc_stats = {};

    std::thread([&app]() {
        std::mt19937 rng(42);
        std::uniform_real_distribution<double> jitter(-0.15, 0.15);
        std::uniform_real_distribution<double> dist_w(-0.05, 0.05);

        int total = app.mc_samples;
        int passed = 0;
        int failed = 0;
        int timeout = 0;

        for (int i = 0; i < total; ++i) {
            unicycle::State s0 = app.robot_start;
            s0.x = std::clamp(s0.x + jitter(rng), 0.0, 10.0);
            s0.y = std::clamp(s0.y + jitter(rng), 0.0, 10.0);

            std::vector<unicycle::State> traj;
            traj.push_back(s0);
            unicycle::State cur = s0;
            int psi = 1;
            bool reached = false;
            bool hit = false;

            for (int step = 0; step < app.sim_max_steps; ++step) {
                if (cur.x < 0.0 || cur.x > 10.0 || cur.y < 0.0 || cur.y > 10.0) {
                    hit = true;
                    break;
                }

                grid::GridCoord gc = grid::quantize(cur);
                int lab = grid::get_region_label(cur.x, cur.y, app.regions);
                if (lab == 5) {
                    hit = true;
                    break;
                }

                int next_psi = automaton::dfa_transition(psi, lab);
                psi = next_psi;
                if (psi == 4) {
                    reached = true;
                    break;
                }
                if (psi == 5) {
                    hit = true;
                    break;
                }

                int pidx = automaton::product_index(psi, grid::flat_index(gc));
                uint8_t sigma = (pidx < static_cast<int>(app.ctrl.size())) ? app.ctrl[pidx] : 0;
                if (sigma == 0 || sigma > 15) {
                    break;
                }

                const unicycle::Control& u = unicycle::ALL_CONTROLS[sigma - 1];
                std::array<double, 3> noise = {dist_w(rng), dist_w(rng), dist_w(rng)};
                cur = unicycle::step(cur, u, noise, 1.0);
                cur.x = std::clamp(cur.x, 0.0, 10.0);
                cur.y = std::clamp(cur.y, 0.0, 10.0);
                traj.push_back(cur);
            }

            if (reached) {
                ++passed;
            } else if (hit) {
                ++failed;
            } else {
                ++timeout;
            }

            if (i < 80) {
                app.mc_trajectories.push_back(traj);
            }

            app.mc_progress = static_cast<float>(i + 1) / total;
        }

        app.mc_stats.total_runs   = total;
        app.mc_stats.successful   = passed;
        app.mc_stats.failed       = failed;
        app.mc_stats.timeout      = timeout;
        app.mc_stats.success_rate = (total > 0) ? (static_cast<float>(passed) / total) : 0.0f;
        app.mc_running = false;
        app.mc_progress = 1.0f;
    }).detach();
}

void trigger_synthesis(AppState& app) {
    if (app.solving || !app.abs_tables_ready) return;

    app.solving = true;
    app.controller_ready = false;
    app.solve_progress = 0.0f;
    app.solve_changes = 0;
    app.solve_iterations = 0;

    std::thread([&app]() {
        auto t0 = std::chrono::high_resolution_clock::now();
        app.V = automaton::init_value_function();
        app.ctrl = automaton::init_controller();

        app.solve_iterations = solver::solve_reachability_openmp(
            app.V, app.ctrl, app.regions, app.abs_tables,
            [&app](int it, float ch) {
                app.solve_iterations = it;
                app.solve_changes = static_cast<int>(ch);
                app.solve_progress = std::min(1.0f, static_cast<float>(it) / 35.0f);
            }
        );

        auto t1 = std::chrono::high_resolution_clock::now();
        app.solve_duration = std::chrono::duration<float>(t1 - t0).count();
        app.solving = false;
        app.controller_ready = true;
        app.solve_progress = 1.0f;
        app.arena_modified = false;
        app.initial_regions = app.regions;

        solver::save_controller_binary(app.controller_filename, app.V, app.ctrl);

        if (app.auto_simulate_after_solve) {
            app.auto_simulate_after_solve = false;
            run_simulation(app);
        }
    }).detach();
}

} // namespace robot_nav::engine
