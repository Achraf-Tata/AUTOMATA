#include "simulation_engine.hpp"
#include "backward_reachability.hpp"
#include <random>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>

namespace robot_nav::engine {

using namespace std;
using namespace chrono;
using namespace grid;
using namespace automaton;
using namespace solver;
using namespace unicycle;
using enum RunStatus;

// Advances the simulated robot by a single discrete sampling step (dt = 1.0 s).
// Here is how each step works in our closed-loop pipeline:
// 1. Guard checks: make sure controller policy is loaded, steps < max_steps, and robot is inside [0, 10] m.
// 2. Continuous-to-discrete translation: quantize robot continuous pose (x, y, theta) to spatial cell index.
// 3. Environment query: check which labeled region the robot is in (FreeSpace, R1, R2, R3, or R4 obstacle).
// 4. Mission DFA update: advance the automaton state psi' = delta(psi, label).
//    - If psi' == 4 (GoalReached), we won! Stop simulation with success.
//    - If psi' == 5 (SinkTrap), we violated safety or visited both R1 and R2! Stop with violation.
// 5. Look up optimal control: query policy table ctrl[product_index(psi', cell_idx)].
// 6. Kinematic propagation: integrate unicycle equations with bounded random disturbance w in [-0.05, 0.05].
// 7. Record the new pose into active_trajectory for the UI drawing canvas.
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
        app.last_status = Timeout;
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

    // Boundary check: ensure the robot stays within the 10x10 meter workspace
    if (!is_in_workspace(app.sim_robot_pos)) {
        app.sim_running = false;
        app.has_run_result = true;
        app.last_status = OutOfBounds;
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

    GridCoord gc = quantize(app.sim_robot_pos);
    int lab = get_region_label(app.sim_robot_pos.x, app.sim_robot_pos.y, app.regions);

    // Collision check: region label 5 corresponds to obstacle R4
    if (lab == 5) {
        app.sim_running = false;
        app.has_run_result = true;
        app.last_status = ObstacleCollision;
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

    // Update the mission automaton state based on the current region observation
    int next_psi = dfa_transition(app.sim_dfa_state, lab);
    app.sim_dfa_state = next_psi;

    // Check if the accepting goal state (q4) is reached
    if (next_psi == 4) {
        app.sim_running = false;
        app.has_run_result = true;
        app.last_status = GoalReached;
        app.final_state = app.sim_robot_pos;
        app.final_step = app.sim_step;
        return false;
    }

    // Check if the sink trap state (q5) was entered (e.g., visiting both R1 and R2)
    if (next_psi == 5) {
        app.sim_running = false;
        app.has_run_result = true;
        app.last_status = ExclusivityViolation;
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

    // Look up the feedback control in the precomputed product controller table
    int pidx = product_index(next_psi, flat_index(gc));
    uint8_t sigma = (pidx < static_cast<int>(app.ctrl.size())) ? app.ctrl[pidx] : 0;

    if (sigma == 0 || sigma > 15) {
        app.sim_running = false;
        app.has_run_result = true;
        app.last_status = Uncontrollable;
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

    // sigma is 1-indexed (1..15), mapping to ALL_CONTROLS[0..14]
    const Control& u = ALL_CONTROLS[sigma - 1];
    app.sim_last_u = u;

    // Apply bounded disturbance vector w in [-0.05, 0.05] m if noise is enabled
    array<double, 3> noise = {0.0, 0.0, 0.0};
    if (app.sim_with_noise) {
        static mt19937 rng(1337);
        uniform_real_distribution<double> dist(-0.05, 0.05);
        noise = {dist(rng), dist(rng), dist(rng)};
    }

    // Discrete-time forward integration: x(k+1) = f(x(k), u(k)) + w(k)
    app.sim_robot_pos = step(app.sim_robot_pos, u, noise, 1.0);

    app.active_trajectory.push_back(app.sim_robot_pos);
    ++app.sim_step;

    if (!is_in_workspace(app.sim_robot_pos)) {
        app.sim_running = false;
        app.has_run_result = true;
        app.last_status = OutOfBounds;
        app.final_state = app.sim_robot_pos;
        app.final_step = app.sim_step;
        return false;
    }

    return true;
}

// Runs the full simulation from start to finish without per-frame animation delays
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

// Helper to simulate a single closed-loop trajectory under random physical disturbances
static AppState::BundleRoute simulate_single_bundle_route(const AppState& app, const State& start_pose, mt19937& rng) {
    AppState::BundleRoute route;
    route.path.push_back(start_pose);
    State s = start_pose;
    int q = 1;
    uniform_real_distribution<double> dist(-0.05, 0.05);

    for (int step_idx = 0; step_idx < app.sim_max_steps; ++step_idx) {
        if (!is_in_workspace(s)) {
            route.status = OutOfBounds;
            break;
        }

        int lab = get_region_label(s.x, s.y, app.regions);
        if (lab == 5) {
            route.status = ObstacleCollision;
            break;
        }

        int next_q = dfa_transition(q, lab);
        q = next_q;

        if (q == 2 && route.waypoint_visited == 0) {
            route.waypoint_visited = 1;
        } else if (q == 3 && route.waypoint_visited == 0) {
            route.waypoint_visited = 2;
        }

        if (q == 4) {
            route.status = GoalReached;
            break;
        }
        if (q == 5) {
            route.status = ExclusivityViolation;
            break;
        }

        GridCoord gc = quantize(s);
        int pidx = product_index(q, flat_index(gc));
        uint8_t sigma = (pidx < static_cast<int>(app.ctrl.size())) ? app.ctrl[pidx] : 0;
        if (sigma == 0 || sigma > 15) {
            route.status = Uncontrollable;
            break;
        }

        const Control& u = ALL_CONTROLS[sigma - 1];
        array<double, 3> noise = {dist(rng), dist(rng), dist(rng)};
        s = step(s, u, noise, 1.0);
        route.path.push_back(s);
        if (!is_in_workspace(s)) {
            route.status = OutOfBounds;
            break;
        }
    }

    if (route.status == Idle && route.path.size() >= static_cast<size_t>(app.sim_max_steps)) {
        route.status = Timeout;
    }
    return route;
}

// Simulates a bundle of runs with random disturbances from starting pose to discover all possible routes
void generate_trajectory_bundle(AppState& app, int num_runs) {
    if (!app.controller_ready) return;

    app.trajectory_bundle.clear();
    app.trajectory_bundle.reserve(num_runs);
    app.bundle_r1_count = 0;
    app.bundle_r2_count = 0;
    app.bundle_success_count = 0;

    mt19937 rng(static_cast<unsigned int>(chrono::system_clock::now().time_since_epoch().count()));

    for (int i = 0; i < num_runs; ++i) {
        auto route = simulate_single_bundle_route(app, app.robot_start, rng);
        if (route.status == GoalReached) {
            ++app.bundle_success_count;
            if (route.waypoint_visited == 1) {
                ++app.bundle_r1_count;
            } else if (route.waypoint_visited == 2) {
                ++app.bundle_r2_count;
            }
        }
        app.trajectory_bundle.push_back(move(route));
    }
}

// Launches backward reachability synthesis on a background worker thread
// so the UI remains fluid and responsive during computation
void trigger_synthesis(AppState& app) {
    if (app.solving || !app.abs_tables_ready) return;
    app.solving = true;
    app.controller_ready = false;
    app.sim_running = false;
    app.solve_progress = 0.0f;
    app.solve_changes = 0;
    app.solve_iterations = 0;
    app.synthesis_progress = make_shared<SynthesisProgress>();
    const auto progress = app.synthesis_progress;
    // The worker owns snapshots; GUI edits cannot modify an active calculation.
    app.synthesis_job = async(launch::async,
        [regions = app.regions, tables = app.abs_tables, parallel = app.use_parallel, progress]() {
            SynthesisOutput output;
            output.regions = regions;
            output.values = init_value_function();
            output.controls = init_controller();
            const auto start = steady_clock::now();
            output.iterations = solve_reachability_backward_cpp(
                output.values, output.controls, regions, tables,
                [progress](int iteration, float changes) {
                    progress->iteration.store(iteration);
                    progress->changes.store(static_cast<int>(changes));
                }, parallel, [progress]() { return progress->cancelled.load(); });
            output.duration = duration<float>(steady_clock::now() - start).count();
            return output;
        });
}

bool poll_synthesis(AppState& app, bool wait) {
    if (!app.synthesis_job.valid()) return false;
    app.solve_iterations = app.synthesis_progress->iteration.load();
    app.solve_changes = app.synthesis_progress->changes.load();
    app.solve_progress = min(0.99f, app.solve_iterations / 100.0f);
    if (wait) app.synthesis_job.wait();
    if (app.synthesis_job.wait_for(milliseconds(0)) != future_status::ready) return false;
    app.solving = false;
    try {
        auto output = app.synthesis_job.get();
        if (app.synthesis_progress->cancelled.load() || output.iterations < 0) return false;
        if (!(output.regions == app.regions)) {
            app.trigger_alert("Arena Changed", "The completed controller belongs to the previous arena.",
                              "Regions changed during synthesis.", "Run the solver for the current arena.");
            return false;
        }
        app.V = move(output.values);
        app.ctrl = move(output.controls);
        app.solve_iterations = output.iterations;
        app.solve_duration = output.duration;
        app.solve_progress = 1.0f;
        app.controller_ready = true;
        app.arena_modified = false;
        app.initial_regions = output.regions;
        save_controller_binary(app.controller_filename, app.V, app.ctrl);
        if (app.auto_simulate_after_solve) {
            app.auto_simulate_after_solve = false;
            run_simulation(app);
        }
        return true;
    } catch (const exception& error) {
        app.trigger_alert("Synthesis Failed", error.what(),
                          "The background calculation could not finish.", "Run the solver again.");
        return false;
    }
}

void cancel_synthesis(AppState& app) {
    if (!app.synthesis_job.valid()) return;
    app.synthesis_progress->cancelled.store(true);
    poll_synthesis(app, true);
}

} // namespace robot_nav::engine
