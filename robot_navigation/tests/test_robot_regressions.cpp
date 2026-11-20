#include "simulation_engine.hpp"
#include <iostream>
#include <stdexcept>

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void test_workspace_exit() {
    robot_nav::AppState app;
    robot_nav::init_app_state(app);
    app.robot_start = {9.9, 4.0, 0.0};
    robot_nav::reset_simulation(app);
    app.controller_ready = true;
    app.sim_with_noise = false;
    app.ctrl.assign(automaton::NTOTAL, 9); // v = 1 m/s, no turn.
    require(!robot_nav::engine::sim_step_once(app), "Outward movement must stop immediately");
    require(app.last_status == robot_nav::RunStatus::OutOfBounds, "Missing boundary violation");
    require(app.final_state.x > 10.8, "The out-of-bounds position was clipped");
    require(app.final_step == 1, "Wrong boundary-exit step");

    robot_nav::engine::generate_trajectory_bundle(app, 5);
    for (const auto& route : app.trajectory_bundle) {
        require(route.status == robot_nav::RunStatus::OutOfBounds, "Bundle hid a boundary violation");
        require(route.path.back().x > 10.8, "Bundle clipped a boundary crossing");
    }
    require(grid::is_in_workspace({0.0, 10.0, 0.0}), "Workspace boundary is included");
}

int main() {
    try {
        test_workspace_exit();
        std::cout << "Robot regression checks passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
