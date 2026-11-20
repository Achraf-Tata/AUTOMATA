#include "simulation_engine.hpp"
#include "backward_reachability.hpp"
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <fstream>

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

void test_region_boundaries() {
    grid::RegionConfig regions;
    regions.r4_x0 = 1.02; regions.r4_x1 = 1.04;
    regions.r4_y0 = 1.02; regions.r4_y1 = 1.04;
    require(grid::get_region_label(1.05, 1.05, regions) == 1, "Expected free cell center");
    require(grid::get_cell_region_label(1.0, 1.1, 1.0, 1.1, regions) == 5,
            "An obstacle inside a cell was missed");
    regions = {};
    require(grid::get_cell_region_label(4.2, 4.3, 8.8, 8.9, regions) == 2,
            "A cell entirely in R1 must retain its label");
    require(grid::get_cell_region_label(3.95, 4.05, 8.8, 8.9, regions) == 5,
            "A cell straddling R1 must be excluded");
    require(grid::get_cell_region_label(2.9, 3.0, 4.0, 4.1, regions) == 5,
            "Contact with the closed obstacle boundary must be excluded");
}

void test_background_lifetime() {
    robot_nav::AppState app;
    robot_nav::init_app_state(app);
    app.abs_tables = grid::precompute_abstraction_tables();
    app.abs_tables_ready = true;
    robot_nav::engine::trigger_synthesis(app);
    app.regions.r1_x0 += 0.1;
    require(!robot_nav::engine::poll_synthesis(app, true), "A stale arena result was published");
    require(!app.controller_ready && !app.solving, "Stale synthesis left an active controller");
    robot_nav::engine::trigger_synthesis(app);
    robot_nav::engine::cancel_synthesis(app);
    require(!app.solving && !app.synthesis_job.valid(), "Worker was not joined on cancellation");
    {
        robot_nav::AppState closing;
        robot_nav::init_app_state(closing);
        closing.abs_tables = app.abs_tables;
        closing.abs_tables_ready = true;
        robot_nav::engine::trigger_synthesis(closing);
        // Destruction waits safely; the worker never references closing.
    }
}

void test_controller_configuration() {
    const auto file = std::filesystem::temp_directory_path() / "automata-controller-regression.bin";
    const std::string filename = file.string();
    auto values = automaton::init_value_function();
    auto controls = automaton::init_controller();
    grid::RegionConfig regions;
    require(solver::save_controller_binary(filename.c_str(), values, controls, regions), "Save failed");
    require(solver::save_controller_binary(filename.c_str(), values, controls, regions), "Atomic replacement failed");
    std::vector<float> loaded_values{42};
    std::vector<uint8_t> loaded_controls{7};
    regions.r1_x0 += 0.1;
    require(!solver::load_controller_binary(filename.c_str(), loaded_values, loaded_controls, regions),
            "A controller for a different arena was accepted");
    require(loaded_values == std::vector<float>{42} && loaded_controls == std::vector<uint8_t>{7},
            "Failed loading modified the active controller");
    regions = {};
    require(solver::load_controller_binary(filename.c_str(), loaded_values, loaded_controls, regions), "Matching file rejected");
    require(values == loaded_values && controls == loaded_controls, "Controller round trip changed data");
    {
        std::fstream damaged(file, std::ios::binary | std::ios::in | std::ios::out);
        damaged.seekp(-1, std::ios::end);
        damaged.put(1);
    }
    require(!solver::load_controller_binary(filename.c_str(), loaded_values, loaded_controls), "Payload corruption accepted");
    std::filesystem::resize_file(file, 20);
    require(!solver::load_controller_binary(filename.c_str(), loaded_values, loaded_controls), "Truncated file accepted");
    {
        std::ofstream legacy(file, std::ios::binary | std::ios::trunc);
        legacy.write("CTRL1.00", 8);
    }
    require(!solver::load_controller_binary(filename.c_str(), loaded_values, loaded_controls), "Unverifiable legacy file accepted");
    std::filesystem::remove(file);
}

int main() {
    try {
        test_workspace_exit();
        test_region_boundaries();
        test_background_lifetime();
        test_controller_configuration();
        std::cout << "Robot regression checks passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
