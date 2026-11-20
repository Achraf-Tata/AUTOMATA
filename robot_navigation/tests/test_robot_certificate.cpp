#include "simulation_engine.hpp"
#include "backward_reachability.hpp"
#include <array>
#include <iostream>
#include <stdexcept>

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void check_concrete_successors(const grid::AbstractionTables& tables) {
    for (int x : {5, 25, 50, 75, 95}) {
        for (int y : {5, 25, 50, 75, 95}) {
            for (int heading = 0; heading < grid::NTHETA; ++heading) {
                for (int input = 0; input < 15; ++input) {
                    if (!tables.get_valid1(x, heading, input) || !tables.get_valid2(y, heading, input)) continue;
                    for (int corner = 0; corner < 8; ++corner) {
                        const unicycle::State initial{
                            (x + ((corner & 1) != 0)) * grid::DX,
                            (y + ((corner & 2) != 0)) * grid::DY,
                            grid::TH_MIN + (heading + ((corner & 4) != 0)) * grid::DTHETA
                        };
                        for (int disturbance = 0; disturbance < 8; ++disturbance) {
                            const std::array<double, 3> noise{
                                (disturbance & 1) ? 0.05 : -0.05,
                                (disturbance & 2) ? 0.05 : -0.05,
                                (disturbance & 4) ? 0.05 : -0.05
                            };
                            const auto next = unicycle::step(initial, unicycle::ALL_CONTROLS[input], noise);
                            const auto cell = grid::quantize(next);
                            check(grid::is_in_workspace(next), "A valid input left the workspace");
                            check(cell.cell_x >= tables.get_c1_min(x, heading, input) &&
                                  cell.cell_x <= tables.get_c1_max(x, heading, input), "Missing concrete X successor");
                            check(cell.cell_y >= tables.get_c2_min(y, heading, input) &&
                                  cell.cell_y <= tables.get_c2_max(y, heading, input), "Missing concrete Y successor");
                            const int first = tables.get_c3_min(heading, input);
                            const int last = tables.get_c3_max(heading, input);
                            const bool covered = first <= last
                                ? cell.cell_theta >= first && cell.cell_theta <= last
                                : cell.cell_theta >= first || cell.cell_theta <= last;
                            check(covered, "Missing concrete angular successor");
                        }
                    }
                }
            }
        }
    }
}

void check_policy(const std::vector<float>& values, const std::vector<uint8_t>& controls,
                  const grid::AbstractionTables& tables, const grid::RegionConfig& regions) {
    std::array<std::array<uint8_t, grid::NY>, grid::NX> labels{};
    for (int x = 0; x < grid::NX; ++x) {
        for (int y = 0; y < grid::NY; ++y) {
            labels[x][y] = grid::get_cell_observation_mask(
                x * grid::DX, (x + 1) * grid::DX, y * grid::DY, (y + 1) * grid::DY, regions);
        }
    }
    size_t checked = 0;
    for (int q = 1; q <= 3; ++q) {
        for (int x = 0; x < grid::NX; ++x) {
            for (int y = 0; y < grid::NY; ++y) {
                for (int heading = 0; heading < grid::NTHETA; ++heading) {
                    const int state = automaton::product_index(q, grid::flat_index(x, y, heading));
                    if (values[state] >= automaton::INF) continue;
                    ++checked;
                    check(!(labels[x][y] & 16), "An unsafe cell was certified");
                    check(controls[state] >= 1 && controls[state] <= 15, "A winning state has no command");
                    const int input = controls[state] - 1;
                    check(tables.get_valid1(x, heading, input) && tables.get_valid2(y, heading, input),
                          "A certified command may leave the domain");
                    const int first = tables.get_c3_min(heading, input);
                    const int count = (tables.get_c3_max(heading, input) - first + grid::NTHETA) % grid::NTHETA + 1;
                    for (int sx = tables.get_c1_min(x, heading, input); sx <= tables.get_c1_max(x, heading, input); ++sx) {
                        for (int sy = tables.get_c2_min(y, heading, input); sy <= tables.get_c2_max(y, heading, input); ++sy) {
                            for (int label = 1; label <= 5; ++label) {
                                if (!(labels[sx][sy] & (1u << (label - 1)))) continue;
                                const int next_q = automaton::dfa_transition(q, label);
                                for (int offset = 0; offset < count; ++offset) {
                                    const int next = automaton::product_index(next_q,
                                        grid::flat_index(sx, sy, (first + offset) % grid::NTHETA));
                                    check(values[next] < values[state], "A selected command does not guarantee progress");
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    check(checked > 0, "The certificate contains no nonterminal winning state");
    std::cout << "Verified all successors of " << checked << " nonterminal winning states.\n";
}

int main() {
    try {
        const auto tables = grid::precompute_abstraction_tables();
        const grid::RegionConfig regions;
        check_concrete_successors(tables);
        auto values = automaton::init_value_function();
        auto controls = automaton::init_controller();
        const int parallel_iterations = solver::solve_reachability_backward_cpp(values, controls, regions, tables);
        check(parallel_iterations < 100, "Parallel synthesis did not converge");
        auto sequential_values = automaton::init_value_function();
        auto sequential_controls = automaton::init_controller();
        const int sequential_iterations = solver::solve_reachability_backward_cpp(
            sequential_values, sequential_controls, regions, tables, nullptr, false);
        check(parallel_iterations == sequential_iterations && values == sequential_values && controls == sequential_controls,
              "Parallel and sequential policies differ");
        check_policy(values, controls, tables, regions);
        for (const auto start : std::array<unicycle::State, 3>{{{0.5, 4.5, 0}, {0.5, 4, 0}, {6.8, 9, 0}}}) {
            robot_nav::AppState app;
            robot_nav::init_app_state(app);
            app.V = values;
            app.ctrl = controls;
            app.controller_ready = true;
            app.robot_start = start;
            app.sim_with_noise = false;
            robot_nav::engine::run_simulation(app);
            check(app.last_status == robot_nav::RunStatus::GoalReached, "A paper scenario failed");
            robot_nav::engine::generate_trajectory_bundle(app, 50);
            check(app.bundle_success_count == 50, "A perturbed paper scenario failed");
            std::cout << "Paper start (" << start.x << ", " << start.y << "): "
                      << app.sim_step << " steps, 50/50 perturbed runs passed.\n";
        }
        auto edited_regions = regions;
        edited_regions.r1_x0 += 0.03;
        edited_regions.r1_x1 += 0.03;
        edited_regions.r4_x0 += 0.04;
        edited_regions.r4_x1 += 0.04;
        auto edited_values = automaton::init_value_function();
        auto edited_controls = automaton::init_controller();
        check(solver::solve_reachability_backward_cpp(edited_values, edited_controls, edited_regions, tables) < 100,
              "Synthesis did not converge for non-aligned regions");
        check_policy(edited_values, edited_controls, tables, edited_regions);
        std::cout << "Robot certificate checks passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
