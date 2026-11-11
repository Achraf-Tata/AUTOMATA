#include "unicycle_model.hpp"
#include "grid_quantizer.hpp"
#include "product_automaton.hpp"
#include "reachability_cpu.hpp"

#include <iostream>
#include <chrono>
#include <cmath>
#include <vector>
#include <cstdio>

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "Assertion failed: " << msg << " (" << #cond << ") at line " << __LINE__ << std::endl; \
            return 1; \
        } \
    } while (0)

int main() {
    std::cout << "Running robot navigation test suite..." << std::endl;

    {
        unicycle::State s0 = {1.0, 2.0, 0.0};
        unicycle::Control u0 = {1.0, 0.0};
        std::array<double, 3> noise_zero = {0.0, 0.0, 0.0};

        unicycle::State s1 = unicycle::step(s0, u0, noise_zero, 1.0);
        TEST_ASSERT(std::abs(s1.x - 2.0) < 1e-6, "Kinematics x forward motion");
        TEST_ASSERT(std::abs(s1.y - 2.0) < 1e-6, "Kinematics y straight motion");
        TEST_ASSERT(std::abs(s1.theta - 0.0) < 1e-6, "Kinematics theta orientation");

        TEST_ASSERT(std::abs(unicycle::wrap_angle(3.5 * M_PI) - (-0.5 * M_PI)) < 1e-5, "wrap_angle positive");
        TEST_ASSERT(std::abs(unicycle::wrap_angle(-3.5 * M_PI) - (0.5 * M_PI)) < 1e-5, "wrap_angle negative");

        unicycle::Box cell_box = {0.95, 1.05, 1.95, 2.05, -0.1, 0.1};
        unicycle::Box rbox = unicycle::compute_reachable_box(cell_box, u0, 0.05, 1.0);
        TEST_ASSERT(rbox.x_min < s1.x && rbox.x_max > s1.x, "Reachable box covers nominal x");
        TEST_ASSERT(rbox.y_min < s1.y && rbox.y_max > s1.y, "Reachable box covers nominal y");
        std::cout << "Unicycle kinematics tests passed." << std::endl;
    }

    {
        grid::RegionConfig cfg;
        grid::GridCoord gc0 = grid::quantize({0.05, 0.05, -M_PI + 0.01});
        TEST_ASSERT(gc0.cell_x == 0 && gc0.cell_y == 0 && gc0.cell_theta == 0, "Quantize bottom-left cell");

        grid::GridCoord gc_max = grid::quantize({9.99, 9.99, M_PI - 0.01});
        TEST_ASSERT(gc_max.cell_x == 99 && gc_max.cell_y == 99 && gc_max.cell_theta == 29, "Quantize top-right cell");

        unicycle::State center = grid::cell_center({50, 50, 15});
        grid::GridCoord gc_re = grid::quantize(center);
        TEST_ASSERT(gc_re.cell_x == 50 && gc_re.cell_y == 50 && gc_re.cell_theta == 15, "Cell center roundtrip");

        TEST_ASSERT(grid::get_region_label(4.5, 9.0, cfg) == 2, "Label R1");
        TEST_ASSERT(grid::get_region_label(9.0, 2.5, cfg) == 3, "Label R2");
        TEST_ASSERT(grid::get_region_label(2.5, 1.0, cfg) == 4, "Label R3");
        TEST_ASSERT(grid::get_region_label(5.0, 5.0, cfg) == 5, "Label R4");
        TEST_ASSERT(grid::get_region_label(0.5, 0.5, cfg) == 1, "Label Free");

        auto abs_tables = grid::precompute_abstraction_tables(0.05, 1.0);
        TEST_ASSERT(abs_tables.get_c1_max(50, 15, 0) >= abs_tables.get_c1_min(50, 15, 0), "Valid x bounds");
        TEST_ASSERT(abs_tables.get_c2_max(50, 15, 0) >= abs_tables.get_c2_min(50, 15, 0), "Valid y bounds");
        std::cout << "Grid quantizer and abstraction tests passed." << std::endl;
    }

    {
        TEST_ASSERT(automaton::dfa_transition(1, 1) == 1, "q1 + free -> q1");
        TEST_ASSERT(automaton::dfa_transition(1, 2) == 2, "q1 + R1 -> q2");
        TEST_ASSERT(automaton::dfa_transition(1, 3) == 3, "q1 + R2 -> q3");
        TEST_ASSERT(automaton::dfa_transition(1, 4) == 1, "q1 + R3 -> q1");
        TEST_ASSERT(automaton::dfa_transition(1, 5) == 5, "q1 + R4 -> q5");

        TEST_ASSERT(automaton::dfa_transition(2, 1) == 2, "q2 + free -> q2");
        TEST_ASSERT(automaton::dfa_transition(2, 2) == 2, "q2 + R1 -> q2");
        TEST_ASSERT(automaton::dfa_transition(2, 3) == 5, "q2 + R2 -> q5");
        TEST_ASSERT(automaton::dfa_transition(2, 4) == 4, "q2 + R3 -> q4");
        TEST_ASSERT(automaton::dfa_transition(2, 5) == 5, "q2 + R4 -> q5");

        TEST_ASSERT(automaton::dfa_transition(3, 1) == 3, "q3 + free -> q3");
        TEST_ASSERT(automaton::dfa_transition(3, 2) == 5, "q3 + R1 -> q5");
        TEST_ASSERT(automaton::dfa_transition(3, 3) == 3, "q3 + R2 -> q3");
        TEST_ASSERT(automaton::dfa_transition(3, 4) == 4, "q3 + R3 -> q4");
        TEST_ASSERT(automaton::dfa_transition(3, 5) == 5, "q3 + R4 -> q5");

        TEST_ASSERT(automaton::dfa_transition(4, 2) == 4, "q4 absorbing");
        TEST_ASSERT(automaton::dfa_transition(5, 4) == 5, "q5 absorbing");

        auto V = automaton::init_value_function();
        auto ctrl = automaton::init_controller();
        TEST_ASSERT(V.size() == static_cast<size_t>(automaton::NTOTAL), "V size");
        TEST_ASSERT(ctrl.size() == static_cast<size_t>(automaton::NTOTAL), "ctrl size");
        int base_goal = (4 - 1) * grid::NCELLS;
        TEST_ASSERT(V[base_goal + 100] == 0.0f, "Goal state V is 0");
        int base_q1 = (1 - 1) * grid::NCELLS;
        TEST_ASSERT(V[base_q1 + 100] == automaton::INF, "q1 state V is INF initially");
        std::cout << "Product automaton tests passed." << std::endl;
    }

    {
        grid::RegionConfig cfg;
        auto abs_tables = grid::precompute_abstraction_tables(0.05, 1.0);
        auto V = automaton::init_value_function();
        auto ctrl = automaton::init_controller();

        auto t0 = std::chrono::high_resolution_clock::now();
        int iters = solver::solve_reachability_openmp(V, ctrl, cfg, abs_tables);
        auto t1 = std::chrono::high_resolution_clock::now();
        double elapsed_sec = std::chrono::duration<double>(t1 - t0).count();

        std::cout << "Solver completed " << iters << " iterations in " << elapsed_sec << " s." << std::endl;
        TEST_ASSERT(iters > 10 && iters <= 50, "Solver converged in realistic iteration count");

        grid::GridCoord gc_start = grid::quantize({1.0, 2.0, 0.0});
        int pidx_q1 = automaton::product_index(1, grid::flat_index(gc_start));
        TEST_ASSERT(V[pidx_q1] < 1e7f, "Start position is controllable in V");

        const char* test_path = "test_ctrl_tmp.bin";
        bool saved = solver::save_controller_binary(test_path, V, ctrl);
        TEST_ASSERT(saved, "Save controller binary");

        std::vector<float> V_loaded;
        std::vector<uint8_t> ctrl_loaded;
        bool loaded = solver::load_controller_binary(test_path, V_loaded, ctrl_loaded);
        TEST_ASSERT(loaded, "Load controller binary");
        TEST_ASSERT(V_loaded.size() == V.size(), "Loaded size matches V");
        TEST_ASSERT(ctrl_loaded.size() == ctrl.size(), "Loaded size matches ctrl");
        TEST_ASSERT(V_loaded[pidx_q1] == V[pidx_q1], "Loaded start V matches");

        std::remove(test_path);
        std::cout << "Serialization tests passed." << std::endl;

        unicycle::State rp = {1.0, 2.0, 0.0};
        int psi = 1;
        bool reached = false;
        int steps = 0;

        for (; steps < 150; ++steps) {
            grid::GridCoord gc = grid::quantize(rp);
            int lab = grid::get_region_label(rp.x, rp.y, cfg);
            int next_psi = automaton::dfa_transition(psi, lab);
            psi = next_psi;

            if (psi == 4) {
                reached = true;
                break;
            }
            if (psi == 5) {
                TEST_ASSERT(false, "Simulation entered invalid state");
            }

            int pidx = automaton::product_index(psi, grid::flat_index(gc));
            uint8_t sigma = ctrl[pidx];
            TEST_ASSERT(sigma >= 1 && sigma <= 15, "Controller returned valid action");

            const unicycle::Control& u = unicycle::ALL_CONTROLS[sigma - 1];
            std::array<double, 3> noise = {0.0, 0.0, 0.0};
            rp = unicycle::step(rp, u, noise, 1.0);
        }

        TEST_ASSERT(reached, "Robot reached target state q=4 within 150 steps");
        std::cout << "Closed-loop simulation reached goal in " << steps << " steps." << std::endl;
    }

    std::cout << "All test suites completed successfully." << std::endl;
    return 0;
}
