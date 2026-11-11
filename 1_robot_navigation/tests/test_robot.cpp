// =============================================================================
// test_robot.cpp — Verification suite for 1_robot_navigation
// Tests Member 1 (Kinematics & Bounding Box)
// Tests Member 2 (Grid Quantizer & Region Labels)
// Tests Member 3 (Product Automaton & Value Function)
// Tests Member 4 (OpenMP Solver & Binary Serialization)
// =============================================================================
#include "unicycle_model.hpp"
#include "grid_quantizer.hpp"
#include "product_automaton.hpp"
#include "reachability_cpu.hpp"

#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include <cstdio>

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[-] FAILED: " << msg << " (" << #cond << ") at line " << __LINE__ << std::endl; \
            return 1; \
        } \
    } while (0)

int main() {
    std::cout << "=== Running Robot Navigation Verification Suite ===" << std::endl;

    // -------------------------------------------------------------------------
    // 1. Member 1: Unicycle Model & Interval Reachable Box
    // -------------------------------------------------------------------------
    std::cout << "[*] Testing Member 1: Unicycle Kinematics & Reachable Sets..." << std::endl;
    {
        unicycle::State s0 = {1.0, 2.0, 0.0};
        unicycle::Control u0 = {1.0, 0.0}; // forward 1 m/s, straight
        std::array<double, 3> noise_zero = {0.0, 0.0, 0.0};

        unicycle::State s1 = unicycle::step(s0, u0, noise_zero, 1.0);
        TEST_ASSERT(std::abs(s1.x - 2.0) < 1e-6, "Kinematics x forward motion");
        TEST_ASSERT(std::abs(s1.y - 2.0) < 1e-6, "Kinematics y straight motion");
        TEST_ASSERT(std::abs(s1.theta - 0.0) < 1e-6, "Kinematics theta orientation");

        // Angle wrapping test
        TEST_ASSERT(std::abs(unicycle::wrap_angle(3.5 * M_PI) - (-0.5 * M_PI)) < 1e-5, "wrap_angle positive");
        TEST_ASSERT(std::abs(unicycle::wrap_angle(-3.5 * M_PI) - (0.5 * M_PI)) < 1e-5, "wrap_angle negative");

        // Reachable box test
        unicycle::Box cell_box = {0.95, 1.05, 1.95, 2.05, -0.1, 0.1};
        unicycle::Box rbox = unicycle::compute_reachable_box(cell_box, u0, 0.05, 1.0);
        TEST_ASSERT(rbox.x_min < s1.x && rbox.x_max > s1.x, "Reachable box covers nominal x");
        TEST_ASSERT(rbox.y_min < s1.y && rbox.y_max > s1.y, "Reachable box covers nominal y");
        std::cout << "    [+] Member 1 tests PASSED." << std::endl;
    }

    // -------------------------------------------------------------------------
    // 2. Member 2: Grid Quantizer & Region Labels
    // -------------------------------------------------------------------------
    std::cout << "[*] Testing Member 2: Grid Quantizer & Region Labelling..." << std::endl;
    {
        grid::RegionConfig cfg; // default regions
        // Quantization
        grid::GridCoord gc0 = grid::quantize({0.05, 0.05, -M_PI + 0.01});
        TEST_ASSERT(gc0.c1 == 0 && gc0.c2 == 0 && gc0.c3 == 0, "Quantize bottom-left cell");

        grid::GridCoord gc_max = grid::quantize({9.99, 9.99, M_PI - 0.01});
        TEST_ASSERT(gc_max.c1 == 99 && gc_max.c2 == 99 && gc_max.c3 == 29, "Quantize top-right cell");

        // Cell center roundtrip
        unicycle::State center = grid::cell_center({50, 50, 15});
        grid::GridCoord gc_re = grid::quantize(center);
        TEST_ASSERT(gc_re.c1 == 50 && gc_re.c2 == 50 && gc_re.c3 == 15, "Cell center roundtrip");

        // Region labels: R1=[4,5]x[8.5,9.5], R2=[8.5,9.5]x[2,3], R3=[2,3]x[0.5,1.5], R4=[3,7]x[3,7]
        TEST_ASSERT(grid::get_region_label(4.5, 9.0, cfg) == 1, "Label R1");
        TEST_ASSERT(grid::get_region_label(9.0, 2.5, cfg) == 2, "Label R2");
        TEST_ASSERT(grid::get_region_label(2.5, 1.0, cfg) == 3, "Label R3 (goal)");
        TEST_ASSERT(grid::get_region_label(5.0, 5.0, cfg) == 4, "Label R4 (obstacle)");
        TEST_ASSERT(grid::get_region_label(0.5, 0.5, cfg) == 0, "Label 0 (free space)");

        // Footprint precomputation
        grid::FootprintTable fp = grid::precompute_action_footprints(0.05, 1.0);
        TEST_ASSERT(fp[0][0].dc1_max >= fp[0][0].dc1_min, "Footprint valid x range");
        TEST_ASSERT(fp[0][0].dc2_max >= fp[0][0].dc2_min, "Footprint valid y range");
        std::cout << "    [+] Member 2 tests PASSED." << std::endl;
    }

    // -------------------------------------------------------------------------
    // 3. Member 3: Product Automaton & DFA Transitions
    // -------------------------------------------------------------------------
    std::cout << "[*] Testing Member 3: Product Automaton DFA..." << std::endl;
    {
        // q=1 (searching):
        TEST_ASSERT(automaton::dfa_transition(1, 0) == 1, "q1 + free -> q1");
        TEST_ASSERT(automaton::dfa_transition(1, 1) == 2, "q1 + R1 -> q2");
        TEST_ASSERT(automaton::dfa_transition(1, 2) == 3, "q1 + R2 -> q3");
        TEST_ASSERT(automaton::dfa_transition(1, 3) == 1, "q1 + R3 -> q1 (not active yet)");
        TEST_ASSERT(automaton::dfa_transition(1, 4) == 5, "q1 + R4 -> q5 (hit obstacle)");

        // q=2 (R1 visited):
        TEST_ASSERT(automaton::dfa_transition(2, 0) == 2, "q2 + free -> q2");
        TEST_ASSERT(automaton::dfa_transition(2, 1) == 2, "q2 + R1 -> q2");
        TEST_ASSERT(automaton::dfa_transition(2, 2) == 5, "q2 + R2 -> q5 (forbidden duplicate)");
        TEST_ASSERT(automaton::dfa_transition(2, 3) == 4, "q2 + R3 -> q4 (GOAL REACHED!)");
        TEST_ASSERT(automaton::dfa_transition(2, 4) == 5, "q2 + R4 -> q5 (hit obstacle)");

        // q=3 (R2 visited):
        TEST_ASSERT(automaton::dfa_transition(3, 0) == 3, "q3 + free -> q3");
        TEST_ASSERT(automaton::dfa_transition(3, 1) == 5, "q3 + R1 -> q5 (forbidden duplicate)");
        TEST_ASSERT(automaton::dfa_transition(3, 2) == 3, "q3 + R2 -> q3");
        TEST_ASSERT(automaton::dfa_transition(3, 3) == 4, "q3 + R3 -> q4 (GOAL REACHED!)");
        TEST_ASSERT(automaton::dfa_transition(3, 4) == 5, "q3 + R4 -> q5 (hit obstacle)");

        // q=4 (goal) & q=5 (sink): absorbing
        TEST_ASSERT(automaton::dfa_transition(4, 1) == 4, "q4 absorbing");
        TEST_ASSERT(automaton::dfa_transition(5, 3) == 5, "q5 absorbing");

        // Initialization
        auto V = automaton::init_value_function();
        auto ctrl = automaton::init_controller();
        TEST_ASSERT(V.size() == static_cast<size_t>(automaton::NTOTAL), "V total size");
        TEST_ASSERT(ctrl.size() == static_cast<size_t>(automaton::NTOTAL), "ctrl total size");

        // Check goal V is 0, rest is INF
        int base_goal = (4 - 1) * grid::NCELLS;
        TEST_ASSERT(V[base_goal + 100] == 0.0f, "Goal state V is 0");
        int base_q1 = (1 - 1) * grid::NCELLS;
        TEST_ASSERT(V[base_q1 + 100] == automaton::INF, "q1 state V is INF initially");
        std::cout << "    [+] Member 3 tests PASSED." << std::endl;
    }

    // -------------------------------------------------------------------------
    // 4. Member 4: Binary Serialization
    // -------------------------------------------------------------------------
    std::cout << "[*] Testing Member 4: Controller Serialization..." << std::endl;
    {
        std::vector<float> V_test(automaton::NTOTAL, 123.45f);
        std::vector<uint8_t> ctrl_test(automaton::NTOTAL, 7u);
        const char* test_path = "test_ctrl_tmp.bin";

        bool saved = solver::save_controller_binary(test_path, V_test, ctrl_test);
        TEST_ASSERT(saved, "Save controller binary");

        std::vector<float> V_loaded;
        std::vector<uint8_t> ctrl_loaded;
        bool loaded = solver::load_controller_binary(test_path, V_loaded, ctrl_loaded);
        TEST_ASSERT(loaded, "Load controller binary");
        TEST_ASSERT(V_loaded.size() == V_test.size(), "Loaded size matches V");
        TEST_ASSERT(ctrl_loaded.size() == ctrl_test.size(), "Loaded size matches ctrl");
        TEST_ASSERT(V_loaded[42] == 123.45f, "Loaded V content matches");
        TEST_ASSERT(ctrl_loaded[42] == 7u, "Loaded ctrl content matches");

        std::remove(test_path);
        std::cout << "    [+] Member 4 serialization tests PASSED." << std::endl;
    }

    std::cout << "\n>>> ALL 4 VERIFICATION TEST SUITES PASSED CLEANLY! <<<" << std::endl;
    return 0;
}
