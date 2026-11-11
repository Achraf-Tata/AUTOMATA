#include "thermostat_model.hpp"
#include "interval_abstraction.hpp"
#include "safety_solver.hpp"
#include "bitset_solver.hpp"

#include <iostream>
#include <cstdlib>
#include <cmath>

static void check_condition(bool condition, const char* msg) {
    if (!condition) {
        std::cerr << "Assertion failed: " << msg << "\n";
        std::exit(1);
    }
}

void test_continuous_model() {
    std::cout << "[Test 1] Testing continuous model and boundaries...\n";

    double T1 = thermostat::step_euler(20.0, 0, 5.0, 1.0);
    double T2 = thermostat::step_euler(20.0, 0, 10.0, 1.0);
    double T3 = thermostat::step_euler(20.0, 1, 5.0, 1.0);
    check_condition(T1 < T2, "Warmer weather should yield higher next temperature");
    check_condition(T1 < T3, "Heater ON should yield higher next temperature");

    auto [lo, hi] = thermostat::step_interval(19.0, 20.0, 0, 5.0, 12.0, 1.0);
    check_condition(lo < hi, "Interval lower bound must be strictly less than upper bound");
    check_condition(std::abs(lo - thermostat::step_euler(19.0, 0, 5.0, 1.0)) < 1e-9, "Interval lower bound mismatch");
    check_condition(std::abs(hi - thermostat::step_euler(20.0, 0, 12.0, 1.0)) < 1e-9, "Interval upper bound mismatch");

    check_condition(thermostat::verify_continuous_boundary_invariance(), "Boundary invariance verification failed");
    std::cout << "  Passed.\n";
}

void test_interval_abstraction() {
    std::cout << "[Test 2] Testing interval quantization and post computation...\n";

    check_condition(thermostat::quantize(15.0) == 0, "quantize(15.0) != 0");
    check_condition(thermostat::quantize(15.1) == 0, "quantize(15.1) != 0");
    check_condition(thermostat::quantize(19.0) == 20, "quantize(19.0) != 20");
    check_condition(thermostat::quantize(21.9) == 34, "quantize(21.9) != 34");
    check_condition(thermostat::quantize(14.9) == thermostat::SINK_CELL, "Underflow should land in sink");
    check_condition(thermostat::quantize(25.1) == thermostat::SINK_CELL, "Overflow should land in sink");

    auto [b_lo, b_hi] = thermostat::cell_bounds(20);
    check_condition(std::abs(b_lo - 19.0) < 1e-6, "Cell 20 lower bound != 19.0");
    check_condition(std::abs(b_hi - 19.2) < 1e-6, "Cell 20 upper bound != 19.2");

    auto post0 = thermostat::compute_post_cells(20, 0);
    auto post1 = thermostat::compute_post_cells(20, 1);
    check_condition(!post0.empty(), "Post(20, 0) must not be empty");
    check_condition(!post1.empty(), "Post(20, 1) must not be empty");
    check_condition(post1[0] > post0[0], "Heater ON must shift post range upwards");

    std::cout << "  Passed.\n";
}

void test_safety_solver() {
    std::cout << "[Test 3] Testing Algorithm 1 fixed point solver & bitset equivalence...\n";

    auto trans = thermostat::build_transition_table();
    auto btable = thermostat::bitset::build_bitset_table();

    int it_std = 0;
    auto R_std = thermostat::solve_maximal_invariant_set(trans, it_std);

    int it_bit = 0;
    uint64_t R_bit = thermostat::bitset::solve_fixed_point_bitset(btable, it_bit);

    int count = 0;
    for (int s = 0; s <= thermostat::NUM_CELLS; ++s) {
        bool bit = (R_bit & (1ULL << s)) != 0;
        check_condition(R_std[s] == bit, "Standard and bitset solver disagreement");
        if (R_std[s]) ++count;
    }

    check_condition(count > 0, "Invariant set must not be empty");
    check_condition(count <= 15, "Invariant set cannot exceed safe set size");
    std::cout << "  Maximal invariant set size = " << count << " cells. Both solvers agree.\n";
    std::cout << "  Passed.\n";
}

void test_closed_loop_stress() {
    std::cout << "[Test 4] Stress testing closed-loop controller under extreme weather...\n";

    auto trans = thermostat::build_transition_table();
    int iter = 0;
    auto R_star = thermostat::solve_maximal_invariant_set(trans, iter);
    auto ctrl = thermostat::extract_controller(R_star, trans);

    // Test under freezing weather Te = 5.0 C for 10,000 steps
    double T_cold = 20.0;
    for (int t = 0; t < 10000; ++t) {
        int cell = thermostat::quantize(T_cold);
        int u = ctrl[cell];
        check_condition(T_cold >= thermostat::SAFE_MIN && T_cold <= thermostat::SAFE_MAX,
                        "Safety violation under cold weather stress");
        T_cold = thermostat::step_euler(T_cold, u, thermostat::TE_MIN, 1.0);
    }

    // Test under warm weather Te = 12.0 C for 10,000 steps
    double T_warm = 20.0;
    for (int t = 0; t < 10000; ++t) {
        int cell = thermostat::quantize(T_warm);
        int u = ctrl[cell];
        check_condition(T_warm >= thermostat::SAFE_MIN && T_warm <= thermostat::SAFE_MAX,
                        "Safety violation under warm weather stress");
        T_warm = thermostat::step_euler(T_warm, u, thermostat::TE_MAX, 1.0);
    }

    std::cout << "  Zero violations observed under 20,000 extreme-weather test steps.\n";
    std::cout << "  Passed.\n";
}

int main() {
    std::cout << "Running Room Thermostat Test Suite...\n\n";
    test_continuous_model();
    test_interval_abstraction();
    test_safety_solver();
    test_closed_loop_stress();
    std::cout << "\nAll Room Thermostat tests passed successfully.\n";
    return 0;
}
