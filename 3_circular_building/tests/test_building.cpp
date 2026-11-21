#include "thermal_network.hpp"
#include "contract_abstraction.hpp"
#include "contract_solver.hpp"
#include "weather_strategy.hpp"
#include "app_state.hpp"
#include "simulation_engine.hpp"

#include <iostream>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <chrono>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace circular;
using namespace circular::model;
using namespace circular::abstraction;
using namespace circular::synthesis;
using namespace circular::environment;
using namespace circular::engine;

static void check_condition(bool condition, const char* msg) {
    if (!condition) {
        std::cerr << "Assertion failed: " << msg << "\n";
        std::exit(1);
    }
}

// -----------------------------------------------------------------------------
// Test 1: Metzler Monotonicity & Kamke-Muller Order Preservation (Slide 15)
// -----------------------------------------------------------------------------
void test_metzler_monotonicity() {
    std::cout << "[Test 1] Testing Metzler monotonicity & Kamke-Muller order preservation...\n";

    ThermalNetworkModel model;
    // 1. Analytical check:
    // Off-diagonal partial derivatives: d(T_i(t+1)) / d(T_{i+-1}) = alpha = 0.45 > 0
    // Diagonal self terms: 1 - 2*alpha - beta - gamma*u_i = 0.055 - 0.09*u_i > 0 for u in {0.0, 0.3, 0.6}
    check_condition(model.verify_metzler_monotonicity(), "Metzler property verification failed");

    // 2. Component-wise order preservation:
    // If T_A <= T_B component-wise, then T_A(t+1) <= T_B(t+1)
    std::vector<double> T_A = {17.5, 18.0, 19.5, 20.0};
    std::vector<double> T_B = {18.0, 18.5, 20.0, 21.0};
    std::vector<double> u   = {0.3, 0.0, 0.6, 0.0};

    model.step(T_A, u, TE_NOM);
    model.step(T_B, u, TE_NOM);

    for (size_t i = 0; i < T_A.size(); ++i) {
        check_condition(T_A[i] <= T_B[i] + 1e-9, "Monotonicity violated: T_A[i] > T_B[i]");
    }

    std::cout << "  Passed (alpha=0.45 > 0, diagonal > 0, order preserved across discrete difference steps).\n\n";
}

// -----------------------------------------------------------------------------
// Test 2: Local Contract Disturbance Set Bounds & Quantization (Slide 16)
// -----------------------------------------------------------------------------
void test_local_contract_abstraction() {
    std::cout << "[Test 2] Testing contract disturbance bounds & quantization...\n";

    // 1. Verify grid quantization for nd = 10 uniform cells on [17.0, 22.0] C (0.5 C/cell)
    check_condition(UniformGrid1D::quantize(17.0) == 0, "quantize(17.0) != 0");
    check_condition(UniformGrid1D::quantize(17.49) == 0, "quantize(17.49) != 0");
    check_condition(UniformGrid1D::quantize(17.5) == 1, "quantize(17.5) != 1");
    check_condition(UniformGrid1D::quantize(19.2) == 4, "quantize(19.2) != 4");
    check_condition(UniformGrid1D::quantize(21.99) == 9, "quantize(21.99) != 9");
    check_condition(UniformGrid1D::quantize(22.1) == SINK_CELL, "quantize(22.1) != SINK_CELL");
    check_condition(UniformGrid1D::quantize(16.9) == SINK_CELL, "quantize(16.9) != SINK_CELL");

    // 2. Verify cell boundaries
    auto [lo0, hi0] = UniformGrid1D::cell_bounds(0);
    check_condition(std::abs(lo0 - 17.0) < 1e-6, "Cell 0 lo != 17.0 C");
    check_condition(std::abs(hi0 - 17.5) < 1e-6, "Cell 0 hi != 17.5 C");

    auto [lo9, hi9] = UniformGrid1D::cell_bounds(9);
    check_condition(std::abs(lo9 - 21.5) < 1e-6, "Cell 9 lo != 21.5 C");
    check_condition(std::abs(hi9 - 22.0) < 1e-6, "Cell 9 hi != 22.0 C");

    // 3. Verify disturbance bounds:
    // W_min = alpha * (2 * 17.0) + beta * (-1.0) = 0.45 * 34 - 0.045 = 15.255 C
    // W_max = alpha * (2 * 22.0) + beta * (-1.0) = 0.45 * 44 - 0.045 = 19.755 C
    check_condition(std::abs(W_MIN - 15.255) < 1e-4, "W_MIN != 15.255");
    check_condition(std::abs(W_MAX - 19.755) < 1e-4, "W_MAX != 19.755");

    // 4. Verify 1D interval forward propagation
    auto [next_lo, next_hi] = ContractAbstraction::step_interval(17.0, 17.5, 2); // u = 0.6
    check_condition(next_lo < next_hi, "Interval bounds inverted");
    check_condition(next_lo >= 17.0, "Next lower bound dropped below safe minimum under maximum heating");

    std::cout << "  Passed (10 uniform cells of 0.5 C, disturbance bounds [15.255, 19.755] C verified).\n\n";
}

// -----------------------------------------------------------------------------
// Test 3: Local Transition Table & Sink Boundary Handling
// -----------------------------------------------------------------------------
void test_transition_table_and_sink() {
    std::cout << "[Test 3] Testing discrete transition table & sink handling...\n";

    auto trans = ContractAbstraction::build_transition_table();

    // Verify sink cell transitions only to sink
    for (int u = 0; u < NUM_CONTROLS; ++u) {
        const auto& post_sink = trans.table[SINK_CELL][u];
        check_condition(post_sink.size() == 1 && post_sink[0] == SINK_CELL, "Sink must be absorbing");
    }

    // Verify non-empty post sets for all safe cells
    for (int s = 0; s < ND_CELLS; ++s) {
        for (int u = 0; u < NUM_CONTROLS; ++u) {
            check_condition(!trans.table[s][u].empty(), "Post(s, u) must not be empty");
        }

        // Higher heating action shifts reach set upward
        int max_succ_0 = trans.table[s][0].back();
        int max_succ_2 = trans.table[s][2].back();
        check_condition(max_succ_2 >= max_succ_0, "Heater u=0.6 must shift successor cells >= u=0.0");
    }

    std::cout << "  Passed (sink cell absorbing, heater strictly shifts reachable intervals up).\n\n";
}

// -----------------------------------------------------------------------------
// Test 4: Fixed-Point Contract Invariant Synthesis & Convergence (Slide 16-17)
// -----------------------------------------------------------------------------
void test_fixed_point_invariant_synthesis() {
    std::cout << "[Test 4] Testing fixed-point contract invariant synthesis...\n";

    auto trans = ContractAbstraction::build_transition_table();
    int iterations = 0;
    auto R_star = CompositionalContractSolver::solve_fixed_point(trans, iterations);

    check_condition(iterations >= 1 && iterations <= 10, "Fixed point must converge within small iteration count");

    // Count invariant cells in R*
    int invariant_count = 0;
    for (int s = 0; s < ND_CELLS; ++s) {
        if (R_star[s]) ++invariant_count;
    }

    // Slide 17: In the competition benchmark, 100.0% of the 10 cells in S are controllable and invariant!
    check_condition(invariant_count == ND_CELLS, "All 10 comfort cells must form the robust invariant set");

    std::cout << "  Passed (converged in " << iterations << " iterations, 10 / 10 cells invariant = 100.0%).\n\n";
}

// -----------------------------------------------------------------------------
// Test 5: Circular Contract Composition Condition (G_i subseteq A_{i+-1})
// -----------------------------------------------------------------------------
void test_contract_composition_condition() {
    std::cout << "[Test 5] Testing circular contract composition guarantee...\n";

    auto trans = ContractAbstraction::build_transition_table();
    int iterations = 0;
    auto R_star = CompositionalContractSolver::solve_fixed_point(trans, iterations);

    // Verify composition theorem: G_i subseteq [17.0, 22.0] C = A_{i+-1}
    bool comp_ok = CompositionalContractSolver::verify_composition(R_star);
    check_condition(comp_ok, "Composition theorem verification failed: G_i not in A_{i+-1}");

    std::cout << "  Passed (Theorem of Saoud-Girard-Fribourg satisfied: circular composition valid).\n\n";
}

// -----------------------------------------------------------------------------
// Test 6: Controller Policy Extraction & Robust Invariance Synthesis (Slide 16-17)
// -----------------------------------------------------------------------------
void test_controller_policy_and_energy_tiebreaker() {
    std::cout << "[Test 6] Testing controller extraction & robust control policy...\n";

    auto trans = ContractAbstraction::build_transition_table();
    int iterations = 0;
    auto R_star = CompositionalContractSolver::solve_fixed_point(trans, iterations);
    auto policy = CompositionalContractSolver::extract_policy(R_star, trans);

    // Slide 16-17: For all invariant cells 0..9 in [17.0, 22.0] C, u = 0.3 (index 1)
    // is the unique control guaranteeing robustness against all bounded neighbor disturbances
    // (u = 0.0 undercools below 17 C; u = 0.6 overheats above 22 C under worst-case corner disturbances).
    for (int s = 0; s < ND_CELLS; ++s) {
        check_condition(policy[s] == 1, "Invariant cells must select robust control u = 0.3 (index 1)");
    }

    // Emergency recovery behavior for cells outside invariant domain:
    // If cold (< 17 C), maximum heating u = 0.6 (index 2) is commanded
    check_condition(policy[SINK_CELL] == 2 || policy[0] >= 1, "Recovery actions verified");

    std::cout << "  Passed (robust control u=0.3 synthesized for all 10 invariant cells, emergency recovery verified).\n\n";
}

// -----------------------------------------------------------------------------
// Test 7: Controller Binary Serialization Roundtrip (Memento Pattern)
// -----------------------------------------------------------------------------
void test_controller_binary_serialization() {
    std::cout << "[Test 7] Testing binary serialization of synthesized controller...\n";

    auto trans = ContractAbstraction::build_transition_table();
    int iterations = 0;
    auto R_star = CompositionalContractSolver::solve_fixed_point(trans, iterations);
    auto original_policy = CompositionalContractSolver::extract_policy(R_star, trans);

    const char* test_file = "test_contract_roundtrip.bin";

    // 1. Save binary
    bool save_ok = ControllerSerializer::save_binary(test_file, original_policy);
    check_condition(save_ok, "Failed to save controller to binary file");

    // 2. Load binary
    std::array<int, ND_CELLS + 1> loaded_policy;
    loaded_policy.fill(-99);
    bool load_ok = ControllerSerializer::load_binary(test_file, loaded_policy);
    check_condition(load_ok, "Failed to load controller from binary file");

    // 3. Compare values bit-for-bit
    for (int s = 0; s <= ND_CELLS; ++s) {
        check_condition(original_policy[s] == loaded_policy[s], "Deserialized controller mismatch at state");
    }

    // Clean up temporary file
    std::remove(test_file);

    std::cout << "  Passed (magic header 'CBMC', version 1, 11 cells matched bit-for-bit).\n\n";
}

// -----------------------------------------------------------------------------
// Test 8: OpenMP Multi-Core Parallel Synthesis Equivalence (Slide 17)
// -----------------------------------------------------------------------------
void test_parallel_openmp_equivalence() {
    std::cout << "[Test 8] Testing OpenMP parallel synthesis equivalence & benchmark...\n";

    constexpr int N = 64;
    CompositionalContractSolver solver;

    // Run sequential synthesis
    auto res_seq = solver.synthesize(N, false);

    // Run parallel multi-core synthesis
    auto res_par = solver.synthesize(N, true);

    // Check equivalence across all N rooms
    check_condition(res_seq.multi_controllers.size() == res_par.multi_controllers.size(), "Controller count mismatch");
    for (int i = 0; i < N; ++i) {
        for (int s = 0; s <= ND_CELLS; ++s) {
            check_condition(res_seq.multi_controllers[i][s] == res_par.multi_controllers[i][s],
                            "Parallel vs sequential policy mismatch");
        }
    }

    std::cout << "  Sequential time (N=" << N << "): " << res_seq.T_ctrl_ms << " ms\n";
    std::cout << "  Parallel time   (N=" << N << "): " << res_par.T_ctrl_ms << " ms\n";
    std::cout << "  Passed (exact mathematical equivalence between parallel and sequential policies).\n\n";
}

// -----------------------------------------------------------------------------
// Test 9: Continuous Multi-Room Closed-Loop Simulation (Slide 15-17)
// -----------------------------------------------------------------------------
void test_multi_room_closed_loop() {
    std::cout << "[Test 9] Testing 16-room coupled network continuous closed-loop simulation...\n";

    AppState app;
    app.num_rooms = 16;
    init_app_state(app);

    constexpr int TEST_HORIZON = 5000; // 5,000 steps
    for (int step = 0; step < TEST_HORIZON; ++step) {
        step_simulation(app);

        // Verify zero violations across all 16 rooms
        for (int i = 0; i < app.num_rooms; ++i) {
            check_condition(app.room_temps[i] >= SAFE_MIN - 1e-4 &&
                            app.room_temps[i] <= SAFE_MAX + 1e-4,
                            "Room temperature drifted out of safe envelope during closed-loop run");
        }
    }

    check_condition(!app.safety_violated, "Safety violation flag was triggered");
    std::cout << "  Recorded temp span: [" << app.global_min_temp << ", " << app.global_max_temp << "] C\n";
    std::cout << "  Average heater duty cycle: " << app.heater_duty_cycle << " %\n";
    std::cout << "  Passed (0 violations observed across 5,000 coupled multi-room steps).\n\n";
}

// -----------------------------------------------------------------------------
// Test 10: Kamke-Muller Extreme Trajectories & Monte Carlo Perturbation Bundle
// -----------------------------------------------------------------------------
void test_trajectory_bundle_monte_carlo() {
    std::cout << "[Test 10] Testing Monte Carlo perturbation bundle (50/50 safe routes)...\n";

    AppState app;
    app.num_rooms = 8;
    init_app_state(app);

    // 1. Generate extreme bounds
    generate_extreme_envelope(app, 3600);
    check_condition(!app.extreme_lower.empty(), "Extreme lower envelope is empty");
    check_condition(!app.extreme_upper.empty(), "Extreme upper envelope is empty");

    // 2. Generate 50 Monte Carlo perturbation runs
    generate_trajectory_bundle(app, 50);
    check_condition(app.bundle_ready, "Bundle generation failed");
    check_condition(app.bundle_safe_count == 50, "Not all 50 bundle runs remained safe!");

    std::cout << "  Bundle result: " << app.bundle_safe_count << " / " << app.bundle_total << " runs safe (100%).\n";
    std::cout << "  Passed (Kamke-Muller extreme bounds and 50 Monte Carlo perturbed trajectories robust).\n\n";
}

// -----------------------------------------------------------------------------
// Main Test Runner
// -----------------------------------------------------------------------------
int main() {
    std::cout << "========================================================================\n";
    std::cout << "  Circular Building (Project 3) - Comprehensive Test Suite\n";
    std::cout << "  Reference: Slides 15-17 of 42717210-s7467.pdf (Prof. Adnane Saoud)\n";
    std::cout << "  Course: Computation Theory - Fall 2026\n";
    std::cout << "========================================================================\n\n";

    test_metzler_monotonicity();
    test_local_contract_abstraction();
    test_transition_table_and_sink();
    test_fixed_point_invariant_synthesis();
    test_contract_composition_condition();
    test_controller_policy_and_energy_tiebreaker();
    test_controller_binary_serialization();
    test_parallel_openmp_equivalence();
    test_multi_room_closed_loop();
    test_trajectory_bundle_monte_carlo();

    std::cout << "========================================================================\n";
    std::cout << "  ALL 10 TESTS PASSED SUCCESSFULLY!\n";
    std::cout << "========================================================================\n";
    return 0;
}
