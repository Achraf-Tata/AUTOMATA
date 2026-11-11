#include "circular_network.hpp"
#include "contract_abstraction.hpp"
#include "contract_solver.hpp"
#include "compositional_cpu.hpp"

#include <iostream>
#include <vector>
#include <cstdlib>
#include <cmath>

static void check_condition(bool condition, const char* msg) {
    if (!condition) {
        std::cerr << "Assertion failed: " << msg << "\n";
        std::exit(1);
    }
}

void test_metzler_monotonicity() {
    std::cout << "[Test 1] Testing Metzler monotonicity & trajectory order preservation...\n";

    check_condition(circular::verify_metzler_property(), "Metzler property check failed");

    // Order preservation test
    std::vector<double> T_A = {19.0, 19.5, 20.0, 20.5};
    std::vector<double> T_B = {19.2, 19.8, 20.1, 20.9};
    std::vector<int> u = {1, 0, 1, 0};

    circular::step_network_euler(T_A, u, 8.0, 1.0);
    circular::step_network_euler(T_B, u, 8.0, 1.0);

    for (size_t i = 0; i < T_A.size(); ++i) {
        check_condition(T_A[i] <= T_B[i], "Monotonicity violated: T_A[i] > T_B[i]");
    }
    std::cout << "  Passed.\n";
}

void test_local_contract_abstraction() {
    std::cout << "[Test 2] Testing contract disturbance bounds & quantization...\n";

    check_condition(circular::quantize(15.0) == 0, "quantize(15.0) != 0");
    check_condition(circular::quantize(19.0) == 20, "quantize(19.0) != 20");
    check_condition(circular::quantize(21.9) == 34, "quantize(21.9) != 34");

    auto [lo, hi] = circular::step_local_interval(19.0, 19.2, 1);
    check_condition(lo < hi, "Local interval lower bound must be less than upper bound");

    auto post0 = circular::compute_local_post(20, 0);
    auto post1 = circular::compute_local_post(20, 1);
    check_condition(!post0.empty(), "Post(20, 0) is empty");
    check_condition(!post1.empty(), "Post(20, 1) is empty");
    check_condition(post1[0] > post0[0], "Heater ON must shift reachable cells up");

    std::cout << "  Passed.\n";
}

void test_contract_synthesis_and_composition() {
    std::cout << "[Test 3] Testing contract synthesis and circular composition...\n";

    auto trans = circular::build_local_transition_table();
    int iter = 0;
    auto R_star = circular::solve_local_invariant_contract(trans, iter);

    int count = 0;
    for (int s = 0; s <= circular::NUM_CELLS; ++s) {
        if (R_star[s]) ++count;
    }

    check_condition(count > 0, "Invariant set must not be empty");
    check_condition(circular::verify_contract_composition(R_star), "Contract composition check failed");

    std::cout << "  Local contract converged with " << count << " invariant cells.\n";
    std::cout << "  Passed.\n";
}

void test_multi_room_closed_loop() {
    std::cout << "[Test 4] Testing 8-room closed-loop network simulation...\n";

    auto trans = circular::build_local_transition_table();
    int iter = 0;
    auto R_star = circular::solve_local_invariant_contract(trans, iter);
    auto ctrl = circular::extract_local_controller(R_star, trans);

    constexpr int N = 8;
    std::vector<double> T(N, 20.0);
    std::vector<int> u(N, 0);

    for (int step = 0; step < 2000; ++step) {
        double Te = (step % 2 == 0) ? circular::TE_MIN : circular::TE_MAX;

        for (int i = 0; i < N; ++i) {
            int cell = circular::quantize(T[i]);
            u[i] = ctrl[cell];
            check_condition(T[i] >= circular::SAFE_MIN && T[i] <= circular::SAFE_MAX,
                            "Room temperature out of safe bounds during closed-loop step");
        }

        circular::step_network_euler(T, u, Te, 1.0);
    }

    std::cout << "  Zero violations observed across 2,000 multi-room coupled steps.\n";
    std::cout << "  Passed.\n";
}

int main() {
    std::cout << "Running Circular Building Test Suite...\n\n";
    test_metzler_monotonicity();
    test_local_contract_abstraction();
    test_contract_synthesis_and_composition();
    test_multi_room_closed_loop();
    std::cout << "\nAll Circular Building tests passed successfully.\n";
    return 0;
}
