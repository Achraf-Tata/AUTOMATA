#include "circular_network.hpp"
#include "contract_abstraction.hpp"
#include "contract_solver.hpp"
#include "compositional_cpu.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main() {
    std::cout << "========================================================\n";
    std::cout << "  Project 3: Circular Building (Assume-Guarantee Contracts)\n";
    std::cout << "========================================================\n\n";

    // 1. Verify Metzler Monotonicity
    bool metzler_ok = circular::verify_metzler_property();
    std::cout << "[Step 1] Metzler monotonicity (Kamke-Muller cooperative coupling): "
              << (metzler_ok ? "VERIFIED (BETA = 0.02 > 0)" : "FAILED") << "\n\n";
    if (!metzler_ok) return 1;

    // 2. Monolithic grid curse of dimensionality demonstration
    circular::benchmark_monolithic_curse(7);

    // 3. Assume-Guarantee Contract Synthesis for Local Room
    std::cout << "[Step 2] Synthesizing Assume-Guarantee local contract...\n";
    auto trans = circular::build_local_transition_table();
    int iterations = 0;
    auto t0 = std::chrono::high_resolution_clock::now();
    auto R_star = circular::solve_local_invariant_contract(trans, iterations);
    auto t1 = std::chrono::high_resolution_clock::now();
    double synth_us = std::chrono::duration<double, std::micro>(t1 - t0).count();

    int invariant_cells = 0;
    for (int s = 0; s <= circular::NUM_CELLS; ++s) {
        if (R_star[s]) ++invariant_cells;
    }

    std::cout << "         Local invariant contract R* converged in " << iterations << " iterations.\n";
    std::cout << "         Synthesis time: " << std::fixed << std::setprecision(2) << synth_us << " us\n";
    std::cout << "         Invariant cells: " << invariant_cells << " / 15 in safe range [19.0, 22.0] C\n";

    bool comp_ok = circular::verify_contract_composition(R_star);
    std::cout << "         Contract composition condition G_i subseteq A_{i+-1}: "
              << (comp_ok ? "VERIFIED (Circular contract holds formally)" : "FAILED") << "\n\n";
    if (!comp_ok) return 1;

    auto local_ctrl = circular::extract_local_controller(R_star, trans);

    // 4. Scalability Benchmark across N rooms
    std::cout << "[Step 3] Parallel Scalability Benchmark via OpenMP:\n";
    std::vector<int> room_counts = {4, 8, 16, 32, 64, 100, 500, 1000};
    std::cout << std::setw(10) << "Rooms (N)"
              << std::setw(20) << "Total Grid States"
              << std::setw(20) << "Synthesis Time"
              << std::setw(20) << "Time per Room" << "\n";
    std::cout << "----------------------------------------------------------------------\n";

    for (int n : room_counts) {
        std::vector<std::array<int, circular::NUM_CELLS + 1>> controllers;
        double duration_ms = 0.0;
        circular::synthesize_compositional_openmp(n, controllers, duration_ms);

        double us_per_room = (duration_ms * 1000.0) / n;
        std::cout << std::setw(10) << n
                  << std::setw(20) << (n * 50)
                  << std::setw(17) << std::fixed << std::setprecision(2) << duration_ms << " ms"
                  << std::setw(17) << std::fixed << std::setprecision(2) << us_per_room << " us\n";
    }
    std::cout << "----------------------------------------------------------------------\n\n";

    // 5. 24-Hour continuous closed-loop simulation of N=16 coupled building
    constexpr int SIM_ROOMS = 16;
    constexpr int DURATION_SEC = 86400;
    std::cout << "[Step 4] Simulating 16-room coupled network for 24 hours (86,400 s)...\n";

    std::vector<double> T(SIM_ROOMS);
    for (int i = 0; i < SIM_ROOMS; ++i) {
        T[i] = 19.5 + 0.1 * (i % 15); // Staggered initial temperatures within safe set
    }

    double global_min_T = 25.0;
    double global_max_T = 15.0;
    long long total_heater_on = 0;
    bool any_violation = false;

    std::ofstream csv("circular_building_24h.csv");
    csv << "time_sec,outside_temp";
    for (int i = 0; i < SIM_ROOMS; ++i) {
        csv << ",room_" << i << "_temp,room_" << i << "_heater";
    }
    csv << "\n";

    std::vector<int> u(SIM_ROOMS, 0);

    for (int t = 0; t < DURATION_SEC; ++t) {
        double Te = 8.5 + 3.5 * std::sin(2.0 * M_PI * static_cast<double>(t) / 86400.0);

        for (int i = 0; i < SIM_ROOMS; ++i) {
            int cell = circular::quantize(T[i]);
            u[i] = (cell >= 0 && cell <= circular::NUM_CELLS) ? local_ctrl[cell] : 1;
            if (u[i] == 1) ++total_heater_on;

            if (T[i] < global_min_T) global_min_T = T[i];
            if (T[i] > global_max_T) global_max_T = T[i];

            if (T[i] < circular::SAFE_MIN || T[i] > circular::SAFE_MAX) {
                any_violation = true;
                std::cerr << "Violation in Room " << i << " at t = " << t << " s: T = " << T[i] << " C\n";
                break;
            }
        }
        if (any_violation) break;

        if (t % 60 == 0) {
            csv << t << "," << std::fixed << std::setprecision(3) << Te;
            for (int i = 0; i < SIM_ROOMS; ++i) {
                csv << "," << T[i] << "," << u[i];
            }
            csv << "\n";
        }

        circular::step_network_euler(T, u, Te, 1.0);
    }
    csv.close();

    std::cout << "         Global min room temp: " << std::fixed << std::setprecision(3) << global_min_T << " C\n";
    std::cout << "         Global max room temp: " << std::fixed << std::setprecision(3) << global_max_T << " C\n";
    double avg_duty = (100.0 * total_heater_on) / (static_cast<double>(SIM_ROOMS) * DURATION_SEC);
    std::cout << "         Average heater duty:  " << std::fixed << std::setprecision(1) << avg_duty << " %\n";
    std::cout << "         Network invariance:   " << (!any_violation ? "STRICTLY SATISFIED (0 violations across all 16 rooms)" : "VIOLATED") << "\n";
    std::cout << "         Logged trajectory to: circular_building_24h.csv\n\n";

    if (any_violation) return 1;
    std::cout << "All Circular Building verification checks passed successfully.\n";
    return 0;
}
