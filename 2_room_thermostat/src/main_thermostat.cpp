#include "thermostat_model.hpp"
#include "interval_abstraction.hpp"
#include "safety_solver.hpp"
#include "bitset_solver.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main() {
    std::cout << "========================================================\n";
    std::cout << "  Project 2: Room Thermostat Safety Invariance\n";
    std::cout << "========================================================\n\n";

    // 1. Verify continuous boundary conditions
    bool boundary_ok = thermostat::verify_continuous_boundary_invariance();
    std::cout << "[Step 1] Continuous boundary invariance: "
              << (boundary_ok ? "VERIFIED (d/dt T > 0 at 19 C, d/dt T < 0 at 22 C)" : "FAILED")
              << "\n";
    if (!boundary_ok) {
        std::cerr << "Fatal: continuous boundary invariance violated.\n";
        return 1;
    }

    // 2. Build abstraction transition table
    std::cout << "[Step 2] Building 50-interval transition abstraction...\n";
    auto trans = thermostat::build_transition_table();
    auto btable = thermostat::bitset::build_bitset_table();

    // 3. Solve maximal robustly invariant set (Algorithm 1)
    int iter_std = 0;
    auto t0 = std::chrono::high_resolution_clock::now();
    auto R_star_vec = thermostat::solve_maximal_invariant_set(trans, iter_std);
    auto t1 = std::chrono::high_resolution_clock::now();
    double time_std_us = std::chrono::duration<double, std::micro>(t1 - t0).count();

    int iter_bit = 0;
    auto tb0 = std::chrono::high_resolution_clock::now();
    uint64_t R_star_mask = thermostat::bitset::solve_fixed_point_bitset(btable, iter_bit);
    auto tb1 = std::chrono::high_resolution_clock::now();
    double time_bit_us = std::chrono::duration<double, std::micro>(tb1 - tb0).count();

    int r_size = 0;
    for (int s = 0; s <= thermostat::NUM_CELLS; ++s) {
        if (R_star_vec[s]) ++r_size;
    }

    std::cout << "[Step 3] Maximal invariant set R* converged in " << iter_std << " iterations.\n";
    std::cout << "         Standard solver time: " << std::fixed << std::setprecision(2) << time_std_us << " us\n";
    std::cout << "         Bitset solver time:   " << std::fixed << std::setprecision(2) << time_bit_us << " us\n";
    std::cout << "         Invariant cells:      " << r_size << " / 15 safe cells in [19.0, 22.0] C\n";

    // Verify consistency between solvers
    for (int s = 0; s <= thermostat::NUM_CELLS; ++s) {
        bool bit_set = (R_star_mask & (1ULL << s)) != 0;
        if (R_star_vec[s] != bit_set) {
            std::cerr << "Fatal: solver mismatch at cell " << s << "\n";
            return 1;
        }
    }

    // 4. Extract control policy with energy-saving tie-breaker
    auto controller = thermostat::extract_controller(R_star_vec, trans);
    std::cout << "[Step 4] Synthesized feedback control law u = C(T):\n";
    for (int s = thermostat::SAFE_CELL_START; s <= thermostat::SAFE_CELL_END; ++s) {
        auto [lo, hi] = thermostat::cell_bounds(s);
        std::cout << "         Cell " << std::setw(2) << s
                  << " [" << std::fixed << std::setprecision(1) << lo << ", " << hi << "] C -> "
                  << (controller[s] == 0 ? "Heater OFF (0)" : "Heater ON  (1)") << "\n";
    }

    // 5. 24-Hour continuous closed-loop simulation under diurnal weather
    std::cout << "\n[Step 5] Simulating 24 hours (86,400 seconds) under diurnal weather cycle...\n";
    constexpr int DURATION_SEC = 86400;
    double T = 20.5;
    double min_T = T, max_T = T;
    int heater_on_count = 0;
    bool violation = false;

    std::ofstream csv("room_thermostat_24h.csv");
    csv << "time_sec,outside_temp,room_temp,heater_control\n";

    for (int t = 0; t < DURATION_SEC; ++t) {
        double Te = 8.5 + 3.5 * std::sin(2.0 * M_PI * static_cast<double>(t) / 86400.0);

        int cell = thermostat::quantize(T);
        int u = (cell >= 0 && cell <= thermostat::NUM_CELLS) ? controller[cell] : 1;
        if (u == 1) ++heater_on_count;

        if (t % 60 == 0) {
            csv << t << "," << std::fixed << std::setprecision(3) << Te << "," << T << "," << u << "\n";
        }

        if (T < min_T) min_T = T;
        if (T > max_T) max_T = T;

        if (T < thermostat::SAFE_MIN || T > thermostat::SAFE_MAX) {
            violation = true;
            std::cerr << "Violation at t = " << t << " s: T = " << T << " C\n";
            break;
        }

        T = thermostat::step_euler(T, u, Te, 1.0);
    }
    csv.close();

    std::cout << "         Minimum room temp: " << std::fixed << std::setprecision(3) << min_T << " C\n";
    std::cout << "         Maximum room temp: " << std::fixed << std::setprecision(3) << max_T << " C\n";
    std::cout << "         Heater duty cycle: " << std::fixed << std::setprecision(1)
              << (100.0 * heater_on_count / DURATION_SEC) << " %\n";
    std::cout << "         Safety invariant:  " << (!violation ? "STRICTLY SATISFIED (0 violations)" : "VIOLATED") << "\n";
    std::cout << "         Logged trajectory to: room_thermostat_24h.csv\n\n";

    if (violation) {
        return 1;
    }
    std::cout << "All verification checks passed successfully.\n";
    return 0;
}
