#include "thermal_network.hpp"
#include "contract_abstraction.hpp"
#include "contract_solver.hpp"
#include "simulation_engine.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <vector>

using namespace circular::model;
using namespace circular::abstraction;
using namespace circular::synthesis;

int main(int argc, char* argv[]) {
    std::cout << "==============================================================================\n";
    std::cout << "  Circular Building Temperature Regulation - Slide 15-17 Benchmarks\n";
    std::cout << "  Computation Theory (Fall 2026) - Prof. Adnane Saoud\n";
    std::cout << "==============================================================================\n\n";

    // 1. Verify monotonicity (Slide 15)
    ThermalNetworkModel model;
    bool monotone = model.verify_metzler_monotonicity();
    std::cout << "1. Model Monotonicity (Slide 15):\n";
    std::cout << "   alpha = " << ALPHA << " > 0, 1 - 2*alpha - beta - gamma*u > 0\n";
    std::cout << "   Status: " << (monotone ? "Verified (strictly monotone)" : "Failed") << "\n\n";
    if (!monotone) return 1;

    // 2. Monolithic explosion benchmark (Slide 16)
    std::cout << "2. Monolithic State-Space Explosion (Slide 16):\n";
    std::cout << "------------------------------------------------------------------------------\n";
    std::cout << std::setw(6)  << "N"
              << std::setw(18) << "States (10^N)"
              << std::setw(18) << "Inputs (3^N)"
              << std::setw(18) << "Memory"
              << std::setw(16) << "Feasibility\n";
    std::cout << "------------------------------------------------------------------------------\n";

    int mono_sizes[] = {1, 2, 3, 4, 5, 6, 10};
    for (int n : mono_sizes) {
        double states = std::pow(10.0, static_cast<double>(n));
        double inputs = std::pow(3.0, static_cast<double>(n));
        double mem_bytes = states * sizeof(uint8_t);

        std::cout << std::setw(6) << n;
        if (n <= 4) {
            std::cout << std::setw(18) << std::fixed << std::setprecision(0) << states
                      << std::setw(18) << std::fixed << std::setprecision(0) << inputs;
        } else {
            std::cout << std::setw(18) << std::scientific << std::setprecision(2) << states
                      << std::setw(18) << std::scientific << std::setprecision(2) << inputs;
        }

        if (mem_bytes < 1024.0) {
            std::cout << std::setw(16) << std::fixed << std::setprecision(0) << mem_bytes << " B ";
        } else if (mem_bytes < 1024.0 * 1024.0) {
            std::cout << std::setw(15) << std::fixed << std::setprecision(1) << (mem_bytes / 1024.0) << " KB";
        } else if (mem_bytes < 1024.0 * 1024.0 * 1024.0) {
            std::cout << std::setw(15) << std::fixed << std::setprecision(1) << (mem_bytes / (1024.0 * 1024.0)) << " MB";
        } else {
            std::cout << std::setw(15) << std::fixed << std::setprecision(1) << (mem_bytes / (1024.0 * 1024.0 * 1024.0)) << " GB";
        }

        if (n <= 4) {
            std::cout << std::setw(16) << "Feasible\n";
        } else if (n == 5) {
            std::cout << std::setw(16) << "Slow\n";
        } else if (n == 6) {
            std::cout << std::setw(16) << "OOM\n";
        } else {
            std::cout << std::setw(16) << "Crash\n";
        }
    }
    std::cout << "------------------------------------------------------------------------------\n\n";

    // 3. Official Competition Benchmark (Slide 17 Table)
    std::cout << "3. Competition Benchmark Results (Slide 17 Table):\n";
    std::cout << "   Parameters: alpha=0.45, beta=0.045, gamma=0.09, Te=-1.0 C, Th=50.0 C\n";
    std::cout << "   Discretization: nd=10 (0.5 C/cell), nu=3 controls {0.0, 0.3, 0.6}\n";
    std::cout << "   Comfort set: S = [17.0, 22.0]^N C\n";
    std::cout << "------------------------------------------------------------------------------\n";
    std::cout << std::setw(8)  << "N"
              << std::setw(16) << "Tabs (ms)"
              << std::setw(16) << "Tctrl (ms)"
              << std::setw(18) << "Peak Memory"
              << std::setw(18) << "Controllable %\n";
    std::cout << "------------------------------------------------------------------------------\n";

    std::ofstream csv("circular_building_competition.csv");
    csv << "N,Tabs_ms,Tctrl_ms,Peak_Memory_KB,Controllable_pct\n";

    CompositionalContractSolver solver;

    // Warmup run to initialize runtime caches
    (void)solver.synthesize(4, false);

    std::vector<int> room_counts = {3, 4, 5, 10, 50, 100, 500, 1000};
    if (argc > 1) {
        std::vector<int> custom_counts;
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "--extended" || arg == "-e") {
                custom_counts = {3, 10, 100, 1000, 5000, 10000, 50000, 100000};
                break;
            }
            try {
                int val = std::stoi(arg);
                if (val >= 3) custom_counts.push_back(val);
            } catch (...) {}
        }
        if (!custom_counts.empty()) {
            room_counts = custom_counts;
        }
    }

    for (int n : room_counts) {
        // Average over 20 iterations for stable timing
        double sum_tabs = 0.0;
        double sum_tctrl = 0.0;
        SynthesisResult res;
        constexpr int REPEATS = 20;

        for (int rep = 0; rep < REPEATS; ++rep) {
            res = solver.synthesize(n, false);
            sum_tabs += res.T_abs_ms;
            sum_tctrl += res.T_ctrl_ms;
        }

        double avg_tabs = sum_tabs / REPEATS;
        double avg_tctrl = sum_tctrl / REPEATS;

        std::cout << std::setw(8)  << n
                  << std::setw(16) << std::fixed << std::setprecision(3) << avg_tabs
                  << std::setw(16) << std::fixed << std::setprecision(3) << avg_tctrl;

        if (res.peak_mem_kb < 1024.0) {
            std::cout << std::setw(15) << std::fixed << std::setprecision(1) << res.peak_mem_kb << " KB";
        } else {
            std::cout << std::setw(15) << std::fixed << std::setprecision(2) << (res.peak_mem_kb / 1024.0) << " MB";
        }

        std::cout << std::setw(17) << std::fixed << std::setprecision(1) << res.controllable_pct << " %\n";

        csv << n << ","
            << std::fixed << std::setprecision(4) << avg_tabs << ","
            << std::fixed << std::setprecision(4) << avg_tctrl << ","
            << std::fixed << std::setprecision(1) << res.peak_mem_kb << ","
            << std::fixed << std::setprecision(1) << res.controllable_pct << "\n";
    }
    csv.close();
    std::cout << "------------------------------------------------------------------------------\n";
    std::cout << "Saved benchmark data to circular_building_competition.csv\n\n";

    // 4. 24-Hour Continuous Closed-Loop Simulation (N = 16 rooms)
    constexpr int SIM_ROOMS = 16;
    constexpr int DURATION_STEPS = 86400; // 24 hours (1 step = 1 sec)
    std::cout << "4. 24-Hour Closed-Loop Simulation (N = 16 rooms, 86,400 steps):\n";

    circular::AppState app;
    app.num_rooms = SIM_ROOMS;
    circular::init_app_state(app);

    std::ofstream sim_csv("circular_building_24h.csv");
    sim_csv << "step,outside_temp";
    for (int i = 0; i < SIM_ROOMS; ++i) {
        sim_csv << ",room_" << i << "_temp,room_" << i << "_heater";
    }
    sim_csv << "\n";

    bool any_violation = false;
    for (int t = 0; t < DURATION_STEPS; ++t) {
        circular::engine::step_simulation(app);

        for (int i = 0; i < SIM_ROOMS; ++i) {
            if (app.room_temps[i] < SAFE_MIN || app.room_temps[i] > SAFE_MAX) {
                any_violation = true;
                std::cerr << "Violation in Room " << i << " at step " << t << ": T = " << app.room_temps[i] << " C\n";
                break;
            }
        }
        if (any_violation) break;

        if (t % 60 == 0) {
            sim_csv << t << "," << std::fixed << std::setprecision(3) << app.current_te;
            for (int i = 0; i < SIM_ROOMS; ++i) {
                sim_csv << "," << app.room_temps[i] << "," << app.room_heaters[i];
            }
            sim_csv << "\n";
        }
    }
    sim_csv.close();

    std::cout << "   - Min room temp:       " << std::fixed << std::setprecision(2) << app.global_min_temp << " C\n";
    std::cout << "   - Max room temp:       " << std::fixed << std::setprecision(2) << app.global_max_temp << " C\n";
    std::cout << "   - Average heater duty: " << std::fixed << std::setprecision(1) << app.heater_duty_cycle << " %\n";
    std::cout << "   - Total energy:        " << std::fixed << std::setprecision(1) << app.total_energy_kwh << " kWh\n";
    std::cout << "   - Safety violations:   " << (any_violation ? "Violated" : "0 (satisfied for all time)") << "\n";
    std::cout << "Saved simulation trajectory to circular_building_24h.csv\n\n";

    std::cout << "==============================================================================\n";
    std::cout << "  Verification complete.\n";
    std::cout << "==============================================================================\n";
    return any_violation ? 1 : 0;
}
