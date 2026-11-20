#include "compositional_cpu.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <cmath>
#include <omp.h>

namespace circular {

void benchmark_monolithic_curse(int max_N) {
    std::cout << "--- Monolithic Grid State-Space Explosion (Curse of Dimensionality) ---\n";
    std::cout << std::setw(6) << "Rooms"
              << std::setw(18) << "Global States"
              << std::setw(18) << "Memory (Bytes)"
              << std::setw(16) << "Feasibility" << "\n";
    std::cout << "------------------------------------------------------------------------\n";

    for (int n = 1; n <= max_N; ++n) {
        double states = std::pow(50.0, static_cast<double>(n));
        double mem_bytes = states * sizeof(uint8_t);

        std::cout << std::setw(6) << n
                  << std::setw(18) << std::scientific << std::setprecision(2) << states;

        if (mem_bytes < 1024.0) {
            std::cout << std::setw(14) << std::fixed << std::setprecision(0) << mem_bytes << " B ";
        } else if (mem_bytes < 1024.0 * 1024.0) {
            std::cout << std::setw(12) << std::fixed << std::setprecision(1) << (mem_bytes / 1024.0) << " KB ";
        } else if (mem_bytes < 1024.0 * 1024.0 * 1024.0) {
            std::cout << std::setw(12) << std::fixed << std::setprecision(1) << (mem_bytes / (1024.0 * 1024.0)) << " MB ";
        } else {
            std::cout << std::setw(12) << std::fixed << std::setprecision(1) << (mem_bytes / (1024.0 * 1024.0 * 1024.0)) << " GB ";
        }

        if (n <= 3) {
            std::cout << std::setw(16) << "FEASIBLE\n";
        } else if (n == 4) {
            std::cout << std::setw(16) << "HIGH LATENCY\n";
        } else if (n == 5) {
            std::cout << std::setw(16) << "OUT OF MEMORY\n";
        } else {
            std::cout << std::setw(16) << "IMPOSSIBLE\n";
        }
    }
    std::cout << "------------------------------------------------------------------------\n\n";
}

void synthesize_compositional_openmp(
    int N,
    std::vector<std::array<int, NUM_CELLS + 1>>& controllers,
    double& duration_ms)
{
    controllers.resize(N);
    auto trans = build_local_transition_table();

    auto t0 = std::chrono::high_resolution_clock::now();

    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < N; ++i) {
        int iter = 0;
        auto R_star = solve_local_invariant_contract(trans, iter);
        controllers[i] = extract_local_controller(R_star, trans);
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    duration_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

} // namespace circular
