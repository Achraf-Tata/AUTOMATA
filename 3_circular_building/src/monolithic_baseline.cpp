#include "contract_solver.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

namespace circular::synthesis {

// Demonstrates the exponential state-space explosion of monolithic discretization
// from Slide 16 of 42717210-s7467.pdf:
// "State space: uniform partition, nd = 10 cells per dimension => nd^N = 10^N symbolic states.
//  Input space: each u_i quantized into nu = 3 values {0, 0.3, 0.6} => 3^N symbolic inputs.
//  => the curse of dimensionality is the whole point."
void MonolithicCurseEvaluator::print_curse_benchmark(int max_N) {
    std::cout << "--- Monolithic Grid State-Space Explosion (Slide 16: 10^N States, 3^N Inputs) ---\n";
    std::cout << std::setw(6) << "Rooms"
              << std::setw(18) << "Global States (10^N)"
              << std::setw(18) << "Inputs (3^N)"
              << std::setw(18) << "Memory (Bytes)"
              << std::setw(16) << "Feasibility" << "\n";
    std::cout << "------------------------------------------------------------------------------------\n";

    for (int n = 1; n <= max_N; ++n) {
        double states = std::pow(10.0, static_cast<double>(n));
        double inputs = std::pow(3.0, static_cast<double>(n));
        double mem_bytes = states * sizeof(uint8_t);

        std::cout << std::setw(6) << n
                  << std::setw(18) << std::scientific << std::setprecision(2) << states
                  << std::setw(18) << std::scientific << std::setprecision(2) << inputs;

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
            std::cout << std::setw(16) << "FEASIBLE\n";
        } else if (n == 5) {
            std::cout << std::setw(16) << "HIGH LATENCY\n";
        } else if (n <= 7) {
            std::cout << std::setw(16) << "OUT OF MEMORY\n";
        } else {
            std::cout << std::setw(16) << "IMPOSSIBLE\n";
        }
    }
    std::cout << "------------------------------------------------------------------------------------\n\n";
}

} // namespace circular::synthesis
