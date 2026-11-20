#pragma once
#include "contract_solver.hpp"
#include <vector>
#include <array>

namespace circular {

// Demonstrates the exponential curse of dimensionality for monolithic discretization
void benchmark_monolithic_curse(int max_N);

// Compositional synthesis of N room controllers in parallel via OpenMP
void synthesize_compositional_openmp(
    int N,
    std::vector<std::array<int, NUM_CELLS + 1>>& controllers,
    double& duration_ms);

} // namespace circular
