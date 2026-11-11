// =============================================================================
// reachability_cpu.hpp — Member 4
//
// OpenMP-parallelized backward reachability synthesis solver.
// Implementation based on:
// "Approches symboliques pour le contrôle des systèmes non linéaires"
// by Antoine Girard, Pierre-Jean Meyer, Adnane Saoud (2024).
// =============================================================================
#pragma once
#include "product_automaton.hpp"
#include "grid_quantizer.hpp"
#include <vector>
#include <cstdint>
#include <functional>

namespace solver {

inline constexpr int MAX_ITERATIONS = 100;

using ProgressCallback = std::function<void(int iter, float changes)>;

/// Runs the backward reachability fixed-point loop on CPU with OpenMP.
int solve_reachability_openmp(
    std::vector<float>&              V,
    std::vector<uint8_t>&            ctrl,
    const grid::RegionConfig&        regions,
    const grid::AbstractionTables&   abs_tables,
    ProgressCallback                 callback = nullptr);

/// Controller file I/O
bool save_controller_binary(const char*                 path,
                            const std::vector<float>&   V,
                            const std::vector<uint8_t>& ctrl);

bool load_controller_binary(const char*            path,
                            std::vector<float>&    V,
                            std::vector<uint8_t>&  ctrl);

} // namespace solver
