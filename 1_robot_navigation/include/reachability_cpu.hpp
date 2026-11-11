#pragma once
#include "product_automaton.hpp"
#include "grid_quantizer.hpp"
#include <vector>
#include <cstdint>
#include <functional>

namespace solver {

inline constexpr int MAX_ITERATIONS = 100;

using ProgressCallback = std::function<void(int iter, float changes)>;

int solve_reachability_openmp(
    std::vector<float>&              V,
    std::vector<uint8_t>&            ctrl,
    const grid::RegionConfig&        regions,
    const grid::AbstractionTables&   abs_tables,
    ProgressCallback                 callback = nullptr);

bool save_controller_binary(const char*                 path,
                            const std::vector<float>&   V,
                            const std::vector<uint8_t>& ctrl);

bool load_controller_binary(const char*            path,
                            std::vector<float>&    V,
                            std::vector<uint8_t>&  ctrl);

} // namespace solver
