// =============================================================================
// reachability_cpu.hpp — Member 4
//
// OpenMP-parallelized backward reachability solver.
//
// The algorithm runs a fixed-point iteration over V[q][cell]:
//   For each product state (q, cell), find the control u that minimises
//   the worst-case cost over all successor cells reachable under u from cell,
//   given the current DFA state q.
//
// "Backward reachability" means we propagate cost from the goal (V=0) outward:
//   V_new(q, c) = min over u { max over successors s' of V_old(dfa(q,label(s')), s') }
//
// This is the standard minimax dynamic programming from the paper (Section 2.1).
//
// We iterate until no entry of V changes by more than CONVERGENCE_EPS,
// or until MAX_ITERATIONS is reached.
// =============================================================================
#pragma once
#include "product_automaton.hpp"
#include "grid_quantizer.hpp"
#include <vector>
#include <cstdint>
#include <atomic>
#include <string>
#include <functional>

namespace solver {

// ---------------------------------------------------------------------------
// Solver parameters
// ---------------------------------------------------------------------------

inline constexpr float  CONVERGENCE_EPS = 1e-4f; ///< stop when max ΔV < this
inline constexpr int    MAX_ITERATIONS  = 500;    ///< hard cap on iterations

// ---------------------------------------------------------------------------
// Progress callback
// ---------------------------------------------------------------------------

/// Called after each iteration with (iteration_number, max_delta_V).
/// The GUI uses this to update a progress bar.
using ProgressCallback = std::function<void(int iter, float delta)>;

// ---------------------------------------------------------------------------
// Main solver
// ---------------------------------------------------------------------------

/// Runs the backward reachability fixed-point loop on the CPU using OpenMP.
///
/// How it works:
///   1. For each DFA state q (1..5) in parallel:
///   2.   For each grid cell (c1, c2, c3):
///   3.     For each of the 15 controls u:
///   4.       Compute ALL successor cells using compute_reachable_box +
///              quantize; take the worst-case (max) V among them under the
///              DFA transition dfa(q, label(successor)).
///   5.     Keep the control that gives the MINIMUM worst-case cost.
///   6.   Update V and ctrl if V_new < V_old.
///   7. Repeat until convergence or MAX_ITERATIONS.
///
/// @param V         Value function (modified in place), length NTOTAL.
/// @param ctrl      Controller (modified in place), length NTOTAL.
/// @param regions   Region configuration (positions of R1-R4).
/// @param footprint Precomputed action footprints from grid_quantizer.
/// @param callback  Optional progress callback (may be nullptr).
/// @return          Number of iterations actually performed.
int solve_reachability_openmp(
    std::vector<float>&        V,
    std::vector<uint8_t>&      ctrl,
    const grid::RegionConfig&  regions,
    const grid::FootprintTable& footprint,
    ProgressCallback           callback = nullptr);

// ---------------------------------------------------------------------------
// Controller persistence
// ---------------------------------------------------------------------------

/// Saves V and ctrl arrays to a binary file so they can be reloaded
/// without re-solving (solving takes ~30 s on 24 cores).
///
/// File format (all little-endian):
///   8 bytes  — magic "CTRL1.00"
///   4 bytes  — NTOTAL (uint32_t)
///   NTOTAL*4 — V array (float32)
///   NTOTAL*1 — ctrl array (uint8_t)
///
/// @param path   Output file path, e.g. "controller.bin".
/// @param V      Value function.
/// @param ctrl   Controller.
/// @return       true on success.
bool save_controller_binary(const char*                  path,
                             const std::vector<float>&    V,
                             const std::vector<uint8_t>&  ctrl);

/// Loads V and ctrl from a file saved by save_controller_binary.
/// Returns false if the file is missing or the magic/size does not match.
bool load_controller_binary(const char*             path,
                             std::vector<float>&     V,
                             std::vector<uint8_t>&   ctrl);

} // namespace solver
