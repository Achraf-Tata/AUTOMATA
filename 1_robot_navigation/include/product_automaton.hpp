#pragma once

#include "grid_partition.hpp"
#include "grid_quantizer.hpp"
#include "dfa.hpp"
#include "transition_system.hpp"
#include <vector>
#include <unordered_set>
#include <cstddef>

namespace symbolic::automaton {

using namespace std;
using namespace domain;
using namespace grid;

// coordinates of mission regions: R1, R2, R3 (goal), R4 (obstacle)
struct ArenaRegions {
    BoundingBox waypoint_1{4.0, 5.0, 8.5, 9.5, -3.141592653589793, 3.141592653589793};
    BoundingBox waypoint_2{8.5, 9.5, 2.0, 3.0, -3.141592653589793, 3.141592653589793};
    BoundingBox goal      {2.0, 3.0, 0.5, 1.5, -3.141592653589793, 3.141592653589793};
    BoundingBox obstacle  {3.0, 7.0, 3.0, 7.0, -3.141592653589793, 3.141592653589793};
};

// combined state: (grid_cell, dfa_state)
struct ProductState {
    size_t s = 0;
    AutomatonState q = AutomatonState::Initial;
};

// Combined tracker: pairs each physical grid cell with where we are in the mission
class ProductAutomaton {
public:
    size_t num_cells = 0;
    static constexpr size_t DFA_STATE_COUNT = 5;

    DeterministicFiniteAutomaton dfa;
    ArenaRegions regions;

    // Region label (free, R1, R2, R3, R4) for each grid cell
    vector<ObservationSymbol> cell_labels;

    // Build the combined tracker and classify each grid cell into its region
    ProductAutomaton(
        size_t n_cells,
        const GridPartition& grid,
        const ArenaRegions& regs = ArenaRegions());

    // Total combinations: spatial_cells * 5 mission stages
    [[nodiscard]] size_t total_product_states() const noexcept;

    // Flattens (cell, mission_stage) into a single array index: (q - 1) * total_cells + cell
    [[nodiscard]] size_t encode_product_index(
        size_t s,
        AutomatonState q) const noexcept;

    // Unpacks an array index back into (cell, mission_stage)
    [[nodiscard]] ProductState decode_product_index(
        size_t p) const noexcept;

    // Get the precomputed region label for a spatial cell
    [[nodiscard]] ObservationSymbol get_cell_observation(
        size_t s) const noexcept;

    // Check which region a continuous position (x, y) is located inside
    [[nodiscard]] ObservationSymbol evaluate_point_observation(
        double x,
        double y) const noexcept;

    // Find all combinations where the mission is in the accepting goal state (q4)
    [[nodiscard]] unordered_set<size_t> compute_target_product_states() const;

    // Find all combinations where the mission failed (q5)
    [[nodiscard]] unordered_set<size_t> compute_sink_product_states() const;

private:
    // Classify all cells in the grid by region
    void precompute_cell_labels(const GridPartition& grid);
};

} // namespace symbolic::automaton

namespace symbolic {
    using automaton::ArenaRegions;
    using automaton::ProductState;
    using automaton::ProductAutomaton;
}

namespace automaton {

using namespace std;
using namespace grid;

inline constexpr int NDFA = 5;
inline constexpr int NCELLS = TOTAL_SPATIAL_CELLS;
inline constexpr int NTOTAL = NDFA * TOTAL_SPATIAL_CELLS;
inline constexpr float INF = 1e9f;

int dfa_transition(int current_state, int observation_label) noexcept;
vector<float> init_value_function();
vector<uint8_t> init_controller();

inline int product_index(int automaton_state, int cell_flat) noexcept {
    return (automaton_state - 1) * TOTAL_SPATIAL_CELLS + cell_flat;
}

} // namespace automaton

