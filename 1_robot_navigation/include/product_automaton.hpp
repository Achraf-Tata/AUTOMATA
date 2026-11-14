#pragma once

#include "grid_partition.hpp"
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
    size_t spatial_cell_index = 0;
    AutomatonState automaton_state = AutomatonState::Initial;
};

// product automaton: combines physical robot cells with high-level DFA states
class ProductAutomaton {
public:
    size_t spatial_cell_count = 0;
    static constexpr size_t DFA_STATE_COUNT = 5;

    DeterministicFiniteAutomaton specification_dfa;
    ArenaRegions arena_regions;

    // region label (free, R1, R2, R3, R4) for each grid cell
    vector<ObservationSymbol> cell_observation_labels;

    // build product automaton and classify each grid cell into its region
    ProductAutomaton(
        size_t total_spatial_cells,
        const GridPartition& partition,
        const ArenaRegions& regions = ArenaRegions());

    // total states: spatial_cells * 5 DFA states
    [[nodiscard]] size_t total_product_states() const noexcept;

    // pack (cell, dfa_state) into a 1D index: (q - 1) * total_cells + cell
    [[nodiscard]] size_t encode_product_index(
        size_t spatial_cell_index,
        AutomatonState automaton_state) const noexcept;

    // unpack a 1D product index back into (cell, dfa_state)
    [[nodiscard]] ProductState decode_product_index(
        size_t product_index) const noexcept;

    // read region label for a cell
    [[nodiscard]] ObservationSymbol get_cell_observation(
        size_t spatial_cell_index) const noexcept;

    // check which region contains point (x, y)
    [[nodiscard]] ObservationSymbol evaluate_point_observation(
        double x_coordinate,
        double y_coordinate) const noexcept;

    // find all product states that have reached the goal (q4)
    [[nodiscard]] unordered_set<size_t> compute_target_product_states() const;

    // find all product states that are in the sink trap (q5)
    [[nodiscard]] unordered_set<size_t> compute_sink_product_states() const;

private:
    // precompute region labels for all grid cells
    void precompute_cell_labels(const GridPartition& partition);
};

} // namespace symbolic::automaton

namespace symbolic {
    using automaton::ArenaRegions;
    using automaton::ProductState;
    using automaton::ProductAutomaton;
}
