#pragma once

#include "grid_partition.hpp"
#include "dfa.hpp"
#include "transition_system.hpp"
#include <vector>
#include <unordered_set>
#include <cstddef>

namespace symbolic::automaton {

struct ArenaRegions {
    domain::BoundingBox waypoint_1{4.0, 5.0, 8.5, 9.5, -3.141592653589793, 3.141592653589793};
    domain::BoundingBox waypoint_2{8.5, 9.5, 2.0, 3.0, -3.141592653589793, 3.141592653589793};
    domain::BoundingBox goal      {2.0, 3.0, 0.5, 1.5, -3.141592653589793, 3.141592653589793};
    domain::BoundingBox obstacle  {3.0, 7.0, 3.0, 7.0, -3.141592653589793, 3.141592653589793};
};

struct ProductState {
    std::size_t spatial_cell_index = 0;
    AutomatonState automaton_state = AutomatonState::Initial;
};

class ProductAutomaton {
public:
    std::size_t spatial_cell_count = 0;
    static constexpr std::size_t DFA_STATE_COUNT = 5;

    DeterministicFiniteAutomaton specification_dfa;
    ArenaRegions arena_regions;

    // Precomputed spatial cell observation labels
    std::vector<ObservationSymbol> cell_observation_labels;

    // Construct product automaton and precompute cell labels from spatial partition
    ProductAutomaton(
        std::size_t total_spatial_cells,
        const grid::GridPartition& partition,
        const ArenaRegions& regions = ArenaRegions());

    // Returns total product states: |S| x |Q|
    [[nodiscard]] std::size_t total_product_states() const noexcept;

    // Encodes spatial cell index and DFA state into a flat 1D product state index
    [[nodiscard]] std::size_t encode_product_index(
        std::size_t spatial_cell_index,
        AutomatonState automaton_state) const noexcept;

    // Decodes a flat 1D product state index into spatial cell index and DFA state
    [[nodiscard]] ProductState decode_product_index(
        std::size_t product_index) const noexcept;

    // Retrieves precomputed observation symbol for a spatial cell
    [[nodiscard]] ObservationSymbol get_cell_observation(
        std::size_t spatial_cell_index) const noexcept;

    // Determines observation symbol for continuous coordinates (x, y)
    [[nodiscard]] ObservationSymbol evaluate_point_observation(
        double x_coordinate,
        double y_coordinate) const noexcept;

    // Returns set of all product states whose DFA component is accepting
    [[nodiscard]] std::unordered_set<std::size_t> compute_target_product_states() const;

    // Returns set of all product states whose DFA component is a sink trap
    [[nodiscard]] std::unordered_set<std::size_t> compute_sink_product_states() const;

private:
    // Precomputes observation label for each grid cell box
    void precompute_cell_labels(const grid::GridPartition& partition);
};

} // namespace symbolic::automaton

