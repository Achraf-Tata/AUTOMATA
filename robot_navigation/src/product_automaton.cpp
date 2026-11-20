#include "product_automaton.hpp"
#include "grid_quantizer.hpp"

namespace symbolic::automaton {

using namespace std;
using namespace domain;
using namespace grid;
using enum AutomatonState;
using enum ObservationSymbol;

// constructor: build product automaton and classify each grid cell into its region
ProductAutomaton::ProductAutomaton(
    size_t n_cells,
    const GridPartition& grid,
    const ArenaRegions& regs)
    : num_cells(n_cells),
      dfa(create_navigation_specification_dfa()),
      regions(regs)
{
    precompute_cell_labels(grid);
}

// total product states: spatial_cells * 5 DFA states
size_t ProductAutomaton::total_product_states() const noexcept {
    return num_cells * DFA_STATE_COUNT;
}

// pack (cell, dfa_state) into a 1D index: (q - 1) * total_cells + cell
size_t ProductAutomaton::encode_product_index(
    size_t s,
    AutomatonState q) const noexcept
{
    auto q_offset = static_cast<size_t>(q) - 1;
    return q_offset * num_cells + s;
}

// unpack a 1D product index back into (cell, dfa_state)
ProductState ProductAutomaton::decode_product_index(
    size_t p) const noexcept
{
    size_t q_offset = p / num_cells;
    size_t s = p % num_cells;

    return ProductState{
        s,
        static_cast<AutomatonState>(q_offset + 1)
    };
}

// determine which region a 2D coordinate belongs to (obstacle R4 takes top priority)
ObservationSymbol ProductAutomaton::evaluate_point_observation(
    double x,
    double y) const noexcept
{
    // Obstacle R4 has highest priority
    if (x >= regions.obstacle.x_range.low &&
        x <= regions.obstacle.x_range.high &&
        y >= regions.obstacle.y_range.low &&
        y <= regions.obstacle.y_range.high)
    {
        return Obstacle;
    }

    // Waypoint R1
    if (x >= regions.waypoint_1.x_range.low &&
        x <= regions.waypoint_1.x_range.high &&
        y >= regions.waypoint_1.y_range.low &&
        y <= regions.waypoint_1.y_range.high)
    {
        return Waypoint1;
    }

    // Waypoint R2
    if (x >= regions.waypoint_2.x_range.low &&
        x <= regions.waypoint_2.x_range.high &&
        y >= regions.waypoint_2.y_range.low &&
        y <= regions.waypoint_2.y_range.high)
    {
        return Waypoint2;
    }

    // Goal R3
    if (x >= regions.goal.x_range.low &&
        x <= regions.goal.x_range.high &&
        y >= regions.goal.y_range.low &&
        y <= regions.goal.y_range.high)
    {
        return Goal;
    }

    return FreeSpace;
}

// Use the same whole-cell classification as the optimized solver.
void ProductAutomaton::precompute_cell_labels(const GridPartition& grid) {
    cell_labels.resize(num_cells, FreeSpace);
    cell_observation_masks.resize(num_cells);
    const ::grid::RegionConfig config{
        regions.waypoint_1.x_range.low, regions.waypoint_1.x_range.high,
        regions.waypoint_1.y_range.low, regions.waypoint_1.y_range.high,
        regions.waypoint_2.x_range.low, regions.waypoint_2.x_range.high,
        regions.waypoint_2.y_range.low, regions.waypoint_2.y_range.high,
        regions.goal.x_range.low, regions.goal.x_range.high,
        regions.goal.y_range.low, regions.goal.y_range.high,
        regions.obstacle.x_range.low, regions.obstacle.x_range.high,
        regions.obstacle.y_range.low, regions.obstacle.y_range.high
    };

    for (size_t s = 0; s < num_cells; ++s) {
        BoundingBox box = grid.get_cell_box(s);
        const int label = ::grid::get_cell_region_label(
            box.x_range.low, box.x_range.high, box.y_range.low, box.y_range.high, config);
        cell_labels[s] = static_cast<ObservationSymbol>(label - 1);
        cell_observation_masks[s] = ::grid::get_cell_observation_mask(
            box.x_range.low, box.x_range.high, box.y_range.low, box.y_range.high, config);
    }
}

// get precomputed region label for a cell
ObservationSymbol ProductAutomaton::get_cell_observation(size_t s) const noexcept{
    if (s < cell_labels.size()) {
        return cell_labels[s];
    }
    return FreeSpace;
}

// collect all product states where the DFA is in the accepting goal state (q4)
unordered_set<size_t> ProductAutomaton::compute_target_product_states() const {
    unordered_set<size_t> targets;

    for (size_t s = 0; s < num_cells; ++s) {
        targets.insert(encode_product_index(s, GoalReached));
    }

    return targets;
}

// collect all product states where the DFA is in the sink trap state (q5)
unordered_set<size_t> ProductAutomaton::compute_sink_product_states() const {
    unordered_set<size_t> sinks;

    for (size_t s = 0; s < num_cells; ++s) {
        sinks.insert(encode_product_index(s, SinkTrap));
    }

    return sinks;
}

} // namespace symbolic::automaton

namespace automaton {

using namespace std;

// DFA transition table lookup for fast integer indices
int dfa_transition(int q, int sym) noexcept {
    static constexpr int TABLE[5][5] = {
        { 1, 2, 3, 1, 5 },
        { 2, 2, 5, 4, 5 },
        { 3, 5, 3, 4, 5 },
        { 4, 4, 4, 4, 4 },
        { 5, 5, 5, 5, 5 }
    };
    if (q < 1 || q > 5 || sym < 1 || sym > 5) {
        return q;
    }
    return TABLE[q - 1][sym - 1];
}

// initialize value function: all states set to infinity, goal states (q4) set to 0
vector<float> init_value_function() {
    vector<float> V(NTOTAL, INF);
    int q4_base = (4 - 1) * NCELLS;
    for (int s = 0; s < NCELLS; ++s) {
        V[q4_base + s] = 0.0f;
    }
    return V;
}

// initialize controller table with zeros (uncontrolled)
vector<uint8_t> init_controller() {
    return vector<uint8_t>(NTOTAL, 0u);
}

} // namespace automaton
