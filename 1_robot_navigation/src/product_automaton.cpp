#include "product_automaton.hpp"

namespace symbolic::automaton {

ProductAutomaton::ProductAutomaton(
    std::size_t total_spatial_cells,
    const grid::GridPartition& partition,
    const ArenaRegions& regions)
    : spatial_cell_count(total_spatial_cells),
      specification_dfa(create_navigation_specification_dfa()),
      arena_regions(regions)
{
    precompute_cell_labels(partition);
}

std::size_t ProductAutomaton::total_product_states() const noexcept {
    return spatial_cell_count * DFA_STATE_COUNT;
}

std::size_t ProductAutomaton::encode_product_index(
    std::size_t spatial_cell_index,
    AutomatonState automaton_state) const noexcept
{
    auto state_offset = static_cast<std::size_t>(automaton_state) - 1;
    return state_offset * spatial_cell_count + spatial_cell_index;
}

ProductState ProductAutomaton::decode_product_index(
    std::size_t product_index) const noexcept
{
    std::size_t state_offset = product_index / spatial_cell_count;
    std::size_t spatial_index = product_index % spatial_cell_count;

    return ProductState{
        spatial_index,
        static_cast<AutomatonState>(state_offset + 1)
    };
}

ObservationSymbol ProductAutomaton::evaluate_point_observation(
    double x_coordinate,
    double y_coordinate) const noexcept
{
    // Obstacle R4 has highest priority
    if (x_coordinate >= arena_regions.obstacle.x_range.lower_bound &&
        x_coordinate <= arena_regions.obstacle.x_range.upper_bound &&
        y_coordinate >= arena_regions.obstacle.y_range.lower_bound &&
        y_coordinate <= arena_regions.obstacle.y_range.upper_bound)
    {
        return ObservationSymbol::Obstacle;
    }

    // Waypoint R1
    if (x_coordinate >= arena_regions.waypoint_1.x_range.lower_bound &&
        x_coordinate <= arena_regions.waypoint_1.x_range.upper_bound &&
        y_coordinate >= arena_regions.waypoint_1.y_range.lower_bound &&
        y_coordinate <= arena_regions.waypoint_1.y_range.upper_bound)
    {
        return ObservationSymbol::Waypoint1;
    }

    // Waypoint R2
    if (x_coordinate >= arena_regions.waypoint_2.x_range.lower_bound &&
        x_coordinate <= arena_regions.waypoint_2.x_range.upper_bound &&
        y_coordinate >= arena_regions.waypoint_2.y_range.lower_bound &&
        y_coordinate <= arena_regions.waypoint_2.y_range.upper_bound)
    {
        return ObservationSymbol::Waypoint2;
    }

    // Goal R3
    if (x_coordinate >= arena_regions.goal.x_range.lower_bound &&
        x_coordinate <= arena_regions.goal.x_range.upper_bound &&
        y_coordinate >= arena_regions.goal.y_range.lower_bound &&
        y_coordinate <= arena_regions.goal.y_range.upper_bound)
    {
        return ObservationSymbol::Goal;
    }

    return ObservationSymbol::FreeSpace;
}

void ProductAutomaton::precompute_cell_labels(const grid::GridPartition& partition) {
    cell_observation_labels.resize(spatial_cell_count, ObservationSymbol::FreeSpace);

    for (std::size_t cell_index = 0; cell_index < spatial_cell_count; ++cell_index) {
        domain::BoundingBox cell_box = partition.get_cell_box(cell_index);
        auto center_point = cell_box.center();

        cell_observation_labels[cell_index] = evaluate_point_observation(center_point[0], center_point[1]);
    }
}

ObservationSymbol ProductAutomaton::get_cell_observation(
    std::size_t spatial_cell_index) const noexcept
{
    if (spatial_cell_index < cell_observation_labels.size()) {
        return cell_observation_labels[spatial_cell_index];
    }
    return ObservationSymbol::FreeSpace;
}

std::unordered_set<std::size_t> ProductAutomaton::compute_target_product_states() const {
    std::unordered_set<std::size_t> target_product_states;

    for (std::size_t spatial_index = 0; spatial_index < spatial_cell_count; ++spatial_index) {
        // Goal reached (q4) is the accepting target state
        std::size_t target_product_index = encode_product_index(spatial_index, AutomatonState::GoalReached);
        target_product_states.insert(target_product_index);
    }

    return target_product_states;
}

std::unordered_set<std::size_t> ProductAutomaton::compute_sink_product_states() const {
    std::unordered_set<std::size_t> sink_product_states;

    for (std::size_t spatial_index = 0; spatial_index < spatial_cell_count; ++spatial_index) {
        // Sink trap (q5) is the error/obstacle state
        std::size_t sink_product_index = encode_product_index(spatial_index, AutomatonState::SinkTrap);
        sink_product_states.insert(sink_product_index);
    }

    return sink_product_states;
}

} // namespace symbolic::automaton

