#include "transition_system.hpp"

namespace symbolic::abstraction {

// Allocates the 2D transition table of successor lists
TransitionSystem::TransitionSystem(std::size_t total_states, std::size_t total_inputs)
    : state_count(total_states), input_count(total_inputs)
{
    transition_table.resize(state_count, std::vector<std::vector<std::size_t>>(input_count));
}

// Builds the transition relation by computing reachable boxes for each cell and control
void TransitionSystem::build_abstraction(
    const grid::GridPartition& partition,
    const model::UnicycleModel& model,
    const std::function<void(std::size_t, std::size_t)>& progress_callback)
{
    state_count = partition.total_cells();
    input_count = model.num_controls();
    transition_table.assign(state_count, std::vector<std::vector<std::size_t>>(input_count));

    for (std::size_t source_state_index = 0; source_state_index < state_count; ++source_state_index) {
        domain::BoundingBox cell_box = partition.get_cell_box(source_state_index);

        for (std::size_t control_index = 0; control_index < input_count; ++control_index) {
            const auto& control_input = model.get_control(control_index);
            domain::BoundingBox reachable_box = model.compute_reachable_box(cell_box, control_input);

            // Over-approximation: find all grid cells that intersect the reachable box
            std::vector<std::size_t> successor_cells = partition.query_overlapping_cells(reachable_box);
            transition_table[source_state_index][control_index] = std::move(successor_cells);
        }

        if (progress_callback && (source_state_index % 2000 == 0 || source_state_index == state_count - 1)) {
            progress_callback(source_state_index + 1, state_count);
        }
    }
}

// Returns the list of post successor states for a given state and control
const std::vector<std::size_t>& TransitionSystem::get_successors(std::size_t state_index, std::size_t control_index) const {
    return transition_table[state_index][control_index];
}

// Computes the total number of transitions in the graph
std::size_t TransitionSystem::total_transitions() const noexcept {
    std::size_t count = 0;
    for (const auto& state_entry : transition_table) {
        for (const auto& successor_list : state_entry) {
            count += successor_list.size();
        }
    }
    return count;
}

// Checks if state index is valid
bool TransitionSystem::is_valid_state(std::size_t state_index) const noexcept {
    return state_index < state_count;
}

} // namespace symbolic::abstraction
