#include "transition_system.hpp"

namespace symbolic::abstraction {

using namespace std;
using namespace domain;
using namespace grid;
using namespace model;

// allocate 2D transition table: state_count x input_count
TransitionSystem::TransitionSystem(size_t total_states, size_t total_inputs)
    : state_count(total_states), input_count(total_inputs)
{
    transition_table.resize(state_count, vector<vector<size_t>>(input_count));
}

// compute abstraction: for each cell and control action, find all reachable next cells
void TransitionSystem::build_abstraction(
    const GridPartition& partition,
    const UnicycleModel& model,
    const function<void(size_t, size_t)>& progress_callback)
{
    state_count = partition.total_cells();
    input_count = model.num_controls();
    transition_table.assign(state_count, vector<vector<size_t>>(input_count));

    for (size_t source_state_index = 0; source_state_index < state_count; ++source_state_index) {
        BoundingBox cell_box = partition.get_cell_box(source_state_index);

        for (size_t control_index = 0; control_index < input_count; ++control_index) {
            const auto& control_input = model.get_control(control_index);
            BoundingBox reachable_box = model.compute_reachable_box(cell_box, control_input);

            // over-approximation: all cells touching the reachable box are potential successors
            vector<size_t> successor_cells = partition.query_overlapping_cells(reachable_box);
            transition_table[source_state_index][control_index] = move(successor_cells);
        }

        if (progress_callback && (source_state_index % 2000 == 0 || source_state_index == state_count - 1)) {
            progress_callback(source_state_index + 1, state_count);
        }
    }
}

// return the list of reachable next cells from a given state and control
const vector<size_t>& TransitionSystem::get_successors(size_t state_index, size_t control_index) const {
    return transition_table[state_index][control_index];
}

// count total number of directed edges in the transition system
size_t TransitionSystem::total_transitions() const noexcept {
    size_t count = 0;
    for (const auto& state_entry : transition_table) {
        for (const auto& successor_list : state_entry) {
            count += successor_list.size();
        }
    }
    return count;
}

// check if a cell index is valid
bool TransitionSystem::is_valid_state(size_t state_index) const noexcept {
    return state_index < state_count;
}

} // namespace symbolic::abstraction
