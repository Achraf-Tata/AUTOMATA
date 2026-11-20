#include "transition_system.hpp"

namespace symbolic::abstraction {

using namespace std;
using namespace domain;
using namespace grid;
using namespace model;

// allocate 2D transition table: num_states x num_inputs
TransitionSystem::TransitionSystem(size_t n_states, size_t n_inputs)
    : num_states(n_states), num_inputs(n_inputs)
{
    transitions.resize(num_states, vector<vector<size_t>>(num_inputs));
}

// compute abstraction: for each cell and control action, find all reachable next cells
void TransitionSystem::build_abstraction(
    const GridPartition& grid,
    const UnicycleModel& model,
    const function<void(size_t, size_t)>& callback)
{
    num_states = grid.total_cells();
    num_inputs = model.num_controls();
    transitions.assign(num_states, vector<vector<size_t>>(num_inputs));

    for (size_t s = 0; s < num_states; ++s) {
        BoundingBox box = grid.get_cell_box(s);

        for (size_t u = 0; u < num_inputs; ++u) {
            const auto& ctrl = model.get_control(u);
            BoundingBox rbox = model.compute_reachable_box(box, ctrl);

            // A partially out-of-domain image makes this input unavailable.
            if (rbox.x_range.low < grid.domain.x_range.low ||
                rbox.x_range.high > grid.domain.x_range.high ||
                rbox.y_range.low < grid.domain.y_range.low ||
                rbox.y_range.high > grid.domain.y_range.high) {
                continue;
            }

            // over-approximation: all cells touching the reachable box are potential successors
            vector<size_t> succs = grid.query_overlapping_cells(rbox);
            transitions[s][u] = move(succs);
        }

        if (callback && (s % 2000 == 0 || s == num_states - 1)) {
            callback(s + 1, num_states);
        }
    }
}

// return the list of reachable next cells from a given state and control
const vector<size_t>& TransitionSystem::get_successors(size_t s, size_t u) const {
    return transitions[s][u];
}

// count total number of directed edges in the transition system
size_t TransitionSystem::total_transitions() const noexcept {
    size_t count = 0;
    for (const auto& row : transitions) {
        for (const auto& succs : row) {
            count += succs.size();
        }
    }
    return count;
}

// check if a cell index is valid
bool TransitionSystem::is_valid_state(size_t s) const noexcept {
    return s < num_states;
}

} // namespace symbolic::abstraction
