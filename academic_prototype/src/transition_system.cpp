#include "transition_system.hpp"

namespace symbolic::abstraction {

TransitionSystem::TransitionSystem(std::size_t states, std::size_t inputs)
    : num_states(states), num_inputs(inputs)
{
    transitions.resize(num_states, std::vector<std::vector<std::size_t>>(num_inputs));
}

void TransitionSystem::build_abstraction(
    const grid::GridPartition& partition,
    const model::UnicycleModel& model,
    const std::function<void(std::size_t, std::size_t)>& progress_callback)
{
    num_states = partition.total_cells();
    num_inputs = model.num_controls();
    transitions.assign(num_states, std::vector<std::vector<std::size_t>>(num_inputs));

    for (std::size_t q = 0; q < num_states; ++q) {
        domain::BoundingBox cell_box = partition.get_cell_box(q);

        for (std::size_t u_idx = 0; u_idx < num_inputs; ++u_idx) {
            const auto& u = model.get_control(u_idx);
            domain::BoundingBox reachable_box = model.compute_reachable_box(cell_box, u);

            // Over-approximation: find all grid cells that intersect the reachable box
            std::vector<std::size_t> succs = partition.query_overlapping_cells(reachable_box);
            transitions[q][u_idx] = std::move(succs);
        }

        if (progress_callback && (q % 2000 == 0 || q == num_states - 1)) {
            progress_callback(q + 1, num_states);
        }
    }
}

const std::vector<std::size_t>& TransitionSystem::get_post(std::size_t state, std::size_t input) const {
    return transitions[state][input];
}

std::size_t TransitionSystem::total_transitions() const noexcept {
    std::size_t count = 0;
    for (const auto& state_trans : transitions) {
        for (const auto& post_list : state_trans) {
            count += post_list.size();
        }
    }
    return count;
}

bool TransitionSystem::is_valid_state(std::size_t state) const noexcept {
    return state < num_states;
}

} // namespace symbolic::abstraction
