#pragma once

#include "grid_partition.hpp"
#include "unicycle_model.hpp"
#include <vector>
#include <cstddef>
#include <functional>

namespace symbolic::abstraction {

class TransitionSystem {
public:
    std::size_t state_count = 0;
    std::size_t input_count = 0;

    // Adjacency table: transition_table[state_index][control_index] = list of post successor state indices
    std::vector<std::vector<std::vector<std::size_t>>> transition_table;

    TransitionSystem() = default;
    TransitionSystem(std::size_t total_states, std::size_t total_inputs);

    void build_abstraction(
        const grid::GridPartition& partition,
        const model::UnicycleModel& model,
        const std::function<void(std::size_t, std::size_t)>& progress_callback = nullptr);

    [[nodiscard]] const std::vector<std::size_t>& get_successors(
        std::size_t state_index,
        std::size_t control_index) const;

    [[nodiscard]] std::size_t total_transitions() const noexcept;
    [[nodiscard]] bool is_valid_state(std::size_t state_index) const noexcept;
};

} // namespace symbolic::abstraction
