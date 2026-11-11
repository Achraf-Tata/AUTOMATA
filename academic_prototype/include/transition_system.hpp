#pragma once

#include "grid_partition.hpp"
#include "unicycle_model.hpp"
#include <vector>
#include <cstddef>
#include <functional>

namespace symbolic::abstraction {

class TransitionSystem {
public:
    std::size_t num_states = 0;
    std::size_t num_inputs = 0;

    // Adjacency structure: transitions[state][input] = list of post successor states
    std::vector<std::vector<std::vector<std::size_t>>> transitions;

    TransitionSystem() = default;
    TransitionSystem(std::size_t states, std::size_t inputs);

    void build_abstraction(const grid::GridPartition& partition,
                           const model::UnicycleModel& model,
                           const std::function<void(std::size_t, std::size_t)>& progress_callback = nullptr);

    [[nodiscard]] const std::vector<std::size_t>& get_post(std::size_t state, std::size_t input) const;
    [[nodiscard]] std::size_t total_transitions() const noexcept;
    [[nodiscard]] bool is_valid_state(std::size_t state) const noexcept;
};

} // namespace symbolic::abstraction
