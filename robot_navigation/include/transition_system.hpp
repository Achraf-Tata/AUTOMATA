#pragma once

#include "grid_partition.hpp"
#include "unicycle_model.hpp"
#include <vector>
#include <cstddef>
#include <functional>

namespace symbolic::abstraction {

using namespace std;
using namespace grid;
using namespace model;

// Map of all possible moves: tells us which cells the robot might land in after each move
class TransitionSystem {
public:
    size_t num_states = 0;
    size_t num_inputs = 0;

    // Lookup table: transitions[s][u] = list of possible next cells the robot could land in
    vector<vector<vector<size_t>>> transitions;

    // Default constructor
    TransitionSystem() = default;

    // Allocate table with given number of states and controls
    TransitionSystem(size_t n_states, size_t n_inputs);

    // Builds the move map by testing every move from every grid cell using the robot equations
    void build_abstraction(
        const GridPartition& grid,
        const UnicycleModel& model,
        const function<void(size_t, size_t)>& callback = nullptr);

    // Get the list of cells the robot might end up in after taking this move
    [[nodiscard]] const vector<size_t>& get_successors(
        size_t s,
        size_t u) const;

    // Total number of possible connections across the whole map
    [[nodiscard]] size_t total_transitions() const noexcept;

    // Check if a cell state index is within valid bounds
    [[nodiscard]] bool is_valid_state(size_t s) const noexcept;
};

} // namespace symbolic::abstraction

namespace symbolic {
    using abstraction::TransitionSystem;
}
