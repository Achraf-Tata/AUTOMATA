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

// discrete transition system representing the abstracted robot dynamics on the grid
class TransitionSystem {
public:
    size_t state_count = 0;
    size_t input_count = 0;

    // transition graph table: table[state][control] = list of reachable successor cells
    vector<vector<vector<size_t>>> transition_table;

    // default constructor
    TransitionSystem() = default;

    // allocate table with given number of states and controls
    TransitionSystem(size_t total_states, size_t total_inputs);

    // build the transition graph by propagating each grid cell under all controls
    void build_abstraction(
        const GridPartition& partition,
        const UnicycleModel& model,
        const function<void(size_t, size_t)>& progress_callback = nullptr);

    // get list of reachable next cells from a given cell and control action
    [[nodiscard]] const vector<size_t>& get_successors(
        size_t state_index,
        size_t control_index) const;

    // count total number of edges in the transition graph
    [[nodiscard]] size_t total_transitions() const noexcept;

    // check if a cell state index is within valid bounds
    [[nodiscard]] bool is_valid_state(size_t state_index) const noexcept;
};

} // namespace symbolic::abstraction

namespace symbolic {
    using abstraction::TransitionSystem;
}
