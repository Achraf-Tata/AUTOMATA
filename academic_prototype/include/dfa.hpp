#pragma once

#include <vector>
#include <string>
#include <unordered_set>
#include <cstdint>

namespace symbolic::automaton {

enum class ObservationSymbol : std::uint8_t {
    FreeSpace = 0,
    Waypoint1 = 1,
    Waypoint2 = 2,
    Goal      = 3,
    Obstacle  = 4
};

enum class AutomatonState : std::uint8_t {
    Initial          = 1,
    VisitedWaypoint1 = 2,
    VisitedWaypoint2 = 3,
    GoalReached      = 4,
    SinkTrap         = 5
};

class DeterministicFiniteAutomaton {
public:
    AutomatonState initial_state = AutomatonState::Initial;
    std::unordered_set<AutomatonState> accepting_states;
    std::unordered_set<AutomatonState> sink_states;

    DeterministicFiniteAutomaton();

    [[nodiscard]] AutomatonState transition(
        AutomatonState current_state,
        ObservationSymbol observation_symbol) const noexcept;

    // Language evaluation: determines if a finite word of observations belongs to L(A)
    [[nodiscard]] bool accepts_word(
        const std::vector<ObservationSymbol>& observation_word) const noexcept;

    [[nodiscard]] AutomatonState evaluate_trace(
        const std::vector<ObservationSymbol>& observation_trace) const noexcept;

    [[nodiscard]] bool is_accepting_state(AutomatonState state) const noexcept;
    [[nodiscard]] bool is_sink_state(AutomatonState state) const noexcept;

    [[nodiscard]] static std::string state_to_string(AutomatonState state);
    [[nodiscard]] static std::string symbol_to_string(ObservationSymbol symbol);

private:
    // Transition table: [current_state_index - 1][symbol_index] -> next_state
    static constexpr std::uint8_t STATE_COUNT = 5;
    static constexpr std::uint8_t SYMBOL_COUNT = 5;
    AutomatonState transition_table[STATE_COUNT][SYMBOL_COUNT]{};

    void initialize_transition_table();
};

// Factory for specification: Visit R1 or R2 exclusively, then R3, avoiding R4
DeterministicFiniteAutomaton create_navigation_specification_dfa();

} // namespace symbolic::automaton
