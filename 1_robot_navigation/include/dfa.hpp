#pragma once

#include <vector>
#include <string>
#include <unordered_set>
#include <cstdint>

namespace symbolic::automaton {

using namespace std;

// symbols detected by the robot: free space, waypoints R1/R2, goal R3, obstacle R4
enum class ObservationSymbol : uint8_t {
    FreeSpace = 0,
    Waypoint1 = 1,
    Waypoint2 = 2,
    Goal      = 3,
    Obstacle  = 4
};

// DFA states for the mission specification:
// q1 = start, q2 = visited R1, q3 = visited R2, q4 = goal reached (accepting), q5 = sink trap (failed)
enum class AutomatonState : uint8_t {
    Initial          = 1,
    VisitedWaypoint1 = 2,
    VisitedWaypoint2 = 3,
    GoalReached      = 4,
    SinkTrap         = 5
};

// Deterministic Finite Automaton (DFA) that tracks high-level mission progress
class DeterministicFiniteAutomaton {
public:
    AutomatonState initial_state = AutomatonState::Initial;
    unordered_set<AutomatonState> accepting_states;
    unordered_set<AutomatonState> sink_states;

    // initialize DFA: starts in q1, accepting = {q4}, sink = {q5}
    DeterministicFiniteAutomaton();

    // transition function: delta(q, symbol) -> returns next automaton state
    [[nodiscard]] AutomatonState transition(
        AutomatonState current_state,
        ObservationSymbol observation_symbol) const noexcept;

    // check if an observation word satisfies the mission (ends in accepting state q4)
    [[nodiscard]] bool accepts_word(
        const vector<ObservationSymbol>& observation_word) const noexcept;

    // run a sequence of observations through the DFA from start and return final state
    [[nodiscard]] AutomatonState evaluate_trace(
        const vector<ObservationSymbol>& observation_trace) const noexcept;

    // check if this is an accepting goal state (q4)
    [[nodiscard]] bool is_accepting_state(AutomatonState state) const noexcept;

    // check if this is a permanent sink trap state (q5)
    [[nodiscard]] bool is_sink_state(AutomatonState state) const noexcept;

    // convert state enum to readable text for printing/debugging
    [[nodiscard]] static string state_to_string(AutomatonState state);

    // convert symbol enum to readable text for printing/debugging
    [[nodiscard]] static string symbol_to_string(ObservationSymbol symbol);

private:
    // 5x5 lookup table: delta[current_state - 1][symbol]
    static constexpr uint8_t STATE_COUNT = 5;
    static constexpr uint8_t SYMBOL_COUNT = 5;
    AutomatonState transition_table[STATE_COUNT][SYMBOL_COUNT]{};

    void initialize_transition_table();
};

// create the navigation mission DFA: visit (R1 xor R2), then R3, avoiding R4
DeterministicFiniteAutomaton create_navigation_specification_dfa();

} // namespace symbolic::automaton

namespace symbolic {
    using automaton::ObservationSymbol;
    using automaton::AutomatonState;
    using automaton::DeterministicFiniteAutomaton;
    using automaton::create_navigation_specification_dfa;
}

