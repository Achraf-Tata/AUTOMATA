#include "dfa.hpp"

namespace symbolic::automaton {

DeterministicFiniteAutomaton::DeterministicFiniteAutomaton() {
    initial_state = AutomatonState::Initial;
    accepting_states = { AutomatonState::GoalReached };
    sink_states = { AutomatonState::SinkTrap };

    initialize_transition_table();
}

void DeterministicFiniteAutomaton::initialize_transition_table() {
    // Specification from project presentation:
    // "Go to R1 or to R2, but only to one of them, then in R3, while avoiding R4 all the way."
    //
    // Symbols:
    //   0: FreeSpace
    //   1: Waypoint1 (R1)
    //   2: Waypoint2 (R2)
    //   3: Goal (R3)
    //   4: Obstacle (R4)
    //
    // States:
    //   q1 (Initial):
    //     FreeSpace  -> q1
    //     Waypoint1  -> q2
    //     Waypoint2  -> q3
    //     Goal       -> q1 (cannot reach goal before visiting a waypoint)
    //     Obstacle   -> q5 (sink)
    transition_table[0][0] = AutomatonState::Initial;
    transition_table[0][1] = AutomatonState::VisitedWaypoint1;
    transition_table[0][2] = AutomatonState::VisitedWaypoint2;
    transition_table[0][3] = AutomatonState::Initial;
    transition_table[0][4] = AutomatonState::SinkTrap;

    //   q2 (VisitedWaypoint1):
    //     FreeSpace  -> q2
    //     Waypoint1  -> q2
    //     Waypoint2  -> q5 (exclusivity violation: cannot visit both R1 and R2)
    //     Goal       -> q4 (success!)
    //     Obstacle   -> q5 (sink)
    transition_table[1][0] = AutomatonState::VisitedWaypoint1;
    transition_table[1][1] = AutomatonState::VisitedWaypoint1;
    transition_table[1][2] = AutomatonState::SinkTrap;
    transition_table[1][3] = AutomatonState::GoalReached;
    transition_table[1][4] = AutomatonState::SinkTrap;

    //   q3 (VisitedWaypoint2):
    //     FreeSpace  -> q3
    //     Waypoint1  -> q5 (exclusivity violation: cannot visit both R1 and R2)
    //     Waypoint2  -> q3
    //     Goal       -> q4 (success!)
    //     Obstacle   -> q5 (sink)
    transition_table[2][0] = AutomatonState::VisitedWaypoint2;
    transition_table[2][1] = AutomatonState::SinkTrap;
    transition_table[2][2] = AutomatonState::VisitedWaypoint2;
    transition_table[2][3] = AutomatonState::GoalReached;
    transition_table[2][4] = AutomatonState::SinkTrap;

    //   q4 (GoalReached - accepting trap state):
    //     All symbols -> q4
    for (std::uint8_t symbol_index = 0; symbol_index < SYMBOL_COUNT; ++symbol_index) {
        transition_table[3][symbol_index] = AutomatonState::GoalReached;
    }

    //   q5 (SinkTrap - non-accepting sink state):
    //     All symbols -> q5
    for (std::uint8_t symbol_index = 0; symbol_index < SYMBOL_COUNT; ++symbol_index) {
        transition_table[4][symbol_index] = AutomatonState::SinkTrap;
    }
}

AutomatonState DeterministicFiniteAutomaton::transition(
    AutomatonState current_state,
    ObservationSymbol observation_symbol) const noexcept
{
    auto state_index = static_cast<std::uint8_t>(current_state) - 1;
    auto symbol_index = static_cast<std::uint8_t>(observation_symbol);

    if (state_index >= STATE_COUNT || symbol_index >= SYMBOL_COUNT) {
        return AutomatonState::SinkTrap;
    }

    return transition_table[state_index][symbol_index];
}

AutomatonState DeterministicFiniteAutomaton::evaluate_trace(
    const std::vector<ObservationSymbol>& observation_trace) const noexcept
{
    AutomatonState current_state = initial_state;
    for (const auto& observation_symbol : observation_trace) {
        current_state = transition(current_state, observation_symbol);
        if (current_state == AutomatonState::SinkTrap) {
            break;
        }
    }
    return current_state;
}

bool DeterministicFiniteAutomaton::accepts_word(
    const std::vector<ObservationSymbol>& observation_word) const noexcept
{
    AutomatonState final_state = evaluate_trace(observation_word);
    return is_accepting_state(final_state);
}

bool DeterministicFiniteAutomaton::is_accepting_state(AutomatonState state) const noexcept {
    return accepting_states.find(state) != accepting_states.end();
}

bool DeterministicFiniteAutomaton::is_sink_state(AutomatonState state) const noexcept {
    return sink_states.find(state) != sink_states.end();
}

std::string DeterministicFiniteAutomaton::state_to_string(AutomatonState state) {
    switch (state) {
        case AutomatonState::Initial:          return "q1 (Initial)";
        case AutomatonState::VisitedWaypoint1: return "q2 (Visited R1)";
        case AutomatonState::VisitedWaypoint2: return "q3 (Visited R2)";
        case AutomatonState::GoalReached:      return "q4 (Goal Reached [Accepting])";
        case AutomatonState::SinkTrap:         return "q5 (Sink Trap [Violation])";
        default:                               return "UnknownState";
    }
}

std::string DeterministicFiniteAutomaton::symbol_to_string(ObservationSymbol symbol) {
    switch (symbol) {
        case ObservationSymbol::FreeSpace: return "FreeSpace";
        case ObservationSymbol::Waypoint1: return "Waypoint1 (R1)";
        case ObservationSymbol::Waypoint2: return "Waypoint2 (R2)";
        case ObservationSymbol::Goal:      return "Goal (R3)";
        case ObservationSymbol::Obstacle:  return "Obstacle (R4)";
        default:                           return "UnknownSymbol";
    }
}

DeterministicFiniteAutomaton create_navigation_specification_dfa() {
    return DeterministicFiniteAutomaton();
}

} // namespace symbolic::automaton
