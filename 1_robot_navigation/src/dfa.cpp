#include "dfa.hpp"

namespace symbolic::automaton {

using namespace std;
using enum AutomatonState;
using enum ObservationSymbol;

// constructor: start in state q1, q4 is the goal, q5 is the trap
DeterministicFiniteAutomaton::DeterministicFiniteAutomaton() {
    initial_state = Initial;
    accepting_states = { GoalReached };
    sink_states = { SinkTrap };

    initialize_transition_table();
}

// build the transition table delta(q, symbol)
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
    transition_table[0][0] = Initial;
    transition_table[0][1] = VisitedWaypoint1;
    transition_table[0][2] = VisitedWaypoint2;
    transition_table[0][3] = Initial;
    transition_table[0][4] = SinkTrap;

    //   q2 (VisitedWaypoint1):
    //     FreeSpace  -> q2
    //     Waypoint1  -> q2
    //     Waypoint2  -> q5 (exclusivity violation: cannot visit both R1 and R2)
    //     Goal       -> q4 (success!)
    //     Obstacle   -> q5 (sink)
    transition_table[1][0] = VisitedWaypoint1;
    transition_table[1][1] = VisitedWaypoint1;
    transition_table[1][2] = SinkTrap;
    transition_table[1][3] = GoalReached;
    transition_table[1][4] = SinkTrap;

    //   q3 (VisitedWaypoint2):
    //     FreeSpace  -> q3
    //     Waypoint1  -> q5 (exclusivity violation: cannot visit both R1 and R2)
    //     Waypoint2  -> q3
    //     Goal       -> q4 (success!)
    //     Obstacle   -> q5 (sink)
    transition_table[2][0] = VisitedWaypoint2;
    transition_table[2][1] = SinkTrap;
    transition_table[2][2] = VisitedWaypoint2;
    transition_table[2][3] = GoalReached;
    transition_table[2][4] = SinkTrap;

    //   q4 (GoalReached - accepting trap state):
    //     All symbols -> q4
    for (uint8_t symbol_index = 0; symbol_index < SYMBOL_COUNT; ++symbol_index) {
        transition_table[3][symbol_index] = GoalReached;
    }

    //   q5 (SinkTrap - non-accepting sink state):
    //     All symbols -> q5
    for (uint8_t symbol_index = 0; symbol_index < SYMBOL_COUNT; ++symbol_index) {
        transition_table[4][symbol_index] = SinkTrap;
    }
}

// single transition step: return delta(state, symbol)
AutomatonState DeterministicFiniteAutomaton::transition(
    AutomatonState current_state,
    ObservationSymbol observation_symbol) const noexcept
{
    auto state_index = static_cast<uint8_t>(current_state) - 1;
    auto symbol_index = static_cast<uint8_t>(observation_symbol);

    if (state_index >= STATE_COUNT || symbol_index >= SYMBOL_COUNT) {
        return SinkTrap;
    }

    return transition_table[state_index][symbol_index];
}

// trace a sequence of observations from q1 and return the final state reached
AutomatonState DeterministicFiniteAutomaton::evaluate_trace(
    const vector<ObservationSymbol>& observation_trace) const noexcept
{
    AutomatonState current_state = initial_state;
    for (const auto& observation_symbol : observation_trace) {
        current_state = transition(current_state, observation_symbol);
        if (current_state == SinkTrap) {
            break;
        }
    }
    return current_state;
}

// check if the sequence of observations satisfies our mission (ends in q4)
bool DeterministicFiniteAutomaton::accepts_word(
    const vector<ObservationSymbol>& observation_word) const noexcept
{
    AutomatonState final_state = evaluate_trace(observation_word);
    return is_accepting_state(final_state);
}

// check if a state is in the accepting set (q4)
bool DeterministicFiniteAutomaton::is_accepting_state(AutomatonState state) const noexcept {
    return accepting_states.find(state) != accepting_states.end();
}

// check if a state is in the sink trap set (q5)
bool DeterministicFiniteAutomaton::is_sink_state(AutomatonState state) const noexcept {
    return sink_states.find(state) != sink_states.end();
}

// string description for printing DFA state in logs
string DeterministicFiniteAutomaton::state_to_string(AutomatonState state) {
    switch (state) {
        case Initial:          return "q1 (Initial)";
        case VisitedWaypoint1: return "q2 (Visited R1)";
        case VisitedWaypoint2: return "q3 (Visited R2)";
        case GoalReached:      return "q4 (Goal Reached [Accepting])";
        case SinkTrap:         return "q5 (Sink Trap [Violation])";
        default:               return "UnknownState";
    }
}

// string description for printing observation symbol in logs
string DeterministicFiniteAutomaton::symbol_to_string(ObservationSymbol symbol) {
    switch (symbol) {
        case FreeSpace: return "FreeSpace";
        case Waypoint1: return "Waypoint1 (R1)";
        case Waypoint2: return "Waypoint2 (R2)";
        case Goal:      return "Goal (R3)";
        case Obstacle:  return "Obstacle (R4)";
        default:        return "UnknownSymbol";
    }
}

// helper function to create our navigation mission DFA
DeterministicFiniteAutomaton create_navigation_specification_dfa() {
    return DeterministicFiniteAutomaton();
}

} // namespace symbolic::automaton

