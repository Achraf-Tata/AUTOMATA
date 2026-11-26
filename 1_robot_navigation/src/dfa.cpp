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
    AutomatonState q,
    ObservationSymbol sym) const noexcept
{
    auto q_idx = static_cast<uint8_t>(q) - 1;
    auto sym_idx = static_cast<uint8_t>(sym);

    if (q_idx >= STATE_COUNT || sym_idx >= SYMBOL_COUNT) {
        return SinkTrap;
    }

    return transition_table[q_idx][sym_idx];
}

// trace a sequence of observations from q1 and return the final state reached
AutomatonState DeterministicFiniteAutomaton::evaluate_trace(
    const vector<ObservationSymbol>& trace) const noexcept
{
    AutomatonState q = initial_state;
    for (const auto& sym : trace) {
        q = transition(q, sym);
        if (q == SinkTrap) {
            break;
        }
    }
    return q;
}

// check if the sequence of observations satisfies our mission (ends in q4)
bool DeterministicFiniteAutomaton::accepts_word(
    const vector<ObservationSymbol>& word) const noexcept
{
    AutomatonState q_final = evaluate_trace(word);
    return is_accepting_state(q_final);
}

// check if a state is in the accepting set (q4)
bool DeterministicFiniteAutomaton::is_accepting_state(AutomatonState q) const noexcept {
    return accepting_states.find(q) != accepting_states.end();
}

// check if a state is in the sink trap set (q5)
bool DeterministicFiniteAutomaton::is_sink_state(AutomatonState q) const noexcept {
    return sink_states.find(q) != sink_states.end();
}

// string description for printing DFA state in logs
string DeterministicFiniteAutomaton::state_to_string(AutomatonState q) {
    switch (q) {
        case Initial:          return "q1 (Initial)";
        case VisitedWaypoint1: return "q2 (Visited R1)";
        case VisitedWaypoint2: return "q3 (Visited R2)";
        case GoalReached:      return "q4 (Goal Reached [Accepting])";
        case SinkTrap:         return "q5 (Sink Trap [Violation])";
        default:               return "UnknownState";
    }
}

// string description for printing observation symbol in logs
string DeterministicFiniteAutomaton::symbol_to_string(ObservationSymbol sym) {
    switch (sym) {
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

