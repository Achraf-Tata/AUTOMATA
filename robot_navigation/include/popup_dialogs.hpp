#pragma once
#include "app_state.hpp"

namespace robot_nav::dialogs {

// Renders modal popups when exceptional events occur:
// 1. Arena Modified Warning: warns the user that regions were moved/resized after controller synthesis,
//    offering one-click re-synthesis or ignoring the change.
// 2. Alert Modal: clearly explains simulation stops (obstacle collision, DFA violation, out of bounds, etc.)
//    with structured "What happened", "Why it occurred", and "How to fix it" guidance.
void render_popup_dialogs(AppState& app);

} // namespace robot_nav::dialogs
