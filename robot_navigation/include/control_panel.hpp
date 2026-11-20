#pragma once
#include "app_state.hpp"

namespace robot_nav::ui {

// Renders the right-hand dashboard containing:
// Card 1 (Controller): Run solver button, background synthesis progress bar, load/save binary buttons.
// Card 2 (Simulation): Play/Pause animation, Step Once, Reset, speed slider, noise toggle, horizon limit.
// Card 3 (Status & Telemetry): Live DFA state (q1..q5), robot (x, y, theta) coordinates, and run outcome banner.
void render_right_cards(AppState& app, float width);

} // namespace robot_nav::ui
