#pragma once
#include "app_state.hpp"

namespace robot_nav::ui {

// Top navigation bar
void render_header_bar(AppState& app, float width);

// Right side 5 cards
void render_right_cards(AppState& app, float width);

} // namespace robot_nav::ui
