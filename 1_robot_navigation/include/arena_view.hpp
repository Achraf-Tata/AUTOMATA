#pragma once
#include "app_state.hpp"

namespace robot_nav::arena {

// Render Arena card and Legend card
void render_arena_card(AppState& app, float width, float height);
void render_legend_card(AppState& app, float width);

} // namespace robot_nav::arena
