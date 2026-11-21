#pragma once

#include "app_state.hpp"

namespace circular::view {

// Renders the main circular building floor plan canvas and embedded rolling temperature plot
void render_building_view(AppState& app, float width, float height);

} // namespace circular::view
