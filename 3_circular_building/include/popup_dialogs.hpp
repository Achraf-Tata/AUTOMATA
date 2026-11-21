#pragma once

#include "app_state.hpp"

namespace circular::dialogs {

// Renders modal popups: About Theory, Curse of Dimensionality, and Help
void render_popups(AppState& app);

// Request opening dialogs
void open_about_dialog();
void open_curse_dialog();

} // namespace circular::dialogs
