#pragma once
#include "imgui.h"

namespace robot_nav::theme {

// Holds loaded typography font pointers for regular, bold, title, and caption text
struct Fonts {
    ImFont* regular = nullptr;
    ImFont* bold    = nullptr;
    ImFont* title   = nullptr;
    ImFont* small   = nullptr;
};

extern Fonts g_fonts;

// UI color palette matching modern technical design aesthetics
namespace colors {
    inline const ImVec4 bg_canvas        = {0.968f, 0.957f, 0.937f, 1.000f}; // Warm paper canvas
    inline const ImVec4 card_bg          = {1.000f, 1.000f, 1.000f, 1.000f}; // Pure white card surface
    inline const ImVec4 card_border      = {0.871f, 0.855f, 0.824f, 1.000f}; // Subtle warm border
    inline const ImVec4 text_primary     = {0.090f, 0.120f, 0.180f, 1.000f}; // High-contrast text
    inline const ImVec4 text_muted       = {0.450f, 0.500f, 0.580f, 1.000f}; // Secondary text
    inline const ImVec4 primary_blue     = {0.055f, 0.435f, 0.914f, 1.000f}; // Accent button blue
    inline const ImVec4 primary_hover    = {0.145f, 0.500f, 0.980f, 1.000f}; // Button hover state
    inline const ImVec4 green_badge_bg   = {0.133f, 0.690f, 0.298f, 1.000f}; // Success badge green
    inline const ImVec4 banner_green_bg  = {0.863f, 0.988f, 0.906f, 1.000f}; // Success banner background
    inline const ImVec4 banner_green_txt = {0.082f, 0.502f, 0.204f, 1.000f}; // Success banner text
    inline const ImVec4 banner_red_bg    = {0.996f, 0.886f, 0.886f, 1.000f}; // Error banner background
    inline const ImVec4 banner_red_txt   = {0.725f, 0.110f, 0.110f, 1.000f}; // Error banner text
    inline const ImVec4 banner_amber_bg  = {0.996f, 0.953f, 0.780f, 1.000f}; // Warning banner background
    inline const ImVec4 banner_amber_txt = {0.706f, 0.325f, 0.035f, 1.000f}; // Warning banner text
    inline const ImVec4 input_border     = {0.800f, 0.820f, 0.860f, 1.000f}; // Input field border
} // namespace colors

// Configures global ImGui style settings (padding, roundings, colors)
void setup_theme();

// Loads vector TTF fonts (Segoe UI / Arial) from Windows system directory
void load_fonts();

// Custom card container widgets with drop shadows and rounded borders
bool begin_card(const char* str_id, float width, float height = 0.0f);
bool begin_card(const char* str_id, const ImVec2& size);
void end_card();

// Custom UI button controls
bool outline_button(const char* label, const ImVec2& size = {0.0f, 0.0f});
bool primary_button(const char* label, const ImVec2& size = {0.0f, 0.0f});
bool toggle_switch(const char* label, bool* v);
void pill_badge(const char* text, const ImVec4& bg_color, const ImVec4& text_color);
void status_banner(const char* text, const ImVec4& bg_color, const ImVec4& text_color, bool is_success = true);

} // namespace robot_nav::theme
