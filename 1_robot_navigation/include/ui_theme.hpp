#pragma once
#include "imgui.h"

namespace robot_nav::theme {

struct Fonts {
    ImFont* regular = nullptr;
    ImFont* bold    = nullptr;
    ImFont* title   = nullptr;
    ImFont* small   = nullptr;
};

extern Fonts g_fonts;

// Color palette constants
namespace colors {
    inline const ImVec4 bg_canvas        = {0.968f, 0.957f, 0.937f, 1.000f}; // Warm paper
    inline const ImVec4 card_bg          = {1.000f, 1.000f, 1.000f, 1.000f}; // Pure white
    inline const ImVec4 card_border      = {0.871f, 0.855f, 0.824f, 1.000f}; // Soft warm gray
    inline const ImVec4 text_primary     = {0.090f, 0.120f, 0.180f, 1.000f}; // Near black
    inline const ImVec4 text_muted       = {0.450f, 0.500f, 0.580f, 1.000f}; // Subtitle gray
    inline const ImVec4 primary_blue     = {0.055f, 0.435f, 0.914f, 1.000f}; // Action blue
    inline const ImVec4 primary_hover    = {0.145f, 0.500f, 0.980f, 1.000f};
    inline const ImVec4 green_badge_bg   = {0.133f, 0.690f, 0.298f, 1.000f}; // #22c55e
    inline const ImVec4 banner_green_bg  = {0.863f, 0.988f, 0.906f, 1.000f}; // #dcfce7
    inline const ImVec4 banner_green_txt = {0.082f, 0.502f, 0.204f, 1.000f}; // #15803d
    inline const ImVec4 banner_red_bg    = {0.996f, 0.886f, 0.886f, 1.000f}; // #fee2e2
    inline const ImVec4 banner_red_txt   = {0.725f, 0.110f, 0.110f, 1.000f}; // #b91c1c
    inline const ImVec4 banner_amber_bg  = {0.996f, 0.953f, 0.780f, 1.000f};
    inline const ImVec4 banner_amber_txt = {0.706f, 0.325f, 0.035f, 1.000f};
    inline const ImVec4 input_border     = {0.800f, 0.820f, 0.860f, 1.000f};
} // namespace colors

void setup_theme();
void load_fonts();

// Custom widgets matching the reference design
bool begin_card(const char* str_id, const ImVec2& size = {0.0f, 0.0f});
void end_card();

bool outline_button(const char* label, const ImVec2& size = {0.0f, 0.0f});
bool primary_button(const char* label, const ImVec2& size = {0.0f, 0.0f});
bool toggle_switch(const char* label, bool* v);
void pill_badge(const char* text, const ImVec4& bg_color, const ImVec4& text_color);
void status_banner(const char* text, const ImVec4& bg_color, const ImVec4& text_color, bool is_success = true);

} // namespace robot_nav::theme
