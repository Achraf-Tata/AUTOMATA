#include "ui_theme.hpp"
#include <filesystem>
#include <cmath>
#include <algorithm>

namespace circular::theme {

using namespace std;
using namespace std::filesystem;
using namespace ImGui;
using namespace colors;

Fonts g_fonts;

void load_fonts() {
    ImGuiIO& io = GetIO();
    io.Fonts->Clear();

    const char* font_path_reg  = "C:\\Windows\\Fonts\\segoeui.ttf";
    const char* font_path_bold = "C:\\Windows\\Fonts\\segoeuib.ttf";

    ImFontConfig cfg;
    cfg.OversampleH = 3;
    cfg.OversampleV = 2;
    cfg.PixelSnapH = false;
    cfg.RasterizerMultiply = 1.15f;

    if (exists(font_path_reg)) {
        g_fonts.regular = io.Fonts->AddFontFromFileTTF(font_path_reg, 17.0f, &cfg);
        g_fonts.small   = io.Fonts->AddFontFromFileTTF(font_path_reg, 14.0f, &cfg);
        if (exists(font_path_bold)) {
            g_fonts.bold  = io.Fonts->AddFontFromFileTTF(font_path_bold, 17.0f, &cfg);
            g_fonts.title = io.Fonts->AddFontFromFileTTF(font_path_bold, 22.0f, &cfg);
        } else {
            g_fonts.bold  = g_fonts.regular;
            g_fonts.title = g_fonts.regular;
        }
    } else {
        g_fonts.regular = io.Fonts->AddFontDefault(&cfg);
        g_fonts.bold    = g_fonts.regular;
        g_fonts.title   = g_fonts.regular;
        g_fonts.small   = g_fonts.regular;
    }
}

void setup_theme() {
    ImGuiStyle& style = GetStyle();

    style.WindowRounding    = 0.0f;
    style.ChildRounding     = 7.0f;
    style.FrameRounding     = 5.0f;
    style.PopupRounding     = 7.0f;
    style.GrabRounding      = 5.0f;
    style.TabRounding       = 5.0f;

    style.WindowBorderSize  = 0.0f;
    style.ChildBorderSize   = 1.0f;
    style.FrameBorderSize   = 1.0f;
    style.PopupBorderSize   = 1.0f;

    style.WindowPadding     = {14.0f, 12.0f};
    style.FramePadding      = {10.0f, 6.0f};
    style.ItemSpacing       = {10.0f, 7.0f};
    style.ItemInnerSpacing  = {6.0f, 5.0f};
    style.ScrollbarSize     = 10.0f;
    style.ScrollbarRounding = 5.0f;

    ImVec4* clrs = style.Colors;
    clrs[ImGuiCol_WindowBg]             = bg_canvas;
    clrs[ImGuiCol_ChildBg]              = card_bg;
    clrs[ImGuiCol_PopupBg]              = card_bg;
    clrs[ImGuiCol_Border]               = card_border;
    clrs[ImGuiCol_BorderShadow]         = {0.0f, 0.0f, 0.0f, 0.0f};

    clrs[ImGuiCol_Text]                 = text_primary;
    clrs[ImGuiCol_TextDisabled]         = text_muted;

    clrs[ImGuiCol_FrameBg]              = {0.985f, 0.985f, 0.985f, 1.0f};
    clrs[ImGuiCol_FrameBgHovered]       = {0.960f, 0.960f, 0.970f, 1.0f};
    clrs[ImGuiCol_FrameBgActive]        = {0.920f, 0.930f, 0.950f, 1.0f};

    clrs[ImGuiCol_TitleBg]              = card_bg;
    clrs[ImGuiCol_TitleBgActive]        = card_bg;
    clrs[ImGuiCol_TitleBgCollapsed]     = card_bg;

    clrs[ImGuiCol_Button]               = primary_blue;
    clrs[ImGuiCol_ButtonHovered]        = primary_hover;
    clrs[ImGuiCol_ButtonActive]         = {0.040f, 0.350f, 0.750f, 1.0f};

    clrs[ImGuiCol_CheckMark]            = primary_blue;
    clrs[ImGuiCol_SliderGrab]           = primary_blue;
    clrs[ImGuiCol_SliderGrabActive]     = primary_hover;

    clrs[ImGuiCol_Header]               = {0.940f, 0.950f, 0.970f, 1.0f};
    clrs[ImGuiCol_HeaderHovered]        = {0.900f, 0.920f, 0.950f, 1.0f};
    clrs[ImGuiCol_HeaderActive]         = {0.860f, 0.890f, 0.930f, 1.0f};

    clrs[ImGuiCol_Separator]            = {0.900f, 0.890f, 0.870f, 1.0f};
    clrs[ImGuiCol_SeparatorHovered]     = primary_blue;
    clrs[ImGuiCol_SeparatorActive]      = primary_hover;

    clrs[ImGuiCol_Tab]                  = {0.940f, 0.940f, 0.940f, 1.0f};
    clrs[ImGuiCol_TabHovered]           = {0.900f, 0.920f, 0.950f, 1.0f};
    clrs[ImGuiCol_TabActive]            = card_bg;
}

bool begin_card(const char* str_id, float width, float height) {
    return begin_card(str_id, ImVec2(width, height));
}

bool begin_card(const char* str_id, const ImVec2& size) {
    PushStyleColor(ImGuiCol_ChildBg, card_bg);
    PushStyleColor(ImGuiCol_Border, card_border);
    PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
    PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
    PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(16.0f, 14.0f));

    bool open = BeginChild(str_id, size, true, ImGuiWindowFlags_None);
    return open;
}

void end_card() {
    EndChild();
    PopStyleVar(3);
    PopStyleColor(2);
}

bool outline_button(const char* label, const ImVec2& size) {
    PushStyleColor(ImGuiCol_Button,        ImVec4(1.0f, 1.0f, 1.0f, 0.0f));
    PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.94f, 0.95f, 0.97f, 1.0f));
    PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.88f, 0.90f, 0.94f, 1.0f));
    PushStyleColor(ImGuiCol_Text,          text_primary);
    PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);

    bool clicked = Button(label, size);

    PopStyleVar();
    PopStyleColor(4);
    return clicked;
}

bool primary_button(const char* label, const ImVec2& size) {
    PushStyleColor(ImGuiCol_Button,        primary_blue);
    PushStyleColor(ImGuiCol_ButtonHovered, primary_hover);
    PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.04f, 0.35f, 0.75f, 1.0f));
    PushStyleColor(ImGuiCol_Text,          ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
    PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);

    bool clicked = Button(label, size);

    PopStyleVar();
    PopStyleColor(4);
    return clicked;
}

bool toggle_switch(const char* label, bool* v) {
    ImVec2 p = GetCursorScreenPos();
    ImDrawList* dl = GetWindowDrawList();

    float height = GetFrameHeight();
    float width  = height * 1.8f;
    float radius = height * 0.5f;

    InvisibleButton(label, ImVec2(width, height));
    bool clicked = IsItemClicked();
    if (clicked) *v = !(*v);

    ImU32 col_bg = *v ? GetColorU32(primary_blue) : IM_COL32(200, 204, 212, 255);
    dl->AddRectFilled(p, ImVec2(p.x + width, p.y + height), col_bg, radius);

    float t_x = *v ? (p.x + width - radius) : (p.x + radius);
    dl->AddCircleFilled(ImVec2(t_x, p.y + radius), radius - 2.0f, IM_COL32(255, 255, 255, 255));

    SameLine();
    Text("%s", label);
    return clicked;
}

void pill_badge(const char* text, const ImVec4& bg_color, const ImVec4& text_color) {
    PushStyleColor(ImGuiCol_Button, bg_color);
    PushStyleColor(ImGuiCol_ButtonHovered, bg_color);
    PushStyleColor(ImGuiCol_ButtonActive, bg_color);
    PushStyleColor(ImGuiCol_Text, text_color);
    PushStyleVar(ImGuiStyleVar_FrameRounding, 12.0f);
    PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10.0f, 3.0f));

    Button(text);

    PopStyleVar(2);
    PopStyleColor(4);
}

void status_banner(const char* text, const ImVec4& bg_color, const ImVec4& text_color, bool is_success) {
    (void)is_success;
    ImVec2 size = ImVec2(GetContentRegionAvail().x, 34.0f);
    ImVec2 p = GetCursorScreenPos();
    ImDrawList* dl = GetWindowDrawList();

    ImU32 col_bg = ColorConvertFloat4ToU32(bg_color);
    ImU32 col_txt = ColorConvertFloat4ToU32(text_color);

    dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), col_bg, 6.0f);
    dl->AddText(ImVec2(p.x + 12.0f, p.y + 8.0f), col_txt, text);

    Dummy(size);
}

ImU32 get_temperature_color(double T, float alpha) {
    uint8_t a = static_cast<uint8_t>(std::clamp(alpha, 0.0f, 1.0f) * 255.0f);

    if (T < 17.0) {
        // Cold gradient: 14 C (deep navy) -> 17 C (soft cyan)
        double t = std::clamp((T - 14.0) / 3.0, 0.0, 1.0);
        uint8_t r = static_cast<uint8_t>(30  + t * 30);
        uint8_t g = static_cast<uint8_t>(90  + t * 90);
        uint8_t b = static_cast<uint8_t>(200 + t * 40);
        return IM_COL32(r, g, b, a);
    } else if (T <= 22.0) {
        // Safe comfort gradient: 17 C -> 22 C (vibrant emerald green)
        double t = (T - 17.0) / 5.0;
        uint8_t r = static_cast<uint8_t>(40  + t * 40);
        uint8_t g = static_cast<uint8_t>(190 + t * 20);
        uint8_t b = static_cast<uint8_t>(100 - t * 40);
        return IM_COL32(r, g, b, a);
    } else {
        // Hot gradient: 22 C (amber) -> 25 C (red)
        double t = std::clamp((T - 22.0) / 3.0, 0.0, 1.0);
        uint8_t r = static_cast<uint8_t>(230 + t * 20);
        uint8_t g = static_cast<uint8_t>(130 - t * 80);
        uint8_t b = static_cast<uint8_t>(40  - t * 20);
        return IM_COL32(r, g, b, a);
    }
}

} // namespace circular::theme
