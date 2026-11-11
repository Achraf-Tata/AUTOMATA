#include "ui_theme.hpp"
#include <filesystem>
#include <cmath>

namespace robot_nav::theme {

Fonts g_fonts;

void load_fonts() {
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->Clear();

    const char* font_path_reg  = "C:\\Windows\\Fonts\\segoeui.ttf";
    const char* font_path_bold = "C:\\Windows\\Fonts\\segoeuib.ttf";

    ImFontConfig cfg;
    cfg.OversampleH = 3;
    cfg.OversampleV = 2;
    cfg.PixelSnapH = false;
    cfg.RasterizerMultiply = 1.15f;

    if (std::filesystem::exists(font_path_reg)) {
        g_fonts.regular = io.Fonts->AddFontFromFileTTF(font_path_reg, 17.0f, &cfg);
        g_fonts.small   = io.Fonts->AddFontFromFileTTF(font_path_reg, 14.0f, &cfg);
        if (std::filesystem::exists(font_path_bold)) {
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
    ImGuiStyle& style = ImGui::GetStyle();

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
    style.ItemSpacing       = {10.0f, 8.0f};
    style.ItemInnerSpacing  = {6.0f, 6.0f};

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg]             = colors::bg_canvas;
    colors[ImGuiCol_ChildBg]              = colors::card_bg;
    colors[ImGuiCol_PopupBg]              = ImVec4(1.0f, 1.0f, 1.0f, 0.99f);
    colors[ImGuiCol_Border]               = colors::card_border;
    colors[ImGuiCol_BorderShadow]         = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);

    colors[ImGuiCol_FrameBg]              = ImVec4(0.98f, 0.98f, 0.99f, 1.00f);
    colors[ImGuiCol_FrameBgHovered]       = ImVec4(0.95f, 0.96f, 0.98f, 1.00f);
    colors[ImGuiCol_FrameBgActive]        = ImVec4(0.90f, 0.93f, 0.97f, 1.00f);

    colors[ImGuiCol_TitleBg]              = colors::card_bg;
    colors[ImGuiCol_TitleBgActive]        = colors::card_bg;
    colors[ImGuiCol_TitleBgCollapsed]     = colors::card_bg;

    colors[ImGuiCol_MenuBarBg]            = colors::card_bg;
    colors[ImGuiCol_ScrollbarBg]          = ImVec4(0.96f, 0.96f, 0.96f, 0.60f);
    colors[ImGuiCol_ScrollbarGrab]        = ImVec4(0.78f, 0.80f, 0.84f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.68f, 0.70f, 0.75f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabActive]  = ImVec4(0.58f, 0.60f, 0.65f, 1.00f);

    colors[ImGuiCol_CheckMark]            = colors::primary_blue;
    colors[ImGuiCol_SliderGrab]           = colors::primary_blue;
    colors[ImGuiCol_SliderGrabActive]     = colors::primary_hover;

    colors[ImGuiCol_Button]               = colors::primary_blue;
    colors[ImGuiCol_ButtonHovered]        = colors::primary_hover;
    colors[ImGuiCol_ButtonActive]         = ImVec4(0.04f, 0.35f, 0.75f, 1.00f);

    colors[ImGuiCol_Header]               = ImVec4(0.93f, 0.95f, 0.98f, 1.00f);
    colors[ImGuiCol_HeaderHovered]        = ImVec4(0.88f, 0.92f, 0.97f, 1.00f);
    colors[ImGuiCol_HeaderActive]         = ImVec4(0.82f, 0.88f, 0.95f, 1.00f);

    colors[ImGuiCol_Separator]            = ImVec4(0.88f, 0.87f, 0.85f, 1.00f);
    colors[ImGuiCol_Text]                 = colors::text_primary;
    colors[ImGuiCol_TextDisabled]         = colors::text_muted;
    colors[ImGuiCol_ModalWindowDimBg]     = ImVec4(0.10f, 0.12f, 0.16f, 0.50f);
}

bool begin_card(const char* str_id, float width, float height) {
    ImGui::PushStyleColor(ImGuiCol_ChildBg, colors::card_bg);
    ImGui::PushStyleColor(ImGuiCol_Border, colors::card_border);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(16.0f, 14.0f));

    ImGuiChildFlags child_flags = ImGuiChildFlags_Borders;
    if (height <= 0.0f) {
        child_flags |= ImGuiChildFlags_AutoResizeY;
    }
    return ImGui::BeginChild(str_id, ImVec2(width, height), child_flags,
                             ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
}

bool begin_card(const char* str_id, const ImVec2& size) {
    return begin_card(str_id, size.x, size.y);
}

void end_card() {
    ImGui::EndChild();
    ImGui::PopStyleVar(3);
    ImGui::PopStyleColor(2);
}

bool outline_button(const char* label, const ImVec2& size) {
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.94f, 0.96f, 1.00f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.88f, 0.93f, 0.99f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Border,        colors::primary_blue);
    ImGui::PushStyleColor(ImGuiCol_Text,          colors::primary_blue);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding,   5.0f);

    bool clicked = ImGui::Button(label, size);

    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(5);
    return clicked;
}

bool primary_button(const char* label, const ImVec2& size) {
    ImGui::PushStyleColor(ImGuiCol_Button,        colors::primary_blue);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, colors::primary_hover);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.04f, 0.35f, 0.75f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_Text,          ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 6.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(12.0f, 7.0f));

    if (g_fonts.bold) ImGui::PushFont(g_fonts.bold);
    bool clicked = ImGui::Button(label, size);
    if (g_fonts.bold) ImGui::PopFont();

    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(4);
    return clicked;
}

bool toggle_switch(const char* label, bool* v) {
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    float height = 20.0f;
    float width = 36.0f;
    float radius = height * 0.5f;

    ImGui::InvisibleButton(label, ImVec2(width, height));
    bool clicked = ImGui::IsItemClicked();
    if (clicked) {
        *v = !(*v);
    }

    ImU32 bg_color = *v ? ImColor(colors::primary_blue) : ImColor(0.82f, 0.84f, 0.88f, 1.0f);
    draw_list->AddRectFilled(p, ImVec2(p.x + width, p.y + height), bg_color, radius);

    float circle_x = *v ? (p.x + width - radius) : (p.x + radius);
    draw_list->AddCircleFilled(ImVec2(circle_x, p.y + radius), radius - 2.0f, IM_COL32(255, 255, 255, 255));

    ImGui::SameLine(0, 10.0f);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);

    return clicked;
}

void pill_badge(const char* text, const ImVec4& bg_color, const ImVec4& text_color) {
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    if (g_fonts.bold) ImGui::PushFont(g_fonts.bold);
    ImVec2 text_size = ImGui::CalcTextSize(text);
    float pad_x = 10.0f;
    float pad_y = 4.0f;
    ImVec2 badge_size = {text_size.x + pad_x * 2.0f, text_size.y + pad_y * 2.0f};

    draw_list->AddRectFilled(p, {p.x + badge_size.x, p.y + badge_size.y}, ImColor(bg_color), badge_size.y * 0.5f);
    draw_list->AddText({p.x + pad_x, p.y + pad_y}, ImColor(text_color), text);
    if (g_fonts.bold) ImGui::PopFont();

    ImGui::Dummy(badge_size);
}

void status_banner(const char* text, const ImVec4& bg_color, const ImVec4& text_color, bool is_success) {
    ImVec2 p = ImGui::GetCursorScreenPos();
    float avail_w = ImGui::GetContentRegionAvail().x;
    float banner_h = 34.0f;
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    draw_list->AddRectFilled(p, {p.x + avail_w, p.y + banner_h}, ImColor(bg_color), 5.0f);

    float icon_radius = 7.0f;
    ImVec2 icon_ctr = {p.x + 18.0f, p.y + banner_h * 0.5f};
    if (is_success) {
        draw_list->AddCircleFilled(icon_ctr, icon_radius, ImColor(text_color));
        draw_list->AddLine({icon_ctr.x - 3.5f, icon_ctr.y}, {icon_ctr.x - 1.0f, icon_ctr.y + 2.5f}, IM_COL32(255, 255, 255, 255), 1.8f);
        draw_list->AddLine({icon_ctr.x - 1.0f, icon_ctr.y + 2.5f}, {icon_ctr.x + 3.5f, icon_ctr.y - 2.5f}, IM_COL32(255, 255, 255, 255), 1.8f);
    } else {
        draw_list->AddCircleFilled(icon_ctr, icon_radius, ImColor(text_color));
        draw_list->AddLine({icon_ctr.x - 2.5f, icon_ctr.y - 2.5f}, {icon_ctr.x + 2.5f, icon_ctr.y + 2.5f}, IM_COL32(255, 255, 255, 255), 1.8f);
        draw_list->AddLine({icon_ctr.x + 2.5f, icon_ctr.y - 2.5f}, {icon_ctr.x - 2.5f, icon_ctr.y + 2.5f}, IM_COL32(255, 255, 255, 255), 1.8f);
    }

    if (g_fonts.bold) ImGui::PushFont(g_fonts.bold);
    draw_list->AddText({p.x + 34.0f, p.y + (banner_h - ImGui::GetTextLineHeight()) * 0.5f}, ImColor(text_color), text);
    if (g_fonts.bold) ImGui::PopFont();

    ImGui::Dummy(ImVec2(avail_w, banner_h));
}

} // namespace robot_nav::theme
