#include "count_input.hpp"
#include "thermostat.hpp"
#include "thermostat_font.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_stdlib.h>

#include <algorithm>
#include <charconv>
#include <deque>
#include <exception>
#include <iostream>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

using namespace thermostat;

// ---------------------------------------------------------------------------
// Design System Color Palette (Warm Cream, Sage Green, Forest Green)
// ---------------------------------------------------------------------------
constexpr ImVec4 bg_canvas_color       = ImVec4(0.965f, 0.957f, 0.941f, 1.0f); // Warm parchment #F6F4F0
constexpr ImVec4 bg_card_color         = ImVec4(0.984f, 0.980f, 0.969f, 1.0f); // Soft card white #FAF8F5
constexpr ImVec4 border_subtle_color   = ImVec4(0.880f, 0.863f, 0.835f, 1.0f); // Stone border #E0DCD5
constexpr ImVec4 text_primary_color    = ImVec4(0.118f, 0.110f, 0.102f, 1.0f); // Charcoal #1F1C1A
constexpr ImVec4 text_muted_color      = ImVec4(0.478f, 0.451f, 0.420f, 1.0f); // Warm grey #7A736B
constexpr ImVec4 brand_green_color     = ImVec4(0.082f, 0.306f, 0.235f, 1.0f); // Forest green #154E3C
constexpr ImVec4 brand_green_hover     = ImVec4(0.110f, 0.380f, 0.294f, 1.0f); // Forest hover #1C614B
constexpr ImVec4 brand_green_active    = ImVec4(0.060f, 0.235f, 0.180f, 1.0f); // Forest active #0F3C2E
constexpr ImVec4 btn_light_color       = ImVec4(0.945f, 0.933f, 0.910f, 1.0f); // Light cream btn #F1EEE8
constexpr ImVec4 btn_light_hover       = ImVec4(0.890f, 0.875f, 0.847f, 1.0f); // Light cream hover
constexpr ImVec4 btn_light_active      = ImVec4(0.835f, 0.820f, 0.788f, 1.0f); // Light cream active
constexpr ImVec4 success_green_color   = ImVec4(0.086f, 0.502f, 0.239f, 1.0f); // Rich green text #16803D
constexpr ImVec4 error_red_color       = ImVec4(0.820f, 0.220f, 0.200f, 1.0f); // Error red

// Drawing Colors
constexpr ImU32 plot_bg_color          = IM_COL32(234, 243, 232, 255); // Pale sage #EAF3E8
constexpr ImU32 plot_grid_color        = IM_COL32(212, 226, 210, 255); // Subtle green grid
constexpr ImU32 plot_border_color      = IM_COL32(200, 216, 198, 255); // Plot border
constexpr ImU32 trajectory_amber_color = IM_COL32(230, 138, 0, 255);   // Golden trajectory line #E68A00

struct Fonts {
    ImFont* title;       // 26px bold
    ImFont* heading;     // 19px bold
    ImFont* body_bold;   // 16px bold
    ImFont* stat_value;  // 22px bold
    ImFont* small_muted; // 13px
};

enum Profile { ramp, alternating, cold, warm, custom };
constexpr const char* profiles[] = {
    "Ramp from 17 to 22 °C",
    "Alternating 17 and 22 °C",
    "All rooms at 17 °C",
    "All rooms at 22 °C",
    "Custom temperatures",
};

struct TemperatureSample {
    float coldest;
    float mean;
    float warmest;
};

TemperatureSample summarize(std::span<const double> temperatures) {
    const auto [coldest, warmest] = std::minmax_element(temperatures.begin(), temperatures.end());
    const double mean = std::accumulate(temperatures.begin(), temperatures.end(), 0.0)
                      / static_cast<double>(temperatures.size());
    return {static_cast<float>(*coldest), static_cast<float>(mean), static_cast<float>(*warmest)};
}

std::vector<double> initial_temperatures(std::size_t count, int profile) {
    check_live_room_limit(count);
    std::vector<double> temperatures(count);
    for (std::size_t i = 0; i < count; ++i) {
        switch (profile) {
        case alternating: temperatures[i] = i % 2 == 0 ? min_temperature : max_temperature; break;
        case cold: temperatures[i] = min_temperature; break;
        case warm: temperatures[i] = max_temperature; break;
        default:
            temperatures[i] = min_temperature
                            + (max_temperature - min_temperature)
                            * (static_cast<double>(i) / static_cast<double>(std::max<std::size_t>(count - 1, 1)));
            break;
        }
    }
    return temperatures;
}

std::vector<double> read_temperatures(std::string input) {
    std::replace(input.begin(), input.end(), ',', ' ');
    std::istringstream entries(std::move(input));
    std::vector<double> temperatures;
    for (std::string entry; entries >> entry;) {
        double temperature{};
        const char* end = entry.data() + entry.size();
        const char* start = entry.data() + (entry.front() == '+' ? 1 : 0);
        const auto [parsed, error] = std::from_chars(start, end, temperature);
        if (error != std::errc{} || parsed != end || !quantize(temperature)) {
            throw std::invalid_argument("Enter a temperature from 17 to 22 °C for room "
                                        + std::to_string(temperatures.size() + 1) + ".");
        }
        check_live_room_limit(temperatures.size() + 1);
        temperatures.push_back(temperature);
    }
    if (temperatures.size() < 3) {
        throw std::invalid_argument("Enter at least three temperatures, one per room.");
    }
    return temperatures;
}

// Crisp vector thermometer icon drawn directly with ImDrawList
void draw_thermometer_icon(ImDrawList* draw, ImVec2 pos, float height, ImU32 color) {
    const float stem_w = height * 0.28f;
    const float bulb_r = height * 0.26f;
    const float bulb_cx = pos.x + bulb_r + 2.0f;
    const float bulb_cy = pos.y + height - bulb_r - 2.0f;
    const float stem_top = pos.y + stem_w * 0.5f + 2.0f;
    const float stem_left = bulb_cx - stem_w * 0.5f;
    const float stem_right = bulb_cx + stem_w * 0.5f;

    // Tube background/border
    draw->AddRectFilled(ImVec2(stem_left, stem_top), ImVec2(stem_right, bulb_cy), color, stem_w * 0.5f);
    draw->AddCircleFilled(ImVec2(bulb_cx, bulb_cy), bulb_r, color, 16);

    // Inner highlight
    draw->AddCircleFilled(ImVec2(bulb_cx, bulb_cy), bulb_r * 0.55f, IM_COL32(255, 255, 255, 200), 12);
}

void draw_temperature_history(const std::deque<TemperatureSample>& history, const Fonts& fonts) {
    const auto sample = history.empty() ? TemperatureSample{18.125f, 18.125f, 18.125f} : history.back();

    // Section Header with inline min/mean/max indicators matching the UI
    ImGui::PushFont(fonts.heading);
    ImGui::TextUnformatted("Temperature history");
    ImGui::PopFont();

    ImGui::SameLine();
    ImGui::TextDisabled("  |  ");
    ImGui::SameLine();

    ImGui::TextUnformatted("Min ");
    ImGui::SameLine();
    ImGui::PushFont(fonts.body_bold);
    ImGui::TextColored(success_green_color, "%.3f °C", sample.coldest);
    ImGui::PopFont();

    ImGui::SameLine(0.0f, 18.0f);
    ImGui::TextUnformatted("Mean ");
    ImGui::SameLine();
    ImGui::PushFont(fonts.body_bold);
    ImGui::TextColored(success_green_color, "%.3f °C", sample.mean);
    ImGui::PopFont();

    ImGui::SameLine(0.0f, 18.0f);
    ImGui::TextUnformatted("Max ");
    ImGui::SameLine();
    ImGui::PushFont(fonts.body_bold);
    ImGui::TextColored(success_green_color, "%.3f °C", sample.warmest);
    ImGui::PopFont();

    ImGui::Spacing();

    const auto origin = ImGui::GetCursorScreenPos();
    const auto width = ImGui::GetContentRegionAvail().x;
    const float height = 150.0f;
    ImGui::Dummy(ImVec2(width, height));

    const float label_w = ImGui::CalcTextSize("22 °C").x + 16.0f;
    const float left = origin.x + label_w + 8.0f;
    const float right = origin.x + width - 12.0f;
    const float top = origin.y + 12.0f;
    const float bottom = origin.y + height - 12.0f;

    const auto y_of = [top, bottom](float temperature) {
        return bottom - (temperature - 17.0f) / 5.0f * (bottom - top);
    };

    auto* draw = ImGui::GetWindowDrawList();

    // Green canvas background & border
    draw->AddRectFilled(ImVec2(left, top), ImVec2(right, bottom), plot_bg_color, 7.0f);
    draw->AddRect(ImVec2(left, top), ImVec2(right, bottom), plot_border_color, 7.0f, 0, 1.0f);

    // Horizontal grid lines across [17, 22] C
    for (int deg = 17; deg <= 22; ++deg) {
        const float y = y_of(static_cast<float>(deg));
        draw->AddLine(ImVec2(left, y), ImVec2(right, y), plot_grid_color, 1.0f);
        if (deg == 17 || deg == 22) {
            const char* label = (deg == 17) ? "17 °C" : "22 °C";
            draw->AddText(ImVec2(origin.x + 4.0f, y - ImGui::GetTextLineHeight() * 0.5f),
                          IM_COL32(80, 75, 70, 255), label);
        }
    }

    if (history.empty()) return;

    // Draw trajectory lines in golden amber with end point circle
    const auto draw_line = [&](float TemperatureSample::*field) {
        const float dx = (right - left) / static_cast<float>(std::max<std::size_t>(history.size() - 1, 1));
        ImVec2 previous(left, y_of(history.front().*field));
        for (std::size_t i = 1; i < history.size(); ++i) {
            const ImVec2 point(left + dx * static_cast<float>(i), y_of(history[i].*field));
            draw->AddLine(previous, point, trajectory_amber_color, 2.5f);
            previous = point;
        }
        draw->AddCircleFilled(previous, 4.0f, trajectory_amber_color);
        draw->AddCircle(previous, 4.0f, IM_COL32(255, 255, 255, 220), 12, 1.5f);
    };

    draw_line(&TemperatureSample::mean);
}

std::string describe_successors(BinMask mask) {
    std::string result;
    for (int bin = 0; bin < bin_count; ++bin) {
        if (mask & bin_bit(bin)) {
            if (!result.empty()) {
                result += ", ";
            }
            result += std::to_string(bin);
        }
    }
    if (mask & unsafe) {
        if (!result.empty()) {
            result += " + ";
        }
        result += "unsafe";
    }
    return result.empty() ? "-" : result;
}

class Dashboard {
public:
    explicit Dashboard(Fonts fonts)
        : fonts_(fonts), controller_(build_controller()), building_(initial_temperatures(4, ramp)) {
        record_sample();
    }

    void draw(float frame_seconds) {
        if (running_) {
            elapsed_ += std::min(frame_seconds, 0.25f);
            const float tick_time = 1.0f / static_cast<float>(ticks_per_second_);
            while (running_ && elapsed_ >= tick_time) {
                elapsed_ -= tick_time;
                advance();
            }
        }

        const auto* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::SetNextWindowScroll(ImVec2(0.0f, 0.0f));

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(24.0f, 20.0f));
        ImGui::Begin("Building thermostat", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove
                     | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus);
        ImGui::PopStyleVar();

        // 1. Header (Icon + Title + Subtitle)
        draw_header();
        ImGui::Spacing();

        // 2. Navigation Tabs (Simulation vs Controller)
        draw_tabs();
        ImGui::Spacing();

        // 3. Main Content Body (Allocates space for bottom footer)
        const float footer_height = 80.0f;
        const float content_height = ImGui::GetContentRegionAvail().y - footer_height - 12.0f;

        if (ImGui::BeginChild("MainContentRegion", ImVec2(0.0f, content_height), false)) {
            if (active_tab_ == 0) {
                draw_simulation_view();
            } else {
                draw_controller_view();
            }
        }
        ImGui::EndChild();

        // 4. Footer Timings Bar (Full-width card at bottom)
        draw_footer_timings();

        ImGui::End();
    }

private:
    const Fonts fonts_;
    const Controller controller_;
    Building building_;
    std::deque<TemperatureSample> history_;
    int room_count_int_ = 4;
    std::optional<std::size_t> certified_rooms_;
    int profile_ = ramp;
    int ticks_per_second_ = 30;
    bool running_ = false;
    float elapsed_ = 0.0f;
    std::string custom_temperatures_ = "17, 18.5, 20, 22";
    std::string setup_error_;
    std::string error_;
    int active_tab_ = 0; // 0 = Simulation, 1 = Controller

    void draw_header() {
        auto* draw = ImGui::GetWindowDrawList();
        const auto cursor = ImGui::GetCursorScreenPos();
        draw_thermometer_icon(draw, ImVec2(cursor.x, cursor.y + 4.0f), 32.0f, ImGui::GetColorU32(brand_green_color));

        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 38.0f);
        ImGui::PushFont(fonts_.title);
        ImGui::TextUnformatted("Building thermostat");
        ImGui::PopFont();

        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 38.0f);
        ImGui::TextColored(text_muted_color, "%zu rooms in a ring  ·  Comfort range %d–%d °C",
                           building_.temperatures().size(), min_temperature, max_temperature);
    }

    void draw_tabs() {
        const float tab_w = 120.0f;
        const float tab_h = 32.0f;

        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 7.0f);
        ImGui::PushFont(fonts_.body_bold);

        // Simulation Tab
        if (active_tab_ == 0) {
            ImGui::PushStyleColor(ImGuiCol_Button, brand_green_color);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, brand_green_hover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, brand_green_active);
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, btn_light_color);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, btn_light_hover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, btn_light_active);
            ImGui::PushStyleColor(ImGuiCol_Text, text_primary_color);
        }
        if (ImGui::Button("Simulation", ImVec2(tab_w, tab_h))) {
            active_tab_ = 0;
        }
        ImGui::PopStyleColor(4);

        ImGui::SameLine(0.0f, 8.0f);

        // Controller Tab
        if (active_tab_ == 1) {
            ImGui::PushStyleColor(ImGuiCol_Button, brand_green_color);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, brand_green_hover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, brand_green_active);
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, btn_light_color);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, btn_light_hover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, btn_light_active);
            ImGui::PushStyleColor(ImGuiCol_Text, text_primary_color);
        }
        if (ImGui::Button("Controller", ImVec2(tab_w, tab_h))) {
            active_tab_ = 1;
        }
        ImGui::PopStyleColor(4);

        ImGui::PopFont();
        ImGui::PopStyleVar();
    }

    void record_sample() {
        history_.push_back(summarize(building_.temperatures()));
        if (history_.size() > 180) {
            history_.pop_front();
        }
    }

    void apply_setup() {
        try {
            const std::size_t count = static_cast<std::size_t>(std::max(3, room_count_int_));
            Building next(profile_ == custom ? read_temperatures(custom_temperatures_)
                                             : initial_temperatures(count, profile_));
            std::deque<TemperatureSample> history{summarize(next.temperatures())};
            building_ = std::move(next);
            history_.swap(history);
            running_ = false;
            elapsed_ = 0.0f;
            certified_rooms_ = count;
            setup_error_.clear();
            error_.clear();
        } catch (const std::exception& error) {
            setup_error_ = error.what();
        }
    }

    void certify_size() {
        try {
            certified_rooms_ = static_cast<std::size_t>(std::max(3, room_count_int_));
            setup_error_.clear();
        } catch (const std::exception& error) {
            setup_error_ = error.what();
        }
    }

    void advance() {
        try {
            building_.step(controller_.policy);
            record_sample();
        } catch (const std::exception& error) {
            running_ = false;
            error_ = error.what();
        }
    }

    void draw_simulation_view() {
        const float left_col_width = 280.0f;

        // --- Left Card: Initial Conditions ---
        ImGui::PushStyleColor(ImGuiCol_ChildBg, bg_card_color);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(18.0f, 18.0f));

        if (ImGui::BeginChild("InitialConditionsCard", ImVec2(left_col_width, 0.0f), ImGuiChildFlags_Borders)) {
            ImGui::PushFont(fonts_.heading);
            ImGui::TextUnformatted("Initial conditions");
            ImGui::PopFont();
            ImGui::Spacing();

            // Starting temperatures
            ImGui::TextColored(text_muted_color, "Starting temperatures");
            ImGui::SetNextItemWidth(-1.0f);
            if (ImGui::Combo("##profile", &profile_, profiles, IM_ARRAYSIZE(profiles))) {
                setup_error_.clear();
            }
            ImGui::Spacing();

            if (profile_ == custom) {
                ImGui::TextColored(text_muted_color, "Room temperatures (°C)");
                ImGui::InputTextMultiline("##custom-temps", &custom_temperatures_, ImVec2(-1.0f, 65.0f));
                ImGui::TextWrapped("Values in [17, 22] separated by commas.");
            } else {
                // Room count stepper
                ImGui::TextColored(text_muted_color, "Room count");
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputInt("##room-count-stepper", &room_count_int_, 1, 5)) {
                    if (room_count_int_ < 3) room_count_int_ = 3;
                    if (room_count_int_ > 1000000) room_count_int_ = 1000000;
                    setup_error_.clear();
                }
            }
            ImGui::Spacing();
            ImGui::Spacing();

            // Buttons: Apply & reset, Certify
            const float btn_w = (ImGui::GetContentRegionAvail().x - 10.0f) * 0.55f;
            const float cert_w = ImGui::GetContentRegionAvail().x - btn_w - 10.0f;

            // Apply & reset button (Green)
            ImGui::PushStyleColor(ImGuiCol_Button, brand_green_color);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, brand_green_hover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, brand_green_active);
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
            ImGui::PushFont(fonts_.body_bold);
            if (ImGui::Button("Apply & reset", ImVec2(btn_w, 34.0f))) {
                apply_setup();
            }
            ImGui::PopFont();
            ImGui::PopStyleColor(4);

            ImGui::SameLine(0.0f, 10.0f);

            // Certify button (Light cream)
            ImGui::PushStyleColor(ImGuiCol_Button, btn_light_color);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, btn_light_hover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, btn_light_active);
            ImGui::PushStyleColor(ImGuiCol_Text, text_primary_color);
            ImGui::PushStyleColor(ImGuiCol_Border, border_subtle_color);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
            if (ImGui::Button("Certify", ImVec2(cert_w, 34.0f))) {
                certify_size();
            }
            ImGui::PopStyleVar();
            ImGui::PopStyleColor(5);

            ImGui::Spacing();
            ImGui::Spacing();

            // Certified status message
            const auto rooms = certified_rooms_.value_or(building_.temperatures().size());
            const double pct = certified_percentage(controller_.policy, rooms);
            ImGui::PushFont(fonts_.body_bold);
            ImGui::TextColored(success_green_color, "%.0f%% of initial cells certified", pct);
            ImGui::PopFont();

            if (!setup_error_.empty()) {
                ImGui::Spacing();
                ImGui::TextColored(error_red_color, "%s", setup_error_.c_str());
            }
        }
        ImGui::EndChild();
        ImGui::PopStyleVar(2);
        ImGui::PopStyleColor();

        ImGui::SameLine(0.0f, 20.0f);

        // --- Right Area: Live Simulation, History, and Table ---
        if (ImGui::BeginChild("SimulationRightArea", ImVec2(0.0f, 0.0f), false)) {
            // Control Bar: Run/Pause, Step, Speed slider, Tick counter
            draw_control_bar();
            ImGui::Spacing();

            // Temperature history plot
            draw_temperature_history(history_, fonts_);
            ImGui::Spacing();

            // Room status table
            draw_room_table();
        }
        ImGui::EndChild();
    }

    void draw_control_bar() {
        ImGui::PushFont(fonts_.body_bold);

        // Run / Pause button (Green)
        ImGui::PushStyleColor(ImGuiCol_Button, brand_green_color);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, brand_green_hover);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, brand_green_active);
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        const bool run_clicked = ImGui::Button(running_ ? " Pause " : "  Run  ", ImVec2(90.0f, 32.0f));
        ImGui::PopStyleColor(4);
        if (run_clicked) {
            running_ = !running_;
            elapsed_ = 0.0f;
            error_.clear();
        }

        ImGui::SameLine(0.0f, 10.0f);

        // Step button (Light button)
        ImGui::BeginDisabled(running_);
        ImGui::PushStyleColor(ImGuiCol_Button, btn_light_color);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, btn_light_hover);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, btn_light_active);
        ImGui::PushStyleColor(ImGuiCol_Text, text_primary_color);
        ImGui::PushStyleColor(ImGuiCol_Border, border_subtle_color);
        ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
        if (ImGui::Button("Step", ImVec2(76.0f, 32.0f))) {
            error_.clear();
            advance();
        }
        ImGui::PopStyleVar();
        ImGui::PopStyleColor(5);
        ImGui::EndDisabled();

        ImGui::SameLine(0.0f, 24.0f);

        // Speed slider
        ImGui::AlignTextToFramePadding();
        ImGui::Text("%d ticks/s", ticks_per_second_);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(140.0f);
        ImGui::SliderInt("##speed", &ticks_per_second_, 1, 60, "");

        ImGui::SameLine(0.0f, 24.0f);
        ImGui::TextDisabled("|");
        ImGui::SameLine(0.0f, 24.0f);

        // Tick counter
        ImGui::Text("Tick %zu", building_.tick());
        ImGui::PopFont();

        if (!error_.empty()) {
            ImGui::Spacing();
            ImGui::TextColored(error_red_color, "%s", error_.c_str());
        }
    }

    void draw_room_table() {
        const auto temperatures = building_.temperatures();
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
        ImGui::PushStyleColor(ImGuiCol_ChildBg, bg_card_color);

        if (ImGui::BeginTable("RoomsTable", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY
                                             | ImGuiTableFlags_BordersOuterH | ImGuiTableFlags_BordersInnerV
                                             | ImGuiTableFlags_SizingStretchProp,
                              ImVec2(0.0f, ImGui::GetContentRegionAvail().y))) {
            ImGui::TableSetupColumn("Room", ImGuiTableColumnFlags_WidthStretch, 0.9f);
            ImGui::TableSetupColumn("Temperature (°C)", ImGuiTableColumnFlags_WidthStretch, 1.3f);
            ImGui::TableSetupColumn("Current bin", ImGuiTableColumnFlags_WidthStretch, 1.0f);
            ImGui::TableSetupColumn("Next heater", ImGuiTableColumnFlags_WidthStretch, 1.0f);
            ImGui::TableSetupScrollFreeze(0, 1);

            ImGui::PushFont(fonts_.body_bold);
            ImGui::TableHeadersRow();
            ImGui::PopFont();

            ImGuiListClipper clipper;
            clipper.Begin(static_cast<int>(temperatures.size()));
            while (clipper.Step()) {
                for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
                    ImGui::TableNextRow(ImGuiTableRowFlags_None, 24.0f);
                    ImGui::TableNextColumn();
                    ImGui::Text("Room %d", i + 1);

                    ImGui::TableNextColumn();
                    ImGui::Text("%.3f", temperatures[i]);

                    ImGui::TableNextColumn();
                    ImGui::Text("%d", quantize(temperatures[i]).value_or(-1));

                    ImGui::TableNextColumn();
                    if (const auto action = choose_action(controller_.policy, temperatures[i])) {
                        ImGui::Text("%.1f", heater_setting(*action));
                    } else {
                        ImGui::TextUnformatted("-");
                    }
                }
            }
            ImGui::EndTable();
        }
        ImGui::PopStyleColor();
        ImGui::PopStyleVar();
    }

    void draw_footer_timings() const {
        const auto origin = ImGui::GetCursorScreenPos();
        const float width = ImGui::GetContentRegionAvail().x;
        const float height = 68.0f;

        auto* draw = ImGui::GetWindowDrawList();
        // Top separator line
        draw->AddLine(ImVec2(origin.x, origin.y), ImVec2(origin.x + width, origin.y),
                      ImGui::GetColorU32(border_subtle_color), 1.0f);

        ImGui::Dummy(ImVec2(width, 6.0f));

        const float col_w = width / 3.0f;
        const auto metric = [&](int col_idx, const char* label, double us_val) {
            ImGui::SetCursorScreenPos(ImVec2(origin.x + col_idx * col_w + 14.0f, origin.y + 12.0f));
            ImGui::BeginGroup();
            ImGui::PushFont(fonts_.small_muted);
            ImGui::TextColored(text_muted_color, "%s", label);
            ImGui::PopFont();

            ImGui::PushFont(fonts_.stat_value);
            ImGui::Text("%.3f µs", us_val);
            ImGui::PopFont();
            ImGui::EndGroup();

            if (col_idx < 2) {
                const float sep_x = origin.x + (col_idx + 1) * col_w;
                draw->AddLine(ImVec2(sep_x, origin.y + 12.0f), ImVec2(sep_x, origin.y + height - 8.0f),
                              ImGui::GetColorU32(border_subtle_color), 1.0f);
            }
        };

        metric(0, "Abstraction", controller_.abstraction_us);
        metric(1, "Synthesis", controller_.synthesis_us);
        metric(2, "Total", controller_.abstraction_us + controller_.synthesis_us);
    }

    void draw_controller_view() const {
        const auto rooms = certified_rooms_.value_or(building_.temperatures().size());
        const double percentage = certified_percentage(controller_.policy, rooms);

        ImGui::PushFont(fonts_.heading);
        ImGui::TextUnformatted("Decentralized Assume-Guarantee Contract");
        ImGui::PopFont();
        ImGui::TextColored(success_green_color, "%.2f%% of initial cells certified for %zu rooms.", percentage, rooms);
        ImGui::TextWrapped("Each room measures its own local temperature and applies the shared contract policy. "
                           "The circular composition guarantees safety across the entire building.");
        ImGui::Spacing();

        ImGui::PushFont(fonts_.heading);
        ImGui::TextUnformatted("Successor bins by heater setting");
        ImGui::PopFont();
        ImGui::TextDisabled("Unsafe marks a possible successor temperature outside [17, 22] °C.");
        ImGui::Spacing();

        if (ImGui::BeginTable("TransitionTable", 6, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH
                                                  | ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableSetupColumn("Bin", ImGuiTableColumnFlags_WidthFixed, 40.0f);
            ImGui::TableSetupColumn("Range (°C)", ImGuiTableColumnFlags_WidthFixed, 110.0f);
            ImGui::TableSetupColumn("Heater 0.0");
            ImGui::TableSetupColumn("Heater 0.3");
            ImGui::TableSetupColumn("Heater 0.6");
            ImGui::TableSetupColumn("Allowed", ImGuiTableColumnFlags_WidthFixed, 70.0f);

            ImGui::PushFont(fonts_.body_bold);
            ImGui::TableHeadersRow();
            ImGui::PopFont();

            for (int bin = 0; bin < bin_count; ++bin) {
                const double lower = min_temperature + static_cast<double>(bin) / bins_per_degree;
                const double upper = lower + 1.0 / bins_per_degree;
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("%d", bin);
                ImGui::TableNextColumn();
                ImGui::Text("[%.1f, %.1f%c", lower, upper, bin == bin_count - 1 ? ']' : ')');

                std::string allowed;
                for (int action = 0; action < action_count; ++action) {
                    ImGui::TableNextColumn();
                    const BinMask successors = controller_.transitions[bin][action];
                    ImGui::TextWrapped("%s", describe_successors(successors).c_str());
                    if (controller_.policy.allowed[bin] & action_bit(action)) {
                        if (!allowed.empty()) {
                            allowed += ", ";
                        }
                        const auto tenths = heater_tenths[action];
                        allowed += std::to_string(tenths / 10) + "." + std::to_string(tenths % 10);
                    }
                }
                ImGui::TableNextColumn();
                ImGui::PushFont(fonts_.body_bold);
                ImGui::TextColored(success_green_color, "%s", allowed.empty() ? "-" : allowed.c_str());
                ImGui::PopFont();
            }
            ImGui::EndTable();
        }
    }
};

Fonts style_interface() {
    ImGui::StyleColorsLight();
    auto& style = ImGui::GetStyle();

    style.WindowPadding     = ImVec2(24.0f, 18.0f);
    style.FramePadding      = ImVec2(10.0f, 6.0f);
    style.ItemSpacing       = ImVec2(12.0f, 9.0f);
    style.CellPadding       = ImVec2(12.0f, 7.0f);
    style.FrameRounding     = 6.0f;
    style.ChildRounding     = 8.0f;
    style.TabRounding       = 6.0f;
    style.ScrollbarRounding = 6.0f;
    style.ScrollbarSize     = 13.0f;

    auto& colors = style.Colors;
    colors[ImGuiCol_Text]                  = text_primary_color;
    colors[ImGuiCol_TextDisabled]          = text_muted_color;
    colors[ImGuiCol_WindowBg]              = bg_canvas_color;
    colors[ImGuiCol_ChildBg]               = bg_card_color;
    colors[ImGuiCol_PopupBg]               = bg_card_color;
    colors[ImGuiCol_Border]                = border_subtle_color;
    colors[ImGuiCol_BorderShadow]          = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    colors[ImGuiCol_FrameBg]               = bg_card_color;
    colors[ImGuiCol_FrameBgHovered]        = ImVec4(0.95f, 0.94f, 0.92f, 1.0f);
    colors[ImGuiCol_FrameBgActive]         = ImVec4(0.92f, 0.91f, 0.88f, 1.0f);
    colors[ImGuiCol_TitleBg]               = bg_canvas_color;
    colors[ImGuiCol_TitleBgActive]         = bg_canvas_color;
    colors[ImGuiCol_Button]                = btn_light_color;
    colors[ImGuiCol_ButtonHovered]         = btn_light_hover;
    colors[ImGuiCol_ButtonActive]          = btn_light_active;
    colors[ImGuiCol_Header]                = btn_light_color;
    colors[ImGuiCol_HeaderHovered]         = btn_light_hover;
    colors[ImGuiCol_HeaderActive]          = btn_light_active;
    colors[ImGuiCol_SliderGrab]            = brand_green_color;
    colors[ImGuiCol_SliderGrabActive]      = brand_green_active;
    colors[ImGuiCol_ScrollbarBg]           = bg_canvas_color;
    colors[ImGuiCol_ScrollbarGrab]         = ImVec4(0.80f, 0.78f, 0.75f, 1.0f);
    colors[ImGuiCol_ScrollbarGrabHovered]  = ImVec4(0.70f, 0.68f, 0.65f, 1.0f);
    colors[ImGuiCol_ScrollbarGrabActive]   = brand_green_color;
    colors[ImGuiCol_CheckMark]             = brand_green_color;
    colors[ImGuiCol_TableHeaderBg]         = ImVec4(0.92f, 0.90f, 0.87f, 1.0f);
    colors[ImGuiCol_TableBorderStrong]     = border_subtle_color;
    colors[ImGuiCol_TableBorderLight]      = ImVec4(0.90f, 0.88f, 0.85f, 1.0f);
    colors[ImGuiCol_TableRowBg]            = bg_card_color;
    colors[ImGuiCol_TableRowBgAlt]         = ImVec4(0.95f, 0.94f, 0.91f, 1.0f);

    auto& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImFontConfig font;
    font.FontDataOwnedByAtlas = false;
    const auto load = [&](float size) {
        return io.Fonts->AddFontFromMemoryTTF(thermostat_font, static_cast<int>(sizeof(thermostat_font)), size, &font);
    };

    io.FontDefault = load(15.0f);
    return {
        load(26.0f), // title
        load(18.0f), // heading
        load(15.0f), // body_bold
        load(22.0f), // stat_value
        load(13.0f)  // small_muted
    };
}

} // namespace

int main() {
    glfwSetErrorCallback([](int, const char* message) { std::cerr << message << '\n'; });
    if (!glfwInit()) {
        return 1;
    }
#ifdef __APPLE__
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#else
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif
    GLFWwindow* window = glfwCreateWindow(1060, 740, "Building thermostat", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return 1;
    }

    glfwSetWindowSizeLimits(window, 820, 600, GLFW_DONT_CARE, GLFW_DONT_CARE);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;
    const Fonts fonts = style_interface();

    const bool input_ready = ImGui_ImplGlfw_InitForOpenGL(window, true);
    const bool renderer_ready = input_ready && ImGui_ImplOpenGL3_Init(glsl_version);
    int status = 1;
    if (renderer_ready) {
        try {
            Dashboard dashboard(fonts);
            while (!glfwWindowShouldClose(window)) {
                glfwPollEvents();
                ImGui_ImplOpenGL3_NewFrame();
                ImGui_ImplGlfw_NewFrame();
                ImGui::NewFrame();
                dashboard.draw(ImGui::GetIO().DeltaTime);
                ImGui::Render();

                int width{};
                int height{};
                glfwGetFramebufferSize(window, &width, &height);
                glViewport(0, 0, width, height);
                glClearColor(0.965f, 0.957f, 0.941f, 1.0f); // Match canvas bg
                glClear(GL_COLOR_BUFFER_BIT);
                ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
                glfwSwapBuffers(window);
            }
            status = 0;
        } catch (const std::exception& error) {
            std::cerr << error.what() << '\n';
        }
        ImGui_ImplOpenGL3_Shutdown();
    } else {
        std::cerr << "Could not initialize the thermostat interface.\n";
    }
    if (input_ready) {
        ImGui_ImplGlfw_Shutdown();
    }
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return status;
}
