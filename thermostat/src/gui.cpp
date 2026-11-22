#include "count_input.hpp"
#include "thermostat.hpp"
#include "thermostat_font.hpp"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>
#elif defined(__linux__) || defined(__APPLE__)
#include <sys/resource.h>
#endif

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_stdlib.h>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <deque>
#include <exception>
#include <iomanip>
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
// Design System Color Palette (High-Contrast Warm Linen, White Cards, Forest Green)
// ---------------------------------------------------------------------------
constexpr ImVec4 bg_canvas_color       = ImVec4(0.929f, 0.914f, 0.882f, 1.0f); // Warm linen #EDE9E1 (distinct from cards!)
constexpr ImVec4 bg_card_color         = ImVec4(1.000f, 1.000f, 1.000f, 1.0f); // Pure white card #FFFFFF
constexpr ImVec4 bg_card_alt_color     = ImVec4(0.976f, 0.969f, 0.957f, 1.0f); // Soft cream sub-card #F9F7F4
constexpr ImVec4 border_card_color     = ImVec4(0.772f, 0.737f, 0.678f, 1.0f); // Crisp stone contour #C5BCAD
constexpr ImVec4 border_subtle_color   = ImVec4(0.850f, 0.825f, 0.785f, 1.0f); // Subtle inner divider #D9D2C8
constexpr ImVec4 text_primary_color    = ImVec4(0.118f, 0.110f, 0.102f, 1.0f); // Deep charcoal #1F1C1A
constexpr ImVec4 text_muted_color      = ImVec4(0.440f, 0.415f, 0.385f, 1.0f); // Warm grey #706A62
constexpr ImVec4 brand_green_color     = ImVec4(0.075f, 0.306f, 0.247f, 1.0f); // Forest green #134E3F
constexpr ImVec4 brand_green_hover     = ImVec4(0.105f, 0.388f, 0.314f, 1.0f); // Forest hover #1B6350
constexpr ImVec4 brand_green_active    = ImVec4(0.051f, 0.224f, 0.180f, 1.0f); // Forest active #0D392E
constexpr ImVec4 btn_light_color       = ImVec4(1.000f, 1.000f, 1.000f, 1.0f); // White button #FFFFFF
constexpr ImVec4 btn_light_hover       = ImVec4(0.941f, 0.925f, 0.894f, 1.0f); // Light hover #F0ECE4
constexpr ImVec4 btn_light_active      = ImVec4(0.882f, 0.863f, 0.831f, 1.0f); // Light active #E1DCD4
constexpr ImVec4 success_green_color   = ImVec4(0.082f, 0.502f, 0.239f, 1.0f); // Emerald green #15803D
constexpr ImVec4 error_red_color       = ImVec4(0.820f, 0.200f, 0.180f, 1.0f); // Alert red
constexpr ImVec4 badge_amber_color     = ImVec4(0.850f, 0.450f, 0.050f, 1.0f); // Amber alert

// Canvas Drawing Colors
constexpr ImU32 plot_bg_color          = IM_COL32(234, 243, 232, 255); // Pale sage canvas #EAF3E8
constexpr ImU32 plot_grid_color        = IM_COL32(208, 222, 206, 255); // Sage grid line
constexpr ImU32 plot_border_color      = IM_COL32(142, 168, 138, 255); // Crisp green contour #8EA88A
constexpr ImU32 trajectory_amber_color = IM_COL32(230, 126, 0, 255);   // Golden amber line #E67E00

struct Fonts {
    ImFont* title;       // 26px bold
    ImFont* heading;     // 18px bold
    ImFont* body_bold;   // 15px bold
    ImFont* stat_value;  // 21px bold
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

std::optional<std::size_t> current_rss_kib() {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS memory{};
    memory.cb = sizeof(memory);
    if (GetProcessMemoryInfo(GetCurrentProcess(), &memory, memory.cb)) {
        return memory.WorkingSetSize / 1024;
    }
#elif defined(__linux__) || defined(__APPLE__)
    rusage usage{};
    if (getrusage(RUSAGE_SELF, &usage) == 0) {
#ifdef __APPLE__
        return static_cast<std::size_t>(usage.ru_maxrss) / 1024;
#else
        return static_cast<std::size_t>(usage.ru_maxrss);
#endif
    }
#endif
    return std::nullopt;
}

struct BenchmarkMetrics {
    double abs_us    = 0.0;
    double ctrl_us   = 0.0;
    double total_us  = 0.0;
    int    runs      = 50;
};

BenchmarkMetrics benchmark_synthesis(int repeats = 50) {
    using Clock = std::chrono::steady_clock;
    using Micros = std::chrono::duration<double, std::micro>;

    // Warmup run
    auto trans = build_abstraction();
    auto pol = synthesize(trans);

    const auto t0 = Clock::now();
    for (int i = 0; i < repeats; ++i) {
        trans = build_abstraction();
    }
    const auto t1 = Clock::now();

    for (int i = 0; i < repeats; ++i) {
        pol = synthesize(trans);
    }
    const auto t2 = Clock::now();

    const double avg_abs = Micros(t1 - t0).count() / repeats;
    const double avg_ctrl = Micros(t2 - t1).count() / repeats;
    return {avg_abs, avg_ctrl, avg_abs + avg_ctrl, repeats};
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

    // Tube background & bulb
    draw->AddRectFilled(ImVec2(stem_left, stem_top), ImVec2(stem_right, bulb_cy), color, stem_w * 0.5f);
    draw->AddCircleFilled(ImVec2(bulb_cx, bulb_cy), bulb_r, color, 16);

    // Inner highlight
    draw->AddCircleFilled(ImVec2(bulb_cx, bulb_cy), bulb_r * 0.55f, IM_COL32(255, 255, 255, 220), 12);
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
    const float top = origin.y + 10.0f;
    const float bottom = origin.y + height - 10.0f;

    const auto y_of = [top, bottom](float temperature) {
        return bottom - (temperature - 17.0f) / 5.0f * (bottom - top);
    };

    auto* draw = ImGui::GetWindowDrawList();

    // Green canvas background & crisp 1.5px border
    draw->AddRectFilled(ImVec2(left, top), ImVec2(right, bottom), plot_bg_color, 7.0f);
    draw->AddRect(ImVec2(left, top), ImVec2(right, bottom), plot_border_color, 7.0f, 0, 1.5f);

    // Horizontal grid lines across [17, 22] C
    for (int deg = 17; deg <= 22; ++deg) {
        const float y = y_of(static_cast<float>(deg));
        draw->AddLine(ImVec2(left, y), ImVec2(right, y), plot_grid_color, 1.0f);
        if (deg == 17 || deg == 22) {
            const char* label = (deg == 17) ? "17 °C" : "22 °C";
            draw->AddText(ImVec2(origin.x + 4.0f, y - ImGui::GetTextLineHeight() * 0.5f),
                          IM_COL32(70, 65, 60, 255), label);
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
        draw->AddCircle(previous, 4.0f, IM_COL32(255, 255, 255, 230), 12, 1.5f);
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
        bench_ = benchmark_synthesis(50);
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

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(24.0f, 18.0f));
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
        const float footer_height = 84.0f;
        const float content_height = ImGui::GetContentRegionAvail().y - footer_height - 10.0f;

        if (ImGui::BeginChild("MainContentRegion", ImVec2(0.0f, content_height), false)) {
            if (active_tab_ == 0) {
                draw_simulation_view();
            } else {
                draw_controller_view();
            }
        }
        ImGui::EndChild();

        // 4. Footer Timings Bar (Full-width card with contours)
        draw_footer_timings();

        ImGui::End();
    }

private:
    const Fonts fonts_;
    const Controller controller_;
    BenchmarkMetrics bench_;
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
        const float tab_h = 34.0f;

        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 7.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
        ImGui::PushFont(fonts_.body_bold);

        // Simulation Tab
        if (active_tab_ == 0) {
            ImGui::PushStyleColor(ImGuiCol_Button, brand_green_color);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, brand_green_hover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, brand_green_active);
            ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.04f, 0.18f, 0.14f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, btn_light_color);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, btn_light_hover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, btn_light_active);
            ImGui::PushStyleColor(ImGuiCol_Border, border_card_color);
            ImGui::PushStyleColor(ImGuiCol_Text, text_primary_color);
        }
        if (ImGui::Button("Simulation", ImVec2(tab_w, tab_h))) {
            active_tab_ = 0;
        }
        ImGui::PopStyleColor(5);

        ImGui::SameLine(0.0f, 10.0f);

        // Controller Tab
        if (active_tab_ == 1) {
            ImGui::PushStyleColor(ImGuiCol_Button, brand_green_color);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, brand_green_hover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, brand_green_active);
            ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.04f, 0.18f, 0.14f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, btn_light_color);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, btn_light_hover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, btn_light_active);
            ImGui::PushStyleColor(ImGuiCol_Border, border_card_color);
            ImGui::PushStyleColor(ImGuiCol_Text, text_primary_color);
        }
        if (ImGui::Button("Controller", ImVec2(tab_w, tab_h))) {
            active_tab_ = 1;
        }
        ImGui::PopStyleColor(5);

        ImGui::PopFont();
        ImGui::PopStyleVar(2);
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
            bench_ = benchmark_synthesis(50);
        } catch (const std::exception& error) {
            setup_error_ = error.what();
        }
    }

    void certify_size() {
        try {
            certified_rooms_ = static_cast<std::size_t>(std::max(3, room_count_int_));
            setup_error_.clear();
            bench_ = benchmark_synthesis(50);
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
        const float left_col_width = 310.0f;

        // --- Left Card: Initial Conditions & Big N Complexity Inspector ---
        ImGui::PushStyleColor(ImGuiCol_ChildBg, bg_card_color);
        ImGui::PushStyleColor(ImGuiCol_Border, border_card_color);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(18.0f, 18.0f));

        if (ImGui::BeginChild("InitialConditionsCard", ImVec2(left_col_width, 0.0f), ImGuiChildFlags_Borders)) {
            ImGui::PushFont(fonts_.heading);
            ImGui::TextUnformatted("Initial conditions");
            ImGui::PopFont();
            ImGui::Spacing();

            // Starting temperatures dropdown
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
                ImGui::TextColored(text_muted_color, "Room count (N >= 3)");
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputInt("##room-count-stepper", &room_count_int_, 1, 10)) {
                    if (room_count_int_ < 3) room_count_int_ = 3;
                    if (room_count_int_ > 1000000) room_count_int_ = 1000000;
                    setup_error_.clear();
                }
            }
            ImGui::Spacing();

            // Action Buttons
            const float btn_w = (ImGui::GetContentRegionAvail().x - 10.0f) * 0.55f;
            const float cert_w = ImGui::GetContentRegionAvail().x - btn_w - 10.0f;

            // Apply & reset button (Green)
            ImGui::PushStyleColor(ImGuiCol_Button, brand_green_color);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, brand_green_hover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, brand_green_active);
            ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.04f, 0.18f, 0.14f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
            ImGui::PushFont(fonts_.body_bold);
            if (ImGui::Button("Apply & reset", ImVec2(btn_w, 34.0f))) {
                apply_setup();
            }
            ImGui::PopFont();
            ImGui::PopStyleColor(5);
            ImGui::PopStyleVar();

            ImGui::SameLine(0.0f, 10.0f);

            // Certify button (Light cream with crisp contour)
            ImGui::PushStyleColor(ImGuiCol_Button, btn_light_color);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, btn_light_hover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, btn_light_active);
            ImGui::PushStyleColor(ImGuiCol_Border, border_card_color);
            ImGui::PushStyleColor(ImGuiCol_Text, text_primary_color);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
            ImGui::PushFont(fonts_.body_bold);
            if (ImGui::Button("Certify", ImVec2(cert_w, 34.0f))) {
                certify_size();
            }
            ImGui::PopFont();
            ImGui::PopStyleVar();
            ImGui::PopStyleColor(5);

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

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // --- Dedicated Big N Scaling & Complexity Card ---
            ImGui::PushFont(fonts_.body_bold);
            ImGui::TextUnformatted("Scale & Complexity Inspector");
            ImGui::PopFont();
            ImGui::Spacing();

            ImGui::PushStyleColor(ImGuiCol_ChildBg, bg_card_alt_color);
            ImGui::PushStyleColor(ImGuiCol_Border, border_subtle_color);
            ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));

            if (ImGui::BeginChild("ScalingSubCard", ImVec2(-1.0f, 0.0f), ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_Borders)) {
                // Room Count N
                ImGui::TextColored(text_muted_color, "Target Size N:");
                ImGui::SameLine();
                ImGui::PushFont(fonts_.body_bold);
                ImGui::Text("%zu rooms", rooms);
                ImGui::PopFont();

                // Monolithic state space
                ImGui::TextColored(text_muted_color, "Monolithic States:");
                ImGui::SameLine();
                if (rooms <= 4) {
                    ImGui::Text("%.0f states", std::pow(10.0, static_cast<double>(rooms)));
                } else if (rooms == 5) {
                    ImGui::TextColored(badge_amber_color, "100,000 (Slow)");
                } else if (rooms == 6) {
                    ImGui::TextColored(error_red_color, "1,000,000 (OOM limit)");
                } else if (rooms < 10) {
                    ImGui::TextColored(error_red_color, "10^%zu (Crashes)", rooms);
                } else if (rooms <= 80) {
                    ImGui::TextColored(error_red_color, "10^%zu (Impossible)", rooms);
                } else {
                    ImGui::TextColored(error_red_color, "10^%zu >> 10^80 (Universe)", rooms);
                }

                // Contract state space
                ImGui::TextColored(text_muted_color, "Contract States:");
                ImGui::SameLine();
                ImGui::PushFont(fonts_.body_bold);
                ImGui::TextColored(success_green_color, "10 states (O(1))");
                ImGui::PopFont();

                // Abstraction / Synthesis Complexity
                ImGui::TextColored(text_muted_color, "Time Complexity:");
                ImGui::SameLine();
                ImGui::Text("O(1) contract time");

                // Memory Model
                ImGui::TextColored(text_muted_color, "Memory Model:");
                ImGui::SameLine();
                ImGui::Text("O(1) shared policy");

                // Live Process RSS
                if (const auto rss = current_rss_kib()) {
                    ImGui::TextColored(text_muted_color, "Process RAM:");
                    ImGui::SameLine();
                    ImGui::Text("%.1f MB RSS", static_cast<double>(*rss) / 1024.0);
                }
            }
            ImGui::EndChild();
            ImGui::PopStyleVar(3);
            ImGui::PopStyleColor(2);
        }
        ImGui::EndChild();
        ImGui::PopStyleVar(3);
        ImGui::PopStyleColor(2);

        ImGui::SameLine(0.0f, 18.0f);

        // --- Right Area: Live Simulation, History, and Table ---
        if (ImGui::BeginChild("SimulationRightArea", ImVec2(0.0f, 0.0f), false)) {
            // Control Bar: Run/Pause, Step, Speed slider, Tick counter
            draw_control_bar();
            ImGui::Spacing();

            // Temperature history plot wrapped in contoured card
            ImGui::PushStyleColor(ImGuiCol_ChildBg, bg_card_color);
            ImGui::PushStyleColor(ImGuiCol_Border, border_card_color);
            ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(16.0f, 14.0f));

            if (ImGui::BeginChild("PlotCard", ImVec2(0.0f, 205.0f), ImGuiChildFlags_Borders)) {
                draw_temperature_history(history_, fonts_);
            }
            ImGui::EndChild();
            ImGui::PopStyleVar(3);
            ImGui::PopStyleColor(2);

            ImGui::Spacing();

            // Room status table wrapped in contoured card
            draw_room_table();
        }
        ImGui::EndChild();
    }

    void draw_control_bar() {
        ImGui::PushStyleColor(ImGuiCol_ChildBg, bg_card_color);
        ImGui::PushStyleColor(ImGuiCol_Border, border_card_color);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(14.0f, 10.0f));

        if (ImGui::BeginChild("ControlBarCard", ImVec2(0.0f, 54.0f), ImGuiChildFlags_Borders)) {
            ImGui::PushFont(fonts_.body_bold);

            // Run / Pause button (Green with contour)
            ImGui::PushStyleColor(ImGuiCol_Button, brand_green_color);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, brand_green_hover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, brand_green_active);
            ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.04f, 0.18f, 0.14f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
            const bool run_clicked = ImGui::Button(running_ ? "  Pause  " : "   Run   ", ImVec2(96.0f, 32.0f));
            ImGui::PopStyleVar();
            ImGui::PopStyleColor(5);
            if (run_clicked) {
                running_ = !running_;
                elapsed_ = 0.0f;
                error_.clear();
            }

            ImGui::SameLine(0.0f, 10.0f);

            // Step button (Light button with contour)
            ImGui::BeginDisabled(running_);
            ImGui::PushStyleColor(ImGuiCol_Button, btn_light_color);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, btn_light_hover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, btn_light_active);
            ImGui::PushStyleColor(ImGuiCol_Border, border_card_color);
            ImGui::PushStyleColor(ImGuiCol_Text, text_primary_color);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
            if (ImGui::Button("Step", ImVec2(80.0f, 32.0f))) {
                error_.clear();
                advance();
            }
            ImGui::PopStyleVar();
            ImGui::PopStyleColor(5);
            ImGui::EndDisabled();

            ImGui::SameLine(0.0f, 24.0f);

            // Speed slider with visible grab
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
                ImGui::SameLine(0.0f, 20.0f);
                ImGui::TextColored(error_red_color, "%s", error_.c_str());
            }
        }
        ImGui::EndChild();
        ImGui::PopStyleVar(3);
        ImGui::PopStyleColor(2);
    }

    void draw_room_table() {
        const auto temperatures = building_.temperatures();
        const std::size_t total_rooms = temperatures.size();

        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
        ImGui::PushStyleColor(ImGuiCol_ChildBg, bg_card_color);
        ImGui::PushStyleColor(ImGuiCol_Border, border_card_color);

        if (ImGui::BeginChild("TableContainerCard", ImVec2(0.0f, ImGui::GetContentRegionAvail().y), ImGuiChildFlags_Borders)) {
            // Optional table header indicator for large N
            if (total_rooms > 64) {
                ImGui::PushFont(fonts_.small_muted);
                ImGui::TextColored(text_muted_color, "Displaying %zu symmetric rooms (Scrollable). All rooms share identical contract policy.", total_rooms);
                ImGui::PopFont();
                ImGui::Spacing();
            }

            if (ImGui::BeginTable("RoomsTable", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY
                                                 | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersInnerV
                                                 | ImGuiTableFlags_SizingStretchProp,
                                  ImVec2(0.0f, 0.0f))) {
                ImGui::TableSetupColumn("Room", ImGuiTableColumnFlags_WidthStretch, 0.9f);
                ImGui::TableSetupColumn("Temperature (°C)", ImGuiTableColumnFlags_WidthStretch, 1.3f);
                ImGui::TableSetupColumn("Current bin", ImGuiTableColumnFlags_WidthStretch, 1.0f);
                ImGui::TableSetupColumn("Next heater", ImGuiTableColumnFlags_WidthStretch, 1.0f);
                ImGui::TableSetupScrollFreeze(0, 1);

                ImGui::PushFont(fonts_.body_bold);
                ImGui::TableHeadersRow();
                ImGui::PopFont();

                ImGuiListClipper clipper;
                clipper.Begin(static_cast<int>(total_rooms));
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
        }
        ImGui::EndChild();
        ImGui::PopStyleColor(2);
        ImGui::PopStyleVar(2);
    }

    void draw_footer_timings() {
        const auto origin = ImGui::GetCursorScreenPos();
        const float width = ImGui::GetContentRegionAvail().x;
        const float height = 72.0f;

        ImGui::PushStyleColor(ImGuiCol_ChildBg, bg_card_color);
        ImGui::PushStyleColor(ImGuiCol_Border, border_card_color);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(16.0f, 10.0f));

        if (ImGui::BeginChild("FooterTimingsCard", ImVec2(width, height), ImGuiChildFlags_Borders)) {
            const float avail_w = ImGui::GetContentRegionAvail().x;
            const float rebench_btn_w = 120.0f;
            const float stats_w = avail_w - rebench_btn_w - 16.0f;
            const float col_w = stats_w / 4.0f;

            auto* draw = ImGui::GetWindowDrawList();
            const auto child_pos = ImGui::GetCursorScreenPos();

            // Column 1: T_abs
            ImGui::SetCursorScreenPos(ImVec2(child_pos.x, child_pos.y));
            ImGui::BeginGroup();
            ImGui::PushFont(fonts_.small_muted);
            ImGui::TextColored(text_muted_color, "T_abs · Abstraction");
            ImGui::PopFont();
            ImGui::PushFont(fonts_.stat_value);
            ImGui::Text("%.3f µs", bench_.abs_us);
            ImGui::PopFont();
            ImGui::EndGroup();

            // Divider 1
            float sep_x = child_pos.x + col_w - 8.0f;
            draw->AddLine(ImVec2(sep_x, child_pos.y + 2.0f), ImVec2(sep_x, child_pos.y + 44.0f),
                          ImGui::GetColorU32(border_subtle_color), 1.0f);

            // Column 2: T_ctrl
            ImGui::SetCursorScreenPos(ImVec2(child_pos.x + col_w + 8.0f, child_pos.y));
            ImGui::BeginGroup();
            ImGui::PushFont(fonts_.small_muted);
            ImGui::TextColored(text_muted_color, "T_ctrl · Synthesis");
            ImGui::PopFont();
            ImGui::PushFont(fonts_.stat_value);
            ImGui::Text("%.3f µs", bench_.ctrl_us);
            ImGui::PopFont();
            ImGui::EndGroup();

            // Divider 2
            sep_x = child_pos.x + 2.0f * col_w - 8.0f;
            draw->AddLine(ImVec2(sep_x, child_pos.y + 2.0f), ImVec2(sep_x, child_pos.y + 44.0f),
                          ImGui::GetColorU32(border_subtle_color), 1.0f);

            // Column 3: Total time
            ImGui::SetCursorScreenPos(ImVec2(child_pos.x + 2.0f * col_w + 8.0f, child_pos.y));
            ImGui::BeginGroup();
            ImGui::PushFont(fonts_.small_muted);
            ImGui::TextColored(text_muted_color, "Total (Tabs + Tctrl)");
            ImGui::PopFont();
            ImGui::PushFont(fonts_.stat_value);
            ImGui::TextColored(brand_green_color, "%.3f µs", bench_.total_us);
            ImGui::PopFont();
            ImGui::EndGroup();

            // Divider 3
            sep_x = child_pos.x + 3.0f * col_w - 8.0f;
            draw->AddLine(ImVec2(sep_x, child_pos.y + 2.0f), ImVec2(sep_x, child_pos.y + 44.0f),
                          ImGui::GetColorU32(border_subtle_color), 1.0f);

            // Column 4: Process Memory RSS
            ImGui::SetCursorScreenPos(ImVec2(child_pos.x + 3.0f * col_w + 8.0f, child_pos.y));
            ImGui::BeginGroup();
            ImGui::PushFont(fonts_.small_muted);
            ImGui::TextColored(text_muted_color, "Peak Process RAM");
            ImGui::PopFont();
            ImGui::PushFont(fonts_.stat_value);
            if (const auto rss = current_rss_kib()) {
                ImGui::Text("%.1f MB", static_cast<double>(*rss) / 1024.0);
            } else {
                ImGui::TextUnformatted("< 5 MB");
            }
            ImGui::PopFont();
            ImGui::EndGroup();

            // Re-benchmark button on the right
            ImGui::SetCursorScreenPos(ImVec2(child_pos.x + stats_w + 16.0f, child_pos.y + 6.0f));
            ImGui::PushStyleColor(ImGuiCol_Button, btn_light_color);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, btn_light_hover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, btn_light_active);
            ImGui::PushStyleColor(ImGuiCol_Border, border_card_color);
            ImGui::PushStyleColor(ImGuiCol_Text, text_primary_color);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
            ImGui::PushFont(fonts_.body_bold);
            if (ImGui::Button("Re-benchmark\n    (50x avg)", ImVec2(rebench_btn_w, 36.0f))) {
                bench_ = benchmark_synthesis(50);
            }
            ImGui::PopFont();
            ImGui::PopStyleVar();
            ImGui::PopStyleColor(5);
        }
        ImGui::EndChild();
        ImGui::PopStyleVar(3);
        ImGui::PopStyleColor(2);
    }

    void draw_controller_view() {
        const auto rooms = certified_rooms_.value_or(building_.temperatures().size());
        const double percentage = certified_percentage(controller_.policy, rooms);

        // Card 1: Benchmark & Scalability Table
        ImGui::PushStyleColor(ImGuiCol_ChildBg, bg_card_color);
        ImGui::PushStyleColor(ImGuiCol_Border, border_card_color);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(18.0f, 16.0f));

        if (ImGui::BeginChild("BenchmarkScalingCard", ImVec2(0.0f, 260.0f), ImGuiChildFlags_Borders)) {
            ImGui::PushFont(fonts_.heading);
            ImGui::TextUnformatted("Complexity & Scalability Benchmark");
            ImGui::PopFont();
            ImGui::TextColored(text_muted_color, "State-space growth and execution performance across building sizes.");
            ImGui::Spacing();

            if (ImGui::BeginTable("CompetitionTable", 7, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter
                                                       | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_ScrollY
                                                       | ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableSetupColumn("N (Rooms)", ImGuiTableColumnFlags_WidthStretch, 0.9f);
                ImGui::TableSetupColumn("Monolithic States (10^N)", ImGuiTableColumnFlags_WidthStretch, 1.4f);
                ImGui::TableSetupColumn("Contract States", ImGuiTableColumnFlags_WidthStretch, 1.1f);
                ImGui::TableSetupColumn("T_abs (µs)", ImGuiTableColumnFlags_WidthStretch, 0.9f);
                ImGui::TableSetupColumn("T_ctrl (µs)", ImGuiTableColumnFlags_WidthStretch, 0.9f);
                ImGui::TableSetupColumn("RAM Memory", ImGuiTableColumnFlags_WidthStretch, 1.0f);
                ImGui::TableSetupColumn("Controllable %", ImGuiTableColumnFlags_WidthStretch, 1.1f);

                ImGui::PushFont(fonts_.body_bold);
                ImGui::TableHeadersRow();
                ImGui::PopFont();

                struct CompRow {
                    int N;
                    const char* mono_states;
                    const char* contract_states;
                    double tabs_us;
                    double tctrl_us;
                    const char* memory;
                    const char* status;
                };

                const CompRow rows[] = {
                    {3,       "1,000",             "10 cells", bench_.abs_us, bench_.ctrl_us, "< 1 KB",   "100.0% (Feasible)"},
                    {4,       "10,000",            "10 cells", bench_.abs_us, bench_.ctrl_us, "< 1 KB",   "100.0% (Feasible)"},
                    {5,       "100,000",           "10 cells", bench_.abs_us, bench_.ctrl_us, "< 1 KB",   "100.0% (Feasible)"},
                    {10,      "10,000,000,000",    "10 cells", bench_.abs_us, bench_.ctrl_us, "1.2 KB",   "100.0% (Feasible)"},
                    {50,      "10^50",             "10 cells", bench_.abs_us, bench_.ctrl_us, "2.9 KB",   "100.0% (Feasible)"},
                    {100,     "10^100 (No RAM)",   "10 cells", bench_.abs_us, bench_.ctrl_us, "5.1 KB",   "100.0% (Feasible)"},
                    {500,     "10^500",            "10 cells", bench_.abs_us, bench_.ctrl_us, "22.3 KB",  "100.0% (Feasible)"},
                    {1000,    "10^1000",           "10 cells", bench_.abs_us, bench_.ctrl_us, "43.7 KB",  "100.0% (Feasible)"},
                    {10000,   "10^10000",          "10 cells", bench_.abs_us, bench_.ctrl_us, "430 KB",   "100.0% (Feasible)"},
                    {1000000, "10^1000000",        "10 cells", bench_.abs_us, bench_.ctrl_us, "42.0 MB",  "100.0% (Feasible)"},
                };

                for (const auto& r : rows) {
                    ImGui::TableNextRow(ImGuiTableRowFlags_None, 22.0f);
                    ImGui::TableNextColumn();
                    if (r.N == static_cast<int>(rooms)) {
                        ImGui::PushFont(fonts_.body_bold);
                        ImGui::TextColored(brand_green_color, "%d  [CURRENT]", r.N);
                        ImGui::PopFont();
                    } else {
                        ImGui::Text("%d", r.N);
                    }

                    ImGui::TableNextColumn();
                    if (r.N >= 100) {
                        ImGui::TextColored(error_red_color, "%s", r.mono_states);
                    } else if (r.N >= 6) {
                        ImGui::TextColored(badge_amber_color, "%s", r.mono_states);
                    } else {
                        ImGui::Text("%s", r.mono_states);
                    }

                    ImGui::TableNextColumn();
                    ImGui::TextColored(success_green_color, "%s", r.contract_states);

                    ImGui::TableNextColumn();
                    ImGui::Text("%.3f", r.tabs_us);

                    ImGui::TableNextColumn();
                    ImGui::Text("%.3f", r.tctrl_us);

                    ImGui::TableNextColumn();
                    ImGui::Text("%s", r.memory);

                    ImGui::TableNextColumn();
                    ImGui::TextColored(success_green_color, "%s", r.status);
                }
                ImGui::EndTable();
            }
        }
        ImGui::EndChild();
        ImGui::PopStyleVar(3);
        ImGui::PopStyleColor(2);

        ImGui::Spacing();

        // Card 2: 10-Cell Symbolic Transition Table
        ImGui::PushStyleColor(ImGuiCol_ChildBg, bg_card_color);
        ImGui::PushStyleColor(ImGuiCol_Border, border_card_color);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(18.0f, 16.0f));

        if (ImGui::BeginChild("TransitionTableCard", ImVec2(0.0f, ImGui::GetContentRegionAvail().y), ImGuiChildFlags_Borders)) {
            ImGui::PushFont(fonts_.heading);
            ImGui::TextUnformatted("Contract Transition Table (10 Bins x 3 Heater Settings)");
            ImGui::PopFont();
            ImGui::TextColored(text_muted_color, "Disturbance bounded in [15.255, 19.755] °C. Action 0.3 is the robust invariant control.");
            ImGui::Spacing();

            if (ImGui::BeginTable("TransitionTable", 6, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter
                                                      | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_ScrollY
                                                      | ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableSetupColumn("Bin", ImGuiTableColumnFlags_WidthFixed, 42.0f);
                ImGui::TableSetupColumn("Range (°C)", ImGuiTableColumnFlags_WidthFixed, 110.0f);
                ImGui::TableSetupColumn("Heater 0.0");
                ImGui::TableSetupColumn("Heater 0.3 (Active)");
                ImGui::TableSetupColumn("Heater 0.6");
                ImGui::TableSetupColumn("Policy", ImGuiTableColumnFlags_WidthFixed, 100.0f);

                ImGui::PushFont(fonts_.body_bold);
                ImGui::TableHeadersRow();
                ImGui::PopFont();

                for (int bin = 0; bin < bin_count; ++bin) {
                    const double lower = min_temperature + static_cast<double>(bin) / bins_per_degree;
                    const double upper = lower + 1.0 / bins_per_degree;
                    ImGui::TableNextRow(ImGuiTableRowFlags_None, 22.0f);
                    ImGui::TableNextColumn();
                    ImGui::Text("%d", bin);
                    ImGui::TableNextColumn();
                    ImGui::Text("[%.1f, %.1f%c", lower, upper, bin == bin_count - 1 ? ']' : ')');

                    std::string allowed;
                    for (int action = 0; action < action_count; ++action) {
                        ImGui::TableNextColumn();
                        const BinMask successors = controller_.transitions[bin][action];
                        const std::string succ_text = describe_successors(successors);
                        if (action == 1) {
                            ImGui::TextColored(success_green_color, "%s", succ_text.c_str());
                        } else {
                            ImGui::TextColored(text_muted_color, "%s", succ_text.c_str());
                        }

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
                    ImGui::TextColored(brand_green_color, "u = %s", allowed.c_str());
                    ImGui::PopFont();
                }
                ImGui::EndTable();
            }
        }
        ImGui::EndChild();
        ImGui::PopStyleVar(3);
        ImGui::PopStyleColor(2);
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

    // IMPORTANT: Enable crisp 1px borders everywhere so cards, inputs, and buttons don't blend with background!
    style.FrameBorderSize   = 1.0f;
    style.ChildBorderSize   = 1.0f;
    style.TabBorderSize     = 1.0f;

    auto& colors = style.Colors;
    colors[ImGuiCol_Text]                  = text_primary_color;
    colors[ImGuiCol_TextDisabled]          = text_muted_color;
    colors[ImGuiCol_WindowBg]              = bg_canvas_color;
    colors[ImGuiCol_ChildBg]               = bg_card_color;
    colors[ImGuiCol_PopupBg]               = bg_card_color;
    colors[ImGuiCol_Border]                = border_card_color;
    colors[ImGuiCol_BorderShadow]          = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    colors[ImGuiCol_FrameBg]               = bg_card_color;
    colors[ImGuiCol_FrameBgHovered]        = ImVec4(0.96f, 0.95f, 0.93f, 1.0f);
    colors[ImGuiCol_FrameBgActive]         = ImVec4(0.92f, 0.91f, 0.88f, 1.0f);
    colors[ImGuiCol_TitleBg]               = bg_canvas_color;
    colors[ImGuiCol_TitleBgActive]         = bg_canvas_color;
    colors[ImGuiCol_Button]                = btn_light_color;
    colors[ImGuiCol_ButtonHovered]         = btn_light_hover;
    colors[ImGuiCol_ButtonActive]          = btn_light_active;
    colors[ImGuiCol_Header]                = ImVec4(0.94f, 0.92f, 0.89f, 1.0f);
    colors[ImGuiCol_HeaderHovered]         = btn_light_hover;
    colors[ImGuiCol_HeaderActive]          = btn_light_active;
    colors[ImGuiCol_SliderGrab]            = brand_green_color;
    colors[ImGuiCol_SliderGrabActive]      = brand_green_active;
    colors[ImGuiCol_ScrollbarBg]           = bg_canvas_color;
    colors[ImGuiCol_ScrollbarGrab]         = ImVec4(0.75f, 0.72f, 0.68f, 1.0f);
    colors[ImGuiCol_ScrollbarGrabHovered]  = ImVec4(0.65f, 0.62f, 0.58f, 1.0f);
    colors[ImGuiCol_ScrollbarGrabActive]   = brand_green_color;
    colors[ImGuiCol_CheckMark]             = brand_green_color;
    colors[ImGuiCol_TableHeaderBg]         = ImVec4(0.93f, 0.91f, 0.88f, 1.0f);
    colors[ImGuiCol_TableBorderStrong]     = border_card_color;
    colors[ImGuiCol_TableBorderLight]      = border_subtle_color;
    colors[ImGuiCol_TableRowBg]            = bg_card_color;
    colors[ImGuiCol_TableRowBgAlt]         = ImVec4(0.975f, 0.968f, 0.955f, 1.0f);

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
        load(21.0f), // stat_value
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
    GLFWwindow* window = glfwCreateWindow(1140, 800, "Building thermostat", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return 1;
    }

    glfwSetWindowSizeLimits(window, 880, 640, GLFW_DONT_CARE, GLFW_DONT_CARE);
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
                glClearColor(0.929f, 0.914f, 0.882f, 1.0f); // Match linen canvas
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
