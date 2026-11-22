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

constexpr ImU32 cold_color = IM_COL32(110, 174, 248, 255);
constexpr ImU32 mean_color = IM_COL32(80, 224, 186, 255);
constexpr ImU32 warm_color = IM_COL32(251, 179, 109, 255);
constexpr ImVec4 error_color(1.0f, 0.62f, 0.52f, 1.0f);

struct Fonts {
    ImFont* title;
    ImFont* heading;
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
                            * (static_cast<double>(i) / static_cast<double>(count - 1));
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

void draw_temperature_history(const std::deque<TemperatureSample>& history) {
    const auto sample = history.back();
    ImGui::TextUnformatted("Temperature history");
    ImGui::SameLine();
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(cold_color), "Min %.3f °C", sample.coldest);
    ImGui::SameLine();
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(mean_color), "Mean %.3f °C", sample.mean);
    ImGui::SameLine();
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(warm_color), "Max %.3f °C", sample.warmest);

    const auto origin = ImGui::GetCursorScreenPos();
    const auto width = ImGui::GetContentRegionAvail().x;
    const float height = 170.0f;
    ImGui::Dummy(ImVec2(width, height));

    const float left = origin.x + ImGui::CalcTextSize("22 °C").x + 24.0f;
    const float right = origin.x + width - 14.0f;
    const float top = origin.y + 10.0f;
    const float bottom = origin.y + height - 10.0f;
    const auto y_of = [top, bottom](float temperature) {
        return bottom - (temperature - 16.5f) / 6.0f * (bottom - top);
    };

    auto* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(origin, ImVec2(origin.x + width, origin.y + height),
                        IM_COL32(19, 31, 42, 255), 7.0f);
    draw->AddRectFilled(ImVec2(left, y_of(22.0f)), ImVec2(right, y_of(17.0f)),
                        IM_COL32(31, 66, 68, 255));
    for (const float temperature : {17.0f, 22.0f}) {
        const float y = y_of(temperature);
        draw->AddLine(ImVec2(left, y), ImVec2(right, y), IM_COL32(78, 125, 128, 255));
        draw->AddText(ImVec2(origin.x + 12.0f, y - ImGui::GetTextLineHeight() * 0.5f),
                      IM_COL32(180, 198, 205, 255), temperature == 17.0f ? "17 °C" : "22 °C");
    }

    // A fixed vertical scale keeps the comfort band comparable across runs.
    const auto line = [&](float TemperatureSample::*field, ImU32 color) {
        const float dx = (right - left) / static_cast<float>(std::max<std::size_t>(history.size() - 1, 1));
        ImVec2 previous(left, y_of(history.front().*field));
        draw->AddCircleFilled(previous, 2.5f, color);
        for (std::size_t i = 1; i < history.size(); ++i) {
            const ImVec2 point(left + dx * static_cast<float>(i), y_of(history[i].*field));
            draw->AddLine(previous, point, color, 2.0f);
            previous = point;
        }
    };
    line(&TemperatureSample::coldest, cold_color);
    line(&TemperatureSample::mean, mean_color);
    line(&TemperatureSample::warmest, warm_color);
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
        : fonts_(fonts), controller_(build_controller()), building_(initial_temperatures(8, ramp)) {
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
        ImGui::Begin("Building thermostat", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove
                     | ImGuiWindowFlags_NoSavedSettings);
        ImGui::PushFont(fonts_.title);
        ImGui::TextUnformatted("Building thermostat");
        ImGui::PopFont();
        ImGui::TextDisabled("Ring of %zu rooms   ·   Comfort range %d to %d °C",
                            building_.temperatures().size(), min_temperature, max_temperature);
        ImGui::Spacing();
        draw_timings();
        ImGui::Spacing();

        if (ImGui::BeginChild("Dashboard content") && ImGui::BeginTabBar("Thermostat tabs")) {
            if (ImGui::BeginTabItem("Simulation")) {
                draw_simulation();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Controller")) {
                draw_controller();
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        ImGui::EndChild();
        ImGui::End();
    }

private:
    const Fonts fonts_;
    const Controller controller_;
    Building building_;
    std::deque<TemperatureSample> history_;
    std::string room_count_ = "8";
    std::optional<std::size_t> certified_rooms_;
    int profile_ = ramp;
    int ticks_per_second_ = 8;
    bool running_ = false;
    float elapsed_ = 0.0f;
    std::string custom_temperatures_ = "17, 18.5, 20, 22";
    std::string setup_error_;
    std::string error_;

    void draw_timings() const {
        const float width = std::min(720.0f, ImGui::GetContentRegionAvail().x);
        if (ImGui::BeginTable("Controller timings", 3,
                              ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_Borders,
                              ImVec2(width, 0.0f))) {
            ImGui::TableNextRow();
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, ImGui::GetColorU32(ImGuiCol_ChildBg));
            const auto metric = [&](const char* label, double microseconds) {
                ImGui::TableNextColumn();
                ImGui::TextDisabled("%s", label);
                ImGui::PushFont(fonts_.heading);
                ImGui::Text("%.3f µs", microseconds);
                ImGui::PopFont();
            };
            metric("T_abs · Abstraction", controller_.abstraction_us);
            metric("T_ctrl · Synthesis", controller_.synthesis_us);
            metric("T_abs + T_ctrl · Total", controller_.abstraction_us + controller_.synthesis_us);
            ImGui::EndTable();
        }
    }

    void record_sample() {
        history_.push_back(summarize(building_.temperatures()));
        if (history_.size() > 180) {
            history_.pop_front();
        }
    }

    void apply_setup() {
        try {
            Building next(profile_ == custom ? read_temperatures(custom_temperatures_)
                                             : initial_temperatures(parse_room_count(room_count_), profile_));
            std::deque<TemperatureSample> history{summarize(next.temperatures())};
            building_ = std::move(next);
            history_.swap(history);
            running_ = false;
            elapsed_ = 0.0f;
            certified_rooms_.reset();
            setup_error_.clear();
            error_.clear();
        } catch (const std::exception& error) {
            setup_error_ = error.what();
        }
    }

    void certify_size() {
        try {
            certified_rooms_ = parse_room_count(room_count_);
            setup_error_.clear();
        } catch (const std::invalid_argument& error) {
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

    void draw_setup(float width) {
        if (ImGui::BeginChild("Initial conditions", ImVec2(width, 0.0f),
                              ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY)) {
            ImGui::PushFont(fonts_.heading);
            ImGui::TextUnformatted("Initial conditions");
            ImGui::PopFont();

            const float available = ImGui::GetContentRegionAvail().x;
            const bool inline_fields = profile_ != custom && available >= 540.0f;
            ImGui::BeginGroup();
            ImGui::TextUnformatted("Starting temperatures");
            ImGui::SetNextItemWidth(inline_fields ? available * 0.6f : -1.0f);
            if (ImGui::Combo("##profile", &profile_, profiles, IM_ARRAYSIZE(profiles))) {
                setup_error_.clear();
            }
            ImGui::EndGroup();

            if (profile_ == custom) {
                ImGui::TextUnformatted("Room temperatures (°C)");
                if (ImGui::InputTextMultiline("##room-temperatures", &custom_temperatures_,
                                              ImVec2(-1.0f, 82.0f))) {
                    setup_error_.clear();
                }
                ImGui::TextWrapped("Enter one value from 17 to 22 °C per room (at least 3). Use a decimal point "
                                   "and separate values with spaces, commas, or new lines.");
            } else {
                if (inline_fields) {
                    ImGui::SameLine();
                }
                ImGui::BeginGroup();
                ImGui::TextUnformatted("Room count (min. 3)");
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputText("##room-count", &room_count_)) {
                    setup_error_.clear();
                }
                ImGui::EndGroup();
            }

            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.66f, 0.61f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.30f, 0.76f, 0.71f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.15f, 0.57f, 0.53f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.04f, 0.12f, 0.15f, 1.0f));
            const bool apply = ImGui::Button("Apply & reset");
            ImGui::PopStyleColor(4);
            if (apply) {
                apply_setup();
            }
            if (profile_ != custom) {
                ImGui::SameLine();
                if (ImGui::Button("Certify")) {
                    certify_size();
                }
                ImGui::SetItemTooltip("Compute the certified percentage for this room count.");
            }
            if (!setup_error_.empty()) {
                ImGui::PushStyleColor(ImGuiCol_Text, error_color);
                ImGui::TextWrapped("%s", setup_error_.c_str());
                ImGui::PopStyleColor();
            }
            if (certified_rooms_) {
                ImGui::TextWrapped("%.2f%% of initial cells certified for %zu rooms.",
                                   certified_percentage(controller_.policy, *certified_rooms_), *certified_rooms_);
            }
        }
        ImGui::EndChild();
    }

    void draw_simulation() {
        const bool wide = ImGui::GetContentRegionAvail().x >= 970.0f;
        draw_setup(wide ? 318.0f : 0.0f);
        if (!wide) {
            draw_live_simulation();
            return;
        }
        ImGui::SameLine(0.0f, 24.0f);
        if (ImGui::BeginChild("Live simulation")) {
            draw_live_simulation();
        }
        ImGui::EndChild();
    }

    void draw_live_simulation() {
        if (ImGui::Button(running_ ? "Pause###playback" : "Run###playback", ImVec2(86.0f, 0.0f))) {
            running_ = !running_;
            elapsed_ = 0.0f;
            error_.clear();
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(running_);
        if (ImGui::Button("Step", ImVec2(76.0f, 0.0f))) {
            error_.clear();
            advance();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::SetNextItemWidth(155.0f);
        ImGui::SliderInt("Ticks per second", &ticks_per_second_, 1, 30, "%d", ImGuiSliderFlags_AlwaysClamp);
        ImGui::SameLine(0.0f, 26.0f);
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("Tick %zu", building_.tick());

        if (!error_.empty()) {
            ImGui::TextColored(error_color, "%s", error_.c_str());
        }
        ImGui::Spacing();
        draw_temperature_history(history_);

        const auto temperatures = building_.temperatures();
        if (ImGui::BeginTable("Rooms", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY
                                        | ImGuiTableFlags_SizingStretchProp,
                              ImVec2(0.0f, std::max(160.0f, ImGui::GetContentRegionAvail().y)))) {
            ImGui::TableSetupColumn("Room", ImGuiTableColumnFlags_WidthStretch, 0.7f);
            ImGui::TableSetupColumn("Temperature (°C)", ImGuiTableColumnFlags_WidthStretch, 1.4f);
            ImGui::TableSetupColumn("Current bin", ImGuiTableColumnFlags_WidthStretch, 1.0f);
            ImGui::TableSetupColumn("Next heater", ImGuiTableColumnFlags_WidthStretch, 1.0f);
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableHeadersRow();

            ImGuiListClipper clipper;
            clipper.Begin(static_cast<int>(temperatures.size()));
            while (clipper.Step()) {
                for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::Text("%d", i + 1);
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

    void draw_controller() const {
        const auto rooms = certified_rooms_.value_or(building_.temperatures().size());
        const double percentage = certified_percentage(controller_.policy, rooms);
        ImGui::TextWrapped("%.2f%% of initial cells certified for %zu rooms.", percentage, rooms);
        ImGui::TextWrapped("Each room uses its own temperature. The policy covers both neighbors from 17 to 22 °C.");
        ImGui::Spacing();

        ImGui::PushFont(fonts_.heading);
        ImGui::TextUnformatted("Complexity");
        ImGui::PopFont();
        ImGui::TextWrapped("All %zu rooms share one table with %d temperature bins and %d heater settings. "
                           "A joint temperature grid would have %d^%zu states.",
                           rooms, bin_count, action_count, bin_count, rooms);
        ImGui::TextWrapped("Abstraction and synthesis use O(1) time and memory in the number of rooms. "
                           "The timings above are measured at launch.");
        ImGui::TextWrapped("For N rooms, simulation takes O(N) time per tick and O(N) memory "
                           "for the current and next temperature readings.");
        ImGui::Spacing();

        ImGui::PushFont(fonts_.heading);
        ImGui::TextUnformatted("Successor bins by heater setting");
        ImGui::PopFont();
        ImGui::TextDisabled("Unsafe marks a possible next temperature outside the comfort range.");
        if (ImGui::BeginTable("Transition table", 6, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH
                                                    | ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableSetupColumn("Bin", ImGuiTableColumnFlags_WidthFixed, 34.0f);
            ImGui::TableSetupColumn("Range (°C)", ImGuiTableColumnFlags_WidthFixed, 119.0f);
            ImGui::TableSetupColumn("Heater 0.0");
            ImGui::TableSetupColumn("Heater 0.3");
            ImGui::TableSetupColumn("Heater 0.6");
            ImGui::TableSetupColumn("Allowed", ImGuiTableColumnFlags_WidthFixed, 70.0f);
            ImGui::TableHeadersRow();
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
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(mean_color), "%s",
                                   allowed.empty() ? "-" : allowed.c_str());
            }
            ImGui::EndTable();
        }
    }
};

Fonts style_interface() {
    ImGui::StyleColorsDark();
    auto& style = ImGui::GetStyle();
    style.WindowPadding = ImVec2(24.0f, 18.0f);
    style.FramePadding = ImVec2(11.0f, 8.0f);
    style.ItemSpacing = ImVec2(12.0f, 9.0f);
    style.CellPadding = ImVec2(12.0f, 7.0f);
    style.FrameRounding = 7.0f;
    style.ChildRounding = 9.0f;
    style.TabRounding = 7.0f;
    style.ScrollbarSize = 15.0f;
    auto& colors = style.Colors;
    colors[ImGuiCol_Text] = ImVec4(0.92f, 0.95f, 0.97f, 1.0f);
    colors[ImGuiCol_TextDisabled] = ImVec4(0.66f, 0.74f, 0.81f, 1.0f);
    colors[ImGuiCol_WindowBg] = ImVec4(0.05f, 0.08f, 0.12f, 1.0f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.08f, 0.12f, 0.17f, 1.0f);
    colors[ImGuiCol_PopupBg] = ImVec4(0.09f, 0.14f, 0.20f, 1.0f);
    colors[ImGuiCol_Border] = ImVec4(0.17f, 0.24f, 0.30f, 1.0f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.13f, 0.20f, 0.27f, 1.0f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.16f, 0.28f, 0.34f, 1.0f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.18f, 0.31f, 0.37f, 1.0f);
    colors[ImGuiCol_Button] = ImVec4(0.13f, 0.25f, 0.30f, 1.0f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.17f, 0.36f, 0.39f, 1.0f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.20f, 0.41f, 0.43f, 1.0f);
    colors[ImGuiCol_HeaderHovered] = colors[ImGuiCol_ButtonHovered];
    colors[ImGuiCol_HeaderActive] = colors[ImGuiCol_ButtonActive];
    colors[ImGuiCol_SliderGrab] = ImVec4(0.27f, 0.74f, 0.69f, 1.0f);
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.38f, 0.84f, 0.78f, 1.0f);
    colors[ImGuiCol_Tab] = colors[ImGuiCol_ChildBg];
    colors[ImGuiCol_TabHovered] = colors[ImGuiCol_ButtonHovered];
    colors[ImGuiCol_TabSelected] = colors[ImGuiCol_Button];
    colors[ImGuiCol_TabSelectedOverline] = colors[ImGuiCol_SliderGrab];
    colors[ImGuiCol_TableHeaderBg] = colors[ImGuiCol_ChildBg];
    colors[ImGuiCol_TableBorderStrong] = colors[ImGuiCol_Border];
    colors[ImGuiCol_TableBorderLight] = ImVec4(0.12f, 0.18f, 0.24f, 1.0f);

    auto& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImFontConfig font;
    font.FontDataOwnedByAtlas = false;
    const auto load = [&](float size) {
        return io.Fonts->AddFontFromMemoryTTF(thermostat_font, static_cast<int>(sizeof(thermostat_font)), size, &font);
    };
    io.FontDefault = load(18.0f);
    return {load(32.0f), load(23.0f)};
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
    GLFWwindow* window = glfwCreateWindow(1120, 820, "Building Thermostat", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return 1;
    }

    glfwSetWindowSizeLimits(window, 780, 620, GLFW_DONT_CARE, GLFW_DONT_CARE);
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
                glClearColor(0.05f, 0.08f, 0.12f, 1.0f);
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
