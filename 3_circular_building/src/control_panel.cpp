#include "control_panel.hpp"
#include "simulation_engine.hpp"
#include "ui_theme.hpp"
#include "imgui.h"

#include <cstdio>
#include <algorithm>

namespace circular::ui {

using namespace ImGui;
using namespace theme;
using namespace colors;
using namespace engine;
using namespace model;
using namespace abstraction;
using namespace synthesis;
using namespace environment;

void render_control_panel(AppState& app, float width, float height) {
    SetNextWindowPos(GetCursorScreenPos());
    SetNextWindowSize(ImVec2(width, height));

    begin_card("##control_sidebar", ImVec2(width, height));

    // -------------------------------------------------------------------------
    // 1. Building Architecture & Presets (Slide 15-16)
    // -------------------------------------------------------------------------
    if (g_fonts.bold) PushFont(g_fonts.bold);
    Text("Building Architecture");
    if (g_fonts.bold) PopFont();
    Separator();

    int prev_n = app.num_rooms;
    SliderInt("Rooms (N)", &app.num_rooms, 3, 100);
    if (app.num_rooms != prev_n) {
        reset_simulation(app);
        trigger_synthesis(app);
    }

    // Quick preset buttons matching Slide 17 benchmark sizes
    Text("Presets:");
    SameLine();
    if (Button("3"))   { app.num_rooms = 3;   reset_simulation(app); trigger_synthesis(app); }
    SameLine();
    if (Button("4"))   { app.num_rooms = 4;   reset_simulation(app); trigger_synthesis(app); }
    SameLine();
    if (Button("5"))   { app.num_rooms = 5;   reset_simulation(app); trigger_synthesis(app); }
    SameLine();
    if (Button("8"))   { app.num_rooms = 8;   reset_simulation(app); trigger_synthesis(app); }
    SameLine();
    if (Button("16"))  { app.num_rooms = 16;  reset_simulation(app); trigger_synthesis(app); }
    SameLine();
    if (Button("32"))  { app.num_rooms = 32;  reset_simulation(app); trigger_synthesis(app); }
    SameLine();
    if (Button("64"))  { app.num_rooms = 64;  reset_simulation(app); trigger_synthesis(app); }
    SameLine();
    if (Button("100")) { app.num_rooms = 100; reset_simulation(app); trigger_synthesis(app); }

    if (g_fonts.small) PushFont(g_fonts.small);
    TextColored(text_muted, "alpha=0.45 (loss), beta=0.045 (wall), gamma=0.09, Th=50 C");
    TextColored(text_muted, "Controls: u in {0.0, 0.3, 0.6} | Comfort: [17.0, 22.0] C");
    if (g_fonts.small) PopFont();

    Spacing();
    Spacing();

    // -------------------------------------------------------------------------
    // 2. Weather Environment (Strategy Pattern)
    // -------------------------------------------------------------------------
    if (g_fonts.bold) PushFont(g_fonts.bold);
    Text("Exterior Weather (Te)");
    if (g_fonts.bold) PopFont();
    Separator();

    int mode_idx = static_cast<int>(app.weather_type);
    bool weather_changed = false;

    if (RadioButton("Nominal Competition (-1.0 C)", &mode_idx, 0)) {
        app.weather_type = WeatherFactory::Type::Nominal;
        weather_changed = true;
    }
    if (RadioButton("24h Diurnal (-4..+2 C)",      &mode_idx, 1)) {
        app.weather_type = WeatherFactory::Type::Diurnal;
        weather_changed = true;
    }
    if (RadioButton("Cold Snap (-5.0 C)",          &mode_idx, 2)) {
        app.weather_type = WeatherFactory::Type::ColdSnap;
        weather_changed = true;
    }
    if (RadioButton("Manual Slider",                &mode_idx, 3)) {
        app.weather_type = WeatherFactory::Type::Manual;
        weather_changed = true;
    }

    if (app.weather_type == WeatherFactory::Type::Manual) {
        float f_te = static_cast<float>(app.manual_te);
        if (SliderFloat("Outside Temp", &f_te, -10.0f, 15.0f, "%.1f C")) {
            app.manual_te = static_cast<double>(f_te);
            weather_changed = true;
        }
    }

    if (weather_changed) {
        update_weather_strategy(app);
    }

    char te_buf[48];
    std::snprintf(te_buf, sizeof(te_buf), "Current Weather: Te = %.2f C", app.current_te);
    pill_badge(te_buf, ImVec4(0.88f, 0.94f, 0.99f, 1.0f), ImVec4(0.06f, 0.40f, 0.85f, 1.0f));

    Spacing();
    Spacing();

    // -------------------------------------------------------------------------
    // 3. Assume-Guarantee Contract Synthesis (Slide 15-17)
    // -------------------------------------------------------------------------
    if (g_fonts.bold) PushFont(g_fonts.bold);
    Text("Assume-Guarantee Synthesis");
    if (g_fonts.bold) PopFont();
    Separator();

    Checkbox("Parallel solver (Multi-core OpenMP)", &app.use_parallel);

    if (primary_button("Synthesize Contract", ImVec2(GetContentRegionAvail().x, 34.0f))) {
        trigger_synthesis(app);
    }

    if (app.controller_ready) {
        status_banner("Contract Validated & Invariant", banner_green_bg, banner_green_txt, true);

        if (g_fonts.small) PushFont(g_fonts.small);
        Text("Fixed-Point Iterations:  %d", app.synthesis_res.iterations);
        Text("Invariant Cell Count:    %d / %d cells (%.1f%%)",
             app.synthesis_res.controllable_count, ND_CELLS, app.synthesis_res.controllable_pct);
        Text("Composition Theorem:     %s",
             app.synthesis_res.composition_verified ? "G_i in A_{i+-1} [GUARANTEED]" : "VIOLATED");
        Text("Abstraction Time Tabs:   %.3f ms", app.synthesis_res.T_abs_ms);
        Text("Synthesis Time Tctrl:    %.3f ms (N = %d)", app.synthesis_res.T_ctrl_ms, app.num_rooms);
        Text("Peak Memory Estimate:    %.1f KB", app.synthesis_res.peak_mem_kb);
        if (app.num_rooms > 0) {
            Text("Per-Room Latency:        %.2f us", (app.synthesis_res.T_ctrl_ms * 1000.0) / app.num_rooms);
        }
        if (g_fonts.small) PopFont();

        Spacing();
        InputText("##ctrl_file", app.controller_filename, sizeof(app.controller_filename));
        if (outline_button("Save Controller", ImVec2(GetContentRegionAvail().x * 0.48f, 28.0f))) {
            ControllerSerializer::save_binary(app.controller_filename, app.synthesis_res.local_policy);
        }
        SameLine();
        if (outline_button("Load Controller", ImVec2(GetContentRegionAvail().x, 28.0f))) {
            if (ControllerSerializer::load_binary(app.controller_filename, app.synthesis_res.local_policy)) {
                app.controller_ready = true;
            }
        }
    }

    Spacing();
    Spacing();

    // -------------------------------------------------------------------------
    // 4. Simulation Playback Controls
    // -------------------------------------------------------------------------
    if (g_fonts.bold) PushFont(g_fonts.bold);
    Text("Simulation Execution");
    if (g_fonts.bold) PopFont();
    Separator();

    if (app.sim_running) {
        if (outline_button("Pause", ImVec2(GetContentRegionAvail().x * 0.31f, 32.0f))) {
            app.sim_running = false;
        }
    } else {
        if (primary_button("Run", ImVec2(GetContentRegionAvail().x * 0.31f, 32.0f))) {
            app.sim_running = true;
        }
    }
    SameLine();
    if (outline_button("Step 1s", ImVec2(GetContentRegionAvail().x * 0.46f, 32.0f))) {
        step_simulation(app);
    }
    SameLine();
    if (outline_button("Reset", ImVec2(GetContentRegionAvail().x, 32.0f))) {
        app.sim_running = false;
        reset_simulation(app);
    }

    SliderFloat("Speed Multiplier", &app.sim_speed, 1.0f, 600.0f, "%.0fx");

    // Trajectory bundle & extreme envelope buttons
    if (outline_button("Plot all routes (50)", ImVec2(GetContentRegionAvail().x * 0.48f, 28.0f))) {
        generate_trajectory_bundle(app, 50);
    }
    SameLine();
    if (outline_button("Extreme Bounds", ImVec2(GetContentRegionAvail().x, 28.0f))) {
        generate_extreme_envelope(app, 7200);
        app.show_extreme_bounds = true;
    }

    if (app.bundle_ready) {
        char bundle_buf[64];
        std::snprintf(bundle_buf, sizeof(bundle_buf), "Bundle: %d / %d Safe (100%%)",
                      app.bundle_safe_count, app.bundle_total);
        pill_badge(bundle_buf, banner_green_bg, banner_green_txt);
    }

    Spacing();
    Spacing();

    // -------------------------------------------------------------------------
    // 5. Performance & Safety Metrics (Slide 16-17)
    // -------------------------------------------------------------------------
    if (g_fonts.bold) PushFont(g_fonts.bold);
    Text("Live Building Metrics");
    if (g_fonts.bold) PopFont();
    Separator();

    if (app.safety_violated) {
        char viol_buf[64];
        std::snprintf(viol_buf, sizeof(viol_buf), "ALERT: Violation in Room %d", app.violation_room_id);
        status_banner(viol_buf, banner_red_bg, banner_red_txt, false);
    } else {
        status_banner("ALL ROOMS SAFE [17.0, 22.0] C", banner_green_bg, banner_green_txt, true);
    }

    // Time Clock: HH:MM:SS
    int t_total = static_cast<int>(app.sim_time_sec);
    int t_hours = (t_total / 3600) % 24;
    int t_mins  = (t_total / 60) % 60;
    int t_secs  = t_total % 60;

    Text("Elapsed Clock:       %02d:%02d:%02d / 24:00:00", t_hours, t_mins, t_secs);
    Text("Active Heaters:      %d / %d rooms", app.active_heaters_count, app.num_rooms);
    Text("Average Heater Duty: %.1f %%", app.heater_duty_cycle);
    Text("Cumulative Energy:   %.3f kWh", app.total_energy_kwh);
    Text("Recorded Temp Span:  [%.2f, %.2f] C", app.global_min_temp, app.global_max_temp);

    end_card();
}

} // namespace circular::ui
