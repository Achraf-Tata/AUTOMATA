#include "control_panel.hpp"
#include "ui_theme.hpp"
#include "simulation_engine.hpp"
#include "imgui.h"

#include <cstdio>
#include <cmath>
#include <algorithm>

namespace robot_nav::ui {

using namespace std;
using namespace ImGui;
using namespace theme;
using namespace colors;
using namespace engine;
using namespace grid;
using namespace unicycle;
using enum RunStatus;

void render_right_cards(AppState& app, float width) {
    // -------------------------------------------------------------------------
    // Card 1: Controller
    // -------------------------------------------------------------------------
    begin_card("##card_controller", width);

    if (g_fonts.title) PushFont(g_fonts.title);
    Text("Controller");
    if (g_fonts.title) PopFont();

    float avail_w = GetContentRegionAvail().x;
    SameLine(avail_w - 95.0f);
    if (outline_button("Run solver", ImVec2(105.0f, 28.0f))) {
        trigger_synthesis(app);
    }

    if (g_fonts.small) PushFont(g_fonts.small);
    TextColored(text_muted, "Backward reachability synthesis");
    if (g_fonts.small) PopFont();

    Spacing();
    Checkbox("Parallel solver (Multi-core)", &app.use_parallel);
    Spacing();

    // Status pill badge and metrics
    if (app.solving) {
        ProgressBar(app.solve_progress, ImVec2(avail_w, 24.0f));
        if (g_fonts.small) PushFont(g_fonts.small);
        char prog_buf[64];
        snprintf(prog_buf, sizeof(prog_buf), "Iteration %d (%d new states)",
                 app.solve_iterations, app.solve_changes);
        TextColored(text_muted, "%s", prog_buf);
        if (g_fonts.small) PopFont();
    } else {
        if (app.controller_ready) {
            pill_badge("Ready", green_badge_bg, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        } else {
            pill_badge("Not Ready", ImVec4(0.85f, 0.40f, 0.40f, 1.0f), ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        }

        SameLine(0, 10.0f);
        TextColored(text_muted, "|");
        SameLine(0, 10.0f);

        if (app.controller_ready) {
            if (app.solve_iterations > 0) {
                char iter_buf[64];
                snprintf(iter_buf, sizeof(iter_buf), "%d iterations, %.3f s",
                              app.solve_iterations, app.solve_duration);
                Text("%s", iter_buf);
            } else {
                Text("Loaded from disk");
            }
        } else {
            TextColored(text_muted, "No controller in memory");
        }

    }

    end_card();
    Spacing();

    // -------------------------------------------------------------------------
    // Card 2: Simulation
    // -------------------------------------------------------------------------
    begin_card("##card_simulation", width);

    if (g_fonts.title) PushFont(g_fonts.title);
    Text("Simulation");
    if (g_fonts.title) PopFont();

    Spacing();

    float sim_avail_w = GetContentRegionAvail().x;
    float col_w       = (sim_avail_w - 14.0f) * 0.5f;

    // Row 1: X and Y coordinates
    BeginGroup();
    AlignTextToFramePadding();
    Text("Initial X");
    SameLine();
    SetNextItemWidth(col_w - 85.0f);
    float x_val = static_cast<float>(app.robot_start.x);
    if (InputFloat("##sim_x", &x_val, 0.0f, 0.0f, "%.2f")) {
        app.robot_start.x = clamp(static_cast<double>(x_val), 0.05, 9.95);
        if (!app.sim_running) {
            app.sim_robot_pos = app.robot_start;
            app.active_trajectory.clear();
            app.active_trajectory.push_back(app.robot_start);
            app.has_run_result = false;
        }
    }
    SameLine();
    Text("m");
    EndGroup();

    SameLine(col_w + 14.0f);

    BeginGroup();
    AlignTextToFramePadding();
    Text("Initial Y");
    SameLine();
    SetNextItemWidth(col_w - 85.0f);
    float y_val = static_cast<float>(app.robot_start.y);
    if (InputFloat("##sim_y", &y_val, 0.0f, 0.0f, "%.2f")) {
        app.robot_start.y = clamp(static_cast<double>(y_val), 0.05, 9.95);
        if (!app.sim_running) {
            app.sim_robot_pos = app.robot_start;
            app.active_trajectory.clear();
            app.active_trajectory.push_back(app.robot_start);
            app.has_run_result = false;
        }
    }
    SameLine();
    Text("m");
    EndGroup();

    Spacing();

    // Row 2: Angle and Horizon Steps
    BeginGroup();
    AlignTextToFramePadding();
    Text("Heading");
    SameLine();
    SetNextItemWidth(col_w - 85.0f);
    float th_deg = static_cast<float>(app.robot_start.theta * 180.0 / M_PI);
    if (SliderFloat("##sim_th", &th_deg, -180.0f, 180.0f, "%.1f deg")) {
        app.robot_start.theta = static_cast<double>(th_deg) * M_PI / 180.0;
        if (!app.sim_running) {
            app.sim_robot_pos.theta = app.robot_start.theta;
            app.has_run_result = false;
        }
    }
    EndGroup();

    SameLine(col_w + 14.0f);

    BeginGroup();
    AlignTextToFramePadding();
    Text("Horizon");
    SameLine();
    SetNextItemWidth(col_w - 95.0f);
    InputInt("##sim_horizon", &app.sim_max_steps, 0);
    if (app.sim_max_steps < 10) app.sim_max_steps = 10;
    if (app.sim_max_steps > 1000) app.sim_max_steps = 1000;
    SameLine();
    Text("steps");
    EndGroup();

    Spacing();

    // Row 3: Animation speed and Disturbance toggle
    BeginGroup();
    AlignTextToFramePadding();
    Text("Speed");
    SameLine();
    SetNextItemWidth(col_w - 85.0f);
    SliderFloat("##sim_anim_speed", &app.anim_speed, 1.0f, 60.0f, "%.0f step/s");
    EndGroup();

    SameLine(col_w + 14.0f);

    BeginGroup();
    toggle_switch("Disturbances w in W", &app.sim_with_noise);
    if (IsItemHovered()) {
        SetTooltip("Bounded disturbance w in [-0.05, 0.05]^3 as defined in project slides.");
    }
    EndGroup();

    Spacing();

    // Row 4: Action Buttons
    if (!app.sim_running) {
        float btn_w1 = (sim_avail_w - 16.0f) * 0.50f;
        float btn_w2 = (sim_avail_w - 16.0f) * 0.25f;

        if (primary_button("Start simulation", ImVec2(btn_w1, 36.0f))) {
            if (app.arena_modified) {
                app.show_arena_modified_modal = true;
            } else {
                app.instant_sim = false;
                app.sim_running = true;
                app.sim_paused  = false;
                app.sim_step    = 0;
                app.sim_dfa_state = 1;
                app.sim_robot_pos = app.robot_start;
                app.active_trajectory.clear();
                app.active_trajectory.push_back(app.robot_start);
                app.has_run_result = false;
            }
        }

        SameLine(0, 8.0f);
        if (outline_button("Instant run", ImVec2(btn_w2, 36.0f))) {
            if (app.arena_modified) {
                app.show_arena_modified_modal = true;
            } else {
                run_simulation(app);
            }
        }

        SameLine(0, 8.0f);
        if (outline_button("Reset", ImVec2(btn_w2, 36.0f))) {
            reset_simulation(app);
            app.trajectory_bundle.clear();
        }

        Spacing();
        if (outline_button("Plot all routes (50)", ImVec2(sim_avail_w, 32.0f))) {
            if (app.arena_modified) {
                app.show_arena_modified_modal = true;
            } else if (app.controller_ready) {
                generate_trajectory_bundle(app, app.bundle_runs);
            } else {
                app.trigger_alert(
                    "Controller Not Ready",
                    "Cannot plot routes without a valid controller in memory.",
                    "The reachability feedback policy is required to simulate routes.",
                    "Click 'Run solver' first."
                );
            }
        }
        if (!app.trajectory_bundle.empty()) {
            if (g_fonts.small) PushFont(g_fonts.small);
            char b_stat[128];
            snprintf(b_stat, sizeof(b_stat), "Bundle: %d/%zu safe (%d via R1, %d via R2)",
                     app.bundle_success_count, app.trajectory_bundle.size(),
                     app.bundle_r1_count, app.bundle_r2_count);
            TextColored(text_muted, "%s", b_stat);
            if (g_fonts.small) PopFont();
        }
    } else {
        float btn_w = (sim_avail_w - 16.0f) / 3.0f;
        if (outline_button(app.sim_paused ? "Resume" : "Pause", ImVec2(btn_w, 36.0f))) {
            app.sim_paused = !app.sim_paused;
        }
        SameLine(0, 8.0f);
        if (outline_button("Step", ImVec2(btn_w, 36.0f))) {
            app.sim_paused = true;
            sim_step_once(app);
        }
        SameLine(0, 8.0f);
        if (outline_button("Stop", ImVec2(btn_w, 36.0f))) {
            app.sim_running = false;
            app.sim_paused  = false;
            app.has_run_result = true;
            app.final_state = app.sim_robot_pos;
            app.final_step = app.sim_step;
        }
    }

    end_card();
    Spacing();

    // -------------------------------------------------------------------------
    // Card 3: Run result (clean default init state)
    // -------------------------------------------------------------------------
    begin_card("##card_run_result", width);

    if (g_fonts.title) PushFont(g_fonts.title);
    Text("Run result");
    if (g_fonts.title) PopFont();

    Spacing();

    // Status banner
    if (app.sim_running) {
        status_banner("Simulation running...", banner_amber_bg, banner_amber_txt, true);
    } else if (app.has_run_result) {
        if (app.last_status == GoalReached) {
            status_banner("Goal reached (Region 3)", banner_green_bg, banner_green_txt, true);
        } else if (app.last_status == ObstacleCollision) {
            status_banner("Obstacle collision (Region 4)", banner_red_bg, banner_red_txt, false);
        } else if (app.last_status == OutOfBounds) {
            status_banner("Out of bounds", banner_red_bg, banner_red_txt, false);
        } else if (app.last_status == ExclusivityViolation) {
            status_banner("Specification violation", banner_red_bg, banner_red_txt, false);
        } else if (app.last_status == Timeout) {
            status_banner("Step limit reached", banner_amber_bg, banner_amber_txt, false);
        } else if (app.last_status == Uncontrollable) {
            status_banner("Uncontrollable state", banner_red_bg, banner_red_txt, false);
        } else {
            status_banner("Stopped", card_bg, text_muted, true);
        }
    } else {
        status_banner("Ready to run", card_bg, text_muted, true);
    }

    Spacing();

    // 4 metric cards (evenly distributed)
    float res_avail_w = GetContentRegionAvail().x;
    float gap = 8.0f;
    float box_w = (res_avail_w - 3.0f * gap) / 4.0f;
    float box_h = 50.0f;

    auto render_metric = [&](const char* label, const char* value) {
        ImVec2 p = GetCursorScreenPos();
        ImDrawList* dl = GetWindowDrawList();
        dl->AddRectFilled(p, {p.x + box_w, p.y + box_h}, IM_COL32(248, 250, 252, 255), 6.0f);
        dl->AddRect(p, {p.x + box_w, p.y + box_h}, IM_COL32(226, 232, 240, 255), 6.0f, 0, 1.0f);

        if (g_fonts.small) PushFont(g_fonts.small);
        dl->AddText({p.x + 8.0f, p.y + 6.0f}, IM_COL32(100, 116, 139, 255), label);
        if (g_fonts.small) PopFont();

        if (g_fonts.bold) PushFont(g_fonts.bold);
        dl->AddText({p.x + 8.0f, p.y + 24.0f}, IM_COL32(15, 23, 42, 255), value);
        if (g_fonts.bold) PopFont();

        Dummy(ImVec2(box_w, box_h));
    };

    char b_x[32], b_y[32], b_th[32], b_st[32];
    const State& disp_s = app.sim_running ? app.sim_robot_pos :
                                    (app.has_run_result ? app.final_state : app.robot_start);
    int disp_step = app.sim_running ? app.sim_step :
                    (app.has_run_result ? app.final_step : 0);

    snprintf(b_x, sizeof(b_x), "%.2f m", disp_s.x);
    snprintf(b_y, sizeof(b_y), "%.2f m", disp_s.y);
    snprintf(b_th, sizeof(b_th), "%.1f deg", disp_s.theta * 180.0 / M_PI);
    snprintf(b_st, sizeof(b_st), "%d / %d", disp_step, app.sim_max_steps);

    render_metric("X", b_x);
    SameLine(0, gap);
    render_metric("Y", b_y);
    SameLine(0, gap);
    render_metric("Angle", b_th);
    SameLine(0, gap);
    render_metric("Step", b_st);

    end_card();
    Spacing();

    // -------------------------------------------------------------------------
    // Card 4: Specification & Regions (Fixed coordinate labels)
    // -------------------------------------------------------------------------
    begin_card("##card_specification", width);

    if (g_fonts.title) PushFont(g_fonts.title);
    Text("Specification & Regions");
    if (g_fonts.title) PopFont();

    if (g_fonts.small) PushFont(g_fonts.small);
    TextColored(text_muted, "Visit R1 or R2, then R3, avoiding R4 at all times.");
    if (g_fonts.small) PopFont();

    Spacing();

    float spec_avail_w = GetContentRegionAvail().x;
    float reg_col_w    = (spec_avail_w - 16.0f) * 0.5f;

    auto render_region_entry = [](const char* name, const char* role,
                                  double x0, double x1, double y0, double y1,
                                  ImU32 dot_color) {
        ImVec2 p = GetCursorScreenPos();
        ImDrawList* dl = GetWindowDrawList();

        dl->AddCircleFilled({p.x + 5.0f, p.y + 8.0f}, 4.0f, dot_color);
        SetCursorPosX(GetCursorPosX() + 14.0f);

        if (g_fonts.bold) PushFont(g_fonts.bold);
        Text("%s (%s)", name, role);
        if (g_fonts.bold) PopFont();

        SetCursorPosX(GetCursorPosX() + 14.0f);
        if (g_fonts.small) PushFont(g_fonts.small);
        TextColored(text_muted, "[%.1f, %.1f] x [%.1f, %.1f]", x0, x1, y0, y1);
        if (g_fonts.small) PopFont();
    };

    BeginGroup();
    render_region_entry("R1", "Waypoint 1", app.regions.r1_x0, app.regions.r1_x1, app.regions.r1_y0, app.regions.r1_y1, IM_COL32(37, 99, 235, 255));
    Spacing();
    render_region_entry("R2", "Waypoint 2", app.regions.r2_x0, app.regions.r2_x1, app.regions.r2_y0, app.regions.r2_y1, IM_COL32(37, 99, 235, 255));
    EndGroup();

    SameLine(reg_col_w + 16.0f);

    BeginGroup();
    render_region_entry("R3", "Goal", app.regions.r3_x0, app.regions.r3_x1, app.regions.r3_y0, app.regions.r3_y1, IM_COL32(34, 197, 94, 255));
    Spacing();
    render_region_entry("R4", "Obstacle", app.regions.r4_x0, app.regions.r4_x1, app.regions.r4_y0, app.regions.r4_y1, IM_COL32(239, 68, 68, 255));
    EndGroup();

    end_card();
}

} // namespace robot_nav::ui
