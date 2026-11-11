#include "control_panel.hpp"
#include "ui_theme.hpp"
#include "simulation_engine.hpp"
#include "reachability_cpu.hpp"
#include "imgui.h"

#include <cstdio>
#include <cmath>
#include <algorithm>

namespace robot_nav::ui {

void render_right_cards(AppState& app, float width) {
    // -------------------------------------------------------------------------
    // Card 1: Controller
    // -------------------------------------------------------------------------
    theme::begin_card("##card_controller", width);

    if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
    ImGui::Text("Controller");
    if (theme::g_fonts.title) ImGui::PopFont();

    float avail_w = ImGui::GetContentRegionAvail().x;
    ImGui::SameLine(avail_w - 95.0f);
    if (theme::outline_button("Run solver", ImVec2(105.0f, 28.0f))) {
        engine::trigger_synthesis(app);
    }

    if (theme::g_fonts.small) ImGui::PushFont(theme::g_fonts.small);
    ImGui::TextColored(theme::colors::text_muted, "Backward reachability synthesis (35 iterations)");
    if (theme::g_fonts.small) ImGui::PopFont();

    ImGui::Spacing();

    // Status pill badge and metrics + Save/Load
    if (app.solving) {
        ImGui::ProgressBar(app.solve_progress, ImVec2(avail_w, 24.0f));
    } else {
        if (app.controller_ready) {
            theme::pill_badge("Ready", theme::colors::green_badge_bg, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        } else {
            theme::pill_badge("Not Ready", ImVec4(0.85f, 0.40f, 0.40f, 1.0f), ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        }

        ImGui::SameLine(0, 10.0f);
        ImGui::TextColored(theme::colors::text_muted, "|");
        ImGui::SameLine(0, 10.0f);

        if (app.controller_ready) {
            if (app.solve_iterations > 0) {
                char iter_buf[64];
                std::snprintf(iter_buf, sizeof(iter_buf), "%d iterations, %.3f s",
                              app.solve_iterations, app.solve_duration);
                ImGui::Text("%s", iter_buf);
            } else {
                ImGui::Text("Loaded from disk");
            }
        } else {
            ImGui::TextColored(theme::colors::text_muted, "No controller in memory");
        }

        // Save & Load buttons on the right of the status row
        ImGui::SameLine(avail_w - 110.0f);
        if (theme::outline_button("Save", ImVec2(52.0f, 24.0f))) {
            if (app.controller_ready) {
                solver::save_controller_binary(app.controller_filename, app.V, app.ctrl);
            }
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Save controller policy to %s", app.controller_filename);
        }

        ImGui::SameLine(0, 6.0f);
        if (theme::outline_button("Load", ImVec2(52.0f, 24.0f))) {
            if (solver::load_controller_binary(app.controller_filename, app.V, app.ctrl)) {
                app.controller_ready = true;
            }
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Load controller policy from %s", app.controller_filename);
        }
    }

    theme::end_card();
    ImGui::Spacing();

    // -------------------------------------------------------------------------
    // Card 2: Simulation
    // -------------------------------------------------------------------------
    theme::begin_card("##card_simulation", width);

    if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
    ImGui::Text("Simulation");
    if (theme::g_fonts.title) ImGui::PopFont();

    ImGui::Spacing();

    float sim_avail_w = ImGui::GetContentRegionAvail().x;
    float col_w       = (sim_avail_w - 14.0f) * 0.5f;

    // Row 1: X and Y coordinates
    ImGui::BeginGroup();
    ImGui::AlignTextToFramePadding();
    ImGui::Text("Initial X");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(col_w - 85.0f);
    float x_val = static_cast<float>(app.robot_start.x);
    if (ImGui::InputFloat("##sim_x", &x_val, 0.0f, 0.0f, "%.2f")) {
        app.robot_start.x = std::clamp(static_cast<double>(x_val), 0.05, 9.95);
        if (!app.sim_running) {
            app.sim_robot_pos = app.robot_start;
            app.active_trajectory.clear();
            app.active_trajectory.push_back(app.robot_start);
            app.has_run_result = false;
        }
    }
    ImGui::SameLine();
    ImGui::Text("m");
    ImGui::EndGroup();

    ImGui::SameLine(col_w + 14.0f);

    ImGui::BeginGroup();
    ImGui::AlignTextToFramePadding();
    ImGui::Text("Initial Y");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(col_w - 85.0f);
    float y_val = static_cast<float>(app.robot_start.y);
    if (ImGui::InputFloat("##sim_y", &y_val, 0.0f, 0.0f, "%.2f")) {
        app.robot_start.y = std::clamp(static_cast<double>(y_val), 0.05, 9.95);
        if (!app.sim_running) {
            app.sim_robot_pos = app.robot_start;
            app.active_trajectory.clear();
            app.active_trajectory.push_back(app.robot_start);
            app.has_run_result = false;
        }
    }
    ImGui::SameLine();
    ImGui::Text("m");
    ImGui::EndGroup();

    ImGui::Spacing();

    // Row 2: Angle and Horizon Steps
    ImGui::BeginGroup();
    ImGui::AlignTextToFramePadding();
    ImGui::Text("Heading");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(col_w - 85.0f);
    float th_deg = static_cast<float>(app.robot_start.theta * 180.0 / M_PI);
    if (ImGui::SliderFloat("##sim_th", &th_deg, -180.0f, 180.0f, "%.1f deg")) {
        app.robot_start.theta = static_cast<double>(th_deg) * M_PI / 180.0;
        if (!app.sim_running) {
            app.sim_robot_pos.theta = app.robot_start.theta;
            app.has_run_result = false;
        }
    }
    ImGui::EndGroup();

    ImGui::SameLine(col_w + 14.0f);

    ImGui::BeginGroup();
    ImGui::AlignTextToFramePadding();
    ImGui::Text("Horizon");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(col_w - 95.0f);
    ImGui::InputInt("##sim_horizon", &app.sim_max_steps, 0);
    if (app.sim_max_steps < 10) app.sim_max_steps = 10;
    if (app.sim_max_steps > 1000) app.sim_max_steps = 1000;
    ImGui::SameLine();
    ImGui::Text("steps");
    ImGui::EndGroup();

    ImGui::Spacing();

    // Row 3: Animation speed and Disturbance toggle
    ImGui::BeginGroup();
    ImGui::AlignTextToFramePadding();
    ImGui::Text("Speed");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(col_w - 85.0f);
    ImGui::SliderFloat("##sim_anim_speed", &app.anim_speed, 1.0f, 60.0f, "%.0f step/s");
    ImGui::EndGroup();

    ImGui::SameLine(col_w + 14.0f);

    ImGui::BeginGroup();
    theme::toggle_switch("Disturbances w in W", &app.sim_with_noise);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Bounded disturbance w in [-0.05, 0.05]^3 as defined in project slides.");
    }
    ImGui::EndGroup();

    ImGui::Spacing();

    // Row 4: Action Buttons
    if (!app.sim_running) {
        float btn_w1 = (sim_avail_w - 16.0f) * 0.50f;
        float btn_w2 = (sim_avail_w - 16.0f) * 0.25f;

        if (theme::primary_button("Start simulation", ImVec2(btn_w1, 36.0f))) {
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

        ImGui::SameLine(0, 8.0f);
        if (theme::outline_button("Instant run", ImVec2(btn_w2, 36.0f))) {
            if (app.arena_modified) {
                app.show_arena_modified_modal = true;
            } else {
                engine::run_simulation(app);
            }
        }

        ImGui::SameLine(0, 8.0f);
        if (theme::outline_button("Reset", ImVec2(btn_w2, 36.0f))) {
            reset_simulation(app);
        }
    } else {
        float btn_w = (sim_avail_w - 16.0f) / 3.0f;
        if (theme::outline_button(app.sim_paused ? "Resume" : "Pause", ImVec2(btn_w, 36.0f))) {
            app.sim_paused = !app.sim_paused;
        }
        ImGui::SameLine(0, 8.0f);
        if (theme::outline_button("Step", ImVec2(btn_w, 36.0f))) {
            app.sim_paused = true;
            engine::sim_step_once(app);
        }
        ImGui::SameLine(0, 8.0f);
        if (theme::outline_button("Stop", ImVec2(btn_w, 36.0f))) {
            app.sim_running = false;
            app.sim_paused  = false;
            app.has_run_result = true;
            app.final_state = app.sim_robot_pos;
            app.final_step = app.sim_step;
        }
    }

    theme::end_card();
    ImGui::Spacing();

    // -------------------------------------------------------------------------
    // Card 3: Run result (clean default init state)
    // -------------------------------------------------------------------------
    theme::begin_card("##card_run_result", width);

    if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
    ImGui::Text("Run result");
    if (theme::g_fonts.title) ImGui::PopFont();

    ImGui::Spacing();

    // Status banner
    if (app.sim_running) {
        theme::status_banner("Simulation running...", theme::colors::banner_amber_bg, theme::colors::banner_amber_txt, true);
    } else if (app.has_run_result) {
        if (app.last_status == RunStatus::GoalReached) {
            theme::status_banner("Goal reached (Region 3)", theme::colors::banner_green_bg, theme::colors::banner_green_txt, true);
        } else if (app.last_status == RunStatus::ObstacleCollision) {
            theme::status_banner("Obstacle collision (Region 4)", theme::colors::banner_red_bg, theme::colors::banner_red_txt, false);
        } else if (app.last_status == RunStatus::OutOfBounds) {
            theme::status_banner("Out of bounds", theme::colors::banner_red_bg, theme::colors::banner_red_txt, false);
        } else if (app.last_status == RunStatus::ExclusivityViolation) {
            theme::status_banner("Specification violation", theme::colors::banner_red_bg, theme::colors::banner_red_txt, false);
        } else if (app.last_status == RunStatus::Timeout) {
            theme::status_banner("Step limit reached", theme::colors::banner_amber_bg, theme::colors::banner_amber_txt, false);
        } else if (app.last_status == RunStatus::Uncontrollable) {
            theme::status_banner("Uncontrollable state", theme::colors::banner_red_bg, theme::colors::banner_red_txt, false);
        } else {
            theme::status_banner("Stopped", theme::colors::card_bg, theme::colors::text_muted, true);
        }
    } else {
        theme::status_banner("Ready to run", theme::colors::card_bg, theme::colors::text_muted, true);
    }

    ImGui::Spacing();

    // 4 metric cards (evenly distributed)
    float res_avail_w = ImGui::GetContentRegionAvail().x;
    float gap = 8.0f;
    float box_w = (res_avail_w - 3.0f * gap) / 4.0f;
    float box_h = 50.0f;

    auto render_metric = [&](const char* label, const char* value) {
        ImVec2 p = ImGui::GetCursorScreenPos();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(p, {p.x + box_w, p.y + box_h}, IM_COL32(248, 250, 252, 255), 6.0f);
        dl->AddRect(p, {p.x + box_w, p.y + box_h}, IM_COL32(226, 232, 240, 255), 6.0f, 0, 1.0f);

        if (theme::g_fonts.small) ImGui::PushFont(theme::g_fonts.small);
        dl->AddText({p.x + 8.0f, p.y + 6.0f}, IM_COL32(100, 116, 139, 255), label);
        if (theme::g_fonts.small) ImGui::PopFont();

        if (theme::g_fonts.bold) ImGui::PushFont(theme::g_fonts.bold);
        dl->AddText({p.x + 8.0f, p.y + 24.0f}, IM_COL32(15, 23, 42, 255), value);
        if (theme::g_fonts.bold) ImGui::PopFont();

        ImGui::Dummy(ImVec2(box_w, box_h));
    };

    char b_x[32], b_y[32], b_th[32], b_st[32];
    const unicycle::State& disp_s = app.sim_running ? app.sim_robot_pos :
                                    (app.has_run_result ? app.final_state : app.robot_start);
    int disp_step = app.sim_running ? app.sim_step :
                    (app.has_run_result ? app.final_step : 0);

    std::snprintf(b_x, sizeof(b_x), "%.2f m", disp_s.x);
    std::snprintf(b_y, sizeof(b_y), "%.2f m", disp_s.y);
    std::snprintf(b_th, sizeof(b_th), "%.1f deg", disp_s.theta * 180.0 / M_PI);
    std::snprintf(b_st, sizeof(b_st), "%d / %d", disp_step, app.sim_max_steps);

    render_metric("X", b_x);
    ImGui::SameLine(0, gap);
    render_metric("Y", b_y);
    ImGui::SameLine(0, gap);
    render_metric("Angle", b_th);
    ImGui::SameLine(0, gap);
    render_metric("Step", b_st);

    theme::end_card();
    ImGui::Spacing();

    // -------------------------------------------------------------------------
    // Card 4: Specification & Regions (Fixed coordinate labels)
    // -------------------------------------------------------------------------
    theme::begin_card("##card_specification", width);

    if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
    ImGui::Text("Specification & Regions");
    if (theme::g_fonts.title) ImGui::PopFont();

    if (theme::g_fonts.small) ImGui::PushFont(theme::g_fonts.small);
    ImGui::TextColored(theme::colors::text_muted, "Visit R1 or R2, then R3, avoiding R4 at all times.");
    if (theme::g_fonts.small) ImGui::PopFont();

    ImGui::Spacing();

    float spec_avail_w = ImGui::GetContentRegionAvail().x;
    float reg_col_w    = (spec_avail_w - 16.0f) * 0.5f;

    auto render_region_entry = [](const char* name, const char* role,
                                  double x0, double x1, double y0, double y1,
                                  ImU32 dot_color) {
        ImVec2 p = ImGui::GetCursorScreenPos();
        ImDrawList* dl = ImGui::GetWindowDrawList();

        dl->AddCircleFilled({p.x + 5.0f, p.y + 8.0f}, 4.0f, dot_color);
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 14.0f);

        if (theme::g_fonts.bold) ImGui::PushFont(theme::g_fonts.bold);
        ImGui::Text("%s (%s)", name, role);
        if (theme::g_fonts.bold) ImGui::PopFont();

        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 14.0f);
        if (theme::g_fonts.small) ImGui::PushFont(theme::g_fonts.small);
        ImGui::TextColored(theme::colors::text_muted, "[%.1f, %.1f] x [%.1f, %.1f]", x0, x1, y0, y1);
        if (theme::g_fonts.small) ImGui::PopFont();
    };

    ImGui::BeginGroup();
    render_region_entry("R1", "Waypoint 1", app.regions.r1_x0, app.regions.r1_x1, app.regions.r1_y0, app.regions.r1_y1, IM_COL32(37, 99, 235, 255));
    ImGui::Spacing();
    render_region_entry("R2", "Waypoint 2", app.regions.r2_x0, app.regions.r2_x1, app.regions.r2_y0, app.regions.r2_y1, IM_COL32(37, 99, 235, 255));
    ImGui::EndGroup();

    ImGui::SameLine(reg_col_w + 16.0f);

    ImGui::BeginGroup();
    render_region_entry("R3", "Goal", app.regions.r3_x0, app.regions.r3_x1, app.regions.r3_y0, app.regions.r3_y1, IM_COL32(34, 197, 94, 255));
    ImGui::Spacing();
    render_region_entry("R4", "Obstacle", app.regions.r4_x0, app.regions.r4_x1, app.regions.r4_y0, app.regions.r4_y1, IM_COL32(239, 68, 68, 255));
    ImGui::EndGroup();

    theme::end_card();
}

} // namespace robot_nav::ui
