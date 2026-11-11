#include "control_panel.hpp"
#include "ui_theme.hpp"
#include "simulation_engine.hpp"
#include "reachability_cpu.hpp"
#include "imgui.h"

#include <cstdio>
#include <cmath>
#include <algorithm>

namespace robot_nav::ui {

void render_header_bar(AppState& app, float width) {
    theme::begin_card("##header_bar", ImVec2(width, 50.0f));

    // Left side: Reset icon button, divider, Title and Subtitle
    if (theme::outline_button("<", ImVec2(28.0f, 26.0f))) {
        reset_simulation(app);
    }

    ImGui::SameLine(0, 12.0f);
    ImGui::TextColored(theme::colors::text_muted, "|");
    ImGui::SameLine(0, 12.0f);

    if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
    ImGui::Text("Robot Navigation");
    if (theme::g_fonts.title) ImGui::PopFont();

    ImGui::SameLine(0, 12.0f);
    if (theme::g_fonts.small) ImGui::PushFont(theme::g_fonts.small);
    ImGui::TextColored(theme::colors::text_muted, "Path Planning with Disturbances");
    if (theme::g_fonts.small) ImGui::PopFont();

    // Right side: Controller filename input, Save, Load
    float right_block_w = 330.0f;
    if (width > 600.0f) {
        ImGui::SameLine(width - right_block_w);

        ImGui::SetNextItemWidth(160.0f);
        ImGui::InputText("##ctrl_file", app.controller_filename, sizeof(app.controller_filename));

        ImGui::SameLine(0, 8.0f);
        if (theme::outline_button("Save", ImVec2(60.0f, 26.0f))) {
            if (app.controller_ready) {
                solver::save_controller_binary(app.controller_filename, app.V, app.ctrl);
            }
        }

        ImGui::SameLine(0, 6.0f);
        if (theme::outline_button("Load", ImVec2(60.0f, 26.0f))) {
            if (solver::load_controller_binary(app.controller_filename, app.V, app.ctrl)) {
                app.controller_ready = true;
            }
        }
    }

    theme::end_card();
}

void render_right_cards(AppState& app, float width) {
    // -------------------------------------------------------------------------
    // Card 1: Controller
    // -------------------------------------------------------------------------
    theme::begin_card("##card_controller", ImVec2(width, 105.0f));

    if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
    ImGui::Text("Controller");
    if (theme::g_fonts.title) ImGui::PopFont();

    ImGui::SameLine(width - 115.0f);
    if (theme::outline_button("Run solver", ImVec2(95.0f, 28.0f))) {
        engine::trigger_synthesis(app);
    }

    if (theme::g_fonts.small) ImGui::PushFont(theme::g_fonts.small);
    ImGui::TextColored(theme::colors::text_muted, "Fixed-point backward reachability synthesis");
    if (theme::g_fonts.small) ImGui::PopFont();

    ImGui::Spacing();

    // Status pill badge and metrics
    if (app.solving) {
        ImGui::ProgressBar(app.solve_progress, ImVec2(width - 32.0f, 22.0f));
    } else {
        if (app.controller_ready) {
            theme::pill_badge("Ready", theme::colors::green_badge_bg, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        } else {
            theme::pill_badge("Not Ready", ImVec4(0.85f, 0.40f, 0.40f, 1.0f), ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        }

        ImGui::SameLine(0, 14.0f);
        ImGui::TextColored(theme::colors::text_muted, "|");
        ImGui::SameLine(0, 14.0f);

        if (app.controller_ready) {
            if (app.solve_iterations > 0) {
                char iter_buf[64];
                std::snprintf(iter_buf, sizeof(iter_buf), "%d iterations, %.3f s",
                              app.solve_iterations, app.solve_duration);
                ImGui::Text("%s", iter_buf);
            } else {
                ImGui::Text("Loaded from %s", app.controller_filename);
            }
        } else {
            ImGui::TextColored(theme::colors::text_muted, "No controller in memory");
        }
    }

    theme::end_card();
    ImGui::Spacing();

    // -------------------------------------------------------------------------
    // Card 2: Simulation
    // -------------------------------------------------------------------------
    theme::begin_card("##card_simulation", ImVec2(width, 245.0f));

    if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
    ImGui::Text("Simulation");
    if (theme::g_fonts.title) ImGui::PopFont();

    ImGui::Spacing();

    float col_w = (width - 44.0f) * 0.5f;

    // Row 1: X and Heading Angle
    ImGui::AlignTextToFramePadding();
    ImGui::Text("X");
    ImGui::SameLine(24.0f);
    ImGui::SetNextItemWidth(col_w - 55.0f);
    float x_val = static_cast<float>(app.robot_start.x);
    if (ImGui::InputFloat("##sim_x", &x_val, 0.0f, 0.0f, "%.2f")) {
        app.robot_start.x = std::clamp(static_cast<double>(x_val), 0.0, 10.0);
        if (!app.sim_running) {
            app.sim_robot_pos.x = app.robot_start.x;
            app.active_trajectory.clear();
            app.active_trajectory.push_back(app.robot_start);
            app.has_run_result = false;
        }
    }
    ImGui::SameLine();
    ImGui::Text("m");

    ImGui::SameLine(col_w + 35.0f);
    ImGui::Text("Angle");
    ImGui::SameLine(col_w + 80.0f);
    ImGui::SetNextItemWidth(col_w - 95.0f);
    float th_deg = static_cast<float>(app.robot_start.theta * 180.0 / M_PI);
    if (ImGui::SliderFloat("##sim_th", &th_deg, -180.0f, 180.0f, "%.1f deg")) {
        app.robot_start.theta = static_cast<double>(th_deg) * M_PI / 180.0;
        if (!app.sim_running) {
            app.sim_robot_pos.theta = app.robot_start.theta;
            app.has_run_result = false;
        }
    }

    // Row 2: Y and Horizon
    ImGui::AlignTextToFramePadding();
    ImGui::Text("Y");
    ImGui::SameLine(24.0f);
    ImGui::SetNextItemWidth(col_w - 55.0f);
    float y_val = static_cast<float>(app.robot_start.y);
    if (ImGui::InputFloat("##sim_y", &y_val, 0.0f, 0.0f, "%.2f")) {
        app.robot_start.y = std::clamp(static_cast<double>(y_val), 0.0, 10.0);
        if (!app.sim_running) {
            app.sim_robot_pos.y = app.robot_start.y;
            app.active_trajectory.clear();
            app.active_trajectory.push_back(app.robot_start);
            app.has_run_result = false;
        }
    }
    ImGui::SameLine();
    ImGui::Text("m");

    ImGui::SameLine(col_w + 35.0f);
    ImGui::Text("Steps");
    ImGui::SameLine(col_w + 80.0f);
    ImGui::SetNextItemWidth(col_w - 95.0f);
    ImGui::InputInt("##sim_horizon", &app.sim_max_steps, 0);

    // Row 3: Animation speed and Disturbance toggle
    ImGui::Spacing();
    ImGui::AlignTextToFramePadding();
    ImGui::Text("Speed");
    ImGui::SameLine(50.0f);
    ImGui::SetNextItemWidth(140.0f);
    ImGui::SliderFloat("##sim_anim_speed", &app.anim_speed, 1.0f, 60.0f, "%.0f steps/s");

    ImGui::SameLine(210.0f);
    theme::toggle_switch("Noise w in [-0.05, 0.05]", &app.sim_with_noise);

    ImGui::Spacing();

    // Simulation Action Buttons
    if (!app.sim_running) {
        float btn_w1 = (width - 46.0f) * 0.50f;
        float btn_w2 = (width - 46.0f) * 0.25f;

        if (theme::primary_button("Start simulation", ImVec2(btn_w1, 34.0f))) {
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

        ImGui::SameLine();
        if (theme::outline_button("Instant run", ImVec2(btn_w2, 34.0f))) {
            if (app.arena_modified) {
                app.show_arena_modified_modal = true;
            } else {
                engine::run_simulation(app);
            }
        }

        ImGui::SameLine();
        if (theme::outline_button("Reset", ImVec2(btn_w2, 34.0f))) {
            reset_simulation(app);
        }
    } else {
        float btn_w = (width - 46.0f) / 3.0f;
        if (theme::outline_button(app.sim_paused ? "Resume" : "Pause", ImVec2(btn_w, 34.0f))) {
            app.sim_paused = !app.sim_paused;
        }
        ImGui::SameLine();
        if (theme::outline_button("Step", ImVec2(btn_w, 34.0f))) {
            app.sim_paused = true;
            engine::sim_step_once(app);
        }
        ImGui::SameLine();
        if (theme::outline_button("Stop", ImVec2(btn_w, 34.0f))) {
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
    theme::begin_card("##card_run_result", ImVec2(width, 130.0f));

    if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
    ImGui::Text("Run result");
    if (theme::g_fonts.title) ImGui::PopFont();

    ImGui::Spacing();

    // Status banner
    if (app.sim_running) {
        theme::status_banner("Simulation running...", theme::colors::banner_amber_bg, theme::colors::banner_amber_txt, true);
    } else if (app.has_run_result) {
        if (app.last_status == RunStatus::GoalReached) {
            theme::status_banner("Goal reached", theme::colors::banner_green_bg, theme::colors::banner_green_txt, true);
        } else if (app.last_status == RunStatus::ObstacleCollision) {
            theme::status_banner("Obstacle collision (R4)", theme::colors::banner_red_bg, theme::colors::banner_red_txt, false);
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

    // 4 metric columns
    float col_step = (width - 32.0f) / 4.0f;
    auto metric_box = [&](int col_idx, const char* label, const char* value) {
        float x_pos = 16.0f + col_idx * col_step;
        ImGui::SetCursorPosX(x_pos);

        if (theme::g_fonts.small) ImGui::PushFont(theme::g_fonts.small);
        ImGui::TextColored(theme::colors::text_muted, "%s", label);
        if (theme::g_fonts.small) ImGui::PopFont();

        ImGui::SetCursorPosX(x_pos);
        if (theme::g_fonts.bold) ImGui::PushFont(theme::g_fonts.bold);
        ImGui::Text("%s", value);
        if (theme::g_fonts.bold) ImGui::PopFont();
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

    ImVec2 saved_pos = ImGui::GetCursorPos();
    metric_box(0, "X", b_x);
    ImGui::SetCursorPos({saved_pos.x, saved_pos.y});
    metric_box(1, "Y", b_y);
    ImGui::SetCursorPos({saved_pos.x, saved_pos.y});
    metric_box(2, "Angle", b_th);
    ImGui::SetCursorPos({saved_pos.x, saved_pos.y});
    metric_box(3, "Step", b_st);

    theme::end_card();
    ImGui::Spacing();

    // -------------------------------------------------------------------------
    // Card 4: Monte Carlo
    // -------------------------------------------------------------------------
    theme::begin_card("##card_monte_carlo", ImVec2(width, 130.0f));

    if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
    ImGui::Text("Monte Carlo");
    if (theme::g_fonts.title) ImGui::PopFont();

    ImGui::Spacing();

    ImGui::AlignTextToFramePadding();
    ImGui::Text("Sample runs");
    ImGui::SameLine(110.0f);
    ImGui::SetNextItemWidth(80.0f);
    ImGui::InputInt("##mc_samples", &app.mc_samples, 0);

    ImGui::SameLine(width - 135.0f);
    if (theme::outline_button("Run analysis", ImVec2(115.0f, 28.0f))) {
        engine::run_monte_carlo_analysis(app);
    }

    ImGui::Spacing();

    // Progress bar
    ImVec2 p_bar = ImGui::GetCursorScreenPos();
    float bar_w = width - 32.0f;
    float bar_h = 6.0f;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p_bar, {p_bar.x + bar_w, p_bar.y + bar_h}, IM_COL32(226, 232, 240, 255), 3.0f);
    float fill_w = bar_w * std::clamp(app.mc_progress, 0.0f, 1.0f);
    dl->AddRectFilled(p_bar, {p_bar.x + fill_w, p_bar.y + bar_h}, IM_COL32(34, 197, 94, 255), 3.0f);
    ImGui::Dummy(ImVec2(bar_w, bar_h));

    ImGui::Spacing();

    // Result summary text
    if (app.mc_stats.total_runs > 0) {
        ImVec2 check_pos = ImGui::GetCursorScreenPos();
        float check_r = 6.0f;
        ImVec2 c_ctr = {check_pos.x + 8.0f, check_pos.y + 9.0f};
        dl->AddCircleFilled(c_ctr, check_r, IM_COL32(34, 197, 94, 255));
        dl->AddLine({c_ctr.x - 3.0f, c_ctr.y}, {c_ctr.x - 1.0f, c_ctr.y + 2.0f}, IM_COL32(255, 255, 255, 255), 1.5f);
        dl->AddLine({c_ctr.x - 1.0f, c_ctr.y + 2.0f}, {c_ctr.x + 3.0f, c_ctr.y - 2.0f}, IM_COL32(255, 255, 255, 255), 1.5f);

        ImGui::SetCursorPosX(32.0f);
        char mc_summary[128];
        std::snprintf(mc_summary, sizeof(mc_summary), "%d passed, %d failed, %d timed out",
                      app.mc_stats.successful, app.mc_stats.failed, app.mc_stats.timeout);
        ImGui::Text("%s", mc_summary);
    } else {
        ImGui::TextColored(theme::colors::text_muted, "No analysis performed yet.");
    }

    theme::end_card();
    ImGui::Spacing();

    // -------------------------------------------------------------------------
    // Card 5: Arena Regions
    // -------------------------------------------------------------------------
    theme::begin_card("##card_arena_regions", ImVec2(width, 100.0f));

    if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
    ImGui::Text("Arena Regions");
    if (theme::g_fonts.title) ImGui::PopFont();

    ImGui::Spacing();

    float col2_w = (width - 32.0f) * 0.5f;

    // Column 1
    ImGui::BeginGroup();
    ImGui::Text("R1: [%.1f, %.1f] x [%.1f, %.1f]",
                app.regions.r1_x0, app.regions.r1_x1, app.regions.r1_y0, app.regions.r1_y1);
    ImGui::Text("R2: [%.1f, %.1f] x [%.1f, %.1f]",
                app.regions.r2_x0, app.regions.r2_x1, app.regions.r2_y0, app.regions.r2_y1);
    ImGui::EndGroup();

    ImGui::SameLine(col2_w + 16.0f);

    // Column 2
    ImGui::BeginGroup();
    ImGui::Text("R3 (Goal): [%.1f, %.1f] x [%.1f, %.1f]",
                app.regions.r3_x0, app.regions.r3_y0, app.regions.r3_y1, app.regions.r3_y1);
    ImGui::Text("R4 (Obs): [%.1f, %.1f] x [%.1f, %.1f]",
                app.regions.r4_x0, app.regions.r4_x1, app.regions.r4_y0, app.regions.r4_y1);
    ImGui::EndGroup();

    theme::end_card();
}

} // namespace robot_nav::ui
