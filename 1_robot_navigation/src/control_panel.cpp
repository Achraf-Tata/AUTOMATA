#include "control_panel.hpp"
#include "ui_theme.hpp"
#include "simulation_engine.hpp"
#include "reachability_cpu.hpp"
#include "imgui.h"

#include <cstdio>
#include <cmath>

namespace robot_nav::ui {

void render_header_bar(AppState& app, float width) {
    theme::begin_card("##header_bar", ImVec2(width, 54.0f));

    // Left side: Arrow button, divider, Title
    if (theme::outline_button("\xe2\x86\x90", ImVec2(32.0f, 28.0f))) {
        // Back action / reset initial view
    }

    ImGui::SameLine(0, 14.0f);
    ImGui::TextColored(theme::colors::text_muted, "|");
    ImGui::SameLine(0, 14.0f);

    if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
    ImGui::Text("Robot Navigation");
    if (theme::g_fonts.title) ImGui::PopFont();

    // Right side: Controller filename input, Save, Load
    float right_block_w = 340.0f;
    ImGui::SameLine(width - right_block_w);

    ImGui::SetNextItemWidth(170.0f);
    ImGui::InputText("##ctrl_file", app.controller_filename, sizeof(app.controller_filename));

    ImGui::SameLine(0, 10.0f);
    if (theme::outline_button("Save", ImVec2(65.0f, 28.0f))) {
        if (app.controller_ready) {
            solver::save_controller_binary(app.controller_filename, app.V, app.ctrl);
        }
    }

    ImGui::SameLine(0, 8.0f);
    if (theme::outline_button("Load", ImVec2(65.0f, 28.0f))) {
        if (solver::load_controller_binary(app.controller_filename, app.V, app.ctrl)) {
            app.controller_ready = true;
        }
    }

    theme::end_card();
}

void render_right_cards(AppState& app, float width) {
    // -------------------------------------------------------------------------
    // Card 1: Controller
    // -------------------------------------------------------------------------
    theme::begin_card("##card_controller", ImVec2(width, 118.0f));

    if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
    ImGui::Text("Controller");
    if (theme::g_fonts.title) ImGui::PopFont();

    if (theme::g_fonts.small) ImGui::PushFont(theme::g_fonts.small);
    ImGui::TextColored(theme::colors::text_muted, "Symbolic Control for Non-linear Systems (Girard et al.)");
    if (theme::g_fonts.small) ImGui::PopFont();

    ImGui::Spacing();

    // Status pill badge and metrics
    if (app.solving) {
        ImGui::ProgressBar(app.solve_progress, ImVec2(width - 170.0f, 28.0f));
    } else {
        if (app.controller_ready) {
            theme::pill_badge("\xe2\x9c\x93 Ready", theme::colors::green_badge_bg, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        } else {
            theme::pill_badge("Not Ready", ImVec4(0.85f, 0.40f, 0.40f, 1.0f), ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        }

        ImGui::SameLine(0, 16.0f);
        ImGui::TextColored(theme::colors::text_muted, "|");
        ImGui::SameLine(0, 16.0f);

        char iter_buf[64];
        std::snprintf(iter_buf, sizeof(iter_buf), "%d iterations \xe2\x80\xa2 %.3f s",
                      app.solve_iterations, app.solve_duration);
        ImGui::Text("%s", iter_buf);
    }

    ImGui::SameLine(width - 130.0f);
    if (theme::outline_button("Run solver", ImVec2(100.0f, 30.0f))) {
        engine::trigger_synthesis(app);
    }

    theme::end_card();
    ImGui::Spacing();

    // -------------------------------------------------------------------------
    // Card 2: Simulation
    // -------------------------------------------------------------------------
    theme::begin_card("##card_simulation", ImVec2(width, 215.0f));

    if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
    ImGui::Text("Simulation");
    if (theme::g_fonts.title) ImGui::PopFont();

    ImGui::Spacing();

    // 2x2 Input grid
    float col_w = (width - 60.0f) * 0.5f;

    // Row 1: X and Heading
    ImGui::AlignTextToFramePadding();
    ImGui::Text("X");
    ImGui::SameLine(30.0f);
    ImGui::SetNextItemWidth(col_w - 70.0f);
    float x_val = static_cast<float>(app.robot_start.x);
    if (ImGui::InputFloat("##sim_x", &x_val, 0.0f, 0.0f, "%.2f")) {
        app.robot_start.x = std::clamp(static_cast<double>(x_val), 0.0, 10.0);
    }
    ImGui::SameLine();
    ImGui::Text("m");

    ImGui::SameLine(col_w + 50.0f);
    ImGui::Text("Heading \xce\xb8");
    ImGui::SameLine(col_w + 140.0f);
    ImGui::SetNextItemWidth(col_w - 170.0f);
    float th_deg = static_cast<float>(app.robot_start.theta * 180.0 / M_PI);
    if (ImGui::InputFloat("##sim_th", &th_deg, 0.0f, 0.0f, "%.1f")) {
        app.robot_start.theta = static_cast<double>(th_deg) * M_PI / 180.0;
    }
    ImGui::SameLine();
    ImGui::Text("\xc2\xb0");

    // Row 2: Y and Horizon
    ImGui::AlignTextToFramePadding();
    ImGui::Text("Y");
    ImGui::SameLine(30.0f);
    ImGui::SetNextItemWidth(col_w - 70.0f);
    float y_val = static_cast<float>(app.robot_start.y);
    if (ImGui::InputFloat("##sim_y", &y_val, 0.0f, 0.0f, "%.2f")) {
        app.robot_start.y = std::clamp(static_cast<double>(y_val), 0.0, 10.0);
    }
    ImGui::SameLine();
    ImGui::Text("m");

    ImGui::SameLine(col_w + 50.0f);
    ImGui::Text("Horizon");
    ImGui::SameLine(col_w + 140.0f);
    ImGui::SetNextItemWidth(col_w - 170.0f);
    ImGui::InputInt("##sim_horizon", &app.sim_max_steps, 0);

    ImGui::Spacing();
    theme::toggle_switch("Disturbance w \xe2\x88\x88 [-0.05, 0.05]", &app.sim_with_noise);

    ImGui::Spacing();
    // Primary simulation action button
    if (theme::primary_button("\xe2\x96\xb6 Run simulation", ImVec2(width - 32.0f, 40.0f))) {
        if (app.arena_modified) {
            app.show_arena_modified_modal = true;
        } else {
            engine::run_simulation(app);
        }
    }

    theme::end_card();
    ImGui::Spacing();

    // -------------------------------------------------------------------------
    // Card 3: Run result
    // -------------------------------------------------------------------------
    theme::begin_card("##card_run_result", ImVec2(width, 140.0f));

    if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
    ImGui::Text("Run result");
    if (theme::g_fonts.title) ImGui::PopFont();

    ImGui::Spacing();

    // Status banner
    if (app.last_status == RunStatus::GoalReached) {
        theme::status_banner("Goal reached", theme::colors::banner_green_bg, theme::colors::banner_green_txt, true);
    } else if (app.last_status == RunStatus::ObstacleCollision) {
        theme::status_banner("Obstacle collision", theme::colors::banner_red_bg, theme::colors::banner_red_txt, false);
    } else if (app.last_status == RunStatus::OutOfBounds) {
        theme::status_banner("Out of bounds", theme::colors::banner_red_bg, theme::colors::banner_red_txt, false);
    } else if (app.last_status == RunStatus::ExclusivityViolation) {
        theme::status_banner("Specification violation", theme::colors::banner_red_bg, theme::colors::banner_red_txt, false);
    } else if (app.last_status == RunStatus::Timeout) {
        theme::status_banner("Horizon timeout", theme::colors::banner_amber_bg, theme::colors::banner_amber_txt, false);
    } else if (app.last_status == RunStatus::Uncontrollable) {
        theme::status_banner("Uncontrollable state", theme::colors::banner_red_bg, theme::colors::banner_red_txt, false);
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
    std::snprintf(b_x, sizeof(b_x), "%.2f m", app.final_state.x);
    std::snprintf(b_y, sizeof(b_y), "%.2f m", app.final_state.y);
    std::snprintf(b_th, sizeof(b_th), "%.1f \xc2\xb0", app.final_state.theta * 180.0 / M_PI);
    std::snprintf(b_st, sizeof(b_st), "%d / %d", app.final_step, app.sim_max_steps);

    ImVec2 saved_pos = ImGui::GetCursorPos();
    metric_box(0, "X", b_x);
    ImGui::SetCursorPos({saved_pos.x, saved_pos.y});
    metric_box(1, "Y", b_y);
    ImGui::SetCursorPos({saved_pos.x, saved_pos.y});
    metric_box(2, "\xce\xb8", b_th);
    ImGui::SetCursorPos({saved_pos.x, saved_pos.y});
    metric_box(3, "Step", b_st);

    theme::end_card();
    ImGui::Spacing();

    // -------------------------------------------------------------------------
    // Card 4: Monte Carlo
    // -------------------------------------------------------------------------
    theme::begin_card("##card_monte_carlo", ImVec2(width, 138.0f));

    if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
    ImGui::Text("Monte Carlo");
    if (theme::g_fonts.title) ImGui::PopFont();

    ImGui::Spacing();

    // Sample runs input and action button
    ImGui::AlignTextToFramePadding();
    ImGui::Text("Sample runs");
    ImGui::SameLine(120.0f);
    ImGui::SetNextItemWidth(100.0f);
    ImGui::InputInt("##mc_samples", &app.mc_samples, 0);

    ImGui::SameLine(width - 150.0f);
    if (theme::outline_button("Run analysis", ImVec2(120.0f, 30.0f))) {
        engine::run_monte_carlo_analysis(app);
    }

    ImGui::Spacing();

    // Progress bar
    ImVec2 p_bar = ImGui::GetCursorScreenPos();
    float bar_w = width - 32.0f;
    float bar_h = 8.0f;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p_bar, {p_bar.x + bar_w, p_bar.y + bar_h}, IM_COL32(226, 232, 240, 255), 4.0f);
    float fill_w = bar_w * std::clamp(app.mc_progress, 0.0f, 1.0f);
    dl->AddRectFilled(p_bar, {p_bar.x + fill_w, p_bar.y + bar_h}, IM_COL32(34, 197, 94, 255), 4.0f);
    ImGui::Dummy(ImVec2(bar_w, bar_h));

    ImGui::Spacing();

    // Result summary text with green checkmark
    ImVec2 check_pos = ImGui::GetCursorScreenPos();
    float check_r = 7.0f;
    ImVec2 c_ctr = {check_pos.x + 8.0f, check_pos.y + 10.0f};
    dl->AddCircleFilled(c_ctr, check_r, IM_COL32(34, 197, 94, 255));
    dl->AddLine({c_ctr.x - 3.5f, c_ctr.y}, {c_ctr.x - 1.0f, c_ctr.y + 2.5f}, IM_COL32(255, 255, 255, 255), 1.6f);
    dl->AddLine({c_ctr.x - 1.0f, c_ctr.y + 2.5f}, {c_ctr.x + 3.5f, c_ctr.y - 2.5f}, IM_COL32(255, 255, 255, 255), 1.6f);

    ImGui::SetCursorPosX(36.0f);
    char mc_summary[128];
    std::snprintf(mc_summary, sizeof(mc_summary), "%d passed \xe2\x80\xa2 %d failed \xe2\x80\xa2 %d timed out",
                  app.mc_stats.successful, app.mc_stats.failed, app.mc_stats.timeout);
    ImGui::Text("%s", mc_summary);

    theme::end_card();
    ImGui::Spacing();

    // -------------------------------------------------------------------------
    // Card 5: Arena Regions
    // -------------------------------------------------------------------------
    theme::begin_card("##card_arena_regions", ImVec2(width, 105.0f));

    if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
    ImGui::Text("Arena Regions");
    if (theme::g_fonts.title) ImGui::PopFont();

    ImGui::Spacing();

    float col2_w = (width - 32.0f) * 0.5f;

    // Column 1
    ImGui::BeginGroup();
    ImGui::Text("R1: [%.1f, %.1f] \xc3\x97 [%.1f, %.1f]",
                app.regions.r1_x0, app.regions.r1_x1, app.regions.r1_y0, app.regions.r1_y1);
    ImGui::Text("R2: [%.1f, %.1f] \xc3\x97 [%.1f, %.1f]",
                app.regions.r2_x0, app.regions.r2_x1, app.regions.r2_y0, app.regions.r2_y1);
    ImGui::EndGroup();

    ImGui::SameLine(col2_w + 16.0f);

    // Column 2
    ImGui::BeginGroup();
    ImGui::Text("R3 (Goal): [%.1f, %.1f] \xc3\x97 [%.1f, %.1f]",
                app.regions.r3_x0, app.regions.r3_y0, app.regions.r3_y1, app.regions.r3_y1);
    ImGui::Text("R4 (Obs): [%.1f, %.1f] \xc3\x97 [%.1f, %.1f]",
                app.regions.r4_x0, app.regions.r4_y0, app.regions.r4_y1, app.regions.r4_y1);
    ImGui::EndGroup();

    theme::end_card();
}

} // namespace robot_nav::ui
