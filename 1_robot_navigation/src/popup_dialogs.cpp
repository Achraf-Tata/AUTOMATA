#include "popup_dialogs.hpp"
#include "ui_theme.hpp"
#include "simulation_engine.hpp"
#include "imgui.h"

namespace robot_nav::dialogs {

void render_popup_dialogs(AppState& app) {
    // -------------------------------------------------------------------------
    // 1. Arena Layout Modified Warning Modal
    // -------------------------------------------------------------------------
    if (app.show_arena_modified_modal) {
        ImGui::OpenPopup("Arena Layout Modified##modal");
    }

    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(520.0f, 0.0f));

    if (ImGui::BeginPopupModal("Arena Layout Modified##modal", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
        ImGui::TextColored(theme::colors::banner_amber_txt, "Arena Layout Modified");
        if (theme::g_fonts.title) ImGui::PopFont();

        ImGui::Spacing();
        ImGui::TextWrapped(
            "You have moved or resized one or more arena regions since the controller was synthesized. "
            "Simulating using the previous policy may cause collisions with modified obstacles or failure to reach waypoints."
        );
        ImGui::Spacing();
        ImGui::TextColored(theme::colors::text_muted, "Choose how you want to proceed:");
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        if (theme::primary_button("Solve and simulate", ImVec2(180.0f, 36.0f))) {
            app.show_arena_modified_modal = false;
            app.auto_simulate_after_solve = true;
            ImGui::CloseCurrentPopup();
            engine::trigger_synthesis(app);
        }

        ImGui::SameLine();
        if (theme::outline_button("Solve only", ImVec2(130.0f, 36.0f))) {
            app.show_arena_modified_modal = false;
            app.auto_simulate_after_solve = false;
            ImGui::CloseCurrentPopup();
            engine::trigger_synthesis(app);
        }

        ImGui::SameLine();
        if (theme::outline_button("Continue anyway", ImVec2(160.0f, 36.0f))) {
            app.show_arena_modified_modal = false;
            ImGui::CloseCurrentPopup();
            engine::run_simulation(app);
        }

        ImGui::Spacing();
        if (ImGui::Button("Cancel", ImVec2(90.0f, 30.0f))) {
            app.show_arena_modified_modal = false;
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }

    // -------------------------------------------------------------------------
    // 2. Edge Case Alert Modal
    // -------------------------------------------------------------------------
    if (app.show_alert_modal) {
        ImGui::OpenPopup("Simulation Alert##modal");
    }

    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(560.0f, 0.0f));

    if (ImGui::BeginPopupModal("Simulation Alert##modal", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
        ImGui::TextColored(theme::colors::banner_red_txt, "%s", app.alert_title.c_str());
        if (theme::g_fonts.title) ImGui::PopFont();

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        if (theme::g_fonts.bold) ImGui::PushFont(theme::g_fonts.bold);
        ImGui::Text("What happened:");
        if (theme::g_fonts.bold) ImGui::PopFont();
        ImGui::TextWrapped("%s", app.alert_what.c_str());

        ImGui::Spacing();
        if (theme::g_fonts.bold) ImGui::PushFont(theme::g_fonts.bold);
        ImGui::Text("Why it happened:");
        if (theme::g_fonts.bold) ImGui::PopFont();
        ImGui::TextWrapped("%s", app.alert_why.c_str());

        ImGui::Spacing();
        if (theme::g_fonts.bold) ImGui::PushFont(theme::g_fonts.bold);
        ImGui::Text("Recommendation:");
        if (theme::g_fonts.bold) ImGui::PopFont();
        ImGui::TextWrapped("%s", app.alert_recommendation.c_str());

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        if (theme::primary_button("Dismiss", ImVec2(120.0f, 34.0f))) {
            app.show_alert_modal = false;
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }
}

} // namespace robot_nav::dialogs
