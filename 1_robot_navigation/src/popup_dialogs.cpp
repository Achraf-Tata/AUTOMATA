#include "popup_dialogs.hpp"
#include "ui_theme.hpp"
#include "simulation_engine.hpp"
#include "imgui.h"

namespace robot_nav::dialogs {

using namespace std;
using namespace ImGui;
using namespace theme;
using namespace colors;
using namespace engine;

void render_popup_dialogs(AppState& app) {
    // -------------------------------------------------------------------------
    // 1. Arena Layout Modified Warning Modal
    // -------------------------------------------------------------------------
    if (app.show_arena_modified_modal) {
        OpenPopup("Arena Layout Modified##modal");
    }

    ImVec2 center = GetMainViewport()->GetCenter();
    SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    SetNextWindowSize(ImVec2(520.0f, 0.0f));

    if (BeginPopupModal("Arena Layout Modified##modal", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (g_fonts.title) PushFont(g_fonts.title);
        TextColored(banner_amber_txt, "Arena Layout Modified");
        if (g_fonts.title) PopFont();

        Spacing();
        TextWrapped(
            "You have moved or resized one or more arena regions since the controller was synthesized. "
            "Simulating using the previous policy may cause collisions with modified obstacles or failure to reach waypoints."
        );
        Spacing();
        TextColored(text_muted, "Choose how you want to proceed:");
        Spacing();
        Separator();
        Spacing();

        if (primary_button("Solve and simulate", ImVec2(180.0f, 36.0f))) {
            app.show_arena_modified_modal = false;
            app.auto_simulate_after_solve = true;
            CloseCurrentPopup();
            trigger_synthesis(app);
        }

        SameLine();
        if (outline_button("Solve only", ImVec2(130.0f, 36.0f))) {
            app.show_arena_modified_modal = false;
            app.auto_simulate_after_solve = false;
            CloseCurrentPopup();
            trigger_synthesis(app);
        }

        SameLine();
        if (outline_button("Continue anyway", ImVec2(160.0f, 36.0f))) {
            app.show_arena_modified_modal = false;
            CloseCurrentPopup();
            run_simulation(app);
        }

        Spacing();
        if (Button("Cancel", ImVec2(90.0f, 30.0f))) {
            app.show_arena_modified_modal = false;
            CloseCurrentPopup();
        }

        EndPopup();
    }

    // -------------------------------------------------------------------------
    // 2. Edge Case Alert Modal
    // -------------------------------------------------------------------------
    if (app.show_alert_modal) {
        OpenPopup("Simulation Alert##modal");
    }

    SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    SetNextWindowSize(ImVec2(560.0f, 0.0f));

    if (BeginPopupModal("Simulation Alert##modal", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (g_fonts.title) PushFont(g_fonts.title);
        TextColored(banner_red_txt, "%s", app.alert_title.c_str());
        if (g_fonts.title) PopFont();

        Spacing();
        Separator();
        Spacing();

        if (g_fonts.bold) PushFont(g_fonts.bold);
        Text("What happened:");
        if (g_fonts.bold) PopFont();
        TextWrapped("%s", app.alert_what.c_str());

        Spacing();
        if (g_fonts.bold) PushFont(g_fonts.bold);
        Text("Why it happened:");
        if (g_fonts.bold) PopFont();
        TextWrapped("%s", app.alert_why.c_str());

        Spacing();
        if (g_fonts.bold) PushFont(g_fonts.bold);
        Text("Recommendation:");
        if (g_fonts.bold) PopFont();
        TextWrapped("%s", app.alert_recommendation.c_str());

        Spacing();
        Separator();
        Spacing();

        if (primary_button("Dismiss", ImVec2(120.0f, 34.0f))) {
            app.show_alert_modal = false;
            CloseCurrentPopup();
        }

        EndPopup();
    }
}

} // namespace robot_nav::dialogs
