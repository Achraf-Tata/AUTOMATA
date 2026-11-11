#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include "app_state.hpp"
#include "ui_theme.hpp"
#include "arena_view.hpp"
#include "control_panel.hpp"
#include "popup_dialogs.hpp"
#include "simulation_engine.hpp"
#include "reachability_cpu.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>
#include <cstdio>
#include <algorithm>

static GLFWwindow* g_window = nullptr;

namespace robot_nav::gui {
GLFWwindow* get_window() {
    return g_window;
}
} // namespace robot_nav::gui

static bool init_window_and_gl(int width, int height, const char* title) {
    if (!glfwInit()) return false;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    g_window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!g_window) {
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(g_window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    robot_nav::theme::load_fonts();
    robot_nav::theme::setup_theme();

    ImGui_ImplGlfw_InitForOpenGL(g_window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    return true;
}

static void shutdown_window_and_gl() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    if (g_window) glfwDestroyWindow(g_window);
    glfwTerminate();
}

int main() {
    constexpr int WINDOW_WIDTH  = 1500;
    constexpr int WINDOW_HEIGHT = 900;

    if (!init_window_and_gl(WINDOW_WIDTH, WINDOW_HEIGHT, "Robot Navigation")) {
        return 1;
    }

    robot_nav::AppState app;
    robot_nav::init_app_state(app);

    // Precompute Girard unicycle abstraction growth bounds
    app.abs_tables = grid::precompute_abstraction_tables();
    app.abs_tables_ready = true;

    // Load precomputed controller binary if available on disk
    if (solver::load_controller_binary(app.controller_filename, app.V, app.ctrl)) {
        app.controller_ready = true;
    }

    static double s_last_anim_step = 0.0;

    while (!glfwWindowShouldClose(g_window)) {
        glfwPollEvents();

        // Animated simulation stepper at user-configured speed
        double current_time = glfwGetTime();
        if (app.sim_running && !app.sim_paused) {
            if (app.instant_sim) {
                while (app.sim_running && !app.sim_paused) {
                    if (!robot_nav::engine::sim_step_once(app)) {
                        break;
                    }
                }
            } else {
                double interval = 1.0 / std::clamp(static_cast<double>(app.anim_speed), 1.0, 60.0);
                if (current_time - s_last_anim_step >= interval) {
                    robot_nav::engine::sim_step_once(app);
                    s_last_anim_step = current_time;
                }
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiIO& io = ImGui::GetIO();

        // Responsive fullscreen root viewport using io.DisplaySize
        ImGui::SetNextWindowPos({0.0f, 0.0f});
        ImGui::SetNextWindowSize(io.DisplaySize);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));

        ImGui::Begin("##root_canvas", nullptr,
                     ImGuiWindowFlags_NoTitleBar  |
                     ImGuiWindowFlags_NoResize    |
                     ImGuiWindowFlags_NoMove      |
                     ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);

        float total_w = io.DisplaySize.x - 24.0f;
        float total_h = io.DisplaySize.y - 20.0f;

        // Responsive Main Layout: Left Arena + Right Control Cards
        float body_h = total_h;

        float right_panel_w = std::clamp(total_w * 0.38f, 380.0f, 540.0f);
        float left_arena_w  = total_w - right_panel_w - 12.0f;

        // Left Column (Arena + Legend)
        ImGui::BeginGroup();
        float legend_h  = 66.0f;
        float arena_h   = body_h - legend_h - 10.0f;
        robot_nav::arena::render_arena_card(app, left_arena_w, arena_h);
        ImGui::Spacing();
        robot_nav::arena::render_legend_card(app, left_arena_w);
        ImGui::EndGroup();

        ImGui::SameLine(0, 12.0f);

        // Right Column (Scrollable so cards never clip off screen edges)
        ImGui::BeginChild("##right_column_scroll", ImVec2(right_panel_w, body_h), false,
                          ImGuiWindowFlags_None);
        robot_nav::ui::render_right_cards(app, right_panel_w - 8.0f);
        ImGui::EndChild();

        // 3. Modal Dialogs (Edge Case Alerts & Arena Modified Warning)
        robot_nav::dialogs::render_popup_dialogs(app);

        ImGui::End();
        ImGui::PopStyleVar();

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(g_window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        const auto& c = robot_nav::theme::colors::bg_canvas;
        glClearColor(c.x, c.y, c.z, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(g_window);
    }

    shutdown_window_and_gl();
    return 0;
}
