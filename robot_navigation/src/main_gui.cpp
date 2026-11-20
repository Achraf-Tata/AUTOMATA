#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include "app_state.hpp"
#include "ui_theme.hpp"
#include "arena_view.hpp"
#include "control_panel.hpp"
#include "popup_dialogs.hpp"
#include "simulation_engine.hpp"
#include "backward_reachability.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>
#include <cstdio>
#include <algorithm>

using namespace std;
using namespace ImGui;
using namespace robot_nav;
using namespace robot_nav::theme;
using namespace robot_nav::arena;
using namespace robot_nav::ui;
using namespace robot_nav::dialogs;
using namespace robot_nav::engine;
using namespace solver;
using namespace grid;
using namespace colors;

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

    GLFWmonitor* primary_monitor = glfwGetPrimaryMonitor();
    if (primary_monitor) {
        const GLFWvidmode* video_mode = glfwGetVideoMode(primary_monitor);
        if (video_mode) {
            width = min(width, static_cast<int>(video_mode->width * 0.95));
            height = min(height, static_cast<int>(video_mode->height * 0.90));
        }
    }

    g_window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!g_window) {
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(g_window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    CreateContext();
    ImGuiIO& io = GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    load_fonts();
    setup_theme();

    ImGui_ImplGlfw_InitForOpenGL(g_window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    return true;
}

static void shutdown_window_and_gl() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    DestroyContext();
    if (g_window) glfwDestroyWindow(g_window);
    glfwTerminate();
}

int main() {
    constexpr int WINDOW_WIDTH  = 1450;
    constexpr int WINDOW_HEIGHT = 820;

    if (!init_window_and_gl(WINDOW_WIDTH, WINDOW_HEIGHT, "Robot Navigation")) {
        return 1;
    }

    AppState app;
    init_app_state(app);

    // Precompute Girard unicycle abstraction growth bounds
    app.abs_tables = precompute_abstraction_tables();
    app.abs_tables_ready = true;

    // Load precomputed controller binary if available on disk across search paths
    const char* fallback_controller_paths[] = {
        app.controller_filename,
        "controller.bin",
        "../controller.bin",
        "../../controller.bin",
        "robot_navigation/controller.bin"
    };

    for (const char* candidate_path : fallback_controller_paths) {
        if (load_controller_binary(candidate_path, app.V, app.ctrl)) {
            app.controller_ready = true;
            snprintf(app.controller_filename, sizeof(app.controller_filename), "%s", candidate_path);
            break;
        }
    }

    static double s_last_anim_step = 0.0;

    while (!glfwWindowShouldClose(g_window)) {
        glfwPollEvents();

        // Animated simulation stepper at user-configured speed
        double current_time = glfwGetTime();
        if (app.sim_running && !app.sim_paused) {
            if (app.instant_sim) {
                while (app.sim_running && !app.sim_paused) {
                    if (!sim_step_once(app)) {
                        break;
                    }
                }
            } else {
                double interval = 1.0 / clamp(static_cast<double>(app.anim_speed), 1.0, 60.0);
                if (current_time - s_last_anim_step >= interval) {
                    sim_step_once(app);
                    s_last_anim_step = current_time;
                }
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        NewFrame();

        ImGuiIO& io = GetIO();

        // Responsive fullscreen root viewport using io.DisplaySize
        SetNextWindowPos({0.0f, 0.0f});
        SetNextWindowSize(io.DisplaySize);
        PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));

        Begin("##root_canvas", nullptr,
              ImGuiWindowFlags_NoTitleBar  |
              ImGuiWindowFlags_NoResize    |
              ImGuiWindowFlags_NoMove      |
              ImGuiWindowFlags_NoScrollbar |
              ImGuiWindowFlags_NoBringToFrontOnFocus);

        float total_w = io.DisplaySize.x - 24.0f;
        float total_h = io.DisplaySize.y - 20.0f;

        // Responsive Main Layout: Left Arena + Right Control Cards
        float body_h = total_h;

        float right_panel_w = clamp(total_w * 0.38f, 380.0f, 540.0f);
        float left_arena_w  = total_w - right_panel_w - 12.0f;

        // Left Column (Arena + Legend)
        BeginGroup();
        float legend_h  = 66.0f;
        float arena_h   = body_h - legend_h - 10.0f;
        render_arena_card(app, left_arena_w, arena_h);
        Spacing();
        render_legend_card(app, left_arena_w);
        EndGroup();

        SameLine(0, 12.0f);

        // Right Column (Scrollable so cards never clip off screen edges)
        BeginChild("##right_column_scroll", ImVec2(right_panel_w, body_h), false,
                   ImGuiWindowFlags_None);
        render_right_cards(app, right_panel_w - 8.0f);
        EndChild();

        // 3. Modal Dialogs (Edge Case Alerts & Arena Modified Warning)
        render_popup_dialogs(app);

        End();
        PopStyleVar();

        Render();
        int display_w, display_h;
        glfwGetFramebufferSize(g_window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        const auto& c = bg_canvas;
        glClearColor(c.x, c.y, c.z, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(GetDrawData());
        glfwSwapBuffers(g_window);
    }

    shutdown_window_and_gl();
    return 0;
}
