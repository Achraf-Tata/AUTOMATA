#include "app_state.hpp"
#include "ui_theme.hpp"
#include "building_view.hpp"
#include "control_panel.hpp"
#include "popup_dialogs.hpp"
#include "simulation_engine.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>
#include <cstdio>
#include <algorithm>

using namespace circular;
using namespace circular::theme;
using namespace circular::view;
using namespace circular::ui;
using namespace circular::dialogs;
using namespace circular::engine;

static GLFWwindow* g_window = nullptr;

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
            width = std::min(width, static_cast<int>(video_mode->width * 0.95));
            height = std::min(height, static_cast<int>(video_mode->height * 0.90));
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
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
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
    ImGui::DestroyContext();
    if (g_window) {
        glfwDestroyWindow(g_window);
        g_window = nullptr;
    }
    glfwTerminate();
}

int main() {
    constexpr int default_width  = 1440;
    constexpr int default_height = 920;

    if (!init_window_and_gl(default_width, default_height, "Circular Building (Assume-Guarantee Multi-Room Control) - Computation Theory Fall 2026")) {
        std::fprintf(stderr, "Failed to initialize GLFW / OpenGL\n");
        return 1;
    }

    AppState app;
    init_app_state(app);

    double last_frame_time = glfwGetTime();
    double sim_accumulator = 0.0;

    while (!glfwWindowShouldClose(g_window)) {
        glfwPollEvents();

        double current_time = glfwGetTime();
        double dt_frame = current_time - last_frame_time;
        last_frame_time = current_time;

        // Advance simulation steps if running
        if (app.sim_running && !app.sim_paused) {
            sim_accumulator += dt_frame * static_cast<double>(app.sim_speed);
            int max_steps_per_frame = 200;
            while (sim_accumulator >= 1.0 && max_steps_per_frame-- > 0) {
                step_simulation(app);
                sim_accumulator -= 1.0;
                if (app.sim_step >= app.sim_max_steps) {
                    app.sim_running = false;
                    break;
                }
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Top Menu Bar
        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Synthesize Contract")) {
                    trigger_synthesis(app);
                }
                if (ImGui::MenuItem("Save Controller Binary")) {
                    ControllerSerializer::save_binary(app.controller_filename, app.synthesis_res.local_policy);
                }
                if (ImGui::MenuItem("Load Controller Binary")) {
                    if (ControllerSerializer::load_binary(app.controller_filename, app.synthesis_res.local_policy)) {
                        app.controller_ready = true;
                    }
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Reset Simulation")) {
                    reset_simulation(app);
                }
                if (ImGui::MenuItem("Exit", "Alt+F4")) {
                    glfwSetWindowShouldClose(g_window, GLFW_TRUE);
                }
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Rooms")) {
                if (ImGui::MenuItem("3 Rooms", nullptr, app.num_rooms == 3))   { app.num_rooms = 3;   reset_simulation(app); trigger_synthesis(app); }
                if (ImGui::MenuItem("4 Rooms", nullptr, app.num_rooms == 4))   { app.num_rooms = 4;   reset_simulation(app); trigger_synthesis(app); }
                if (ImGui::MenuItem("5 Rooms", nullptr, app.num_rooms == 5))   { app.num_rooms = 5;   reset_simulation(app); trigger_synthesis(app); }
                if (ImGui::MenuItem("8 Rooms", nullptr, app.num_rooms == 8))   { app.num_rooms = 8;   reset_simulation(app); trigger_synthesis(app); }
                if (ImGui::MenuItem("16 Rooms", nullptr, app.num_rooms == 16)) { app.num_rooms = 16;  reset_simulation(app); trigger_synthesis(app); }
                if (ImGui::MenuItem("32 Rooms", nullptr, app.num_rooms == 32)) { app.num_rooms = 32;  reset_simulation(app); trigger_synthesis(app); }
                if (ImGui::MenuItem("64 Rooms", nullptr, app.num_rooms == 64)) { app.num_rooms = 64;  reset_simulation(app); trigger_synthesis(app); }
                if (ImGui::MenuItem("100 Rooms", nullptr, app.num_rooms == 100)) { app.num_rooms = 100; reset_simulation(app); trigger_synthesis(app); }
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Theory")) {
                if (ImGui::MenuItem("About Circular Building...")) {
                    open_about_dialog();
                }
                if (ImGui::MenuItem("Curse of Dimensionality...")) {
                    open_curse_dialog();
                }
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        // Main Fullscreen Host Window
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(vp->WorkPos);
        ImGui::SetNextWindowSize(vp->WorkSize);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));

        ImGuiWindowFlags host_flags = ImGuiWindowFlags_NoDecoration |
                                     ImGuiWindowFlags_NoMove |
                                     ImGuiWindowFlags_NoResize |
                                     ImGuiWindowFlags_NoBringToFrontOnFocus;

        if (ImGui::Begin("##main_host_window", nullptr, host_flags)) {
            float total_w = ImGui::GetContentRegionAvail().x;
            float total_h = ImGui::GetContentRegionAvail().y;

            float sidebar_w = 410.0f;
            float view_w    = total_w - sidebar_w - 12.0f;

            // Left Column: Control Panel
            render_control_panel(app, sidebar_w, total_h);

            // Right Column: Circular Building View & Temperature Plot
            ImGui::SameLine();
            render_building_view(app, view_w, total_h);
        }
        ImGui::End();
        ImGui::PopStyleVar();

        // Render Popups if requested
        render_popups(app);

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(g_window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.968f, 0.957f, 0.937f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(g_window);
    }

    shutdown_window_and_gl();
    return 0;
}
