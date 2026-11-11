#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include "gui_arena.hpp"
#include "reachability_cpu.hpp"
#include "product_automaton.hpp"
#include "grid_quantizer.hpp"
#include "unicycle_model.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include <thread>
#include <chrono>
#include <random>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <cstdio>
#include <filesystem>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static GLFWwindow* g_window = nullptr;
static ImFont* g_font_regular = nullptr;
static ImFont* g_font_bold    = nullptr;
static ImFont* g_font_large   = nullptr;

namespace gui {

bool init_graphics(int window_w, int window_h, const char* title) {
    if (!glfwInit()) return false;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4); // 4x MSAA

    g_window = glfwCreateWindow(window_w, window_h, title, nullptr, nullptr);
    if (!g_window) {
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(g_window);
    glfwSwapInterval(1); // vsync

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Load clean modern fonts
    const char* font_path_reg  = "C:\\Windows\\Fonts\\segoeui.ttf";
    const char* font_path_bold = "C:\\Windows\\Fonts\\segoeuib.ttf";
    if (std::filesystem::exists(font_path_reg)) {
        g_font_regular = io.Fonts->AddFontFromFileTTF(font_path_reg, 17.0f);
        if (std::filesystem::exists(font_path_bold)) {
            g_font_bold  = io.Fonts->AddFontFromFileTTF(font_path_bold, 17.0f);
            g_font_large = io.Fonts->AddFontFromFileTTF(font_path_bold, 21.0f);
        } else {
            g_font_bold  = g_font_regular;
            g_font_large = g_font_regular;
        }
    } else {
        io.Fonts->AddFontDefault();
    }

    // Modern professional dark palette
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding    = 8.0f;
    style.ChildRounding     = 6.0f;
    style.FrameRounding     = 5.0f;
    style.PopupRounding     = 6.0f;
    style.GrabRounding      = 5.0f;
    style.TabRounding       = 5.0f;
    style.WindowBorderSize  = 1.0f;
    style.ChildBorderSize   = 1.0f;
    style.FrameBorderSize   = 1.0f;
    style.WindowPadding     = {12.0f, 12.0f};
    style.FramePadding      = {10.0f, 6.0f};
    style.ItemSpacing       = {10.0f, 8.0f};
    style.ItemInnerSpacing  = {6.0f, 6.0f};

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg]             = ImVec4(0.07f, 0.08f, 0.10f, 1.00f);
    colors[ImGuiCol_ChildBg]              = ImVec4(0.09f, 0.11f, 0.14f, 1.00f);
    colors[ImGuiCol_PopupBg]              = ImVec4(0.10f, 0.12f, 0.15f, 0.98f);
    colors[ImGuiCol_Border]               = ImVec4(0.18f, 0.22f, 0.28f, 1.00f);
    colors[ImGuiCol_BorderShadow]         = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_FrameBg]              = ImVec4(0.13f, 0.16f, 0.20f, 1.00f);
    colors[ImGuiCol_FrameBgHovered]       = ImVec4(0.18f, 0.22f, 0.28f, 1.00f);
    colors[ImGuiCol_FrameBgActive]        = ImVec4(0.22f, 0.27f, 0.35f, 1.00f);
    colors[ImGuiCol_TitleBg]              = ImVec4(0.07f, 0.08f, 0.10f, 1.00f);
    colors[ImGuiCol_TitleBgActive]        = ImVec4(0.09f, 0.11f, 0.14f, 1.00f);
    colors[ImGuiCol_MenuBarBg]            = ImVec4(0.09f, 0.11f, 0.14f, 1.00f);
    colors[ImGuiCol_ScrollbarBg]          = ImVec4(0.07f, 0.08f, 0.10f, 0.50f);
    colors[ImGuiCol_ScrollbarGrab]        = ImVec4(0.20f, 0.25f, 0.32f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.28f, 0.35f, 0.44f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabActive]  = ImVec4(0.35f, 0.44f, 0.55f, 1.00f);
    colors[ImGuiCol_CheckMark]            = ImVec4(0.22f, 0.65f, 1.00f, 1.00f);
    colors[ImGuiCol_SliderGrab]           = ImVec4(0.22f, 0.65f, 1.00f, 1.00f);
    colors[ImGuiCol_SliderGrabActive]     = ImVec4(0.38f, 0.75f, 1.00f, 1.00f);
    colors[ImGuiCol_Button]               = ImVec4(0.14f, 0.35f, 0.68f, 1.00f);
    colors[ImGuiCol_ButtonHovered]        = ImVec4(0.18f, 0.45f, 0.85f, 1.00f);
    colors[ImGuiCol_ButtonActive]         = ImVec4(0.11f, 0.28f, 0.55f, 1.00f);
    colors[ImGuiCol_Header]               = ImVec4(0.13f, 0.17f, 0.22f, 1.00f);
    colors[ImGuiCol_HeaderHovered]        = ImVec4(0.18f, 0.23f, 0.30f, 1.00f);
    colors[ImGuiCol_HeaderActive]         = ImVec4(0.22f, 0.28f, 0.38f, 1.00f);
    colors[ImGuiCol_Separator]            = ImVec4(0.18f, 0.22f, 0.28f, 1.00f);
    colors[ImGuiCol_Text]                 = ImVec4(0.92f, 0.94f, 0.97f, 1.00f);
    colors[ImGuiCol_TextDisabled]         = ImVec4(0.48f, 0.54f, 0.62f, 1.00f);

    ImGui_ImplGlfw_InitForOpenGL(g_window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    return true;
}

void shutdown_graphics() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    if (g_window) glfwDestroyWindow(g_window);
    glfwTerminate();
}

static ImVec2 world_to_screen(float wx, float wy, ImVec2 origin, float scale, float canvas_size) {
    return { origin.x + wx * scale,
             origin.y + canvas_size - wy * scale };
}

static ImVec2 screen_to_world(float sx, float sy, ImVec2 origin, float scale, float canvas_size) {
    return { (sx - origin.x) / scale,
             (canvas_size - (sy - origin.y)) / scale };
}

void trigger_synthesis(AppState& app) {
    if (app.solving || !app.abs_tables_ready) return;
    app.solving = true;
    app.controller_ready = false;
    app.solve_progress = 0.0f;
    app.solve_changes = 0;
    app.solve_iteration = 0;

    std::thread([&app]() {
        auto t0 = std::chrono::high_resolution_clock::now();
        app.V = automaton::init_value_function();
        app.ctrl = automaton::init_controller();

        app.solve_iteration = solver::solve_reachability_openmp(
            app.V, app.ctrl, app.regions, app.abs_tables,
            [&app](int it, float ch) {
                app.solve_iteration = it;
                app.solve_changes = static_cast<int>(ch);
                app.solve_progress = std::min(1.0f, static_cast<float>(it) / 40.0f);
            }
        );

        auto t1 = std::chrono::high_resolution_clock::now();
        app.solve_duration = std::chrono::duration<float>(t1 - t0).count();
        app.solving = false;
        app.controller_ready = true;
        app.solve_progress = 1.0f;

        solver::save_controller_binary(app.controller_path_buf, app.V, app.ctrl);
    }).detach();
}

void render_arena(AppState& app, float canvas_size) {
    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    float scale = canvas_size / 10.0f;
    ImVec2 origin = canvas_pos;

    ImDrawList* dl = ImGui::GetWindowDrawList();

    // Subtle canvas background
    dl->AddRectFilled(canvas_pos,
                      {canvas_pos.x + canvas_size, canvas_pos.y + canvas_size},
                      IM_COL32(14, 17, 22, 255));

    // Coordinate grid lines & ticks
    if (app.show_grid) {
        ImU32 major_grid = IM_COL32(36, 42, 52, 220);
        ImU32 minor_grid = IM_COL32(24, 28, 36, 180);
        for (int i = 0; i <= 20; ++i) {
            float frac = i * 0.5f;
            float px = origin.x + frac * scale;
            float py = origin.y + frac * scale;
            bool major = (i % 2 == 0);
            dl->AddLine({px, origin.y}, {px, origin.y + canvas_size}, major ? major_grid : minor_grid, 1.0f);
            dl->AddLine({origin.x, py}, {origin.x + canvas_size, py}, major ? major_grid : minor_grid, 1.0f);

            if (major && i < 20) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "%d", i / 2);
                dl->AddText({origin.x + (i / 2) * scale + 4.0f, origin.y + canvas_size - 18.0f},
                            IM_COL32(80, 92, 108, 200), buf);
            }
        }
    }

    auto draw_region = [&](double x0, double y0, double x1, double y1,
                           ImU32 fill, ImU32 border, const char* title, const char* subtitle) {
        ImVec2 a = world_to_screen(static_cast<float>(x0), static_cast<float>(y0), origin, scale, canvas_size);
        ImVec2 b = world_to_screen(static_cast<float>(x1), static_cast<float>(y1), origin, scale, canvas_size);
        ImVec2 tl = {std::min(a.x, b.x), std::min(a.y, b.y)};
        ImVec2 br = {std::max(a.x, b.x), std::max(a.y, b.y)};

        dl->AddRectFilled(tl, br, fill);
        dl->AddRect(tl, br, border, 4.0f, 0, 2.0f);

        ImVec2 ctr = {(tl.x + br.x) * 0.5f, (tl.y + br.y) * 0.5f};
        if (g_font_bold) ImGui::PushFont(g_font_bold);
        ImVec2 ts_title = ImGui::CalcTextSize(title);
        dl->AddText({ctr.x - ts_title.x * 0.5f, ctr.y - ts_title.y - 2.0f}, IM_COL32(255, 255, 255, 240), title);
        if (g_font_bold) ImGui::PopFont();

        ImVec2 ts_sub = ImGui::CalcTextSize(subtitle);
        dl->AddText({ctr.x - ts_sub.x * 0.5f, ctr.y + 2.0f}, IM_COL32(210, 220, 230, 210), subtitle);
    };

    // R4: Obstacle (Red)
    draw_region(app.regions.r4_x0, app.regions.r4_y0, app.regions.r4_x1, app.regions.r4_y1,
                IM_COL32(218, 54, 51, 100), IM_COL32(248, 81, 73, 255), "R4", "Obstacle");

    // R1: Waymark 1 (Amber)
    draw_region(app.regions.r1_x0, app.regions.r1_y0, app.regions.r1_x1, app.regions.r1_y1,
                IM_COL32(187, 128, 9, 110), IM_COL32(210, 153, 34, 255), "R1", "Waymark 1");

    // R2: Waymark 2 (Blue)
    draw_region(app.regions.r2_x0, app.regions.r2_y0, app.regions.r2_x1, app.regions.r2_y1,
                IM_COL32(31, 111, 235, 110), IM_COL32(88, 166, 255, 255), "R2", "Waymark 2");

    // R3: Goal (Green)
    draw_region(app.regions.r3_x0, app.regions.r3_y0, app.regions.r3_x1, app.regions.r3_y1,
                IM_COL32(35, 134, 54, 110), IM_COL32(63, 185, 80, 255), "R3", "Goal");

    // Monte Carlo trajectories bundle
    if (app.show_all_mc && !app.mc_trajectories.empty()) {
        ImU32 mc_col = IM_COL32(88, 166, 255, 60);
        for (const auto& traj : app.mc_trajectories) {
            for (size_t i = 1; i < traj.size(); ++i) {
                ImVec2 pa = world_to_screen(static_cast<float>(traj[i-1].x), static_cast<float>(traj[i-1].y), origin, scale, canvas_size);
                ImVec2 pb = world_to_screen(static_cast<float>(traj[i].x),   static_cast<float>(traj[i].y),   origin, scale, canvas_size);
                dl->AddLine(pa, pb, mc_col, 1.2f);
            }
        }
    }

    // Active trajectory (bright white)
    if (app.sim_trajectory.size() > 1) {
        for (size_t i = 1; i < app.sim_trajectory.size(); ++i) {
            ImVec2 pa = world_to_screen(static_cast<float>(app.sim_trajectory[i-1].x), static_cast<float>(app.sim_trajectory[i-1].y), origin, scale, canvas_size);
            ImVec2 pb = world_to_screen(static_cast<float>(app.sim_trajectory[i].x),   static_cast<float>(app.sim_trajectory[i].y),   origin, scale, canvas_size);
            dl->AddLine(pa, pb, IM_COL32(255, 255, 255, 240), 2.5f);
        }
    }

    // Robot representation: chassis disk + direction arrow
    {
        unicycle::State& rp = app.sim_running ? app.sim_robot_pos : app.robot_start;
        ImVec2 rpos = world_to_screen(static_cast<float>(rp.x), static_cast<float>(rp.y), origin, scale, canvas_size);
        float r_radius = scale * 0.22f;

        // Chassis circle
        dl->AddCircleFilled(rpos, r_radius, IM_COL32(25, 30, 40, 240));
        dl->AddCircle(rpos, r_radius, IM_COL32(255, 215, 0, 255), 24, 2.0f);

        // Direction pointer
        float len = scale * 0.45f;
        float th  = -static_cast<float>(rp.theta);
        ImVec2 tip   = {rpos.x + len * std::cos(th), rpos.y + len * std::sin(th)};
        ImVec2 left  = {rpos.x + (len * 0.40f) * std::cos(th + 2.5f), rpos.y + (len * 0.40f) * std::sin(th + 2.5f)};
        ImVec2 right = {rpos.x + (len * 0.40f) * std::cos(th - 2.5f), rpos.y + (len * 0.40f) * std::sin(th - 2.5f)};

        ImU32 pointer_col = IM_COL32(255, 220, 60, 255);
        dl->AddLine(rpos, tip, pointer_col, 3.0f);
        dl->AddLine(tip, left, pointer_col, 2.5f);
        dl->AddLine(tip, right, pointer_col, 2.5f);
    }

    // Mouse interaction
    ImGui::InvisibleButton("##arena_btn", {canvas_size, canvas_size});
    bool hovered   = ImGui::IsItemHovered();
    bool lmb_down  = ImGui::IsMouseDown(ImGuiMouseButton_Left);
    bool lmb_click = ImGui::IsMouseClicked(ImGuiMouseButton_Left);
    ImVec2 mpos    = ImGui::GetMousePos();
    ImVec2 mworld  = screen_to_world(mpos.x, mpos.y, origin, scale, canvas_size);
    double mx      = static_cast<double>(mworld.x);
    double my      = static_cast<double>(mworld.y);

    auto corner_check = [&](double x0, double y0, double x1, double y1) -> int {
        double d = 0.35;
        if (std::abs(mx - x0) < d && std::abs(my - y0) < d) return 1; // BL
        if (std::abs(mx - x1) < d && std::abs(my - y0) < d) return 2; // BR
        if (std::abs(mx - x0) < d && std::abs(my - y1) < d) return 3; // TL
        if (std::abs(mx - x1) < d && std::abs(my - y1) < d) return 4; // TR
        return 0;
    };

    auto inside_region = [&](double x0, double y0, double x1, double y1) -> bool {
        return mx >= x0 && mx <= x1 && my >= y0 && my <= y1;
    };

    static ImVec2 s_drag_start_mouse = {0.0f, 0.0f};
    static double s_drag_start_x0 = 0.0, s_drag_start_y0 = 0.0;
    static double s_drag_start_x1 = 0.0, s_drag_start_y1 = 0.0;

    struct RegionRef { int id; double *x0, *y0, *x1, *y1; };
    RegionRef reg_refs[4] = {
        {1, &app.regions.r1_x0, &app.regions.r1_y0, &app.regions.r1_x1, &app.regions.r1_y1},
        {2, &app.regions.r2_x0, &app.regions.r2_y0, &app.regions.r2_x1, &app.regions.r2_y1},
        {3, &app.regions.r3_x0, &app.regions.r3_y0, &app.regions.r3_x1, &app.regions.r3_y1},
        {4, &app.regions.r4_x0, &app.regions.r4_y0, &app.regions.r4_x1, &app.regions.r4_y1},
    };

    if (hovered && lmb_click && !app.sim_running) {
        app.resize_region = 0;
        app.resize_corner = 0;
        app.dragging_region = 0;
        app.dragging_robot = false;

        for (auto& r : reg_refs) {
            int c = corner_check(*r.x0, *r.y0, *r.x1, *r.y1);
            if (c != 0) {
                app.resize_region = r.id;
                app.resize_corner = c;
                s_drag_start_mouse = mworld;
                s_drag_start_x0 = *r.x0; s_drag_start_y0 = *r.y0;
                s_drag_start_x1 = *r.x1; s_drag_start_y1 = *r.y1;
                break;
            }
        }

        if (app.resize_region == 0) {
            for (auto& r : reg_refs) {
                if (inside_region(*r.x0, *r.y0, *r.x1, *r.y1)) {
                    app.dragging_region = r.id;
                    s_drag_start_mouse = mworld;
                    s_drag_start_x0 = *r.x0; s_drag_start_y0 = *r.y0;
                    s_drag_start_x1 = *r.x1; s_drag_start_y1 = *r.y1;
                    break;
                }
            }
        }

        if (app.resize_region == 0 && app.dragging_region == 0) {
            double dx = mx - app.robot_start.x;
            double dy = my - app.robot_start.y;
            if (dx * dx + dy * dy < 0.45 * 0.45) {
                app.dragging_robot = true;
                s_drag_start_mouse = mworld;
            }
        }
    }

    if (lmb_down) {
        double ddx = static_cast<double>(mworld.x - s_drag_start_mouse.x);
        double ddy = static_cast<double>(mworld.y - s_drag_start_mouse.y);

        auto apply_op = [&](double* x0, double* y0, double* x1, double* y1) {
            if (app.dragging_region != 0) {
                double w = s_drag_start_x1 - s_drag_start_x0;
                double h = s_drag_start_y1 - s_drag_start_y0;
                *x0 = std::clamp(s_drag_start_x0 + ddx, 0.0, 10.0 - w);
                *y0 = std::clamp(s_drag_start_y0 + ddy, 0.0, 10.0 - h);
                *x1 = *x0 + w;
                *y1 = *y0 + h;
            } else if (app.resize_region != 0) {
                switch (app.resize_corner) {
                    case 1:
                        *x0 = std::clamp(s_drag_start_x0 + ddx, 0.0, *x1 - 0.2);
                        *y0 = std::clamp(s_drag_start_y0 + ddy, 0.0, *y1 - 0.2);
                        break;
                    case 2:
                        *x1 = std::clamp(s_drag_start_x1 + ddx, *x0 + 0.2, 10.0);
                        *y0 = std::clamp(s_drag_start_y0 + ddy, 0.0, *y1 - 0.2);
                        break;
                    case 3:
                        *x0 = std::clamp(s_drag_start_x0 + ddx, 0.0, *x1 - 0.2);
                        *y1 = std::clamp(s_drag_start_y1 + ddy, *y0 + 0.2, 10.0);
                        break;
                    case 4:
                        *x1 = std::clamp(s_drag_start_x1 + ddx, *x0 + 0.2, 10.0);
                        *y1 = std::clamp(s_drag_start_y1 + ddy, *y0 + 0.2, 10.0);
                        break;
                }
            }
        };

        int active_id = (app.dragging_region != 0) ? app.dragging_region : app.resize_region;
        if (active_id >= 1 && active_id <= 4) {
            apply_op(reg_refs[active_id - 1].x0, reg_refs[active_id - 1].y0,
                     reg_refs[active_id - 1].x1, reg_refs[active_id - 1].y1);
        }
        if (app.dragging_robot) {
            app.robot_start.x = std::clamp(mx, 0.0, 10.0);
            app.robot_start.y = std::clamp(my, 0.0, 10.0);
        }
    } else {
        app.dragging_region = 0;
        app.dragging_robot  = false;
        app.resize_region   = 0;
        app.resize_corner   = 0;
    }

    // Resize handles
    if (!app.sim_running) {
        auto draw_handles = [&](double x0, double y0, double x1, double y1, ImU32 col) {
            double pts[4][2] = {{x0, y0}, {x1, y0}, {x0, y1}, {x1, y1}};
            for (auto& p : pts) {
                ImVec2 sp = world_to_screen(static_cast<float>(p[0]), static_cast<float>(p[1]), origin, scale, canvas_size);
                dl->AddCircleFilled(sp, 5.0f, col);
                dl->AddCircle(sp, 7.0f, IM_COL32(255, 255, 255, 200), 12, 1.5f);
            }
        };
        draw_handles(app.regions.r1_x0, app.regions.r1_y0, app.regions.r1_x1, app.regions.r1_y1, IM_COL32(210, 153, 34, 255));
        draw_handles(app.regions.r2_x0, app.regions.r2_y0, app.regions.r2_x1, app.regions.r2_y1, IM_COL32(88, 166, 255, 255));
        draw_handles(app.regions.r3_x0, app.regions.r3_y0, app.regions.r3_x1, app.regions.r3_y1, IM_COL32(63, 185, 80, 255));
        draw_handles(app.regions.r4_x0, app.regions.r4_y0, app.regions.r4_x1, app.regions.r4_y1, IM_COL32(248, 81, 73, 255));
    }

    if (hovered) {
        int lab = grid::get_region_label(mx, my, app.regions);
        const char* lab_names[] = {"Free Space (1)", "Waymark R1 (2)", "Waymark R2 (3)", "Goal R3 (4)", "Obstacle R4 (5)"};
        const char* ln = (lab >= 1 && lab <= 5) ? lab_names[lab - 1] : "Unknown";
        ImGui::SetTooltip("Position: (%.2f, %.2f) m\nRegion: %s", mx, my, ln);
    }

    dl->AddRect(canvas_pos, {canvas_pos.x + canvas_size, canvas_pos.y + canvas_size},
                IM_COL32(56, 68, 85, 255), 4.0f, 0, 1.5f);
}

void render_control_panel(AppState& app) {
    if (g_font_large) ImGui::PushFont(g_font_large);
    ImGui::Text("Mobile Robot Navigation");
    if (g_font_large) ImGui::PopFont();
    ImGui::TextColored({0.55f, 0.62f, 0.72f, 1.0f}, "Symbolic Control for Non-linear Systems (Girard et al.)");
    ImGui::Spacing();
    ImGui::Separator();

    // -------------------------------------------------------------------------
    // 1. Controller Synthesis
    // -------------------------------------------------------------------------
    if (g_font_bold) ImGui::PushFont(g_font_bold);
    ImGui::Text("Controller Synthesis");
    if (g_font_bold) ImGui::PopFont();

    if (app.solving) {
        ImGui::ProgressBar(app.solve_progress, {-1.0f, 26.0f});
        ImGui::Text("Synthesizing... Iteration %d (changed: %d)", app.solve_iteration, app.solve_changes);
    } else {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.40f, 0.78f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.18f, 0.48f, 0.90f, 1.0f));
        if (ImGui::Button("Run Reachability Solver", {-1.0f, 32.0f})) {
            trigger_synthesis(app);
        }
        ImGui::PopStyleColor(2);

        if (app.controller_ready) {
            ImGui::TextColored({0.25f, 0.85f, 0.45f, 1.0f},
                               "Controller ready (%d iterations, %.3f s)",
                               app.solve_iteration, app.solve_duration);
        } else {
            ImGui::TextColored({0.95f, 0.40f, 0.40f, 1.0f}, "No controller available. Click above to solve.");
        }
    }

    ImGui::Spacing();
    ImGui::InputText("##ctrl_path", app.controller_path_buf, sizeof(app.controller_path_buf));
    ImGui::SameLine();
    if (ImGui::Button("Save") && app.controller_ready) {
        solver::save_controller_binary(app.controller_path_buf, app.V, app.ctrl);
    }
    ImGui::SameLine();
    if (ImGui::Button("Load")) {
        if (solver::load_controller_binary(app.controller_path_buf, app.V, app.ctrl)) {
            app.controller_ready = true;
        }
    }

    ImGui::Spacing();
    ImGui::Separator();

    // -------------------------------------------------------------------------
    // 2. Closed-Loop Simulation
    // -------------------------------------------------------------------------
    if (g_font_bold) ImGui::PushFont(g_font_bold);
    ImGui::Text("Simulation");
    if (g_font_bold) ImGui::PopFont();

    ImGui::Text("Initial Pose: (%.2f, %.2f) m", app.robot_start.x, app.robot_start.y);

    float th_deg = static_cast<float>(app.robot_start.theta * 180.0 / M_PI);
    if (ImGui::SliderFloat("Heading Angle", &th_deg, -180.0f, 180.0f, "%.1f deg")) {
        app.robot_start.theta = static_cast<double>(th_deg) * M_PI / 180.0;
    }

    ImGui::SliderInt("Horizon Limit", &app.sim_max_steps, 20, 300);
    ImGui::Checkbox("Disturbance w in [-0.05, 0.05]", &app.sim_with_noise);

    if (!app.sim_running) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.55f, 0.30f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.20f, 0.65f, 0.38f, 1.0f));
        if (ImGui::Button("Start Simulation", {-1.0f, 32.0f})) {
            if (!app.controller_ready) {
                trigger_synthesis(app);
            }
            app.sim_running = true;
            app.sim_paused  = false;
            app.sim_step    = 0;
            app.sim_dfa_state = 1;
            app.sim_robot_pos = app.robot_start;
            app.sim_trajectory.clear();
            app.sim_trajectory.push_back(app.robot_start);
        }
        ImGui::PopStyleColor(2);
    } else {
        if (ImGui::Button(app.sim_paused ? "Resume" : "Pause", {120.0f, 30.0f})) {
            app.sim_paused = !app.sim_paused;
        }
        ImGui::SameLine();
        if (ImGui::Button("Stop", {120.0f, 30.0f})) {
            app.sim_running = false;
            app.sim_paused  = false;
        }
    }

    // Telemetry Box
    ImGui::Spacing();
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.11f, 0.14f, 0.18f, 1.0f));
    ImGui::BeginChild("##telemetry", {0.0f, 100.0f}, true);

    const char* dfa_labels[] = {
        "q1: Searching (R1 or R2)",
        "q2: R1 Visited (Seeking R3, avoiding R2/R4)",
        "q3: R2 Visited (Seeking R3, avoiding R1/R4)",
        "q4: Goal Reached (Success)",
        "q5: Obstacle Collision (Violation)"
    };
    int q_disp = std::clamp(app.sim_dfa_state, 1, 5);
    ImVec4 q_col = (q_disp == 4) ? ImVec4(0.25f, 0.90f, 0.45f, 1.0f) :
                   (q_disp == 5) ? ImVec4(0.95f, 0.35f, 0.35f, 1.0f) :
                                   ImVec4(0.95f, 0.75f, 0.25f, 1.0f);

    ImGui::Text("Step: %d / %d", app.sim_step, app.sim_max_steps);
    ImGui::TextColored(q_col, "Status: %s", dfa_labels[q_disp - 1]);
    ImGui::Text("State: x = %.2f m, y = %.2f m, theta = %.1f deg",
                app.sim_robot_pos.x, app.sim_robot_pos.y,
                app.sim_robot_pos.theta * 180.0 / M_PI);
    ImGui::Text("Input: v = %.3f m/s, w = %.2f rad/s", app.sim_last_u.v, app.sim_last_u.w);

    ImGui::EndChild();
    ImGui::PopStyleColor();

    ImGui::Spacing();
    ImGui::Separator();

    // -------------------------------------------------------------------------
    // 3. Monte Carlo Analysis
    // -------------------------------------------------------------------------
    if (g_font_bold) ImGui::PushFont(g_font_bold);
    ImGui::Text("Monte Carlo Verification");
    if (g_font_bold) ImGui::PopFont();

    ImGui::SliderInt("Sample Runs", &app.mc_runs, 10, 200);

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.65f, 0.45f, 0.12f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.75f, 0.52f, 0.16f, 1.0f));
    if (ImGui::Button("Run Monte Carlo Analysis", {-1.0f, 32.0f}) && !app.mc_running) {
        if (!app.controller_ready) {
            trigger_synthesis(app);
        }
        app.mc_running = true;
        std::thread([&app]() {
            run_monte_carlo(app);
            app.mc_running = false;
        }).detach();
    }
    ImGui::PopStyleColor(2);

    if (app.mc_running) {
        ImGui::TextColored({0.95f, 0.75f, 0.25f, 1.0f}, "Evaluating runs...");
    } else if (app.mc_stats.total_runs > 0) {
        auto& s = app.mc_stats;
        ImGui::TextColored({0.25f, 0.90f, 0.45f, 1.0f},
                           "Success: %d / %d (%.1f%%)",
                           s.successful, s.total_runs, s.success_rate * 100.0f);
        ImGui::Text("Failed: %d | Timeout: %d", s.failed, s.timeout);
    }
    ImGui::Checkbox("Display trajectory bundle", &app.show_all_mc);

    ImGui::Spacing();
    ImGui::Separator();

    // -------------------------------------------------------------------------
    // 4. Region Coordinates Readout
    // -------------------------------------------------------------------------
    if (g_font_bold) ImGui::PushFont(g_font_bold);
    ImGui::Text("Arena Regions");
    if (g_font_bold) ImGui::PopFont();

    ImGui::Columns(2, nullptr, false);
    auto reg_line = [](const char* name, double x0, double y0, double x1, double y1) {
        ImGui::Text("%s: [%.1f, %.1f] x [%.1f, %.1f]", name, x0, x1, y0, y1);
    };
    reg_line("R1", app.regions.r1_x0, app.regions.r1_y0, app.regions.r1_x1, app.regions.r1_y1);
    reg_line("R2", app.regions.r2_x0, app.regions.r2_y0, app.regions.r2_x1, app.regions.r2_y1);
    ImGui::NextColumn();
    reg_line("R3 (Goal)", app.regions.r3_x0, app.regions.r3_y0, app.regions.r3_x1, app.regions.r3_y1);
    reg_line("R4 (Obs)",  app.regions.r4_x0, app.regions.r4_y0, app.regions.r4_x1, app.regions.r4_y1);
    ImGui::Columns(1);

    ImGui::Spacing();
    ImGui::Checkbox("Display Grid Lines", &app.show_grid);
    ImGui::TextColored({0.48f, 0.54f, 0.62f, 1.0f}, "Framerate: %.1f FPS", ImGui::GetIO().Framerate);
}

bool sim_step_once(AppState& app) {
    if (!app.controller_ready || app.sim_step >= app.sim_max_steps) {
        app.sim_running = false;
        return false;
    }

    if (app.sim_robot_pos.x < 0.0 || app.sim_robot_pos.x > 10.0 ||
        app.sim_robot_pos.y < 0.0 || app.sim_robot_pos.y > 10.0) {
        app.sim_dfa_state = 5;
        app.sim_running = false;
        return false;
    }

    grid::GridCoord gc = grid::quantize(app.sim_robot_pos);
    int lab = grid::get_region_label(app.sim_robot_pos.x, app.sim_robot_pos.y, app.regions);
    int next_psi = automaton::dfa_transition(app.sim_dfa_state, lab);
    app.sim_dfa_state = next_psi;

    if (next_psi == 4 || next_psi == 5) {
        app.sim_running = false;
        return false;
    }

    int pidx = automaton::product_index(next_psi, grid::flat_index(gc));
    uint8_t sigma = app.ctrl[pidx];
    if (sigma == 0 || sigma > 15) {
        app.sim_running = false;
        return false;
    }

    const unicycle::Control& u = unicycle::ALL_CONTROLS[sigma - 1];
    app.sim_last_u = u;

    std::array<double, 3> noise = {0.0, 0.0, 0.0};
    if (app.sim_with_noise) {
        static std::mt19937 rng(std::random_device{}());
        std::uniform_real_distribution<double> dist(-0.05, 0.05);
        noise = {dist(rng), dist(rng), dist(rng)};
    }

    app.sim_robot_pos = unicycle::step(app.sim_robot_pos, u, noise, 1.0);
    app.sim_robot_pos.x = std::clamp(app.sim_robot_pos.x, 0.0, 10.0);
    app.sim_robot_pos.y = std::clamp(app.sim_robot_pos.y, 0.0, 10.0);

    app.sim_trajectory.push_back(app.sim_robot_pos);
    ++app.sim_step;
    return true;
}

SimulationResult run_closed_loop(
    const unicycle::State&         x0,
    int                            max_steps,
    bool                           with_noise,
    const std::vector<uint8_t>&    ctrl,
    const grid::RegionConfig&      regions)
{
    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> dist(-0.05, 0.05);

    SimulationResult res;
    res.trajectory.push_back(x0);
    unicycle::State rp = x0;
    int psi = 1;

    for (int s = 0; s < max_steps; ++s) {
        if (rp.x < 0.0 || rp.x > 10.0 || rp.y < 0.0 || rp.y > 10.0) {
            res.hit_obstacle = true;
            res.steps_taken = s;
            return res;
        }

        grid::GridCoord gc = grid::quantize(rp);
        int lab = grid::get_region_label(rp.x, rp.y, regions);
        int next_psi = automaton::dfa_transition(psi, lab);
        psi = next_psi;

        if (psi == 4) {
            res.reached_goal = true;
            res.steps_taken  = s;
            return res;
        }
        if (psi == 5) {
            res.hit_obstacle = true;
            res.steps_taken  = s;
            return res;
        }

        int pidx = automaton::product_index(psi, grid::flat_index(gc));
        uint8_t sigma = ctrl[pidx];
        if (sigma == 0 || sigma > 15) {
            res.steps_taken = s;
            return res;
        }

        const unicycle::Control& u = unicycle::ALL_CONTROLS[sigma - 1];
        std::array<double, 3> noise = {0.0, 0.0, 0.0};
        if (with_noise) {
            noise = {dist(rng), dist(rng), dist(rng)};
        }

        rp = unicycle::step(rp, u, noise, 1.0);
        rp.x = std::clamp(rp.x, 0.0, 10.0);
        rp.y = std::clamp(rp.y, 0.0, 10.0);
        res.trajectory.push_back(rp);
    }

    res.steps_taken = max_steps;
    return res;
}

void run_monte_carlo(AppState& app) {
    app.mc_trajectories.clear();
    app.mc_stats = {};
    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> jitter(-0.15, 0.15);

    for (int i = 0; i < app.mc_runs; ++i) {
        unicycle::State x0 = app.robot_start;
        x0.x = std::clamp(x0.x + jitter(rng), 0.0, 10.0);
        x0.y = std::clamp(x0.y + jitter(rng), 0.0, 10.0);

        auto res = run_closed_loop(x0, app.sim_max_steps, true, app.ctrl, app.regions);
        app.mc_trajectories.push_back(res.trajectory);
        if (res.reached_goal) {
            ++app.mc_stats.successful;
        } else if (res.hit_obstacle) {
            ++app.mc_stats.failed;
        } else {
            ++app.mc_stats.timeout;
        }
    }
    app.mc_stats.total_runs   = app.mc_runs;
    app.mc_stats.success_rate = (app.mc_runs > 0)
        ? static_cast<float>(app.mc_stats.successful) / app.mc_runs
        : 0.0f;
}

} // namespace gui

int main() {
    constexpr int WIN_W = 1600;
    constexpr int WIN_H = 920;
    constexpr float ARENA_SIZE  = 860.0f;
    constexpr float PANEL_WIDTH = WIN_W - ARENA_SIZE;

    if (!gui::init_graphics(WIN_W, WIN_H, "Robot Navigation System")) {
        std::fprintf(stderr, "Failed to initialize graphics\n");
        return 1;
    }

    gui::AppState app;

    app.abs_tables = grid::precompute_abstraction_tables();
    app.abs_tables_ready = true;

    if (!solver::load_controller_binary(app.controller_path_buf, app.V, app.ctrl)) {
        gui::trigger_synthesis(app);
    } else {
        app.controller_ready = true;
    }

    while (!glfwWindowShouldClose(g_window)) {
        glfwPollEvents();

        if (app.sim_running && !app.sim_paused) {
            gui::sim_step_once(app);
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos({0.0f, 0.0f});
        ImGui::SetNextWindowSize({static_cast<float>(WIN_W), static_cast<float>(WIN_H)});
        ImGui::Begin("##root_win", nullptr,
                     ImGuiWindowFlags_NoTitleBar  |
                     ImGuiWindowFlags_NoResize    |
                     ImGuiWindowFlags_NoMove      |
                     ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);

        ImGui::BeginChild("##arena_child", {ARENA_SIZE, ARENA_SIZE}, false, ImGuiWindowFlags_NoScrollbar);
        gui::render_arena(app, ARENA_SIZE);
        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("##panel_child", {PANEL_WIDTH - 25.0f, static_cast<float>(WIN_H) - 20.0f}, false);
        gui::render_control_panel(app);
        ImGui::EndChild();

        ImGui::End();

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(g_window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.07f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(g_window);
    }

    gui::shutdown_graphics();
    return 0;
}
