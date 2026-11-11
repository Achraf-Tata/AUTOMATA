// =============================================================================
// main_robot.cpp — Member 5
//
// Program entry point and GUI implementation.
//
// The window layout:
//   [ Arena canvas (left, square) | Control panel (right) ]
//
// Solver runs in a background std::thread to keep the GUI responsive.
// Progress is communicated via atomic variables in AppState.
// =============================================================================
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include "gui_arena.hpp"
#include "reachability_cpu.hpp"
#include "product_automaton.hpp"
#include "grid_quantizer.hpp"
#include "unicycle_model.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include <thread>
#include <atomic>
#include <mutex>
#include <random>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <cstdio>

// ============================================================================
// Globals (file-scope only)
// ============================================================================

static GLFWwindow* g_window = nullptr;

// ============================================================================
// Graphics init / shutdown
// ============================================================================

namespace gui {

bool init_graphics(int window_w, int window_h, const char* title) {
    if (!glfwInit()) return false;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

    g_window = glfwCreateWindow(window_w, window_h, title, nullptr, nullptr);
    if (!g_window) { glfwTerminate(); return false; }
    glfwMakeContextCurrent(g_window);
    glfwSwapInterval(1); // vsync

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Style: dark theme with colourful accents
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding  = 6.0f;
    style.FrameRounding   = 4.0f;
    style.GrabRounding    = 4.0f;
    style.FramePadding    = {6, 4};
    style.ItemSpacing     = {8, 6};
    style.Colors[ImGuiCol_Header]        = ImVec4(0.26f, 0.59f, 0.98f, 0.31f);
    style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.26f, 0.59f, 0.98f, 0.80f);
    style.Colors[ImGuiCol_Button]        = ImVec4(0.26f, 0.59f, 0.98f, 0.40f);
    style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);

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

// ============================================================================
// Coordinate helpers: world ↔ screen
// ============================================================================

/// Converts a world (x, y) point to ImGui screen coordinates.
/// World (0,0) maps to canvas bottom-left; y increases upward in world coords
/// but downward in screen coords, so we flip y.
static ImVec2 world_to_screen(float wx, float wy,
                               ImVec2 origin, float scale, float canvas_size) {
    return { origin.x + wx * scale,
             origin.y + canvas_size - wy * scale };
}

/// Inverse: screen → world.
static ImVec2 screen_to_world(float sx, float sy,
                               ImVec2 origin, float scale, float canvas_size) {
    return { (sx - origin.x) / scale,
             (canvas_size - (sy - origin.y)) / scale };
}

// ============================================================================
// Arena renderer
// ============================================================================

void render_arena(AppState& app, float canvas_size) {
    // --- setup canvas ---
    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    float scale = canvas_size / 10.0f; // 10 world units across canvas
    ImVec2 origin = canvas_pos;

    // Store for mouse interaction
    static ImVec2 s_origin;
    static float  s_scale;
    static float  s_canvas_size;
    s_origin      = origin;
    s_scale       = scale;
    s_canvas_size = canvas_size;

    ImDrawList* dl = ImGui::GetWindowDrawList();

    // --- background ---
    dl->AddRectFilled(canvas_pos,
                      {canvas_pos.x + canvas_size, canvas_pos.y + canvas_size},
                      IM_COL32(30, 30, 40, 255));

    // --- grid lines (every 1 unit) ---
    if (app.show_grid) {
        ImU32 grid_col = IM_COL32(60, 60, 80, 255);
        for (int i = 0; i <= 10; ++i) {
            float px = origin.x + i * scale;
            dl->AddLine({px, origin.y}, {px, origin.y + canvas_size}, grid_col);
            float py = origin.y + i * scale;
            dl->AddLine({origin.x, py}, {origin.x + canvas_size, py}, grid_col);
        }
    }

    // --- helper lambda to draw a world-space rectangle ---
    auto draw_region = [&](double x0, double y0, double x1, double y1,
                           ImU32 fill, ImU32 border, const char* label) {
        ImVec2 a = world_to_screen(static_cast<float>(x0), static_cast<float>(y0), origin, scale, canvas_size);
        ImVec2 b = world_to_screen(static_cast<float>(x1), static_cast<float>(y1), origin, scale, canvas_size);
        // a is bottom-left in world → top-right in screen (y flipped)
        // b is top-right in world  → bottom-left in screen
        // So min/max to get screen rect corners:
        ImVec2 tl = {std::min(a.x, b.x), std::min(a.y, b.y)};
        ImVec2 br = {std::max(a.x, b.x), std::max(a.y, b.y)};
        dl->AddRectFilled(tl, br, fill);
        dl->AddRect(tl, br, border, 0, 0, 2.0f);
        // Label centred
        ImVec2 ctr = {(tl.x + br.x) * 0.5f, (tl.y + br.y) * 0.5f};
        ImVec2 ts  = ImGui::CalcTextSize(label);
        dl->AddText({ctr.x - ts.x * 0.5f, ctr.y - ts.y * 0.5f},
                    IM_COL32(255, 255, 255, 220), label);
    };

    // --- draw R4 (obstacle, red, behind others) ---
    draw_region(app.regions.r4_x0, app.regions.r4_y0,
                app.regions.r4_x1, app.regions.r4_y1,
                IM_COL32(180, 40, 40, 100), IM_COL32(220, 60, 60, 255), "R4\n(obstacle)");

    // --- draw R1 (green) ---
    draw_region(app.regions.r1_x0, app.regions.r1_y0,
                app.regions.r1_x1, app.regions.r1_y1,
                IM_COL32(40, 180, 80, 120), IM_COL32(60, 220, 100, 255), "R1\n(visit 1st)");

    // --- draw R2 (blue) ---
    draw_region(app.regions.r2_x0, app.regions.r2_y0,
                app.regions.r2_x1, app.regions.r2_y1,
                IM_COL32(40, 100, 200, 120), IM_COL32(60, 140, 240, 255), "R2\n(visit 2nd)");

    // --- draw R3 (yellow, goal) ---
    draw_region(app.regions.r3_x0, app.regions.r3_y0,
                app.regions.r3_x1, app.regions.r3_y1,
                IM_COL32(200, 180, 40, 120), IM_COL32(240, 210, 60, 255), "R3\n(goal)");

    // --- draw Monte Carlo trajectories (thin grey lines) ---
    if (app.show_all_mc && !app.mc_trajectories.empty()) {
        ImU32 mc_col = IM_COL32(140, 140, 160, 60);
        for (const auto& traj : app.mc_trajectories) {
            for (size_t i = 1; i < traj.size(); ++i) {
                ImVec2 pa = world_to_screen((float)traj[i-1].x, (float)traj[i-1].y,
                                            origin, scale, canvas_size);
                ImVec2 pb = world_to_screen((float)traj[i].x,   (float)traj[i].y,
                                            origin, scale, canvas_size);
                dl->AddLine(pa, pb, mc_col, 1.0f);
            }
        }
    }

    // --- draw current simulation trajectory (white) ---
    if (app.sim_trajectory.size() > 1) {
        for (size_t i = 1; i < app.sim_trajectory.size(); ++i) {
            ImVec2 pa = world_to_screen((float)app.sim_trajectory[i-1].x,
                                        (float)app.sim_trajectory[i-1].y,
                                        origin, scale, canvas_size);
            ImVec2 pb = world_to_screen((float)app.sim_trajectory[i].x,
                                        (float)app.sim_trajectory[i].y,
                                        origin, scale, canvas_size);
            dl->AddLine(pa, pb, IM_COL32(255, 255, 255, 200), 2.0f);
        }
    }

    // --- draw robot as arrow ---
    {
        unicycle::State& rp = app.sim_running ? app.sim_robot_pos : app.robot_start;
        ImVec2 rpos = world_to_screen((float)rp.x, (float)rp.y, origin, scale, canvas_size);
        float  len  = scale * 0.4f;
        float  th   = -(float)rp.theta; // flip for screen coords
        ImVec2 tip  = {rpos.x + len * std::cos(th), rpos.y + len * std::sin(th)};
        ImVec2 tail = {rpos.x - len * 0.5f * std::cos(th),
                       rpos.y - len * 0.5f * std::sin(th)};
        ImVec2 left = {rpos.x + (len * 0.4f) * std::cos(th + 2.4f),
                       rpos.y + (len * 0.4f) * std::sin(th + 2.4f)};
        ImVec2 right= {rpos.x + (len * 0.4f) * std::cos(th - 2.4f),
                       rpos.y + (len * 0.4f) * std::sin(th - 2.4f)};
        ImU32 rcol = IM_COL32(255, 220, 50, 255);
        dl->AddLine(tail, tip,   rcol, 2.5f);
        dl->AddLine(tip,  left,  rcol, 2.0f);
        dl->AddLine(tip,  right, rcol, 2.0f);
        dl->AddCircleFilled(rpos, 5.0f, rcol);
    }

    // --- invisible button over arena to capture mouse events ---
    ImGui::InvisibleButton("##arena", {canvas_size, canvas_size});
    bool hovered = ImGui::IsItemHovered();
    bool lmb_down = ImGui::IsMouseDown(ImGuiMouseButton_Left);
    bool lmb_click = ImGui::IsMouseClicked(ImGuiMouseButton_Left);
    ImVec2 mpos = ImGui::GetMousePos();

    // Convert mouse to world coords
    ImVec2 mworld = screen_to_world(mpos.x, mpos.y, origin, scale, canvas_size);
    float mx = mworld.x, my = mworld.y;

    // Corner-resize: check if mouse is near a region corner (within 0.3 world units)
    static int s_resize_region = 0;  // 1-4
    static int s_resize_corner = 0;  // 1=BL,2=BR,3=TL,4=TR
    auto corner_check = [&](double x0, double y0, double x1, double y1) -> int {
        double d = 0.3;
        double dmx = static_cast<double>(mx);
        double dmy = static_cast<double>(my);
        if (std::abs(dmx-x0)<d && std::abs(dmy-y0)<d) return 1; // BL
        if (std::abs(dmx-x1)<d && std::abs(dmy-y0)<d) return 2; // BR
        if (std::abs(dmx-x0)<d && std::abs(dmy-y1)<d) return 3; // TL
        if (std::abs(dmx-x1)<d && std::abs(dmy-y1)<d) return 4; // TR
        return 0;
    };
    auto inside_region = [&](double x0, double y0, double x1, double y1) -> bool {
        double dmx = static_cast<double>(mx);
        double dmy = static_cast<double>(my);
        return dmx >= x0 && dmx <= x1 && dmy >= y0 && dmy <= y1;
    };

    static ImVec2 s_drag_start_mouse  = {0,0};
    static double s_drag_start_x0 = 0.0, s_drag_start_y0 = 0.0;
    static double s_drag_start_x1 = 0.0, s_drag_start_y1 = 0.0;

    if (hovered && lmb_click && !app.sim_running) {
        s_resize_region = 0; s_resize_corner = 0;
        // Check corners first (resize takes priority over drag)
        struct RR { int id; double *x0,*y0,*x1,*y1; };
        RR regions[4] = {
            {1,&app.regions.r1_x0,&app.regions.r1_y0,&app.regions.r1_x1,&app.regions.r1_y1},
            {2,&app.regions.r2_x0,&app.regions.r2_y0,&app.regions.r2_x1,&app.regions.r2_y1},
            {3,&app.regions.r3_x0,&app.regions.r3_y0,&app.regions.r3_x1,&app.regions.r3_y1},
            {4,&app.regions.r4_x0,&app.regions.r4_y0,&app.regions.r4_x1,&app.regions.r4_y1},
        };
        for (auto& r : regions) {
            int c = corner_check(*r.x0,*r.y0,*r.x1,*r.y1);
            if (c) {
                s_resize_region = r.id; s_resize_corner = c;
                s_drag_start_mouse = mworld;
                s_drag_start_x0 = *r.x0; s_drag_start_y0 = *r.y0;
                s_drag_start_x1 = *r.x1; s_drag_start_y1 = *r.y1;
                break;
            }
        }
        if (s_resize_region == 0) {
            // Check drag (move whole region)
            app.dragging_region = 0;
            for (auto& r : regions) {
                if (inside_region(*r.x0,*r.y0,*r.x1,*r.y1)) {
                    app.dragging_region = r.id;
                    s_drag_start_mouse = mworld;
                    s_drag_start_x0 = *r.x0; s_drag_start_y0 = *r.y0;
                    s_drag_start_x1 = *r.x1; s_drag_start_y1 = *r.y1;
                    break;
                }
            }
            if (app.dragging_region == 0) {
                // Check robot
                double dx = static_cast<double>(mx) - app.robot_start.x;
                double dy = static_cast<double>(my) - app.robot_start.y;
                if (dx*dx+dy*dy < 0.3*0.3) {
                    app.dragging_robot = true;
                    s_drag_start_mouse = mworld;
                }
            }
        }
    }

    // Apply drag / resize while mouse held
    if (lmb_down) {
        double ddx = static_cast<double>(mx - s_drag_start_mouse.x);
        double ddy = static_cast<double>(my - s_drag_start_mouse.y);

        auto apply_region = [&](double* x0, double* y0, double* x1, double* y1) {
            if (app.dragging_region) {
                *x0 = std::clamp(s_drag_start_x0 + ddx, 0.0, 10.0);
                *y0 = std::clamp(s_drag_start_y0 + ddy, 0.0, 10.0);
                *x1 = std::clamp(s_drag_start_x1 + ddx, 0.0, 10.0);
                *y1 = std::clamp(s_drag_start_y1 + ddy, 0.0, 10.0);
            } else if (s_resize_region) {
                switch (s_resize_corner) {
                    case 1: // BL
                        *x0 = std::clamp(s_drag_start_x0 + ddx, 0.0, *x1 - 0.2);
                        *y0 = std::clamp(s_drag_start_y0 + ddy, 0.0, *y1 - 0.2);
                        break;
                    case 2: // BR
                        *x1 = std::clamp(s_drag_start_x1 + ddx, *x0 + 0.2, 10.0);
                        *y0 = std::clamp(s_drag_start_y0 + ddy, 0.0, *y1 - 0.2);
                        break;
                    case 3: // TL
                        *x0 = std::clamp(s_drag_start_x0 + ddx, 0.0, *x1 - 0.2);
                        *y1 = std::clamp(s_drag_start_y1 + ddy, *y0 + 0.2, 10.0);
                        break;
                    case 4: // TR
                        *x1 = std::clamp(s_drag_start_x1 + ddx, *x0 + 0.2, 10.0);
                        *y1 = std::clamp(s_drag_start_y1 + ddy, *y0 + 0.2, 10.0);
                        break;
                }
            }
        };

        int drag_id = (app.dragging_region) ? app.dragging_region : s_resize_region;
        switch (drag_id) {
            case 1: apply_region(&app.regions.r1_x0,&app.regions.r1_y0,
                                 &app.regions.r1_x1,&app.regions.r1_y1); break;
            case 2: apply_region(&app.regions.r2_x0,&app.regions.r2_y0,
                                 &app.regions.r2_x1,&app.regions.r2_y1); break;
            case 3: apply_region(&app.regions.r3_x0,&app.regions.r3_y0,
                                 &app.regions.r3_x1,&app.regions.r3_y1); break;
            case 4: apply_region(&app.regions.r4_x0,&app.regions.r4_y0,
                                 &app.regions.r4_x1,&app.regions.r4_y1); break;
        }
        if (app.dragging_robot) {
            app.robot_start.x = std::clamp(static_cast<double>(mx), 0.0, 10.0);
            app.robot_start.y = std::clamp(static_cast<double>(my), 0.0, 10.0);
        }
    } else {
        // Mouse released
        app.dragging_region = 0;
        app.dragging_robot  = false;
        s_resize_region = 0;
        s_resize_corner = 0;
    }

    // Draw resize handles on each region corner
    if (!app.sim_running) {
        auto draw_handles = [&](double x0, double y0, double x1, double y1, ImU32 col) {
            double pts[4][2] = {{x0,y0},{x1,y0},{x0,y1},{x1,y1}};
            for (auto& p : pts) {
                ImVec2 sp = world_to_screen(static_cast<float>(p[0]), static_cast<float>(p[1]), origin, scale, canvas_size);
                dl->AddCircleFilled(sp, 5.0f, col);
                dl->AddCircle(sp, 7.0f, IM_COL32(255,255,255,180), 12, 1.5f);
            }
        };
        draw_handles(app.regions.r1_x0,app.regions.r1_y0,app.regions.r1_x1,app.regions.r1_y1,IM_COL32(60,220,100,255));
        draw_handles(app.regions.r2_x0,app.regions.r2_y0,app.regions.r2_x1,app.regions.r2_y1,IM_COL32(60,140,240,255));
        draw_handles(app.regions.r3_x0,app.regions.r3_y0,app.regions.r3_x1,app.regions.r3_y1,IM_COL32(240,210,60,255));
        draw_handles(app.regions.r4_x0,app.regions.r4_y0,app.regions.r4_x1,app.regions.r4_y1,IM_COL32(220,60,60,255));
    }

    // Coordinate tooltip on hover
    if (hovered) {
        ImGui::SetTooltip("World (%.2f, %.2f)\nDFA state: q=%d",
                          mx, my, app.sim_dfa_state);
    }

    // Border around the arena
    dl->AddRect(canvas_pos, {canvas_pos.x+canvas_size, canvas_pos.y+canvas_size},
                IM_COL32(180,180,200,255), 0, 0, 2.0f);
}

// ============================================================================
// Control panel
// ============================================================================

void render_control_panel(AppState& app) {
    ImGui::SetNextWindowPos({0,0}, ImGuiCond_Always);
    // Right panel content
    ImGui::TextColored({0.3f,0.8f,1.0f,1.0f}, "AUTOMATA — Robot Navigation");
    ImGui::Separator();

    // ---- Solver section ----
    ImGui::TextDisabled("STEP 1: Set regions, then run the solver.");
    if (!app.footprint_ready) {
        ImGui::TextColored({1,0.6f,0.2f,1}, "Precomputing footprint table...");
    }
    if (ImGui::Button(app.solving ? "Solving..." : "Run Solver", {-1, 0})) {
        if (!app.solving && app.footprint_ready) {
            app.controller_ready = false;
            app.solving = true;
            app.solve_progress = 0.0f;
            app.V    = automaton::init_value_function();
            app.ctrl = automaton::init_controller();
            // Launch solver in background thread
            std::thread([&app]() {
                app.solve_iteration = solver::solve_reachability_openmp(
                    app.V, app.ctrl, app.regions, app.footprint,
                    [&app](int iter, float delta) {
                        app.solve_iteration = iter;
                        app.solve_delta     = delta;
                        app.solve_progress  = std::min(1.0f,
                            (float)iter / solver::MAX_ITERATIONS);
                    });
                app.solving         = false;
                app.controller_ready = true;
            }).detach();
        }
    }
    if (app.solving) {
        ImGui::ProgressBar(app.solve_progress, {-1, 0});
        ImGui::Text("Iteration %d  |  delta %.5f", app.solve_iteration, app.solve_delta);
    } else if (app.controller_ready) {
        ImGui::TextColored({0.2f,1,0.4f,1}, "Controller ready. Iterations: %d",
                           app.solve_iteration);
    }

    // ---- Save/Load controller ----
    ImGui::Spacing();
    ImGui::InputText("##path", app.controller_path_buf, sizeof(app.controller_path_buf));
    ImGui::SameLine();
    if (ImGui::Button("Save")) {
        solver::save_controller_binary(app.controller_path_buf, app.V, app.ctrl);
    }
    ImGui::SameLine();
    if (ImGui::Button("Load")) {
        if (solver::load_controller_binary(app.controller_path_buf, app.V, app.ctrl)) {
            app.controller_ready = true;
        }
    }

    ImGui::Separator();

    // ---- Simulation section ----
    ImGui::TextDisabled("STEP 2: Place robot, then simulate.");
    ImGui::Text("Robot start: (%.2f, %.2f, %.1f deg)",
                app.robot_start.x, app.robot_start.y,
                app.robot_start.theta * 180.0 / M_PI);
    float th_deg = static_cast<float>(app.robot_start.theta * 180.0 / M_PI);
    if (ImGui::SliderFloat("Heading (deg)", &th_deg, -180.0f, 180.0f, "%.1f deg")) {
        app.robot_start.theta = static_cast<double>(th_deg) * M_PI / 180.0;
    }
    ImGui::SliderInt("Max steps", &app.sim_max_steps, 50, 500);
    ImGui::Checkbox("Add noise", &app.sim_with_noise);

    if (!app.sim_running) {
        if (ImGui::Button("Start Simulation", {-1,0}) && app.controller_ready) {
            app.sim_running    = true;
            app.sim_paused     = false;
            app.sim_step       = 0;
            app.sim_dfa_state  = 1;
            app.sim_robot_pos  = app.robot_start;
            app.sim_trajectory.clear();
            app.sim_trajectory.push_back(app.robot_start);
        }
    } else {
        if (ImGui::Button(app.sim_paused ? "Resume" : "Pause", {80,0}))
            app.sim_paused = !app.sim_paused;
        ImGui::SameLine();
        if (ImGui::Button("Stop", {80,0})) {
            app.sim_running = false;
            app.sim_paused  = false;
        }
        ImGui::Text("Step %d / %d  |  DFA q=%d",
                    app.sim_step, app.sim_max_steps, app.sim_dfa_state);
    }

    ImGui::Separator();

    // ---- Monte Carlo section ----
    ImGui::TextDisabled("STEP 3: Monte Carlo (50 runs).");
    ImGui::SliderInt("Runs", &app.mc_runs, 10, 200);
    if (ImGui::Button("Run Monte Carlo", {-1,0}) && app.controller_ready && !app.mc_running) {
        app.mc_running = true;
        std::thread([&app]() {
            run_monte_carlo(app);
            app.mc_running = false;
        }).detach();
    }
    if (app.mc_running) {
        ImGui::TextColored({1,0.8f,0,1}, "Running...");
    } else if (app.mc_stats.total_runs > 0) {
        auto& s = app.mc_stats;
        ImGui::TextColored({0.2f,1,0.4f,1}, "Success: %d / %d  (%.1f%%)",
                           s.successful, s.total_runs, s.success_rate * 100.0f);
        ImGui::Text("Failed (R4): %d  |  Timeout: %d", s.failed, s.timeout);
    }
    ImGui::Checkbox("Show all trajectories", &app.show_all_mc);

    ImGui::Separator();

    // ---- Region readouts ----
    ImGui::TextDisabled("Region positions (drag on canvas to edit):");
    ImGui::Columns(2, nullptr, false);
    auto reg_line = [](const char* name, double x0, double y0, double x1, double y1) {
        ImGui::Text("%s: [%.1f-%.1f] x [%.1f-%.1f]", name, x0, x1, y0, y1);
    };
    reg_line("R1", app.regions.r1_x0, app.regions.r1_y0, app.regions.r1_x1, app.regions.r1_y1);
    reg_line("R2", app.regions.r2_x0, app.regions.r2_y0, app.regions.r2_x1, app.regions.r2_y1);
    ImGui::NextColumn();
    reg_line("R3", app.regions.r3_x0, app.regions.r3_y0, app.regions.r3_x1, app.regions.r3_y1);
    reg_line("R4", app.regions.r4_x0, app.regions.r4_y0, app.regions.r4_x1, app.regions.r4_y1);
    ImGui::Columns(1);

    ImGui::Separator();
    ImGui::Checkbox("Show grid", &app.show_grid);
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
}

// ============================================================================
// Simulation helpers
// ============================================================================

bool sim_step_once(AppState& app) {
    if (!app.controller_ready || app.sim_step >= app.sim_max_steps) {
        app.sim_running = false;
        return false;
    }
    if (app.sim_dfa_state == 4 || app.sim_dfa_state == 5) {
        app.sim_running = false;
        return false;
    }

    auto& rp = app.sim_robot_pos;
    grid::GridCoord gc = grid::quantize({rp.x, rp.y, rp.theta});
    int pidx = automaton::product_index(app.sim_dfa_state, grid::flat_index(gc));
    uint8_t ui = app.ctrl[pidx];
    if (ui == 255) { app.sim_running = false; return false; }

    const unicycle::Control& u = unicycle::ALL_CONTROLS[ui];
    std::array<double,3> noise = {0,0,0};
    if (app.sim_with_noise) {
        static std::mt19937 rng(std::random_device{}());
        std::uniform_real_distribution<double> dist(-0.05, 0.05);
        noise = {dist(rng), dist(rng), dist(rng)};
    }
    rp = unicycle::step(rp, u, noise);
    // Clamp to arena
    rp.x = std::clamp(rp.x, 0.0, 10.0);
    rp.y = std::clamp(rp.y, 0.0, 10.0);

    int label = grid::get_region_label(rp.x, rp.y, app.regions);
    app.sim_dfa_state = automaton::dfa_transition(app.sim_dfa_state, label);
    app.sim_trajectory.push_back(rp);
    ++app.sim_step;
    return true;
}

SimulationResult run_closed_loop(
    const unicycle::State&       x0,
    int                          dfa_start,
    int                          max_steps,
    bool                         with_noise,
    const std::vector<float>&    V,
    const std::vector<uint8_t>&  ctrl,
    const grid::RegionConfig&    regions)
{
    (void)V;
    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> dist(-0.05, 0.05);

    SimulationResult res;
    res.trajectory.push_back(x0);
    unicycle::State rp = x0;
    int q = dfa_start;

    for (int s = 0; s < max_steps; ++s) {
        if (q == 4) { res.reached_goal = true;  res.steps_taken = s; return res; }
        if (q == 5) { res.hit_obstacle  = true; res.steps_taken = s; return res; }
        grid::GridCoord gc = grid::quantize({rp.x, rp.y, rp.theta});
        int pidx = automaton::product_index(q, grid::flat_index(gc));
        uint8_t ui = ctrl[pidx];
        if (ui == 255) break;
        const unicycle::Control& u = unicycle::ALL_CONTROLS[ui];
        std::array<double,3> noise = {0,0,0};
        if (with_noise) noise = {dist(rng), dist(rng), dist(rng)};
        rp = unicycle::step(rp, u, noise);
        rp.x = std::clamp(rp.x, 0.0, 10.0);
        rp.y = std::clamp(rp.y, 0.0, 10.0);
        int label = grid::get_region_label(rp.x, rp.y, regions);
        q = automaton::dfa_transition(q, label);
        res.trajectory.push_back(rp);
    }
    res.steps_taken = max_steps;
    return res;
}

void run_monte_carlo(AppState& app) {
    app.mc_trajectories.clear();
    app.mc_stats = {};
    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> jitter(-0.2, 0.2);

    for (int i = 0; i < app.mc_runs; ++i) {
        unicycle::State x0 = app.robot_start;
        x0.x = std::clamp(x0.x + jitter(rng), 0.0, 10.0);
        x0.y = std::clamp(x0.y + jitter(rng), 0.0, 10.0);
        auto res = run_closed_loop(x0, 1, app.sim_max_steps, true,
                                   app.V, app.ctrl, app.regions);
        app.mc_trajectories.push_back(res.trajectory);
        if (res.reached_goal) ++app.mc_stats.successful;
        else if (res.hit_obstacle) ++app.mc_stats.failed;
        else ++app.mc_stats.timeout;
    }
    app.mc_stats.total_runs  = app.mc_runs;
    app.mc_stats.success_rate = (float)app.mc_stats.successful / app.mc_runs;
}

} // namespace gui

// ============================================================================
// main
// ============================================================================

int main() {
    // Window size: 1600 × 900
    constexpr int WIN_W = 1600;
    constexpr int WIN_H = 900;
    constexpr float ARENA_SIZE  = 820.0f; // canvas square side in pixels
    constexpr float PANEL_WIDTH = WIN_W - ARENA_SIZE;

    if (!gui::init_graphics(WIN_W, WIN_H, "Robot Navigation — AUTOMATA")) {
        std::fprintf(stderr, "Failed to initialise graphics\n");
        return 1;
    }

    gui::AppState app;

    // Precompute footprint table in a background thread
    std::thread([&app]() {
        app.footprint       = grid::precompute_action_footprints();
        app.footprint_ready = true;
    }).detach();

    // Try to load a previously saved controller
    if (solver::load_controller_binary("controller.bin", app.V, app.ctrl)) {
        app.controller_ready = true;
    }

    // ---- Main render loop ----
    while (!glfwWindowShouldClose(g_window)) {
        glfwPollEvents();

        // Advance simulation one step per frame if running and not paused
        if (app.sim_running && !app.sim_paused) {
            gui::sim_step_once(app);
        }

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Full-screen dockspace window
        ImGui::SetNextWindowPos({0, 0});
        ImGui::SetNextWindowSize({(float)WIN_W, (float)WIN_H});
        ImGui::Begin("##root", nullptr,
                     ImGuiWindowFlags_NoTitleBar |
                     ImGuiWindowFlags_NoResize   |
                     ImGuiWindowFlags_NoMove     |
                     ImGuiWindowFlags_NoScrollbar|
                     ImGuiWindowFlags_NoBringToFrontOnFocus);

        // --- Arena (left side) ---
        ImGui::BeginChild("##arena_panel", {ARENA_SIZE, ARENA_SIZE}, false,
                          ImGuiWindowFlags_NoScrollbar);
        gui::render_arena(app, ARENA_SIZE);
        ImGui::EndChild();

        ImGui::SameLine();

        // --- Control panel (right side) ---
        ImGui::BeginChild("##ctrl_panel", {PANEL_WIDTH - 10, (float)WIN_H - 20}, false);
        gui::render_control_panel(app);
        ImGui::EndChild();

        ImGui::End();

        // Render
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(g_window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(g_window);
    }

    gui::shutdown_graphics();
    return 0;
}
