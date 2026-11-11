#include "arena_view.hpp"
#include "ui_theme.hpp"
#include "imgui.h"

#include <cmath>
#include <algorithm>
#include <cstdio>

namespace robot_nav::arena {

static ImVec2 world_to_screen(float wx, float wy, ImVec2 origin, float scale) {
    return {
        origin.x + wx * scale,
        origin.y + (10.0f - wy) * scale
    };
}

static ImVec2 screen_to_world(float sx, float sy, ImVec2 origin, float scale) {
    return {
        (sx - origin.x) / scale,
        10.0f - (sy - origin.y) / scale
    };
}

void render_arena_card(AppState& app, float width, float height) {
    theme::begin_card("##arena_card", ImVec2(width, height));

    // -------------------------------------------------------------------------
    // Header
    // -------------------------------------------------------------------------
    if (theme::g_fonts.title) ImGui::PushFont(theme::g_fonts.title);
    ImGui::Text("Arena");
    if (theme::g_fonts.title) ImGui::PopFont();

    ImGui::SameLine(width - 260.0f);
    ImGui::Checkbox("Grid (10 x 10)", &app.show_grid);
    ImGui::SameLine(width - 120.0f);
    ImGui::Checkbox("Trajectories", &app.show_trajectories);

    ImGui::Spacing();

    // -------------------------------------------------------------------------
    // Arena Canvas & Plot Area
    // -------------------------------------------------------------------------
    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    const float margin_left   = 40.0f;
    const float margin_right  = 16.0f;
    const float margin_top    = 12.0f;
    const float margin_bottom = 32.0f;

    float avail_w = width - 28.0f;
    float avail_h = height - 58.0f;

    float plot_dim = std::min(avail_w - (margin_left + margin_right),
                              avail_h - (margin_top + margin_bottom));
    if (plot_dim < 280.0f) plot_dim = 280.0f;

    ImVec2 plot_origin = {canvas_pos.x + margin_left, canvas_pos.y + margin_top};
    float scale = plot_dim / 10.0f;

    // Background of plot
    dl->AddRectFilled(plot_origin,
                      {plot_origin.x + plot_dim, plot_origin.y + plot_dim},
                      IM_COL32(255, 255, 255, 255));

    // Outer border of plot box
    dl->AddRect(plot_origin,
                {plot_origin.x + plot_dim, plot_origin.y + plot_dim},
                IM_COL32(148, 163, 184, 255), 0.0f, 0, 1.2f);

    // -------------------------------------------------------------------------
    // Grid lines & axis ticks
    // -------------------------------------------------------------------------
    ImU32 grid_col = IM_COL32(235, 238, 242, 255);
    ImU32 axis_col = IM_COL32(100, 116, 139, 255);
    ImU32 text_col = IM_COL32(71, 85, 105, 255);

    if (app.show_grid) {
        for (int i = 1; i < 10; ++i) {
            float px = plot_origin.x + i * scale;
            float py = plot_origin.y + i * scale;
            dl->AddLine({px, plot_origin.y}, {px, plot_origin.y + plot_dim}, grid_col, 1.0f);
            dl->AddLine({plot_origin.x, py}, {plot_origin.x + plot_dim, py}, grid_col, 1.0f);
        }
    }

    // Tick marks and numbers
    if (theme::g_fonts.small) ImGui::PushFont(theme::g_fonts.small);
    for (int i = 0; i <= 10; ++i) {
        float px = plot_origin.x + i * scale;
        float py = plot_origin.y + (10 - i) * scale;

        // X tick & label
        dl->AddLine({px, plot_origin.y + plot_dim}, {px, plot_origin.y + plot_dim + 4.0f}, axis_col, 1.2f);
        char buf_x[8];
        std::snprintf(buf_x, sizeof(buf_x), "%d", i);
        ImVec2 sz_x = ImGui::CalcTextSize(buf_x);
        dl->AddText({px - sz_x.x * 0.5f, plot_origin.y + plot_dim + 5.0f}, text_col, buf_x);

        // Y tick & label
        dl->AddLine({plot_origin.x - 4.0f, py}, {plot_origin.x, py}, axis_col, 1.2f);
        char buf_y[8];
        std::snprintf(buf_y, sizeof(buf_y), "%d", i);
        ImVec2 sz_y = ImGui::CalcTextSize(buf_y);
        dl->AddText({plot_origin.x - 7.0f - sz_y.x, py - sz_y.y * 0.5f}, text_col, buf_y);
    }
    if (theme::g_fonts.small) ImGui::PopFont();

    // Axis titles
    if (theme::g_fonts.regular) ImGui::PushFont(theme::g_fonts.regular);
    const char* label_x = "X (m)";
    ImVec2 sz_lx = ImGui::CalcTextSize(label_x);
    dl->AddText({plot_origin.x + (plot_dim - sz_lx.x) * 0.5f, plot_origin.y + plot_dim + 19.0f},
                text_col, label_x);

    const char* label_y = "Y (m)";
    ImVec2 sz_ly = ImGui::CalcTextSize(label_y);
    dl->AddText({plot_origin.x - margin_left + 4.0f, plot_origin.y + (plot_dim - sz_ly.y) * 0.5f},
                text_col, label_y);
    if (theme::g_fonts.regular) ImGui::PopFont();

    // -------------------------------------------------------------------------
    // Region Drawing Helper
    // -------------------------------------------------------------------------
    auto draw_region = [&](double x0, double y0, double x1, double y1,
                           ImU32 fill, ImU32 border, ImU32 text_c,
                           const char* title, const char* subtitle) {
        ImVec2 a = world_to_screen(static_cast<float>(x0), static_cast<float>(y0), plot_origin, scale);
        ImVec2 b = world_to_screen(static_cast<float>(x1), static_cast<float>(y1), plot_origin, scale);
        ImVec2 tl = {std::min(a.x, b.x), std::min(a.y, b.y)};
        ImVec2 br = {std::max(a.x, b.x), std::max(a.y, b.y)};

        dl->AddRectFilled(tl, br, fill);
        dl->AddRect(tl, br, border, 0.0f, 0, 1.8f);

        // Center labels
        ImVec2 ctr = {(tl.x + br.x) * 0.5f, (tl.y + br.y) * 0.5f};
        if (theme::g_fonts.bold) ImGui::PushFont(theme::g_fonts.bold);
        ImVec2 ts_title = ImGui::CalcTextSize(title);
        dl->AddText({ctr.x - ts_title.x * 0.5f, ctr.y - ts_title.y - 1.0f}, text_c, title);
        if (theme::g_fonts.bold) ImGui::PopFont();

        if (theme::g_fonts.small) ImGui::PushFont(theme::g_fonts.small);
        ImVec2 ts_sub = ImGui::CalcTextSize(subtitle);
        dl->AddText({ctr.x - ts_sub.x * 0.5f, ctr.y + 2.0f}, text_c, subtitle);
        if (theme::g_fonts.small) ImGui::PopFont();
    };

    // Draw R1: Waypoint 1 (Yellow / Amber)
    draw_region(app.regions.r1_x0, app.regions.r1_y0, app.regions.r1_x1, app.regions.r1_y1,
                IM_COL32(254, 240, 199, 150), IM_COL32(245, 158, 11, 255), IM_COL32(180, 83, 9, 255),
                "R1", "Waypoint 1");

    // Draw R2: Waypoint 2 (Blue)
    draw_region(app.regions.r2_x0, app.regions.r2_y0, app.regions.r2_x1, app.regions.r2_y1,
                IM_COL32(219, 234, 254, 150), IM_COL32(59, 130, 246, 255), IM_COL32(29, 78, 216, 255),
                "R2", "Waypoint 2");

    // Draw R4: Obstacle (Red / Salmon)
    draw_region(app.regions.r4_x0, app.regions.r4_y0, app.regions.r4_x1, app.regions.r4_y1,
                IM_COL32(254, 226, 226, 160), IM_COL32(239, 68, 68, 255), IM_COL32(185, 28, 28, 255),
                "R4", "Obstacle");

    // Draw R3: Goal (Green)
    draw_region(app.regions.r3_x0, app.regions.r3_y0, app.regions.r3_x1, app.regions.r3_y1,
                IM_COL32(220, 252, 231, 160), IM_COL32(34, 197, 94, 255), IM_COL32(21, 128, 61, 255),
                "R3", "Goal");

    // -------------------------------------------------------------------------
    // Corner handles for interactive resizing
    // -------------------------------------------------------------------------
    auto draw_corner_handles = [&](double x0, double y0, double x1, double y1, ImU32 col) {
        double pts[4][2] = {{x0, y0}, {x1, y0}, {x0, y1}, {x1, y1}};
        for (auto& p : pts) {
            ImVec2 sp = world_to_screen(static_cast<float>(p[0]), static_cast<float>(p[1]), plot_origin, scale);
            dl->AddCircleFilled(sp, 4.5f, IM_COL32(255, 255, 255, 255));
            dl->AddCircle(sp, 4.5f, col, 16, 1.8f);
        }
    };
    draw_corner_handles(app.regions.r1_x0, app.regions.r1_y0, app.regions.r1_x1, app.regions.r1_y1, IM_COL32(245, 158, 11, 255));
    draw_corner_handles(app.regions.r2_x0, app.regions.r2_y0, app.regions.r2_x1, app.regions.r2_y1, IM_COL32(59, 130, 246, 255));
    draw_corner_handles(app.regions.r3_x0, app.regions.r3_y0, app.regions.r3_x1, app.regions.r3_y1, IM_COL32(34, 197, 94, 255));
    draw_corner_handles(app.regions.r4_x0, app.regions.r4_y0, app.regions.r4_x1, app.regions.r4_y1, IM_COL32(239, 68, 68, 255));

    // -------------------------------------------------------------------------
    // Active path (Bold navy polyline with soft cyan glow and step nodes)
    // -------------------------------------------------------------------------
    if (app.show_trajectories && app.active_trajectory.size() > 1) {
        ImU32 glow_col = IM_COL32(56, 189, 248, 85);
        ImU32 line_col = IM_COL32(15, 59, 117, 255);

        // Glow pass
        for (size_t i = 1; i < app.active_trajectory.size(); ++i) {
            ImVec2 p0 = world_to_screen(static_cast<float>(app.active_trajectory[i - 1].x), static_cast<float>(app.active_trajectory[i - 1].y), plot_origin, scale);
            ImVec2 p1 = world_to_screen(static_cast<float>(app.active_trajectory[i].x),     static_cast<float>(app.active_trajectory[i].y),     plot_origin, scale);
            dl->AddLine(p0, p1, glow_col, 5.0f);
        }
        // Crisp navy line
        for (size_t i = 1; i < app.active_trajectory.size(); ++i) {
            ImVec2 p0 = world_to_screen(static_cast<float>(app.active_trajectory[i - 1].x), static_cast<float>(app.active_trajectory[i - 1].y), plot_origin, scale);
            ImVec2 p1 = world_to_screen(static_cast<float>(app.active_trajectory[i].x),     static_cast<float>(app.active_trajectory[i].y),     plot_origin, scale);
            dl->AddLine(p0, p1, line_col, 2.5f);

            // Small step node circles
            dl->AddCircleFilled(p0, 2.5f, IM_COL32(255, 255, 255, 255));
            dl->AddCircle(p0, 2.5f, line_col, 12, 1.2f);
        }
    }

    // Faint initial pose indicator if the robot moved to the goal
    if (app.has_run_result && !app.sim_running) {
        ImVec2 start_pt = world_to_screen(static_cast<float>(app.robot_start.x),
                                          static_cast<float>(app.robot_start.y),
                                          plot_origin, scale);
        dl->AddCircle(start_pt, 7.0f, IM_COL32(148, 163, 184, 180), 16, 1.4f);
        dl->AddCircleFilled(start_pt, 2.5f, IM_COL32(148, 163, 184, 180));
    }

    // -------------------------------------------------------------------------
    // Robot directional representation
    // -------------------------------------------------------------------------
    // When the simulation finishes or reaches the goal, leave the robot at its final state!
    const unicycle::State& rp = (app.sim_running || app.has_run_result) ? app.sim_robot_pos : app.robot_start;
    ImVec2 r_screen = world_to_screen(static_cast<float>(rp.x), static_cast<float>(rp.y), plot_origin, scale);

    float th = -static_cast<float>(rp.theta);
    float arrow_len = 24.0f;
    float arrow_wing = 15.0f;

    ImVec2 tip = {r_screen.x + arrow_len * std::cos(th),
                  r_screen.y + arrow_len * std::sin(th)};
    ImVec2 left_wing = {r_screen.x + arrow_wing * std::cos(th + 2.5f),
                        r_screen.y + arrow_wing * std::sin(th + 2.5f)};
    ImVec2 right_wing = {r_screen.x + arrow_wing * std::cos(th - 2.5f),
                         r_screen.y + arrow_wing * std::sin(th - 2.5f)};
    ImVec2 notch = {r_screen.x + (arrow_wing * 0.45f) * std::cos(th + static_cast<float>(M_PI)),
                    r_screen.y + (arrow_wing * 0.45f) * std::sin(th + static_cast<float>(M_PI))};

    // Body shadow & fill
    ImVec2 shadow_offset = {1.5f, 2.0f};
    dl->AddTriangleFilled({tip.x + shadow_offset.x, tip.y + shadow_offset.y},
                          {left_wing.x + shadow_offset.x, left_wing.y + shadow_offset.y},
                          {notch.x + shadow_offset.x, notch.y + shadow_offset.y},
                          IM_COL32(0, 0, 0, 35));
    dl->AddTriangleFilled({tip.x + shadow_offset.x, tip.y + shadow_offset.y},
                          {notch.x + shadow_offset.x, notch.y + shadow_offset.y},
                          {right_wing.x + shadow_offset.x, right_wing.y + shadow_offset.y},
                          IM_COL32(0, 0, 0, 35));

    dl->AddTriangleFilled(tip, left_wing, notch, IM_COL32(56, 189, 248, 255));
    dl->AddTriangleFilled(tip, notch, right_wing, IM_COL32(37, 99, 235, 255));

    dl->AddLine(tip, left_wing, IM_COL32(15, 59, 117, 255), 1.8f);
    dl->AddLine(left_wing, notch, IM_COL32(15, 59, 117, 255), 1.8f);
    dl->AddLine(notch, right_wing, IM_COL32(15, 59, 117, 255), 1.8f);
    dl->AddLine(right_wing, tip, IM_COL32(15, 59, 117, 255), 1.8f);

    // Center pivot circle
    dl->AddCircleFilled(r_screen, 4.0f, IM_COL32(255, 255, 255, 255));
    dl->AddCircle(r_screen, 4.0f, IM_COL32(15, 59, 117, 255), 12, 1.5f);

    // Rotation handle at arrow tip
    if (!app.sim_running) {
        dl->AddCircleFilled(tip, 4.5f, IM_COL32(255, 215, 0, 240));
        dl->AddCircle(tip, 4.5f, IM_COL32(15, 59, 117, 255), 12, 1.5f);
    }

    // -------------------------------------------------------------------------
    // Mouse Interaction (Moving robot, rotating robot, dragging/resizing regions)
    // -------------------------------------------------------------------------
    ImGui::SetCursorScreenPos(plot_origin);
    ImGui::InvisibleButton("##arena_plot_hitbox", ImVec2(plot_dim, plot_dim));

    bool is_hovered = ImGui::IsItemHovered();
    bool lmb_down   = ImGui::IsMouseDown(ImGuiMouseButton_Left);
    bool lmb_click  = ImGui::IsMouseClicked(ImGuiMouseButton_Left);
    bool rmb_down   = ImGui::IsMouseDown(ImGuiMouseButton_Right);
    bool rmb_click  = ImGui::IsMouseClicked(ImGuiMouseButton_Right);

    ImVec2 mouse_pos = ImGui::GetMousePos();
    ImVec2 mouse_world = screen_to_world(mouse_pos.x, mouse_pos.y, plot_origin, scale);

    double mx = static_cast<double>(mouse_world.x);
    double my = static_cast<double>(mouse_world.y);

    struct RegRef { int id; double *x0, *y0, *x1, *y1; };
    RegRef refs[4] = {
        {1, &app.regions.r1_x0, &app.regions.r1_y0, &app.regions.r1_x1, &app.regions.r1_y1},
        {2, &app.regions.r2_x0, &app.regions.r2_y0, &app.regions.r2_x1, &app.regions.r2_y1},
        {3, &app.regions.r3_x0, &app.regions.r3_y0, &app.regions.r3_x1, &app.regions.r3_y1},
        {4, &app.regions.r4_x0, &app.regions.r4_y0, &app.regions.r4_x1, &app.regions.r4_y1},
    };

    static ImVec2 s_drag_start_mouse = {0.0f, 0.0f};
    static double s_orig_x0 = 0.0, s_orig_y0 = 0.0, s_orig_x1 = 0.0, s_orig_y1 = 0.0;

    auto corner_hit = [&](double x0, double y0, double x1, double y1) -> int {
        double d = 0.40;
        if (std::abs(mx - x0) < d && std::abs(my - y0) < d) return 1; // BL
        if (std::abs(mx - x1) < d && std::abs(my - y0) < d) return 2; // BR
        if (std::abs(mx - x0) < d && std::abs(my - y1) < d) return 3; // TL
        if (std::abs(mx - x1) < d && std::abs(my - y1) < d) return 4; // TR
        return 0;
    };

    // Check hit on robot
    float dist_robot_center = std::sqrt(
        (mouse_pos.x - r_screen.x) * (mouse_pos.x - r_screen.x) +
        (mouse_pos.y - r_screen.y) * (mouse_pos.y - r_screen.y));
    float dist_robot_tip = std::sqrt(
        (mouse_pos.x - tip.x) * (mouse_pos.x - tip.x) +
        (mouse_pos.y - tip.y) * (mouse_pos.y - tip.y));

    bool hover_robot_tip = (dist_robot_tip < 12.0f);
    bool hover_robot_center = (dist_robot_center < 18.0f);

    if (is_hovered && !app.sim_running) {
        // 1. Initial click handling
        if (lmb_click) {
            app.resize_region = 0;
            app.resize_corner = 0;
            app.dragging_region = 0;
            app.dragging_robot_pos = false;
            app.dragging_robot_rot = false;

            if (hover_robot_tip) {
                if (app.has_run_result) {
                    app.robot_start = app.sim_robot_pos;
                    app.has_run_result = false;
                }
                app.dragging_robot_rot = true;
            } else if (hover_robot_center) {
                if (app.has_run_result) {
                    app.robot_start = app.sim_robot_pos;
                    app.has_run_result = false;
                }
                app.dragging_robot_pos = true;
            } else {
                // Check region corners
                for (auto& r : refs) {
                    int c = corner_hit(*r.x0, *r.y0, *r.x1, *r.y1);
                    if (c != 0) {
                        app.resize_region = r.id;
                        app.resize_corner = c;
                        s_drag_start_mouse = mouse_world;
                        s_orig_x0 = *r.x0; s_orig_y0 = *r.y0;
                        s_orig_x1 = *r.x1; s_orig_y1 = *r.y1;
                        break;
                    }
                }
                // Check region interiors
                if (app.resize_region == 0) {
                    for (auto& r : refs) {
                        if (mx >= *r.x0 && mx <= *r.x1 && my >= *r.y0 && my <= *r.y1) {
                            app.dragging_region = r.id;
                            s_drag_start_mouse = mouse_world;
                            s_orig_x0 = *r.x0; s_orig_y0 = *r.y0;
                            s_orig_x1 = *r.x1; s_orig_y1 = *r.y1;
                            break;
                        }
                    }
                }
            }
        } else if (rmb_click && (hover_robot_center || hover_robot_tip)) {
            // Right-click drag rotates the robot
            if (app.has_run_result) {
                app.robot_start = app.sim_robot_pos;
                app.has_run_result = false;
            }
            app.dragging_robot_rot = true;
        }
    }

    // 2. Active dragging update
    if (lmb_down || rmb_down) {
        if (app.dragging_robot_pos && lmb_down) {
            app.robot_start.x = std::clamp(mx, 0.05, 9.95);
            app.robot_start.y = std::clamp(my, 0.05, 9.95);
            app.sim_robot_pos = app.robot_start;
            app.active_trajectory.clear();
            app.active_trajectory.push_back(app.robot_start);
            app.has_run_result = false;
        } else if (app.dragging_robot_rot) {
            double ddx = mx - app.robot_start.x;
            double ddy = my - app.robot_start.y;
            if (std::abs(ddx) > 1e-3 || std::abs(ddy) > 1e-3) {
                app.robot_start.theta = std::atan2(ddy, ddx);
                app.sim_robot_pos.theta = app.robot_start.theta;
                app.has_run_result = false;
            }
        } else if (app.dragging_region != 0 || app.resize_region != 0) {
            mark_arena_modified(app);
            double ddx = static_cast<double>(mouse_world.x - s_drag_start_mouse.x);
            double ddy = static_cast<double>(mouse_world.y - s_drag_start_mouse.y);

            int active_id = (app.dragging_region != 0) ? app.dragging_region : app.resize_region;
            if (active_id >= 1 && active_id <= 4) {
                auto& r = refs[active_id - 1];
                if (app.dragging_region != 0) {
                    double w = s_orig_x1 - s_orig_x0;
                    double h = s_orig_y1 - s_orig_y0;
                    *r.x0 = std::clamp(s_orig_x0 + ddx, 0.0, 10.0 - w);
                    *r.y0 = std::clamp(s_orig_y0 + ddy, 0.0, 10.0 - h);
                    *r.x1 = *r.x0 + w;
                    *r.y1 = *r.y0 + h;
                } else if (app.resize_region != 0) {
                    switch (app.resize_corner) {
                        case 1:
                            *r.x0 = std::clamp(s_orig_x0 + ddx, 0.0, *r.x1 - 0.2);
                            *r.y0 = std::clamp(s_orig_y0 + ddy, 0.0, *r.y1 - 0.2);
                            break;
                        case 2:
                            *r.x1 = std::clamp(s_orig_x1 + ddx, *r.x0 + 0.2, 10.0);
                            *r.y0 = std::clamp(s_orig_y0 + ddy, 0.0, *r.y1 - 0.2);
                            break;
                        case 3:
                            *r.x0 = std::clamp(s_orig_x0 + ddx, 0.0, *r.x1 - 0.2);
                            *r.y1 = std::clamp(s_orig_y1 + ddy, *r.y0 + 0.2, 10.0);
                            break;
                        case 4:
                            *r.x1 = std::clamp(s_orig_x1 + ddx, *r.x0 + 0.2, 10.0);
                            *r.y1 = std::clamp(s_orig_y1 + ddy, *r.y0 + 0.2, 10.0);
                            break;
                    }
                }
            }
        }
    } else {
        app.dragging_robot_pos = false;
        app.dragging_robot_rot = false;
        app.dragging_region    = 0;
        app.resize_region      = 0;
        app.resize_corner      = 0;
    }

    // Tooltip
    if (is_hovered) {
        if (hover_robot_center || hover_robot_tip || app.dragging_robot_pos || app.dragging_robot_rot) {
            ImGui::SetTooltip("Robot Pose:\nPosition: (%.2f m, %.2f m)\nHeading:  %.1f deg\n[LMB Drag Center]: Move robot\n[RMB or Drag Tip]: Rotate heading",
                              rp.x, rp.y, rp.theta * 180.0 / M_PI);
        } else {
            ImGui::SetTooltip("Coordinates: X = %.2f m, Y = %.2f m", mx, my);
        }
    }

    theme::end_card();
}

void render_legend_card(AppState& /*app*/, float width) {
    theme::begin_card("##legend_card", ImVec2(width, 66.0f));

    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 p = ImGui::GetCursorScreenPos();

    if (theme::g_fonts.bold) ImGui::PushFont(theme::g_fonts.bold);
    ImGui::Text("Legend");
    if (theme::g_fonts.bold) ImGui::PopFont();

    ImGui::SameLine();
    ImGui::TextColored(theme::colors::text_muted, "|");
    ImGui::SameLine(0, 14.0f);

    float base_y = p.y + 2.0f;
    float col_x = ImGui::GetCursorScreenPos().x;

    auto draw_item_icon_arrow = [&](float x, float y) {
        ImVec2 c = {x + 8.0f, y + 8.0f};
        dl->AddTriangleFilled({c.x + 7.0f, c.y}, {c.x - 5.0f, c.y - 5.0f}, {c.x - 2.0f, c.y}, IM_COL32(56, 189, 248, 255));
        dl->AddTriangleFilled({c.x + 7.0f, c.y}, {c.x - 2.0f, c.y}, {c.x - 5.0f, c.y + 5.0f}, IM_COL32(37, 99, 235, 255));
        dl->AddLine({c.x + 7.0f, c.y}, {c.x - 5.0f, c.y - 5.0f}, IM_COL32(15, 59, 117, 255), 1.2f);
        dl->AddLine({c.x - 5.0f, c.y - 5.0f}, {c.x - 2.0f, c.y}, IM_COL32(15, 59, 117, 255), 1.2f);
        dl->AddLine({c.x - 2.0f, c.y}, {c.x - 5.0f, c.y + 5.0f}, IM_COL32(15, 59, 117, 255), 1.2f);
        dl->AddLine({c.x - 5.0f, c.y + 5.0f}, {c.x + 7.0f, c.y}, IM_COL32(15, 59, 117, 255), 1.2f);
    };

    auto draw_item_line_active = [&](float x, float y) {
        dl->AddLine({x, y + 8.0f}, {x + 20.0f, y + 8.0f}, IM_COL32(56, 189, 248, 80), 4.0f);
        dl->AddLine({x, y + 8.0f}, {x + 20.0f, y + 8.0f}, IM_COL32(15, 59, 117, 255), 2.2f);
    };

    auto draw_item_bundle = [&](float x, float y) {
        dl->AddLine({x, y + 5.0f},  {x + 20.0f, y + 5.0f},  IM_COL32(56, 189, 248, 120), 1.2f);
        dl->AddLine({x, y + 8.0f},  {x + 20.0f, y + 8.0f},  IM_COL32(56, 189, 248, 160), 1.2f);
        dl->AddLine({x, y + 11.0f}, {x + 20.0f, y + 11.0f}, IM_COL32(56, 189, 248, 120), 1.2f);
    };

    auto draw_item_box = [&](float x, float y, ImU32 fill, ImU32 border) {
        dl->AddRectFilled({x, y + 2.0f}, {x + 15.0f, y + 15.0f}, fill, 3.0f);
        dl->AddRect({x, y + 2.0f}, {x + 15.0f, y + 15.0f}, border, 3.0f, 0, 1.4f);
    };

    if (theme::g_fonts.small) ImGui::PushFont(theme::g_fonts.small);

    // Row 1: Robot, Path, R1, R2
    float x_cur = col_x;
    draw_item_icon_arrow(x_cur, base_y);
    dl->AddText({x_cur + 20.0f, base_y}, IM_COL32(30, 41, 59, 255), "Robot");
    x_cur += 85.0f;

    draw_item_line_active(x_cur, base_y);
    dl->AddText({x_cur + 26.0f, base_y}, IM_COL32(30, 41, 59, 255), "Path");
    x_cur += 85.0f;

    draw_item_box(x_cur, base_y, IM_COL32(254, 240, 199, 255), IM_COL32(245, 158, 11, 255));
    dl->AddText({x_cur + 22.0f, base_y}, IM_COL32(30, 41, 59, 255), "R1 Waypoint");
    x_cur += 115.0f;

    draw_item_box(x_cur, base_y, IM_COL32(219, 234, 254, 255), IM_COL32(59, 130, 246, 255));
    dl->AddText({x_cur + 22.0f, base_y}, IM_COL32(30, 41, 59, 255), "R2 Waypoint");

    // Row 2: Bundle, R3 Goal, R4 Obstacle
    float row2_y = base_y + 20.0f;
    x_cur = col_x;

    draw_item_bundle(x_cur, row2_y);
    dl->AddText({x_cur + 26.0f, row2_y}, IM_COL32(30, 41, 59, 255), "Bundle");
    x_cur += 85.0f;

    draw_item_box(x_cur, row2_y, IM_COL32(220, 252, 231, 255), IM_COL32(34, 197, 94, 255));
    dl->AddText({x_cur + 22.0f, row2_y}, IM_COL32(30, 41, 59, 255), "R3 Goal");
    x_cur += 105.0f;

    draw_item_box(x_cur, row2_y, IM_COL32(254, 226, 226, 255), IM_COL32(239, 68, 68, 255));
    dl->AddText({x_cur + 22.0f, row2_y}, IM_COL32(30, 41, 59, 255), "R4 Obstacle");

    if (theme::g_fonts.small) ImGui::PopFont();

    theme::end_card();
}

} // namespace robot_nav::arena
