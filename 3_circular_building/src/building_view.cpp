#include "building_view.hpp"
#include "ui_theme.hpp"
#include "imgui.h"

#include <cmath>
#include <algorithm>
#include <vector>
#include <cstdio>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace circular::view {

using namespace ImGui;
using namespace theme;
using namespace colors;
using namespace model;
using namespace abstraction;

// Helper to draw an arrowhead vector on the canvas
static void draw_arrow(ImDrawList* dl, ImVec2 start, ImVec2 end, ImU32 color, float thickness = 2.0f, float head_size = 6.0f) {
    dl->AddLine(start, end, color, thickness);

    float dx = end.x - start.x;
    float dy = end.y - start.y;
    float len = std::hypot(dx, dy);
    if (len < 1e-3f) return;

    dx /= len;
    dy /= len;

    ImVec2 p1 = {end.x - head_size * dx - head_size * 0.5f * (-dy), end.y - head_size * dy - head_size * 0.5f * dx};
    ImVec2 p2 = {end.x - head_size * dx + head_size * 0.5f * (-dy), end.y - head_size * dy + head_size * 0.5f * dx};

    dl->AddTriangleFilled(end, p1, p2, color);
}

void render_building_view(AppState& app, float width, float height) {
    begin_card("##building_view_card", ImVec2(width, height));

    // -------------------------------------------------------------------------
    // Header & Display Toggles
    // -------------------------------------------------------------------------
    if (g_fonts.title) PushFont(g_fonts.title);
    Text("Circular Building (N = %d Rooms)", app.num_rooms);
    if (g_fonts.title) PopFont();

    SameLine(width - 520.0f);
    Checkbox("Flux arrows", &app.show_flux_arrows);
    SameLine(width - 410.0f);
    Checkbox("Heater glow", &app.show_heaters);
    SameLine(width - 300.0f);
    Checkbox("Extreme bounds", &app.show_extreme_bounds);
    SameLine(width - 170.0f);
    Checkbox("Route bundle", &app.show_bundle);
    SameLine(width - 60.0f);
    Checkbox("Chart", &app.show_temp_plot);

    Spacing();

    // -------------------------------------------------------------------------
    // Canvas Geometry Setup
    // -------------------------------------------------------------------------
    ImVec2 canvas_pos = GetCursorScreenPos();
    ImDrawList* dl = GetWindowDrawList();
    const ImU32 card_border_col = GetColorU32(card_border);

    float avail_w = GetContentRegionAvail().x;
    float avail_h = GetContentRegionAvail().y;

    float ring_h = app.show_temp_plot ? (avail_h * 0.62f) : avail_h;
    float plot_h = app.show_temp_plot ? (avail_h * 0.38f - 10.0f) : 0.0f;

    // Center of the circular ring
    ImVec2 center = {canvas_pos.x + avail_w * 0.5f, canvas_pos.y + ring_h * 0.5f};
    float max_radius = std::min(avail_w * 0.45f, ring_h * 0.46f);
    float r_out = max_radius;
    float r_in  = max_radius * 0.52f;
    float r_mid = (r_out + r_in) * 0.5f;

    ImVec2 mouse_pos = GetIO().MousePos;
    int hovered_room = -1;

    // -------------------------------------------------------------------------
    // 1. Draw Central Courtyard (Outside Environment & Weather Station)
    // -------------------------------------------------------------------------
    if (app.show_weather_center) {
        // Courtyard background circle
        dl->AddCircleFilled(center, r_in - 2.0f, IM_COL32(245, 247, 250, 255), 64);
        dl->AddCircle(center, r_in - 2.0f, card_border_col, 64, 1.5f);

        // Outside Weather Text
        char weather_buf[64];
        std::snprintf(weather_buf, sizeof(weather_buf), "Te: %.1f C", app.current_te);

        if (g_fonts.bold) PushFont(g_fonts.bold);
        ImVec2 text_sz = CalcTextSize(weather_buf);
        dl->AddText(ImVec2(center.x - text_sz.x * 0.5f, center.y - 18.0f), GetColorU32(text_primary), weather_buf);
        if (g_fonts.bold) PopFont();

        const char* mode_name = app.weather_strategy ? app.weather_strategy->get_name() : "Nominal (-1.0 C)";

        if (g_fonts.small) PushFont(g_fonts.small);
        ImVec2 sub_sz = CalcTextSize(mode_name);
        dl->AddText(ImVec2(center.x - sub_sz.x * 0.5f, center.y + 2.0f), GetColorU32(text_muted), mode_name);

        char heat_buf[64];
        std::snprintf(heat_buf, sizeof(heat_buf), "Active: %d / %d", app.active_heaters_count, app.num_rooms);
        ImVec2 heat_sz = CalcTextSize(heat_buf);
        dl->AddText(ImVec2(center.x - heat_sz.x * 0.5f, center.y + 18.0f), GetColorU32(primary_blue), heat_buf);
        if (g_fonts.small) PopFont();
    }

    // -------------------------------------------------------------------------
    // 2. Draw Circular Ring of N Rooms
    // -------------------------------------------------------------------------
    const int N = app.num_rooms;
    const float angle_step = 2.0f * static_cast<float>(M_PI) / N;

    if (app.room_temps.size() != static_cast<size_t>(N)) {
        reset_room_temperatures(app);
    }

    float m_dx = mouse_pos.x - center.x;
    float m_dy = mouse_pos.y - center.y;
    float m_dist = std::hypot(m_dx, m_dy);
    float m_angle = std::atan2(m_dy, m_dx);
    if (m_angle < 0.0f) m_angle += 2.0f * static_cast<float>(M_PI);

    for (int i = 0; i < N; ++i) {
        float theta0 = i * angle_step;
        float theta1 = (i + 1) * angle_step;

        // Check if mouse hovers room i
        if (m_dist >= r_in && m_dist <= r_out && m_angle >= theta0 && m_angle < theta1) {
            hovered_room = i;
        }

        // Subdivide room arc for smooth rendering
        const int arc_subdiv = 12;
        std::vector<ImVec2> poly_pts;
        poly_pts.reserve(arc_subdiv * 2 + 2);

        for (int k = 0; k <= arc_subdiv; ++k) {
            float th = theta0 + (theta1 - theta0) * (static_cast<float>(k) / arc_subdiv);
            poly_pts.push_back({center.x + r_out * std::cos(th), center.y + r_out * std::sin(th)});
        }
        for (int k = arc_subdiv; k >= 0; --k) {
            float th = theta0 + (theta1 - theta0) * (static_cast<float>(k) / arc_subdiv);
            poly_pts.push_back({center.x + r_in * std::cos(th), center.y + r_in * std::sin(th)});
        }

        // Room thermal fill color
        double t_val = app.room_temps[i];
        ImU32 fill_col = get_temperature_color(t_val, (hovered_room == i) ? 0.95f : 0.85f);
        dl->AddConvexPolyFilled(poly_pts.data(), static_cast<int>(poly_pts.size()), fill_col);

        // Room border outline
        ImU32 border_col = (hovered_room == i) ? IM_COL32(30, 30, 40, 255) : IM_COL32(255, 255, 255, 200);
        float border_thick = (hovered_room == i) ? 2.5f : 1.2f;
        dl->AddPolyline(poly_pts.data(), static_cast<int>(poly_pts.size()), border_col, ImDrawFlags_Closed, border_thick);

        // Active heater indicator (amber outer arc glow)
        if (app.show_heaters && i < static_cast<int>(app.room_heaters.size()) && app.room_heaters[i] > 0.0) {
            float h_thick = (app.room_heaters[i] >= 0.5) ? 4.5f : 2.5f;
            std::vector<ImVec2> heater_arc;
            for (int k = 0; k <= arc_subdiv; ++k) {
                float th = theta0 + (theta1 - theta0) * (static_cast<float>(k) / arc_subdiv);
                heater_arc.push_back({center.x + (r_out + 1.5f) * std::cos(th), center.y + (r_out + 1.5f) * std::sin(th)});
            }
            dl->AddPolyline(heater_arc.data(), static_cast<int>(heater_arc.size()), col_heater_active, 0, h_thick);
        }

        // Room label at centroid
        if (app.show_labels && N <= 32) {
            float theta_mid = (theta0 + theta1) * 0.5f;
            ImVec2 lbl_pos = {center.x + r_mid * std::cos(theta_mid), center.y + r_mid * std::sin(theta_mid)};

            char id_str[16];
            std::snprintf(id_str, sizeof(id_str), "R%d", i);

            char temp_str[16];
            std::snprintf(temp_str, sizeof(temp_str), "%.1f", t_val);

            if (g_fonts.small) PushFont(g_fonts.small);
            ImVec2 id_sz = CalcTextSize(id_str);
            ImVec2 t_sz  = CalcTextSize(temp_str);

            dl->AddText(ImVec2(lbl_pos.x - id_sz.x * 0.5f, lbl_pos.y - id_sz.y - 1.0f), IM_COL32(20, 25, 35, 255), id_str);
            dl->AddText(ImVec2(lbl_pos.x - t_sz.x * 0.5f, lbl_pos.y + 1.0f), IM_COL32(10, 15, 25, 255), temp_str);
            if (g_fonts.small) PopFont();
        }

        // Radial dividing wall
        ImVec2 wall_start = {center.x + r_in * std::cos(theta0), center.y + r_in * std::sin(theta0)};
        ImVec2 wall_end   = {center.x + r_out * std::cos(theta0), center.y + r_out * std::sin(theta0)};
        dl->AddLine(wall_start, wall_end, IM_COL32(200, 204, 212, 255), 1.5f);

        // Heat conduction flux arrows across interior walls (alpha = 0.45)
        if (app.show_flux_arrows && N <= 32) {
            int left_idx = (i - 1 + N) % N;
            double t_left = app.room_temps[left_idx];
            double flux = app.alpha * (t_left - t_val);

            if (std::abs(flux) > 0.01) {
                float wall_mid_x = (wall_start.x + wall_end.x) * 0.5f;
                float wall_mid_y = (wall_start.y + wall_end.y) * 0.5f;

                float perp_x = -std::sin(theta0);
                float perp_y = std::cos(theta0);

                float arrow_len = std::clamp(static_cast<float>(std::abs(flux) * 40.0), 8.0f, 26.0f);
                float dir = (flux > 0.0) ? 1.0f : -1.0f;

                ImVec2 a_start = {wall_mid_x - dir * arrow_len * 0.5f * perp_x, wall_mid_y - dir * arrow_len * 0.5f * perp_y};
                ImVec2 a_end   = {wall_mid_x + dir * arrow_len * 0.5f * perp_x, wall_mid_y + dir * arrow_len * 0.5f * perp_y};

                ImU32 arrow_col = IM_COL32(230, 126, 34, 220);
                draw_arrow(dl, a_start, a_end, arrow_col, 2.0f, 5.0f);
            }
        }
    }

    dl->AddCircle(center, r_out, card_border_col, 96, 1.5f);
    dl->AddCircle(center, r_in,  card_border_col, 96, 1.5f);

    // -------------------------------------------------------------------------
    // 3. Hover Tooltip for Selected Room
    // -------------------------------------------------------------------------
    if (hovered_room >= 0 && hovered_room < N) {
        int i = hovered_room;
        int left_i = (i - 1 + N) % N;
        int right_i = (i + 1) % N;

        double t_i = app.room_temps[i];
        double t_l = app.room_temps[left_i];
        double t_r = app.room_temps[right_i];
        double u_i = (i < static_cast<int>(app.room_heaters.size())) ? app.room_heaters[i] : 0.0;
        int cell_id = UniformGrid1D::quantize(t_i);

        double q_loss       = app.beta * (t_i - app.current_te);
        double q_cond_left  = app.alpha * (t_i - t_l);
        double q_cond_right = app.alpha * (t_i - t_r);

        BeginTooltip();
        if (g_fonts.bold) PushFont(g_fonts.bold);
        Text("Room %d Details (Slide 15 Model)", i);
        if (g_fonts.bold) PopFont();
        Separator();

        Text("Temperature:   %.3f C %s", t_i, (t_i >= SAFE_MIN && t_i <= SAFE_MAX) ? "[COMFORT SAFE]" : "[ALERT]");
        Text("Discrete Cell: Cell %d / %d (%.1f .. %.1f C)", cell_id, ND_CELLS,
             SAFE_MIN + cell_id * CELL_W, SAFE_MIN + (cell_id + 1) * CELL_W);
        Text("Heater Input:  u = %.1f (%s)", u_i, (u_i >= 0.5) ? "High (0.6)" : (u_i > 0.1 ? "Medium (0.3)" : "OFF (0.0)"));
        Separator();
        Text("Heat Loss to Outside:   %.4f K/step", q_loss);
        Text("Conduction to Left R%d:  %.4f K/step", left_i, q_cond_left);
        Text("Conduction to Right R%d: %.4f K/step", right_i, q_cond_right);
        EndTooltip();

        if (IsMouseClicked(0)) {
            app.selected_room = i;
        }
    }

    // -------------------------------------------------------------------------
    // 4. Live Multi-Room Rolling Temperature Strip Chart
    // -------------------------------------------------------------------------
    if (app.show_temp_plot) {
        ImVec2 plot_pos = {canvas_pos.x, canvas_pos.y + ring_h + 8.0f};
        ImVec2 plot_size = {avail_w, plot_h};

        dl->AddRectFilled(plot_pos, ImVec2(plot_pos.x + plot_size.x, plot_pos.y + plot_size.y),
                          IM_COL32(252, 252, 254, 255), 6.0f);
        dl->AddRect(plot_pos, ImVec2(plot_pos.x + plot_size.x, plot_pos.y + plot_size.y),
                    card_border_col, 6.0f, 0, 1.0f);

        const float p_left   = plot_pos.x + 55.0f;
        const float p_right  = plot_pos.x + plot_size.x - 20.0f;
        const float p_top    = plot_pos.y + 18.0f;
        const float p_bottom = plot_pos.y + plot_size.y - 25.0f;

        // Plot display range: 14.0 C to 24.0 C
        constexpr float T_PLOT_MIN = 14.0f;
        constexpr float T_PLOT_MAX = 24.0f;

        auto temp_to_y = [&](float temp) -> float {
            float frac = (temp - T_PLOT_MIN) / (T_PLOT_MAX - T_PLOT_MIN);
            return p_bottom - frac * (p_bottom - p_top);
        };

        // Comfort safe band [17.0, 22.0] C from Slide 16 shaded green
        float y_safe_top = temp_to_y(static_cast<float>(SAFE_MAX)); // 22.0 C
        float y_safe_bot = temp_to_y(static_cast<float>(SAFE_MIN)); // 17.0 C

        dl->AddRectFilled(ImVec2(p_left, y_safe_top), ImVec2(p_right, y_safe_bot),
                          IM_COL32(46, 204, 113, 35));
        dl->AddLine(ImVec2(p_left, y_safe_top), ImVec2(p_right, y_safe_top),
                    IM_COL32(46, 204, 113, 160), 1.0f);
        dl->AddLine(ImVec2(p_left, y_safe_bot), ImVec2(p_right, y_safe_bot),
                    IM_COL32(46, 204, 113, 160), 1.0f);

        // Y-axis labels
        if (g_fonts.small) PushFont(g_fonts.small);
        dl->AddText(ImVec2(plot_pos.x + 6.0f, y_safe_top - 6.0f), IM_COL32(46, 170, 90, 255), "22.0 C (Max)");
        dl->AddText(ImVec2(plot_pos.x + 6.0f, y_safe_bot - 6.0f), IM_COL32(46, 170, 90, 255), "17.0 C (Min)");
        dl->AddText(ImVec2(plot_pos.x + 6.0f, temp_to_y(24.0f) - 6.0f), GetColorU32(text_muted), "24.0 C");
        dl->AddText(ImVec2(plot_pos.x + 6.0f, temp_to_y(14.0f) - 6.0f), GetColorU32(text_muted), "14.0 C");
        if (g_fonts.small) PopFont();

        // ---------------------------------------------------------------------
        // Render Monte Carlo Bundle Overlay (50 runs)
        // ---------------------------------------------------------------------
        if (app.show_bundle && app.bundle_ready && !app.trajectory_bundle.empty()) {
            for (const auto& b : app.trajectory_bundle) {
                if (b.times.size() < 2) continue;
                size_t num_pts = b.times.size();
                float b_t0 = b.times.front();
                float b_t1 = b.times.back();
                if (b_t1 <= b_t0) continue;

                std::vector<ImVec2> b_min_pts;
                std::vector<ImVec2> b_max_pts;
                b_min_pts.reserve(num_pts);
                b_max_pts.reserve(num_pts);

                for (size_t k = 0; k < num_pts; ++k) {
                    float fx = (b.times[k] - b_t0) / (b_t1 - b_t0);
                    float sx = p_left + fx * (p_right - p_left);
                    b_min_pts.push_back({sx, temp_to_y(b.min_temps[k])});
                    b_max_pts.push_back({sx, temp_to_y(b.max_temps[k])});
                }

                ImU32 b_col = b.all_safe ? IM_COL32(46, 204, 113, 30) : IM_COL32(231, 76, 60, 40);
                dl->AddPolyline(b_min_pts.data(), static_cast<int>(b_min_pts.size()), b_col, 0, 1.0f);
                dl->AddPolyline(b_max_pts.data(), static_cast<int>(b_max_pts.size()), b_col, 0, 1.0f);
            }
        }

        // ---------------------------------------------------------------------
        // Render Kamke-Muller Extreme Trajectory Bounds
        // ---------------------------------------------------------------------
        if (app.show_extreme_bounds && !app.extreme_lower.empty()) {
            size_t n_ext = app.extreme_lower.size();
            std::vector<ImVec2> ext_lo_pts;
            std::vector<ImVec2> ext_hi_pts;
            ext_lo_pts.reserve(n_ext);
            ext_hi_pts.reserve(n_ext);

            for (size_t k = 0; k < n_ext; k += 10) {
                float fx = static_cast<float>(k) / static_cast<float>(n_ext);
                float sx = p_left + fx * (p_right - p_left);
                ext_lo_pts.push_back({sx, temp_to_y(static_cast<float>(app.extreme_lower[k]))});
                ext_hi_pts.push_back({sx, temp_to_y(static_cast<float>(app.extreme_upper[k]))});
            }

            ImU32 ext_col = IM_COL32(142, 68, 173, 160);
            dl->AddPolyline(ext_lo_pts.data(), static_cast<int>(ext_lo_pts.size()), ext_col, 0, 1.5f);
            dl->AddPolyline(ext_hi_pts.data(), static_cast<int>(ext_hi_pts.size()), ext_col, 0, 1.5f);
        }

        // ---------------------------------------------------------------------
        // Render Rolling History Trajectories (Min, Max, Avg, Outside Te)
        // ---------------------------------------------------------------------
        if (app.history.size() >= 2) {
            float t_start = app.history.front().time_sec;
            float t_end   = app.history.back().time_sec;
            float time_span = std::max(t_end - t_start, 1.0f);

            std::vector<ImVec2> pts_min;
            std::vector<ImVec2> pts_max;
            std::vector<ImVec2> pts_avg;
            std::vector<ImVec2> pts_te;

            pts_min.reserve(app.history.size());
            pts_max.reserve(app.history.size());
            pts_avg.reserve(app.history.size());
            pts_te.reserve(app.history.size());

            for (const auto& pt : app.history) {
                float fx = (pt.time_sec - t_start) / time_span;
                float sx = p_left + fx * (p_right - p_left);

                pts_min.push_back({sx, temp_to_y(pt.min_T)});
                pts_max.push_back({sx, temp_to_y(pt.max_T)});
                pts_avg.push_back({sx, temp_to_y(pt.avg_T)});
                pts_te.push_back({sx, temp_to_y(pt.Te)});
            }

            // Outside Weather Te line (dashed cyan)
            dl->AddPolyline(pts_te.data(), static_cast<int>(pts_te.size()), IM_COL32(41, 128, 185, 200), 0, 1.5f);

            // Shaded envelope between min and max room temperature
            std::vector<ImVec2> env_pts;
            env_pts.reserve(pts_min.size() + pts_max.size());
            for (const auto& p : pts_min) env_pts.push_back(p);
            for (auto it = pts_max.rbegin(); it != pts_max.rend(); ++it) env_pts.push_back(*it);
            dl->AddConvexPolyFilled(env_pts.data(), static_cast<int>(env_pts.size()), IM_COL32(46, 204, 113, 60));

            dl->AddPolyline(pts_min.data(), static_cast<int>(pts_min.size()), IM_COL32(39, 174, 96, 255), 0, 1.5f);
            dl->AddPolyline(pts_max.data(), static_cast<int>(pts_max.size()), IM_COL32(39, 174, 96, 255), 0, 1.5f);
            dl->AddPolyline(pts_avg.data(), static_cast<int>(pts_avg.size()), IM_COL32(22, 160, 133, 255), 0, 2.0f);
        }

        // Plot legends
        if (g_fonts.small) PushFont(g_fonts.small);
        float leg_x = p_left + 10.0f;
        float leg_y = p_top + 4.0f;

        dl->AddRectFilled(ImVec2(leg_x, leg_y + 2.0f), ImVec2(leg_x + 12.0f, leg_y + 8.0f), IM_COL32(46, 204, 113, 100));
        dl->AddText(ImVec2(leg_x + 16.0f, leg_y), GetColorU32(text_primary), "All Rooms [Tmin, Tmax]");

        leg_x += 160.0f;
        dl->AddLine(ImVec2(leg_x, leg_y + 6.0f), ImVec2(leg_x + 12.0f, leg_y + 6.0f), IM_COL32(41, 128, 185, 255), 2.0f);
        dl->AddText(ImVec2(leg_x + 16.0f, leg_y), GetColorU32(text_primary), "Outside Te(t)");

        if (app.show_extreme_bounds) {
            leg_x += 120.0f;
            dl->AddLine(ImVec2(leg_x, leg_y + 6.0f), ImVec2(leg_x + 12.0f, leg_y + 6.0f), IM_COL32(142, 68, 173, 255), 2.0f);
            dl->AddText(ImVec2(leg_x + 16.0f, leg_y), GetColorU32(text_primary), "Kamke-Muller Bounds");
        }

        if (app.show_bundle) {
            leg_x += 160.0f;
            dl->AddLine(ImVec2(leg_x, leg_y + 6.0f), ImVec2(leg_x + 12.0f, leg_y + 6.0f), IM_COL32(46, 204, 113, 200), 1.5f);
            dl->AddText(ImVec2(leg_x + 16.0f, leg_y), GetColorU32(text_primary), "Bundle (50 runs)");
        }
        if (g_fonts.small) PopFont();
    }

    Dummy(ImVec2(avail_w, avail_h));
    end_card();
}

} // namespace circular::view
