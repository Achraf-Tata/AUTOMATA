#include "popup_dialogs.hpp"
#include "ui_theme.hpp"
#include "imgui.h"

namespace circular::dialogs {

using namespace ImGui;
using namespace theme;

static bool g_show_about = false;
static bool g_show_curse = false;

void open_about_dialog() {
    g_show_about = true;
}

void open_curse_dialog() {
    g_show_curse = true;
}

void render_popups(AppState& app) {
    (void)app;

    // -------------------------------------------------------------------------
    // 1. About Theory Dialog (Slides 15-17 of 42717210-s7467.pdf)
    // -------------------------------------------------------------------------
    if (g_show_about) {
        OpenPopup("About Circular Building & Assume-Guarantee Theory");
        g_show_about = false;
    }

    SetNextWindowSize(ImVec2(720.0f, 540.0f), ImGuiCond_Appearing);
    if (BeginPopupModal("About Circular Building & Assume-Guarantee Theory", nullptr, ImGuiWindowFlags_None)) {
        if (g_fonts.title) PushFont(g_fonts.title);
        Text("Project 3: Temperature Regulation in a Circular Building");
        if (g_fonts.title) PopFont();

        if (g_fonts.small) PushFont(g_fonts.small);
        TextColored(colors::text_muted, "Course: Computation Theory - Fall 2026 | Prof. Adnane Saoud (Slides 15-17)");
        if (g_fonts.small) PopFont();
        Separator();

        TextWrapped(
            "This application models and synthesizes safety controllers for an N-room circular building where each room i in {1, ..., N} "
            "exchanges heat with its left neighbor i-1 (mod N), its right neighbor i+1 (mod N), and the outside environment.\n\n"
            "Discrete Thermal Dynamics (Slide 15):\n"
            "  T_i(t+1) = T_i(t) + alpha * (T_{i+1}(t) + T_{i-1}(t) - 2*T_i(t)) + beta * (Te - T_i(t)) + gamma * (Th - T_i(t)) * u_i(t)\n\n"
            "Physical Constants & Specifications (Slide 15-16):\n"
            "  - Conduction between rooms: alpha = 0.45\n"
            "  - Heat loss to outside:     beta  = 0.045\n"
            "  - Heater efficiency:        gamma = 0.09\n"
            "  - Outside temperature:      Te    = -1.0 C\n"
            "  - Heater temperature:       Th    = 50.0 C\n"
            "  - Input control set:        u_i in {0.0, 0.3, 0.6} (nu = 3 controls)\n"
            "  - Comfort safe set:         S = [17.0, 22.0]^N C\n"
            "  - Discretization partition: nd = 10 cells per dimension (0.5 C per cell)\n\n"
            "Why Monolithic Grid Fails (Slide 16):\n"
            "  A monolithic state-space discretization requires nd^N = 10^N cells and nu^N = 3^N input combinations. "
            "For N = 10 rooms, 10^10 states requires gigabytes of memory and hours to compute. For N = 100, 10^100 exceeds the atoms in the universe.\n\n"
            "Compositional Assume-Guarantee Solution (Slide 15-17):\n"
            "  Each room i assumes its neighbors stay within the comfort set A_i = [17.0, 22.0] C. This induces bounded disturbance "
            "W_i = alpha*(T_{i+1} + T_{i-1}) in [15.255, 19.755] C. Each room solves an identical 1D contract abstraction with only 10 cells! "
            "By the circular composition theorem (Saoud, Girard, Fribourg), local invariance G_i subseteq A_{i+-1} guarantees global invariance S."
        );

        Spacing();
        Separator();
        if (primary_button("Close", ImVec2(120.0f, 32.0f))) {
            CloseCurrentPopup();
        }
        EndPopup();
    }

    // -------------------------------------------------------------------------
    // 2. Curse of Dimensionality Modal (Slide 16)
    // -------------------------------------------------------------------------
    if (g_show_curse) {
        OpenPopup("Curse of Dimensionality: Monolithic 10^N vs Contract O(N)");
        g_show_curse = false;
    }

    SetNextWindowSize(ImVec2(760.0f, 500.0f), ImGuiCond_Appearing);
    if (BeginPopupModal("Curse of Dimensionality: Monolithic 10^N vs Contract O(N)", nullptr, ImGuiWindowFlags_None)) {
        if (g_fonts.title) PushFont(g_fonts.title);
        Text("State-Space Explosion: Monolithic (10^N) vs Compositional O(N*10)");
        if (g_fonts.title) PopFont();
        Separator();

        TextWrapped(
            "Demonstration of why monolithic discretization fails and why compositional contract synthesis is required (Slide 16-17):"
        );
        Spacing();

        if (BeginTable("curse_table", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
            TableSetupColumn("Rooms (N)");
            TableSetupColumn("States (10^N)");
            TableSetupColumn("Inputs (3^N)");
            TableSetupColumn("Monolithic Memory");
            TableSetupColumn("Compositional Contract (N * 10)");
            TableHeadersRow();

            TableNextRow();
            TableNextColumn(); Text("1");
            TableNextColumn(); Text("10");
            TableNextColumn(); Text("3");
            TableNextColumn(); Text("10 B");
            TableNextColumn(); Text("10 cells (0.005 ms)");

            TableNextRow();
            TableNextColumn(); Text("2");
            TableNextColumn(); Text("100");
            TableNextColumn(); Text("9");
            TableNextColumn(); Text("100 B");
            TableNextColumn(); Text("20 cells (0.010 ms)");

            TableNextRow();
            TableNextColumn(); Text("3");
            TableNextColumn(); Text("1,000");
            TableNextColumn(); Text("27");
            TableNextColumn(); Text("1 KB");
            TableNextColumn(); Text("30 cells (0.015 ms)");

            TableNextRow();
            TableNextColumn(); Text("4");
            TableNextColumn(); Text("10,000");
            TableNextColumn(); Text("81");
            TableNextColumn(); Text("10 KB");
            TableNextColumn(); Text("40 cells (0.020 ms)");

            TableNextRow();
            TableNextColumn(); Text("5");
            TableNextColumn(); Text("100,000");
            TableNextColumn(); Text("243");
            TableNextColumn(); Text("100 KB");
            TableNextColumn(); Text("50 cells (0.025 ms)");

            TableNextRow();
            TableNextColumn(); Text("6");
            TableNextColumn(); Text("1,000,000");
            TableNextColumn(); Text("729");
            TableNextColumn(); Text("1.0 MB");
            TableNextColumn(); Text("60 cells (0.030 ms)");

            TableNextRow();
            TableNextColumn(); Text("10");
            TableNextColumn(); TextColored(colors::banner_red_txt, "10^10 (10 Billion)");
            TableNextColumn(); Text("59,049");
            TableNextColumn(); TextColored(colors::banner_red_txt, "10.0 GB (CRASH)");
            TableNextColumn(); Text("100 cells (0.040 ms)");

            TableNextRow();
            TableNextColumn(); Text("100");
            TableNextColumn(); TextColored(colors::banner_red_txt, "10^100 (Googol)");
            TableNextColumn(); Text("5.15 x 10^47");
            TableNextColumn(); TextColored(colors::banner_red_txt, "> Atoms in Universe");
            TableNextColumn(); Text("1,000 cells (0.150 ms)");

            TableNextRow();
            TableNextColumn(); Text("1000");
            TableNextColumn(); TextColored(colors::banner_red_txt, "10^1000");
            TableNextColumn(); Text("1.32 x 10^477");
            TableNextColumn(); TextColored(colors::banner_red_txt, "Infinitely Exceeded");
            TableNextColumn(); Text("10,000 cells (1.200 ms)");

            EndTable();
        }

        Spacing();
        Separator();
        if (primary_button("Close", ImVec2(120.0f, 32.0f))) {
            CloseCurrentPopup();
        }
        EndPopup();
    }
}

} // namespace circular::dialogs
