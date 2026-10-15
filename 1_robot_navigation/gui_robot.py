# -*- coding: utf-8 -*-
"""
Mobile Robot Navigation — Standalone GUI with Direct Map Drag & Resize
======================================================================
Run with:  python gui_robot.py
Requires:  matplotlib, numpy, numba
Depends:   symbolic_control.py (same folder)

Direct Map Interaction:
- Drag Robot to reposition starting point
- Drag inside Obstacle (R4), Target (R3), or Waypoints (R1, R2) to move them
- Drag any corner of Obstacle (R4) to resize its width and height directly on map
- Click on empty space to move robot
- Warning dialog when map is modified before running
- Multi-trajectory disturbance simulation
"""

import sys
import math
import copy
import threading
import numpy as np
import tkinter as tk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.patches as patches

# Local import
import os
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
import symbolic_control as sc

# Theme Palette (GitHub Dark)
BG      = "#0d1117"
SURFACE = "#161b22"
BORDER  = "#30363d"
TEXT    = "#e6edf3"
MUTED   = "#8b949e"
GREEN   = "#3fb950"
BLUE    = "#58a6ff"
AMBER   = "#d29922"
RED     = "#f85149"
CYAN    = "#388bfd"
PURPLE  = "#bc8cff"

# Default Region colors
R1_COLOR = "#d29922"  # Amber
R2_COLOR = "#388bfd"  # Blue
R3_COLOR = "#3fb950"  # Green
R4_COLOR = "#f85149"  # Red

# Default Benchmark Layout (Antoine Girard et al., 2024)
PAPER_LAYOUT = {
    'R1': {'x': (4.0, 5.0), 'y': (8.5, 9.5), 'color': R1_COLOR, 'label': 'Waymark 1 (R1)'},
    'R2': {'x': (8.5, 9.5), 'y': (2.0, 3.0), 'color': R2_COLOR, 'label': 'Waymark 2 (R2)'},
    'R3': {'x': (2.0, 3.0), 'y': (0.5, 1.5), 'color': R3_COLOR, 'label': 'Final Goal (R3)'},
    'R4': {'x': (3.0, 7.0), 'y': (3.0, 7.0), 'color': R4_COLOR, 'label': 'Obstacle (R4)'}
}


class RobotGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("Mobile Robot Navigation - Reachability Controller")
        root.configure(bg=BG)
        root.resizable(True, True)
        root.minsize(1120, 760)

        # State
        self.regions = copy.deepcopy(PAPER_LAYOUT)
        self.resolution_choice = "fine"  # "fast" or "fine"
        self.map_modified = False
        self._ignore_modified_warning = False
        self._auto_run_after = None

        # Mouse Interaction Drag State
        self.drag_mode = None  # None, ('move', 'robot'), ('move', key), ('resize', 'R4', corner)
        self.drag_offset = (0.0, 0.0)

        # Backend Controller
        self.ctrl = sc.SymbolicController()
        loaded = self.ctrl.load_cached_controller()
        if not loaded:
            self.ctrl.synthesize()

        # Simulation State
        self.x = np.array([1.0, 2.0, 0.0], dtype=np.float64)
        self.psi = 1
        self.status = "running"
        self.step_count = 0
        self.traj_x = [self.x[0]]
        self.traj_y = [self.x[1]]
        self.last_u = np.array([0.0, 0.0])
        self.last_w = np.array([0.0, 0.0, 0.0])

        # Monte Carlo Trajectory Bundle
        self.monte_carlo_trajectories = []
        self.monte_carlo_stats = None

        self._playing = False
        self._timer_id = None
        self._is_synthesizing = False
        self._noise_enabled = tk.BooleanVar(value=True)

        # Build Layout
        self._build_header()
        self._build_main()
        self._build_footer()

        # Initial Render
        self._redraw_arena()
        self._update_telemetry()
        self._log_write("Initialized. Direct click-and-drag active for all elements.", "info")

    # ─────────────────────────────────────────────────────────
    # Layout Builders
    # ─────────────────────────────────────────────────────────
    def _build_header(self):
        bar = tk.Frame(self.root, bg=SURFACE, height=48)
        bar.pack(fill="x", side="top")
        bar.pack_propagate(False)

        title_frame = tk.Frame(bar, bg=SURFACE)
        title_frame.pack(side="left", padx=16, pady=8)

        tk.Label(title_frame, text="Mobile Robot Navigation", bg=SURFACE, fg=TEXT,
                 font=("Segoe UI", 13, "bold")).pack(side="left")
        tk.Label(title_frame, text="   -   Direct Map Interaction and Reachability Controller",
                 bg=SURFACE, fg=MUTED, font=("Segoe UI", 9)).pack(side="left")

    def _build_main(self):
        pane = tk.PanedWindow(self.root, orient="horizontal", bg=BG, sashwidth=4, sashrelief="flat")
        pane.pack(fill="both", expand=True, padx=8, pady=8)

        # Left: Arena Plot
        left = tk.Frame(pane, bg=SURFACE, bd=1, relief="flat")
        pane.add(left, minsize=660, stretch="always")

        top_info = tk.Frame(left, bg=SURFACE)
        top_info.pack(fill="x", padx=12, pady=(8, 2))

        tk.Label(top_info, text="Environment Arena  [0, 10] m x [0, 10] m",
                 bg=SURFACE, fg=MUTED, font=("Segoe UI", 9, "bold")).pack(side="left")
        tk.Label(top_info, text="Drag elements to move. Drag corners of R4 to resize.",
                 bg=SURFACE, fg=BLUE, font=("Segoe UI", 8)).pack(side="right")

        # Matplotlib figure
        self.fig = Figure(figsize=(6.5, 6.0), dpi=96, facecolor="#010409")
        self.ax = self.fig.add_subplot(111, facecolor="#010409")
        self.fig.tight_layout(pad=2.0)

        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=6, pady=2)

        # Mouse direct manipulation bindings
        self.canvas.mpl_connect("button_press_event", self._on_mouse_down)
        self.canvas.mpl_connect("motion_notify_event", self._on_mouse_move)
        self.canvas.mpl_connect("button_release_event", self._on_mouse_up)

        # Bottom Simulation Controls bar
        bb = tk.Frame(left, bg=SURFACE)
        bb.pack(fill="x", padx=8, pady=(4, 8))

        self._btn_play = self._btn(bb, "Play", self._toggle_play, SURFACE, GREEN)
        self._btn(bb, "Step", self._on_click_step, SURFACE, BLUE)
        self._btn(bb, "Run to Target", self._on_click_run_to_goal, SURFACE, AMBER)
        self._btn(bb, "Reset", self._reset_run, SURFACE, MUTED)
        self._btn(bb, "50 Noise Runs", self._on_click_monte_carlo, SURFACE, RED)
        self._btn(bb, "Clear", self._clear_monte_carlo, SURFACE, MUTED)

        # Disturbance toggle
        cb_noise = tk.Checkbutton(bb, text="Disturbance (+/-0.05)", variable=self._noise_enabled,
                                  bg=SURFACE, fg=TEXT, selectcolor="#010409",
                                  activebackground=SURFACE, activeforeground=TEXT,
                                  font=("Segoe UI", 8))
        cb_noise.pack(side="right", padx=6)

        # Right: Sidebar
        right = tk.Frame(pane, bg=BG)
        pane.add(right, minsize=370, stretch="never")

        self._build_layout_card(right)
        self._build_elements_info_card(right)
        self._build_synthesis_card(right)
        self._build_telemetry_card(right)
        self._build_log_card(right)

    def _build_layout_card(self, parent):
        card = self._card(parent, "Map Layout and Starting Position")
        body = tk.Frame(card, bg=SURFACE)
        body.pack(fill="x", padx=10, pady=(2, 6))

        row1 = tk.Frame(body, bg=SURFACE)
        row1.pack(fill="x", pady=2)

        btn_paper = tk.Button(row1, text="Reset Layout",
                              command=self._reset_to_paper_layout,
                              bg="#161b22", fg=TEXT, activebackground="#21262d",
                              activeforeground=BLUE, bd=1, relief="solid",
                              font=("Segoe UI", 8, "bold"), padx=8, pady=4, cursor="hand2")
        btn_paper.pack(side="left", fill="x", expand=True, padx=(0, 2))

        btn_init = tk.Button(row1, text="Set Start (6.8, 9.0)",
                             command=self._set_paper_start,
                             bg="#161b22", fg=TEXT, activebackground="#21262d",
                             activeforeground=GREEN, bd=1, relief="solid",
                             font=("Segoe UI", 8, "bold"), padx=8, pady=4, cursor="hand2")
        btn_init.pack(side="left", fill="x", expand=True, padx=(2, 0))

        row2 = tk.Frame(body, bg=SURFACE)
        row2.pack(fill="x", pady=(4, 2))

        btn_mc = tk.Button(row2, text="Simulate 50 Noise Runs",
                           command=self._on_click_monte_carlo,
                           bg="#21262d", fg=RED, activebackground="#30363d",
                           activeforeground="#ffffff", bd=1, relief="solid",
                           font=("Segoe UI", 8, "bold"), padx=8, pady=5, cursor="hand2")
        btn_mc.pack(fill="x")

    def _build_elements_info_card(self, parent):
        card = self._card(parent, "Elements and Dimensions")
        body = tk.Frame(card, bg=SURFACE)
        body.pack(fill="x", padx=10, pady=(2, 6))

        self.lbl_elem_info = tk.Label(body,
            text="Obstacle R4: [3.0, 7.0] x [3.0, 7.0]  (4.0m x 4.0m)\nTarget R3:   [2.0, 3.0] x [0.5, 1.5]  (1.0m x 1.0m)\nDrag inside to move. Drag corners to resize.",
            bg=SURFACE, fg=MUTED, font=("Segoe UI Mono", 8), justify="left")
        self.lbl_elem_info.pack(anchor="w")

    def _build_synthesis_card(self, parent):
        card = self._card(parent, "Controller Computation")
        body = tk.Frame(card, bg=SURFACE)
        body.pack(fill="x", padx=10, pady=(2, 6))

        # Resolution selectors
        res_frame = tk.Frame(body, bg=SURFACE)
        res_frame.pack(fill="x", pady=2)

        self.btn_fast_res = tk.Button(res_frame, text="Fast Grid (60x60x20)",
                                      command=lambda: self._set_resolution("fast"),
                                      bg="#161b22", fg=MUTED, bd=1, relief="solid",
                                      font=("Segoe UI", 8), padx=6, pady=3, cursor="hand2")
        self.btn_fast_res.pack(side="left", fill="x", expand=True, padx=(0, 2))

        self.btn_fine_res = tk.Button(res_frame, text="Fine Grid (100x100x30)",
                                      command=lambda: self._set_resolution("fine"),
                                      bg="#21262d", fg=BLUE, bd=1, relief="solid",
                                      font=("Segoe UI", 8, "bold"), padx=6, pady=3, cursor="hand2")
        self.btn_fine_res.pack(side="left", fill="x", expand=True, padx=(2, 0))

        # Update controller button
        self._btn_syn = tk.Button(body, text="Update Controller",
                                  command=self._start_synthesis,
                                  bg="#21262d", fg=AMBER, activebackground="#30363d",
                                  activeforeground="#ffffff", bd=1, relief="solid",
                                  font=("Segoe UI", 9, "bold"), pady=6, cursor="hand2")
        self._btn_syn.pack(fill="x", pady=(6, 4))

        # Progress Bar Canvas
        self.prog_canvas = tk.Canvas(body, height=8, bg="#0d1117", highlightthickness=1,
                                     highlightbackground=BORDER, relief="flat")
        self.prog_canvas.pack(fill="x", pady=2)
        self.prog_rect = self.prog_canvas.create_rectangle(0, 0, 0, 8, fill=BLUE, width=0)

        self._syn_status = tk.Label(body, text="Controller ready.",
                                    bg=SURFACE, fg=GREEN, font=("Segoe UI", 8))
        self._syn_status.pack(anchor="w", pady=(2, 0))

    def _build_telemetry_card(self, parent):
        card = self._card(parent, "DFA Automaton and State")
        body = tk.Frame(card, bg=SURFACE)
        body.pack(fill="x", padx=10, pady=(2, 6))

        # DFA state badge list
        self.dfa_badges = {}
        states_info = [
            (1, "q1: Seeking R1 or R2", "(avoiding R4)"),
            (2, "q2: R1 Visited", "(heading to target R3)"),
            (3, "q3: R2 Visited", "(heading to target R3)"),
            (4, "q4: Target Reached", "(goal achieved)"),
            (5, "q5: Trap State", "(collision with obstacle)"),
        ]
        for s_idx, name, desc in states_info:
            row = tk.Frame(body, bg=SURFACE)
            row.pack(fill="x", pady=1)
            dot = tk.Label(row, text="[ ]", bg=SURFACE, fg=MUTED, font=("Segoe UI Mono", 8))
            dot.pack(side="left", padx=(0, 4))
            lbl = tk.Label(row, text=f"{name} {desc}", bg=SURFACE, fg=MUTED,
                           font=("Segoe UI", 8), anchor="w")
            lbl.pack(side="left", fill="x", expand=True)
            self.dfa_badges[s_idx] = (dot, lbl)

        # Telemetry grid
        grid = tk.Frame(body, bg=SURFACE)
        grid.pack(fill="x", pady=(4, 0))
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)

        self._t_pos = self._stat_box(grid, 0, 0, "Position (x1, x2)", "(1.00, 2.00) m")
        self._t_head = self._stat_box(grid, 0, 1, "Heading (theta)", "0.0 deg")
        self._t_ctrl = self._stat_box(grid, 1, 0, "Control (v, w)", "v=0.00, w=0.00")
        self._t_step = self._stat_box(grid, 1, 1, "Time Step (t)", "0 s")

    def _stat_box(self, parent, r, c, label, val):
        box = tk.Frame(parent, bg="#0d1117", bd=1, relief="solid")
        box.grid(row=r, column=c, padx=2, pady=2, sticky="nsew")
        tk.Label(box, text=label, bg="#0d1117", fg=MUTED, font=("Segoe UI", 7, "bold")).pack(anchor="w", padx=4, pady=(2, 0))
        val_lbl = tk.Label(box, text=val, bg="#0d1117", fg=TEXT, font=("Segoe UI Mono", 8, "bold"))
        val_lbl.pack(anchor="w", padx=4, pady=(0, 2))
        return val_lbl

    def _build_log_card(self, parent):
        card = self._card(parent, "Execution Log")
        self.log_text = tk.Text(card, bg="#010409", fg=TEXT, height=5, bd=0,
                                font=("Segoe UI Mono", 8), state="disabled", wrap="word")
        self.log_text.pack(fill="both", expand=True, padx=8, pady=(2, 6))
        self.log_text.tag_config("info", foreground=MUTED)
        self.log_text.tag_config("warn", foreground=AMBER)
        self.log_text.tag_config("succ", foreground=GREEN)
        self.log_text.tag_config("fail", foreground=RED)

    def _build_footer(self):
        foot = tk.Frame(self.root, bg=SURFACE, height=24)
        foot.pack(fill="x", side="bottom")
        foot.pack_propagate(False)

        self._foot_lbl = tk.Label(foot, text="Discrete Unicycle Model - Antoine Girard, Pierre-Jean Meyer, Adnane Saoud (2024)",
                                  bg=SURFACE, fg=MUTED, font=("Segoe UI", 8))
        self._foot_lbl.pack(side="left", padx=12)

        tk.Label(foot, text="Direct canvas mouse manipulation - Fixed-point reachability",
                 bg=SURFACE, fg=CYAN, font=("Segoe UI", 8)).pack(side="right", padx=12)

    # ─────────────────────────────────────────────────────────
    # Matplotlib Rendering & Handles
    # ─────────────────────────────────────────────────────────
    def _redraw_arena(self):
        self.ax.clear()
        self.ax.set_facecolor("#010409")
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(0, 10)
        self.ax.set_aspect("equal")

        # Styling
        self.ax.tick_params(colors=MUTED, labelsize=8)
        for spine in self.ax.spines.values():
            spine.set_color(BORDER)
        self.ax.grid(True, color="#21262d", linestyle="--", linewidth=0.5, alpha=0.6)

        # 1. Draw Regions (Waypoints, Target, Obstacles)
        for key, reg in self.regions.items():
            rx, ry = reg['x'], reg['y']
            w = rx[1] - rx[0]
            h = ry[1] - ry[0]
            col = reg.get('color', RED)
            is_obs = (key == 'R4' or key.startswith('Obs'))

            rect = patches.Rectangle((rx[0], ry[0]), w, h, linewidth=1.8,
                                     edgecolor=col, facecolor=col,
                                     alpha=0.35 if not is_obs else 0.5, zorder=2)
            self.ax.add_patch(rect)

            cx = rx[0] + w / 2.0
            cy = ry[0] + h / 2.0
            lbl_text = reg.get('label', key)
            self.ax.text(cx, cy, lbl_text, color=col, fontsize=8,
                         fontweight="bold", ha="center", va="center", zorder=3)

            # Draw corner resize handles on Obstacle R4
            if is_obs:
                handles = [
                    (rx[0], ry[0]), (rx[1], ry[0]),
                    (rx[0], ry[1]), (rx[1], ry[1])
                ]
                for hx, hy in handles:
                    self.ax.scatter(hx, hy, s=40, color="#ffffff", edgecolors=RED,
                                    linewidth=1.2, zorder=4)

        # 2. Monte Carlo Trajectory Bundle
        if self.monte_carlo_trajectories:
            for res in self.monte_carlo_trajectories:
                traj = res['trajectory']
                self.ax.plot(traj[:, 0], traj[:, 1], '-o', color='#f85149',
                             alpha=0.45, linewidth=1.2, markersize=3, zorder=5)

            # Mark starting state
            sx, sy = self.monte_carlo_stats['x0'][0], self.monte_carlo_stats['x0'][1]
            self.ax.plot(sx, sy, 's', color='#ffffff', markeredgecolor='#000000',
                         markersize=8, zorder=7, label=f"Start ({sx:.1f}, {sy:.1f})")

        # 3. Single Simulation Trajectory
        if len(self.traj_x) > 1 and not self.monte_carlo_trajectories:
            self.ax.plot(self.traj_x, self.traj_y, color=BLUE, linewidth=2.0, alpha=0.85, zorder=5)
            self.ax.scatter(self.traj_x[:-1], self.traj_y[:-1], color=BLUE, s=15, alpha=0.6, zorder=5)

        # 4. Start marker for single run
        if not self.monte_carlo_trajectories:
            self.ax.scatter(self.traj_x[0], self.traj_y[0], color="#a371f7",
                            s=60, edgecolors="white", linewidth=1.2, zorder=6, label="Start")

        # 5. Robot avatar & heading arrow
        rx, ry, rth = self.x[0], self.x[1], self.x[2]
        self.ax.scatter(rx, ry, color="#f0883e", s=90, edgecolors="white", linewidth=1.5, zorder=8)

        arrow_len = 0.5
        dx = arrow_len * math.cos(rth)
        dy = arrow_len * math.sin(rth)
        self.ax.arrow(rx, ry, dx, dy, head_width=0.18, head_length=0.15,
                      fc="white", ec="white", zorder=9)

        self.canvas.draw_idle()
        self._update_elements_info_label()

    def _update_elements_info_label(self):
        r4 = self.regions.get('R4', {}).get('x', (3, 7)), self.regions.get('R4', {}).get('y', (3, 7))
        w4 = r4[0][1] - r4[0][0]
        h4 = r4[1][1] - r4[1][0]
        r3 = self.regions.get('R3', {}).get('x', (2, 3)), self.regions.get('R3', {}).get('y', (0.5, 1.5))
        w3 = r3[0][1] - r3[0][0]
        h3 = r3[1][1] - r3[1][0]

        self.lbl_elem_info.config(
            text=f"Obstacle R4: [{r4[0][0]:.1f}, {r4[0][1]:.1f}] x [{r4[1][0]:.1f}, {r4[1][1]:.1f}] ({w4:.1f}m x {h4:.1f}m)\n"
                 f"Target R3:   [{r3[0][0]:.1f}, {r3[0][1]:.1f}] x [{r3[1][0]:.1f}, {r3[1][1]:.1f}] ({w3:.1f}m x {h3:.1f}m)\n"
                 f"Drag inside to move. Drag corners to resize."
        )

    # ─────────────────────────────────────────────────────────
    # Direct Mouse Drag & Resize Handlers
    # ─────────────────────────────────────────────────────────
    def _on_mouse_down(self, event):
        if event.xdata is None or event.ydata is None or event.inaxes != self.ax:
            return
        if self._playing:
            self._pause_playing()

        cx = float(event.xdata)
        cy = float(event.ydata)

        # 1. Check corner resize handles on Obstacle R4
        r4_x = self.regions['R4']['x']
        r4_y = self.regions['R4']['y']
        threshold = 0.45

        if (cx - r4_x[1])**2 + (cy - r4_y[1])**2 < threshold**2:
            self.drag_mode = ('resize', 'R4', 'top_right')
            return
        elif (cx - r4_x[0])**2 + (cy - r4_y[1])**2 < threshold**2:
            self.drag_mode = ('resize', 'R4', 'top_left')
            return
        elif (cx - r4_x[1])**2 + (cy - r4_y[0])**2 < threshold**2:
            self.drag_mode = ('resize', 'R4', 'bottom_right')
            return
        elif (cx - r4_x[0])**2 + (cy - r4_y[0])**2 < threshold**2:
            self.drag_mode = ('resize', 'R4', 'bottom_left')
            return

        # 2. Check if clicked on robot
        if (cx - self.x[0])**2 + (cy - self.x[1])**2 < 0.45**2:
            self.drag_mode = ('move', 'robot')
            return

        # 3. Check inside any region (move region)
        for key in ['R4', 'R3', 'R1', 'R2']:
            if key in self.regions:
                rx = self.regions[key]['x']
                ry = self.regions[key]['y']
                if rx[0] <= cx <= rx[1] and ry[0] <= cy <= ry[1]:
                    self.drag_mode = ('move', key)
                    self.drag_offset = (cx - rx[0], cy - ry[0])
                    return

        # 4. If clicked empty space: move robot directly there
        self.x[0] = np.clip(cx, 0.2, 9.8)
        self.x[1] = np.clip(cy, 0.2, 9.8)
        self._reset_run()
        self.drag_mode = ('move', 'robot')
        self._log_write(f"Robot placed at ({self.x[0]:.2f}, {self.x[1]:.2f}) m.", "info")

    def _on_mouse_move(self, event):
        if self.drag_mode is None or event.xdata is None or event.ydata is None or event.inaxes != self.ax:
            return

        cx = float(event.xdata)
        cy = float(event.ydata)
        action = self.drag_mode[0]

        if action == 'move':
            target = self.drag_mode[1]
            if target == 'robot':
                self.x[0] = np.clip(cx, 0.2, 9.8)
                self.x[1] = np.clip(cy, 0.2, 9.8)
                self.traj_x = [self.x[0]]
                self.traj_y = [self.x[1]]
                self._redraw_arena()
                self._update_telemetry()
            else:
                # Move region
                reg = self.regions[target]
                w = reg['x'][1] - reg['x'][0]
                h = reg['y'][1] - reg['y'][0]
                new_x0 = np.clip(cx - self.drag_offset[0], 0.0, 10.0 - w)
                new_y0 = np.clip(cy - self.drag_offset[1], 0.0, 10.0 - h)
                reg['x'] = (new_x0, new_x0 + w)
                reg['y'] = (new_y0, new_y0 + h)
                self._redraw_arena()

        elif action == 'resize':
            target = self.drag_mode[1]
            corner = self.drag_mode[2]
            reg = self.regions[target]
            x0, x1 = reg['x']
            y0, y1 = reg['y']
            min_size = 0.8

            if corner == 'top_right':
                reg['x'] = (x0, np.clip(cx, x0 + min_size, 10.0))
                reg['y'] = (y0, np.clip(cy, y0 + min_size, 10.0))
            elif corner == 'top_left':
                reg['x'] = (np.clip(cx, 0.0, x1 - min_size), x1)
                reg['y'] = (y0, np.clip(cy, y0 + min_size, 10.0))
            elif corner == 'bottom_right':
                reg['x'] = (x0, np.clip(cx, x0 + min_size, 10.0))
                reg['y'] = (np.clip(cy, 0.0, y1 - min_size), y1)
            elif corner == 'bottom_left':
                reg['x'] = (np.clip(cx, 0.0, x1 - min_size), x1)
                reg['y'] = (np.clip(cy, 0.0, y1 - min_size), y1)

            self._redraw_arena()

    def _on_mouse_up(self, event):
        if self.drag_mode is not None:
            action, target = self.drag_mode[0], self.drag_mode[1]
            if target != 'robot':
                self.map_modified = True
                self._ignore_modified_warning = False
                self.monte_carlo_trajectories = []
                self._syn_status.config(text="Map modified. Controller update required.", fg=AMBER)
                self._log_write(f"Updated position of {target}. Click 'Update Controller' to apply.", "warn")
            self.drag_mode = None
            self._redraw_arena()

    # ─────────────────────────────────────────────────────────
    # Layout Resets & Paper Shortcuts
    # ─────────────────────────────────────────────────────────
    def _reset_to_paper_layout(self):
        if self._playing:
            self._pause_playing()
        self.regions = copy.deepcopy(PAPER_LAYOUT)
        self.map_modified = True
        self._ignore_modified_warning = False
        self.monte_carlo_trajectories = []
        self._syn_status.config(text="Layout reset. Controller update recommended.", fg=AMBER)
        self._redraw_arena()
        self._log_write("Restored default layout. Click 'Update Controller' to apply.", "info")

    def _set_paper_start(self):
        if self._playing:
            self._pause_playing()
        self.x = np.array([6.8, 9.0, 0.0], dtype=np.float64)
        self.psi = 1
        self.status = "running"
        self.step_count = 0
        self.traj_x = [self.x[0]]
        self.traj_y = [self.x[1]]
        self.monte_carlo_trajectories = []
        self._redraw_arena()
        self._update_telemetry()
        self._log_write("Robot placed at starting position: (6.8, 9.0, 0.0).", "info")

    def _set_resolution(self, res_type):
        self.resolution_choice = res_type
        if res_type == "fast":
            self.btn_fast_res.config(bg="#21262d", fg=BLUE, font=("Segoe UI", 8, "bold"))
            self.btn_fine_res.config(bg="#161b22", fg=MUTED, font=("Segoe UI", 8))
        else:
            self.btn_fine_res.config(bg="#21262d", fg=BLUE, font=("Segoe UI", 8, "bold"))
            self.btn_fast_res.config(bg="#161b22", fg=MUTED, font=("Segoe UI", 8))

    # ─────────────────────────────────────────────────────────
    # Warning Dialog when Map is Modified
    # ─────────────────────────────────────────────────────────
    def _show_map_modified_dialog(self, next_action="play"):
        """Displays modal warning dialog when map is modified before running."""
        dlg = tk.Toplevel(self.root)
        dlg.title("Map Modified")
        dlg.configure(bg=SURFACE)
        dlg.resizable(False, False)
        dlg.transient(self.root)
        dlg.grab_set()

        # Center over parent
        pw = self.root.winfo_width()
        ph = self.root.winfo_height()
        px = self.root.winfo_rootx()
        py = self.root.winfo_rooty()
        dw, dh = 460, 240
        dx = px + max(0, (pw - dw) // 2)
        dy = py + max(0, (ph - dh) // 2)
        dlg.geometry(f"{dw}x{dh}+{dx}+{dy}")

        head = tk.Frame(dlg, bg="#21262d", height=45)
        head.pack(fill="x")
        head.pack_propagate(False)

        tk.Label(head, text="Notice: Map Has Been Modified", bg="#21262d", fg=AMBER,
                 font=("Segoe UI", 11, "bold")).pack(side="left", padx=16, pady=10)

        body = tk.Frame(dlg, bg=SURFACE, padx=16, pady=14)
        body.pack(fill="both", expand=True)

        msg = (
            "The environment regions or obstacle dimensions were modified.\n\n"
            "The current controller was computed for the previous layout. "
            "Running without updating the controller may cause collisions or sub-optimal trajectories.\n\n"
            "Would you like to recalculate the controller now, or proceed anyway?"
        )
        tk.Label(body, text=msg, bg=SURFACE, fg=TEXT, justify="left",
                 wraplength=420, font=("Segoe UI", 9)).pack(anchor="w")

        btn_box = tk.Frame(dlg, bg=SURFACE, padx=16, pady=12)
        btn_box.pack(fill="x", side="bottom")

        def on_resynthesize():
            dlg.destroy()
            self._start_synthesis(auto_run_after=next_action)

        def on_proceed():
            dlg.destroy()
            self._ignore_modified_warning = True
            self._execute_action(next_action)

        def on_cancel():
            dlg.destroy()

        b_cancel = tk.Button(btn_box, text="Cancel", command=on_cancel,
                             bg="#21262d", fg=MUTED, activebackground=BORDER,
                             activeforeground=TEXT, bd=1, relief="solid",
                             font=("Segoe UI", 8, "bold"), padx=12, pady=5, cursor="hand2")
        b_cancel.pack(side="right", padx=(6, 0))

        b_proceed = tk.Button(btn_box, text="Proceed Anyway", command=on_proceed,
                              bg="#21262d", fg=TEXT, activebackground=BORDER,
                              activeforeground=TEXT, bd=1, relief="solid",
                              font=("Segoe UI", 8, "bold"), padx=12, pady=5, cursor="hand2")
        b_proceed.pack(side="right", padx=(6, 0))

        b_resyn = tk.Button(btn_box, text="Recalculate & Run", command=on_resynthesize,
                            bg=GREEN, fg="#010409", activebackground="#2ea043",
                            activeforeground="#ffffff", bd=0,
                            font=("Segoe UI", 8, "bold"), padx=14, pady=5, cursor="hand2")
        b_resyn.pack(side="right", padx=(0, 6))

        dlg.focus_set()

    def _execute_action(self, action):
        if action == "play":
            self._start_playing()
        elif action == "step":
            self._step()
            self._redraw_arena()
            self._update_telemetry()
        elif action == "run_to_goal":
            self._run_to_completion(bypass_warning=True)
        elif action == "monte_carlo":
            self._execute_monte_carlo()

    # ─────────────────────────────────────────────────────────
    # Background Controller Computation
    # ─────────────────────────────────────────────────────────
    def _start_synthesis(self, auto_run_after=None):
        if self._is_synthesizing:
            return
        if self._playing:
            self._pause_playing()

        self._auto_run_after = auto_run_after
        self._is_synthesizing = True
        self._btn_syn.config(text="Computing...", fg=MUTED)
        self._syn_status.config(text="Computing backward reachability fixed point...", fg=AMBER)
        self._set_progress_bar(10)

        res = [60, 60, 20] if self.resolution_choice == "fast" else [100, 100, 30]

        thread = threading.Thread(target=self._run_synthesis_worker, args=(res,), daemon=True)
        thread.start()

    def _run_synthesis_worker(self, res):
        try:
            new_ctrl = sc.SymbolicController(n_x=res)
            new_ctrl.set_regions(copy.deepcopy(self.regions))

            def on_progress(it, changes, controllable_cnt):
                val = min(95, 10 + it * 2.5)
                self.root.after(0, lambda: self._update_syn_progress(val, it, controllable_cnt))

            it, elapsed = new_ctrl.synthesize(progress_callback=on_progress)
            self.ctrl = new_ctrl
            self.root.after(0, lambda: self._on_synthesis_done(it, elapsed))
        except Exception as e:
            self.root.after(0, lambda: self._log_write(f"Computation error: {e}", "fail"))
            self._is_synthesizing = False

    def _update_syn_progress(self, val, it, controllable_cnt):
        self._set_progress_bar(val)
        self._syn_status.config(text=f"Iteration {it}... {controllable_cnt:,} controllable states", fg=AMBER)

    def _on_synthesis_done(self, it, elapsed):
        self._is_synthesizing = False
        self.map_modified = False
        self._ignore_modified_warning = False
        self._set_progress_bar(100)
        self._btn_syn.config(text="Update Controller", fg=AMBER)
        res_txt = "60x60x20" if self.resolution_choice == "fast" else "100x100x30"
        self._syn_status.config(text=f"Controller ready: computed in {elapsed:.2f}s ({it} iterations, {res_txt})", fg=GREEN)
        self._log_write(f"New controller ready ({it} iterations, {elapsed:.2f}s).", "succ")
        self._reset_run()

        if self._auto_run_after:
            act = self._auto_run_after
            self._auto_run_after = None
            self._execute_action(act)

    def _set_progress_bar(self, percent):
        canvas_w = self.prog_canvas.winfo_width()
        if canvas_w < 10:
            canvas_w = 300
        w = int(canvas_w * (percent / 100.0))
        self.prog_canvas.coords(self.prog_rect, 0, 0, w, 8)
        self.prog_canvas.itemconfig(self.prog_rect, fill=GREEN if percent >= 100 else BLUE)

    # ─────────────────────────────────────────────────────────
    # Simulation Logic
    # ─────────────────────────────────────────────────────────
    def _step(self):
        if self.status in ("success", "sink", "uncontrollable", "out_of_bounds"):
            return False

        with_noise = self._noise_enabled.get()
        x_next, psi_next, u, w, status = self.ctrl.step_simulation(self.x, self.psi, with_noise=with_noise)

        old_psi = self.psi
        self.x = x_next
        self.psi = psi_next
        self.last_u = u
        self.last_w = w
        self.status = status
        self.step_count += 1

        self.traj_x.append(self.x[0])
        self.traj_y.append(self.x[1])

        # State transition logs
        if old_psi != self.psi:
            if self.psi == 2:
                self._log_write(f"[t={self.step_count}] Reached R1. State -> q2 (heading to target R3).", "succ")
            elif self.psi == 3:
                self._log_write(f"[t={self.step_count}] Reached R2. State -> q3 (heading to target R3).", "succ")
            elif self.psi == 4:
                self._log_write(f"[t={self.step_count}] Target R3 reached. Task completed.", "succ")
            elif self.psi == 5:
                self._log_write(f"[t={self.step_count}] Collision: robot entered obstacle.", "fail")

        if status == "success":
            return False
        elif status in ("sink", "uncontrollable"):
            return False

        return True

    def _on_click_step(self):
        if self.map_modified and not self._ignore_modified_warning and self.step_count == 0:
            self._show_map_modified_dialog(next_action="step")
            return
        self._step()
        self._redraw_arena()
        self._update_telemetry()

    def _toggle_play(self):
        if not self._playing:
            if self.map_modified and not self._ignore_modified_warning:
                self._show_map_modified_dialog(next_action="play")
                return
            self._start_playing()
        else:
            self._pause_playing()

    def _start_playing(self):
        self._playing = True
        self._btn_play.config(text="Pause", fg=AMBER)
        self._schedule_play()

    def _pause_playing(self):
        self._playing = False
        self._btn_play.config(text="Play", fg=GREEN)
        if self._timer_id:
            self.root.after_cancel(self._timer_id)
            self._timer_id = None

    def _schedule_play(self):
        if self._playing:
            can_continue = self._step()
            self._redraw_arena()
            self._update_telemetry()
            if can_continue:
                self._timer_id = self.root.after(100, self._schedule_play)
            else:
                self._pause_playing()

    def _on_click_run_to_goal(self):
        if self.map_modified and not self._ignore_modified_warning:
            self._show_map_modified_dialog(next_action="run_to_goal")
            return
        self._run_to_completion(bypass_warning=True)

    def _run_to_completion(self, bypass_warning=False):
        if self._playing:
            self._pause_playing()
        for _ in range(60):
            can_continue = self._step()
            if not can_continue:
                break
        self._redraw_arena()
        self._update_telemetry()

    def _reset_run(self):
        if self._playing:
            self._pause_playing()
        self.psi = 1
        self.status = "running"
        self.step_count = 0
        self.traj_x = [self.x[0]]
        self.traj_y = [self.x[1]]
        self.last_u = np.array([0.0, 0.0])
        self.last_w = np.array([0.0, 0.0, 0.0])
        self.monte_carlo_trajectories = []
        self._redraw_arena()
        self._update_telemetry()

    # ─────────────────────────────────────────────────────────
    # Monte Carlo Multi-Trajectory Simulation
    # ─────────────────────────────────────────────────────────
    def _on_click_monte_carlo(self):
        if self._playing:
            self._pause_playing()
        if self.map_modified and not self._ignore_modified_warning:
            self._show_map_modified_dialog(next_action="monte_carlo")
            return
        self._execute_monte_carlo()

    def _execute_monte_carlo(self):
        x0 = [float(self.x[0]), float(self.x[1]), float(self.x[2])]
        self._log_write(f"Simulating 50 runs with disturbance noise from ({x0[0]:.2f}, {x0[1]:.2f})...", "info")
        self.monte_carlo_trajectories = []
        success_count = 0

        for seed in range(50):
            res = self.ctrl.simulate_trajectory(x0, seed=seed, with_noise=True)
            self.monte_carlo_trajectories.append(res)
            if res['success']:
                success_count += 1

        self.monte_carlo_stats = {
            'total': 50,
            'success': success_count,
            'x0': x0.copy()
        }

        self._redraw_arena()
        rate = (success_count / 50.0) * 100.0
        self._log_write(f"50 runs completed: {success_count}/50 reached target R3 ({rate:.1f}% success).", "succ")
        self._syn_status.config(text=f"Simulation results: {success_count}/50 reached target ({rate:.1f}%)", fg=GREEN)

    def _clear_monte_carlo(self):
        self.monte_carlo_trajectories = []
        self.monte_carlo_stats = None
        self._redraw_arena()
        self._log_write("Cleared noise trajectories.", "info")

    def _update_telemetry(self):
        self._t_pos.config(text=f"({self.x[0]:.2f}, {self.x[1]:.2f}) m")
        deg = math.degrees(self.x[2])
        self._t_head.config(text=f"{deg:.1f} deg")
        self._t_ctrl.config(text=f"v={self.last_u[0]:.2f}, w={self.last_u[1]:.2f}")
        self._t_step.config(text=f"{self.step_count} s")

        for s_idx, (dot, lbl) in self.dfa_badges.items():
            if s_idx == self.psi:
                dot.config(text="[x]", fg=GREEN if s_idx == 4 else (RED if s_idx == 5 else BLUE))
                lbl.config(fg=TEXT, font=("Segoe UI", 8, "bold"))
            else:
                dot.config(text="[ ]", fg=MUTED)
                lbl.config(fg=MUTED, font=("Segoe UI", 8))

    # ─────────────────────────────────────────────────────────
    # UI Utility Helpers
    # ─────────────────────────────────────────────────────────
    def _card(self, parent, title):
        frame = tk.Frame(parent, bg=SURFACE, bd=1, relief="flat")
        frame.pack(fill="x", padx=4, pady=4)
        tk.Label(frame, text=title, bg=SURFACE, fg=MUTED,
                 font=("Segoe UI", 8, "bold")).pack(anchor="w", padx=10, pady=(6, 2))
        return frame

    def _btn(self, parent, text, cmd, bg_col, fg_col):
        btn = tk.Button(parent, text=text, command=cmd,
                        bg=bg_col, fg=fg_col, activebackground=BORDER,
                        activeforeground=TEXT, bd=1, relief="solid",
                        font=("Segoe UI", 8, "bold"), padx=8, pady=3, cursor="hand2")
        btn.pack(side="left", padx=3)
        return btn

    def _log_write(self, msg, tag="info"):
        self.log_text.config(state="normal")
        self.log_text.insert("end", msg + "\n", tag)
        self.log_text.see("end")
        self.log_text.config(state="disabled")


# ══════════════════════════════════════════════════════════════
def main():
    root = tk.Tk()
    app = RobotGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
