# Project 1: Mobile Robot Navigation

## 1. System Model & Specification

Continuous dynamics with sampling period $\tau = 1.0\text{ s}$:

$$
\begin{aligned}
x_1(t+1) &= x_1(t) + \tau u_1(t) \cos(x_3(t)) + \tau w_1(t) \\
x_2(t+1) &= x_2(t) + \tau u_1(t) \sin(x_3(t)) + \tau w_2(t) \\
x_3(t+1) &= x_3(t) + \tau u_2(t) + \tau w_3(t) \pmod{2\pi} \in [-\pi, \pi]
\end{aligned}
$$

* **Parameters**: Linear velocity $u_1 \in [0.25, 1.0]\text{ m/s}$, angular velocity $u_2 \in [-1.0, 1.0]\text{ rad/s}$.
* **Disturbance**: Noise vector $w \in [-0.05, 0.05]^3$.
* **Default Regions**: Arena is $[0, 10] \times [0, 10]$.
  * Waymark 1: $R_1 = [4.0, 5.0] \times [8.5, 9.5]$ (Yellow)
  * Waymark 2: $R_2 = [8.5, 9.5] \times [2.0, 3.0]$ (Cyan)
  * Final Goal: $R_3 = [2.0, 3.0] \times [0.5, 1.5]$ (Green)
  * Obstacle: $R_4 = [3.0, 7.0] \times [3.0, 7.0]$ (Red)
* **Mission**: Visit $R_1$ or $R_2$ (exclusive), then reach $R_3$, while avoiding obstacle $R_4$ at all times.
* **Zero Dependencies**: This project is 100% self-contained. It compiles and runs independently with no dependencies on other folders.

---

## 2. Interactive UI & Arena Manipulation Specification

The application includes an interactive graphical interface (built with **Dear ImGui + OpenGL / GLFW**) allowing the user to manipulate the map directly:

1. **Reposition Regions ($R_1, R_2, R_3, R_4$)**: Click and drag inside any region box to move it anywhere in the arena.
2. **Resize Regions ($R_1, R_2, R_3, R_4$)**: Drag corner/edge handles of any region box to change its width and height.
3. **Reposition Robot**: Click and drag the robot circle/arrow avatar to set any initial continuous coordinate $(x_1, x_2)$ and heading angle $x_3$.
4. **Dynamic On-The-Fly Re-Synthesis**: When the user modifies or resizes any region, clicking "Run" triggers sub-second re-synthesis of the controller for the new layout.
5. **Real-Time Telemetry**: Live panel displaying coordinates $(x, y, \theta)$, active velocity $(v, \omega)$, current DFA state $q$, and trajectory bundle plots.

---

## 3. Exact Member Task Breakdown (What & How)

### Member 1: Kinematics & Reachable Bounding Box
* **File to produce**: `include/unicycle_model.hpp`
* **Functions to implement**:
  1. `State step(State s, Control u, Vector3d w, double dt)`
     * **What it does**: Computes the exact continuous state $[x_1, x_2, x_3]$ at the next time step.
     * **How it does it**: Applies discrete unicycle kinematics: $x_1' = x_1 + \tau u_1 \cos(x_3) + \tau w_1$, $x_2' = x_2 + \tau u_1 \sin(x_3) + \tau w_2$, $x_3' = x_3 + \tau u_2 + \tau w_3$, followed by periodic angle wrapping into $[-\pi, \pi]$ using `atan2(sin(x3), cos(x3))`.
  2. `Box compute_reachable_box(Box cell_box, Control u, Box noise_box, double dt)`
     * **What it does**: Computes the analytical 3D overapproximation box $[\underline{x}_1, \overline{x}_1] \times [\underline{x}_2, \overline{x}_2] \times [\underline{x}_3, \overline{x}_3]$ containing all possible next states.
     * **How it does it**: Evaluates the minimum and maximum of $\cos(\theta)$ and $\sin(\theta)$ over interval $[\underline{\theta}, \overline{\theta}]$, multiplies by linear speed $u_1$, adds cell center displacements, and applies Minkowski addition with the disturbance box $\tau [-0.05, 0.05]^3$.

### Member 2: 3D Grid Quantizer & Spatial Indexing
* **File to produce**: `include/grid_quantizer.hpp`
* **Functions to implement**:
  1. `GridCoord quantize(State s)`
     * **What it does**: Maps continuous state $(x_1, x_2, x_3) \in \mathbb{R}^3$ to grid indices $(c_1, c_2, c_3) \in \mathbb{Z}^3$.
     * **How it does it**: Computes $c_i = \lfloor (x_i - \text{bound\_min}_i) / \Delta x_i \rfloor$, clamped to $[0, N_i - 1]$ for grid resolution $100 \times 100 \times 30$.
  2. `int get_region_label(double x, double y, const RegionConfig& regions)`
     * **What it does**: Returns discrete label (1 = Free, 2 = $R_1$, 3 = $R_2$, 4 = $R_3$, 5 = $R_4$).
     * **How it does it**: Performs 2D Axis-Aligned Bounding Box (AABB) inclusion checks: $r.x_{\min} \le x \le r.x_{\max} \land r.y_{\min} \le y \le r.y_{\max}$. If inside $R_4$, returns 5 immediately (obstacle priority).
  3. `void precompute_action_footprints(OffsetTable& table)`
     * **What it does**: Precomputes discrete cell offset bounding boxes for all 15 control pairs $(u_1, u_2)$ across all heading bins $c_3 \in [0, 29]$.
     * **How it does it**: Loops over heading indices, calls `compute_reachable_box` on unit cells, quantizes resulting corners, and stores min/max index offsets `(dc1_min, dc1_max, dc2_min, dc2_max, dc3_min, dc3_max)`.

### Member 3: Product Automaton & Fixed-Point Game
* **Files to produce**: `include/product_automaton.hpp`, `src/product_automaton.cpp`
* **Functions to implement**:
  1. `int dfa_transition(int q, int label)`
     * **What it does**: Evaluates the next DFA specification state $q' \in \{1, \dots, 5\}$.
     * **How it does it**: Look up in 5x5 transition matrix implementing the mission logic:
       * State 1: Label 2 $\to$ 2 ($R_1$ visited), Label 3 $\to$ 3 ($R_2$ visited), Label 5 $\to$ 5 (Sink).
       * State 2: Label 3 $\to$ 5 (Forbidden $R_2$), Label 4 $\to$ 4 (Target $R_3$), Label 5 $\to$ 5.
       * State 3: Label 2 $\to$ 5 (Forbidden $R_1$), Label 4 $\to$ 4 (Target $R_3$), Label 5 $\to$ 5.
       * State 4: Terminal success state ($q=4$).
       * State 5: Terminal trap state ($q=5$).
  2. `int synthesize_step(ValueArray& V, ValueArray& V_prev, ControllerArray& ctrl, ...)`
     * **What it does**: Runs 1 iteration of the backward reachability fixed-point game.
     * **How it does it**: For each product state $(q, c_1, c_2, c_3)$, checks if there exists a control action $\sigma$ such that for *all* reachable cells $(c_1', c_2', c_3')$ in its disturbance footprint, the successor state is already controllable ($V(q_{\text{succ}}, c') < \infty$). Updates $V(q, c) = \text{iter}$ and records optimal control index.

### Member 4: HPC Dual Backend (OpenMP 24-Thread & CUDA Kernel)
* **Files to produce**: `src/reachability_cpu.cpp`, `src/reachability_cuda.cu`
* **Functions to implement**:
  1. `void solve_reachability_openmp(ValueArray& V, ControllerArray& ctrl)`
     * **What it does**: Executes fixed-point synthesis across the 24 cores of the Intel Core Ultra 9.
     * **How it does it**: Flattens 4D state space into a 1D contiguous array `uint8_t[5 * 100 * 100 * 30]`. Employs OpenMP dynamic scheduling `#pragma omp parallel for collapse(2) schedule(guided)` over spatial coordinates $(c_1, c_2)$, checking convergence using an atomic change counter.
  2. `void solve_reachability_cuda(...)`
     * **What it does**: Accelerates synthesis on the NVIDIA RTX 5080 Laptop GPU.
     * **How it does it**: Launches a 300,000-thread CUDA grid where each thread processes 1 cell. Uses warp voting (`__all_sync`) to test whether all disturbance cells are safe in single clock cycles.
  3. `bool save_controller_binary(const char* path)` / `bool load_controller_binary(const char* path)`
     * **What it does**: Caches synthesized controllers to disk for instantaneous loading.
     * **How it does it**: Writes raw memory buffer to disk with header validation (grid dimensions, region coordinates, checksum).

### Member 5: Interactive UI & Closed-Loop Simulation
* **Files to produce**: `include/gui_arena.hpp`, `src/main_robot.cpp`
* **Functions to implement**:
  1. `void handle_mouse_interaction(RegionConfig& regions, State& robot_pose)`
     * **What it does**: Allows the user to click, drag, reposition, and resize $R_1, R_2, R_3, R_4$ and the robot directly on the map.
     * **How it does it**: Mouse hit-testing:
       * If mouse down within 8 pixels of region corner: enter `RESIZE_MODE` and adjust $(x_{\min}, x_{\max}, y_{\min}, y_{\max})$ based on mouse delta.
       * If mouse down inside region box: enter `DRAG_MODE` and translate entire box $(x \pm dx, y \pm dy)$.
       * If mouse down on robot avatar: enter `ROBOT_MOVE_MODE` and set robot $(x_1, x_2)$ directly.
  2. `void render_arena(const RegionConfig& regions, const Trajectory& traj)`
     * **What it does**: Renders the 2D arena, regions, obstacle handles, and trajectory curves at 60 FPS.
     * **How it does it**: Calls Dear ImGui / OpenGL draw primitives (`ImDrawList::AddRectFilled`, `AddCircleFilled`, `AddPolyline`).
  3. `SimulationResult run_closed_loop_simulation(State x0, int max_steps, bool with_noise)`
     * **What it does**: Simulates the robot navigating the arena using the synthesized controller.
     * **How it does it**: Step loop: quantize continuous $x(t) \to c$, lookup DFA state $q$, lookup optimal control $u^* = \text{ctrl}[q, c]$, inject random noise $w \sim \mathcal{U}(-0.05, 0.05)^3$, integrate continuous state via Member 1's `step()`, update DFA state via Member 3's `dfa_transition()`, repeat until $q=4$ (Goal) or $q=5$ (Trap).

---

## 4. Build & Run
```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/robot_navigation
```
