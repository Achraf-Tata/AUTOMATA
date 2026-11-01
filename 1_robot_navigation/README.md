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

### Exact Numerical Discretization
* **Linear Velocity ($u_1$)**: 3 discrete values: $\{0.25, 0.625, 1.0\}\text{ m/s}$.
* **Angular Velocity ($u_2$)**: 5 discrete values: $\{-1.0, -0.5, 0.0, 0.5, 1.0\}\text{ rad/s}$.
* **Action Space ($U$)**: $3 \times 5 = 15$ discrete input pairs $(u_1, u_2)$, indexed $0 \dots 14$.
* **Process Noise ($W$)**: Hyper-cube $w \in [-0.05, 0.05]^3$.
* **Grid Resolution ($100 \times 100 \times 30 = 300,000$ cells)**:
  * $x_1 \in [0.0, 10.0]\text{ m} \implies N_1 = 100\text{ bins}, \Delta x_1 = 0.1\text{ m}$.
  * $x_2 \in [0.0, 10.0]\text{ m} \implies N_2 = 100\text{ bins}, \Delta x_2 = 0.1\text{ m}$.
  * $x_3 \in [-\pi, \pi]\text{ rad} \implies N_3 = 30\text{ bins}, \Delta x_3 = 2\pi / 30 \approx 0.20944\text{ rad}$.
* **Default Regions in $[0, 10]^2$**:
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
* **Constants**:
  * Linear speeds: `const double U1[3] = {0.25, 0.625, 1.0};`
  * Angular speeds: `const double U2[5] = {-1.0, -0.5, 0.0, 0.5, 1.0};`
* **Functions to implement**:
  1. `State step(State s, Control u, Vector3d w, double dt)`
     * **What it does**: Computes continuous state $[x_1, x_2, x_3]$ at next time step.
     * **How it does it**: Evaluates unicycle update: $x_1' = x_1 + \tau u_1 \cos(x_3) + \tau w_1$, $x_2' = x_2 + \tau u_1 \sin(x_3) + \tau w_2$, $x_3' = x_3 + \tau u_2 + \tau w_3$, followed by periodic angle wrapping into $[-\pi, \pi]$ using `atan2(sin(x3), cos(x3))`.
  2. `Box compute_reachable_box(Box cell_box, Control u, Box noise_box, double dt)`
     * **What it does**: Computes the analytical 3D overapproximation box $[\underline{x}_1, \overline{x}_1] \times [\underline{x}_2, \overline{x}_2] \times [\underline{x}_3, \overline{x}_3]$ containing all possible next states.
     * **How it does it**: Evaluates extrema of $\cos(\theta)$ and $\sin(\theta)$ over interval $[\underline{\theta}, \overline{\theta}]$, multiplies by linear speed $u_1$, adds cell center displacements, and applies Minkowski addition with the disturbance box $\tau [-0.05, 0.05]^3$.

### Member 2: 3D Grid Quantizer & Spatial Indexing
* **File to produce**: `include/grid_quantizer.hpp`
* **Constants**:
  * `N1 = 100, N2 = 100, N3 = 30`
  * `DX1 = 0.1, DX2 = 0.1, DX3 = 2.0 * M_PI / 30.0`
* **Functions to implement**:
  1. `GridCoord quantize(State s)`
     * **What it does**: Maps continuous state $(x_1, x_2, x_3)$ to grid indices $(c_1, c_2, c_3)$.
     * **How it does it**: $c_1 = \text{clamp}(\lfloor x_1 / 0.1 \rfloor, 0, 99)$, $c_2 = \text{clamp}(\lfloor x_2 / 0.1 \rfloor, 0, 99)$, $c_3 = \text{clamp}(\lfloor (x_3 + \pi) / \Delta x_3 \rfloor, 0, 29)$.
  2. `int get_region_label(double x, double y, const RegionConfig& regions)`
     * **What it does**: Returns discrete label (1 = Free, 2 = $R_1$, 3 = $R_2$, 4 = $R_3$, 5 = $R_4$).
     * **How it does it**: Performs 2D AABB checks. If inside $R_4$, returns 5 immediately (obstacle priority). Else checks $R_1, R_2, R_3$. Defaults to 1.
  3. `void precompute_action_footprints(OffsetTable& table)`
     * **What it does**: Precomputes discrete index offset bounds `(dc1_min, dc1_max, dc2_min, dc2_max, dc3_min, dc3_max)` for all 15 control pairs across all 30 heading bins.
     * **How it does it**: Loops over $c_3 \in [0, 29]$ and $\sigma \in [0, 14]$, calls `compute_reachable_box` on unit cell centered at $(0, 0, c_3 \cdot \Delta x_3 - \pi)$, quantizes corners, and stores integer index deltas.

### Member 3: Product Automaton & Fixed-Point Game
* **Files to produce**: `include/product_automaton.hpp`, `src/product_automaton.cpp`
* **Base Case Initialization**:
  $$V_0(q, c_1, c_2, c_3) = \begin{cases} 0 & \text{if } q = 4 \text{ (Target Goal)} \\ \text{INF} & \text{if } q = 5 \text{ (Obstacle Sink, INF = } 10^7\text{)} \\ \text{INF} & \text{if } q \in \{1, 2, 3\} \text{ (Unreached)} \end{cases}$$
* **Functions to implement**:
  1. `int dfa_transition(int q, int label)`
     * **What it does**: Evaluates next DFA specification state $q' \in \{1, \dots, 5\}$.
     * **How it does it**: 5x5 transition table:
       * $q=1$ (Search): Label 2 $\to$ 2 ($R_1$ visited), Label 3 $\to$ 3 ($R_2$ visited), Label 5 $\to$ 5 (Sink).
       * $q=2$ ($R_1$ visited): Label 3 $\to$ 5 (Forbidden $R_2$), Label 4 $\to$ 4 (Goal), Label 5 $\to$ 5.
       * $q=3$ ($R_2$ visited): Label 2 $\to$ 5 (Forbidden $R_1$), Label 4 $\to$ 4 (Goal), Label 5 $\to$ 5.
       * $q=4$ (Success): Self-loop on all labels.
       * $q=5$ (Sink): Self-loop on all labels.
  2. `int synthesize_step(ValueArray& V, ValueArray& V_prev, ControllerArray& ctrl, int iter)`
     * **What it does**: Runs 1 iteration of the backward reachability fixed-point game.
     * **How it does it**: For each state $(q, c_1, c_2, c_3)$ where $V(q, c) == \text{INF}$:
       * Determines $q_{\text{succ}} = \text{dfa\_transition}(q, \text{label}(c_1, c_2))$.
       * If $q_{\text{succ}} == 4$, sets $V(q, c) = \text{iter}$ immediately.
       * If $q_{\text{succ}} == 5$, continues (unsafe).
       * Searches over actions $\sigma \in [0, 14]$: checks if for *all* disturbance successor cells $(c_1', c_2', c_3')$ in the footprint of $\sigma$, $V_{\text{prev}}(q_{\text{succ}}, c') < \text{INF}$.
       * If found: sets $V(q, c) = \text{iter}$, records $\text{ctrl}(q, c) = \sigma$, and increments changes count.

### Member 4: HPC Dual Backend (OpenMP 24-Thread & CUDA Kernel)
* **Files to produce**: `src/reachability_cpu.cpp`, `src/reachability_cuda.cu`
* **Exact Memory Layout**: Flat 1D buffer of size $5 \times 100 \times 100 \times 30 = 1,500,000$ elements:
  `index(q, c1, c2, c3) = (q - 1) * 300000 + c1 * 3000 + c2 * 30 + c3`
* **Functions to implement**:
  1. `void solve_reachability_openmp(ValueArray& V, ControllerArray& ctrl)`
     * **What it does**: Executes fixed-point synthesis across the 24 cores of the Intel Core Ultra 9.
     * **How it does it**: Iterates until changes == 0 (typically converges in $\approx 35$ steps). Parallelizes the double loop over $(c_1, c_2)$ with `#pragma omp parallel for collapse(2) schedule(guided, 64)`.
  2. `void solve_reachability_cuda(...)`
     * **What it does**: Accelerates synthesis on the NVIDIA RTX 5080 Laptop GPU.
     * **How it does it**: Dispatches 300,000 threads. Uses warp voting (`__all_sync`) to evaluate disturbance footprint inclusion in single clock cycles.
  3. `bool save_controller_binary(const char* path)` / `bool load_controller_binary(const char* path)`
     * **What it does**: Caches synthesized controllers to disk for instantaneous loading.
     * **How it does it**: Writes raw memory buffer with header validation (grid dimensions, region coordinates, iterations count).

### Member 5: Interactive UI & Closed-Loop Simulation
* **Files to produce**: `include/gui_arena.hpp`, `src/main_robot.cpp`
* **Functions to implement**:
  1. `void handle_mouse_interaction(RegionConfig& regions, State& robot_pose)`
     * **What it does**: Allows user to click, drag, reposition, and resize $R_1, R_2, R_3, R_4$ and the robot directly on the map.
     * **How it does it**: Mouse hit-testing:
       * Corner handle clicked ($\le 8$ px): adjust $(x_{\min}, x_{\max}, y_{\min}, y_{\max})$ by mouse delta.
       * Inside region clicked: translate box $(x \pm dx, y \pm dy)$.
       * Robot avatar clicked: set continuous position $(x_1, x_2)$ directly.
  2. `void render_arena(const RegionConfig& regions, const Trajectory& traj)`
     * **What it does**: Renders 2D arena, regions, obstacle handles, and trajectory curves at 60 FPS using Dear ImGui / OpenGL.
  3. `SimulationResult run_closed_loop_simulation(State x0, int max_steps, bool with_noise)`
     * **What it does**: Simulates robot navigating arena using synthesized controller.
     * **How it does it**: Loop: quantize $x(t) \to c$, lookup DFA state $q$, lookup optimal control action $u^* = \text{ctrl}[q, c]$, inject random noise $w \sim \mathcal{U}(-0.05, 0.05)^3$, integrate continuous state via Member 1's `step()`, update DFA state via Member 3's `dfa_transition()`, repeat until $q=4$ (Goal) or $q=5$ (Trap).

---

## 4. Build & Run
```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/robot_navigation
```
