# Project 1: Mobile Robot Navigation

## 1. System & Goal

We control a discrete-time unicycle robot in a $10 \times 10$ m arena:

$$
\begin{aligned}
x_1(t+1) &= x_1(t) + \tau u_1(t) \cos(x_3(t)) + \tau w_1(t) \\
x_2(t+1) &= x_2(t) + \tau u_1(t) \sin(x_3(t)) + \tau w_2(t) \\
x_3(t+1) &= x_3(t) + \tau u_2(t) + \tau w_3(t) \pmod{2\pi} \in [-\pi, \pi]
\end{aligned}
$$

* **Sample time**: $\tau = 1.0\text{ s}$.
* **Controls**: 
  * Linear speed $u_1 \in \{0.25, 0.625, 1.0\}\text{ m/s}$
  * Steering speed $u_2 \in \{-1.0, -0.5, 0.0, 0.5, 1.0\}\text{ rad/s}$
  * Total of 15 pairs $(u_1, u_2)$, indexed $0 \dots 14$.
* **Noise**: Bounded disturbance $w_i \in [-0.05, 0.05]$ on each axis.
* **Grid**: 
  * $x_1 \in [0, 10]$ (100 cells, $\Delta x_1 = 0.1\text{ m}$)
  * $x_2 \in [0, 10]$ (100 cells, $\Delta x_2 = 0.1\text{ m}$)
  * $x_3 \in [-\pi, \pi]$ (30 bins, $\Delta x_3 \approx 0.2094\text{ rad}$)
  * Total: $100 \times 100 \times 30 = 300,000$ cells.
* **Default Map**:
  * Waymark 1: $R_1 = [4.0, 5.0] \times [8.5, 9.5]$ (Yellow)
  * Waymark 2: $R_2 = [8.5, 9.5] \times [2.0, 3.0]$ (Cyan)
  * Final Goal: $R_3 = [2.0, 3.0] \times [0.5, 1.5]$ (Green)
  * Obstacle: $R_4 = [3.0, 7.0] \times [3.0, 7.0]$ (Red)
* **Mission**: Visit $R_1$ or $R_2$ (exclusive), then go to $R_3$, and never hit $R_4$.
* **Zero Dependencies**: This folder is self-contained and builds on its own.

---

## 2. Why We Do Each Step (Plain English)

1. **Why unicycle kinematics?**
   Cars and wheeled robots cannot move sideways. We need the heading angle $x_3$ in the state vector, otherwise turning around obstacles is physically impossible.
2. **Why bounding boxes (`compute_reachable_box`)?**
   Because of noise $w$, we can't predict an exact destination point—we get an uncertainty cloud. We compute a bounding box that wraps around all possible landing spots. If this outer box doesn't touch the obstacle, the real robot won't either.
3. **Why a 3D grid ($100 \times 100 \times 30$)?**
   Continuous space has infinite points. You can't run graph search on infinity, so we chop the arena into 300k small cells.
4. **Why a 5-state DFA?**
   Standard controllers have no memory; they only know their current position. But our rule says: *"visit R1 or R2 first, then go to R3"*. The DFA is just a simple state variable ($q \in \{1 \dots 5\}$) that tracks whether we already hit a waymark.
5. **Why a Product Automaton ($X \times Q_{\text{DFA}}$)?**
   To pick the right action, the controller needs to know two things at once: where the robot is physically ($c_1, c_2, c_3$) and what stage the mission is in ($q$). The product automaton pairs them up.
6. **Why backward reachability?**
   Instead of guessing paths from the start, we start at the goal ($q=4$) and work backwards. We find all cells that can reach the goal in 1 step, then 2 steps, then 3 steps, avoiding $R_4$, until no new cells can reach it.
7. **Why an offline lookup table?**
   Solving reachability takes a few seconds. We run it once offline. When the robot is driving, it just reads an array `u = table[q, c1, c2, c3]` in less than a microsecond.

---

## 3. Interactive UI (Dear ImGui)

The desktop app includes an interactive map view:
* **Drag regions**: Click inside $R_1, R_2, R_3,$ or $R_4$ and drag to move them.
* **Resize regions**: Grab the corner handles of any region (especially obstacle $R_4$) to resize it.
* **Move robot**: Click and drag the robot avatar to set starting $(x_1, x_2)$ and heading.
* **Recompute**: When any region is moved or resized, clicking "Run" re-runs synthesis for the new layout.
* **Live telemetry**: Displays $(x, y, \theta)$, current control $(v, \omega)$, DFA state $q$, and trajectory bundle plots.

---

## 4. Member Tasks (What each function does and how)

### Member 1: Kinematics & Reachable Bounding Box
* **File**: `include/unicycle_model.hpp`
* **Constants**: `U1 = {0.25, 0.625, 1.0}`, `U2 = {-1.0, -0.5, 0.0, 0.5, 1.0}`
* **Functions**:
  1. `State step(State s, Control u, Vector3d w, double dt)`
     * **What**: Moves the continuous robot state forward by 1 second.
     * **How**: Calculates $x_1' = x_1 + dt \cdot u_1 \cos(x_3) + dt \cdot w_1$, $x_2' = x_2 + dt \cdot u_1 \sin(x_3) + dt \cdot w_2$, $x_3' = x_3 + dt \cdot u_2 + dt \cdot w_3$, and wraps $x_3'$ into $[-\pi, \pi]$ using `atan2(sin(x3), cos(x3))`.
  2. `Box compute_reachable_box(Box cell_box, Control u, Box noise_box, double dt)`
     * **What**: Returns a box $[x_{\min}, x_{\max}] \times [y_{\min}, y_{\max}] \times [\theta_{\min}, \theta_{\max}]$ covering all possible next states.
     * **How**: Finds the min and max of $\cos(\theta)$ and $\sin(\theta)$ over the cell's heading range $[\theta_{\min}, \theta_{\max}]$, multiplies by speed $u_1$, and adds the noise range $[-0.05, 0.05]$.

### Member 2: Grid Quantizer & Spatial Indexing
* **File**: `include/grid_quantizer.hpp`
* **Constants**: `N1 = 100, N2 = 100, N3 = 30`, `DX1 = 0.1, DX2 = 0.1, DX3 = 2*PI/30`
* **Functions**:
  1. `GridCoord quantize(State s)`
     * **What**: Turns continuous coordinates into grid indices $(c_1, c_2, c_3)$.
     * **How**: Divides $x_1 / 0.1$ and $x_2 / 0.1$ (clamped to $0 \dots 99$), and $(x_3 + \pi) / \Delta x_3$ (clamped to $0 \dots 29$).
  2. `int get_region_label(double x, double y, const RegionConfig& regions)`
     * **What**: Returns region label (1 = Free, 2 = $R_1$, 3 = $R_2$, 4 = $R_3$, 5 = $R_4$).
     * **How**: Tests if point $(x, y)$ is inside each box. Checks obstacle $R_4$ first (returns 5 if inside).
  3. `void precompute_action_footprints(OffsetTable& table)`
     * **What**: Precomputes index offset bounds `(dc1_min, dc1_max, ...)` for all 15 actions and 30 angles.
     * **How**: Calls `compute_reachable_box` for each heading angle and action, quantizes the corners, and stores the resulting integer index ranges.

### Member 3: Product Automaton & Fixed-Point Game
* **Files**: `include/product_automaton.hpp`, `src/product_automaton.cpp`
* **Base Cases**:
  * Goal ($q=4$): $V_0 = 0$
  * Obstacle ($q=5$): $V_0 = 10^7$ (infinity)
  * Active states ($q \in \{1, 2, 3\}$): $V_0 = 10^7$ (unreached)
* **Functions**:
  1. `int dfa_transition(int q, int label)`
     * **What**: Returns next DFA state.
     * **How**: 
       * From $q=1$: label 2 goes to 2 ($R_1$ visited), label 3 goes to 3 ($R_2$ visited), label 5 goes to 5 (hit obstacle).
       * From $q=2$: label 3 goes to 5 (forbidden $R_2$), label 4 goes to 4 (reached goal), label 5 goes to 5.
       * From $q=3$: label 2 goes to 5 (forbidden $R_1$), label 4 goes to 4 (reached goal), label 5 goes to 5.
       * From $q=4, 5$: stays in 4 or 5.
  2. `int synthesize_step(ValueArray& V, ValueArray& V_prev, ControllerArray& ctrl, int iter)`
     * **What**: Runs 1 iteration of the backward reachability loop.
     * **How**: For each cell where $V == 10^7$, gets `q_succ = dfa_transition(q, label)`. If $q_{\text{succ}} == 4$, marks it reachable immediately. Otherwise, checks each of the 15 control actions: if all cells in its disturbance footprint have $V_{\text{prev}} < 10^7$, sets $V = \text{iter}$ and saves the action index.

### Member 4: OpenMP Multi-Threading & CUDA
* **Files**: `src/reachability_cpu.cpp`, `src/reachability_cuda.cu`
* **Array layout**: Flat 1D buffer of $5 \times 100 \times 100 \times 30 = 1,500,000$ values:
  `idx = (q - 1) * 300000 + c1 * 3000 + c2 * 30 + c3`
* **Functions**:
  1. `void solve_reachability_openmp(ValueArray& V, ControllerArray& ctrl)`
     * **What**: Runs the reachability loop in parallel across 24 CPU cores.
     * **How**: Loops until 0 changes are made (usually $\approx 35$ iterations). Parallelizes the outer grid loops with `#pragma omp parallel for collapse(2) schedule(guided)`.
  2. `void solve_reachability_cuda(...)`
     * **What**: GPU version on the RTX 5080.
     * **How**: 300k threads (1 thread per cell). Uses warp voting (`__all_sync`) to check if all footprint cells are safe.
  3. `bool save_controller_binary(const char* path)` / `load_controller_binary(const char* path)`
     * **What**: Saves and loads precomputed controllers to/from disk so we don't have to wait every launch.

### Member 5: Interactive UI & Simulation
* **Files**: `include/gui_arena.hpp`, `src/main_robot.cpp`
* **Functions**:
  1. `void handle_mouse_interaction(RegionConfig& regions, State& robot_pose)`
     * **What**: Mouse drag-and-drop for the map.
     * **How**: If clicked near a region corner, resizes it. If clicked inside a region, moves it. If clicked on the robot, moves starting position.
  2. `void render_arena(const RegionConfig& regions, const Trajectory& traj)`
     * **What**: Draws the arena, regions, robot, and path at 60 FPS using Dear ImGui / OpenGL.
  3. `SimulationResult run_closed_loop_simulation(State x0, int max_steps, bool with_noise)`
     * **What**: Simulates the robot following the controller.
     * **How**: Loop: quantize $(x_1, x_2, x_3) \to c$, lookup action $u = \text{ctrl}[q, c]$, add random noise $w \in [-0.05, 0.05]$, call Member 1's `step()`, update DFA state $q$, repeat until reaching $q=4$ or $q=5$.

---

## 5. Build & Run

```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/robot_navigation
```
