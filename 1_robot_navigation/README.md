# Project 1: Mobile Robot Navigation

## 1. System Model & Specification
* **Dynamics**:
  $$x_1(t+1) = x_1(t) + \tau u_1(t) \cos(x_3(t)) + \tau w_1(t)$$
  $$x_2(t+1) = x_2(t) + \tau u_1(t) \sin(x_3(t)) + \tau w_2(t)$$
  $$x_3(t+1) = x_3(t) + \tau u_2(t) + \tau w_3(t) \pmod{2\pi} \in [-\pi, \pi]$$
* **Parameters**: $\tau = 1.0\text{ s}$, $u_1 \in [0.25, 1.0]\text{ m/s}$, $u_2 \in [-1.0, 1.0]\text{ rad/s}$, $w_i \in [-0.05, 0.05]$.
* **Regions**: Arena $[0, 10]^2$. $R_1=[4, 5]\times[8.5, 9.5]$, $R_2=[8.5, 9.5]\times[2, 3]$, $R_3=[2, 3]\times[0.5, 1.5]$, $R_4=[3, 7]\times[3, 7]$ (Obstacle).
* **Mission**: Visit $R_1$ or $R_2$ (exclusive), then reach $R_3$, avoiding $R_4$.

---

## 2. Exact Member Task Assignments

### Member 1: Kinematics & Reachable Bounding Box
* **File to produce**: `include/unicycle_model.hpp`
* **Exact Tasks**:
  1. Define structs: `struct State { double x, y, theta; };` and `struct Control { double v, w; };`.
  2. Implement `State step(State s, Control u, Vector3d w, double dt)` with angle normalization in $[-\pi, \pi]$.
  3. Implement `Box compute_reachable_box(Box cell_box, Control u, Box noise_box, double dt)` computing analytical lower and upper coordinate bounds $[\underline{x}_i, \overline{x}_i]$.

### Member 2: 3D Grid Quantizer & Spatial Indexing
* **File to produce**: `include/grid_quantizer.hpp`
* **Exact Tasks**:
  1. Implement `GridCoord quantize(State s)` mapping $\mathbb{R}^3 \to \mathbb{Z}^3$ on a $100 \times 100 \times 30$ grid.
  2. Implement `State cell_center(GridCoord c)`.
  3. Implement `int get_region_label(double x, double y)`: Free = 1, $R_1$ = 2, $R_2$ = 3, $R_3$ = 4, $R_4$ = 5.
  4. Precompute control offset bounding boxes `(dc1_min, dc1_max, dc2_min, dc2_max, dc3_min, dc3_max)` for all 15 discrete $(u_1, u_2)$ pairs.

### Member 3: Product Automaton & Fixed-Point Game
* **Files to produce**: `include/product_automaton.hpp`, `src/product_automaton.cpp`
* **Exact Tasks**:
  1. Define DFA transition table: `int dfa_transition(int q, int label)` for states $q_1..q_5$.
  2. Implement backward reachability fixed-point over product space $(Q_{\text{DFA}} \times X_{\text{grid}})$:
     $$V_{k+1}(q, c) = \min_{u \in U} \left\{ \max_{w \in W} V_k(q', c') + 1 \right\}$$
  3. Output synthesized controller map: `uint8_t best_control[5][100][100][30]`.

### Member 4: HPC Engine (OpenMP 24-Thread & CUDA Kernel)
* **Files to produce**: `src/reachability_cpu.cpp`, `src/reachability_cuda.cu`
* **Exact Tasks**:
  1. Store grid memory contiguously: `uint8_t V[5 * 100 * 100 * 30]` for cache locality.
  2. Implement CPU multi-threading: `#pragma omp parallel for collapse(2) schedule(guided)` over the 24 threads of the Intel Core Ultra 9.
  3. Implement CUDA kernel for RTX 5080: 1 thread per cell, using `__all_sync()` for disturbance verification.
  4. Implement binary controller serialization: `save_cache(path)` / `load_cache(path)`.

### Member 5: Closed-Loop Simulation & Telemetry
* **File to produce**: `src/main_robot.cpp`
* **Exact Tasks**:
  1. Run closed-loop continuous simulation from initial state $x_0 = [1.0, 2.0, 0.0]$.
  2. Inject random uniform disturbance $w(t) \sim \mathcal{U}(-0.05, 0.05)^3$ at each step.
  3. Execute 50 Monte Carlo runs and log trajectories to `trajectories.csv`.
  4. Assert acceptance criterion: 100% reach target $q_4$ without collision $q_5$.

---

## 3. Build & Run
```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/robot_navigation
```
