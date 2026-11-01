# Project 1: Mobile Robot Navigation

## 1. Overview & Objective
High-performance C++20 and CUDA implementation of discrete state abstractions, backward reachability game synthesis on a **Product Automaton**, and real-time closed-loop continuous simulation for a discrete-time unicycle mobile robot under linear temporal logic (co-safe LTL) specifications.

* **Paper Reference**: Section 3 (*"Exemple numérique : contrôle d'un robot mobile"*, pp. 11–13) of Antoine Girard, Pierre-Jean Meyer, and Adnane Saoud (2024).
* **Target Hardware**: Intel Core Ultra 9 275HX (24 cores) + NVIDIA GeForce RTX 5080 Laptop GPU (16 GB VRAM).
* **Target Language / APIs**: C++20, OpenMP 5.0, NVIDIA CUDA C++.

---

## 2. Mathematical System Model

### Continuous Dynamics (Discrete-Time Unicycle)
For continuous state $x = [x_1, x_2, x_3]^T \in [0, 10] \times [0, 10] \times [-\pi, \pi]$:
$$x_1(t+1) = x_1(t) + \tau \cdot u_1(t) \cdot \cos(x_3(t)) + \tau \cdot w_1(t)$$
$$x_2(t+1) = x_2(t) + \tau \cdot u_1(t) \cdot \sin(x_3(t)) + \tau \cdot w_2(t)$$
$$x_3(t+1) = x_3(t) + \tau \cdot u_2(t) + \tau \cdot w_3(t) \quad (\text{wrapped to } [-\pi, \pi])$$

* **Controls**: Linear velocity $u_1 \in [0.25, 1.0]$ m/s, angular velocity $u_2 \in [-1.0, 1.0]$ rad/s ($\tau = 1.0$ s).
* **Disturbances**: Process noise bounded in hyper-cube $w \in [-0.05, 0.05]^3$.
* **Co-Safe LTL Mission**: *"Visit region $R_1$ or region $R_2$ (exclusive), then reach final target region $R_3$, while avoiding obstacle $R_4$ at all times."*
* **DFA States ($Q_{\text{DFA}} = \{q_1, \dots, q_5\}$)**:
  - $q_1$: Initial search for $R_1$ or $R_2$ (avoiding $R_4$).
  - $q_2$: Waymark $R_1$ visited $\to$ seek $R_3$ (avoiding $R_2, R_4$).
  - $q_3$: Waymark $R_2$ visited $\to$ seek $R_3$ (avoiding $R_1, R_4$).
  - $q_4$: Target $R_3$ reached (Mission Success / Goal).
  - $q_5$: Obstacle collision or invalid region entry (Sink Trap / Failure).

---

## 3. Advanced Computing & Acceleration Architecture

```
┌────────────────────────────────────────────────────────────────────────┐
│                   PARALLEL PRODUCT AUTOMATON ENGINE                    │
├───────────────────────────────────┬────────────────────────────────────┤
│ CPU Backend (Intel Core Ultra 9)  │ GPU Backend (NVIDIA RTX 5080)      │
├───────────────────────────────────┼────────────────────────────────────┤
│ • 24 OpenMP threads with AVX2     │ • 1 CUDA thread per state cell     │
│ • Contiguous Structure-of-Arrays  │ • Grid size: 300,000 parallel runs │
│ • SIMD bitwise subset checks      │ • Warp ballot sync: __all_sync()   │
│ • Lock-free fixed-point convergence│ • Synthesis time: < 15 ms          │
└───────────────────────────────────┴────────────────────────────────────┘
```

1. **Cell-Level Parallelism**:
   Every state cell $(c_1, c_2, c_3)$ in the $100 \times 100 \times 30$ grid can be evaluated independently during each fixed-point iteration.
2. **CUDA Kernel Mapping**:
   A 3D grid kernel launch `dim3 block(16, 16, 1)` and `dim3 grid(7, 7, 30)` dispatches 300,000 threads directly onto the streaming multiprocessors of the RTX 5080.
3. **Warp-Level Inclusion Checking**:
   Checking whether all disturbance successors for action $u$ fall inside the controllable set uses hardware warp voting (`__all_sync`), eliminating thread branch divergence.

---

## 4. Team Task Repartition (5 Members)

### Member 1: Continuous Kinematics & Analytical Overapproximations
- Implement continuous unicycle forward kinematics and fast analytic angle wrapping in `include/unicycle_model.hpp`.
- Derive the closed-form box overapproximation for reachable states:
  $$[\underline{x}_i(t+1), \overline{x}_i(t+1)] = f_i([\underline{x}, \overline{x}], u) \oplus \tau [-0.05, 0.05]$$
- Provide validation test cases comparing numerical bounds against analytical intervals.

### Member 2: Grid Quantizer & Geometric Classifier
- Implement 3D spatial cell hashing ($\mathbb{R}^3 \to \mathbb{Z}^3$) in `include/grid_quantizer.hpp`.
- Compute lookup tables for region memberships ($R_1, R_2, R_3, R_4$ and free arena).
- Generate the static control action reachability offset tables `(dc1_min, dc1_max, dc2_min, dc2_max, dc3_min, dc3_max)` for each input pair $(u_1, u_2)$.

### Member 3: Product Automaton & Fixed-Point Game Logic
- Implement the 5-state specification DFA transition table in `include/spec_automaton.hpp`.
- Implement the Bellman-style backward reachability fixed-point algorithm on the product state space $(X \times Q_{\text{DFA}})$:
  $$V_{k+1}(q, c) = \min_{u \in U} \left\{ \max_{w \in W} V_k(q', c') + 1 \right\}$$
- Track iteration convergence and write the formal verification report validating that every state in $V < \infty$ has a proven winning strategy.

### Member 4: HPC Dual Backend (OpenMP 24-Core CPU + CUDA GPU)
- Implement the CPU parallel sweep using OpenMP 5.0 in `src/reachability_cpu.cpp` with contiguous 1D memory layout:
  ```cpp
  #pragma omp parallel for collapse(2) schedule(guided, 64)
  for (int c1 = 0; c1 < 100; ++c1) {
      for (int c2 = 0; c2 < 100; ++c2) { ... }
  }
  ```
- Implement the CUDA acceleration kernel in `src/reachability_cuda.cu` for the RTX 5080.
- Implement zero-overhead binary cache serialization (`.bin`) to store precomputed controllers.

### Member 5: Simulation, Telemetry & Interactive GUI
- Build the continuous closed-loop simulation loop in `src/main_robot.cpp` with Monte Carlo disturbance injection ($w \sim \mathcal{U}(-0.05, 0.05)$).
- Build a lightweight interactive visualizer (Dear ImGui + OpenGL or Python bindings):
  - Live 60 FPS robot tracking through waypoints.
  - Interactive click-and-drag to relocate obstacles and observe **instant sub-15ms GPU re-synthesis**.
  - 50-trajectory bundle visualization demonstrating closed-loop robustness.

---

## 5. Modular Code Layout

```
1_robot_navigation/
├── include/
│   ├── unicycle_model.hpp     # Member 1: Physics & kinematics
│   ├── grid_quantizer.hpp     # Member 2: 3D grid & regions
│   ├── spec_automaton.hpp     # Member 3: DFA & product automaton
│   ├── reachability_cpu.hpp   # Member 4: OpenMP parallel solver
│   └── reachability_cuda.cuh  # Member 4: CUDA GPU accelerated kernel
├── src/
│   ├── reachability_cpu.cpp   # CPU solver implementation
│   ├── reachability_cuda.cu   # CUDA kernel implementation
│   └── main_robot.cpp         # Member 5: Closed-loop tests & visualizer
└── CMakeLists.txt             # Dual CPU/CUDA build rules
```

---

## 6. Definition of Done
1. Fixed-point game reaches convergence in under 40 iterations.
2. GPU synthesis completes in under 20 ms on the RTX 5080 (CPU OpenMP completes in under 1.5 seconds).
3. 100% of Monte Carlo trajectories successfully navigate $R_1/R_2 \to R_3$ without colliding with $R_4$.
