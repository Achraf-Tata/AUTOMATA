# Project 3: Circular Building Challenge (Compositional Synthesis)

## 1. Overview & Challenge Objective
The circular building problem is the **competitive multi-team challenge** of the course. The objective is to synthesize a provably safe, robust supervisory controller for an $N$-room ring-shaped building.

* **Paper Reference**: Section 4.1 (*"Abstractions efficaces et compositionnelles"*, pp. 13–15) & Section 4.4 (*"Applications : les bâtiments actifs"*, p. 16) of Antoine Girard, Pierre-Jean Meyer, and Adnane Saoud (2024).
* **The Competition Trap**: Naive monolithic state-space discretization explodes exponentially ($O(G^N)$ states). At $N=4$ it requires $6.25 \times 10^6$ states, and at $N=5$ it demands $3.12 \times 10^8$ states $\implies$ **Out-of-Memory (OOM) crash**.
* **Our Winning Architecture**: 
  1. **Assume-Guarantee Contracts (Section 4.1.4)**: Drops complexity from $O(G^N)$ to **$O(N \cdot G^3)$** (linear in the number of rooms $N$).
  2. **Cooperative / Monotone Order Preservation (Section 1.1.3 & Section 4)**: The Kamke-Müller theorem guarantees that an infinite continuous state box can be bounded by evaluating **only two extreme trajectories**.
  3. **CUDA + 24-Thread Parallelization**: All $N$ room contracts are synthesized simultaneously on the GPU (RTX 5080) and CPU (Intel Core Ultra 9 275HX), enabling synthesis for **$N = 100$ or even $N = 1000$ rooms in milliseconds**.

---

## 2. Mathematical System Model

### Coupled Thermal Network (Ring Topology)
For $N$ identical rooms $i \in \{0, \dots, N-1\}$ with cyclic periodic boundary conditions ($i-1 \pmod N$ and $i+1 \pmod N$):
$$\dot{T}_i(t) = -\alpha \big(T_i(t) - T_e(t)\big) - \beta \Big(\big(T_i(t) - T_{i-1}(t)\big) + \big(T_i(t) - T_{i+1}(t)\big)\Big) + \gamma \cdot u_i(t)$$

* **Continuous State Vector**: $T = [T_0, T_1, \dots, T_{N-1}]^T \in [15.0, 25.0]^N$.
* **Controls**: Independent binary heaters $u_i \in \{0, 1\}$ for each room.
* **Thermal Coupling**: Conduction coefficient $\beta$ through shared walls.
* **Disturbance**: Outdoor weather $T_e(t) \in [T_{e,\min}, T_{e,\max}]$.
* **Global Safety Specification**:
  $$\forall i \in \{0, \dots, N-1\}, \quad \forall t \ge 0, \quad T_i(t) \in [T_{\text{safe, min}}, T_{\text{safe, max}}] = [19.0, 22.0]^\circ\text{C}$$

---

## 3. High-Performance Challenge Architecture

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                        COMPOSITIONAL ASSUME-GUARANTEE ENGINE                           │
├───────────────────────────────────────────┬────────────────────────────────────────────┤
│ Theoretical Foundation (Prof. Saoud et al.)│ High-Performance Computing (RTX 5080 / CPU)│
├───────────────────────────────────────────┼────────────────────────────────────────────┤
│ • Local Subsystems: 1D room models        │ • CUDA Kernel: 1 Block per room            │
│ • Assumption (Ai): Neighbors safe         │ • OpenMP CPU: #pragma omp parallel for     │
│ • Guarantee (Gi): Room i remains safe     │ • SIMD bitset vectorization                │
│ • Circular Contract Composition Theorem   │ • Linear Scaling: N=100 in < 10 ms         │
└───────────────────────────────────────────┴────────────────────────────────────────────┘
```

### The Saoud-Girard-Fribourg Contract Formulation
1. **Assumption ($A_i$)**: The environment inputs (neighbor rooms $i-1$ and $i+1$) stay within $[T_{\text{safe, min}}, T_{\text{safe, max}}]^2$.
2. **Guarantee ($G_i$)**: Room $i$ maintains its own temperature $T_i \in [T_{\text{safe, min}}, T_{\text{safe, max}}]$ using local policy $u_i = C_i(T_i)$.
3. **Circular Composition**: Since the topology is symmetric, if every local room satisfies its contract under $A_i$, the interconnected network provably satisfies global safety without ever computing a global grid!

---

## 4. Team Task Repartition (5 Members)

### Member 1: Coupled Thermal Dynamics & Monotonicity Proofs
- Implement the continuous $N$-dimensional vector ODE in `include/circular_network.hpp`.
- Mathematically prove that the Jacobian $J = \frac{\partial \dot{T}}{\partial T}$ is a **Metzler matrix** ($\frac{\partial \dot{T}_i}{\partial T_j} = \beta \ge 0$ for $j \neq i$).
- Prove the **Kamke-Müller order-preservation theorem**:
  $$x(0) \le y(0) \implies x(t) \le y(t) \quad \forall t \ge 0$$
- Document how this enables extreme-corner trajectory propagation for continuous formal verification.

### Member 2: Local Subsystem Abstraction & Contract Boundaries
- Formulate the local 1D state interval for room $i$ in `include/contract_abstraction.hpp`.
- Formalize neighbor temperature interactions as bounded disturbances:
  $$w_{i,\text{coupling}} = \beta (T_{i-1} + T_{i+1}) \in [2\beta T_{\text{safe, min}}, \; 2\beta T_{\text{safe, max}}]$$
- Compute the discrete local transition set $Post_i(s_i, u_i)$ under the combined disturbance $(T_e, w_{i,\text{coupling}})$.

### Member 3: Assume-Guarantee Synthesis & Contract Verifier
- Implement the local contract safety synthesis solver in `include/contract_solver.hpp`.
- Synthesize the local maximal invariant set $R_i^*$ and deterministic policy $C_i(T_i)$.
- Implement the circular contract compatibility checker: formally verify that $\prod_{i=0}^{N-1} G_i \implies \prod_{i=0}^{N-1} A_i$.

### Member 4: C++ HPC Architecture (OpenMP 24-Thread & CUDA Acceleration)
- Implement multi-threaded synthesis across all $N$ rooms with OpenMP in `src/compositional_cpu.cpp`:
  ```cpp
  #pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < N; ++i) {
      synthesize_local_contract(i, ...);
  }
  ```
- Implement the CUDA acceleration kernel in `src/compositional_cuda.cu` for the RTX 5080, dispatching contract synthesis across thousands of threads.
- Implement the naive monolithic baseline solver in `src/monolithic_solver.cpp` (to produce the comparison graph showing other methods crashing at $N=5$).

### Member 5: Competition Benchmark Suite & Live Ring Dashboard
- Build the automated benchmark script measuring **Execution Time vs. $N$** and **Memory vs. $N$** in `src/main_building.cpp`.
- Generate the **competition winning plots**:
  - Monolithic: Exploding exponentially at $N=1, 2, 3, 4$ and crashing at $N=5$.
  - Compositional: Flat linear line for $N = 3, 4, 8, 16, 32, 64, 100, 1000$.
- Build a live interactive GUI (Dear ImGui + OpenGL):
  - 60 FPS circular heatmap of all $N$ rooms.
  - Live toggling of room heaters and real-time disturbance injection.

---

## 5. Modular Code Layout

```
3_circular_building/
├── include/
│   ├── circular_network.hpp   # Member 1: Coupled ODEs & monotonicity proofs
│   ├── contract_abstraction.hpp # Member 2: Local abstraction & assumptions
│   ├── contract_solver.hpp    # Member 3: Assume-Guarantee solver
│   ├── compositional_cpu.hpp  # Member 4: OpenMP parallel engine
│   └── compositional_cuda.cuh # Member 4: CUDA GPU accelerated kernel
├── src/
│   ├── monolithic_solver.cpp  # Baseline (crashes at N=5)
│   ├── compositional_cpu.cpp  # Scalable CPU engine
│   ├── compositional_cuda.cu  # Scalable GPU engine
│   └── main_building.cpp      # Member 5: Benchmarks & live simulation
└── CMakeLists.txt             # Modular build with OpenMP & CUDA
```

---

## 6. Definition of Done (The Winning Demo)
1. Monolithic baseline solver demonstrates exponential memory explosion on $N=1..4$ and failure at $N=5$.
2. Compositional Assume-Guarantee solver executes for $N=3, 8, 16, 32, 100,$ and $1000$ rooms in **under 50 milliseconds**.
3. Continuous simulation validates 100% safety satisfaction across all $N$ rooms under severe weather fluctuations.
4. Exported comparison graphs and live interactive circular dashboard ready for the project defense.
