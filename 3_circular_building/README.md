# Project 3: Circular Building (Scalable Challenge)

## 1. System Model & Specification

Coupled thermal ODE for $N$ rooms $i \in \{0, \dots, N-1\}$ with cyclic indexing ($i \pm 1 \pmod N$):

$$
\dot{T}_i(t) = -\alpha \big(T_i(t) - T_e(t)\big) - \beta \Big(\big(T_i(t) - T_{i-1}(t)\big) + \big(T_i(t) - T_{i+1}(t)\big)\Big) + \gamma u_i(t)
$$

* **Parameters**: Heat loss $\alpha = 0.05$, conduction $\beta = 0.02$, heater power $\gamma = 1.0$.
* **Domains**: State $T_i \in [15.0, 25.0]^\circ\text{C}$, weather disturbance $T_e \in [5.0, 12.0]^\circ\text{C}$, heater $u_i \in \{0, 1\}$.
* **Global Specification**: $\forall i \in \{0, \dots, N-1\}, \; \forall t \ge 0, \; T_i(t) \in [19.0, 22.0]^\circ\text{C}$.
* **Challenge Goal**: Avoid naive $O(G^N)$ exponential explosion by implementing $O(N)$ Assume-Guarantee contracts scaling to $N \in [3, 1000]$.

---

## 2. Exact Member Task Assignments

### Member 1: Coupled Physics & Metzler Matrix Monotonicity
* **File to produce**: `include/circular_network.hpp`
* **Exact Tasks**:
  1. Implement $N$-dimensional vector ODE integration with cyclic wrap-around ($i-1$ and $i+1 \pmod N$).
  2. Implement Kamke-Müller extreme trajectory bounding: propagate continuous lower bound $\underline{T}(t)$ and upper bound $\overline{T}(t)$ in $O(N)$ operations.
  3. Validate mathematical monotonicity: verify Jacobian off-diagonal elements $\frac{\partial \dot{T}_i}{\partial T_j} = \beta \ge 0$ for all $j \neq i$.

### Member 2: Contract Interval Abstraction
* **File to produce**: `include/contract_abstraction.hpp`
* **Exact Tasks**:
  1. Define local room assumption: $A_i = (T_{i-1}, T_{i+1}) \in [19.0, 22.0]^2$.
  2. Compute bounded coupling disturbance:

$$
w_{i,\text{coupling}} = \beta (T_{i-1} + T_{i+1}) \in [2\beta \cdot 19.0, \; 2\beta \cdot 22.0]
$$

  3. Discretize 1D room interval into $M = 50$ cells and precompute local transition set $Post_i(s_i, u_i, w_{i,\text{coupling}})$.

### Member 3: Assume-Guarantee Contract Synthesis
* **Files to produce**: `include/contract_solver.hpp`, `src/contract_solver.cpp`
* **Exact Tasks**:
  1. Implement local safety invariance fixed-point for room $i$ under assumption $A_i$.
  2. Synthesize local invariant set $R_i^*$ and deterministic policy $u_i = C_i(T_i)$.
  3. Formulate circular contract composition check: verify that local guarantee $G_i \subseteq [19.0, 22.0]$ satisfies neighbors' assumption $A_{i \pm 1}$.

### Member 4: HPC Dual Backend (OpenMP & CUDA Acceleration)
* **Files to produce**: `src/compositional_cpu.cpp`, `src/compositional_cuda.cu`, `src/monolithic_baseline.cpp`
* **Exact Tasks**:
  1. Implement monolithic baseline (`src/monolithic_baseline.cpp`) showing memory crash at $N=5$.
  2. Implement OpenMP multi-room parallelization across 24 CPU threads:
     ```cpp
     #pragma omp parallel for schedule(dynamic)
     for (int i = 0; i < N; ++i) synthesize_room_contract(i, ...);
     ```
  3. Implement CUDA kernel (`src/compositional_cuda.cu`) on RTX 5080 dispatching 1 thread block per room.
  4. Benchmark target: Solve $N = 100$ rooms in $< 10\text{ ms}$.

### Member 5: Benchmark Suite & Multi-Room Heatmap
* **File to produce**: `src/main_building.cpp`
* **Exact Tasks**:
  1. Run automated benchmark comparing Monolithic vs. Assume-Guarantee runtime for $N \in \{3, 4, 8, 16, 32, 64, 100, 1000\}$.
  2. Export runtime and memory data to `benchmark_results.csv`.
  3. Simulate closed-loop 24-hr multi-room thermal network and export state traces.
  4. Assert 0 violations across all $N$ rooms.

---

## 3. Build & Run
```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/circular_building
```
