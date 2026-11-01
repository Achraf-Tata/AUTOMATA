# Project 3: Circular Building (Scalable Challenge)

## 1. System Model & Specification

Coupled thermal ODE for $N$ rooms $i \in \{0, \dots, N-1\}$ with cyclic indexing ($i \pm 1 \pmod N$):

$$
\dot{T}_i(t) = -\alpha \big(T_i(t) - T_e(t)\big) - \beta \Big(\big(T_i(t) - T_{i-1}(t)\big) + \big(T_i(t) - T_{i+1}(t)\big)\Big) + \gamma u_i(t)
$$

### Grouped Discrete-Time Equation for Room $i$
Under the Assume-Guarantee contract, neighbor rooms are assumed to remain safe: $A_i: (T_{i-1}, T_{i+1}) \in [19.0, 22.0]^2$.

With constants $\alpha = 0.05$, $\beta = 0.02$, $\gamma = 1.0$, and $\tau = 1.0\text{ s}$:
* Conduction coefficient: $1 - (\alpha + 2\beta)\tau = 1 - (0.05 + 0.04) = 0.91$.
* Ambient disturbance term: $\alpha \tau T_e \in [0.05 \cdot 5.0, \; 0.05 \cdot 12.0] = [0.25, 0.60]$.
* Coupling disturbance term: $w_{i,\text{coupling}} = \beta \tau (T_{i-1} + T_{i+1}) \in [0.02 \cdot 38.0, \; 0.02 \cdot 44.0] = [0.76, 0.88]$.
* **Total Combined Disturbance**: $W_i = \alpha \tau T_e + w_{i,\text{coupling}} \in [0.25 + 0.76, \; 0.60 + 0.88] = [1.01, 1.48]$.

This yields the exact decoupled 1D discrete update for room $i$:

$$
T_i(t+1) = 0.91 \cdot T_i(t) + W_i + 1.0 \cdot u_i(t), \quad W_i \in [1.01, 1.48]
$$

* **Local Discretization**: Domain $[15.0, 25.0]^\circ\text{C}$ with $M = 50$ cells ($\Delta T = 0.2^\circ\text{C}$). Safe cells are $k \in [20, 34]$.
* **Global Specification**: $\forall i \in \{0, \dots, N-1\}, \; \forall t \ge 0, \; T_i(t) \in [19.0, 22.0]^\circ\text{C}$.
* **Zero Dependencies**: This project is 100% self-contained. It contains its own internal local contract abstraction and solvers with no dependencies on other folders.

---

## 2. Exact Member Task Breakdown (What & How)

### Member 1: Coupled Physics & Metzler Matrix Monotonicity
* **File to produce**: `include/circular_network.hpp`
* **Constants**:
  * `const double ALPHA = 0.05; const double BETA = 0.02; const double GAMMA = 1.0;`
* **Functions to implement**:
  1. `void step_network_euler(std::vector<double>& T, const std::vector<int>& u, double Te, double dt)`
     * **What it does**: Integrates all $N$ coupled room temperatures forward by 1 time step with periodic boundary conditions.
     * **How it does it**: For each room $i$, evaluates left neighbor $T_{(i - 1 + N) \pmod N}$ and right neighbor $T_{(i + 1) \pmod N}$. Evaluates $\dot{T}_i$ and applies forward Euler update: $T_i \leftarrow T_i + dt \cdot \dot{T}_i$.
  2. `void propagate_extreme_trajectories(std::vector<double>& T_lower, std::vector<double>& T_upper, ...)`
     * **What it does**: Bounds the continuous trajectory continuum of all $N$ rooms by propagating only 2 extreme points.
     * **How it does it**: By Kamke-Müller theorem for Metzler systems, simulates $\underline{T}$ with worst-case cold disturbance $T_{e,\min}$ and heaters OFF, and $\overline{T}$ with worst-case warm disturbance $T_{e,\max}$ and heaters ON. Complexity is strictly $O(N)$.
  3. `bool verify_metzler_property()`
     * **What it does**: Mathematically verifies that the network Jacobian is a cooperative Metzler matrix.
     * **How it does it**: Checks that off-diagonal Jacobian elements $\frac{\partial \dot{T}_i}{\partial T_{i \pm 1}} = \beta = 0.02 > 0$, ensuring trajectory order preservation.

### Member 2: Contract Interval Abstraction
* **File to produce**: `include/contract_abstraction.hpp`
* **Constants**:
  * `const double W_MIN = 1.01; const double W_MAX = 1.48;`
  * `const double COEFF_T = 0.91; const double COEFF_U = 1.0;`
* **Functions to implement**:
  1. `std::pair<double, double> step_local_interval(double T_min, double T_max, int u)`
     * **What it does**: Computes continuous reachability interval for room $i$ under total disturbance $W_i \in [1.01, 1.48]$.
     * **How it does it**: $\underline{T}' = 0.91 \cdot \underline{T} + 1.01 + 1.0 \cdot u$ and $\overline{T}' = 0.91 \cdot \overline{T} + 1.48 + 1.0 \cdot u$.
  2. `std::vector<int> compute_local_post(int cell_id, int u)`
     * **What it does**: Precomputes discrete 1D transition sets for an individual room.
     * **How it does it**: Evaluates `step_local_interval` on cell bounds $[15.0 + 0.2k, 15.0 + 0.2(k+1)]$, quantizes resulting $[\underline{T}', \overline{T}']$, and returns all integer cell IDs in range $[c_{\min}', c_{\max}']$.

### Member 3: Assume-Guarantee Contract Synthesis
* **Files to produce**: `include/contract_solver.hpp`, `src/contract_solver.cpp`
* **Initialization**: Set $R_0 = \{20, 21, \dots, 34\}$.
* **Functions to implement**:
  1. `Set synthesize_local_room_contract(int room_id, const LocalAbstraction& abs)`
     * **What it does**: Synthesizes the local invariant set $R_i^*$ and policy $C_i(T_i)$ for room $i$ under assumption $A_i$.
     * **How it does it**: Solves the 1D safety fixed-point loop $R_{k+1} = R_k \cap Pre(R_k)$ using Member 2's local transitions. Complexity is $O(M)$ where $M=50$ cells.
     * **Tie-Breaker**: If both $u_i=0$ and $u_i=1$ are safe, select $u_i=0$ (energy minimization).
  2. `bool verify_circular_contract_compatibility(const std::vector<Set>& local_invariants)`
     * **What it does**: Mathematically proves that the composition of local contracts guarantees global building safety.
     * **How it does it**: Formally checks that the local guarantee $G_i = [19.0, 22.0]$ strictly satisfies neighbor assumptions: $G_i \subseteq A_{i-1}$ and $G_i \subseteq A_{i+1}$ for all $i \in \{0, \dots, N-1\}$.

### Member 4: HPC Dual Backend (OpenMP & CUDA Acceleration)
* **Files to produce**: `src/compositional_cpu.cpp`, `src/compositional_cuda.cu`, `src/monolithic_baseline.cpp`
* **Functions to implement**:
  1. `void benchmark_monolithic_crash(int max_N)`
     * **What it does**: Demonstrates that naive global state discretization fails at $N \ge 5$.
     * **How it does it**: Allocates global grid $M^N$. Measures memory and synthesis time for $N=1, 2, 3, 4$. Catches `std::bad_alloc` at $N=5$ and logs memory explosion.
  2. `void synthesize_compositional_openmp(int N, std::vector<Controller>& controllers)`
     * **What it does**: Parallelizes synthesis for $N=100$ or $N=1000$ rooms across 24 CPU cores.
     * **How it does it**: Dispatches independent room solvers with `#pragma omp parallel for schedule(dynamic)`. Since each room is decoupled under its contract, parallel scaling is linear.
  3. `void synthesize_compositional_cuda(int N, ...)`
     * **What it does**: Synthesizes all $N$ room controllers in parallel on the RTX 5080 GPU in $< 5\text{ ms}$.
     * **How it does it**: Maps 1 CUDA thread block per room, computing fixed-point bitboards directly in GPU shared memory.

### Member 5: Benchmark Suite & Multi-Room Heatmap
* **File to produce**: `src/main_building.cpp`
* **Functions to implement**:
  1. `void run_scalability_benchmark(const char* csv_out)`
     * **What it does**: Compares Monolithic vs. Assume-Guarantee execution time and memory across $N \in \{3, 4, 8, 16, 32, 64, 100, 1000\}$.
     * **How it does it**: Runs both solvers with `std::chrono::high_resolution_clock`, records runtimes, and writes formatted comparison table to `benchmark_results.csv`.
  2. `void simulate_multi_room_network(int N, double duration_sec, const char* trace_out)`
     * **What it does**: Simulates the full $N$-room closed-loop thermal network under sinusoidal weather.
     * **How it does it**: At each time step, all $N$ rooms evaluate their local controller $u_i = C_i(T_i)$, update continuous temperatures via Member 1's `step_network_euler()`, and export temperature heatmaps over time.
  3. `bool assert_all_rooms_safe(const char* trace_in)`
     * **What it does**: Verifies that 100% of rooms satisfied $19.0 \le T_i(t) \le 22.0$ for the entire simulation duration.

---

## 3. Build & Run
```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/circular_building
```
