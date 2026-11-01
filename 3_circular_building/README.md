# Project 3: Circular Building (Scalable Challenge)

## 1. System & Goal

We have $N$ identical rooms arranged in a circle. Each room $i \in \{0, \dots, N-1\}$ shares interior walls with its left neighbor ($i-1$) and right neighbor ($i+1$), wrapping around with periodic boundaries ($i \pm 1 \pmod N$):

$$
\dot{T}_i(t) = -\alpha \big(T_i(t) - T_e(t)\big) - \beta \Big(\big(T_i(t) - T_{i-1}(t)\big) + \big(T_i(t) - T_{i+1}(t)\big)\Big) + \gamma u_i(t)
$$

* **Sample time**: $\tau = 1.0\text{ s}$.
* **Constants**: Wall heat loss $\alpha = 0.05$, inter-room conduction $\beta = 0.02$, heater power $\gamma = 1.0$.
* **Domains**: State $T_i \in [15.0, 25.0]^\circ\text{C}$, outside weather $T_e \in [5.0, 12.0]^\circ\text{C}$, heater $u_i \in \{0, 1\}$.
* **Global Target**: Keep every room inside $[19.0, 22.0]^\circ\text{C}$ for all time.
* **Zero Dependencies**: Self-contained folder that builds independently.

---

## 2. Why We Do Each Step (Plain English)

1. **Why multi-room coupled dynamics?**
   Rooms are not isolated. Heat leaks across shared walls through conduction ($-\beta(T_i - T_{i\pm 1})$). If your neighbor turns off their heater, your room gets colder. We have to model this coupling.
2. **Why does a naive monolithic grid fail ($50^N$ explosion)?**
   If 1 room has 50 intervals, 4 rooms need $50^4 = 6.25 \times 10^6$ cells. A 5-room building needs $50^5 = 312,500,000$ cells, which runs out of memory and crashes the computer.
3. **Why Assume-Guarantee contracts?**
   Instead of solving all $N$ rooms together in a giant grid, each room solves its own 1D problem:
   * Room $i$ **assumes** its neighbors will stay in $[19.0, 22.0]^\circ\text{C}$ ($A_i$).
   * Room $i$ **guarantees** its own heater will keep room $i$ in $[19.0, 22.0]^\circ\text{C}$ ($G_i$).
   * Because each room only solves a 1D model with 50 cells, synthesis scales linearly ($O(N \cdot 50)$) instead of exponentially.
4. **Why is the circular contract mathematically sound?**
   Normally, circular promises ("I'm safe if you're safe, and you're safe if I'm safe") can be invalid due to circular reasoning. The theorem in the paper (Saoud, Girard, Fribourg) proves that in symmetric physical networks, this mutual assumption holds formally without circular deadlocks.
5. **Why the grouped 1D discrete equation?**
   Under assumption $A_i: T_{i-1}, T_{i+1} \in [19, 22]$, neighbor heat exchange contributes $w_{\text{coupling}} = \beta \tau (T_{i-1} + T_{i+1}) \in [0.76, 0.88]$. Combining this with weather disturbance $\alpha \tau T_e \in [0.25, 0.60]$ gives a single combined disturbance range:
   $$W_i = \alpha \tau T_e + w_{\text{coupling}} \in [1.01, 1.48]$$
   This lets room $i$ step forward with a simple 1D equation:
   $$T_i(t+1) = 0.91 \cdot T_i(t) + W_i + 1.0 \cdot u_i(t)$$
6. **Why Metzler monotonicity (Kamke-Müller)?**
   Heat flows from hot to cold ($\partial \dot{T}_i / \partial T_{i\pm 1} = \beta > 0$). Trajectories preserve order: if you start colder, you stay colder. To verify an infinite continuum of temperatures, we only have to simulate the coldest possible corner $\underline{T}$ and the hottest possible corner $\overline{T}$.

---

## 3. Member Tasks (What each function does and how)

### Member 1: Coupled Network ODE & Monotonicity
* **File**: `include/circular_network.hpp`
* **Constants**: `ALPHA = 0.05, BETA = 0.02, GAMMA = 1.0`
* **Functions**:
  1. `void step_network_euler(std::vector<double>& T, const std::vector<int>& u, double Te, double dt)`
     * **What**: Moves the full $N$-room continuous simulation forward by 1 second.
     * **How**: For each room $i$, finds left neighbor $(i - 1 + N) \pmod N$ and right neighbor $(i + 1) \pmod N$, computes $\dot{T}_i$, and applies $T_i \leftarrow T_i + dt \cdot \dot{T}_i$.
  2. `void propagate_extreme_trajectories(std::vector<double>& T_lower, std::vector<double>& T_upper, ...)`
     * **What**: Bounds the temperature range of all $N$ rooms by simulating only 2 trajectories.
     * **How**: By Kamke-Müller theorem, simulates $\underline{T}$ with coldest weather and heaters OFF, and $\overline{T}$ with warmest weather and heaters ON. Complexity is $O(N)$.
  3. `bool verify_metzler_property()`
     * **What**: Sanity check that off-diagonal Jacobian terms are positive ($\beta = 0.02 > 0$).

### Member 2: Local Contract Abstraction
* **File**: `include/contract_abstraction.hpp`
* **Constants**: `W_MIN = 1.01, W_MAX = 1.48, COEFF_T = 0.91`
* **Functions**:
  1. `std::pair<double, double> step_local_interval(double T_min, double T_max, int u)`
     * **What**: Computes next interval for room $i$ under combined disturbance $W_i \in [1.01, 1.48]$.
     * **How**: $\underline{T}' = 0.91 \underline{T} + 1.01 + u$ and $\overline{T}' = 0.91 \overline{T} + 1.48 + u$.
  2. `std::vector<int> compute_local_post(int cell_id, int u)`
     * **What**: Computes successor cells for 1 room on a 50-cell grid over $[15, 25]^\circ\text{C}$.
     * **How**: Calls `step_local_interval` on cell bounds, quantizes the resulting interval, and returns all cell indices in that range.

### Member 3: Assume-Guarantee Contract Synthesis
* **Files**: `include/contract_solver.hpp`, `src/contract_solver.cpp`
* **Initialization**: $R_0 = \{20, \dots, 34\}$ (safe range $[19, 22]^\circ\text{C}$).
* **Functions**:
  1. `Set synthesize_local_room_contract(int room_id, const LocalAbstraction& abs)`
     * **What**: Synthesizes the local invariant set $R_i^*$ and policy $u_i = C_i(T_i)$ for room $i$.
     * **How**: Solves the 1D safety fixed-point loop $R = R \cap Pre(R)$ using Member 2's local transitions. Picks $u=0$ if safe, else $u=1$.
  2. `bool verify_circular_contract_compatibility(const std::vector<Set>& local_invariants)`
     * **What**: Verifies the composition condition.
     * **How**: Checks that each room's local guarantee $G_i \subseteq [19, 22]$ satisfies its neighbors' assumptions $A_{i \pm 1}$.

### Member 4: OpenMP Multi-Threading & CUDA
* **Files**: `src/compositional_cpu.cpp`, `src/compositional_cuda.cu`, `src/monolithic_baseline.cpp`
* **Functions**:
  1. `void benchmark_monolithic_crash(int max_N)`
     * **What**: Demonstrates that naive global state discretization fails at $N \ge 5$.
     * **How**: Allocates global grid $50^N$. Measures memory and runtime for $N=1, 2, 3, 4$. Catches `std::bad_alloc` at $N=5$ and logs the failure.
  2. `void synthesize_compositional_openmp(int N, std::vector<Controller>& controllers)`
     * **What**: Parallelizes synthesis for $N=100+$ rooms across 24 CPU cores.
     * **How**: Runs `#pragma omp parallel for schedule(dynamic)` across all $N$ rooms.
  3. `void synthesize_compositional_cuda(int N, ...)`
     * **What**: GPU kernel running on the RTX 5080, dispatching 1 thread block per room.

### Member 5: Benchmark Suite & Simulation
* **File**: `src/main_building.cpp`
* **Functions**:
  1. `void run_scalability_benchmark(const char* csv_out)`
     * **What**: Measures execution time and memory for $N \in \{3, 4, 8, 16, 32, 64, 100, 1000\}$ and writes to CSV.
  2. `void simulate_multi_room_network(int N, double duration_sec, const char* trace_out)`
     * **What**: Simulates the full $N$-room ring under variable weather for 24 hours.
     * **How**: At each second, every room evaluates its local controller $u_i = C_i(T_i)$, updates temperatures via Member 1's `step_network_euler()`, and logs to CSV.
  3. `bool assert_all_rooms_safe(const char* trace_in)`
     * **What**: Confirms zero violations across all $N$ rooms ($19.0 \le T_i(t) \le 22.0$).

---

## 4. Build & Run

```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/circular_building
```
