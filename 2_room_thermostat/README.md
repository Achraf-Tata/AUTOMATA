# Project 2: Room Thermostat (Safety Invariance)

## 1. System Model & Specification
* **Dynamics**:
  $$\dot{T}(t) = -\alpha \big(T(t) - T_e(t)\big) + \beta \cdot u(t)$$
  $$T(t+1) = (1 - \alpha \tau) T(t) + \alpha \tau \cdot T_e(t) + \beta \tau \cdot u(t)$$
* **Parameters**: $\alpha = 0.05\text{ s}^{-1}$, $\beta = 1.0\text{ K/s}$, $\tau = 1.0\text{ s}$.
* **Domains**: State $T \in [15, 25]^\circ\text{C}$, disturbance $T_e \in [5, 12]^\circ\text{C}$, control $u \in \{0, 1\}$.
* **Safety Specification**: Maintain $T(t) \in [19.0, 22.0]^\circ\text{C}$ for all $t \ge 0$.

---

## 2. Exact Member Task Assignments

### Member 1: Thermal ODE & Continuous Boundary Stability
* **File to produce**: `include/thermostat_model.hpp`
* **Exact Tasks**:
  1. Implement `double step_euler(double T, int u, double Te, double dt)`.
  2. Implement `std::pair<double, double> step_interval(double T_min, double T_max, int u, double Te_min, double Te_max, double dt)`.
  3. Validate continuous boundary invariants:
     - Check $\dot{T} > 0$ at $T = 19.0$ with $u = 1$ under worst-case $T_{e,\min} = 5.0$.
     - Check $\dot{T} < 0$ at $T = 22.0$ with $u = 0$ under worst-case $T_{e,\max} = 12.0$.

### Member 2: Interval Discretizer & Transition Graph
* **File to produce**: `include/interval_abstraction.hpp`
* **Exact Tasks**:
  1. Partition $[15, 25]^\circ\text{C}$ into $M = 50$ intervals ($0.2^\circ\text{C}$ each) + 1 unsafe sink state.
  2. Implement `int quantize(double T)` and `std::pair<double, double> cell_bounds(int cell_id)`.
  3. Precompute transition set `std::vector<int> post(int cell_id, int u)` under disturbance interval $T_e \in [5, 12]$.

### Member 3: Predecessor Operator & Algorithme 1 Solver
* **Files to produce**: `include/safety_solver.hpp`, `src/safety_solver.cpp`
* **Exact Tasks**:
  1. Implement the predecessor operator:
     $$Pre(R) = \left\{ s \in S \;\middle|\; \exists u \in \{0, 1\}, \; Post(s, u) \subseteq R \right\}$$
  2. Implement **Algorithme 1**:
     ```cpp
     Set R = S_safe;
     while (true) {
         Set R_next = intersect(R, Pre(R));
         if (R_next == R) break;
         R = R_next;
     }
     return R; // Maximal Controlled Invariant Set R*
     ```
  3. Output lookup table: `int controller[M]` where $u^*(s) \in \{0, 1\}$.

### Member 4: Bitboard Optimization & AVX2 Acceleration
* **File to produce**: `include/bitset_solver.hpp`
* **Exact Tasks**:
  1. Represent sets using `uint64_t` bitboards (bit $i = 1$ if cell $i \in R$).
  2. Encode `is_subset(Post(s, u), R)` as: `(post_mask[s][u] & ~R_mask) == 0`.
  3. Benchmark fixed-point loop convergence (target: $< 50\,\mu\text{s}$ on Intel Core Ultra 9).

### Member 5: Closed-Loop Simulation & Trace Verification
* **File to produce**: `src/main_thermostat.cpp`
* **Exact Tasks**:
  1. Simulate closed-loop operation for $86,400$ s (24 hours) with variable weather:
     $$T_e(t) = 8.5 + 3.5\sin\left(\frac{2\pi t}{86400}\right)$$
  2. Record $(t, T(t), u(t), T_e(t))$ to `thermostat_sim.csv`.
  3. Assert 0 safety violations: $19.0 \le T(t) \le 22.0$ for all $t$.

---

## 3. Build & Run
```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/room_thermostat
```
