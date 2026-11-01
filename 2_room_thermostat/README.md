# Project 2: Room Thermostat (Safety Invariance)

## 1. System Model & Specification

Continuous thermal ODE and discrete-time Euler update with sampling period $\tau = 1.0\text{ s}$:

$$
\dot{T}(t) = -\alpha \big(T(t) - T_e(t)\big) + \beta u(t)
$$

$$
T(t+1) = (1 - \alpha \tau) T(t) + \alpha \tau T_e(t) + \beta \tau u(t)
$$

### Exact Numerical Discretization
* **Physical Parameters**: $\alpha = 0.05\text{ s}^{-1}$, $\beta = 1.0\text{ K/s}$, $\tau = 1.0\text{ s}$.
* **Domains**: State $T \in [15.0, 25.0]^\circ\text{C}$, disturbance $T_e \in [5.0, 12.0]^\circ\text{C}$, control $u \in \{0, 1\}$.
* **Grid Resolution ($M = 50$ cells, $\Delta T = 0.2^\circ\text{C}$)**:
  * Cell index $k \in [0, 49]$ covers temperature interval $[15.0 + 0.2k, \; 15.0 + 0.2(k+1)]^\circ\text{C}$.
  * Cell index $k = 50$ represents the out-of-bounds unsafe sink state.
* **Safe Target Interval ($[19.0, 22.0]^\circ\text{C}$)**:
  * Safe cell indices: $k \in [20, 34]$ (exactly 15 cells).
  * Index formula: $k_{\text{start}} = \frac{19.0 - 15.0}{0.2} = 20$, $k_{\text{end}} = \frac{22.0 - 15.0}{0.2} - 1 = 34$.
* **Zero Dependencies**: This project is 100% self-contained. It compiles and runs independently with no dependencies on other folders.

---

## 2. Exact Member Task Breakdown (What & How)

### Member 1: Thermal ODE & Continuous Boundary Stability
* **File to produce**: `include/thermostat_model.hpp`
* **Constants**:
  * `const double ALPHA = 0.05; const double BETA = 1.0; const double TAU = 1.0;`
  * `const double TE_MIN = 5.0; const double TE_MAX = 12.0;`
* **Functions to implement**:
  1. `double step_euler(double T, int u, double Te, double dt)`
     * **What it does**: Computes continuous room temperature at step $t+1$.
     * **How it does it**: Evaluates forward Euler formula: $T_{t+1} = (1 - \alpha \cdot dt) T_t + \alpha \cdot dt \cdot T_e + \beta \cdot dt \cdot u$.
  2. `std::pair<double, double> step_interval(double T_min, double T_max, int u, double Te_min, double Te_max, double dt)`
     * **What it does**: Computes the exact reachable interval $[\underline{T}', \overline{T}']$ for an interval of continuous temperatures under all weather disturbances.
     * **How it does it**: By monotonicity: $\underline{T}' = (1 - \alpha \cdot dt) \underline{T} + \alpha \cdot dt \cdot \underline{T}_e + \beta \cdot dt \cdot u$ and $\overline{T}' = (1 - \alpha \cdot dt) \overline{T} + \alpha \cdot dt \cdot \overline{T}_e + \beta \cdot dt \cdot u$.
  3. `bool verify_continuous_boundary_invariance()`
     * **What it does**: Analytically proves that the continuous boundary $[19.0, 22.0]^\circ\text{C}$ can never be breached.
     * **How it does it**: Asserts that at $T = 19.0$, $\dot{T}(19, u=1, T_e=5.0) = -0.05(19 - 5) + 1.0 = +0.30 > 0$, and at $T = 22.0$, $\dot{T}(22, u=0, T_e=12.0) = -0.05(22 - 12) = -0.50 < 0$.

### Member 2: Interval Discretizer & Transition Graph
* **File to produce**: `include/interval_abstraction.hpp`
* **Constants**:
  * `const int M = 50; const double T_MIN = 15.0; const double DT = 0.2;`
* **Functions to implement**:
  1. `int quantize(double T)` / `std::pair<double, double> cell_bounds(int cell_id)`
     * **What it does**: Converts continuous temperature to discrete cell index $k \in [0, 49]$, and returns continuous interval bounds $[15.0 + 0.2k, 15.0 + 0.2(k+1)]$.
     * **How it does it**: Computes $k = \lfloor (T - 15.0) / 0.2 \rfloor$. If $k < 0$ or $k \ge 50$, returns sink index 50.
  2. `std::vector<int> compute_post_cells(int cell_id, int u)`
     * **What it does**: Computes all successor discrete cells reachable from cell $k$ under action $u \in \{0, 1\}$.
     * **How it does it**: Calls Member 1's `step_interval` on cell bounds with $T_e \in [5.0, 12.0]$, quantizes resulting $[\underline{T}', \overline{T}']$, and returns all integer cell IDs in range $[c_{\min}', c_{\max}']$. If bounds exceed $[15, 25]$, includes sink index 50.

### Member 3: Predecessor Operator & Algorithme 1 Solver
* **Files to produce**: `include/safety_solver.hpp`, `src/safety_solver.cpp`
* **Initialization**: Set $R_0 = S_{\text{safe}} = \{20, 21, \dots, 34\}$ (indices 20 to 34).
* **Control Tie-Breaker**: If both $u=0$ and $u=1$ are safe, select $u=0$ to minimize heating power.
* **Functions to implement**:
  1. `Set pre_operator(const Set& R, const TransitionTable& post_table)`
     * **What it does**: Computes the set of states from which the controller can force the system to stay inside $R$.
     * **How it does it**: For each state $s \in [20, 34]$, checks if there exists $u \in \{0, 1\}$ such that for all $s' \in Post(s, u)$, $s' \in R$. If true, adds $s$ to $Pre(R)$.
  2. `Set solve_maximal_invariant_set(const Set& S_safe, ...)`
     * **What it does**: Executes **Algorithme 1** to compute the maximal controlled invariant set $R^*$.
     * **How it does it**: Initializes $R = S_{\text{safe}}$. Iteratively computes $R_{\text{next}} = R \cap Pre(R)$. Terminates when $R_{\text{next}} == R$, returning $R^*$.
  3. `void extract_controller(const Set& R_star, int controller_table[])`
     * **What it does**: Extracts deterministic control policy $u^*(s)$.
     * **How it does it**: For each state $s \in R^*$:
       * If $Post(s, 0) \subseteq R^*$, assign $u^*(s) = 0$ (prioritize energy saving).
       * Else if $Post(s, 1) \subseteq R^*$, assign $u^*(s) = 1$.
       * Else assign $-1$ (uncontrollable).

### Member 4: Bitboard Optimization & AVX2 Acceleration
* **File to produce**: `include/bitset_solver.hpp`
* **Bitmask Constants**:
  * `const uint64_t SAFE_MASK = ((1ULL << 15) - 1) << 20;` (sets bits 20 through 34).
* **Functions to implement**:
  1. `uint64_t compute_pre_bitset(uint64_t R_mask, const uint64_t post_masks[][2])`
     * **What it does**: Computes $Pre(R)$ in microseconds using hardware bitboards.
     * **How it does it**: For each state $s \in [20, 34]$:
       * `bool safe_u0 = ((post_masks[s][0] & ~R_mask) == 0);`
       * `bool safe_u1 = ((post_masks[s][1] & ~R_mask) == 0);`
       * Sets bit $s$ in output mask if `safe_u0 || safe_u1`.
  2. `uint64_t solve_fixed_point_bitset(uint64_t S_safe_mask, ...)`
     * **What it does**: Runs the entire Algorithme 1 loop in $< 50\,\mu\text{s}$.
     * **How it does it**: `while ((R_next = R & compute_pre_bitset(R, ...)) != R) R = R_next;`. Uses `_mm_popcnt_u64` to verify convergence in 1 clock cycle.

### Member 5: Closed-Loop Simulation & Trace Verification
* **File to produce**: `src/main_thermostat.cpp`
* **Functions to implement**:
  1. `void run_simulation(double T0, double duration_sec, const char* csv_out)`
     * **What it does**: Simulates closed-loop 24-hour operation under ambient weather variations.
     * **How it does it**: At each 1-second step, calculates $T_e(t) = 8.5 + 3.5\sin(2\pi t / 86400)$, quantizes continuous $T(t) \to s$, retrieves $u = \text{controller}[s]$, integrates forward via Member 1's `step_euler()`, and logs $(t, T(t), u, T_e(t))$ to CSV.
  2. `bool assert_safety_guarantees(const char* csv_in)`
     * **What it does**: Formally verifies that 0 safety violations occurred.
     * **How it does it**: Reads recorded temperatures and asserts $19.0 \le T(t) \le 22.0$ for all $86,400$ recorded points.

---

## 3. Build & Run
```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/room_thermostat
```
