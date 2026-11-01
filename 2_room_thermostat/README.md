# Project 2: Room Thermostat (Safety Invariance)

## 1. System & Goal

We control a heater in a room to keep the temperature comfortable despite changing outside weather:

$$
\dot{T}(t) = -\alpha \big(T(t) - T_e(t)\big) + \beta u(t)
$$

$$
T(t+1) = (1 - \alpha \tau) T(t) + \alpha \tau T_e(t) + \beta \tau u(t)
$$

* **Sample time**: $\tau = 1.0\text{ s}$.
* **Constants**: Heat loss $\alpha = 0.05\text{ s}^{-1}$, heater power $\beta = 1.0\text{ K/s}$.
* **Domains**: Room temperature $T \in [15.0, 25.0]^\circ\text{C}$, outside weather $T_e \in [5.0, 12.0]^\circ\text{C}$, heater $u \in \{0, 1\}$ (0 = OFF, 1 = ON).
* **Grid**: 50 intervals of width $0.2^\circ\text{C}$ over $[15, 25]^\circ\text{C}$.
  * Cell $k \in [0, 49]$ covers $[15.0 + 0.2k, \; 15.0 + 0.2(k+1)]^\circ\text{C}$.
  * Cell $k = 50$ is the unsafe/out-of-bounds sink state.
* **Safe Target**: Keep temperature inside $[19.0, 22.0]^\circ\text{C}$ forever.
  * Corresponds to cell indices $k \in [20, 34]$ (15 safe cells).
* **Zero Dependencies**: Self-contained folder that builds independently.

---

## 2. Why We Do Each Step (Plain English)

1. **Why the thermal ODE?**
   Heat naturally leaks out through the walls when it's colder outside ($-\alpha(T - T_e)$), and the heater injects warmth ($+\beta u$). Without this model, we can't calculate when to switch the heater on.
2. **Why 50 discrete intervals?**
   Continuous temperature has infinitely many real values. We divide $[15, 25]^\circ\text{C}$ into 50 small intervals so we can treat the problem as a finite graph.
3. **Why transition sets ($Post$)?**
   We don't know the exact outside weather $T_e$; we only know it stays between 5°C and 12°C. From any given interval, depending on what the weather does, the temperature can land in several different next intervals. $Post(s, u)$ is the set of all those possible next intervals.
4. **Why the predecessor operator ($Pre$)?**
   We ask: *"From which intervals can we pick an action (heater ON or OFF) such that EVERY possible weather outcome lands in a safe interval?"* If even a single weather value pushes the temperature out of the safe set, that interval cannot be in $Pre(R)$.
5. **Why Algorithm 1 fixed-point?**
   Being safe for 1 step isn't enough. An interval might be safe for 1 minute, but unavoidably drift into the cold 2 minutes later. Algorithm 1 repeatedly removes intervals that lead into future traps ($R_{k+1} = R_k \cap Pre(R_k)$) until only permanently safe intervals remain (the maximal invariant set $R^*$).
6. **Why a tie-breaker?**
   If both heater OFF ($u=0$) and heater ON ($u=1$) keep the room safe, we choose OFF to save energy.

---

## 3. Member Tasks (What each function does and how)

### Member 1: ODE & Boundary Invariance Check
* **File**: `include/thermostat_model.hpp`
* **Constants**: `ALPHA = 0.05, BETA = 1.0, TAU = 1.0, TE_MIN = 5.0, TE_MAX = 12.0`
* **Functions**:
  1. `double step_euler(double T, int u, double Te, double dt)`
     * **What**: Steps continuous temperature forward by 1 second.
     * **How**: Calculates $T_{t+1} = (1 - \alpha \cdot dt) T_t + \alpha \cdot dt \cdot T_e + \beta \cdot dt \cdot u$.
  2. `std::pair<double, double> step_interval(double T_min, double T_max, int u, double Te_min, double Te_max, double dt)`
     * **What**: Returns the min and max temperature reachable from an interval under all possible weather.
     * **How**: By monotonicity, the lowest temperature comes from $(T_{\min}, T_{e,\min})$ and the highest from $(T_{\max}, T_{e,\max})$.
  3. `bool verify_continuous_boundary_invariance()`
     * **What**: Mathematical sanity check on the boundary limits.
     * **How**: Checks that at 19°C with heater ON, $\dot{T} = -0.05(19 - 5) + 1.0 = +0.30 > 0$ (heats up), and at 22°C with heater OFF, $\dot{T} = -0.05(22 - 12) = -0.50 < 0$ (cools down).

### Member 2: Interval Discretizer & Transition Sets
* **File**: `include/interval_abstraction.hpp`
* **Constants**: `M = 50, T_MIN = 15.0, DT = 0.2`
* **Functions**:
  1. `int quantize(double T)` / `std::pair<double, double> cell_bounds(int k)`
     * **What**: Converts temperature $T$ to cell index $k \in [0, 49]$, and vice versa.
     * **How**: $k = \lfloor (T - 15.0) / 0.2 \rfloor$. If outside $[15, 25]$, returns sink index 50.
  2. `std::vector<int> compute_post_cells(int cell_id, int u)`
     * **What**: Finds all discrete cells that can be reached from cell $k$ under action $u$.
     * **How**: Calls Member 1's `step_interval` on the cell's bounds with weather in $[5, 12]$, quantizes the resulting $[\underline{T}', \overline{T}']$, and returns all cell IDs in that range.

### Member 3: Predecessor Operator & Algorithm 1 Solver
* **Files**: `include/safety_solver.hpp`, `src/safety_solver.cpp`
* **Initialization**: $R_0 = \{20, 21, \dots, 34\}$ (the 15 safe cells covering $[19, 22]^\circ\text{C}$).
* **Functions**:
  1. `Set pre_operator(const Set& R, const TransitionTable& post_table)`
     * **What**: Computes $Pre(R)$.
     * **How**: For each cell $s$, checks if there exists $u \in \{0, 1\}$ such that all cells in $Post(s, u)$ belong to $R$.
  2. `Set solve_maximal_invariant_set(const Set& S_safe, ...)`
     * **What**: Runs Algorithm 1 to find the permanent safe set $R^*$.
     * **How**: Starts with $R = S_{\text{safe}}$. Repeats $R = R \cap Pre(R)$ until $R$ stops shrinking.
  3. `void extract_controller(const Set& R_star, int controller_table[])`
     * **What**: Generates the control lookup table.
     * **How**: For each cell in $R^*$, picks $u=0$ if safe (saves power), otherwise $u=1$.

### Member 4: Bitboard Optimization & AVX2
* **File**: `include/bitset_solver.hpp`
* **Mask**: `const uint64_t SAFE_MASK = ((1ULL << 15) - 1) << 20;` (bits 20 through 34 set).
* **Functions**:
  1. `uint64_t compute_pre_bitset(uint64_t R_mask, const uint64_t post_masks[][2])`
     * **What**: Computes $Pre(R)$ using 64-bit integer bitwise operations.
     * **How**: For each cell $s$, tests `(post_masks[s][u] & ~R_mask) == 0`. If true for $u=0$ or $u=1$, sets bit $s$.
  2. `uint64_t solve_fixed_point_bitset(uint64_t S_safe_mask, ...)`
     * **What**: Solves Algorithm 1 in under 50 microseconds.
     * **How**: Loops `while ((R_next = R & compute_pre_bitset(R, ...)) != R) R = R_next;`. Uses `_mm_popcnt_u64` to verify convergence.

### Member 5: 24-Hour Simulation & Verification
* **File**: `src/main_thermostat.cpp`
* **Functions**:
  1. `void run_simulation(double T0, double duration_sec, const char* csv_out)`
     * **What**: Simulates the closed-loop room for 24 hours under variable weather.
     * **How**: At each second, computes $T_e(t) = 8.5 + 3.5\sin(2\pi t / 86400)$, looks up $u = \text{controller}[\text{quantize}(T)]$, calls Member 1's `step_euler()`, and logs to CSV.
  2. `bool assert_safety_guarantees(const char* csv_in)`
     * **What**: Verifies zero safety violations.
     * **How**: Reads the CSV and asserts $19.0 \le T(t) \le 22.0$ for all 86,400 simulated seconds.

---

## 4. Build & Run

```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/room_thermostat
```
