# Project 2: Room Thermostat (Safety Invariance Synthesis)

## 1. Overview & Objective
High-performance C++20 implementation of discrete interval abstraction, **Algorithme 1 (Safety Controller Synthesis / Maximal Controlled Invariant Set)**, and continuous closed-loop invariance verification for a 1D thermal system subject to persistent ambient disturbances.

* **Paper Reference**: Section 1.1 (*"Un exemple simple : le thermostat"*, p. 5) & Section 2.1 (*"Propriétés de sûreté"*, pp. 7–8) of Antoine Girard, Pierre-Jean Meyer, and Adnane Saoud (2024).
* **Target Paradigms**: Bit-parallel set algebra (`std::bitset` / `uint64_t` bitboards), AVX2 SIMD instructions, sub-millisecond fixed-point solver.
* **Target Language**: Modern C++ (C++20).

---

## 2. Mathematical System Model

### Continuous Dynamics & Euler Discretization
$$\dot{T}(t) = -\alpha \big(T(t) - T_e(t)\big) + \beta \cdot u(t)$$
With sampling step $\tau > 0$, the discrete-time forward update is:
$$T(t+1) = (1 - \alpha \tau) T(t) + \alpha \tau \cdot T_e(t) + \beta \tau \cdot u(t)$$

* **Continuous State**: Room temperature $T \in [15.0, 25.0]^\circ\text{C}$.
* **Control Input**: $u \in \{0, 1\}$ ($0 = \text{OFF}, 1 = \text{ON}$).
* **Disturbance**: External weather $T_e(t) \in [T_{e,\min}, T_{e,\max}]$ (e.g. $[5.0, 12.0]^\circ\text{C}$).
* **Physical Constants**: Heat loss $\alpha = 0.05\text{ s}^{-1}$, heating power $\beta = 1.0\text{ K/s}$, sampling time $\tau = 1.0\text{ s}$.
* **Safety Invariance Specification**:
  $$\forall t \ge 0, \quad T(t) \in [T_{\text{safe, min}}, T_{\text{safe, max}}] = [19.0, 22.0]^\circ\text{C}$$

---

## 3. Advanced Computing & Bit-Parallel Algebra

Instead of allocating memory dynamically for every state set, we represent state subsets as **Bitboards** (`uint64_t` or `std::bitset<N>`):

```
State Index:     [ 0 | 1 | 2 | 3 | 4 | 5 | ... | 63 ]
Bit Representation: 1   1   1   1   0   0           0   (1 = State in Set, 0 = Not in Set)
```

* **Set Intersection ($R_k \cap Pre(R_k)$)**: Compiles to a single CPU machine cycle bitwise `AND` (`&`).
* **Set Inclusion ($Post(s, u) \subseteq R$)**: Evaluated via bitmask checking:
  $$\big( PostMask(s, u) \;\&\; \sim R \big) == 0$$
* **Convergence Check**: Hardware population count `_mm_popcnt_u64` counts surviving states in 1 CPU cycle.
* **Result**: Complete synthesis of the maximal invariant set converges in **under 100 microseconds**.

---

## 4. Team Task Repartition (5 Members)

### Member 1: Thermal ODE & Continuous Boundary Stability
- Formulate the continuous thermal ODE in `include/thermostat_model.hpp`.
- Derive the continuous invariance boundary conditions:
  - At $T = T_{\text{safe, min}}$, verify that $u=1$ ensures $\dot{T} \ge 0$ for all $T_e \in [T_{e,\min}, T_{e,\max}]$.
  - At $T = T_{\text{safe, max}}$, verify that $u=0$ ensures $\dot{T} \le 0$ for all $T_e \in [T_{e,\min}, T_{e,\max}]$.
- Mathematically prove the condition on physical parameters ensuring the safety game is winnable:
  $$\frac{\beta}{\alpha} + T_{e,\min} \ge T_{\text{safe, min}} \quad \text{and} \quad T_{e,\max} \le T_{\text{safe, max}}$$

### Member 2: State Space Discretization & Transition Matrix
- Implement the continuous interval partition ($M$ intervals of width $\Delta T = 0.1^\circ\text{C}$) in `include/interval_abstraction.hpp`.
- Formulate the exact interval reachability mapping:
  $$[\underline{T}', \overline{T}'] = [(1 - \alpha \tau)\underline{T} + \alpha \tau \underline{T}_e + \beta \tau u, \; (1 - \alpha \tau)\overline{T} + \alpha \tau \overline{T}_e + \beta \tau u]$$
- Precompute the discrete transition relation bitmasks `transition_mask[state][u]`.

### Member 3: Predecessor Operator & Algorithme 1 Fixed Point
- Implement the predecessor operator in `include/safety_solver.hpp`:
  $$Pre(R) = \left\{ s \in S \;\middle|\; \exists u \in U, \; Post(s, u) \subseteq R \right\}$$
- Implement **Algorithme 1** (Greatest Fixed Point):
  $$\begin{aligned}
  R_0 &= S_{\text{safe}} \\
  R_{k+1} &= R_k \cap Pre(R_k) \quad \text{until } R_{k+1} = R_k = R^*
  \end{aligned}$$
- Extract the minimal-switching deterministic controller:
  $$u^*(s) = \begin{cases} 0 & \text{if } Post(s, 0) \subseteq R^* \\ 1 & \text{if } Post(s, 1) \subseteq R^* \end{cases}$$

### Member 4: C++ HPC Architecture & Bitset Vectorization
- Implement the bitboard algebra using `std::bitset<128>` and AVX2 intrinsics (`_mm256_and_si256`) in `include/bitset_algebra.hpp`.
- Ensure zero dynamic heap allocations in the critical synthesis loop.
- Provide JSON and binary serialization for the generated lookup table.

### Member 5: Simulation, Verification & Interactive Dashboard
- Build the 24-hour continuous closed-loop simulation loop in `src/main_thermostat.cpp` under sinusoidal ambient weather:
  $$T_e(t) = 8.5 + 3.5\sin\left(\frac{2\pi t}{86400}\right)$$
- Implement a real-time interactive parameter dashboard (Dear ImGui / Matplotlib):
  - Live sliders for $\alpha, \beta, T_{e,\min}, T_{e,\max}$.
  - Instantaneous visual update of the safe invariant set $R^*$ as parameters shift.
  - Continuous phase plots showing temperature staying strictly bounded in $[19.0, 22.0]^\circ\text{C}$.

---

## 5. Modular Code Layout

```
2_room_thermostat/
├── include/
│   ├── thermostat_model.hpp   # Member 1: Thermal ODE & physics
│   ├── interval_abstraction.hpp # Member 2: Discretization & transition masks
│   ├── safety_solver.hpp      # Member 3: Pre(R) & Algorithme 1
│   └── bitset_algebra.hpp     # Member 4: AVX2/bitboard set algebra
├── src/
│   ├── safety_solver.cpp      # Core fixed-point solver
│   └── main_thermostat.cpp    # Member 5: Closed-loop tests & GUI
└── CMakeLists.txt             # Modular build file
```

---

## 6. Definition of Done
1. Fixed-point loop terminates in $< 100\,\mu\text{s}$ with verified maximal invariant set $R^*$.
2. Continuous closed-loop simulation runs for $100,000$ seconds without leaving $[19.0, 22.0]^\circ\text{C}$.
3. Mathematical proof of continuous boundary safety matches code outputs exactly.
