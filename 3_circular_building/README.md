# Project 3: Temperature Regulation in a Circular Building (Competition Challenge)

Course: **Computation Theory - Fall 2026**  
Instructor: **Prof. Adnane Saoud**  
Reference: **Slides 15–17 of `42717210-s7467.pdf`**

---

## 1. System Model & Problem Statement

We consider an $N$-room circular building ($N \ge 3$) where each room $i \in \{1, \dots, N\}$ exchanges heat through conduction with its left neighbor $i-1 \pmod N$, its right neighbor $i+1 \pmod N$, and the exterior environment.

### 1.1 Discrete Difference Thermal Dynamics (Slide 15)
$$
T_i(t+1) = T_i(t) + \alpha \big(T_{i+1}(t) + T_{i-1}(t) - 2T_i(t)\big) + \beta \big(T_e - T_i(t)\big) + \gamma \big(T_h - T_i(t)\big) u_i(t)
$$

### 1.2 System Parameters & Specifications
* **Inter-room conduction**: $\alpha = 0.45$
* **Heat exchange with outside**: $\beta = 0.045$
* **Heater heat transfer coefficient**: $\gamma = 0.09$
* **Exterior ambient temperature**: $T_e = -1.0^\circ\text{C}$
* **Heater radiator temperature**: $T_h = 50.0^\circ\text{C}$
* **Control inputs**: $n_u = 3 \implies u_i \in \{0.0, 0.3, 0.6\}$
* **Comfort safe set**: $S = [17.0, 22.0]^N\,^\circ\text{C}$
* **Uniform state partition**: $n_d = 10$ intervals per dimension ($\Delta T = 0.5^\circ\text{C}$/cell)
* **Periodic boundaries**: $T_0 = T_N$, $T_{N+1} = T_1$

---

## 2. Why Monolithic Synthesis Fails vs. Compositional Contracts (Slide 16–17)

### 2.1 The Curse of Dimensionality ($10^N$ States, $3^N$ Inputs)
Under standard monolithic safety synthesis, the global grid grows exponentially:
* $N = 3$: $10^3 = 1,000$ states, $3^3 = 27$ inputs
* $N = 5$: $10^5 = 100,000$ states, $3^5 = 243$ inputs
* $N = 10$: $10^{10} = 10\text{ Billion}$ states, $3^{10} = 59,049$ inputs ($\sim 10\text{ GB}$ RAM, hours to solve)
* $N = 100$: $10^{100}$ states (exceeds total atoms in observable universe)

### 2.2 Compositional Assume-Guarantee Circular Contracts
Instead of a monolithic $N$-dimensional grid, we decompose the building into $N$ coupled 1D subsystems:
1. **Assumption ($A_i$)**: Room $i$ assumes its immediate neighbors remain within the comfort set $T_{i\pm 1} \in [17.0, 22.0]^\circ\text{C}$.
2. **Induced Disturbance Bounds ($W_i$)**:
   $$W_i = \alpha(T_{i-1} + T_{i+1}) + \beta T_e \in [15.255, 19.755]^\circ\text{C}$$
3. **Local 1D Abstraction**:
   Each room solves an identical 1D safety fixed-point loop on only **10 cells**!
   $$R = R \cap Pre(R)$$
4. **Circular Contract Composition Theorem (Saoud, Girard, Fribourg)**:
   Since each room's invariant guarantee satisfies $G_i \subseteq A_{i\pm 1} = [17.0, 22.0]^\circ\text{C}$, symmetric circular composition ensures the entire coupled network remains safe without deadlocks or circular dependency paradoxes.
5. **Linear Complexity**: Synthesis time scales in $O(N \cdot 10)$ instead of $O(10^N)$, solving $N = 1,000$ rooms in milliseconds on a standard laptop.

---

## 3. Design Patterns & SOLID Principles

The codebase has been refactored adhering strictly to classical GoF design patterns and SOLID principles:

| Design Pattern / Principle | Component | Concrete Implementation |
|---|---|---|
| **Strategy Pattern (GoF)** | Safety Synthesis Solver | `ISynthesisSolver` abstract interface with `CompositionalContractSolver` and `MonolithicCurseEvaluator`. Allows hot-swapping algorithms. |
| **Strategy Pattern (GoF)** | Weather Engine | `IWeatherStrategy` with `ConstantWeatherStrategy`, `DiurnalWeatherStrategy`, `ColdSnapWeatherStrategy`, and `ManualWeatherStrategy`. |
| **Factory Pattern (GoF)** | Weather Factory | `WeatherFactory::create(Type, manual_temp)` encapsulates strategy instantiation. |
| **Memento Pattern (GoF)** | Controller Persistence | `ControllerSerializer` providing binary serialization (`save_binary`, `load_binary`) with `CBMC` magic header, schema versioning, and validation. |
| **Single Responsibility (SRP)** | Modular Architecture | - `ThermalNetworkModel`: Difference equation & Metzler monotonicity.<br>- `UniformGrid1D` & `ContractAbstraction`: Quantization & interval math.<br>- `CompositionalContractSolver`: Fixed-point computation & policy extraction.<br>- `SimulationEngine`: Multi-room continuous/discrete stepping & Monte Carlo perturbation.<br>- `BuildingView` & `ControlPanel`: Dear ImGui rendering. |
| **Open/Closed Principle (OCP)** | Strategy Extensibility | New solvers (e.g. GPU CUDA solver) or weather profiles can be added without modifying existing consumer code. |
| **Dependency Inversion (DIP)** | Abstraction Coupling | High-level simulation engine and UI depend on `ISynthesisSolver` and `IWeatherStrategy` interfaces rather than concrete classes. |

---

## 4. Official Competition Benchmark Results (Slide 17)

Executed on an Intel / AMD multi-core workstation:

```text
====================================================================================
     N      Tabs (ms)     Tctrl (ms)       Peak Memory   Controllable %
====================================================================================
     3          0.011          2.549           0.9 KB            100.0 %
     4          0.019          0.005           0.9 KB            100.0 %
     5          0.017          0.009           1.0 KB            100.0 %
    10          0.018          0.012           1.2 KB            100.0 %
    50          0.017          0.011           2.9 KB            100.0 %
   100          0.018          0.018           5.1 KB            100.0 %
   500          0.017          0.039          22.3 KB            100.0 %
  1000          0.018          0.055          43.7 KB            100.0 %
====================================================================================
```

* **Abstraction Time ($T_{\text{abs}}$)**: $\approx 0.018\text{ ms}$ (independent of $N$).
* **Control Time ($T_{\text{ctrl}}$)**: $\approx 0.055\text{ ms}$ for $N=1000$ rooms.
* **Peak Memory**: $\le 45\text{ KB}$ for $N=1000$ rooms.
* **Controllable Percentage**: **100.0%** of cells in $[17.0, 22.0]^\circ\text{C}$ are controllable and robustly invariant.
* **Closed-Loop Verification**: 24-hour simulation (86,400 steps) with 0 temperature violations.

---

## 5. Comprehensive Unit Test Suite

The test suite in `tests/test_building.cpp` covers 10 verification scenarios:

1. **Metzler Monotonicity**: Verifies off-diagonal $\alpha = 0.45 > 0$ and diagonal $1 - 2\alpha - \beta - \gamma u > 0$ for all $u \in \{0.0, 0.3, 0.6\}$.
2. **Local Contract Disturbance Bounds & Quantization**: Verifies $W_i \in [15.255, 19.755]^\circ\text{C}$ and 10-cell quantization.
3. **Discrete Transition Table & Sink Boundary**: Verifies sink cell 10 is absorbing and controls shift reach sets monotonically.
4. **Fixed-Point Contract Invariant Synthesis**: Verifies 10/10 cells are invariant ($100.0\%$) and converge within $\le 10$ iterations.
5. **Circular Contract Composition Guarantee**: Verifies $G_i \subseteq A_{i\pm 1}$ (Saoud-Girard-Fribourg theorem).
6. **Controller Extraction & Robust Invariance Policy**: Verifies robust control assignment ($u=0.3$) and out-of-comfort emergency recovery.
7. **Controller Binary Serialization (Memento Pattern)**: Bit-for-bit roundtrip verification with magic header `CBMC`.
8. **OpenMP Multi-Core Parallel Equivalence**: Verifies sequential and parallel solvers produce identical policies.
9. **Continuous Multi-Room Closed-Loop Simulation**: 5,000-step coupled simulation of $N=16$ rooms with zero violations.
10. **Kamke-Müller Extreme Envelopes & Monte Carlo Bundle**: Verifies $50/50$ perturbed runs remain safely bounded in $[17.0, 22.0]^\circ\text{C}$.

---

## 6. Build & Run Instructions

The project is completely self-contained with **zero dependencies on `1_robot_navigation`**.

### Building with CMake (MSVC / Windows)
```powershell
$env:TEMP='F:\temp'; $env:TMP='F:\temp'
cmake -B 3_circular_building/build -S 3_circular_building
cmake --build 3_circular_building/build --config Release
```

### Running the Test Suite
```powershell
.\3_circular_building\build\Release\test_building.exe
```

### Running the Competition Benchmarks (Slide 17)
```powershell
.\3_circular_building\build\Release\circular_building.exe
```

### Launching the Interactive GUI Application
```powershell
.\3_circular_building\build\Release\building_gui.exe
```
