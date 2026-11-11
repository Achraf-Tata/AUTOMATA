# Symbolic Control: Standard Academic Prototype (C++20)

## Overview

This directory contains a modular, self-contained academic prototype implementing the foundational pipeline of symbolic control for a unicycle mobile robot with bounded disturbances, following Antoine Girard (2024) and Adnane Saoud's course syllabus.

The prototype is designed as an educational milestone representative of what a student or researcher would typically achieve in a 3-week project phase, focusing on clear software architecture, clean object-oriented principles, and numerical correctness.

---

## Architectural Structure

The codebase is organized into single-responsibility modules following standard software engineering practices:

```
academic_prototype/
|-- CMakeLists.txt              # CMake build configuration
|-- README.md                   # Technical documentation
|-- include/
|   |-- interval.hpp            # 1D interval arithmetic [a, b]
|   |-- bounding_box.hpp        # 3D continuous hyper-rectangles
|   |-- grid_partition.hpp      # Uniform state space quantizer
|   |-- unicycle_model.hpp      # Continuous dynamics and reachable growth bounds
|   |-- transition_system.hpp   # Finite symbolic abstraction (Q, U, Delta)
|   |-- backward_reachability.hpp # Fixed-point backward reachability solver
|   `-- simulator.hpp           # Closed-loop disturbed unicycle simulator
|-- src/
|   |-- interval.cpp
|   |-- bounding_box.cpp
|   |-- grid_partition.cpp
|   |-- unicycle_model.cpp
|   |-- transition_system.cpp
|   |-- backward_reachability.cpp
|   |-- simulator.cpp
|   `-- main.cpp                # Prototype driver application
`-- tests/
    `-- test_prototype.cpp      # Unit test suite
```

---

## Theoretical Foundations

### 1. State Space Discretization (`GridPartition`)
The continuous domain is defined by:
$$X = [0, 10] \times [0, 10] \times [-\pi, \pi]$$
Partitioned into $N_x \times N_y \times N_\theta$ uniform hyper-rectangular cells of width:
$$\eta = (\Delta x, \Delta y, \Delta \theta) = \left( \frac{10}{N_x}, \frac{10}{N_y}, \frac{2\pi}{N_\theta} \right)$$
Each discrete state $q \in \{0, \dots, |Q|-1\}$ bijectively maps to a cell coordinate $(i_x, i_y, i_\theta)$ and continuous center $c \in \mathbb{R}^3$.

### 2. Reachable Set Over-Approximation (`UnicycleModel`)
The perturbed unicycle dynamics are:
$$\begin{cases}
x(t+1) = x(t) + \tau (v \cos\theta + w_1) \\
y(t+1) = y(t) + \tau (v \sin\theta + w_2) \\
\theta(t+1) = \theta(t) + \tau (\omega + w_3) \pmod{2\pi}
\end{cases}$$
under sampling period $\tau = 1.0\text{ s}$, discrete inputs $u = (v, \omega) \in U$, and disturbance bounds $w \in W = [-\mu, \mu]^3$ with $\mu = 0.05$.

For a cell with center $c$ and radii $r = \frac{\eta}{2}$, the continuous reachable set is over-approximated by an axis-aligned bounding box centered at $c'$ with expansion radii $r'$:
$$c_1' = c_1 + \tau v \cos(c_3), \quad r_1' = r_1 + \tau |v| r_3 + \tau \mu$$
$$c_2' = c_2 + \tau v \sin(c_3), \quad r_2' = r_2 + \tau |v| r_3 + \tau \mu$$
$$c_3' = c_3 + \tau \omega, \quad r_3' = r_3 + \tau \mu$$

### 3. Symbolic Abstraction (`TransitionSystem`)
The transition relation $\Delta \subseteq Q \times U \times Q$ connects cell $q$ to cell $q'$ under input $u$ if:
$$\text{ReachableBox}(q, u) \cap \text{CellBox}(q') \neq \emptyset$$

### 4. Backward Reachability Synthesis (`BackwardReachabilitySolver`)
Given a target set $T \subseteq Q$ and obstacle set $O \subseteq Q$, the backward controllable fixed-point algorithm computes the sequence of winning sets $S_k$:
$$S_0 = T \setminus O$$
$$S_{k+1} = S_k \cup \left\{ q \in Q \setminus O \mid \exists u \in U, \emptyset \neq \Delta(q, u) \subseteq S_k \text{ and } \Delta(q, u) \cap O = \emptyset \right\}$$
The iteration terminates when $S_{k+1} = S_k$. For each $q \in S^*$, the feedback controller is stored:
$$\kappa(q) = u^*$$

### 5. Closed-Loop Simulation (`ClosedLoopSimulator`)
Given initial continuous pose $x_0$, the simulator iterates:
1. Quantize $x_k \to q_k = \text{quantize}(x_k)$.
2. If $q_k \in T$, terminate with success.
3. If $q_k \in O$, terminate with collision.
4. Retrieve control $u_k = \kappa(q_k)$.
5. Apply step: $x_{k+1} = f(x_k, u_k, w_k)$ where $w_k \in [-\mu, \mu]^3$ is sampled uniformly.

---

## Building and Running

### Prerequisites
* C++20 compliant compiler (GCC 11+, Clang 13+, MSVC 2022)
* CMake 3.20+

### Build Instructions
```bash
# Configure
cmake -B build -S .

# Build
cmake --build build --config Release
```

### Running Tests
```bash
# Run unit test suite
./build/Release/test_prototype.exe
```

### Running the Driver Application
```bash
# Run with default resolution (50 x 50 x 20 = 50,000 cells)
./build/Release/prototype_navigation.exe

# Or specify custom resolution (nx ny ntheta)
./build/Release/prototype_navigation.exe 40 40 16
```
