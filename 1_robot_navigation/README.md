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

### 4. Finding Winning Moves: Backward Reachability (`BackwardReachabilitySolver`)
Instead of guessing forward paths from every starting point, we work **backward from the finish line** (Goal $T$) step by step:
1. Start at the target: $S_0 = T \setminus O$ (cost = 0 steps).
2. Step backward: a cell $q$ joins the winning club $S_{k+1}$ if the robot has at least one move $u$ where **every possible place noise could blow it** ($\Delta(q, u)$) stays inside the winning club $S_k$ and avoids all obstacles $O$:
$$S_{k+1} = S_k \cup \left\{ q \in Q \setminus O \mid \exists u \in U, \emptyset \neq \Delta(q, u) \subseteq S_k \text{ and } \Delta(q, u) \cap O = \emptyset \right\}$$
3. Stop when no new cells can reach the goal ($S_{k+1} = S_k$). For each winning cell, we record the chosen move in our lookup table:
$$\kappa(q) = u^*$$

### 4b. Staying Safe Forever: The Invariance Game (`SafetySolver`)
While reach-avoid grows winning spots outward from the goal, the safety algorithm (Algorithm 1 in Antoine Girard's paper) works **by elimination**:
1. Start by assuming all open space outside obstacles is safe: $S_0 = Q \setminus O$.
2. In each round, check every cell: if *all* available steering moves could allow noise to push the robot into danger, that cell is a trap and gets pruned:
$$S_{k+1} = \left\{ q \in S_k \mid \exists u \in U, \emptyset \neq \Delta(q, u) \subseteq S_k \right\}$$
3. When no more cells need to be removed ($S_{k+1} = S_k$), whatever is left is the **guaranteed safe zone** where the robot can drive around forever without ever crashing.

### 5. Test-Driving the Robot in Simulation (`ClosedLoopSimulator`)
Given any starting position and heading $x_0$, the simulator lets the robot drive in real time:
1. Locate: find which grid box $q_k$ contains the robot's current position $x_k$.
2. Check status: if $q_k \in T$, we reached the goal! If $q_k \in O$, we crashed!
3. Pick move: look up the chosen steering action from our table: $u_k = \kappa(q_k)$.
4. Step forward: roll the robot forward using the unicycle motion formula, adding a bit of random physical wobble: $x_{k+1} = f(x_k, u_k, w_k)$ where $w_k \in [-\mu, \mu]^3$.

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

### Running the Applications

#### 1. Interactive Graphical User Interface (Dear ImGui + OpenGL)
```bash
./build/Release/prototype_gui.exe
```
Features the complete visual simulation suite:
- Interactive 2D arena canvas with drag-and-drop robot placement and angle orientation
- Live region editing (R1, R2, R3, R4) with resize and move controls
- Real-time animated unicycle simulation with speed slider (1..60 steps/s) and bounded disturbance toggle
- Fast OpenMP solver that computes all winning moves in ~1 second
- Edge-case modal alert popups and arena modification warnings

#### 2. Console Driver Application
```bash
# Run with default resolution (50 x 50 x 20 = 50,000 cells)
./build/Release/prototype_navigation.exe

# Or specify custom resolution (nx ny ntheta)
./build/Release/prototype_navigation.exe 40 40 16
```

