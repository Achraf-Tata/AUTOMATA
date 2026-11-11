# Mobile Robot Navigation System

Symbolic control and automated synthesis for a discrete-time unicycle robot under bounded disturbances in a 10 x 10 m arena.

---

## 1. System Dynamics and Control Goal

We consider the unicycle kinematic model with sampling period $\tau = 1.0\text{ s}$:

$$
\begin{aligned}
x_1(t+1) &= x_1(t) + \tau u_1(t) \cos(x_3(t)) + \tau w_1(t) \\
x_2(t+1) &= x_2(t) + \tau u_1(t) \sin(x_3(t)) + \tau w_2(t) \\
x_3(t+1) &= x_3(t) + \tau u_2(t) + \tau w_3(t) \pmod{2\pi} \in [-\pi, \pi]
\end{aligned}
$$

* **State**: $x = (x_1, x_2, x_3) \in [0, 10] \times [0, 10] \times [-\pi, \pi]$.
* **Input**: $u = (u_1, u_2) \in U_1 \times U_2$ with:
  * Linear velocity $u_1 \in \{0.25, 0.625, 1.0\}\text{ m/s}$
  * Angular velocity $u_2 \in \{-1.0, -0.5, 0.0, 0.5, 1.0\}\text{ rad/s}$
  * Total discrete actions: $3 \times 5 = 15$ control inputs.
* **Disturbance**: Bounded noise $w_i \in [-0.05, 0.05]$ on each coordinate.
* **Workspace Discretization**:
  * $N_1 = 100$ cells along $x_1$ ($\Delta x_1 = 0.1\text{ m}$)
  * $N_2 = 100$ cells along $x_2$ ($\Delta x_2 = 0.1\text{ m}$)
  * $N_3 = 30$ bins along heading $x_3$ ($\Delta x_3 \approx 0.2094\text{ rad}$)
  * Total state partition: $100 \times 100 \times 30 = 300,000$ cells.
* **DFA Specification**: 5 discrete automaton modes enforcing:
  * Visit Waypoint $R_1$ OR Waypoint $R_2$ (mutually exclusive)
  * Subsequently reach Goal $R_3$
  * Never enter Obstacle $R_4$
  * Total product automaton states: $300,000 \times 5 = 1,500,000$ states.

---

## 2. What is `controller.bin`?

`controller.bin` is a precomputed binary file containing:
1. **The backward reachability value function** $V(x_1, x_2, \theta, q)$ (1,500,000 float entries, 6.0 MB).
2. **The synthesized feedback policy** $ctrl(x_1, x_2, \theta, q)$ (1,500,000 unsigned byte entries, 1.5 MB).

### Why do we need it?
Running symbolic abstraction and backward reachability across 1.5 million product states takes ~1.2 to 2.0 seconds of OpenMP parallel execution. By serializing the resulting tables to `controller.bin`:
* The application loads and initializes in less than 2 milliseconds on startup.
* Closed-loop rollouts, trajectory visualization, and Monte Carlo verification can run immediately without recomputing the policy each time.
* If arena regions are modified, the user can re-synthesize and click "Save" to update `controller.bin`.

---

## 3. Modular Architecture

The codebase is refactored into distinct, single-responsibility modules:

| Module | Files | Responsibility |
| :--- | :--- | :--- |
| **State Management** | `include/app_state.hpp`<br>`src/app_state.cpp` | Pure data models: arena geometry, robot poses, controller tables, telemetry, and modal triggers. |
| **UI Theme & Styling** | `include/ui_theme.hpp`<br>`src/ui_theme.cpp` | Warm cream paper theme, Segoe UI typography, custom cards, buttons, badges, and toggle switches. |
| **Arena Canvas** | `include/arena_view.hpp`<br>`src/arena_view.cpp` | 10x10 arena canvas, coordinate axes, interactive drag/resize handles, active path glow, robot arrow cursor, and legend. |
| **Control Panel** | `include/control_panel.hpp`<br>`src/control_panel.cpp` | Top navigation bar and 5 stacked cards (Controller, Simulation, Run result, Monte Carlo, Arena Regions). |
| **Modal Popups** | `include/popup_dialogs.hpp`<br>`src/popup_dialogs.cpp` | Modals for edge cases (Obstacle, Out of bounds, DFA violation, Timeout, Uncontrollable) and arena modification warnings. |
| **Simulation Engine** | `include/simulation_engine.hpp`<br>`src/simulation_engine.cpp` | Step simulation, closed-loop trajectories, Monte Carlo analysis, and OpenMP reachability solver integration. |
| **Main Orchestrator** | `src/main_robot.cpp` | Application entry point: initializes GLFW/OpenGL, loads theme/fonts, executes render loop. |
| **Automaton & Solver** | `include/product_automaton.hpp`<br>`include/reachability_cpu.hpp` | Formal DFA transition rules and OpenMP fixed-point reachability game. |

---

## 4. Modal Dialogs and Edge-Case Handling

### Edge-Case Alerts
When an anomaly occurs during a simulation rollout, a modal popup appears providing:
1. **What happened**: Detailed description of the runtime event.
2. **Why it happened**: Underlying formal or physical root cause (e.g. state outside winning domain, disturbance push).
3. **Recommendation**: Clear action steps to resolve the issue.

Edge cases handled:
* **Obstacle Collision**: Robot entered $R_4$.
* **Out of Bounds**: Robot exited the $[0, 10] \times [0, 10]$ workspace.
* **Specification Violation**: Robot visited both $R_1$ and $R_2$ before reaching $R_3$.
* **Horizon Timeout**: Reached step limit without reaching $R_3$.
* **Uncontrollable State**: Initial or current state has no defined control ($V = \infty$).

### Arena Layout Modified Warning
When any region ($R_1, R_2, R_3, R_4$) is moved or resized by dragging in the arena, the application marks the layout as modified. Clicking "Run simulation" triggers a warning dialog offering:
* **Solve and simulate**: Recomputes the controller for the new layout and runs simulation automatically upon completion.
* **Solve only**: Recomputes the controller without immediately running.
* **Continue anyway**: Tests the old controller against the new arena layout.
* **Cancel**: Returns to the editor without changes.

---

## 5. Build and Execution

### Prerequisites
* C++20 compiler (Visual Studio 2022 Community with MSVC)
* CMake 3.20 or newer
* OpenGL and OpenMP

### Build Commands
```powershell
cd F:\AUTOMATA\1_robot_navigation
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

### Running the Executable
* **GUI Application**:
  ```powershell
  .\build\Release\robot_navigation.exe
  ```
  *Built with `/SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup` to suppress any background Windows console window.*

* **Automated Test Suite**:
  ```powershell
  .\build\Release\test_robot.exe
  ```
