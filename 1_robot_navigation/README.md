# Symbolic Control: Academic Robot Navigation Prototype (C++20)

## 1. What Is This Project?

This project teaches a simulated mobile robot (a **unicycle robot**) how to complete a mission completely on its own, even when random physical disturbances (wind, wheel slip) push it off course.

### The Mission:
1. Start at a designated position in a 10m x 10m room.
2. Visit **Waypoint 1 (R1)** OR **Waypoint 2 (R2)** — but **never both** (an exclusivity rule).
3. After visiting one of the waypoints, reach the **Goal (R3)**.
4. **Never** touch the **Obstacle (R4)** or leave the room.

Instead of guessing forward paths or relying on machine learning, this framework implements **symbolic control**:
The continuous room is discretized into a uniform grid of cells. The solver computes a guaranteed move for every single cell by working backward from the goal, storing the result in a fast lookup table (a "cheat sheet"). When the robot drives, it simply queries this cheat sheet and executes the correct move.

---

## 2. How the Mission Works: The 5 Mission Stages (DFA)

To enforce rules like *"visit R1 or R2 first, then go to R3, but never visit both"*, physical coordinates $(x, y, \theta)$ are not enough. The robot also needs to remember what it has already done.

The mission is governed by a 5-state mission tracker (a Deterministic Finite Automaton, or **DFA**):

* **Stage $q_1$ (Start):** The robot just started. It has not visited any waypoint yet.
  * Stepping into R1 moves the robot to **$q_2$**.
  * Stepping into R2 moves the robot to **$q_3$**.
  * Stepping into Obstacle R4 moves the robot to **$q_5$** (failed).
* **Stage $q_2$ (Visited R1):** The robot is now heading for Goal R3.
  * Stepping into Goal R3 moves the robot to **$q_4$** (mission accomplished!).
  * Stepping into R2 is forbidden (violates exclusivity) and sends it to **$q_5$** (failed).
* **Stage $q_3$ (Visited R2):** The robot is now heading for Goal R3.
  * Stepping into Goal R3 moves the robot to **$q_4$** (mission accomplished!).
  * Stepping into R1 is forbidden (violates exclusivity) and sends it to **$q_5$** (failed).
* **Stage $q_4$ (Goal Reached):** Mission completed successfully!
* **Stage $q_5$ (Trap / Failure):** The robot crashed into an obstacle or broke the rules.

### How Does the Robot Know Which Cells Are Which?
At startup, every physical cell $(i_x, i_y)$ on the map is tagged with what region sits there:
* `Region 1` = Waypoint 1
* `Region 2` = Waypoint 2
* `Region 3` = Goal
* `Region 4` = Obstacle
* `Region 0` = Open / Free space

Whenever the robot moves into a cell, the mission tracker checks this tag and updates the stage ($q_1 \to q_2 \dots$).

---

## 3. How the Cheat Sheet is Built: Backward Reachability (Backtracking)

Trying to search forward from every starting position is slow and unreliable because physical disturbances can blow the robot in multiple directions. 

Instead, the algorithm works **backward from the finish line**, like solving a maze in reverse:

1. **Step 0 (The Base Case):**
   Any state that has already reached the goal (stage $q_4$) is given a distance score of **0 steps**.

2. **Round 1 (One Step Away):**
   The solver examines every possible combination of $(x, y, \theta, q)$ and evaluates each of the 15 steering moves. If a move guarantees that **all possible next cells** (even with worst-case physical noise) land in a distance-0 cell, that state receives a score of **1 step**, and that steering move is recorded in the controller table.

3. **Round 2 (Two Steps Away):**
   The solver identifies cells that have a move guaranteed to land in a cell with score $\le 1$. Those states receive a score of **2 steps**.

4. **Rounds 3 to 35 (Rippling Backward):**
   The wave of solved states ripples backward across the entire map, round by round. By **Round 35**, no new cells can be solved—the wave has reached every controllable part of the room!

### The Result:
The output is a lookup table with 1,500,000 entries (100 X cells $\times$ 100 Y cells $\times$ 30 Angle cells $\times$ 5 Stages):
* `V[p]`: The minimum number of steps to reach the goal from state `p`.
* `ctrl[p]`: The exact steering command (1 out of 15) to use at state `p`.

When driving, looking up a move takes less than a microsecond.

---

## 4. Why Does It Solve 1.5 Million States in ~3.5 Seconds?

A naive implementation checking 1,500,000 states across 15 moves over 35 rounds does over **780 million checks**. In typical Python or unoptimized code, this takes 8 to 10 minutes.

The sequential C++ solver (`backward_reachability.cpp`) finishes all 35 rounds in only **~3.5 seconds on a single CPU core** (or ~5 seconds in the GUI with an intentional 50ms progress delay). Here is why:

### 1. 100% Fits in CPU L3 Cache (Zero RAM Stalls)
* Value table `V`: 1,500,000 floats $\times$ 4 bytes = **6.0 MB**
* Move table `ctrl`: 1,500,000 bytes $\times$ 1 byte = **1.5 MB**
* Precomputed transition tables: **~2.0 MB**
* **Total memory footprint:** **~9.5 MB**
* Modern processors (like Intel Core Ultra or AMD Ryzen) have 24 MB to 36 MB of high-speed **L3 cache**.
* Because the entire 9.5 MB problem fits inside the CPU chip, the processor never has to wait for slow system RAM. Memory reads happen at terabytes per second with zero latency.

### 2. Precomputed 1D Transition Tables (No Trigonometry in the Loop)
Instead of computing unicycle differential equations, $\cos(\theta)$, $\sin(\theta)$, and bounding boxes inside the loop:
* At startup, Antoine Girard's decomposition trick calculates all reachable coordinate ranges once:
  ```cpp
  int sx_min  = tables.get_c1_min(ix, ith, u);
  int sx_max  = tables.get_c1_max(ix, ith, u);
  int sy_min  = tables.get_c2_min(iy, ith, u);
  int sy_max  = tables.get_c2_max(iy, ith, u);
  int sth_min = tables.get_c3_min(ith, u);
  int sth_max = tables.get_c3_max(ith, u);
  ```
* Inside the search loop, checking reachable neighbors is just reading precomputed integer min/max cell bounds.

### 3. Flat Linear Memory Layout & CPU Prefetching
* States are stored in a contiguous 1D array:
  $$\text{index} = (q - 1) \times 300{,}000 + i_x \times (N_y N_\theta) + i_y \times N_\theta + i_\theta$$
* When checking heading angles $\theta$, the memory cells lie directly side-by-side: `xy_stride + sth`.
* The CPU's hardware prefetcher detects this linear access pattern and loads the next cache lines before the code even requests them.

### 4. SIMD Vectorization (`/arch:AVX2 /O2`)
* Because `prev_v` is a contiguous array of 32-bit floats, the compiler uses 256-bit AVX2 registers.
* The CPU checks **8 states simultaneously in a single clock cycle**.

### 5. Instant Short-Circuiting & No Dynamic Memory
* The moment **any** reachable cell is unsafe, the loop immediately aborts that control (`all_safe = false; break;`).
* The moment a valid control is found, the solver saves it and skips checking the remaining moves.
* Zero memory allocations (`new`, `malloc`, or `vector::push_back`) occur during the search.

### Why No Multi-Threading (OpenMP) Is Needed:
Because single-core execution takes only ~3.5 seconds, parallel threading (like OpenMP) is completely unnecessary. Removing it keeps the codebase clean, student-friendly, and free from external threading bugs.

---

## 5. Codebase Structure

Every file has one clear responsibility with clean, concise student-friendly variable names (`s`, `q`, `u`, `ix`, `iy`, `ith`):

| File | Responsibility |
|---|---|
| `interval.hpp` / `.cpp` | 1D math intervals $[a, b]$ for computing bounding ranges under noise. |
| `bounding_box.hpp` / `.cpp` | 3D boxes $[x_1, x_2] \times [y_1, y_2] \times [\theta_1, \theta_2]$ and overlap tests. |
| `grid_partition.hpp` / `.cpp` | Divides continuous space into discrete cells $(i_x, i_y, i_\theta)$ and calculates cell centers. |
| `grid_quantizer.hpp` / `.cpp` | Precomputes Girard's 1D transition lookup tables at startup. |
| `dfa.hpp` / `.cpp` | 5-state mission tracking automaton and state transition logic. |
| `product_automaton.hpp` / `.cpp` | Combines spatial cells with mission stages (1.5 million product states). |
| `unicycle_model.hpp` / `.cpp` | Physical unicycle motion formulas and worst-case reachability expansion. |
| `backward_reachability.hpp` / `.cpp` | Single-threaded sequential backward reachability solver and binary file saving/loading. |
| `safety_solver.hpp` / `.cpp` | Algorithm 1: Staying safe forever (maximal safe invariance game). |
| `simulator.hpp` / `.cpp` | Step-by-step unicycle closed-loop motion simulator with random physical noise. |
| `simulation_engine.hpp` / `.cpp` | Background thread orchestration connecting the GUI to the simulation and solver. |
| `app_state.hpp` / `.cpp` | Central data structure holding regions, robot position, and policy tables. |
| `control_panel.hpp` / `.cpp` | Right-hand control cards (Controller, Simulation controls, and Run results). |
| `arena_view.hpp` / `.cpp` | Interactive 2D OpenGL canvas showing the room, regions, robot pose, and trail. |
| `popup_dialogs.hpp` / `.cpp` | Helpful modal explanation dialogs for invalid user configurations. |
| `ui_theme.hpp` / `.cpp` | Clean, modern visual styling (colors, fonts, card layouts). |
| `main_gui.cpp` | Main GUI entry point (window creation and render loop). |
| `main.cpp` | Standalone console driver application. |
| `tests/test_prototype.cpp` | 9-step automated test suite covering all modules. |

---

## 6. Building and Running

### Prerequisites
* C++20 compliant compiler (MSVC 2022, GCC 11+, or Clang 13+)
* CMake 3.20 or newer

### Build Instructions
```bash
# Configure the build directory
cmake -B build -S .

# Build all targets in Release mode
cmake --build build --config Release
```

### 1. Running the Automated Tests
```bash
./build/Release/test_prototype.exe
```
This runs 9 automated checks verifying interval math, bounding boxes, grid indexing, DFA transitions, reachability convergence (35 iterations in ~3.5 s), binary saving/loading, and virtual driving to Goal R3 in 19 steps.

### 2. Running the Interactive GUI Application
```bash
./build/Release/prototype_gui.exe
```
Features:
* **Interactive 2D Arena:** Drag and drop the robot anywhere; drag the heading arrow to change starting orientation.
* **Live Region Editing:** Drag or resize Waypoints R1/R2, Goal R3, and Obstacle R4.
* **Controller Card:** Click "Run solver" to synthesize the 1.5M state cheat sheet with a live progress bar.
* **Simulation Card:** Click "Start simulation" to watch the robot drive in real time, or "Instant run" for immediate evaluation.
* **Disturbance Toggle:** Turn random disturbances ($w \in [-0.05, 0.05]$) on or off.

### 3. Running the Console Driver
```bash
# Default resolution (50 x 50 x 20 = 50,000 cells)
./build/Release/prototype_navigation.exe

# Custom resolution (nx ny ntheta)
./build/Release/prototype_navigation.exe 40 40 16
```
