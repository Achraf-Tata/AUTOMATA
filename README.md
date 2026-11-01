# Symbolic Control - Course Project (Fall 2026)

C++ implementation of symbolic controllers for nonlinear continuous systems, based on:
> "Approches symboliques pour le contrôle des systèmes non linéaires"  
> Antoine Girard, Pierre-Jean Meyer, Adnane Saoud (Techniques de l'Ingénieur, 2024).

Each project folder is self-contained with its own `CMakeLists.txt` and has zero code dependencies on the other folders.

---

## The Big Idea (Why we do these steps)

Continuous systems (robots, room temperatures) have infinite states and noise. Computers and formal methods (automata, model checking) only work on finite graphs. The paper bridges this in 3 steps:

1. **Abstraction**: Turn continuous differential equations into a finite grid/graph. Because noise is unpredictable, one cell can transition to multiple possible next cells (an overapproximation box guarantees we don't miss anything).
2. **Discrete Synthesis**: Run a fixed-point algorithm on the graph to find a controller that satisfies our mission (e.g., reaching a target or staying in a safe zone forever).
3. **Concretization**: Save the result into a simple lookup table (`u = table[cell]`). At runtime, the robot or thermostat just reads sensor data, finds its cell, and picks the action in $O(1)$ time.

---

## Projects Overview

1. **`1_robot_navigation`**: A unicycle robot that must visit waymark $R_1$ or $R_2$, then reach goal $R_3$, while avoiding obstacle $R_4$. Includes an interactive UI where you can drag and resize regions on the fly.
2. **`2_room_thermostat`**: 1D room temperature control. Uses Algorithm 1 (safety game) to keep temperature in $[19, 22]^\circ\text{C}$ despite changing outside weather.
3. **`3_circular_building`**: $N$ rooms connected in a ring. A naive grid crashes the computer at $N \ge 5$ ($50^5$ states). We use assume-guarantee contracts so it scales linearly up to $N = 100+$ rooms.

---

## Team Roles (5 Members)

Every member contributes code to all three projects:

| Member | Focus | Project 1: Robot | Project 2: Thermostat | Project 3: Building |
| :--- | :--- | :--- | :--- | :--- |
| **Member 1** | **Physics & ODEs** | Unicycle kinematics & noise box | Thermal ODE & boundary check | Coupled ODE & monotonicity proof |
| **Member 2** | **Discretization** | 3D grid & region labels | 50 intervals & transition sets | Local 1D room abstraction |
| **Member 3** | **Synthesis** | 5-state DFA & reachability game | Algorithm 1 safety fixed point | Assume-guarantee contract solver |
| **Member 4** | **C++ & Parallelism**| OpenMP loop & CUDA kernel | Fast bitset math (< 50 µs) | OpenMP & CUDA multi-room scaling |
| **Member 5** | **Simulation & UI** | 50 noise runs & Dear ImGui UI | 24-hr weather run & CSV log | Scalability benchmark & heatmap |

---

## Build & Run

Each project builds on its own:

```bash
# 1. Robot
cd 1_robot_navigation
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/robot_navigation

# 2. Thermostat
cd ../2_room_thermostat
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/room_thermostat

# 3. Building
cd ../3_circular_building
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/circular_building
```
