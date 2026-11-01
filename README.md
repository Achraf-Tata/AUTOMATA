# High-Performance Symbolic Control of Nonlinear Systems

Independent, self-contained C++20 and CUDA implementations of discrete abstractions, safety invariance games, and compositional Assume-Guarantee contracts for nonlinear cyber-physical systems.

* **Reference**: *"Approches symboliques pour le contrôle des systèmes non linéaires"* (Girard, Meyer, Saoud, 2024, Ref. S7467).
* **Course**: Theory of Computation — Prof. Adnane Saoud (UM6P College of Computing).
* **Architecture**: **Zero Cross-Project Code Dependencies**. Each project folder is 100% self-contained with its own independent headers, sources, and `CMakeLists.txt`.

---

## 1. Project Modules (Completely Decoupled)

```
AUTOMATA/
├── 1_robot_navigation/      # Standalone Project 1: Mobile Robot Reachability Game
│   ├── include/             # Self-contained headers
│   ├── src/                 # Self-contained sources
│   ├── CMakeLists.txt       # Independent build configuration
│   └── README.md            # Exact task checklist for Members 1..5
│
├── 2_room_thermostat/       # Standalone Project 2: Room Thermostat Safety Invariance
│   ├── include/             # Self-contained headers
│   ├── src/                 # Self-contained sources
│   ├── CMakeLists.txt       # Independent build configuration
│   └── README.md            # Exact task checklist for Members 1..5
│
├── 3_circular_building/     # Standalone Project 3: N-Room Circular Building Challenge
│   ├── include/             # Self-contained headers
│   ├── src/                 # Self-contained sources
│   ├── CMakeLists.txt       # Independent build configuration
│   └── README.md            # Exact task checklist for Members 1..5
│
├── 42717210-s7467.pdf       # Reference research paper
└── README.md                # Root guide
```

---

## 2. 5-Member Responsibility Matrix (Across Independent Projects)

Every team member implements their specialized layer inside each standalone folder:

| Member | Focus | Project 1: Robot | Project 2: Thermostat | Project 3: Circular Building |
| :--- | :--- | :--- | :--- | :--- |
| **Member 1** | **Physics & ODEs** | Unicycle kinematics & reachable box | 1D thermal ODE & boundary stability | Coupled N-room ODE & Metzler monotonicity |
| **Member 2** | **Discretization** | 3D grid quantizer (100 x 100 x 30) | Interval partition & transition sets | Local room abstraction & assumptions (Ai, Gi) |
| **Member 3** | **Formal Synthesis**| 5-state DFA & Product Automaton game | Algorithme 1 maximal invariant set R* | Assume-Guarantee contract solver O(N) |
| **Member 4** | **HPC & CUDA** | OpenMP 24-core sweep + CUDA kernel | Bitboard set operations (< 50 µs) | Multi-room OpenMP/CUDA scaling (N <= 1000) |
| **Member 5** | **Simulation & UI** | 50 Monte Carlo runs & Interactive UI | 24-hr sinusoidal weather simulation | Benchmark scaling suite & heatmap traces |

---

## 3. Independent Build & Execution

Each project compiles and runs completely independently from within its own directory:

```bash
# Build and run Project 1:
cd 1_robot_navigation
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/robot_navigation

# Build and run Project 2:
cd ../2_room_thermostat
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/room_thermostat

# Build and run Project 3:
cd ../3_circular_building
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/circular_building
```
