# Symbolic Control of Nonlinear Systems (C++20 & CUDA)

Formal controller synthesis for nonlinear cyber-physical systems using discrete abstractions, safety invariance games, and compositional Assume-Guarantee contracts.

* **Reference**: *"Approches symboliques pour le contrôle des systèmes non linéaires"* (Girard, Meyer, Saoud, 2024, Ref. S7467).
* **Course**: Theory of Computation — Prof. Adnane Saoud (UM6P College of Computing).
* **Hardware Target**: Intel Core Ultra 9 275HX (24 Cores) + NVIDIA RTX 5080 Laptop GPU.

---

## 1. Project Directory Structure

```
AUTOMATA/
├── 1_robot_navigation/      # Project 1: Unicycle robot reachability on Product Automaton
│   └── README.md            # Exact task checklist for Members 1..5
│
├── 2_room_thermostat/       # Project 2: 1D thermal safety & Algorithme 1 fixed point
│   └── README.md            # Exact task checklist for Members 1..5
│
├── 3_circular_building/     # Project 3: Scalable N-room Assume-Guarantee challenge
│   └── README.md            # Exact task checklist for Members 1..5
│
├── 42717210-s7467.pdf       # Reference research paper
└── README.md                # Root guide
```

---

## 2. 5-Member Responsibility Matrix

| Member | Focus | Project 1: Robot | Project 2: Thermostat | Project 3: Circular Building |
| :--- | :--- | :--- | :--- | :--- |
| **Member 1** | **Physics & ODEs** | Unicycle kinematics & reachable box | 1D thermal ODE & boundary stability | Coupled N-room ODE & Metzler monotonicity |
| **Member 2** | **Discretization** | 3D grid quantizer (100 x 100 x 30) | Interval partition & transition sets | Local room abstraction & assumptions (Ai, Gi) |
| **Member 3** | **Formal Synthesis**| 5-state DFA & Product Automaton game | Algorithme 1 maximal invariant set R* | Assume-Guarantee contract solver O(N) |
| **Member 4** | **HPC & CUDA** | OpenMP 24-core sweep + CUDA kernel | Bitboard set operations (< 50 µs) | Multi-room OpenMP/CUDA scaling (N <= 1000) |
| **Member 5** | **Simulation** | 50 Monte Carlo noisy trajectories | 24-hr sinusoidal weather simulation | Benchmark scaling suite & heatmap traces |

---

## 3. Detailed Task Checklists

Read the exact task checklist in each folder:
* 🤖 **[1_robot_navigation/README.md](1_robot_navigation/README.md)**
* 🌡️ **[2_room_thermostat/README.md](2_room_thermostat/README.md)**
* 🏢 **[3_circular_building/README.md](3_circular_building/README.md)**
