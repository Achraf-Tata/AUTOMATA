# High-Performance Symbolic Control of Nonlinear Systems

Hardware-accelerated C++20, OpenMP, and CUDA implementation of discrete abstractions, safety invariance synthesis, backward reachability games, and compositional Assume-Guarantee contracts for nonlinear cyber-physical systems under temporal logic specifications.

### Academic Reference
> **"Approches symboliques pour le contrôle des systèmes non linéaires"**  
> Antoine Girard, Pierre-Jean Meyer, Adnane Saoud (*Techniques de l'Ingénieur*, Septembre 2024, Ref. S7467).  
> **Course**: Theory of Computation — Fall 2026 — Instructor: **Prof. Adnane Saoud** (UM6P College of Computing).

---

## 1. High-Performance Hardware & Architecture Stack

* **CPU**: Intel Core Ultra 9 275HX (24 Cores / 24 Threads) $\to$ **OpenMP 5.0 + AVX2 SIMD Bit-Parallelism**.
* **GPU**: NVIDIA GeForce RTX 5080 Laptop GPU (16 GB VRAM) $\to$ **NVIDIA CUDA C++ Kernel Acceleration**.
* **Language Standard**: Modern C++20 (`constexpr`, `concepts`, Structure-of-Arrays memory layout).
* **GUI / Dashboard**: Real-Time Interactive Telemetry (Dear ImGui + OpenGL).

---

## 2. Repository Structure & Project Modules

```
AUTOMATA/
├── 1_robot_navigation/      # Project 1: Unicycle robot under co-safe LTL / Product Automaton
│   └── README.md            # OpenMP + CUDA GPU reachability game architecture
│
├── 2_room_thermostat/       # Project 2: 1D thermal regulation & safety invariance
│   └── README.md            # Algorithme 1 maximal invariant set & bitset vectorization
│
├── 3_circular_building/     # Project 3: N-room circular building challenge
│   └── README.md            # Assume-Guarantee contracts & linear O(N) scaling suite
│
├── 42717210-s7467.pdf       # Prof. Adnane Saoud's reference paper
├── .gitignore               # Configured for C++ builds (build/, bin/, *.o, etc.)
└── README.md                # Unified project hub linking all modules
```

---

## 3. 5-Member Horizontal Engineering Pipeline

Every team member actively implements and defends code across **all three projects**:

| Member | Technical Specialization | Deliverables Across the 3 Projects |
| :--- | :--- | :--- |
| **Member 1** | **Continuous Physics & ODEs** | Unicycle forward kinematics, 1D thermal ODE, coupled $N$-room matrix equations, Metzler matrix / Kamke-Müller monotonicity proof. |
| **Member 2** | **Discretization & Abstraction** | 3D grid cell hashing ($100\times 100\times 30$), temperature interval partition, local contract assumption bounds $(A_i, G_i)$. |
| **Member 3** | **Formal Synthesis & Fixed Points** | 5-state specification DFA, Algorithme 1 maximal invariant set $R^*$, Assume-Guarantee compositional synthesis. |
| **Member 4** | **C++ HPC, OpenMP & CUDA Architecture** | Contiguous 1D flat buffers, AVX2 bitboards, 24-thread OpenMP parallelization, CUDA `.cu` acceleration kernels for the RTX 5080. |
| **Member 5** | **Simulation, Telemetry & Live GUI** | Monte Carlo noise runs, 24-hr sinusoidal weather simulations, automated benchmark suite ($N=3..1000$), live Dear ImGui dashboard. |

---

## 4. Detailed Project Guides

* 🤖 **[Project 1: Mobile Robot Navigation (Product Automaton Reachability)](1_robot_navigation/README.md)**
* 🌡️ **[Project 2: Room Thermostat (Safety Invariance Synthesis)](2_room_thermostat/README.md)**
* 🏢 **[Project 3: Circular Building (Assume-Guarantee Challenge Engine)](3_circular_building/README.md)**
