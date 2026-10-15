# Symbolic Control of Nonlinear Systems - Mobile Robot Navigation

Implementation of discrete abstraction, backward reachability controller computation, and closed-loop simulation for a discrete-time unicycle mobile robot under linear temporal logic (LTL) specifications.

Reference:
> "Approches symboliques pour le controle des systemes non lineaires"  
> Antoine Girard, Pierre-Jean Meyer, Adnane Saoud (Techniques de l'Ingenieur, Septembre 2024, Ref. S7467).  
> Course: Computation Theory - Fall 2026 - Instructor: Prof. Adnane Saoud (UM6P-CC).

---

## System Dynamics and Specification

### Continuous Model
Discrete-time unicycle kinematics with bounded process disturbance noise:

```
x1(t+1) = x1(t) + T * u1(t) * cos(x3(t)) + T * w1(t)
x2(t+1) = x2(t) + T * u1(t) * sin(x3(t)) + T * w2(t)
x3(t+1) = x3(t) + T * u2(t) + T * w3(t)  (wrapped in [-pi, pi])
```

- State constraints: `x1 in [0, 10]` m, `x2 in [0, 10]` m, `x3 in [-pi, pi]` rad
- Control constraints: linear velocity `u1 in [0.25, 1.0]` m/s, angular velocity `u2 in [-1.0, 1.0]` rad/s
- Disturbance bounds: `w in [-0.05, 0.05]^3`
- Sampling time: `T = 1.0` s

### Specification and Automaton
The robot must satisfy the co-Buchi specification:
> "Visit region R1 or region R2 (exclusive), then reach final target region R3, while avoiding obstacle region R4 at all times."

The deterministic finite automaton (DFA) has 5 discrete states:
- `q1`: Initial state (seeking R1 or R2, avoiding R4)
- `q2`: Waymark R1 visited (heading to Target R3, avoiding R2 and R4)
- `q3`: Waymark R2 visited (heading to Target R3, avoiding R1 and R4)
- `q4`: Target R3 reached (goal achieved)
- `q5`: Trap sink state (collision with obstacle R4)

### Abstraction and Synthesis
- State discretization: `100 x 100 x 30 = 300,000` grid cells
- Control discretization: `3 x 5 = 15` discrete input pairs
- Fixed-point controlled backward reachability algorithm compiled with Numba JIT.

---

## Repository Structure

```
automata/
├── 1_robot_navigation/
│   ├── symbolic_control.py          # Continuous unicycle dynamics and backward reachability
│   ├── gui_robot.py                 # Standalone desktop interface with direct map manipulation
│   ├── controller_cache_100x100x30.npz # Precomputed reachability controller (300k cells)
│   └── run.py                       # Standalone local entry point
│
├── 2_room_thermostat/               # Project folder
│   └── .gitkeep
│
├── 3_circular_building/             # Project folder
│   └── .gitkeep
│
├── gui_robot.py                     # Root shortcut to launch robot interface
├── .gitignore                       # Repository ignore rules
└── README.md                        # Documentation
```

---

## Requirements and Installation

Python 3.9+ with the following packages:

```powershell
pip install numpy matplotlib numba
```

*(Standard `tkinter` comes bundled with Python on Windows and macOS).*

---

## Running the Application

Launch the robot navigation interface:

```powershell
python 1_robot_navigation/run.py
```

### Interface Features

1. **Direct Map Manipulation**:
   - Drag the robot avatar to reposition starting coordinates `(x1, x2)`.
   - Drag inside any region (`R1`, `R2`, `R3`, `R4`) to reposition it.
   - Drag the corner handles of Obstacle `R4` to resize its dimensions directly on the map.
   - Click empty space on the arena to set the robot position.

2. **Controller Advisory Dialog**:
   - If regions or obstacles are moved or resized, the interface prompts the user before running, offering to recalculate the reachability controller for the new layout or proceed with the existing controller.

3. **Multi-Trajectory Noise Simulation**:
   - The `50 Noise Runs` button simulates 50 independent closed-loop trajectories under random disturbance noise `w in [-0.05, 0.05]^3`.
   - Plots the trajectory bundle with individual markers to visualize closed-loop robustness under process noise.

4. **Telemetry and DFA Tracking**:
   - Real-time display of position, heading angle, active control inputs `(v, w)`, time step, and active automaton state.
