# Robot Navigation

A C++20 implementation of abstraction-based control for a mobile unicycle robot. A finite-state mission automaton tracks whether the robot has visited waypoint R1 or R2 before reaching goal R3, while avoiding obstacle R4.

[Repository overview and team](../README.md) · [Build and run](../README.md#robot)

## Model and mission

The state is the robot position and heading, $x=(x_1,x_2,\theta)$. The control input is linear and angular velocity, $u=(v,\omega)$. At each sampling step,

$$
x_1^+=x_1+\tau(v\cos\theta+w_1),\qquad
x_2^+=x_2+\tau(v\sin\theta+w_2),\qquad
\theta^+=\theta+\tau(\omega+w_3)\pmod{2\pi}.
$$

The graphical application uses a 10 × 10 m workspace, a one-second sampling period, and disturbances in $[-0.05,0.05]$ on each state component. Its spatial grid has 100 × 100 × 30 cells. Combining these cells with five automaton states gives 1.5 million product states.

The 15 controls combine speeds $\{0.25,0.625,1.0\}$ m/s with angular velocities $\{-1,-0.5,0,0.5,1\}$ rad/s.

| Region | Position in the default arena | Role |
| --- | --- | --- |
| R1 | $[4,5]\times[8.5,9.5]$ | First waypoint option |
| R2 | $[8.5,9.5]\times[2,3]$ | Second waypoint option |
| R3 | $[2,3]\times[0.5,1.5]$ | Final destination |
| R4 | $[3,7]\times[3,7]$ | Obstacle |

The mission automaton has five states. It starts without a visited waypoint, records the visit to R1 or R2, accepts when R3 is reached afterward, and enters a failure state if the obstacle or the other waypoint is visited before completion. Simulation stops when the mission succeeds or fails.

## Controller synthesis

1. Compute coordinate-wise successor intervals using the state-cell widths, bounded derivatives and disturbance bounds.
2. Assign region observations and combine spatial cells with the mission automaton.
3. Start backward reachability from the accepting mission states.
4. Add a state when an input sends every possible successor into an already-winning state.
5. Store a control input for each winning state and use it in the closed-loop simulation.

OpenMP can parallelize the full-grid solver. A separate generic implementation provides the console prototype and safety-game routines.

## Applications and tests

| CMake target | Purpose |
| --- | --- |
| `prototype_gui` | Interactive arena, synthesis controls, animation and trajectory bundles |
| `prototype_navigation` | Console walkthrough with configurable grid dimensions |
| `test_prototype` | Model, abstraction, automaton, synthesis and simulation checks |

Configure from the repository root using the [main build instructions](../README.md#robot). For a single-configuration build, the console accepts grid dimensions as three arguments.

```sh
./build/robot/prototype_navigation 40 40 16
```

The console uses speeds $\{0.25,0.50,0.75\}$ m/s and defaults to a 30 × 30 × 12 grid. Its default run currently reports `UncontrollableState` from the selected starting pose.

The test suite checks interval and box operations, grid indexing, DFA word acceptance, product states, safety invariance, full-resolution synthesis, binary controller round trips, and closed-loop trajectories. Use a Debug build to keep assertion checks enabled.

## Using the GUI

1. Set the robot position and heading and adjust the regions if needed.
2. Select **Run solver** to synthesize a controller for the current arena.
3. Run the simulation with or without bounded disturbances.
4. Display a bundle of trajectories to inspect how different disturbance realizations affect the route.

The default policy file is `controller.bin` in the working directory. The interface can save and load policy tables. Recompute the policy after changing the regions, since the current binary format does not record the arena configuration.

## Implementation notes

Region classification currently uses cell centers, and the simulation clamps positions at workspace boundaries. These cases need further validation against the paper before making a full correctness claim. Background synthesis also needs synchronization of shared state and worker lifetime management.

The theoretical reference is the robot example and the abstraction, reachability and automaton constructions in Girard, Meyer and Saoud. [Paper](https://doi.org/10.51257/a-v1-s7467)
