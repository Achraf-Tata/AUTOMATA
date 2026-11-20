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
2. Compute every possible observation over each whole spatial cell and combine the cells with the mission automaton. Cells intersecting the closed obstacle are excluded.
3. Start backward reachability from the accepting mission states.
4. Add a state when an input sends every possible spatial successor and observation into an already-winning state. Reject inputs whose reachable box extends outside the workspace.
5. Store a control input for each winning state and use it in the closed-loop simulation.

OpenMP can parallelize the full-grid solver. The default console and GUI share this solver, the simulation engine, and the same control samples. A generic grid implementation is also available for discretization experiments and safety games.

## Applications and tests

| CMake target | Purpose |
| --- | --- |
| `prototype_gui` | Interactive arena, synthesis controls, animation and trajectory bundles |
| `prototype_navigation` | Full-resolution console mission; optional custom-grid experiment |
| `test_prototype` | Model, abstraction, automaton, synthesis and simulation checks |
| `test_robot_regressions` | Workspace exits, region boundaries, worker lifetime, final-step arrivals and cache failures |
| `test_robot_certificate` | Policy rank certificates, interval corner checks, parallel equivalence and reference scenarios |

Configure from the repository root using the [main build instructions](../README.md#robot). With Ninja or Unix Makefiles, run the default mission and all registered tests with

```sh
./build/robot/prototype_navigation
ctest --test-dir build/robot --output-on-failure
```

For Windows Ninja, add `.exe` to executable names. For Visual Studio, use `build/robot/Debug/prototype_navigation.exe` and add `-C Debug` to CTest.

The default console starts at $(1,2,0)$ and reaches the goal in 22 steps without disturbances. Passing three grid dimensions, for example `prototype_navigation 40 40 16`, selects the generic solver. Coarse abstractions can produce an empty winning set; the program returns a nonzero status when synthesis or the mission fails. CTest includes a deliberately coarse 10 × 10 × 4 failure case.

Use a Debug build to retain the legacy assertion-based checks. The regression and certificate executables use explicit checks that also remain active in Release builds.

## Using the GUI

1. Set the robot position and heading and adjust the regions if needed.
2. Select **Run solver** to synthesize a controller for the current arena.
3. Run the simulation with or without bounded disturbances.
4. Display a bundle of trajectories to inspect how different disturbance realizations affect the route.

The application automatically caches the policy in `controller.bin` in the working directory and restores a matching cache at startup. The Controller card provides **Run solver** and status information; there are no manual Save or Load buttons. Launch from a writable directory for caching to work. The `CTRL2.00` format records the grid, model parameters, controls and region coordinates, and checks the payload before loading. Writes replace the file atomically; failed loads preserve the active tables. Older files must be regenerated with **Run solver**.

Moving or resizing a region invalidates the controller. Synthesis uses an independent snapshot of the arena; a result for an edited arena is discarded. Closing the window cancels and joins an active calculation.

## Validation

The five CTest groups pass on Windows with GCC and OpenMP. The default full-grid fixed point converges in 36 iterations. The stronger treatment of obstacle contact and ambiguous observations can change the policy and iteration count from the paper's illustration.

| Check | Result |
| --- | --- |
| Selected-policy rank decreases for every symbolic successor, default map | 623,272 nonterminal winning states checked |
| Same certificate after shifting R1 by 0.03 m and the obstacle by 0.04 m | 627,871 nonterminal winning states checked |
| Sequential versus OpenMP synthesis | Identical value and control tables |
| Reference start $(0.5,4.5,0)$ | Goal in 24 steps; 50/50 disturbed runs |
| Reference start $(0.5,4,0)$ | Goal in 24 steps; 50/50 disturbed runs |
| Reference start $(6.8,9,0)$ | Goal in 20 steps; 50/50 disturbed runs |
| Concrete image coverage | Sampled cell corners and disturbance corners lie in their computed successor ranges |
| GUI smoke checks | Synthesis, goal arrival, 50 disturbed trajectories and controller invalidation after a region edit |

The rank certificate checks the constructed symbolic transition relation exhaustively for the chosen commands. It complements the concrete corner checks and simulations; the simulations alone are not a proof. The model is a point robot observed at discrete sampling times.

## Implementation notes

Region masks retain multiple possible observations when a waypoint or goal crosses a cell boundary. The solver quantifies over all those observations. The simulator uses the actual point observation and preserves the physical successor even when it leaves the workspace.

Worker threads own their input snapshots and result arrays. Only atomic progress and cancellation flags are shared; the main thread publishes a completed policy. Cancelled, stale or iteration-limited results are not activated.

The theoretical reference is the robot example and the abstraction, reachability and automaton constructions in Girard, Meyer and Saoud. [Paper](https://doi.org/10.51257/a-v1-s7467)
