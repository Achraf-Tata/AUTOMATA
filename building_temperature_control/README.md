# Building Temperature Control

Symbolic safety control for heated rooms connected in a ring. Each local controller uses a temperature abstraction and bounds on its neighbors to maintain a prescribed comfort interval.

[Repository overview and team](../README.md) · [Build and run](../README.md#building)

## Thermal model

Room $i$ exchanges heat with the exterior and with its two neighbors. Neighbor indices wrap around the ring.

$$
\dot T_i=-\alpha(T_i-T_e)
-\beta\bigl((T_i-T_{i-1})+(T_i-T_{i+1})\bigr)
+\gamma u_i.
$$

The simulation uses a forward Euler step with $\tau=1$ s. The default model has $\alpha=0.05$, $\beta=0.02$, $\gamma=1$, an outdoor temperature in $[5,12]$ °C, and heater commands $u_i\in\{0,1\}$. The desired room-temperature interval is $[19,22]$ °C.

## Local abstraction and contracts

The local grid divides $[15,25]$ °C into 50 intervals. Assuming both neighbors remain in $[19,22]$ °C, their combined influence and the outdoor temperature give

$$
W_i=\alpha\tau T_e+\beta\tau(T_{i-1}+T_{i+1})\in[1.01,1.48].
$$

The local sampled model becomes

$$
T_i^+=0.91T_i+W_i+u_i.
$$

For each interval and heater command, the abstraction computes every possible successor interval. A safety fixed point retains states with at least one command whose successors all remain in the current candidate set. The extracted controller selects a safe heater command.

Compatibility is checked by comparing each local invariant with the neighboring-room assumptions. Under these assumptions, room controllers can be synthesized independently using OpenMP. The local representations avoid explicitly constructing the global $50^N$ state space.

## Benchmarks and simulation

The `circular_building` executable runs the following steps.

1. Check the monotonicity condition used by the thermal model.
2. Print estimates of global grid sizes and one-byte-per-state storage as the number of rooms grows.
3. Run the separate course competition benchmark with a 10-cell local grid, safe temperatures in $[17,22]$ °C and heater inputs $\{0,0.3,0.6\}$.
4. Benchmark OpenMP synthesis for networks from 4 to 1,000 rooms.
5. Simulate 16 coupled rooms over 86,400 one-second steps with outdoor temperature $T_e(t)=8.5+3.5\sin(2\pi t/86400)$.

The program writes temperature and heater samples every minute to `circular_building_24h.csv` in its working directory. The file checked into this folder is an existing simulation output.

The global-grid comparison reports storage estimates and feasibility labels. The competition benchmark uses a separate parameter set from the 16-room simulation, and its reported memory figure is an estimate.

## Build and verification

Use the [main build instructions](../README.md#building) from the repository root. The CMake targets are `circular_building` and `test_building`. OpenMP is required.

The tests exercise monotonicity, interval successor bounds, local invariant synthesis, contract compatibility, and an eight-room closed-loop simulation over 2,000 steps. Use a Debug build to keep assertion checks enabled.

## Source layout

| File | Role |
| --- | --- |
| `include/circular_network.hpp` | Coupled thermal dynamics and extreme-trajectory propagation |
| `include/contract_abstraction.hpp` | Temperature grid and local successor intervals |
| `src/contract_solver.cpp` | Safety fixed point and controller extraction |
| `src/monolithic_baseline.cpp` | Global storage estimates and OpenMP synthesis |
| `src/main_building.cpp` | Benchmarks, 24-hour simulation and CSV output |
| `tests/test_building.cpp` | Thermal network verification suite |
