# Building thermostat

The CPU controller uses the course's discrete-time model for a ring of rooms. Each
room observes its own temperature and chooses from the same local policy.
The contract bounds both neighbors to [17, 22] °C.

## Build and run

From this directory on Ubuntu or WSL, build the GUI with CMake, a C++20 compiler
and the development packages for OpenGL and X11. The GUI needs a graphical
session, such as WSLg on Windows.

```sh
sudo apt install build-essential cmake git libgl1-mesa-dev xorg-dev
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/thermostat_gui
```

In the Simulation tab, **Apply & reset** sets up a live simulation for up to
1,000,000 rooms. **Certify** evaluates the entered room count using the shared
policy, so larger buildings can be certified with constant memory in the room count.

CMake fetches GLFW and Dear ImGui on the first GUI build. If the robot build
already downloaded both, reuse their source directories.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DFETCHCONTENT_SOURCE_DIR_GLFW="$PWD/../build/robot/_deps/glfw-src" \
  -DFETCHCONTENT_SOURCE_DIR_IMGUI="$PWD/../build/robot/_deps/imgui-src"
cmake --build build -j
```

For a headless build, use a separate build directory.

```sh
cmake -S . -B build/headless -DCMAKE_BUILD_TYPE=Release -DTHERMOSTAT_BUILD_GUI=OFF
cmake --build build/headless -j
ctest --test-dir build/headless --output-on-failure
./build/headless/thermostat_cli 1000000
./build/headless/thermostat_cli 8 200
```

Give the CLI a room count to time and certify the controller. Add a tick count to
simulate the coupled building from alternating 17 and 22 °C temperatures.
Timing is reported in microseconds. Peak process RSS includes the executable's
runtime overhead and is also printed after simulation when requested.

## Code structure

| File | Responsibility |
|---|---|
| `src/abstraction.cpp` | Quantize one temperature and build the 10 × 3 transition table. |
| `src/controller.cpp` | Compute the safety fixed point, choose an allowed action, and report the certified fraction. |
| `src/building.cpp` | Run the actual coupled model, with all room updates reading the same snapshot. |
| `src/main.cpp` | Run a headless certification benchmark and optional simulation. |
| `src/gui.cpp` | Configure, run and inspect the same controller with a temperature plot and a transition table. |

## Why the controller works

The local grid has ten half-degree bins per room. The three actions are
0, 0.3 and 0.6. Under the contract, the contribution of the neighbors,
0.45 times the sum of their temperatures, lies in [15.3, 19.8].
For fixed heater power, the next room temperature increases with the current
temperature and the disturbance. Evaluating the two extreme corners bounds
all successors. The transition table represents each in-range successor in a
bitmask and sets an extra bit whenever the interval reaches outside the zone.

The fixed point keeps a bin if some action has a nonempty set of successors
and every successor remains in the candidate set. All ten bins survive for
this model. The computed action is 0.3 in each bin, which puts the next
temperature in [17.081, 21.721] °C over the entire contract.

When all rooms start in range, the guarantees of the neighboring rooms make
each contract assumption true. Each local action keeps its room in range
through the next tick, so the assumptions hold again. Induction certifies
all 10^N safe initial cells for every N ≥ 3. The transition table and policy
are shared across all rooms. Building them costs O(1) in N. The simulator
uses two temperature vectors and takes O(N) time per tick.

The contract decomposition follows the totally decentralized case in §V-A of
[Saoud, Girard and Fribourg, IEEE TAC](https://doi.org/10.1109/TAC.2020.2992446).
The safety proof above applies to the course's synchronous discrete update.
