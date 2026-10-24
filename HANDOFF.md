# MeshMerize Simulator: Agent Handoff Brief

Put this file in the repo root and read it completely before touching any code.

> **Status note.** This brief was written before the simulator was built out.
> Sections 4, 5 and 8 describe that earlier state. For what exists now, how to
> use it and the current list of assumptions, read `README.md`. The rules of
> engagement (section 0), the competition facts (section 2) and the coding
> rules (section 6) still apply unchanged.

---

## 0. Read this first

**This project is extremely important to its owner (Ajay).** He is an electronics engineering student competing in Meshmerize at Techfest, IIT Bombay (finals 16–18 December 2026). He has limited hardware access, so this simulator is how he will develop and tune his maze-solving strategy. A wrong result from the simulator can send him into the finals with a bad strategy. **Correctness and honesty matter more than speed or impressive-looking output.**

Rules of engagement, in priority order:

1. **Never claim something works unless you ran it.** Show the real terminal output of `make test`. If you could not run something, say so plainly.
2. **Never invent competition rules, hardware specs or measurements.** Anything unknown is an *assumption*: name it, put it in one clearly labelled place in the code, and list it in the README. Section 2.3 lists what is unknown.
3. **Small steps, each tested and committed.** One feature, its tests, one commit with a clear message. Do not do large rewrites in one go.
4. **Test labels must say what the condition checks.** Earlier in this project a label said "3 edges" while the condition checked 2 edges, and another label described a path that was not the one being tested. A mislabelled test is dangerous because people trust the text on screen when something fails.
5. **Teach as you go.** The owner is learning (he knows basic C/C++ and Dijkstra theory; structured firmware-style code is new to him). Explain each new piece briefly *before or together with* the code: what it is, why it is designed that way, and what could go wrong. Do not pad with filler. If a point is not important, skip it.
6. **Do not rewrite his files wholesale.** Some files were retyped by the owner in his own style (for example `sim/walker.h` uses member names like `back_exit` and `total_mm`; tests use `name: OK/FAIL` output). Make targeted edits and keep his naming style.
7. **Ask before big design changes** (new dependencies, changing the core data structures, changing the build system).
8. **No new dependencies.** The project must build with only `g++` and `make` on Ubuntu 22.04 (g++ 11.4, GNU Make 4.3). It is meant to be shared with friends' teams who should not have to install anything.

---

## 1. What we are building and why

A **standalone C++17 simulator and strategy lab** for the Meshmerize maze-solving robot competition. Goals:

1. **Develop and test the robot's maze logic on a PC** before (and while) hardware is available. The same `core/` code later compiles for the ESP32 and runs on the robot ("same code in simulation and on the robot").
2. **Answer strategy questions with data**, not guesses. Mainly: how much should the robot explore during the Dry Run, since every second costs points but a missed shortest path costs far more (see section 2.2)?
3. **Be shareable.** 3–4 friends' teams may use it, some with their own heavily optimized algorithms. They plug in through a small, documented interface.
4. **Later: run on a real ESP32** for timing and memory benchmarks (section 8, items P8/P9).

A full ROS 2 / Gazebo simulation was considered and **rejected on purpose**: the robot is a tiny ESP32 car, Gazebo's physics would give false confidence, ROS 2 does not run on the ESP32 (so firmware would have to be rewritten), and setup would eat weeks. Do not reintroduce it.

---

## 2. The competition (facts from the official site)

Source: the Techfest Meshmerize page. These are the only confirmed facts. Re-read the organizers' page and the rules email whenever you can; the site says the website is the latest authority.

### 2.1 Rules

- Autonomous robot follows a **white line** through a **230 cm × 230 cm** maze.
- One attempt per team: **Dry Run (up to 3 min)** then **Actual Run (2 min 30 s, reduced by any Dry Run overrun)**, total 5 min 30 s.
- Dry Run: robot goes from Start to End, stores the path, **red LED glows on arrival at End**. No restriction on covering checkpoints.
- Actual Run: robot restarts from Start and must retrace the **shortest path it found**.
- Checkpoints: **25 points each**, counted once regardless of which run crosses them.
- **Max 3 restarts** in the whole match. Dry Run restart returns the bot to the **last checkpoint**; Actual Run restart returns it to the **start zone**. **The timer never pauses or resets.**
- Robot must fit in **220 × 220 × 220 mm**, onboard power only, **max 24 V DC**, no readymade Lego kits (readymade gear assemblies are allowed), must not damage or mark the arena, must not split into multiple units.
- **No laptops near the arena during the run, and WiFi/Bluetooth devices must be off.** Only one team member handles the robot. (So the ESP32's WiFi/Bluetooth must be disabled in firmware, and there is no serial monitor during runs.)
- Qualifying round: top 5 teams go to the Grand Finale, together with zonal finalists.

### 2.2 Scoring

```
Total = A + B + C + S + L + T - P
A = 25 x checkpoints covered
B = 30 if Dry Run completed
C = 180 - Dry Run time in seconds
S = 30 if the shortest path is completed
L = 5 if the LED glows
T = 150 - Actual Run time in seconds   (only if the shortest path is used)
P = penalties (not defined on the site)
```

**Strategic consequence (important):** finding the true shortest path unlocks `S` and `T`, worth roughly 30 + 130 or more, about 160 points. Each extra second of Dry Run costs 1 point. So thorough exploration is usually worth it **as long as the Dry Run stays under 3 minutes**. A fixed rule such as "stop as soon as End is found" is probably wrong on many mazes, and right on others (when the first route found is already the shortest). The simulator exists to find the best exploration policy.

### 2.3 Unknowns (do not guess silently)

The owner will ask the organizers; until answered, each is an assumption with a configuration value:

| Unknown | Current assumption |
|---|---|
| Maze layout: grid with 90° turns only, or also diagonals/curves? Loops possible? | Design is graph-based so both work. Generator produces grid mazes, optionally with loops. |
| Grid size / line width / cell size | 10×10 cells, 230 mm per cell (matches 230 cm arena), configurable |
| How a checkpoint looks / where it can be | Not modelled yet (node type `Checkpoint` exists) |
| Definition of "shortest": distance or time? | **Distance** (sum of edge lengths). Make weights configurable later (P7). |
| When the Dry Run clock stops (arrival at End? robot stationary?) | Stops on arrival at End |
| Time to carry the robot back to Start between runs | Zero |
| What happens to `C` if Dry Run exceeds 180 s; how overrun reduces the Actual Run | `C` clamped at 0; overrun handling not modelled yet |
| Penalty definitions (`P`) | Not modelled |
| Exact time lost by a restart | Not modelled |

---

## 3. The robot (what the model assumes)

- ESP32 (PlatformIO, board `esp32dev`, Arduino framework in the firmware repo).
- **Two N20 gear motors with quadrature encoders**, robot kept as light as possible.
- Assumed wheel diameter 32 mm (about 100.5 mm per wheel turn). Typical N20 encoder motors give roughly 600–900 counts per output revolution, so about 0.1–0.17 mm per count, far finer than needed. The motor's no-load speed (about 0.8–1.7 m/s depending on the variant and voltage) is **not** the real limit; line sensing, control loop and turning are.
- **Placeholder timing model (guesses, to be replaced with measurements):**
  - Dry Run: 0.5 m/s average, plus 0.30 s at every node
  - Actual Run: 0.9 m/s average, plus 0.25 s at every node
- These live in `RobotModel` (`sim/timed_walker.h`). Results from the simulator are good for **comparing strategies**, not for predicting absolute seconds, until calibrated with real measurements.

---

## 4. Current repo state

Project folder: `meshmerize-sim`. Plain C++17, Makefile, VS Code, Ubuntu.

```
core/
  maze_graph.h   MazeGraph: fixed-size nodes and edges (no heap)
  solver.h       Dijkstra: shortestPath(graph, start, goal, Path&)
  explorer.h     explore<W>(walker, known_graph, stop_at_end): depth-first exploration
sim/
  walker.h       Walker: the robot's "keyhole" into the hidden true maze
  timed_walker.h RobotModel + TimedWalker (a Walker that keeps the clock)
  scoring.h      ScoreBreakdown + playAttempt(): Dry Run + Actual Run + official formula
tests/
  test_hello.cpp, test_graph.cpp, test_solver.cpp,
  test_walker.cpp, test_explorer.cpp, test_scoring.cpp
Makefile         builds and runs every tests/test_*.cpp
```

Build and test: `make test` (stops at the first failing test program; `make clean` removes `build/`). Makefile uses `.RECIPEPREFIX = >` so commands start with `>` instead of a Tab. Flags: `-std=c++17 -Wall -Wextra -Wpedantic -O2 -Icore -Isim`. The build must stay **warning-free**.

Status at handoff: all tests in the 6 test programs passed (owner pasted the output). The old `~/projects/meshmerize` firmware repo (PlatformIO, ESP32) is a **separate** project and must not be modified from here.

### 4.1 Key design decisions already made

- **The maze is a graph, not a grid.** Nodes are start, end, checkpoints, dead ends, junctions and turns; edges are corridors with a length in integer **millimetres**. A straight corridor is a single long edge. This handles grid and non-grid mazes alike, and loops.
- **Fixed-size arrays, no `std::vector`/`new`** in `core/` (constants `MAX_NODES=256`, `MAX_EDGES=512`, `MAX_EXITS=4`). The ESP32 has little RAM and heap growth mid-run can fail.
- **Failed operations return -1 and change nothing** (`addEdge` must not half-add an edge).
- **`INF_MM = 0x3FFFFFFF`**, not `INT_MAX`, so additions cannot overflow.
- **The explorer is a template over a "Walker concept"** (interface below) so the same explorer runs against the simulator now and the real robot later.
- Dijkstra scans for the minimum node (O(n²)), which is fine for n ≤ 256 and simple to verify.

### 4.2 The Walker concept (interface contract for all robot back-ends and friends' algorithms)

```
int      here()       current node id (see known issue 2)
NodeType type()       type of current node
int      exitCount()  number of exits at the current node
int      backExit()   index of the exit that leads back to where we came from (-1 at start)
int      go(int k)    drive through exit k; returns distance in mm, or -1 if k is invalid (and does not move)
int      totalMm()    total distance driven so far
```

### 4.3 Test conventions

- Each test file has `static void check(const char* name, bool ok)` printing `name: OK` or `name: FAIL`, counts failures, and `main` returns the failure count (non-zero stops `make`).
- Every new feature gets tests that cover: normal case, edge cases, bad input, and (where possible) **invariants that do not depend on the implementation** (for example "edges = nodes - 1 for a maze without loops").

---

## 5. Known issues and weaknesses (fix or consciously accept; do not hide)

1. **Scoring bug: the Dry Run must end at End.** `playAttempt` currently stops the Dry Run clock wherever exploration ends. After a *full* exploration the robot is back at Start, but the rules say the Dry Run finishes with the LED glowing at End. Fix: if the explorer's final position is not End, add the time to drive the known shortest path to End (`path_mm / dry.speed + path_edges * dry.node_cost`). **Expected changes in `tests/test_scoring.cpp` after the fix:** the two "full" scores change from `387.08` to `386.08` (exactly 1.0 s more Dry Run in those two mazes); the two "early stop" scores (`212.4` and `393.28`) do not change, because the robot is already at End. Update the test names too.
2. **The Walker hands the explorer the true node id.** A real robot must recognise "I have been here before" from encoder odometry and heading, which drifts. This is the biggest gap between the simulator and the robot (see P6).
3. **The explorer swallows errors:** if `known.addEdge/addNode` fails (graph full, self-loop, capacity) it silently `break`s and returns a result that may look fine. Add an explicit error flag/status to `ExploreResult` and test it.
4. **`MazeGraph` rejects self-loops and has `MAX_EXITS=4`.** Grid mazes cannot produce self-loops or parallel edges (every cycle contains at least 4 turn nodes), but hand-built or curved mazes could. Decide consciously and document.
5. **Large objects on the stack.** `MazeGraph` is about 12 KB; the explorer uses about 4 KB of local arrays; Dijkstra about 3.3 KB; `Path` about 2 KB. On the ESP32 (Arduino loop task stack is typically 8 KB, verify) a `MazeGraph` must be a **global or static**, never a local variable.
6. **Dijkstra tie-breaking is arbitrary.** Among equal-length paths, prefer fewer turns (turns cost time), once a time-based weight exists (P7).
7. **Time model is per node, not per turn.** Every `go()` costs `node_cost` whether it is straight or a turn. Acceptable for the first model, replace in P6.
8. **Checkpoints, restarts, penalties and overrun are not modelled** (A and P are 0).

---

## 6. Coding rules

- `core/` = **pure, portable C++17**. No `iostream`, no `printf`, no STL containers, no dynamic allocation, no exceptions, no Arduino/ESP32 calls, no PC-specific headers. It must compile unchanged for the ESP32. (Header-only is fine.)
- `sim/` and `tests/` may use the standard library (`<cstdio>`, `<cmath>`, `<cstdint>`...) but **no third-party libraries**.
- Deterministic randomness: use the project's own small RNG with explicit seeds (not `rand()`/`<random>` defaults), so the same seed gives the same maze on every machine.
- Integers in millimetres for lengths in `core/`; floating point only in `sim/` timing and scoring.
- Comment the *why*, not the obvious. Keep the owner's readable, simple style.
- Commit after every verified step. Commit messages say what and why.

---

## 7. Definition of done for every feature

1. Spec understood; assumptions written down (in code constants and in the README).
2. Tests written that would fail if the feature were wrong, including invariants and bad-input cases.
3. `make clean && make test` passes with **zero warnings**; real output pasted to the owner.
4. Short plain-language explanation of the new code for the owner.
5. One commit.

---

## 8. Roadmap (do in this order)

### P0: Fix and verify what exists
- Apply known issue 1 (Dry Run ends at End) and update the test expectations as described.
- Add known issue 3 (error flag in `ExploreResult`) with a test that fills the graph on purpose.
- Make sure the Makefile dependency line watches `core/*.h` **and** `sim/*.h`, so a changed header triggers a rebuild:
  `$(BUILD)/%: tests/%.cpp $(wildcard core/*.h sim/*.h) | $(BUILD)`
- Commit.

### P1: Random maze generator (`sim/maze_gen.h`, tests in `tests/test_maze_gen.cpp`)
Spec:
- Own xorshift32 RNG (seed 0 must not break it). Same seed gives the same maze.
- `GridMaze`: width, height, passage flags per cell and direction (0 north, 1 east, 2 south, 3 west; opposite = `(d+2)%4`). Max 20×20 cells.
- `generateGrid(maze, w, h, extra_openings, seed)`: randomized depth-first carving (a perfect maze: exactly one route between any two cells), then knock out `extra_openings` more walls; **each one creates exactly one loop**. Reject sizes < 2 or > the limit.
- `buildGraph(maze, cell_mm, graph, start, end)`: Start = top-left cell, End = bottom-right cell. Nodes are Start, End, dead ends, junctions and **turn cells**; straight corridor cells are skipped and become part of one long edge (length = steps × cell_mm). Each corridor is added exactly once.
- Tests (invariants, independent of the implementation): corridor of 4 cells gives 2 nodes and one 690 mm edge; 2×2 ring gives 4 nodes, 4 edges, shortest 460 mm; perfect maze: `edges == nodes - 1` and total edge length `== passages × cell_mm` (99 passages in 10×10); with `k` extra openings: `edges - nodes + 1 == k` and total length `== (99 + k) × cell_mm`; End reachable; same seed gives identical results; bad sizes rejected.
- Also add an **ASCII maze printer** (no dependencies), runnable as e.g. `make show SEED=7`, to eyeball a maze and the shortest path. Useful for debugging.

### P2: Batch runner (`tools/compare.cpp`, Makefile target `make compare`)
- For seeds 1..N (default 500) and configurable maze size / extra openings, run every strategy through `playAttempt` and report per strategy: **mean and std-dev of total score, % of attempts that found the true shortest path, mean Dry Run time, % of attempts with Dry Run > 180 s, mean Actual Run time**.
- Deterministic. Plain text table, plus an optional CSV file.
- Sanity checks in tests: results must be identical across two runs with the same seeds.

### P3: Exploration strategies (pluggable, each compared in P2)
1. Stop as soon as End is found (existing, `stop_at_end = true`).
2. Explore everything (existing, `stop_at_end = false`).
3. **Nearest-unexplored-first** instead of plain depth-first backtracking (less wasted driving).
4. **Time-budget guard:** keep exploring only while the estimated time to return to End and finish stays under the 180 s limit with a safety margin; otherwise go to End with the best path known.
5. **Position-aware pruning:** give nodes (x, y) estimated from odometry and heading; after End is found, skip branches that cannot beat the best known path (lower bound = distance so far plus straight-line distance to End). Needs P1.
Report honestly which strategy wins under which maze statistics; do not oversell. If the data says "explore fully unless time runs short", say that.

### P4: Checkpoints (A = 25 each)
Model checkpoint nodes (placement unknown: make it configurable), count each once across both runs, and let strategies trade a detour against the 25 points.

### P5: Restarts and penalties
Hooks only until the organizers define them: restart count (max 3), Dry Run restart returns to last checkpoint, Actual Run restart returns to Start, timer keeps running.

### P6: More realistic robot model (replaces the per-node timing)
- **Odometry-based node recognition with noise and drift** (replaces the true-node-id cheat), with loop-closure by position tolerance. Test the failure modes (false merge, missed loop).
- Differential-drive kinematics: motor lag (first-order), max acceleration (stands in for grip), encoder quantization (use the real counts per revolution), line-sensor array position, noise and delay, control-loop rate.
- Only model what changes decisions; no physics engine. All parameters in one `RobotParams` struct that can be loaded from a config file and overwritten with measured values.

### P7: "Shortest" by time instead of distance
Make edge weight configurable (distance, or estimated traversal time including a turn penalty) and tie-break on fewer turns. Compare how often the two definitions pick different paths.

### P8: Documented plug-in interface for friends' algorithms
README section: the Walker concept (section 4.2), how to write your own explorer/solver, how to run `make compare` with it, determinism rules.

### P9: ESP32 benchmark mode (separate PlatformIO project; do not edit the old firmware repo without the owner's go-ahead)
- Compile `core/` unchanged for the ESP32 (copy, symlink or git submodule; ask which).
- Benchmark sketch: run the solver and explorer on generated mazes on the chip and report **compute time and RAM/stack use** (cycle counter or `esp_timer`), with **WiFi and Bluetooth off** (they are not allowed at the competition).
- Later, optional: lock-step hardware-in-the-loop over USB serial (simulator sends fake sensor data, ESP32 returns motor commands; measure the chip's real per-cycle compute time and feed it back into the simulator as latency).

### P10: Calibration
Procedure and script to replace the placeholder speeds/delays with values measured on the real robot (speed, acceleration, turn time, encoder counts per metre).

---

## 9. Report format for each step

Tell the owner, briefly: what changed, why, the exact `make test` output, which assumptions were used, what is still unverified, and what you recommend next. If a result surprises you, say so instead of explaining it away.
