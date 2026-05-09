# MediRoute

**MediRoute** is an adaptive autonomous mobile robot designed to optimise internal hospital logistics in simulation. The system transports medication and laboratory samples between areas such as **Pharmacy**, **ICU**, **Laboratory**, and **Ward**, using a **2D grid-based hospital model** and the **A\* algorithm** for route planning. Unlike static navigation approaches, MediRoute supports **dynamic replanning**, **Q-Learning cost-map adaptation**, and **real-time task queue management** in a changing indoor environment.

## Project goals

The main objective of MediRoute is to improve hospital internal logistics by reducing transport times, improving delivery efficiency, and decreasing the non-clinical workload of healthcare staff. The project focuses on simulation-based validation of adaptive routing behaviour in hospital-like environments, with an emphasis on machine-learning-driven cost adaptation that makes the robot progressively better at avoiding congested corridors.

## Main features

- **Task queue management** — priority-based queue for origin-destination delivery requests, manageable at runtime via REST API
- **Hospital representation as a 2D grid** — 28 × 22 cell map aligned to the Webots world geometry
- **A\* route planning** on a weighted cost map
- **Cell-by-cell route execution** with odometry control
- **Multi-trigger dynamic replanning** — five independent replanning conditions per route step
- **LIDAR-based obstacle detection** — real-time frontal scan blocks cells and triggers immediate rerouting
- **Block decay** — LIDAR blocks expire after a configurable number of traversals and become tentatively passable again
- **GPS position correction** — absolute cell position read after each move to correct odometry drift
- **Q-Learning cost-map adaptation** — tabular Q-Learning updates cell costs from observed traversal times, making A\* avoid corridors that have been slow in previous tasks
- **Explicit cell states** — `FREE`, `CONGESTED`, `BLOCKED`
- **Travel-time monitoring** per cell and per task
- **Performance metrics** per task and per cell
- **Cost-map event log** — timestamped file log of every block/unblock event with full map snapshots
- **Real-time task dispatch** via REST + WebSocket with a minimal web UI

## Architecture overview

The system runs entirely inside **Webots**, which provides the simulation platform, robot model, sensors, and world. The controller logic is structured in five layers:

1. **Simulation Environment Layer**
   Webots world with the Pioneer 3-DX robot, hospital corridors, rooms, and dynamic obstacles.

2. **Environment Representation Layer**
   Logical 2D grid abstraction of the hospital. Each cell stores a traversal cost, an explicit state (`FREE`/`CONGESTED`/`BLOCKED`), and a 4-action Q-table entry.

3. **Navigation and Planning Layer**
   A\* path planning over the weighted grid. When Q-Learning has updated cell costs from experience, A\* automatically favours lower-cost (less congested) corridors on future tasks.

4. **Monitoring and Adaptation Layer**
   Real-time observation of traversal times, LIDAR readings, and GPS position. Costs are updated after every cell via the Bellman equation. LIDAR blocks and their decay are managed here.

5. **Evaluation Layer**
   Per-task and per-cell performance summaries, Q-table summary with a cost-map heat map printed at the end of the simulation.

## Technological baseline

MediRoute builds on the **Webots** simulation platform and uses the **Pioneer 3-DX** robot model as the mobile base. The following capabilities are provided by Webots out of the box:

- robot body and differential-drive wheel configuration
- wheel motors and wheel encoders
- LIDAR range sensor
- GPS sensor
- simulation timing and controller execution loop

The project contribution is therefore the **adaptive hospital logistics behaviour** layered on top of that baseline:

- hospital 2D grid representation and cost map
- A\* route planning with dynamic weights
- task queue management and real-time REST API
- LIDAR-based obstacle detection and block decay
- GPS-based odometry correction
- Q-Learning cost adaptation
- multi-trigger replanning during route execution
- explicit cell-state management
- performance monitoring and event logging

## Q-Learning cost adaptation

After each cell traversal the robot applies the **Bellman update**:

```
Q(s, a) ← Q(s, a) + α · [r + γ · max_a' Q(s', a') − Q(s, a)]
```

| Symbol | Meaning |
|--------|---------|
| `s` | Destination cell of the current step |
| `a` | Direction used to enter `s` (N / E / S / W) |
| `r` | Normalised reward: `−(actual_time / expected_time)` |
| `s'` | Next planned cell (or terminal state at the goal) |
| `α` | Learning rate — `0.3` |
| `γ` | Discount factor — `0.0` (pure EMA of immediate reward, no future propagation) |

**Q-table initialisation.** Every cell starts at `Q_init = −1 / (1 − γ) = −1`, which is the expected immediate reward for a corridor traversed at ideal speed. With `γ = 0` the update reduces to an exponential moving average of the immediate reward, so Q-values never embed discounted future costs that A\* would then sum again.

**Converting Q-values to A\* costs.** The cost assigned to a cell is derived from the most-penalised direction (the `min` across all four Q-values). This prevents un-visited directions — which remain at `Q_init` — from masking a slow direction:

```
deviation = −min(Q[cell]) − |Q_init|      # 0 for ideal, positive for slow
cost      = clamp(round(deviation × 5 + 1), 1, 30)
```

Example convergence values (isolated cell, `γ = 0`):

| Traversal ratio | Q convergence | Learned cost |
|-----------------|---------------|--------------|
| 1× (ideal) | −1.0 | 1 |
| 2× slow | −2.0 | 6 |
| 3× slow | −3.0 | 11 |
| 5× slow | −5.0 | 21 |

After each Q update the corresponding `cost_map` entry is refreshed, so A\* immediately uses the new cost on the next replanning call.

**Interaction with LIDAR blocks.** When the LIDAR detects an obstacle, the cell is set to cost 999 (`BLOCKED`). After `BLOCK_DECAY_TRAVERSALS` (default 15) subsequent traversals the block expires and the cell is set to `TENTATIVE_BLOCK_COST` (25). If the robot then traverses the tentatively-unblocked cell slowly, Q-Learning raises its cost further; if quickly, Q-Learning reduces it toward 1, confirming the obstacle was transient.

## Current implementation

The controller supports:

- sequential execution of delivery tasks with configurable initial queue
- weighted A\* path planning with runtime-updated costs
- LIDAR obstacle detection and dynamic cell blocking
- block age tracking and automatic expiry (decay)
- GPS-based odometry correction after every move
- Q-Learning cost-map adaptation (tabular, 4 actions per cell)
- five-trigger dynamic replanning per route step:
  1. Proactive check — next cell blocked before moving
  2. LIDAR scan — obstacle detected after rotating toward next cell
  3. LIDAR recovery — tentative unblock and retry if no alternative route
  4. GPS correction — replan if GPS position is non-adjacent to planned route
  5. Dynamic event — cost/block changes triggered at a configurable cell
- metrics per task: travel time, traversed cells, replans, route cost
- metrics per cell: visits, rotation time, forward time, delays
- explicit cell-state tracking: `FREE`, `CONGESTED`, `BLOCKED`
- timestamped cost-map event log (`cost_map_events.txt`)
- Q-table summary and ASCII cost heat map printed at simulation end

## Real-time task server (REST + WebSocket)

The controller can host a lightweight task server that lets you push new origin/destination jobs while the simulation runs. A minimal web UI is served at the same address.

Environment variables:

| Variable | Default | Description |
|----------|---------|-------------|
| `HALO_TASK_SERVER_ENABLED` | `1` | Enable the REST/WebSocket server |
| `HALO_TASK_SERVER_HOST` | `127.0.0.1` | Bind address |
| `HALO_TASK_SERVER_PORT` | `8000` | Listen port |
| `HALO_EXIT_ON_EMPTY_QUEUE` | `0` | Exit when queue empties (`1`) or wait for new tasks (`0`) |

Endpoints:

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/locations` | List all named locations |
| `GET` | `/tasks` | List pending and completed tasks |
| `POST` | `/tasks` | Add a task — body: `{ "origin": "PHARMACY", "destination": "ICU", "priority": 1 }` |
| `GET` | `/ws` | WebSocket stream for real-time task updates |

## Configuration reference

Key parameters in `config.py` (all overridable via environment variables where noted):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MAP_SCALE` | `4.0` | Grid scale factor relative to the original layout |
| `CELL_SIZE` | `1.0 m` | Physical size of one grid cell |
| `BLOCK_DECAY_TRAVERSALS` | `15` | Cells traversed before a LIDAR block expires |
| `TENTATIVE_BLOCK_COST` | `25` | Cost assigned when a LIDAR block expires |
| `AUTO_DELAY_THRESHOLD_RATIO` | `1.2` | Forward-time ratio that counts as a delay in metrics |
| `TRIGGER_CELL` | `(27, 21)` | Cell that fires the dynamic map event |
| `PENALTY_TARGETS` | `[]` | Cells penalised by the dynamic event |
| `NEW_BLOCKED_CELLS` | `[]` | Cells blocked by the dynamic event |

Q-Learning parameters in `qlearning.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `QL_ALPHA` | `0.3` | Learning rate |
| `QL_GAMMA` | `0.0` | Discount factor (0 = EMA of immediate reward only) |
| `QL_COST_SCALE` | `5` | Maps Q-deviation to A\* cost range |
| `QL_MAX_COST` | `30` | Maximum cost Q-Learning can assign |
| `QL_INIT_VALUE` | `−1.0` | Initial Q-value (ideal immediate reward, `−1/(1−γ)`) |

## Repository structure

```text
MedirouteGrande/
├── worlds/
│   └── HOSPITAL-MEDIROUTE.wbt       # Webots world (hospital layout + Pioneer 3-DX)
├── controllers/
│   └── HALO/
│       ├── HALO.py                  # Entry point — initialisation and main task loop
│       ├── config.py                # Grid, locations, robot profile, all parameters
│       ├── navigation.py            # A*, cost-map operations, cell-state helpers
│       ├── tasks.py                 # Task execution, route following, Q-Learning update
│       ├── qlearning.py             # Q-table, Bellman update, cost conversion, summary
│       ├── motion.py                # Wheel control, odometry, LIDAR scan, GPS read
│       ├── metrics.py               # Per-task and per-cell metrics registration
│       ├── event_log.py             # Timestamped cost-map event logger
│       ├── task_queue.py            # Priority task queue with runtime add/complete
│       ├── task_server.py           # FastAPI REST + WebSocket server
│       ├── cost_map_events.txt      # Runtime-generated event log (block/unblock)
│       └── ui/
│           └── index.html           # Minimal web UI for task dispatch
├── analyze_world.py                 # Utility — generates grid from Webots world file
└── README.md
```

## Running the simulation

1. Open `worlds/HOSPITAL-MEDIROUTE.wbt` in Webots.
2. The controller starts automatically. The initial task queue is defined in `config.py` under `INITIAL_TASKS`.
3. If the task server is enabled (default), open `http://127.0.0.1:8000` in a browser to add tasks at runtime.
4. At the end of the simulation, the console prints:
   - per-task metric summary
   - per-cell metric summary
   - explicit cell-state table
   - Q-Learning summary: cells that learned significant costs, and an ASCII heat map of the final cost map
