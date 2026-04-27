# MediRoute

**MediRoute** is an adaptive autonomous mobile robot designed to optimize internal hospital logistics in simulation. The system transports medication and laboratory samples between areas such as **Pharmacy**, **ICU**, **Laboratory**, and **Ward**, using a **2D grid-based hospital model** and the **A\* algorithm** for route planning. Unlike static navigation approaches, MediRoute supports **dynamic replanning**, **cost-map adaptation**, and **task queue management** in a changing indoor environment. :contentReference[oaicite:0]{index=0} :contentReference[oaicite:1]{index=1}

## Project goals

The main objective of MediRoute is to improve hospital internal logistics by reducing transport times, improving delivery efficiency, and decreasing the non-clinical workload of healthcare staff. The project focuses on simulation-based validation of adaptive routing behavior in hospital-like environments. :contentReference[oaicite:2]{index=2}

## Main features

MediRoute currently includes:

- **Task queue management** for origin-destination delivery requests
- **Hospital representation as a 2D grid**
- **A\* route planning** on a weighted cost map
- **Cell-by-cell route execution**
- **Dynamic replanning** when congestion or new blockages appear
- **Adaptive cost-map updates**
- **Explicit cell states**: `FREE`, `CONGESTED`, `BLOCKED`
- **Travel-time monitoring**
- **Performance metrics** per task and per cell :contentReference[oaicite:3]{index=3} :contentReference[oaicite:4]{index=4} :contentReference[oaicite:5]{index=5}
- **Real-time task dispatch** via REST + WebSocket (optional UI)

## Architecture overview

The system is implemented entirely in **Webots**, which acts as the simulation platform for the robot, the hospital world, and the execution of experiments. The architecture follows the conceptual structure described in the project documentation:

1. **Simulation Environment Layer**  
   Webots world with robot, corridors, rooms, and obstacles.

2. **Environment Representation Layer**  
   Logical 2D grid abstraction of the hospital.

3. **Navigation and Planning Layer**  
   A\* path planning over the grid and cost map.

4. **Monitoring and Adaptation Layer**  
   Travel-time observation, congestion handling, cost updates, and dynamic replanning.

5. **Evaluation Layer**  
   Performance metrics and execution summaries. :contentReference[oaicite:6]{index=6}

## Technological baseline and prior components

MediRoute is built on top of existing simulation components provided by **Webots**, rather than starting from a completely empty robotics stack. In particular, the project uses the **e-puck robot model** available in Webots as the base mobile platform for simulation and controller development.

This means that certain low-level capabilities were already available as part of the simulation environment, such as:

- robot body and wheel configuration
- wheel motors and wheel sensors
- simulation timing and controller execution
- basic robot/world integration inside Webots

The main contribution of MediRoute is therefore not the creation of a robot model from scratch, but the design and implementation of the **adaptive hospital logistics behavior** on top of that simulation baseline. This includes:

- hospital representation as a 2D grid
- A* route planning
- task queue management
- dynamic congestion handling
- dynamic blockage handling
- route replanning during execution
- explicit cell-state management
- performance monitoring and metrics

In this sense, Webots and the e-puck model provide the technological starting point, while MediRoute adds the project-specific logic required for adaptive hospital logistics.

## Current implementation

The current version of the controller supports:

- sequential execution of hospital delivery tasks
- weighted path planning with A\*
- dynamic congestion penalties
- dynamic cell blocking during execution
- route recalculation from the robot’s current cell
- metrics by task:
  - travel time
  - traversed cells
  - number of replans
  - accumulated route cost
- metrics by cell:
  - visits
  - rotation time
  - forward time
  - total time
  - delay count
  - automatic penalties
- explicit cell-state tracking:
  - `FREE`
  - `CONGESTED`
  - `BLOCKED`

## Real-time task server (REST + WebSocket)

The controller can host a lightweight task server that lets you push new
origin/destination jobs while the simulation runs. A minimal web UI is
served at the same address.

Environment variables:

- `HALO_TASK_SERVER_ENABLED` (default `1`)
- `HALO_TASK_SERVER_HOST` (default `127.0.0.1`)
- `HALO_TASK_SERVER_PORT` (default `8000`)
- `HALO_EXIT_ON_EMPTY_QUEUE` (default `0` to keep running)

Endpoints:

- `GET /locations`
- `GET /tasks`
- `POST /tasks` (payload: `{ origin, destination, priority }`)
- `GET /ws` (WebSocket stream for updates)

## Repository structure

```text
MEDIROUTE/
├── worlds/
│   └── HOSPITAL-MEDIROUTE.wbt
├── controllers/
│   └── HALO/
│       ├── HALO.py
│       ├── config.py
│       ├── motion.py
│       ├── navigation.py
│       ├── metrics.py
│       └── tasks.py
├── README.md
└── .gitattributes
