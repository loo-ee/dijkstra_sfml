# 3D Planetary Rover Simulation (Dijkstra + Raylib)

A 3D physical terrain navigation and rover simulation engine powered by **Dijkstra's Algorithm**, built with **Raylib** and **Jolt Physics**.

The engine computes minimum-energy, slip-safe paths across treacherous Martian/Lunar 3D heightfields using physical cost functions (traction limits, slope incline angles, and gravity potential energy), and simulates a physically driven rover traversing the computed route.

---

## Documentation

Detailed architectural and feature planning documents are located in the [`docs/`](file:///Users/louie/Documents/GitHub/dijkstra_sfml/docs) directory:

1. [docs/architecture-plan.md](file:///Users/louie/Documents/GitHub/dijkstra_sfml/docs/architecture-plan.md): Full architectural overview, Raylib technology stack justification, and codebase evolution plan.
2. [docs/rover-terrain-simulation-plan.md](file:///Users/louie/Documents/GitHub/dijkstra_sfml/docs/rover-terrain-simulation-plan.md): Feature specification for the Planetary Rover on Terrain simulation (terrain heightfield, Coulomb friction limits, Pure Pursuit path follower, and telemetry HUD).
3. [docs/physics-simulation-use-cases.md](file:///Users/louie/Documents/GitHub/dijkstra_sfml/docs/physics-simulation-use-cases.md): Broad physics simulation use cases for graph algorithms (rovers, drones in turbulence, truss collapse, and hydraulic runoff).

---

## Dependencies & Installation (macOS)

### Prerequisites

1. Install Homebrew if you do not already have it.
2. Install **Raylib** (version 6.0+):

```bash
brew install raylib
```

*(Optional for legacy 2D visualizer)*:
```bash
brew install sfml@2
```

---

## Repository Layout

```
.
├── src/
│   ├── rover/                 # 3D Rover simulation source files
│   │   ├── rover_main.cpp     # Simulation entry point & orchestrator
│   │   ├── PlanetaryRover.cpp # Physical rover chassis, suspension & Pure Pursuit
│   │   ├── DijkstraSolver3D.cpp # Physics-cost-weighted 3D Dijkstra solver
│   │   ├── RoverNavGraph.cpp  # Topographic navigation graph generation
│   │   ├── PhysicsWorld.cpp   # Jolt Physics engine integration & rigid bodies
│   │   ├── TerrainHeightfield.cpp # Procedural heightfield generation & Perlin noise
│   │   ├── TerrainChunk.cpp   # Procedural infinite terrain chunks & physics
│   │   ├── ChunkManager.cpp   # Streaming chunk management
│   │   ├── RoverTelemetryHUD.cpp # 2D Mission Control telemetry HUD overlay
│   │   └── GraphRenderer3D.cpp # 3D graph, path, and search visualizer
│   └── visualizer2d/          # 2D Dijkstra visualizer source files (SFML / WebAssembly)
│       ├── main.cpp
│       ├── Button.cpp
│       ├── Graph.cpp
│       ├── GraphManager.cpp
│       └── Vertex.cpp
├── include/
│   ├── rover/                 # 3D Rover simulation headers
│   └── visualizer2d/          # 2D Dijkstra visualizer headers
├── external/                  # Third-party submodules (JoltPhysics)
├── scripts/                   # Auxiliary scripts (MLP training)
└── build/                     # Incremental build artifacts & executables
```

---

## Build Instructions

### 3D Rover Simulation (Raylib + Jolt Physics)

```bash
# Build 3D Rover binary (with fast incremental compilation)
make rover

# Build and execute 3D simulation
make run-rover
```

### 2D SFML Visualizer (Desktop & WebAssembly)

```bash
# Native desktop binary (SFML)
make desktop
make run-desktop

# WebAssembly bundle (Emscripten)
make wasm
```
