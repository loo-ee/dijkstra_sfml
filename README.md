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

## Build Instructions

### Native Desktop (Raylib)

Compile with the Homebrew Raylib include and library paths:

```bash
g++ -std=c++17 -Wall \
    -I"$(brew --prefix raylib)/include" \
    -L"$(brew --prefix raylib)/lib" \
    -lraylib -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo \
    -o build/rover-simulator <sources>
```

### Legacy 2D SFML Visualizer

```bash
make desktop
make run-desktop
```
