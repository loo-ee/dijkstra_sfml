# Feature Plan: 3D Planetary Rover on Terrain Simulation (Raylib + Jolt)

## 1. Overview & Vision

The **Planetary Rover on Terrain** simulation transforms this project from a 2D abstract graph visualizer into a **3D Martian/Lunar exploration simulator** built with **Raylib** and **Jolt Physics**.

In real planetary exploration (e.g., NASA Curiosity, Perseverance), rovers cannot simply drive in straight lines. They must navigate treacherous slopes where loose sand causes wheel spin, avoid boulders that could high-center the chassis, and conserve battery by following elevation contours.

In this simulator:
1. **Procedural 3D Terrain** is generated with mountains, craters, slopes, and boulder fields using Raylib meshes.
2. A **3D Surface Graph** overlays the terrain, where Dijkstra calculates the **minimum-energy, slip-safe path**.
3. A **Physically Simulated Rover** (with chassis, multi-wheel suspension, and motor torque) physically drives the computed path in real time using a waypoint-following controller.
4. An **Interactive Telemetry Dashboard (rlImGui)** visualizes vehicle stability, battery drain, wheel slip, and step-by-step algorithm progression.

---

## 2. Physics & Mathematical Formulation

### 2.1. Terrain Geometry & Surface Gradients
* Terrain elevation is defined by a 2D scalar field $y = h(x, z)$ generated via multi-octave Perlin/Simplex noise superimposed with crater displacement functions (using Raylib's $Y$-up coordinate convention).
* For any node at $(x, z)$, the surface normal vector $\mathbf{n}$ is computed from the spatial derivatives:
  $$\mathbf{n} = \frac{\left(-\frac{\partial h}{\partial x}, 1, -\frac{\partial h}{\partial z}\right)}{\left\|\left(-\frac{\partial h}{\partial x}, 1, -\frac{\partial h}{\partial z}\right)\right\|}$$
* The slope angle $\theta$ relative to gravity (assuming gravity $\mathbf{g} = [0, -9.81, 0]$) is:
  $$\theta = \arccos(\mathbf{n} \cdot \mathbf{j}) \quad \text{where } \mathbf{j} = [0, 1, 0]^T$$

### 2.2. Traction, Slip & Rollover Limits
* **Coulomb Friction Constraint:** With static friction coefficient $\mu_s$, a rover slips on an incline if:
  $$\tan\theta > \mu_s$$
  Any edge exceeding this slope angle is strictly **impassable** ($\text{Cost} = \infty$).
* **Side-Slope (Bank Angle) Rollover Risk:** Traversing across a steep slope side-on shifts the rover's Center of Mass (CoM) outside the wheel track width $T$, risking a fatal rollover. If the roll angle $\phi_{\text{roll}} > \phi_{\text{critical}}$, the edge is disqualified.

### 2.3. The Physical Cost Function for Dijkstra
Given nodes $u = (x_u, y_u, z_u)$ and $v = (x_v, y_v, z_v)$:

$$\text{Cost}(u \to v) = d(u, v) \times \Big[ 1.0 + \alpha \cdot \max(0, \Delta y) + \beta \cdot (\tan\theta)^2 + \gamma \cdot (1.0 - \mu_{\text{surface}}) \Big] + \delta \cdot |\Delta \psi|$$

Where:
* $d(u, v) = \text{Vector3Distance}(u, v)$ (Raylib 3D Euclidean distance)
* $\Delta y = v.y - u.y$ (Gravity work penalty for climbing uphill)
* $\theta$ is the maximum slope angle along segment $(u, v)$
* $\mu_{\text{surface}}$ is the local ground traction (e.g., bedrock = 0.9, packed dirt = 0.6, loose sand = 0.25)
* $|\Delta \psi|$ is the steering heading delta from the preceding edge (penalizing sharp, energy-inefficient turns)
* $\alpha, \beta, \gamma, \delta$ are user-tunable weight multipliers in the UI

---

## 3. Module Architecture & Codebase Integration

```
+---------------------------------------------------------------------------------+
|                                 Application                                     |
|           (Main Loop, Simulation State Machine, rlImGui Telemetry UI)           |
+----------------------------------------+----------------------------------------+
|             Graphics Layer             |             Physics Layer              |
|                (Raylib)                |            (Jolt Physics)              |
|   - Terrain Heightfield Rendering      |   - HeightFieldShape / MeshShape       |
|   - 3D Node & Edge Visualizer          |   - Rover Rigid Bodies (Chassis+Wheels)|
|   - Camera3D (Orbit / Chase Modes)     |   - Suspension Constraints & Raycasts  |
+----------------------------------------+----------------------------------------+
|                               Navigation Core                                   |
|   - RoverNavGraph (Extends GraphManager: 3D node grid, snap to y = h(x,z))      |
|   - Graph.cpp (Dijkstra algorithm, DijkstraSnapshot step history)               |
|   - Pure Pursuit Path Following Controller (Waypoints -> Motor & Steering)      |
+---------------------------------------------------------------------------------+
```

### 3.1. Reusing Existing Components

1. **[Graph.h](file:///Users/louie/Documents/GitHub/dijkstra_sfml/include/Graph.h) and [Graph.cpp](file:///Users/louie/Documents/GitHub/dijkstra_sfml/Graph.cpp)**:
   * Retain the existing `DijkstraSnapshot` step recording and timeline scrubbing.
   * Update internal edge weight types from `int` to `float`.
   * The visual step-by-step inspection feature will highlight 3D nodes (`DrawSphere`) on the mountain as Dijkstra scans candidates.

2. **[GraphManager](file:///Users/louie/Documents/GitHub/dijkstra_sfml/include/GraphManager.h) $\to$ `RoverNavGraph`**:
   * Evolve `GraphManager` into a 3D terrain-aware graph manager.
   * Automatically generate nodes on a regular or hexagonal grid draped over the terrain heightfield:
     $$\text{Node}_i = (x_i, h(x_i, z_i), z_i)$$
   * Create bidirectional edges to 6 or 8 adjacent neighbors.
   * Run line-of-sight raycasts against physical boulder meshes to sever blocked edges.

3. **Rendering & Windowing ([main.cpp](file:///Users/louie/Documents/GitHub/dijkstra_sfml/main.cpp))**:
   * Replaced with Raylib's `InitWindow()`, `BeginMode3D(camera)`, `EndMode3D()`, and `rlImGui`.

---

## 4. Detailed Component Specifications

### 4.1. Procedural Terrain System (`TerrainManager`)
* **Mesh Resolution:** $128 \times 128$ or $256 \times 256$ vertex heightfield grid.
* **Rendering with Raylib:** Built using `GenMeshHeightmap` or dynamically generated `Mesh` uploaded via `UploadMesh()`.
* **Biomes / Surface Types:**
  * **Bedrock:** High friction ($\mu = 0.85$), dark slate gray, safe.
  * **Loose Sand Dunes:** Low friction ($\mu = 0.30$), red/orange, high slip risk.
  * **Craters:** Smooth bowl-shaped depressions with steep rim walls.
* **Physics Representation:** Jolt `HeightFieldShape` (memory-efficient collision geometry for large terrains).

### 4.2. Rover Vehicle Rig (`RoverVehicle`)
* **Chassis:** Box rigid body (mass: $\sim 800\text{ kg}$, center of mass positioned low).
* **Wheels:** 4 or 6 rigid cylinders/spheres connected via spring-damper suspension constraints:
  * Suspension rest length, spring stiffness ($k$), and damping coefficient ($c$).
* **Drive System:**
  * Independent wheel drive (torque applied directly to wheel axes).
  * Skid-steer (differential drive) or Ackermann steering.

### 4.3. Autonomous Path-Following Controller (`RoverController`)
* Implements the **Pure Pursuit Algorithm**:
  1. Identifies the current look-ahead waypoint along the Dijkstra path.
  2. Computes the curvature $\kappa = \frac{2 \sin(\alpha)}{L}$ to steer towards the target.
  3. Modulates motor throttle based on upcoming slope incline (downshifting on steep climbs, braking downhill).
  4. Detects wheel slip: if wheel angular velocity $\omega \cdot R \gg v_{\text{actual}}$, throttle is reduced to regain traction.

### 4.4. Telemetry & User Controls (`UI / rlImGui`)
* **Live Telemetry HUD:**
  * Current Speed ($\text{m/s}$), Pitch & Roll angle gauges (with rollover warning zone).
  * Total Battery Consumed ($\text{kJ} = \int \text{Power} \, dt$).
  * Traction Slip Percentage.
* **Simulation Controls:**
  * Start / Goal selector (Raylib mouse picking with `GetMouseRay()`).
  * Run / Step Dijkstra algorithm (using existing step slider).
  * "Deploy Rover" button to release the physical rover on the computed route.
  * Cost function tuning sliders ($\alpha, \beta, \gamma$).
* **Camera Modes:**
  * **Orbit / Satellite View:** `UpdateCamera(&camera, CAMERA_ORBITAL)`.
  * **Chase Cam:** `UpdateCamera(&camera, CAMERA_THIRD_PERSON)` following the rover rigid body.
  * **Rover Mast Cam:** First-person perspective from the rover's sensor mast.

---

## 5. Preset Scenarios

| Scenario Preset | Description | Algorithmic Challenge |
| :--- | :--- | :--- |
| **1. The Olympus Crater** | Start is at base; Goal is across a deep impact crater with sheer walls. | Direct line scales the cliff (exceeds $\mu$ slip threshold). Dijkstra finds the gentle saddle pass around the rim. |
| **2. Sand Dune Valley** | Short path traverses soft sand dunes; longer path stays on hard bedrock ridge. | Dijkstra weighs higher distance against low friction and rolling resistance, favoring the bedrock detour. |
| **3. Boulder Maze** | Dense distribution of physical rocks scattered on an incline. | Physics raycasts cut edges through rocks; rover navigates a slalom between obstacles without clipping corners. |

---

## 6. Implementation Milestones

### Milestone Checklist:
- [x] **Milestone 0:** Raylib 6.0 installed via Homebrew and linked.
- [ ] **Milestone 1:** Procedural heightfield mesh renders in Raylib 3D with slope-based color shading.
- [ ] **Milestone 2:** `RoverNavGraph` populates nodes snapped to the terrain surface; Dijkstra calculates shortest path avoiding impassable slopes.
- [ ] **Milestone 3:** Jolt Physics heightfield collision active; a test sphere rolls down terrain under gravity.
- [ ] **Milestone 4:** Full 4-wheeled rover model drives autonomously along the Dijkstra path using Pure Pursuit.
- [ ] **Milestone 5:** Live `rlImGui` telemetry panel displays battery, tilt, and slip with user-tunable physics weights.
