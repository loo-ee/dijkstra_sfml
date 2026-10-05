# Phase 4: Physics-Weighted Dijkstra Algorithm & Snapshot History

## 1. Goal & Objectives
Upgrade the core Dijkstra solver to compute optimal paths based on physical energy, work, traction, and slope stability, while preserving the interactive step-by-step snapshot playback from the legacy codebase.

---

## 2. Mathematical Cost Formulation

The edge weight function computes the real mechanical energy required to traverse from node $u$ to neighbor $v$:

```
Cost(u -> v) = PhysicalWork + SlopeSlipPenalty + TurnPenalty
```

### 2.1. Work & Gravity Component
* **Distance:** $d = \|\mathbf{p}_v - \mathbf{p}_u\|$
* **Elevation Delta:** $\Delta y = p_{v.y} - p_{u.y}$
* **Uphill Work:** Climbing uphill consumes battery energy directly proportional to gravitational potential:
  $$E_{\text{gravity}} = \alpha \cdot \max(0, \Delta y)$$
* **Downhill Braking:** Moderate downhill saves energy, but steep downhill requires regenerative braking:
  $$\text{if } \Delta y < 0: \quad E_{\text{gravity}} = -0.3 \cdot |\Delta y|$$

### 2.2. Traction & Coulomb Slip Penalty
Let $\theta$ be the maximum slope angle along segment $(u, v)$ and $\mu_s$ be the local surface friction coefficient:
* **Impassable Threshold:** If $\tan\theta > \mu_s$, the rover wheels will spin helplessly:
  $$\text{Cost}(u \to v) = \infty$$
* **Near-Slip Penalty:** For slopes approaching the slip limit:
  $$P_{\text{slip}} = \beta \cdot \left(\frac{\tan\theta}{\mu_s}\right)^2$$

### 2.3. Combined Cost Equation
$$\text{Cost}(u \to v) = d \cdot \left[ 1.0 + \alpha \cdot \max(0, \Delta y) + \beta \cdot \left(\frac{\tan\theta}{\mu_s}\right)^2 + \gamma \cdot (1.0 - \mu_s) \right] + \delta \cdot (1 - \cos\Delta\psi)$$

Where:
* $\Delta\psi$ is the heading angular change between the previous incoming edge and the new outgoing edge (discourages erratic zig-zag paths).
* $\alpha, \beta, \gamma, \delta$ are interactive weight multipliers adjustable in real time.

---

## 3. Algorithm Replay & Snapshot History

Reusing the battle-tested snapshot architecture in [include/Graph.h](file:///Users/louie/Documents/GitHub/dijkstra_sfml/include/Graph.h) and [Graph.cpp](file:///Users/louie/Documents/GitHub/dijkstra_sfml/Graph.cpp):

```cpp
struct DijkstraSnapshot3D {
    std::string currentNode;
    std::string examiningNeighbor;
    std::string message;

    std::unordered_map<std::string, float> distances;
    std::unordered_map<std::string, std::string> parents;
    std::vector<std::string> visitedNodes;
    std::vector<std::pair<std::string, float>> priorityQueue;

    bool isFinished = false;
    bool pathFound = false;
};
```

### Visual State Feedback in 3D:
* **Current Node ($u$):** Pulsing golden sphere.
* **Examining Neighbor ($v$):** Cyan line connecting $u \to v$ with floating candidate weight cost.
* **Visited Nodes:** Desaturated blue spheres.
* **Final Shortest Path:** Brilliant neon emerald green tube with moving directional arrow pulses.

---

## 4. Algorithmic Test Scenarios

### Scenario A: Flat Detour vs Steep Climb
* **Direct route:** Climbs straight over a $35^\circ$ rocky peak ($d = 50\text{ m}$, cost includes extreme $\Delta y$ and slip).
* **Detour route:** Circles around the base along an elevation contour ($d = 85\text{ m}$, minimal $\Delta y$).
* **Verification:** With $\alpha \ge 2.0$, Dijkstra automatically picks the longer, flatter detour route.

### Scenario B: Sand Basin Avoidance
* Soft sand dunes with low friction ($\mu = 0.25$) placed between Start and Goal.
* **Verification:** Dijkstra diverts across harder bedrock patches to avoid the loose sand hazard.

---

## 5. Acceptance Criteria & Verification

- [ ] Dijkstra evaluates costs using floating-point physical weights.
- [ ] Edges exceeding the friction slip angle ($\tan\theta > \mu_s$) are strictly excluded.
- [ ] Algorithm execution can be scrubbed forward/backward frame-by-frame.
- [ ] Shortest path visualizer renders a glowing green path along the terrain surface.
- [ ] Dijkstra solves a 500-node 3D terrain graph in $< 5\text{ milliseconds}$.
