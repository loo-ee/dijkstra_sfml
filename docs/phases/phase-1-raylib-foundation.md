# Phase 1: Raylib 3D Foundation & Math Decoupling

## 1. Goal & Objectives
Establish the core 3D application architecture using Raylib, decouple graph data structures from legacy SFML 2D renderables, implement 3D orbital camera navigation, and render interactive 3D nodes and edges.

---

## 2. Code Refactoring Specifications

### 2.1. Refactoring [include/Vertex.h](file:///Users/louie/Documents/GitHub/dijkstra_sfml/include/Vertex.h) $\to$ `include/Vertex3D.h`
Remove all SFML dependencies (`sf::CircleShape`, `sf::Vector2f`). Replace with Raylib's native 3D vector type:

```cpp
#pragma once
#include <raylib.h>
#include <string>
#include <vector>
#include <utility>

enum class NodeState {
    DEFAULT,
    START,
    END,
    CURRENT,
    VISITED,
    PATH,
    IMPASSABLE
};

struct Vertex3D {
    std::string name;
    Vector3 position;                                       // 3D coordinates (X, Y, Z)
    std::vector<std::pair<std::string, float>> neighbors;   // (TargetNodeName, PhysicalCost)
    
    NodeState state = NodeState::DEFAULT;
    float minDistanceFromSrc = 1e9f;
    Vertex3D* parent = nullptr;

    // Physical attributes
    float surfaceFriction = 0.6f;
    float slopeAngleRad = 0.0f;
    bool isWalkable = true;

    Vertex3D(const std::string& nodeName, Vector3 pos)
        : name(nodeName), position(pos) {}
};
```

### 2.2. Dedicated 3D Graph Renderer (`GraphRenderer3D`)
Decouple visual drawing completely from data structures:

```cpp
class GraphRenderer3D {
public:
    static void drawNode(const Vertex3D& vertex, float radius = 1.0f);
    static void drawEdge(const Vector3& start, const Vector3& end, float radius, Color color);
    static void drawArrowHead(const Vector3& from, const Vector3& to, Color color);
    static Color getNodeColor(NodeState state);
};
```

* **Node Rendering:** `DrawSphere(vertex.position, radius, color)`
* **Edge Rendering:** `DrawCylinderEx(start, end, radius, radius, 8, color)`
* **Highlighting:** Nodes in `CURRENT` state pulsate slightly using `sinf(GetTime() * 8.0f) * 0.2f`.

---

## 3. 3D Camera & Window Initialization

### 3.1. Orbit Camera Controls
Using Raylib's built-in `Camera3D`:
* **Rotation:** Mouse right-click drag rotates pitch and yaw around target center $(0, 0, 0)$.
* **Zoom:** Mouse wheel zooms in/out (adjusting distance along camera forward vector).
* **Pan:** Mouse middle-click drag shifts camera target in the screen plane.

```cpp
Camera3D camera = { 0 };
camera.position = (Vector3){ 0.0f, 30.0f, 40.0f };
camera.target   = (Vector3){ 0.0f, 0.0f, 0.0f };
camera.up       = (Vector3){ 0.0f, 1.0f, 0.0f };
camera.fovy     = 45.0f;
camera.projection = CAMERA_PERSPECTIVE;

// Main loop:
UpdateCamera(&camera, CAMERA_ORBITAL);
```

---

## 4. Test Scenario: 3D Lattice / Cube Graph

Implement a test scene that spawns a $3 \times 3 \times 3$ grid of 3D nodes interconnected by bidirectional edges to verify:
1. Smooth 60 FPS rendering under camera rotation.
2. Correct depth buffering (`BeginMode3D` enables hardware depth testing automatically).
3. Visual distinction between `START`, `END`, and `DEFAULT` states.

---

## 5. Acceptance Criteria & Verification

- [x] `include/Vertex3D.h` compiles with zero SFML header dependencies.
- [x] Window initializes at $1280 \times 720$ with Raylib 6.0.
- [x] User can orbit, pan, and zoom camera smoothly around a 3D graph.
- [x] Nodes render as 3D colored spheres and edges render as continuous 3D cylinders.
- [x] `make check-raylib` passes cleanly.
