# Phase 2: Procedural Terrain Generation & Surface NavGraph

## 1. Goal & Objectives
Generate an undulating 3D Martian/Lunar landscape with procedural heightfields, compute surface gradients and slope angles, drape a 3D navigational graph over the terrain surface, and implement 3D raycast mouse picking.

---

## 2. Terrain Generation & Mathematics

### 2.1. Heightfield Function $y = h(x, z)$
The terrain elevation is computed using a combination of fractal Brownian motion (fBm) noise and crater displacement masks:

$$h(x, z) = \sum_{i=0}^{N-1} A \cdot \gamma^i \cdot \text{Noise}(f \cdot 2^i x, f \cdot 2^i z) - \text{CraterDisplacement}(x, z)$$

Where:
* **Base Resolution:** $128 \times 128$ grid spacing across a $200\text{ m} \times 200\text{ m}$ area.
* **Crater Function:**
  $$\text{CraterDisplacement}(x, z) = \begin{cases}
  -d \cdot \left(1 - \left(\frac{r}{R}\right)^2\right) + h_{\text{rim}} \cdot e^{-\frac{(r - R)^2}{2\sigma^2}} & \text{if } r \le 1.5R \\
  0 & \text{otherwise}
  \end{cases}$$
  (Creates a bowl with raised rim walls typical of Martian impact craters).

### 2.2. Normal Vectors & Slope Angles
At each vertex $(x, z)$, central finite differences calculate the gradient:

$$\frac{\partial h}{\partial x} \approx \frac{h(x+\Delta, z) - h(x-\Delta, z)}{2\Delta}, \quad \frac{\partial h}{\partial z} \approx \frac{h(x, z+\Delta) - h(x, z-\Delta)}{2\Delta}$$

$$\mathbf{n} = \frac{(-\partial h/\partial x, 1, -\partial h/\partial z)}{\|(-\partial h/\partial x, 1, -\partial h/\partial z)\|}$$

$$\text{Slope Angle } \theta = \arccos(\mathbf{n} \cdot \hat{\mathbf{j}}) = \arccos(n_y)$$

### 2.3. Terrain Visual Shading (Vertex Coloring)
* **Slope $< 15^\circ$:** Reddish Martian dust (`Color{195, 92, 60, 255}`).
* **Slope $15^\circ - 30^\circ$:** Dark exposed bedrock (`Color{110, 68, 55, 255}`).
* **Slope $> 30^\circ$ (Steep Cliff):** Charcoal basalt (`Color{60, 50, 48, 255}`).

---

## 3. Surface Graph Draping (`RoverNavGraph`)

Evolving [include/GraphManager.h](file:///Users/louie/Documents/GitHub/dijkstra_sfml/include/GraphManager.h) into `RoverNavGraph`:

```cpp
class RoverNavGraph {
public:
    void generateTerrainGrid(const TerrainHeightfield& terrain, int gridCols, int gridRows, float spacing);
    
    Vertex3D* getClosestNode(Vector3 worldPos);
    Vertex3D* pickNodeFromRay(Ray mouseRay, float pickRadius = 2.0f);

    void setStartNode(Vertex3D* node);
    void setEndNode(Vertex3D* node);

    const std::vector<Vertex3D*>& getVertices() const { return m_vertices; }

private:
    std::vector<Vertex3D*> m_vertices;
    Vertex3D* m_startNode = nullptr;
    Vertex3D* m_endNode = nullptr;
};
```

1. **Node Spacing:** Nodes are distributed at $(x_i, z_j)$ with height clamped to terrain:
   $$\text{position} = (x_i, h(x_i, z_j) + 0.3\text{ m}, z_j)$$
   *(Raised $0.3\text{ m}$ above the mesh to prevent z-fighting).*
2. **Neighbor Connectivity:** Each internal node connects to its 8-neighborhood (cardinals + diagonals).

---

## 4. 3D Raycast Mouse Picking

Using Raylib's `GetMouseRay()` to select Start and Goal nodes:

```cpp
if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
    Ray ray = GetMouseRay(GetMousePosition(), camera);
    for (Vertex3D* v : navGraph.getVertices()) {
        RayCollision col = GetRayCollisionSphere(ray, v->position, 1.5f);
        if (col.hit) {
            if (IsKeyDown(KEY_LEFT_SHIFT)) {
                navGraph.setEndNode(v);
            } else {
                navGraph.setStartNode(v);
            }
            break;
        }
    }
}
```

---

## 5. Acceptance Criteria & Verification

- [x] Procedural terrain mesh generates smoothly with mountains, craters, and plains.
- [x] Surface normals accurately color terrain based on slope angle $\theta$.
- [x] Graph nodes sit cleanly on the terrain contours without floating or sinking.
- [x] Left-click selects `Start` node (green sphere); Shift + Left-click selects `End` node (red sphere).
- [x] Framerate remains steady at $\ge 60\text{ FPS}$ with $500+$ draped nodes.
