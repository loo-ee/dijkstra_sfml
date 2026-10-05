# Machine Learning for Terrain Traversability & Autonomous Rover Navigation

This document specifies the technical design, feasibility study, model selection analysis, integration architecture, and educational study roadmap for incorporating Machine Learning (ML) into the **3D Physics-Integrated Dijkstra Simulator**.

---

## 1. Executive Summary & Feasibility Analysis

### Project Context
The simulator currently combines:
* **Raylib 6.0** for 3D immediate-mode rendering, camera controls, and high-DPI overlays.
* **Jolt Physics (v5.0.2)** for rigid-body chassis dynamics, 4-wheel raycast spring-damper suspension, continuous collision detection, and procedural heightfield collision.
* **Physics-Weighted 3D Dijkstra Solver** operating on a draped 26×26 (676 nodes, ~2,500 edges) navigation graph.

### Feasibility on Apple M2 Silicon & C++ Stack
* **Hardware Profile:** Apple M2 has 8 CPU cores (4 performance, 4 efficiency), a 10-core Metal GPU, and a 16-core Apple Neural Engine (ANE) with 15.8 TOPS.
* **Frame Budget at 60 FPS:** Total budget is $16.66\,\text{ms}$.
  * Jolt Physics step: $\sim 1.2 - 2.0\,\text{ms}$
  * Raylib rendering & HUD: $\sim 2.0 - 3.5\,\text{ms}$
  * Dijkstra graph solve (instant): $\sim 0.3 - 1.2\,\text{ms}$
  * **Headroom available for ML inference:** $\sim 5 - 8\,\text{ms}$ (comfortably within real-time limits).
* **Verdict:** Highly feasible. A lightweight physics-informed neural network or patch-based CNN can execute in $< 0.5\,\text{ms}$ on M2 CPU or Neural Engine without frame drops.

---

## 2. Model Selection & Comparative Analysis

We evaluate four model paradigms ranging from feature-level regressors to vision segmentation and reinforcement learning:

```
+-----------------------------------------------------------------------------------+
|                            Input Data Modalities                                  |
|   [A: Jolt Physics Features]      [B: 2.5D Elevation Grid]      [C: Camera RGB/D] |
+-------------------------------+------------------------------+--------------------+
                                |                              |
                                v                              v
                     +----------------------+       +----------------------+
                     |  Physics-Informed    |       |  Patch CNN / Vision  |
                     |  MLP / Random Forest |       |  (MobileNet / FLINT) |
                     +----------+-----------+       +----------+-----------+
                                |                              |
                                +--------------+---------------+
                                               |
                                               v
                               +-------------------------------+
                               | Predicted Traversability Cost |
                               |   C(u -> v) for Dijkstra      |
                               +---------------+---------------+
                                               |
                                               v
                               +-------------------------------+
                               | 3D Dijkstra Pathfinding Core  |
                               +---------------+---------------+
                                               |
                                               v
                               +-------------------------------+
                               | Rover Pure Pursuit / RL Actor |
                               +-------------------------------+
```

### Comprehensive Comparison Matrix

| Architecture | Model Class | Input Data | Latency (M2) | Memory Size | C++ Integration | WASM Viability | Recommendation |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Option 1: Physics-Informed Geometry MLP** | 3-layer MLP (`8 -> 32 -> 16 -> 1`) | Geometric/physics features per edge | **$< 0.05\,\mu\text{s}$ / edge** ($< 0.15\,\text{ms}$ whole graph) | **$< 50\,\text{KB}$** | **Zero external deps** (pure C++ struct) | **100% Native & WASM** | **Top Recommendation (Phase 6A)** |
| **Option 2: 2.5D Elevation Patch CNN** | Tiny-CNN / FLINT / MobileNetV3-Small | $16 \times 16$ local height patch | $\sim 0.8 - 1.8\,\text{ms}$ / batch | $\sim 2 - 8\,\text{MB}$ | ONNX Runtime C++ or LibTorch | Moderate (ONNX Runtime Web) | **Strong Future Extension (Phase 6B)** |
| **Option 3: Vision Semantic Segmentation** | Fast-SCNN / YOLOv11n-Seg | Raylib rendered depth/RGB buffer | $\sim 8 - 15\,\text{ms}$ / frame | $\sim 25 - 80\,\text{MB}$ | High (RenderTexture capture + ONNX) | Low (Heavy overhead) | Overkill for heightfields |
| **Option 4: Deep Reinforcement Learning (PPO)** | Actor-Critic MLP Policy | Rover state + next 5 waypoints | $\sim 0.02\,\text{ms}$ / step | $\sim 200\,\text{KB}$ | C++ matrix multiply / ONNX | High | Excellent for local vehicle control |

---

## 3. Deep Dive: Option 1 — Physics-Informed Geometry MLP

### Why This Is the Best Starting Model
1. **Mathematical Consistency:** The simulator already calculates physical metrics per node and edge in `RoverNavGraph.cpp` and `DijkstraSolver3D.cpp`.
2. **Direct Integration:** Instead of hand-tuning arbitrary coefficients ($\alpha, \beta, \gamma, \delta$), the model predicts actual physical risk and energy expenditure based on real simulation trials.
3. **Zero Runtime Friction:** Can be embedded directly as a header-only forward pass with no third-party library installations required.

### Mathematical Formulation
The input feature vector $\mathbf{x} \in \mathbb{R}^8$ for edge $(u \to v)$ is defined as:
$$\mathbf{x} = \begin{bmatrix}
\Delta y & \text{(Elevation gain/loss)} \\
\theta_{\text{segment}} & \text{(Segment inclination angle)} \\
\theta_{\max} & \text{(Max surface slope of nodes } u, v\text{)} \\
\mu_s & \text{(Average friction coefficient)} \\
\Delta \psi & \text{(Heading deviation from parent vector)} \\
h_{\text{roughness}} & \text{(Standard deviation of height samples along edge)} \\
g & \text{(Current planetary gravity: } 3.71, 1.62, 9.81\text{)} \\
d_{\text{euc}} & \text{(3D Euclidean distance)}
\end{bmatrix}$$

The network outputs:
$$\hat{C}(u, v) = d_{\text{euc}} \cdot \left(1.0 + \text{Softplus}\left(W_3 \cdot \text{ReLU}(W_2 \cdot \text{ReLU}(W_1 \mathbf{x} + \mathbf{b}_1) + \mathbf{b}_2) + b_3\right)\right)$$

If $\hat{C}(u, v)$ exceeds a critical safety threshold $C_{\text{thresh}}$, the edge is marked **impassable** ($\infty$).

---

## 4. End-to-End Implementation Architecture

### 4.1. Simulation-to-Data Pipeline (Self-Supervised Generation)
The project itself acts as the dataset generator:
1. **Automated Probe Runs:** A headless runner drives the rover across randomly generated start/goal pairs on all 4 terrain presets (`OLYMPUS_CRATER`, `SCREE_SLOPE`, etc.).
2. **Telemetry Logging:** At each traversed edge $(u, v)$, the simulation logs:
   * Input features $\mathbf{x}$.
   * True measured energy $\Delta E = \int \tau_{\text{motor}} \cdot \omega_{\text{wheel}} \, dt$ (Joules).
   * True measured slip ratio $s = \frac{|\omega r - v_{\text{actual}}|}{\max(\omega r, v_{\text{actual}})}$.
   * Peak chassis tilt angle $\phi_{\text{roll}}, \theta_{\text{pitch}}$.
   * Ground-truth success boolean (reached waypoint without rollover or timeout).
3. **Loss Function:**
   $$\mathcal{L} = \text{MSE}(\hat{C}, C_{\text{ground\_truth}}) + \lambda_{\text{safety}} \cdot \text{BCE}(\hat{P}_{\text{passable}}, y_{\text{safe}})$$

### 4.2. C++ Native Deployment Structure

```cpp
// include/TerrainTraversabilityMLP.h
#pragma once
#include <array>
#include <cmath>

class TerrainTraversabilityMLP {
public:
    static constexpr int INPUT_DIM = 8;
    static constexpr int HIDDEN_1 = 32;
    static constexpr int HIDDEN_2 = 16;

    // Evaluates edge traversability cost in < 50 nanoseconds
    static float predictCost(const std::array<float, INPUT_DIM>& features);
    static bool isTraversable(const std::array<float, INPUT_DIM>& features, float threshold = 50.0f);

private:
    static const float W1[HIDDEN_1][INPUT_DIM];
    static const float B1[HIDDEN_1];
    static const float W2[HIDDEN_2][HIDDEN_1];
    static const float B2[HIDDEN_2];
    static const float W3[1][HIDDEN_2];
    static const float B3[1];
};
```

---

## 5. Areas for Further Study & Skill Development

To master terrain-aware robotics, machine learning for navigation, and real-time physical simulation, focus on the following foundational areas:

### 1. Terramechanics & Planetary Rover Dynamics
* **Core Concepts:**
  * **Bekker Theory:** Soil pressure-sinkage relationships ($p = (k_c/b + k_\phi) z^n$).
  * **Coulomb-Mohr Soil Failure:** Maximum shear strength of loose regolith $\tau = c + \sigma \tan\phi$.
  * **Slip-Sinkage Phenomenon:** How spinning rover wheels dig into Martian sand dunes (e.g., the *Spirit* rover entrapment).
* **Key Literature:**
  * Wong, J. Y., *Theory of Ground Vehicles* (The seminal textbook on ground vehicle dynamics).
  * Iagnemma, K. & Dubowsky, S., *Mobile Robots in Rough Terrain: Estimation, Motion Planning, and Control with Application to Planetary Rovers* (Springer Tracts in Advanced Robotics).

### 2. Learned Heuristics & Search Algorithms
* **Core Concepts:**
  * **Neural A\* & Neural Dijkstra:** Using neural networks to learn admissible heuristics or edge cost reweighting without losing algorithmic optimality guarantees.
  * **Lifelong Planning A\* (LPA\*) & D\* Lite:** Incremental graph search for dynamically updating cost maps as the rover discovers unexpected obstacles.
  * **Graph Neural Networks (GNNs) for Navigation:** Passing node/edge embeddings across the 3D terrain mesh to capture non-local topographical flow.
* **Key Literature:**
  * Yonetani, R. et al., *Path Planning with Neural A\* Search* (ICLR 2021).
  * Choset, H. et al., *Principles of Robot Motion: Theory, Algorithms, and Implementations* (MIT Press).

### 3. Edge ML Inference & High-Performance C++
* **Core Concepts:**
  * **ONNX Runtime Architecture:** Execution Providers (CoreML for Apple Neural Engine, CPU fallback, WebAssembly execution).
  * **Model Quantization (FP32 $\to$ INT8/FP16):** Reducing memory footprint and accelerating SIMD vector math on ARM NEON.
  * **Zero-Allocation C++ Inference:** Designing inference passes that avoid dynamic heap allocations (`malloc`/`new`) inside real-time 60 FPS simulation loops.
* **Hands-on Tools:**
  * `onnxruntime` C++ API.
  * Apple Metal Performance Shaders (MPS) and CoreML Tools (`coremltools`).
  * SIMD vector intrinsics on ARM (`arm_neon.h`).

### 4. Deep Reinforcement Learning for Vehicle Motion
* **Core Concepts:**
  * **PPO (Proximal Policy Optimization):** Stable policy-gradient algorithms for continuous motor torque and steering control.
  * **Sim-to-Real Transfer & Domain Randomization:** Varying gravity, friction, wheel damping, and boulder geometry so trained policies remain robust to environment changes.
  * **Curriculum Learning:** Starting with flat terrain and progressively increasing slope inclination and obstacle density.
* **Recommended Frameworks:**
  * Stable-Baselines3 (PyTorch).
  * Isaac Gym / Isaac Lab or Jolt-based custom training environments.

---

## 6. Implementation Roadmap: Phase 6

| Milestone | Deliverable | Description |
| :--- | :--- | :--- |
| **6.1 Data Logging** | `DataLogger.cpp` | Instrument the simulation loop to record $(u \to v)$ transitions, wheel slip, roll angle, and energy consumption into `.csv` / `.parquet`. |
| **6.2 Offline Training** | `train_traversability.py` | PyTorch training script with feature scaling, MLP training, and loss evaluation against Jolt physics ground truth. |
| **6.3 Model Export** | `weights.h` / `model.onnx` | Export weights as a lightweight C++ header array and ONNX binary with CoreML EP support. |
| **6.4 Dijkstra Integration** | Preset 5 in `DijkstraSolver3D.cpp` | Add a new cost preset: `"Neural Network Traversability"`, comparing ML routes vs classical Euclidean/physics routes. |
| **6.5 Telemetry Visualization** | HUD ML Confidence Card | Add an ML confidence gauge, predicted slip risk bar, and inference latency timer to `RoverTelemetryHUD.cpp`. |
