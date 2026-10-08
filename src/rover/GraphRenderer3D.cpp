#include "GraphRenderer3D.h"
#include "DijkstraSolver3D.h"
#include <cmath>
#include <unordered_map>

Color GraphRenderer3D::getNodeColor(NodeState state) {
    switch (state) {
        case NodeState::DEFAULT:
            return Color{ 40, 215, 255, 255 };  // Electric luminous cyan (high contrast against Martian terrain)
        case NodeState::START:
            return Color{ 46, 230, 113, 255 };  // Brilliant emerald green
        case NodeState::END:
            return Color{ 235, 60, 60, 255 };   // Vibrant laser red
        case NodeState::CURRENT:
            return Color{ 255, 210, 40, 255 };  // Radiant sunfire gold / amber
        case NodeState::VISITED:
            return Color{ 60, 160, 245, 255 };  // Luminous sapphire / sky blue
        case NodeState::PATH:
            return Color{ 46, 230, 113, 255 };  // Radiant emerald jewel
        case NodeState::IMPASSABLE:
            return Color{ 180, 50, 50, 200 };   // Dark crimson hazard
        default:
            return Color{ 40, 215, 255, 255 };
    }
}

void GraphRenderer3D::drawNode(const Vertex3D& vertex, float radius) {
    Color color = getNodeColor(vertex.state);
    float effectiveRadius = radius;
    
    // Highlight CURRENT state with dynamic sinusoidal pulsation
    if (vertex.state == NodeState::CURRENT) {
        effectiveRadius += sinf(static_cast<float>(GetTime()) * 8.0f) * 0.25f * radius;
    }
    
    DrawSphere(vertex.position, effectiveRadius, color);
    DrawSphereWires(vertex.position, effectiveRadius * 1.25f, 6, 6, ColorAlpha(color, 0.60f));
    DrawLine3D(vertex.position, Vector3{ vertex.position.x, vertex.position.y - 0.5f, vertex.position.z }, ColorAlpha(color, 0.75f));
}

void GraphRenderer3D::drawEdge(const Vector3& start, const Vector3& end, float radius, Color color) {
    if (Vector3DistanceSqr(start, end) < 0.0001f) {
        return;
    }
    DrawCylinderEx(start, end, radius, radius, 8, color);
}

void GraphRenderer3D::drawArrowHead(const Vector3& from, const Vector3& to, Color color) {
    Vector3 dir = Vector3Subtract(to, from);
    float length = Vector3Length(dir);
    if (length < 0.001f) {
        return;
    }

    Vector3 normDir = Vector3Scale(dir, 1.0f / length);
    float coneHeight = 1.2f;
    float coneRadius = 0.5f;
    if (coneHeight > length * 0.5f) {
        coneHeight = length * 0.5f;
        coneRadius = coneHeight * 0.4f;
    }
    
    Vector3 coneBase = Vector3Subtract(to, Vector3Scale(normDir, coneHeight));
    DrawCylinderEx(coneBase, to, coneRadius, 0.0f, 8, color);
}

void GraphRenderer3D::drawShortestPath(const std::vector<const Vertex3D*>& path, float time) {
    if (path.size() < 2) return;

    // Slight elevation offset to avoid z-fighting with draped terrain and edges
    Vector3 liftOffset = { 0.0f, 0.45f, 0.0f };

    // 1. Draw glowing emerald conduit along all segments (low-poly 6-sided and 4-sided cylinders)
    for (size_t i = 0; i < path.size() - 1; ++i) {
        Vector3 p1 = Vector3Add(path[i]->position, liftOffset);
        Vector3 p2 = Vector3Add(path[i + 1]->position, liftOffset);

        // Outer Glowing Emerald Conduit
        DrawCylinderEx(p1, p2, 0.32f, 0.32f, 6, Color{ 46, 204, 113, 220 });

        // High-Luminance Core Wire
        DrawCylinderEx(p1, p2, 0.14f, 0.14f, 4, Color{ 220, 255, 230, 255 });
    }

    // 2. Single high-speed energy pulse bead travelling along the shortest path
    float totalProgress = fmodf(time * 3.5f, static_cast<float>(path.size() - 1));
    int segIdx = static_cast<int>(totalProgress);
    float segT = totalProgress - segIdx;
    if (segIdx < static_cast<int>(path.size() - 1)) {
        Vector3 bP1 = Vector3Add(path[segIdx]->position, liftOffset);
        Vector3 bP2 = Vector3Add(path[segIdx + 1]->position, liftOffset);
        Vector3 pulsePos = Vector3Lerp(bP1, bP2, segT);
        DrawSphere(pulsePos, 0.45f, WHITE);
        DrawSphereWires(pulsePos, 0.65f, 4, 4, Color{ 46, 230, 113, 255 });
    }

    // 3. Destination beacon
    Vector3 pEnd = Vector3Add(path.back()->position, liftOffset);
    DrawSphere(pEnd, 0.65f, Color{ 46, 204, 113, 255 });
    DrawSphereWires(pEnd, 0.85f, 6, 6, WHITE);
}

void GraphRenderer3D::drawSearchState(const DijkstraSnapshot3D& snapshot, 
                                      const std::vector<Vertex3D*>& allVertices, 
                                      float time) {
    // Only render search state during active step-by-step exploration
    if (snapshot.currentNode.empty() && snapshot.examiningNeighbor.empty()) return;

    static std::unordered_map<std::string, const Vertex3D*> s_vMap;
    static size_t s_lastVertexCount = 0;
    if (s_lastVertexCount != allVertices.size()) {
        s_vMap.clear();
        for (const Vertex3D* v : allVertices) {
            if (v) s_vMap[v->name] = v;
        }
        s_lastVertexCount = allVertices.size();
    }

    Vector3 lift = { 0.0f, 0.35f, 0.0f };

    // 1. Draw Settled / Visited Nodes (Desaturated Cyan/Blue)
    for (const auto& nodeName : snapshot.visitedNodes) {
        auto it = s_vMap.find(nodeName);
        if (it != s_vMap.end()) {
            Vector3 pos = Vector3Add(it->second->position, lift);
            DrawSphere(pos, 0.35f, Color{ 52, 152, 219, 210 });
        }
    }

    // 2. Highlight Current Active Node u (Pulsing Golden Sphere)
    if (!snapshot.currentNode.empty()) {
        auto it = s_vMap.find(snapshot.currentNode);
        if (it != s_vMap.end()) {
            Vector3 uPos = Vector3Add(it->second->position, lift);
            float pulseR = 0.90f + sinf(time * 8.0f) * 0.18f;
            DrawSphere(uPos, pulseR, Color{ 241, 196, 15, 255 });
            DrawSphereWires(uPos, pulseR * 1.25f, 6, 6, ColorAlpha(WHITE, 0.85f));

            // Expanding ring on ground
            float groundRingR = 1.8f + fmodf(time * 3.0f, 1.2f);
            DrawCircle3D(it->second->position, groundRingR, Vector3{ 0, 1, 0 }, 90.0f, ColorAlpha(GOLD, 0.75f));

            // 3. Highlight Neighbor v Being Examined (Vibrant Cyan Beam)
            if (!snapshot.examiningNeighbor.empty()) {
                auto vIt = s_vMap.find(snapshot.examiningNeighbor);
                if (vIt != s_vMap.end()) {
                    Vector3 vPos = Vector3Add(vIt->second->position, lift);
                    DrawCylinderEx(uPos, vPos, 0.25f, 0.25f, 6, Color{ 0, 240, 255, 255 });
                    DrawSphere(vPos, 0.65f, Color{ 0, 220, 255, 255 });
                }
            }
        }
    }
}
