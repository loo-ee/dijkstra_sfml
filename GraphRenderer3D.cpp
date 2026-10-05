#include "GraphRenderer3D.h"
#include <cmath>

Color GraphRenderer3D::getNodeColor(NodeState state) {
    switch (state) {
        case NodeState::DEFAULT:
            return Color{ 180, 195, 210, 255 }; // Clean metallic silver-blue
        case NodeState::START:
            return Color{ 46, 204, 113, 255 };  // Emerald green
        case NodeState::END:
            return Color{ 231, 76, 60, 255 };   // Vibrant red
        case NodeState::CURRENT:
            return Color{ 241, 196, 15, 255 };  // Bright gold / amber
        case NodeState::VISITED:
            return Color{ 52, 152, 219, 255 };  // Sky blue
        case NodeState::PATH:
            return Color{ 155, 89, 182, 255 };  // Vivid violet
        case NodeState::IMPASSABLE:
            return Color{ 60, 60, 65, 255 };    // Dark basalt / charcoal
        default:
            return LIGHTGRAY;
    }
}

void GraphRenderer3D::drawNode(const Vertex3D& vertex, float radius) {
    Color color = getNodeColor(vertex.state);
    float effectiveRadius = radius;
    
    // Highlight CURRENT state with dynamic sinusoidal pulsation
    if (vertex.state == NodeState::CURRENT) {
        effectiveRadius += sinf(static_cast<float>(GetTime()) * 8.0f) * 0.2f * radius;
    }
    
    DrawSphere(vertex.position, effectiveRadius, color);
    DrawSphereWires(vertex.position, effectiveRadius, 8, 8, ColorAlpha(BLACK, 0.25f));
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
