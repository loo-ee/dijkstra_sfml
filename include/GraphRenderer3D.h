#pragma once
#include <raylib.h>
#include <raymath.h>
#include "Vertex3D.h"

class GraphRenderer3D {
public:
    static void drawNode(const Vertex3D& vertex, float radius = 1.0f);
    static void drawEdge(const Vector3& start, const Vector3& end, float radius, Color color);
    static void drawArrowHead(const Vector3& from, const Vector3& to, Color color);
    static Color getNodeColor(NodeState state);
};
