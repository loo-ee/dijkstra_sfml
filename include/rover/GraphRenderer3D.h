#pragma once
#include <raylib.h>
#include <raymath.h>
#include <vector>
#include <string>
#include <unordered_map>
#include "Vertex3D.h"

struct DijkstraSnapshot3D;

class GraphRenderer3D {
public:
    static void drawNode(const Vertex3D& vertex, float radius = 1.0f);
    static void drawEdge(const Vector3& start, const Vector3& end, float radius, Color color);
    static void drawArrowHead(const Vector3& from, const Vector3& to, Color color);
    static Color getNodeColor(NodeState state);

    // Phase 4 3D Dijkstra Visualization
    static void drawShortestPath(const std::vector<const Vertex3D*>& path, float time);
    static void drawSearchState(const DijkstraSnapshot3D& snapshot, 
                                const std::vector<Vertex3D*>& allVertices, 
                                float time);
};
