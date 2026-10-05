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
    Vector3 surfaceNormal = { 0.0f, 1.0f, 0.0f };
    float surfaceFriction = 0.6f;
    float slopeAngleRad = 0.0f;
    float cliffProximity = 0.0f; // Clearance penalty for nodes adjacent to cliffs/drop-offs
    bool isWalkable = true;

    Vertex3D(const std::string& nodeName, Vector3 pos)
        : name(nodeName), position(pos) {}
};
