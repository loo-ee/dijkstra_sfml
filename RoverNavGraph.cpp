#include "RoverNavGraph.h"
#include <raymath.h>
#include <limits>
#include <algorithm>

RoverNavGraph::RoverNavGraph() {}

RoverNavGraph::~RoverNavGraph() {
    clear();
}

void RoverNavGraph::clear() {
    for (Vertex3D* v : m_vertices) {
        delete v;
    }
    m_vertices.clear();
    m_edges.clear();
    m_startNode = nullptr;
    m_endNode = nullptr;
    m_gridCols = 0;
    m_gridRows = 0;
}

void RoverNavGraph::generateTerrainGrid(const TerrainHeightfield& terrain, int gridCols, int gridRows, float spacing) {
    clear();

    m_gridCols = gridCols;
    m_gridRows = gridRows;
    m_spacing = spacing;

    float offsetX = (gridCols - 1) * spacing * 0.5f;
    float offsetZ = (gridRows - 1) * spacing * 0.5f;

    m_vertices.reserve(gridCols * gridRows);

    // 1. Create Nodes Draped over Terrain (+0.3m elevation offset to prevent z-fighting)
    for (int r = 0; r < gridRows; ++r) {
        for (int c = 0; c < gridCols; ++c) {
            float worldX = c * spacing - offsetX;
            float worldZ = r * spacing - offsetZ;
            float worldY = terrain.getHeight(worldX, worldZ) + 0.3f;

            std::string name = "N_" + std::to_string(c) + "_" + std::to_string(r);
            Vertex3D* node = new Vertex3D(name, Vector3{ worldX, worldY, worldZ });

            // Slope angle and traversability properties
            node->slopeAngleRad = terrain.getSlopeAngleRad(worldX, worldZ);
            node->surfaceFriction = 0.70f;
            node->isWalkable = (node->slopeAngleRad < 32.0f * DEG2RAD);

            if (!node->isWalkable) {
                node->state = NodeState::IMPASSABLE;
            } else {
                node->state = NodeState::DEFAULT;
            }

            m_vertices.push_back(node);
        }
    }

    auto getIndex = [gridCols](int c, int r) {
        return r * gridCols + c;
    };

    // 2. Connect 8-Neighborhood (Cardinals + Diagonals)
    const int dc[] = { 1, -1, 0,  0, 1, -1,  1, -1 };
    const int dr[] = { 0,  0, 1, -1, 1,  1, -1, -1 };

    for (int r = 0; r < gridRows; ++r) {
        for (int c = 0; c < gridCols; ++c) {
            int uIdx = getIndex(c, r);
            Vertex3D* u = m_vertices[uIdx];

            for (int i = 0; i < 8; ++i) {
                int nc = c + dc[i];
                int nr = r + dr[i];

                if (nc >= 0 && nc < gridCols && nr >= 0 && nr < gridRows) {
                    int vIdx = getIndex(nc, nr);
                    Vertex3D* v = m_vertices[vIdx];

                    float dist = Vector3Distance(u->position, v->position);
                    u->neighbors.emplace_back(v->name, dist);

                    // Add unique undirected edge to rendering list
                    if (uIdx < vIdx) {
                        Color edgeColor;
                        if (!u->isWalkable || !v->isWalkable) {
                            edgeColor = Color{ 70, 50, 50, 140 }; // Impassable slope connection
                        } else {
                            edgeColor = Color{ 90, 115, 145, 190 }; // Walkable nav route
                        }
                        m_edges.push_back({ u->position, v->position, edgeColor });
                    }
                }
            }
        }
    }

    // Set Default Start and End Nodes (opposite walkable corners)
    if (!m_vertices.empty()) {
        setStartNode(m_vertices.front());
        setEndNode(m_vertices.back());
    }
}

Vertex3D* RoverNavGraph::pickNodeFromRay(Ray mouseRay, float pickRadius) {
    Vertex3D* closestHitNode = nullptr;
    float minHitDistance = std::numeric_limits<float>::max();

    for (Vertex3D* v : m_vertices) {
        RayCollision col = GetRayCollisionSphere(mouseRay, v->position, pickRadius);
        if (col.hit && col.distance < minHitDistance) {
            minHitDistance = col.distance;
            closestHitNode = v;
        }
    }

    return closestHitNode;
}

Vertex3D* RoverNavGraph::getClosestNode(Vector3 worldPos) {
    Vertex3D* bestNode = nullptr;
    float minDistSq = std::numeric_limits<float>::max();

    for (Vertex3D* v : m_vertices) {
        float dsq = Vector3DistanceSqr(worldPos, v->position);
        if (dsq < minDistSq) {
            minDistSq = dsq;
            bestNode = v;
        }
    }

    return bestNode;
}

void RoverNavGraph::setStartNode(Vertex3D* node) {
    if (!node) return;

    if (m_startNode && m_startNode != m_endNode) {
        m_startNode->state = m_startNode->isWalkable ? NodeState::DEFAULT : NodeState::IMPASSABLE;
    }

    m_startNode = node;
    m_startNode->state = NodeState::START;
}

void RoverNavGraph::setEndNode(Vertex3D* node) {
    if (!node) return;

    if (m_endNode && m_endNode != m_startNode) {
        m_endNode->state = m_endNode->isWalkable ? NodeState::DEFAULT : NodeState::IMPASSABLE;
    }

    m_endNode = node;
    m_endNode->state = NodeState::END;
}
