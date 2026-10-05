#include "RoverNavGraph.h"
#include "PhysicsWorld.h"
#include <raymath.h>
#include <limits>
#include <algorithm>

RoverNavGraph::RoverNavGraph() {}

RoverNavGraph::~RoverNavGraph() {
    clear();
}

void RoverNavGraph::clear() {
    unloadEdgeMeshes();
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

    buildEdgeMeshes();
}

static Mesh buildBatchEdgeMesh(const std::vector<std::pair<Vector3, Vector3>>& edgePairs, float radius, Color color) {
    Mesh mesh = {};
    if (edgePairs.empty()) return mesh;

    int numEdges = static_cast<int>(edgePairs.size());
    int numVertices = numEdges * 8;
    int numTriangles = numEdges * 8;

    mesh.vertexCount = numVertices;
    mesh.triangleCount = numTriangles;

    mesh.vertices = static_cast<float*>(MemAlloc(numVertices * 3 * sizeof(float)));
    mesh.normals = static_cast<float*>(MemAlloc(numVertices * 3 * sizeof(float)));
    mesh.colors = static_cast<unsigned char*>(MemAlloc(numVertices * 4 * sizeof(unsigned char)));
    mesh.indices = static_cast<unsigned short*>(MemAlloc(numTriangles * 3 * sizeof(unsigned short)));

    int vIdx = 0;
    int tIdx = 0;

    for (int i = 0; i < numEdges; ++i) {
        Vector3 p1 = edgePairs[i].first;
        Vector3 p2 = edgePairs[i].second;

        Vector3 dir = Vector3Subtract(p2, p1);
        float len = Vector3Length(dir);
        if (len < 1e-4f) dir = Vector3{ 0, 1, 0 };
        else dir = Vector3Scale(dir, 1.0f / len);

        Vector3 v1 = (fabsf(dir.y) < 0.95f) ? Vector3Normalize(Vector3CrossProduct(dir, Vector3{ 0, 1, 0 }))
                                            : Vector3Normalize(Vector3CrossProduct(dir, Vector3{ 1, 0, 0 }));
        Vector3 v2 = Vector3CrossProduct(dir, v1);

        Vector3 r1 = Vector3Scale(v1, radius);
        Vector3 r2 = Vector3Scale(v2, radius);

        int baseIdx = vIdx;

        // 4 base vertices around p1
        Vector3 ringB[4] = {
            Vector3Add(p1, r1),
            Vector3Add(p1, r2),
            Vector3Subtract(p1, r1),
            Vector3Subtract(p1, r2)
        };

        // 4 top vertices around p2
        Vector3 ringT[4] = {
            Vector3Add(p2, r1),
            Vector3Add(p2, r2),
            Vector3Subtract(p2, r1),
            Vector3Subtract(p2, r2)
        };

        Vector3 sideNorms[4] = { v1, v2, Vector3Negate(v1), Vector3Negate(v2) };

        for (int k = 0; k < 4; ++k) {
            mesh.vertices[(baseIdx + k) * 3 + 0] = ringB[k].x;
            mesh.vertices[(baseIdx + k) * 3 + 1] = ringB[k].y;
            mesh.vertices[(baseIdx + k) * 3 + 2] = ringB[k].z;

            mesh.normals[(baseIdx + k) * 3 + 0] = sideNorms[k].x;
            mesh.normals[(baseIdx + k) * 3 + 1] = sideNorms[k].y;
            mesh.normals[(baseIdx + k) * 3 + 2] = sideNorms[k].z;

            mesh.colors[(baseIdx + k) * 4 + 0] = color.r;
            mesh.colors[(baseIdx + k) * 4 + 1] = color.g;
            mesh.colors[(baseIdx + k) * 4 + 2] = color.b;
            mesh.colors[(baseIdx + k) * 4 + 3] = color.a;

            mesh.vertices[(baseIdx + 4 + k) * 3 + 0] = ringT[k].x;
            mesh.vertices[(baseIdx + 4 + k) * 3 + 1] = ringT[k].y;
            mesh.vertices[(baseIdx + 4 + k) * 3 + 2] = ringT[k].z;

            mesh.normals[(baseIdx + 4 + k) * 3 + 0] = sideNorms[k].x;
            mesh.normals[(baseIdx + 4 + k) * 3 + 1] = sideNorms[k].y;
            mesh.normals[(baseIdx + 4 + k) * 3 + 2] = sideNorms[k].z;

            mesh.colors[(baseIdx + 4 + k) * 4 + 0] = color.r;
            mesh.colors[(baseIdx + 4 + k) * 4 + 1] = color.g;
            mesh.colors[(baseIdx + 4 + k) * 4 + 2] = color.b;
            mesh.colors[(baseIdx + 4 + k) * 4 + 3] = color.a;
        }

        vIdx += 8;

        // 4 side quads (each has 2 triangles)
        for (int k = 0; k < 4; ++k) {
            int nextK = (k + 1) % 4;
            unsigned short bk = static_cast<unsigned short>(baseIdx + k);
            unsigned short bnext = static_cast<unsigned short>(baseIdx + nextK);
            unsigned short tk = static_cast<unsigned short>(baseIdx + 4 + k);
            unsigned short tnext = static_cast<unsigned short>(baseIdx + 4 + nextK);

            // Triangle 1
            mesh.indices[tIdx++] = bk;
            mesh.indices[tIdx++] = tk;
            mesh.indices[tIdx++] = bnext;

            // Triangle 2
            mesh.indices[tIdx++] = bnext;
            mesh.indices[tIdx++] = tk;
            mesh.indices[tIdx++] = tnext;
        }
    }

    UploadMesh(&mesh, false);
    return mesh;
}

void RoverNavGraph::unloadEdgeMeshes() {
    if (m_edgesModelsLoaded) {
        if (m_walkableEdgesModel.meshCount > 0) UnloadModel(m_walkableEdgesModel);
        if (m_blockedEdgesModel.meshCount > 0) UnloadModel(m_blockedEdgesModel);
        m_walkableEdgesModel = {};
        m_blockedEdgesModel = {};
        m_edgesModelsLoaded = false;
    }
}

void RoverNavGraph::buildEdgeMeshes() {
    unloadEdgeMeshes();

    std::vector<std::pair<Vector3, Vector3>> walkablePairs;
    std::vector<std::pair<Vector3, Vector3>> blockedPairs;

    for (const auto& e : m_edges) {
        if (e.isBlocked) {
            blockedPairs.push_back({ e.start, e.end });
        } else {
            walkablePairs.push_back({ e.start, e.end });
        }
    }

    if (!walkablePairs.empty()) {
        Mesh wm = buildBatchEdgeMesh(walkablePairs, 0.05f, Color{ 140, 175, 215, 180 });
        m_walkableEdgesModel = LoadModelFromMesh(wm);
    }
    if (!blockedPairs.empty()) {
        Mesh bm = buildBatchEdgeMesh(blockedPairs, 0.14f, Color{ 240, 45, 45, 240 });
        m_blockedEdgesModel = LoadModelFromMesh(bm);
    }
    m_edgesModelsLoaded = true;
}

void RoverNavGraph::renderEdges() const {
    if (!m_edgesModelsLoaded) return;
    if (m_walkableEdgesModel.meshCount > 0) {
        DrawModel(m_walkableEdgesModel, Vector3Zero(), 1.0f, WHITE);
    }
    if (m_blockedEdgesModel.meshCount > 0) {
        DrawModel(m_blockedEdgesModel, Vector3Zero(), 1.0f, WHITE);
    }
}

void RoverNavGraph::validateEdgesWithPhysics(PhysicsWorld& physics, float clearanceOffset) {
    Vector3 upOffset = { 0.0f, clearanceOffset, 0.0f };

    for (auto& edge : m_edges) {
        Vector3 from = Vector3Add(edge.start, upOffset);
        Vector3 to = Vector3Add(edge.end, upOffset);

        Vector3 hitPoint;
        if (physics.raycast(from, to, &hitPoint)) {
            edge.isBlocked = true;
            edge.color = Color{ 220, 45, 45, 230 }; // Impassable collision obstruction
        }
    }

    // Rebuild GPU edge meshes with updated blocked statuses
    buildEdgeMeshes();
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
