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
    m_spatialNodes.clear();
    m_edges.clear();
    m_blockedEdgesMap.clear();
    m_startNode = nullptr;
    m_endNode = nullptr;
    m_gridCols = 0;
    m_gridRows = 0;
}

void RoverNavGraph::generateTerrainGrid(const TerrainHeightfield& terrain, int gridCols, int gridRows, float spacing) {
    generateCenteredGrid(terrain, Vector3{ 0.0f, 0.0f, 0.0f }, gridCols, gridRows, spacing);
}

void RoverNavGraph::generateCenteredGrid(const TerrainHeightfield& terrain, Vector3 centerPos, int gridCols, int gridRows, float spacing) {
    clear();
    m_gridCols = gridCols;
    m_gridRows = gridRows;
    m_spacing = spacing;
    float radius = (std::max(gridCols, gridRows) * spacing) * 0.5f;
    if (radius < 640.0f) radius = 640.0f; // Ensure full globe surface coverage!
    generatePersistentPlanetaryGrid(terrain, centerPos, radius, spacing);
}

void RoverNavGraph::generatePersistentPlanetaryGrid(const TerrainHeightfield& terrain, Vector3 centerPos, float radius, float spacing) {
    m_spacing = spacing;

    int minGx = static_cast<int>(floorf((centerPos.x - radius) / spacing));
    int maxGx = static_cast<int>(ceilf((centerPos.x + radius) / spacing));
    int minGz = static_cast<int>(floorf((centerPos.z - radius) / spacing));
    int maxGz = static_cast<int>(ceilf((centerPos.z + radius) / spacing));

    float radiusSq = radius * radius;
    std::vector<Vertex3D*> newNodes;
    newNodes.reserve(2048);

    // 1. Create Nodes Draped over Terrain for any unvisited cells within planetary radius
    for (int gz = minGz; gz <= maxGz; ++gz) {
        for (int gx = minGx; gx <= maxGx; ++gx) {
            float worldX = gx * spacing;
            float worldZ = gz * spacing;

            float dx = worldX - centerPos.x;
            float dz = worldZ - centerPos.z;
            if (dx * dx + dz * dz > radiusSq) {
                continue;
            }

            int64_t key = getCellKey(gx, gz);
            if (m_spatialNodes.find(key) != m_spatialNodes.end()) {
                continue; // Node already exists in persistent memory! Preserved!
            }

            float worldY = terrain.getHeight(worldX, worldZ) + 0.35f;
            std::string name = "N_" + std::to_string(gx) + "_" + std::to_string(gz);
            Vertex3D* node = new Vertex3D(name, Vector3{ worldX, worldY, worldZ });

            // Slope angle and traversability properties
            node->surfaceNormal = terrain.getNormal(worldX, worldZ);
            node->slopeAngleRad = acosf(Clamp(node->surfaceNormal.y, -1.0f, 1.0f));
            node->surfaceFriction = 0.70f;
            // Realistic rover mobility limit: slopes >= 22 deg (~40% grade) are impassable
            node->isWalkable = (node->slopeAngleRad < 22.0f * DEG2RAD);

            if (!node->isWalkable) {
                node->state = NodeState::IMPASSABLE;
            } else {
                node->state = NodeState::DEFAULT;
            }

            m_spatialNodes[key] = node;
            m_vertices.push_back(node);
            newNodes.push_back(node);
        }
    }

    if (newNodes.empty() && !m_edges.empty()) {
        return; // All nodes already generated, nothing new to connect
    }

    // 2. Connect Newly Generated Nodes to 8-Neighborhood (Cardinals + Diagonals)
    const int dgx[] = { 1, -1, 0,  0, 1, -1,  1, -1 };
    const int dgz[] = { 0,  0, 1, -1, 1,  1, -1, -1 };

    for (Vertex3D* u : newNodes) {
        int gx = static_cast<int>(roundf(u->position.x / spacing));
        int gz = static_cast<int>(roundf(u->position.z / spacing));

        float maxAdjSlope = u->slopeAngleRad;
        bool nextToCliff = false;

        for (int i = 0; i < 8; ++i) {
            int ngx = gx + dgx[i];
            int ngz = gz + dgz[i];
            int64_t nKey = getCellKey(ngx, ngz);

            auto it = m_spatialNodes.find(nKey);
            if (it != m_spatialNodes.end()) {
                Vertex3D* v = it->second;
                if (!v->isWalkable || v->slopeAngleRad > 18.0f * DEG2RAD) {
                    nextToCliff = true;
                }
                if (v->slopeAngleRad > maxAdjSlope) {
                    maxAdjSlope = v->slopeAngleRad;
                }

                // Add bidirectional edge
                float dist = Vector3Distance(u->position, v->position);
                u->neighbors.emplace_back(v->name, dist);
                v->neighbors.emplace_back(u->name, dist);

                // Add unique undirected edge to rendering list
                bool steep = (!u->isWalkable || !v->isWalkable);
                Color edgeColor = steep ? Color{ 180, 40, 40, 75 } : Color{ 60, 205, 255, 175 };
                m_edges.push_back({ u->position, v->position, edgeColor, steep, u->name, v->name });
                if (steep) {
                    std::string key = (u->name < v->name) ? (u->name + "_" + v->name) : (v->name + "_" + u->name);
                    m_blockedEdgesMap[key] = true;
                }
            }
        }

        if (nextToCliff) {
            u->cliffProximity = 0.85f;
        } else if (maxAdjSlope > 14.0f * DEG2RAD) {
            u->cliffProximity = 0.40f;
        }
    }

    // Set Default Start and End Nodes near center of exploration if not yet set
    if (!m_startNode && !m_vertices.empty()) {
        Vector3 preferredStart = Vector3Add(centerPos, Vector3{ -25.0f, 0.0f, -20.0f });
        Vertex3D* bestStart = nullptr;
        float bestDist = 1e9f;
        for (Vertex3D* v : m_vertices) {
            if (v->isWalkable) {
                float d = Vector3Distance(v->position, preferredStart);
                if (d < bestDist) {
                    bestDist = d;
                    bestStart = v;
                }
            }
        }
        if (bestStart) setStartNode(bestStart);
    }
    if (!m_endNode && m_vertices.size() > 1) {
        Vector3 preferredEnd = Vector3Add(centerPos, Vector3{ 38.0f, 0.0f, 32.0f });
        Vertex3D* bestEnd = nullptr;
        float bestDist = 1e9f;
        for (Vertex3D* v : m_vertices) {
            if (v->isWalkable && v != m_startNode) {
                float d = Vector3Distance(v->position, preferredEnd);
                if (d < bestDist) {
                    bestDist = d;
                    bestEnd = v;
                }
            }
        }
        if (bestEnd) setEndNode(bestEnd);
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
        Mesh wm = buildBatchEdgeMesh(walkablePairs, 0.075f, Color{ 50, 205, 255, 180 });
        m_walkableEdgesModel = LoadModelFromMesh(wm);
    }
    if (!blockedPairs.empty()) {
        Mesh bm = buildBatchEdgeMesh(blockedPairs, 0.04f, Color{ 180, 45, 45, 75 });
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
    m_blockedEdgesMap.clear();

    for (auto& edge : m_edges) {
        std::string key = (edge.startNode < edge.endNode) ? 
                          (edge.startNode + "_" + edge.endNode) : 
                          (edge.endNode + "_" + edge.startNode);

        if (edge.isBlocked) {
            m_blockedEdgesMap[key] = true;
            continue;
        }

        Vector3 from = Vector3Add(edge.start, upOffset);
        Vector3 to = Vector3Add(edge.end, upOffset);

        Vector3 hitPoint;
        if (physics.raycast(from, to, &hitPoint)) {
            edge.isBlocked = true;
            edge.color = Color{ 220, 45, 45, 230 }; // Impassable collision obstruction
            m_blockedEdgesMap[key] = true;
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

Vertex3D* RoverNavGraph::getClosestWalkableNode(Vector3 worldPos) {
    Vertex3D* bestNode = nullptr;
    float minDistSq = std::numeric_limits<float>::max();

    for (Vertex3D* v : m_vertices) {
        if (!v->isWalkable) continue;
        float dsq = Vector3DistanceSqr(worldPos, v->position);
        if (dsq < minDistSq) {
            minDistSq = dsq;
            bestNode = v;
        }
    }

    return bestNode ? bestNode : getClosestNode(worldPos);
}

bool RoverNavGraph::blockEdge(const std::string& nodeA, const std::string& nodeB) {
    std::string key = (nodeA < nodeB) ? (nodeA + "_" + nodeB) : (nodeB + "_" + nodeA);
    m_blockedEdgesMap[key] = true;
    bool found = false;
    for (auto& edge : m_edges) {
        if ((edge.startNode == nodeA && edge.endNode == nodeB) ||
            (edge.startNode == nodeB && edge.endNode == nodeA)) {
            edge.isBlocked = true;
            edge.color = Color{ 220, 50, 50, 200 };
            found = true;
        }
    }
    if (found) {
        buildEdgeMeshes();
    }
    return found;
}

bool RoverNavGraph::blockEdgeBetweenPositions(Vector3 posA, Vector3 posB) {
    Vertex3D* nA = getClosestNode(posA);
    Vertex3D* nB = getClosestNode(posB);
    if (nA && nB && nA != nB) {
        return blockEdge(nA->name, nB->name);
    }
    return false;
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
