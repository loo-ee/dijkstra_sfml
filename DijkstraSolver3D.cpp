#include "DijkstraSolver3D.h"
#include <cmath>
#include <algorithm>
#include <iostream>

DijkstraSolver3D::DijkstraSolver3D() {
    applyPreset(0);
}

DijkstraSolver3D::~DijkstraSolver3D() {}

void DijkstraSolver3D::applyPreset(int presetIndex) {
    switch (presetIndex) {
        case 0:
            // Standard Martian Exploration (balanced energy & terrain traction)
            m_weights.alpha = 2.0f;
            m_weights.beta = 1.5f;
            m_weights.gamma = 1.0f;
            m_weights.delta = 0.8f;
            m_weights.name = "Standard Martian Rover";
            break;
        case 1:
            // Direct / Geometric Distance Priority (minimal terrain weight)
            m_weights.alpha = 0.2f;
            m_weights.beta = 0.3f;
            m_weights.gamma = 0.1f;
            m_weights.delta = 0.2f;
            m_weights.name = "Direct / Euclidean Route";
            break;
        case 2:
            // Extreme Energy Saver (routes strictly around hills and craters)
            m_weights.alpha = 4.5f;
            m_weights.beta = 2.5f;
            m_weights.gamma = 1.5f;
            m_weights.delta = 1.2f;
            m_weights.name = "Energy Saver (Contour Routing)";
            break;
        case 3:
            // High-Traction Safety First (extreme penalty for loose sand and steep inclines)
            m_weights.alpha = 1.5f;
            m_weights.beta = 5.0f;
            m_weights.gamma = 3.5f;
            m_weights.delta = 1.0f;
            m_weights.name = "High-Traction Safety First";
            break;
        default:
            break;
    }
}

void DijkstraSolver3D::setWeights(const CostWeights& weights) {
    m_weights = weights;
}

float DijkstraSolver3D::computeEdgeCost(const Vertex3D* u, const Vertex3D* v, 
                                        const Vertex3D* parentOfU, bool isEdgeBlocked) const {
    if (isEdgeBlocked || !u || !v || !u->isWalkable || !v->isWalkable) {
        return std::numeric_limits<float>::infinity();
    }

    // 1. 3D Euclidean Distance
    float d = Vector3Distance(u->position, v->position);
    if (d < 1e-4f) return 0.0f;

    // 2. Elevation Delta & Work (Uphill climb vs downhill slope descent)
    float dy = v->position.y - u->position.y;
    float gravityFactor = 0.0f;
    if (dy >= 0.0f) {
        gravityFactor = m_weights.alpha * dy;
    } else {
        // Controlled downhill slope traversal
        gravityFactor = -0.3f * fabsf(dy);
    }

    // 3. Slope Angle & Surface Friction
    float horizDist = sqrtf((v->position.x - u->position.x) * (v->position.x - u->position.x) + 
                            (v->position.z - u->position.z) * (v->position.z - u->position.z));
    float segmentSlopeRad = atan2f(fabsf(dy), std::max(horizDist, 1e-3f));
    float maxSlopeRad = std::max(segmentSlopeRad, std::max(u->slopeAngleRad, v->slopeAngleRad));
    float mu_s = 0.5f * (u->surfaceFriction + v->surfaceFriction);
    if (mu_s < 0.05f) mu_s = 0.05f;

    // Coulomb Slip Threshold: if tan(theta) > mu_s, rover wheels spin helplessly
    float tanTheta = tanf(maxSlopeRad);
    if (tanTheta > mu_s) {
        return std::numeric_limits<float>::infinity();
    }

    // Near-slip penalty: beta * (tan(theta) / mu_s)^2
    float slipRatio = tanTheta / mu_s;
    float slipPenalty = m_weights.beta * (slipRatio * slipRatio);

    // Surface roughness / loose soil resistance: gamma * (1.0 - mu_s)
    float frictionPenalty = m_weights.gamma * (1.0f - mu_s);

    // Base physical work multiplier (clamped to ensure strictly positive edge weights)
    float unitCost = std::max(0.1f, 1.0f + gravityFactor + slipPenalty + frictionPenalty);

    // 4. Directional Heading Change Penalty
    float turnPenalty = 0.0f;
    if (parentOfU != nullptr) {
        Vector2 inDir = Vector2Normalize(Vector2{ u->position.x - parentOfU->position.x, u->position.z - parentOfU->position.z });
        Vector2 outDir = Vector2Normalize(Vector2{ v->position.x - u->position.x, v->position.z - u->position.z });
        float cosDpsi = Clamp(inDir.x * outDir.x + inDir.y * outDir.y, -1.0f, 1.0f);
        turnPenalty = m_weights.delta * (1.0f - cosDpsi);
    }

    return d * unitCost + turnPenalty;
}

void DijkstraSolver3D::solveInstant(Vertex3D* start, Vertex3D* end, 
                                   const std::vector<Vertex3D*>& allVertices,
                                   const std::unordered_map<std::string, bool>& blockedEdgesMap) {
    m_startNode = start;
    m_endNode = end;
    m_allVertices = allVertices;
    m_vertexMap.clear();
    m_cachedPathNames.clear();

    if (!m_startNode || !m_endNode || m_allVertices.empty()) {
        m_pathStats = PathStats{};
        return;
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    for (Vertex3D* v : m_allVertices) {
        v->minDistanceFromSrc = std::numeric_limits<float>::infinity();
        v->parent = nullptr;
        m_vertexMap[v->name] = v;
    }

    m_startNode->minDistanceFromSrc = 0.0f;

    // Min-heap Priority Queue: pair<distance, Vertex3D*>
    typedef std::pair<float, Vertex3D*> DistVertexPair;
    std::priority_queue<DistVertexPair, std::vector<DistVertexPair>, std::greater<DistVertexPair>> pq;
    pq.push({ 0.0f, m_startNode });

    auto makeEdgeKey = [](const std::string& a, const std::string& b) {
        return (a < b) ? (a + "_" + b) : (b + "_" + a);
    };

    bool reachedEnd = false;

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        if (d > u->minDistanceFromSrc) continue;
        if (u == m_endNode) {
            reachedEnd = true;
            break;
        }

        for (const auto& neighbor : u->neighbors) {
            auto it = m_vertexMap.find(neighbor.first);
            if (it == m_vertexMap.end()) continue;
            Vertex3D* v = it->second;

            std::string edgeKey = makeEdgeKey(u->name, v->name);
            bool isBlocked = false;
            auto blockIt = blockedEdgesMap.find(edgeKey);
            if (blockIt != blockedEdgesMap.end()) {
                isBlocked = blockIt->second;
            }

            float cost = computeEdgeCost(u, v, u->parent, isBlocked);
            if (std::isinf(cost)) continue;

            float newDist = u->minDistanceFromSrc + cost;
            if (newDist < v->minDistanceFromSrc) {
                v->minDistanceFromSrc = newDist;
                v->parent = u;
                pq.push({ newDist, v });
            }
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    float elapsedMs = std::chrono::duration<float, std::milli>(endTime - startTime).count();

    // Cache resulting path
    if (reachedEnd && m_endNode->parent != nullptr) {
        Vertex3D* curr = m_endNode;
        while (curr != nullptr) {
            m_cachedPathNames.push_back(curr->name);
            curr = curr->parent;
        }
        std::reverse(m_cachedPathNames.begin(), m_cachedPathNames.end());
    }

    computePathStats();
    m_pathStats.computeTimeMs = elapsedMs;
}

void DijkstraSolver3D::solveWithHistory(Vertex3D* start, Vertex3D* end, 
                                        const std::vector<Vertex3D*>& allVertices,
                                        const std::unordered_map<std::string, bool>& blockedEdgesMap) {
    m_startNode = start;
    m_endNode = end;
    m_allVertices = allVertices;
    m_history.clear();
    m_vertexMap.clear();
    m_currentStepIndex = 0;
    m_cachedPathNames.clear();

    if (!m_startNode || !m_endNode || m_allVertices.empty()) {
        m_pathStats = PathStats{};
        return;
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    std::unordered_map<std::string, float> distances;
    std::unordered_map<std::string, std::string> parents;
    std::vector<std::string> visited;

    for (Vertex3D* v : m_allVertices) {
        distances[v->name] = std::numeric_limits<float>::infinity();
        parents[v->name] = "";
        m_vertexMap[v->name] = v;
        v->minDistanceFromSrc = std::numeric_limits<float>::infinity();
        v->parent = nullptr;
    }

    distances[m_startNode->name] = 0.0f;
    m_startNode->minDistanceFromSrc = 0.0f;

    auto makeEdgeKey = [](const std::string& a, const std::string& b) {
        return (a < b) ? (a + "_" + b) : (b + "_" + a);
    };

    auto recordSnapshot = [&](const std::string& current, const std::string& neighbor, 
                              const std::string& msg, float candidateCost, bool finished, bool found) {
        DijkstraSnapshot3D snap;
        snap.currentNode = current;
        snap.examiningNeighbor = neighbor;
        snap.message = msg;
        snap.distances = distances;
        snap.parents = parents;
        snap.visitedNodes = visited;
        snap.candidateEdgeCost = candidateCost;
        snap.isFinished = finished;
        snap.pathFound = found;
        m_history.push_back(snap);
    };

    recordSnapshot("", "", "Mission Initialized: Origin [" + m_startNode->name + "] -> Target [" + m_endNode->name + "]", 0.0f, false, false);

    typedef std::pair<float, Vertex3D*> DistVertexPair;
    std::priority_queue<DistVertexPair, std::vector<DistVertexPair>, std::greater<DistVertexPair>> pq;
    pq.push({ 0.0f, m_startNode });

    bool pathFound = false;

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        if (d > distances[u->name]) continue;

        visited.push_back(u->name);

        recordSnapshot(u->name, "", "Settled Node [" + u->name + "] | Accumulated Energy Cost: " + TextFormat("%.1f", d), 0.0f, false, false);

        if (u == m_endNode) {
            pathFound = true;
            recordSnapshot(u->name, "", std::string("Target Reached! Optimal Physics Cost: ") + TextFormat("%.1f", d), 0.0f, true, true);
            break;
        }

        Vertex3D* parentOfU = parents[u->name].empty() ? nullptr : m_vertexMap[parents[u->name]];

        for (const auto& neighbor : u->neighbors) {
            auto it = m_vertexMap.find(neighbor.first);
            if (it == m_vertexMap.end()) continue;
            Vertex3D* v = it->second;

            // Skip already visited nodes
            if (std::find(visited.begin(), visited.end(), v->name) != visited.end()) {
                continue;
            }

            std::string edgeKey = makeEdgeKey(u->name, v->name);
            bool isBlocked = false;
            auto blockIt = blockedEdgesMap.find(edgeKey);
            if (blockIt != blockedEdgesMap.end()) {
                isBlocked = blockIt->second;
            }

            float cost = computeEdgeCost(u, v, parentOfU, isBlocked);
            if (std::isinf(cost)) {
                continue; // Blocked or slipping
            }

            float newDist = distances[u->name] + cost;
            if (newDist < distances[v->name]) {
                distances[v->name] = newDist;
                parents[v->name] = u->name;
                v->minDistanceFromSrc = newDist;
                v->parent = u;
                pq.push({ newDist, v });

                recordSnapshot(u->name, v->name, "Relaxed Edge: [" + u->name + " -> " + v->name + "] (Cost: " + std::string(TextFormat("%.1f", cost)) + ")", cost, false, false);
            }
        }
    }

    if (!pathFound) {
        recordSnapshot("", "", "No traversable path to destination (All routes blocked or exceed friction threshold).", 0.0f, true, false);
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    float elapsedMs = std::chrono::duration<float, std::milli>(endTime - startTime).count();

    // Cache resulting path
    if (pathFound && !parents[m_endNode->name].empty()) {
        std::string curr = m_endNode->name;
        while (!curr.empty()) {
            m_cachedPathNames.push_back(curr);
            curr = parents[curr];
        }
        std::reverse(m_cachedPathNames.begin(), m_cachedPathNames.end());
    }

    computePathStats();
    m_pathStats.computeTimeMs = elapsedMs;

    // Start at final step by default so shortest path is immediately visible
    if (!m_history.empty()) {
        m_currentStepIndex = m_history.size() - 1;
    }
}

void DijkstraSolver3D::computePathStats() {
    m_pathStats = PathStats{};
    if (m_cachedPathNames.empty() || m_cachedPathNames.size() < 2) {
        m_pathStats.isValid = false;
        return;
    }

    m_pathStats.isValid = true;
    m_pathStats.waypointCount = static_cast<int>(m_cachedPathNames.size());

    float distSum = 0.0f;
    float gainSum = 0.0f;
    float lossSum = 0.0f;
    float maxSlope = 0.0f;

    for (size_t i = 0; i < m_cachedPathNames.size() - 1; ++i) {
        Vertex3D* u = m_vertexMap[m_cachedPathNames[i]];
        Vertex3D* v = m_vertexMap[m_cachedPathNames[i + 1]];
        if (!u || !v) continue;

        float segDist = Vector3Distance(u->position, v->position);
        distSum += segDist;

        float dy = v->position.y - u->position.y;
        if (dy > 0.0f) gainSum += dy;
        else lossSum += fabsf(dy);

        float horizDist = sqrtf((v->position.x - u->position.x) * (v->position.x - u->position.x) + 
                                (v->position.z - u->position.z) * (v->position.z - u->position.z));
        float slopeDeg = atan2f(fabsf(dy), std::max(horizDist, 1e-3f)) * RAD2DEG;
        if (slopeDeg > maxSlope) maxSlope = slopeDeg;
    }

    m_pathStats.totalDistance = distSum;
    m_pathStats.elevationGain = gainSum;
    m_pathStats.elevationLoss = lossSum;
    m_pathStats.maxSlopeDeg = maxSlope;

    if (m_endNode && !std::isinf(m_endNode->minDistanceFromSrc)) {
        m_pathStats.totalEnergyCost = m_endNode->minDistanceFromSrc;
    }
}

void DijkstraSolver3D::update(float dt) {
    if (m_isPlaying && !m_history.empty()) {
        m_playbackTimer += dt;
        while (m_playbackTimer >= m_stepInterval) {
            m_playbackTimer -= m_stepInterval;
            if (m_currentStepIndex + 1 < m_history.size()) {
                m_currentStepIndex++;
            } else {
                m_isPlaying = false;
                break;
            }
        }
    }
}

bool DijkstraSolver3D::stepForward() {
    if (m_currentStepIndex + 1 < m_history.size()) {
        m_currentStepIndex++;
        return true;
    }
    return false;
}

bool DijkstraSolver3D::stepBackward() {
    if (m_currentStepIndex > 0) {
        m_currentStepIndex--;
        return true;
    }
    return false;
}

void DijkstraSolver3D::jumpToStart() {
    m_currentStepIndex = 0;
    m_isPlaying = false;
}

void DijkstraSolver3D::jumpToEnd() {
    if (!m_history.empty()) {
        m_currentStepIndex = m_history.size() - 1;
    }
    m_isPlaying = false;
}

void DijkstraSolver3D::reset() {
    m_currentStepIndex = 0;
    m_isPlaying = false;
    m_playbackTimer = 0.0f;
}

void DijkstraSolver3D::togglePlay() {
    if (m_history.empty()) return;
    if (m_currentStepIndex + 1 >= m_history.size()) {
        m_currentStepIndex = 0; // Loop around to start if at end
    }
    m_isPlaying = !m_isPlaying;
}

bool DijkstraSolver3D::isFinished() const {
    if (m_history.empty()) return false;
    return m_history[m_currentStepIndex].isFinished;
}

bool DijkstraSolver3D::isPathFound() const {
    if (m_history.empty()) return false;
    return m_history[m_currentStepIndex].pathFound;
}

const DijkstraSnapshot3D& DijkstraSolver3D::getCurrentSnapshot() const {
    if (m_history.empty() || m_currentStepIndex >= m_history.size()) {
        return m_emptySnapshot;
    }
    return m_history[m_currentStepIndex];
}

std::vector<const Vertex3D*> DijkstraSolver3D::getShortestPathNodes() const {
    std::vector<const Vertex3D*> nodes;
    if (m_history.empty()) return nodes;

    const auto& snap = m_history[m_currentStepIndex];
    if (!snap.pathFound || !m_endNode) return nodes;

    std::string curr = m_endNode->name;
    while (!curr.empty()) {
        auto it = m_vertexMap.find(curr);
        if (it != m_vertexMap.end()) {
            nodes.push_back(it->second);
        }
        auto parentIt = snap.parents.find(curr);
        if (parentIt != snap.parents.end()) {
            curr = parentIt->second;
        } else {
            break;
        }
    }
    std::reverse(nodes.begin(), nodes.end());
    return nodes;
}
