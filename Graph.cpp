#include "include/Graph.h"

Graph::Graph() {}
Graph::~Graph() {}

void Graph::init(Vertex* startVertex, Vertex* endVertex, const std::vector<Vertex*>& allVertices) {
    m_startVertex = startVertex;
    m_endVertex = endVertex;
    m_allVertices = allVertices;
    m_history.clear();
    m_currentStepIndex = 0;

    if (m_startVertex != nullptr && m_endVertex != nullptr && !m_allVertices.empty()) {
        buildFullAlgorithmHistory();
    }
}

void Graph::buildFullAlgorithmHistory() {
    m_history.clear();

    std::unordered_map<std::string, int> distances;
    std::unordered_map<std::string, std::string> parents;
    std::unordered_map<std::string, Vertex*> vertexMap;
    std::vector<std::string> unvisited;
    std::vector<std::string> visited;

    for (Vertex* v : m_allVertices) {
        distances[v->vertexName] = INF;
        parents[v->vertexName] = "";
        vertexMap[v->vertexName] = v;
        unvisited.push_back(v->vertexName);
    }

    distances[m_startVertex->vertexName] = 0;

    // Helper to record current snapshot
    auto recordSnapshot = [&](const std::string& current, const std::string& neighbor, const std::string& msg, bool finished, bool found) {
        DijkstraSnapshot snap;
        snap.currentNode = current;
        snap.examiningNeighbor = neighbor;
        snap.message = msg;
        snap.distances = distances;
        snap.parents = parents;
        snap.visitedNodes = visited;
        snap.isFinished = finished;
        snap.pathFound = found;

        // Build priority queue snapshot sorted by min distance
        for (const std::string& name : unvisited) {
            snap.priorityQueue.push_back({name, distances[name]});
        }
        std::sort(snap.priorityQueue.begin(), snap.priorityQueue.end(),
            [](const std::pair<std::string, int>& a, const std::pair<std::string, int>& b) {
                return a.second < b.second;
            }
        );

        m_history.push_back(snap);
    };

    recordSnapshot("", "", "Algorithm initialized. Start: " + m_startVertex->vertexName + ", End: " + m_endVertex->vertexName, false, false);

    bool pathFound = false;

    while (!unvisited.empty()) {
        // Find unvisited vertex with smallest min distance
        std::string current = "";
        int minDist = INF;

        for (const std::string& name : unvisited) {
            if (distances[name] < minDist) {
                minDist = distances[name];
                current = name;
            }
        }

        // If remaining unvisited nodes are unreachable
        if (current == "" || minDist == INF) {
            recordSnapshot("", "", "No remaining reachable nodes.", true, pathFound);
            break;
        }

        // Mark as visited
        unvisited.erase(std::remove(unvisited.begin(), unvisited.end(), current), unvisited.end());
        visited.push_back(current);

        recordSnapshot(current, "", "[Visited Node] -> " + current + " (Distance: " + std::to_string(minDist) + ")", false, false);

        if (current == m_endVertex->vertexName) {
            pathFound = true;
            recordSnapshot(current, "", "Destination " + current + " reached! Shortest path distance: " + std::to_string(minDist), true, true);
            break;
        }

        Vertex* currVertexObj = vertexMap[current];

        for (const auto& neighbor : currVertexObj->neighbors) {
            const std::string& neighborName = neighbor.first;
            int edgeWeight = neighbor.second;

            // Check if neighbor is unvisited
            if (std::find(unvisited.begin(), unvisited.end(), neighborName) != unvisited.end()) {
                int newDist = distances[current] + edgeWeight;
                std::string msg = "Calculate " + current + " -> " + neighborName + " (" + std::to_string(distances[current]) + " + " + std::to_string(edgeWeight) + " = " + std::to_string(newDist) + ")";

                if (newDist < distances[neighborName]) {
                    msg += " < " + (distances[neighborName] == INF ? "INF" : std::to_string(distances[neighborName])) + " -> UPDATE COST";
                    distances[neighborName] = newDist;
                    parents[neighborName] = current;
                    recordSnapshot(current, neighborName, msg, false, false);
                } else {
                    msg += " >= " + std::to_string(distances[neighborName]) + " -> MAINTAIN COST";
                    recordSnapshot(current, neighborName, msg, false, false);
                }
            }
        }
    }

    if (m_history.empty() || !m_history.back().isFinished) {
        recordSnapshot("", "", "Algorithm finished.", true, pathFound);
    }
}

bool Graph::stepForward() {
    if (m_history.empty()) return false;
    if (m_currentStepIndex + 1 < m_history.size()) {
        m_currentStepIndex++;
        return true;
    }
    return false;
}

bool Graph::stepBackward() {
    if (m_currentStepIndex > 0) {
        m_currentStepIndex--;
        return true;
    }
    return false;
}

void Graph::reset() {
    m_currentStepIndex = 0;
}

bool Graph::isFinished() const {
    if (m_history.empty()) return false;
    return getCurrentSnapshot().isFinished;
}

bool Graph::isPathFound() const {
    if (m_history.empty()) return false;
    return getCurrentSnapshot().pathFound;
}

const DijkstraSnapshot& Graph::getCurrentSnapshot() const {
    static DijkstraSnapshot emptySnap;
    if (m_history.empty()) return emptySnap;
    return m_history[m_currentStepIndex];
}

std::vector<std::string> Graph::getShortestPath() const {
    std::vector<std::string> path;
    if (m_history.empty() || m_endVertex == nullptr) return path;

    const DijkstraSnapshot& snap = getCurrentSnapshot();
    if (snap.distances.find(m_endVertex->vertexName) == snap.distances.end() || snap.distances.at(m_endVertex->vertexName) == INF) {
        return path;
    }

    std::string curr = m_endVertex->vertexName;
    while (curr != "") {
        path.push_back(curr);
        auto it = snap.parents.find(curr);
        if (it != snap.parents.end()) {
            curr = it->second;
        } else {
            break;
        }
    }

    std::reverse(path.begin(), path.end());
    return path;
}
