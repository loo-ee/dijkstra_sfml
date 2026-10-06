#include "GraphManager.h"
#include <cmath>
#include <random>

GraphManager::GraphManager() {}

GraphManager::~GraphManager() {
    clearVertices();
}

void GraphManager::clearVertices() {
    for (Vertex* v : vertices) {
        delete v;
    }
    vertices.clear();
    nameCounter = 0;
}

std::string GraphManager::generateNextVertexName() {
    std::string name;
    int n = nameCounter;
    do {
        name += static_cast<char>('A' + (n % 26));
        n = n / 26 - 1;
    } while (n >= 0);
    
    // Ensure uniqueness
    while (getOneVertex(name) != nullptr) {
        nameCounter++;
        return generateNextVertexName();
    }

    nameCounter++;
    return name;
}

Vertex* GraphManager::createVertex(const std::string& name, sf::Vector2f centerPos, const std::vector<std::pair<std::string, int>>& neighbors) {
    if (getOneVertex(name) != nullptr) {
        return getOneVertex(name);
    }
    Vertex* created = new Vertex(name, neighbors);
    created->setCenterPos(centerPos);
    vertices.push_back(created);
    return created;
}

Vertex* GraphManager::spawnVertexAt(sf::Vector2f centerPos) {
    std::string name = generateNextVertexName();
    return createVertex(name, centerPos);
}

Vertex* GraphManager::getOneVertex(const std::string& vertexName) {
    for (Vertex* v : vertices) {
        if (v->vertexName == vertexName) return v;
    }
    return nullptr;
}

Vertex* GraphManager::getVertexAt(sf::Vector2f pos) {
    for (Vertex* v : vertices) {
        if (v->contains(pos)) return v;
    }
    return nullptr;
}

std::pair<std::string, std::string> GraphManager::getEdgeAt(sf::Vector2f pos, float threshold) {
    for (size_t i = 0; i < vertices.size(); i++) {
        Vertex* u = vertices[i];
        sf::Vector2f uPos = u->getCenterPos();
        for (size_t j = i + 1; j < vertices.size(); j++) {
            Vertex* v = vertices[j];

            if (!hasEdge(u->vertexName, v->vertexName) && !hasEdge(v->vertexName, u->vertexName)) {
                continue;
            }

            sf::Vector2f vPos = v->getCenterPos();

            // Distance from point pos to segment uPos-vPos
            sf::Vector2f line = vPos - uPos;
            float lengthSq = line.x * line.x + line.y * line.y;
            if (lengthSq == 0.f) continue;

            float t = std::max(0.f, std::min(1.f, ((pos.x - uPos.x) * line.x + (pos.y - uPos.y) * line.y) / lengthSq));
            sf::Vector2f proj = uPos + t * line;
            sf::Vector2f diff = pos - proj;
            float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);

            if (dist <= threshold) {
                if (u->vertexName < v->vertexName) {
                    return {u->vertexName, v->vertexName};
                } else {
                    return {v->vertexName, u->vertexName};
                }
            }
        }
    }
    return {"", ""};
}

bool GraphManager::hasEdge(const std::string& uName, const std::string& vName) const {
    for (Vertex* node : vertices) {
        if (node->vertexName == uName) {
            for (const auto& neighbor : node->neighbors) {
                if (neighbor.first == vName) return true;
            }
        }
    }
    return false;
}

EdgeDirection GraphManager::getEdgeDirection(const std::string& uName, const std::string& vName) const {
    bool uHasV = hasEdge(uName, vName);
    bool vHasU = hasEdge(vName, uName);
    if (uHasV && vHasU) return EdgeDirection::BOTH;
    if (uHasV && !vHasU) return EdgeDirection::FORWARD;
    if (!uHasV && vHasU) return EdgeDirection::BACKWARD;
    return EdgeDirection::NONE;
}

void GraphManager::setEdgeDirection(const std::string& uName, const std::string& vName, EdgeDirection dir) {
    Vertex* u = getOneVertex(uName);
    Vertex* v = getOneVertex(vName);
    if (!u || !v) return;

    auto removeOneWay = [](Vertex* node, const std::string& target) {
        node->neighbors.erase(
            std::remove_if(node->neighbors.begin(), node->neighbors.end(),
                [&target](const std::pair<std::string, int>& pair) { return pair.first == target; }),
            node->neighbors.end()
        );
    };

    removeOneWay(u, vName);
    removeOneWay(v, uName);

    if (dir == EdgeDirection::BOTH) {
        u->neighbors.push_back({vName, 0});
        v->neighbors.push_back({uName, 0});
    } else if (dir == EdgeDirection::FORWARD) {
        u->neighbors.push_back({vName, 0});
    } else if (dir == EdgeDirection::BACKWARD) {
        v->neighbors.push_back({uName, 0});
    }

    updateEdgeWeights();
}

void GraphManager::cycleEdgeDirection(const std::string& uName, const std::string& vName) {
    std::string first = uName < vName ? uName : vName;
    std::string second = uName < vName ? vName : uName;

    EdgeDirection current = getEdgeDirection(first, second);
    if (current == EdgeDirection::BOTH) {
        setEdgeDirection(first, second, EdgeDirection::FORWARD);
    } else if (current == EdgeDirection::FORWARD) {
        setEdgeDirection(first, second, EdgeDirection::BACKWARD);
    } else if (current == EdgeDirection::BACKWARD) {
        setEdgeDirection(first, second, EdgeDirection::BOTH);
    }
}

bool GraphManager::addEdge(const std::string& uName, const std::string& vName) {
    if (uName == vName) return false;
    Vertex* u = getOneVertex(uName);
    Vertex* v = getOneVertex(vName);
    if (!u || !v) return false;

    if (!hasEdge(uName, vName)) {
        u->neighbors.push_back({vName, 0});
    }
    if (!hasEdge(vName, uName)) {
        v->neighbors.push_back({uName, 0});
    }

    updateEdgeWeights();
    return true;
}

bool GraphManager::removeEdge(const std::string& uName, const std::string& vName) {
    Vertex* u = getOneVertex(uName);
    Vertex* v = getOneVertex(vName);
    if (!u || !v) return false;

    auto removeNeighbor = [](Vertex* node, const std::string& target) {
        node->neighbors.erase(
            std::remove_if(node->neighbors.begin(), node->neighbors.end(),
                [&target](const std::pair<std::string, int>& pair) { return pair.first == target; }),
            node->neighbors.end()
        );
    };

    removeNeighbor(u, vName);
    removeNeighbor(v, uName);
    return true;
}

void GraphManager::removeVertex(const std::string& name) {
    Vertex* target = getOneVertex(name);
    if (!target) return;

    for (Vertex* v : vertices) {
        if (v == target) continue;
        v->neighbors.erase(
            std::remove_if(v->neighbors.begin(), v->neighbors.end(),
                [&name](const std::pair<std::string, int>& pair) { return pair.first == name; }),
            v->neighbors.end()
        );
    }

    vertices.erase(std::remove(vertices.begin(), vertices.end(), target), vertices.end());
    delete target;
}

std::vector<Vertex*>& GraphManager::getVertices() {
    return vertices;
}

const std::vector<Vertex*>& GraphManager::getVertices() const {
    return vertices;
}

void GraphManager::updateEdgeWeights() {
    for (Vertex* u : vertices) {
        sf::Vector2f uPos = u->getCenterPos();
        for (auto& neighbor : u->neighbors) {
            Vertex* v = getOneVertex(neighbor.first);
            if (!v) continue;
            sf::Vector2f vPos = v->getCenterPos();
            sf::Vector2f diff = vPos - uPos;
            int dist = static_cast<int>(std::round(std::sqrt(diff.x * diff.x + diff.y * diff.y)));
            neighbor.second = dist;
        }
    }
}

void GraphManager::resetGraphStates(Vertex* start, Vertex* end) {
    for (Vertex* v : vertices) {
        v->minDistanceFromSrc = INF;
        v->parent = nullptr;
        if (v == start) {
            v->setState(NodeState::START);
        } else if (v == end) {
            v->setState(NodeState::END);
        } else {
            v->setState(NodeState::DEFAULT);
        }
    }
}

void GraphManager::loadDefaultPreset() {
    clearVertices();

    createVertex("A", sf::Vector2f(450, 420));
    createVertex("B", sf::Vector2f(500, 180));
    createVertex("C", sf::Vector2f(720, 240));
    createVertex("D", sf::Vector2f(760, 460));
    createVertex("E", sf::Vector2f(860, 200));
    createVertex("F", sf::Vector2f(1000, 150));
    createVertex("G", sf::Vector2f(340, 280));
    createVertex("H", sf::Vector2f(1020, 380));

    addEdge("A", "B");
    addEdge("A", "C");
    addEdge("A", "D");
    addEdge("A", "G");

    addEdge("B", "C");
    addEdge("B", "F");
    addEdge("B", "G");

    addEdge("C", "D");
    addEdge("C", "E");

    addEdge("D", "E");
    addEdge("D", "H");

    addEdge("E", "F");
    addEdge("F", "H");
    
    updateEdgeWeights();
}

void GraphManager::loadGridPreset() {
    clearVertices();

    const int cols = 5;
    const int rows = 3;
    const float startX = 350.f;
    const float startY = 180.f;
    const float spacingX = 160.f;
    const float spacingY = 160.f;

    std::vector<std::vector<std::string>> gridNames(rows, std::vector<std::string>(cols));

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            std::string name = generateNextVertexName();
            gridNames[r][c] = name;
            createVertex(name, sf::Vector2f(startX + c * spacingX, startY + r * spacingY));
        }
    }

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if (c + 1 < cols) addEdge(gridNames[r][c], gridNames[r][c + 1]);
            if (r + 1 < rows) addEdge(gridNames[r][c], gridNames[r + 1][c]);
        }
    }

    updateEdgeWeights();
}

void GraphManager::loadRandomPreset() {
    clearVertices();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> distX(350.f, 1050.f);
    std::uniform_real_distribution<float> distY(150.f, 550.f);

    const int count = 8;
    std::vector<std::string> names;

    for (int i = 0; i < count; i++) {
        std::string name = generateNextVertexName();
        names.push_back(name);
        
        sf::Vector2f pos;
        bool valid = false;
        int attempts = 0;
        while (!valid && attempts < 100) {
            pos = sf::Vector2f(distX(gen), distY(gen));
            valid = true;
            for (Vertex* existing : vertices) {
                sf::Vector2f diff = existing->getCenterPos() - pos;
                if (std::sqrt(diff.x * diff.x + diff.y * diff.y) < 70.f) {
                    valid = false;
                    break;
                }
            }
            attempts++;
        }
        createVertex(name, pos);
    }

    for (size_t i = 0; i < names.size(); i++) {
        Vertex* u = getOneVertex(names[i]);
        if (!u) continue;
        
        std::vector<std::pair<float, std::string>> distances;
        for (size_t j = 0; j < names.size(); j++) {
            if (i == j) continue;
            Vertex* v = getOneVertex(names[j]);
            if (!v) continue;
            sf::Vector2f diff = v->getCenterPos() - u->getCenterPos();
            float d = std::sqrt(diff.x * diff.x + diff.y * diff.y);
            distances.push_back({d, names[j]});
        }
        std::sort(distances.begin(), distances.end());
        for (size_t k = 0; k < std::min<size_t>(2, distances.size()); k++) {
            addEdge(names[i], distances[k].second);
        }
    }

    updateEdgeWeights();
}
