#pragma once

#ifdef __EMSCRIPTEN__
#include "sfml_web_shim.hpp"
#else
#include <SFML/Graphics.hpp>
#endif
#include <vector>
#include <string>
#include <algorithm>
#include <utility>

#include "Vertex.h"

enum class EdgeDirection {
    FORWARD,   // u -> v
    BACKWARD,  // v -> u
    BOTH,      // u <-> v
    NONE
};

struct EdgeInfo {
    std::string u;
    std::string v;
    int weight;
    EdgeDirection direction;
};

class GraphManager {
public:
    GraphManager();
    ~GraphManager();

    Vertex* createVertex(const std::string& name, sf::Vector2f centerPos, const std::vector<std::pair<std::string, int>>& neighbors = {});
    Vertex* spawnVertexAt(sf::Vector2f centerPos);
    
    void removeVertex(const std::string& name);
    bool addEdge(const std::string& u, const std::string& v);
    bool removeEdge(const std::string& u, const std::string& v);
    bool hasEdge(const std::string& u, const std::string& v) const;

    EdgeDirection getEdgeDirection(const std::string& u, const std::string& v) const;
    void cycleEdgeDirection(const std::string& u, const std::string& v);
    void setEdgeDirection(const std::string& u, const std::string& v, EdgeDirection dir);

    Vertex* getOneVertex(const std::string& vertexName);
    Vertex* getVertexAt(sf::Vector2f pos);
    std::pair<std::string, std::string> getEdgeAt(sf::Vector2f pos, float threshold = 8.f);

    std::vector<Vertex*>& getVertices();
    const std::vector<Vertex*>& getVertices() const;

    void updateEdgeWeights();
    void resetGraphStates(Vertex* start = nullptr, Vertex* end = nullptr);
    void clearVertices();

    // Preset Graphs
    void loadDefaultPreset();
    void loadGridPreset();
    void loadRandomPreset();

    std::string generateNextVertexName();

private:
    std::vector<Vertex*> vertices;
    int nameCounter = 0;
};
