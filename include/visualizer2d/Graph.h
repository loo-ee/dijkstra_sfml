#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <limits>

#include "Vertex.h"

struct DijkstraSnapshot {
    std::string currentNode;
    std::string examiningNeighbor;
    std::string message;
    
    std::unordered_map<std::string, int> distances;
    std::unordered_map<std::string, std::string> parents;
    std::vector<std::string> visitedNodes;
    std::vector<std::pair<std::string, int>> priorityQueue; // (Node, MinDist)
    
    bool isFinished = false;
    bool pathFound = false;
};

class Graph {
public:
    Graph();
    ~Graph();

    void init(Vertex* startVertex, Vertex* endVertex, const std::vector<Vertex*>& allVertices);
    
    bool stepForward();
    bool stepBackward();
    void reset();

    bool isFinished() const;
    bool isPathFound() const;
    size_t getCurrentStepIndex() const { return m_currentStepIndex; }
    size_t getTotalSteps() const { return m_history.size(); }
    
    const DijkstraSnapshot& getCurrentSnapshot() const;
    std::vector<std::string> getShortestPath() const;

private:
    Vertex* m_startVertex = nullptr;
    Vertex* m_endVertex = nullptr;
    std::vector<Vertex*> m_allVertices;

    std::vector<DijkstraSnapshot> m_history;
    size_t m_currentStepIndex = 0;

    void buildFullAlgorithmHistory();
};
