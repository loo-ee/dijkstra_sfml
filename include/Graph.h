#pragma once

#include <iostream>
#include <vector>
#include <stack>
#include <algorithm>
#include <string>

#include "Vertex.h"

class Graph {
public:
    Graph() {}
    ~Graph();

    void addEdge(Vertex* vertex);
    void sort(std::vector<Vertex* >& list);
    Vertex* dijkstra(std::string start, std::string end);

private:
    std::vector<Vertex* > unexplored;
};
