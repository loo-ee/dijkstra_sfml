#include "include/Graph.h"

void Graph::addEdge(Vertex* vertex) {
    this->unexplored.push_back(vertex);
}

void Graph::sort(std::vector<Vertex* >& list) {
    for (int i = 0; i < list.size(); i++) {
        for (int k = 0; k < list.size() - 1; k++) {
            if (list.at(k)->minDistanceFromSrc > list.at(k + 1)->minDistanceFromSrc) {
                Vertex* temp = list.at(k);
                list.at(k) = list.at(k + 1);
                list.at(k + 1) = temp;
            }
        }
    }
}

Vertex* Graph::dijkstra(std::string start, std::string end) {
    std::vector<Vertex*> explored;

    auto startElement = std::find_if(this->unexplored.begin(), this->unexplored.end(), [&start](const Vertex* node) {
        return node->vertexName == start;
        });

    std::iter_swap(this->unexplored.begin(), startElement);

    Vertex* firstVertex = this->unexplored.front();
    firstVertex->minDistanceFromSrc = 0;

    while (!this->unexplored.empty()) {
        int exploredNeighborCounter = 0;

        sort(this->unexplored);
        Vertex* topUnex = this->unexplored.front();

        std::cout << "\n[GET LOWEST COST UNVISITED NODE] -> " << topUnex->vertexName << std::endl;
        std::cout << "[FINDING PATH FROM " << topUnex->vertexName << " TO ITS NEIGHBORS]\n";

        explored.push_back(topUnex);
        Vertex* topExplored = explored.back();

        for (Vertex* unexNeighbor : this->unexplored) {
            const std::string& target = unexNeighbor->vertexName;

            auto it = std::find_if(topExplored->neighbors.begin(), topExplored->neighbors.end(), [target](const std::pair<std::string, int>& neighbor) {
                return neighbor.first == target;
                });

            if (it != topExplored->neighbors.end()) {
                float totalDistance = topExplored->minDistanceFromSrc + it->second;
                exploredNeighborCounter++;

                std::cout << "[CALCULATE " << topExplored->vertexName << " -> " << it->first << ": (" << totalDistance;

                if (totalDistance < unexNeighbor->minDistanceFromSrc) {
                    std::cout << " < " << unexNeighbor->minDistanceFromSrc << "), UPDATING " << it->first << " COST]\n";

                    unexNeighbor->parent = topExplored;
                    unexNeighbor->minDistanceFromSrc = totalDistance;
                } else {
                    std::cout << " > " << unexNeighbor->minDistanceFromSrc << "), MAINTAIN " << it->first << " COST]\n";
                }
            }
        }

        if (exploredNeighborCounter == 0) {
            std::cout << "[" << topExplored->vertexName << " HAS NO NEIGHBOR THAT IS UNVISITED]\n";
        }

        this->unexplored.erase(this->unexplored.begin());
    }

    std::cout << "\n[NO UNVISITED VERTICES LEFT]\n";
    auto endElement = std::find_if(explored.begin(), explored.end(), [&end](const Vertex* node) {
        return node->vertexName == end;
        });

    return *endElement;
}

Graph::~Graph() {
    this->unexplored.clear();
}
