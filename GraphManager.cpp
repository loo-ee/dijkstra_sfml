#include "include/GraphManager.h"

void GraphManager::sort(std::vector<Vertex* >& list) {
    for (size_t i = 0; i < list.size(); i++) {
        for (size_t k = 0; k < list.size() - 1; k++) {
            if (list.at(k)->minDistanceFromSrc > list.at(k + 1)->minDistanceFromSrc) {
                Vertex* temp = list.at(k);
                list.at(k) = list.at(k + 1);
                list.at(k + 1) = temp;
            }
        }
    }
}

void GraphManager::createVertex(
        std::string vertex, 
        std::vector<std::pair<std::string, int>> neighbors,
        float posX,
        float posY
) {
    const int vertexWidth = 20;
    const int vertexHeight = 20;
    Vertex* createdVertex = new Vertex(vertex, neighbors);

    createdVertex->setPosition(sf::Vector2f(posX, posY));
    createdVertex->setDimensions(sf::Vector2f(vertexWidth, vertexHeight));

    this->vertices.push_back(createdVertex);
}

void GraphManager::renderText(
    std::string label,
    sf::Vector2f pos,
    sf::Font& font, int size,
    sf::Color color,
    sf::RenderWindow& window
) {
    sf::Text rectText;
    rectText.setPosition(pos);
    rectText.setFont(font);
    rectText.setString(label);
    rectText.setCharacterSize(size);
    rectText.setFillColor(color);

    window.draw(rectText);
}

Vertex* GraphManager::getOneVertex(std::string vertexName) {
    auto foundVertex = std::find_if(this->vertices.begin(), this->vertices.end(), [vertexName](const Vertex* currentVertex) {
        return currentVertex->vertexName == vertexName;
        });

    if (foundVertex != this->vertices.end())
        return *foundVertex;
    else
        return NULL;
}

std::vector<Vertex* >* GraphManager::getVertices() {
    return &this->vertices;
}

void GraphManager::resetParentVertices() {
    for (size_t i = 0; i < this->vertices.size(); i++) {
        this->vertices.at(i)->parent = nullptr;
    }
}

void GraphManager::resetMinDistancesFromSrc() {
    for (size_t i = 0; i < this->vertices.size(); i++) {
        this->vertices.at(i)->minDistanceFromSrc = INF;
    }
}

void GraphManager::clearVertices() {
    for (int i = 0; i < this->vertices.size(); i++) {
        delete this->vertices.at(i);
    }

    this->vertices.clear();
}

GraphManager::~GraphManager() {
    for (int i = 0; i < this->vertices.size(); i++) {
        delete this->vertices.at(i);
    }

    this->vertices.clear();
}
