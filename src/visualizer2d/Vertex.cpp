#include "Vertex.h"

Vertex::Vertex(std::string vertexName, std::vector<std::pair<std::string, int>> neighbors)
    : vertexName(vertexName), neighbors(neighbors), minDistanceFromSrc(INF), parent(nullptr), state(NodeState::DEFAULT) {
    this->vertexCircle.setRadius(RADIUS);
    this->vertexCircle.setOrigin(RADIUS, RADIUS); // Center origin for smooth positioning
    this->vertexCircle.setOutlineThickness(3.f);
    this->updateVisuals();
}

sf::Vector2f Vertex::getCenterPos() const {
    return this->vertexCircle.getPosition();
}

void Vertex::setCenterPos(sf::Vector2f pos) {
    this->vertexCircle.setPosition(pos);
}

bool Vertex::contains(sf::Vector2f point) const {
    sf::Vector2f diff = point - getCenterPos();
    return (diff.x * diff.x + diff.y * diff.y) <= (RADIUS * RADIUS);
}

void Vertex::setState(NodeState newState) {
    this->state = newState;
    this->updateVisuals();
}

void Vertex::updateVisuals() {
    switch (this->state) {
    case NodeState::START:
        this->vertexCircle.setFillColor(sf::Color(34, 197, 94));    // Green #22c55e
        this->vertexCircle.setOutlineColor(sf::Color(134, 239, 172));
        break;
    case NodeState::END:
        this->vertexCircle.setFillColor(sf::Color(239, 68, 68));     // Red #ef4444
        this->vertexCircle.setOutlineColor(sf::Color(252, 165, 165));
        break;
    case NodeState::CURRENT:
        this->vertexCircle.setFillColor(sf::Color(245, 158, 11));    // Amber #f59e0b
        this->vertexCircle.setOutlineColor(sf::Color(253, 230, 138));
        break;
    case NodeState::VISITED:
        this->vertexCircle.setFillColor(sf::Color(2, 132, 199));     // Blue/Cyan #0284c7
        this->vertexCircle.setOutlineColor(sf::Color(186, 230, 253));
        break;
    case NodeState::PATH:
        this->vertexCircle.setFillColor(sf::Color(16, 185, 129));    // Emerald #10b981
        this->vertexCircle.setOutlineColor(sf::Color(167, 243, 208));
        break;
    case NodeState::DEFAULT:
    default:
        this->vertexCircle.setFillColor(sf::Color(51, 65, 85));      // Slate #334155
        this->vertexCircle.setOutlineColor(sf::Color(148, 163, 184));
        break;
    }
}
