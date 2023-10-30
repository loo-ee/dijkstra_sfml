#pragma once

#include <SFML/Graphics.hpp>
#include <vector>
#include <string>
#include <limits>

#define INF std::numeric_limits<int>::max();

struct Vertex {
    std::string vertexName;
    std::vector<std::pair<std::string, int>> neighbors;

    int vertexWeight;
    int minDistanceFromSrc;

    Vertex* parent;
    sf::RectangleShape vertexRect;

    Vertex(std::string vertexName, std::vector<std::pair<std::string, int>> neighbors);
    void setPosition(sf::Vector2f pos);
    void setDimensions(sf::Vector2f dimensions);
    void setColor(sf::Color color);
};
