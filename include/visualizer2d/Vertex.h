#pragma once

#ifdef __EMSCRIPTEN__
#include "sfml_web_shim.hpp"
#else
#include <SFML/Graphics.hpp>
#endif
#include <vector>
#include <string>
#include <limits>

#define INF std::numeric_limits<int>::max()

enum class NodeState {
    DEFAULT,
    START,
    END,
    CURRENT,
    VISITED,
    PATH
};

struct Vertex {
    std::string vertexName;
    std::vector<std::pair<std::string, int>> neighbors;

    int minDistanceFromSrc;
    Vertex* parent;

    sf::CircleShape vertexCircle;
    NodeState state;
    static constexpr float RADIUS = 22.f;

    Vertex(std::string vertexName, std::vector<std::pair<std::string, int>> neighbors = {});

    sf::Vector2f getCenterPos() const;
    void setCenterPos(sf::Vector2f pos);
    bool contains(sf::Vector2f point) const;
    void setState(NodeState newState);
    void updateVisuals();
};
