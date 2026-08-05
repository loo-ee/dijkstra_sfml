#ifdef __EMSCRIPTEN__
#include "include/sfml_web_shim.hpp"
#else
#include <SFML/Graphics.hpp>
#endif
#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <functional>
#include <algorithm>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include "include/GraphManager.h"
#include "include/Graph.h"
#include "include/Button.h"

enum class InteractionMode {
    MOVE,
    ADD_NODE,
    ADD_EDGE,
    SET_START,
    SET_END,
    SET_DIRECTION,
    DELETE_ITEM
};

// Clean Color Palette
const sf::Color COLOR_BG(15, 23, 42);        // #0f172a
const sf::Color COLOR_SIDEBAR(30, 41, 59);   // #1e293b
const sf::Color COLOR_CANVAS(2, 6, 23);       // #020617
const sf::Color COLOR_BORDER(71, 85, 105);    // #475569
const sf::Color COLOR_TEXT(241, 245, 249);    // #f1f5f9
const sf::Color COLOR_MUTED(148, 163, 184);  // #94a3b8

const sf::Color COLOR_EDGE_DEFAULT(71, 85, 105);  // #475569
const sf::Color COLOR_EDGE_ACTIVE(245, 158, 11);  // Amber #f59e0b
const sf::Color COLOR_EDGE_PATH(16, 185, 129);    // Emerald #10b981

void drawThickLine(sf::RenderWindow& window, sf::Vector2f point1, sf::Vector2f point2, float thickness, sf::Color color) {
    sf::Vector2f dir = point2 - point1;
    float length = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    if (length == 0.f) return;

    sf::Vector2f unitDir = dir / length;
    sf::Vector2f normal(-unitDir.y, unitDir.x);
    sf::Vector2f offset = normal * (thickness / 2.f);

    sf::VertexArray quad(sf::Quads, 4);
    quad[0].position = point1 + offset;
    quad[1].position = point2 + offset;
    quad[2].position = point2 - offset;
    quad[3].position = point1 - offset;

    for (int i = 0; i < 4; i++) {
        quad[i].color = color;
    }

    window.draw(quad);
}

void drawArrowHead(sf::RenderWindow& window, sf::Vector2f fromPos, sf::Vector2f toPos, sf::Color color, float size = 12.f) {
    sf::Vector2f dir = toPos - fromPos;
    float length = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    if (length == 0.f) return;

    sf::Vector2f unitDir = dir / length;
    sf::Vector2f tip = toPos - unitDir * (Vertex::RADIUS + 2.f);

    sf::Vector2f normal(-unitDir.y, unitDir.x);
    sf::Vector2f base = tip - unitDir * size;

    sf::VertexArray triangle(sf::Triangles, 3);
    triangle[0].position = tip;
    triangle[1].position = base + normal * (size * 0.55f);
    triangle[2].position = base - normal * (size * 0.55f);

    for (int i = 0; i < 3; i++) {
        triangle[i].color = color;
    }

    window.draw(triangle);
}

void drawEdgeWeightBadge(sf::RenderWindow& window, sf::Vector2f pos, int weight, const sf::Font& font, sf::Color badgeBgColor = sf::Color(30, 41, 59)) {
    std::string textStr = std::to_string(weight);
    sf::Text text;
    text.setFont(font);
    text.setString(textStr);
    text.setCharacterSize(14);
    text.setFillColor(COLOR_TEXT);

    sf::FloatRect textBounds = text.getLocalBounds();
    float paddingX = 8.f;
    float paddingY = 4.f;

    sf::RectangleShape badge;
    badge.setSize(sf::Vector2f(textBounds.width + paddingX * 2.f, textBounds.height + paddingY * 2.f));
    badge.setOrigin(std::floor(badge.getSize().x / 2.f), std::floor(badge.getSize().y / 2.f));
    badge.setPosition(std::floor(pos.x), std::floor(pos.y));
    badge.setFillColor(badgeBgColor);
    badge.setOutlineThickness(1.f);
    badge.setOutlineColor(COLOR_BORDER);

    text.setOrigin(std::floor(textBounds.left + textBounds.width / 2.f), std::floor(textBounds.top + textBounds.height / 2.f));
    text.setPosition(std::floor(pos.x), std::floor(pos.y));

    window.draw(badge);
    window.draw(text);
}

struct AppState {
    unsigned int windowWidth = 1280;
    unsigned int windowHeight = 720;
    const float SIDEBAR_WIDTH = 260.f;
    const float INSPECTOR_WIDTH = 280.f;
    const float PLAYBAR_HEIGHT = 55.f;

    bool isFullscreen = false;
    sf::ContextSettings settings;
    sf::RenderWindow window;

    sf::Font font;
    GraphManager manager;
    Graph solver;

    Vertex* startVertex = nullptr;
    Vertex* endVertex = nullptr;

    InteractionMode currentMode = InteractionMode::MOVE;

    bool isPlaying = false;
    float stepDelay = 0.5f;
    sf::Clock autoStepClock;
    sf::Clock doubleClickClock;

    Vertex* draggedVertex = nullptr;
    Vertex* edgeSourceVertex = nullptr;

    sf::View uiView;
    sf::View canvasView;
    float zoomLevel = 1.0f;
    bool isPanningCanvas = false;
    sf::Vector2i panStartPixelPos;
    sf::Vector2f panStartCenter;

    // UI Buttons
    Button btnMove{sf::Vector2f(15.f, 45.f), sf::Vector2f(230.f, 32.f)};
    Button btnAddNode{sf::Vector2f(15.f, 82.f), sf::Vector2f(230.f, 32.f)};
    Button btnAddEdge{sf::Vector2f(15.f, 119.f), sf::Vector2f(230.f, 32.f)};
    Button btnSetStart{sf::Vector2f(15.f, 156.f), sf::Vector2f(230.f, 32.f)};
    Button btnSetEnd{sf::Vector2f(15.f, 193.f), sf::Vector2f(230.f, 32.f)};
    Button btnSetDirection{sf::Vector2f(15.f, 230.f), sf::Vector2f(230.f, 32.f)};
    Button btnDelete{sf::Vector2f(15.f, 267.f), sf::Vector2f(230.f, 32.f)};

    std::vector<Button*> modeButtons;

    Button btnPresetDefault{sf::Vector2f(15.f, 335.f), sf::Vector2f(230.f, 32.f)};
    Button btnPresetGrid{sf::Vector2f(15.f, 372.f), sf::Vector2f(230.f, 32.f)};
    Button btnPresetRandom{sf::Vector2f(15.f, 409.f), sf::Vector2f(230.f, 32.f)};
    Button btnPresetClear{sf::Vector2f(15.f, 446.f), sf::Vector2f(230.f, 32.f)};

    Button btnResetView{sf::Vector2f(15.f, 505.f), sf::Vector2f(110.f, 32.f)};
    Button btnFullscreen{sf::Vector2f(135.f, 505.f), sf::Vector2f(110.f, 32.f)};
    Button btnSolve{sf::Vector2f(15.f, 570.f), sf::Vector2f(230.f, 45.f)};

    Button btnStepBack{sf::Vector2f(0.f, 0.f), sf::Vector2f(75.f, 34.f)};
    Button btnPlayPause{sf::Vector2f(0.f, 0.f), sf::Vector2f(90.f, 34.f)};
    Button btnStepFwd{sf::Vector2f(0.f, 0.f), sf::Vector2f(75.f, 34.f)};
    Button btnReset{sf::Vector2f(0.f, 0.f), sf::Vector2f(75.f, 34.f)};

    Button btnSpeed1{sf::Vector2f(0.f, 0.f), sf::Vector2f(45.f, 34.f)};
    Button btnSpeed2{sf::Vector2f(0.f, 0.f), sf::Vector2f(45.f, 34.f)};
    Button btnSpeed5{sf::Vector2f(0.f, 0.f), sf::Vector2f(45.f, 34.f)};

    void init() {
        settings.antialiasingLevel = 8;
        window.create(sf::VideoMode(windowWidth, windowHeight), "Dijkstra Visualizer - SFML", sf::Style::Default, settings);
        window.setFramerateLimit(60);

        if (!font.loadFromFile("fonts/Meslo/Meslo LG L Bold Nerd Font Complete.ttf")) {
            std::cerr << "Warning: Could not load custom font.\n";
        }

        for (unsigned int sz : {11, 12, 13, 14, 15, 16}) {
            const_cast<sf::Texture&>(font.getTexture(sz)).setSmooth(false);
        }

        manager.loadDefaultPreset();
        startVertex = manager.getOneVertex("A");
        endVertex = manager.getOneVertex("H");

        btnMove.setButtonText(font, "1. Select / Move", 13);
        btnAddNode.setButtonText(font, "2. Add Node", 13);
        btnAddEdge.setButtonText(font, "3. Add Edge", 13);
        btnSetStart.setButtonText(font, "4. Set Start Node", 13);
        btnSetEnd.setButtonText(font, "5. Set End Node", 13);
        btnSetDirection.setButtonText(font, "6. Edge Direction", 13);
        btnDelete.setButtonText(font, "7. Delete Item", 13);

        btnMove.setCallback([this]() { currentMode = InteractionMode::MOVE; });
        btnAddNode.setCallback([this]() { currentMode = InteractionMode::ADD_NODE; });
        btnAddEdge.setCallback([this]() { currentMode = InteractionMode::ADD_EDGE; });
        btnSetStart.setCallback([this]() { currentMode = InteractionMode::SET_START; });
        btnSetEnd.setCallback([this]() { currentMode = InteractionMode::SET_END; });
        btnSetDirection.setCallback([this]() { currentMode = InteractionMode::SET_DIRECTION; });
        btnDelete.setCallback([this]() { currentMode = InteractionMode::DELETE_ITEM; });

        modeButtons.push_back(&btnMove);
        modeButtons.push_back(&btnAddNode);
        modeButtons.push_back(&btnAddEdge);
        modeButtons.push_back(&btnSetStart);
        modeButtons.push_back(&btnSetEnd);
        modeButtons.push_back(&btnSetDirection);
        modeButtons.push_back(&btnDelete);

        btnPresetDefault.setButtonText(font, "Default Graph", 13);
        btnPresetGrid.setButtonText(font, "Grid Mesh", 13);
        btnPresetRandom.setButtonText(font, "Random Graph", 13);
        btnPresetClear.setButtonText(font, "Clear Canvas", 13);
        btnPresetClear.setColors(sf::Color(153, 27, 27), sf::Color(185, 28, 28), COLOR_TEXT);

        btnResetView.setButtonText(font, "Reset View", 12);
        btnResetView.setCallback([this]() { resetView(); });

        btnFullscreen.setButtonText(font, "Fullscreen", 12);
        btnFullscreen.setCallback([this]() { toggleFullscreen(); });

        btnSolve.setButtonText(font, "RUN DIJKSTRA", 16);
        btnSolve.setColors(sf::Color(16, 185, 129), sf::Color(5, 150, 105), sf::Color::White);

        btnPresetDefault.setCallback([this]() {
            manager.loadDefaultPreset();
            startVertex = manager.getOneVertex("A");
            endVertex = manager.getOneVertex("H");
            resetSolver();
            resetView();
        });

        btnPresetGrid.setCallback([this]() {
            manager.loadGridPreset();
            auto& verts = manager.getVertices();
            if (!verts.empty()) {
                startVertex = verts.front();
                endVertex = verts.back();
            } else {
                startVertex = endVertex = nullptr;
            }
            resetSolver();
            resetView();
        });

        btnPresetRandom.setCallback([this]() {
            manager.loadRandomPreset();
            auto& verts = manager.getVertices();
            if (!verts.empty()) {
                startVertex = verts.front();
                endVertex = verts.back();
            } else {
                startVertex = endVertex = nullptr;
            }
            resetSolver();
            resetView();
        });

        btnPresetClear.setCallback([this]() {
            manager.clearVertices();
            startVertex = endVertex = nullptr;
            resetSolver();
        });

        btnSolve.setCallback([this]() {
            resetSolver();
            isPlaying = true;
        });

        btnStepBack.setButtonText(font, "|< Back", 12);
        btnPlayPause.setButtonText(font, "Play", 13);
        btnPlayPause.setColors(sf::Color(14, 165, 233), sf::Color(56, 189, 248), sf::Color::White);
        btnStepFwd.setButtonText(font, "Fwd >|", 12);
        btnReset.setButtonText(font, "Reset", 12);

        btnSpeed1.setButtonText(font, "1x", 12);
        btnSpeed1.setActive(true);
        btnSpeed2.setButtonText(font, "2x", 12);
        btnSpeed5.setButtonText(font, "5x", 12);

        btnStepBack.setCallback([this]() { isPlaying = false; solver.stepBackward(); });
        btnStepFwd.setCallback([this]() { isPlaying = false; solver.stepForward(); });
        btnPlayPause.setCallback([this]() { isPlaying = !isPlaying; });
        btnReset.setCallback([this]() { isPlaying = false; solver.reset(); });

        btnSpeed1.setCallback([this]() { stepDelay = 0.6f; btnSpeed1.setActive(true); btnSpeed2.setActive(false); btnSpeed5.setActive(false); });
        btnSpeed2.setCallback([this]() { stepDelay = 0.25f; btnSpeed1.setActive(false); btnSpeed2.setActive(true); btnSpeed5.setActive(false); });
        btnSpeed5.setCallback([this]() { stepDelay = 0.08f; btnSpeed1.setActive(false); btnSpeed2.setActive(false); btnSpeed5.setActive(true); });

        updateCanvasViewport();
        repositionPlaybar();
        resetSolver();
    }

    void updateCanvasViewport() {
        windowWidth = window.getSize().x;
        windowHeight = window.getSize().y;

        uiView.setSize(static_cast<float>(windowWidth), static_cast<float>(windowHeight));
        uiView.setCenter(static_cast<float>(windowWidth) / 2.f, static_cast<float>(windowHeight) / 2.f);

        float canvasWidth = std::max(100.f, windowWidth - SIDEBAR_WIDTH - INSPECTOR_WIDTH);
        float canvasHeight = std::max(100.f, windowHeight - PLAYBAR_HEIGHT);

        sf::FloatRect viewport(
            SIDEBAR_WIDTH / static_cast<float>(windowWidth),
            0.f,
            canvasWidth / static_cast<float>(windowWidth),
            canvasHeight / static_cast<float>(windowHeight)
        );

        sf::Vector2f center = canvasView.getCenter();
        if (center.x == 0.f && center.y == 0.f) {
            center = sf::Vector2f(SIDEBAR_WIDTH + canvasWidth / 2.f, canvasHeight / 2.f);
        }

        canvasView.setSize(canvasWidth * zoomLevel, canvasHeight * zoomLevel);
        canvasView.setCenter(center);
        canvasView.setViewport(viewport);
    }

    void resetView() {
        zoomLevel = 1.0f;
        float canvasWidth = std::max(100.f, window.getSize().x - SIDEBAR_WIDTH - INSPECTOR_WIDTH);
        float canvasHeight = std::max(100.f, window.getSize().y - PLAYBAR_HEIGHT);
        canvasView.setCenter(SIDEBAR_WIDTH + canvasWidth / 2.f, canvasHeight / 2.f);
        updateCanvasViewport();
    }

    void toggleFullscreen() {
        isFullscreen = !isFullscreen;
        if (isFullscreen) {
            window.create(sf::VideoMode::getDesktopMode(), "Dijkstra Visualizer - SFML", sf::Style::Fullscreen, settings);
        } else {
            window.create(sf::VideoMode(1280, 720), "Dijkstra Visualizer - SFML", sf::Style::Default, settings);
        }
        window.setFramerateLimit(60);
        updateCanvasViewport();
    }

    void resetSolver() {
        isPlaying = false;
        if (startVertex && endVertex) {
            manager.resetGraphStates(startVertex, endVertex);
            solver.init(startVertex, endVertex, manager.getVertices());
        }
    }

    void repositionPlaybar() {
        float py = window.getSize().y - PLAYBAR_HEIGHT + 10.f;
        float px = SIDEBAR_WIDTH + 15.f;

        btnStepBack.setPosition(sf::Vector2f(px, py));
        btnPlayPause.setPosition(sf::Vector2f(px + 82.f, py));
        btnStepFwd.setPosition(sf::Vector2f(px + 179.f, py));
        btnReset.setPosition(sf::Vector2f(px + 261.f, py));

        btnSpeed1.setPosition(sf::Vector2f(px + 370.f, py));
        btnSpeed2.setPosition(sf::Vector2f(px + 420.f, py));
        btnSpeed5.setPosition(sf::Vector2f(px + 470.f, py));
    }

    void updateAndRender() {
        if (!window.isOpen()) return;

        sf::Vector2i mousePixelPos = sf::Mouse::getPosition(window);
        sf::Vector2f uiMousePos = window.mapPixelToCoords(mousePixelPos, uiView);
        sf::Vector2f canvasMousePos = window.mapPixelToCoords(mousePixelPos, canvasView);

        float currentCanvasWidth = std::max(100.f, window.getSize().x - SIDEBAR_WIDTH - INSPECTOR_WIDTH);
        float currentCanvasHeight = std::max(100.f, window.getSize().y - PLAYBAR_HEIGHT);
        sf::FloatRect canvasPixelArea(SIDEBAR_WIDTH, 0.f, currentCanvasWidth, currentCanvasHeight);

        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }

            if (event.type == sf::Event::Resized) {
                updateCanvasViewport();
                repositionPlaybar();
            }

            // Zooming via Mouse Wheel
            if (event.type == sf::Event::MouseWheelScrolled) {
                if (canvasPixelArea.contains(static_cast<float>(mousePixelPos.x), static_cast<float>(mousePixelPos.y))) {
                    if (event.mouseWheelScroll.delta > 0 && zoomLevel > 0.25f) {
                        zoomLevel *= 0.9f;
                    } else if (event.mouseWheelScroll.delta < 0 && zoomLevel < 4.0f) {
                        zoomLevel *= 1.1111f;
                    }
                    updateCanvasViewport();
                }
            }

            // Mouse Pressed Events
            if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Middle ||
                   (event.mouseButton.button == sf::Mouse::Right && currentMode == InteractionMode::MOVE && !manager.getVertexAt(canvasMousePos)) ||
                   (event.mouseButton.button == sf::Mouse::Left && sf::Keyboard::isKeyPressed(sf::Keyboard::Space))) {
                    isPanningCanvas = true;
                    panStartPixelPos = mousePixelPos;
                    panStartCenter = canvasView.getCenter();
                } else if (event.mouseButton.button == sf::Mouse::Left && canvasPixelArea.contains(static_cast<float>(mousePixelPos.x), static_cast<float>(mousePixelPos.y))) {
                    Vertex* hitVertex = manager.getVertexAt(canvasMousePos);

                    if (doubleClickClock.getElapsedTime().asMilliseconds() < 300) {
                        Vertex* spawned = manager.spawnVertexAt(canvasMousePos);
                        if (!startVertex) startVertex = spawned;
                        else if (!endVertex && endVertex != startVertex) endVertex = spawned;
                        resetSolver();
                    } else {
                        doubleClickClock.restart();

                        switch (currentMode) {
                        case InteractionMode::MOVE:
                            if (hitVertex) {
                                draggedVertex = hitVertex;
                            } else {
                                isPanningCanvas = true;
                                panStartPixelPos = mousePixelPos;
                                panStartCenter = canvasView.getCenter();
                            }
                            break;
                        case InteractionMode::ADD_NODE:
                            if (!hitVertex) {
                                Vertex* spawned = manager.spawnVertexAt(canvasMousePos);
                                if (!startVertex) startVertex = spawned;
                                resetSolver();
                            }
                            break;
                        case InteractionMode::ADD_EDGE:
                            if (hitVertex) {
                                edgeSourceVertex = hitVertex;
                            }
                            break;
                        case InteractionMode::SET_START:
                            if (hitVertex) {
                                startVertex = hitVertex;
                                if (endVertex == startVertex) endVertex = nullptr;
                                resetSolver();
                            }
                            break;
                        case InteractionMode::SET_END:
                            if (hitVertex) {
                                endVertex = hitVertex;
                                if (startVertex == endVertex) startVertex = nullptr;
                                resetSolver();
                            }
                            break;
                        case InteractionMode::SET_DIRECTION:
                            {
                                auto edge = manager.getEdgeAt(canvasMousePos);
                                if (edge.first != "") {
                                    manager.cycleEdgeDirection(edge.first, edge.second);
                                    resetSolver();
                                }
                            }
                            break;
                        case InteractionMode::DELETE_ITEM:
                            if (hitVertex) {
                                if (hitVertex == startVertex) startVertex = nullptr;
                                if (hitVertex == endVertex) endVertex = nullptr;
                                manager.removeVertex(hitVertex->vertexName);
                                resetSolver();
                            } else {
                                auto edge = manager.getEdgeAt(canvasMousePos);
                                if (edge.first != "") {
                                    manager.removeEdge(edge.first, edge.second);
                                    resetSolver();
                                }
                            }
                            break;
                        }
                    }
                } else if (event.mouseButton.button == sf::Mouse::Right && canvasPixelArea.contains(static_cast<float>(mousePixelPos.x), static_cast<float>(mousePixelPos.y))) {
                    Vertex* hitVertex = manager.getVertexAt(canvasMousePos);
                    if (hitVertex) {
                        if (hitVertex == startVertex) startVertex = nullptr;
                        if (hitVertex == endVertex) endVertex = nullptr;
                        manager.removeVertex(hitVertex->vertexName);
                        resetSolver();
                    } else {
                        auto edge = manager.getEdgeAt(canvasMousePos);
                        if (edge.first != "") {
                            manager.cycleEdgeDirection(edge.first, edge.second);
                            resetSolver();
                        }
                    }
                }
            }

            if (event.type == sf::Event::MouseButtonReleased) {
                if (event.mouseButton.button == sf::Mouse::Middle || event.mouseButton.button == sf::Mouse::Right || event.mouseButton.button == sf::Mouse::Left) {
                    isPanningCanvas = false;
                }
                if (event.mouseButton.button == sf::Mouse::Left) {
                    if (draggedVertex) {
                        draggedVertex = nullptr;
                        manager.updateEdgeWeights();
                        resetSolver();
                    }
                    if (edgeSourceVertex) {
                        Vertex* targetVertex = manager.getVertexAt(canvasMousePos);
                        if (targetVertex && targetVertex != edgeSourceVertex) {
                            manager.addEdge(edgeSourceVertex->vertexName, targetVertex->vertexName);
                            resetSolver();
                        }
                        edgeSourceVertex = nullptr;
                    }
                }
            }

            if (event.type == sf::Event::MouseMoved) {
                if (isPanningCanvas) {
                    sf::Vector2i pixelDelta = mousePixelPos - panStartPixelPos;
                    sf::Vector2f worldDelta(
                        pixelDelta.x * (canvasView.getSize().x / currentCanvasWidth),
                        pixelDelta.y * (canvasView.getSize().y / currentCanvasHeight)
                    );
                    canvasView.setCenter(panStartCenter - worldDelta);
                } else if (draggedVertex) {
                    draggedVertex->setCenterPos(canvasMousePos);
                    manager.updateEdgeWeights();
                }
            }

            // Keyboard Shortcuts
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Space) {
                    isPlaying = !isPlaying;
                } else if (event.key.code == sf::Keyboard::Right) {
                    isPlaying = false;
                    solver.stepForward();
                } else if (event.key.code == sf::Keyboard::Left) {
                    isPlaying = false;
                    solver.stepBackward();
                } else if (event.key.code == sf::Keyboard::R) {
                    isPlaying = false;
                    solver.reset();
                } else if (event.key.code == sf::Keyboard::F11 || event.key.code == sf::Keyboard::F) {
                    toggleFullscreen();
                } else if (event.key.code == sf::Keyboard::Home || event.key.code == sf::Keyboard::Num0) {
                    resetView();
                }
            }

            for (Button* btn : modeButtons) btn->handleEvent(event, window);
            btnPresetDefault.handleEvent(event, window);
            btnPresetGrid.handleEvent(event, window);
            btnPresetRandom.handleEvent(event, window);
            btnPresetClear.handleEvent(event, window);
            btnResetView.handleEvent(event, window);
            btnFullscreen.handleEvent(event, window);
            btnSolve.handleEvent(event, window);

            btnStepBack.handleEvent(event, window);
            btnPlayPause.handleEvent(event, window);
            btnStepFwd.handleEvent(event, window);
            btnReset.handleEvent(event, window);

            btnSpeed1.handleEvent(event, window);
            btnSpeed2.handleEvent(event, window);
            btnSpeed5.handleEvent(event, window);
        }

        // Auto stepping logic
        if (isPlaying) {
            btnPlayPause.setButtonText(font, "Pause", 13);
            btnPlayPause.setColors(sf::Color(234, 88, 12), sf::Color(249, 115, 22), sf::Color::White);
            if (autoStepClock.getElapsedTime().asSeconds() >= stepDelay) {
                bool stepped = solver.stepForward();
                autoStepClock.restart();
                if (!stepped || solver.isFinished()) {
                    isPlaying = false;
                }
            }
        } else {
            btnPlayPause.setButtonText(font, "Play", 13);
            btnPlayPause.setColors(sf::Color(14, 165, 233), sf::Color(56, 189, 248), sf::Color::White);
        }

        // Update UI Button hover/active states
        for (size_t i = 0; i < modeButtons.size(); i++) {
            modeButtons[i]->setActive(static_cast<int>(currentMode) == static_cast<int>(i));
            modeButtons[i]->update(uiMousePos);
        }
        btnPresetDefault.update(uiMousePos);
        btnPresetGrid.update(uiMousePos);
        btnPresetRandom.update(uiMousePos);
        btnPresetClear.update(uiMousePos);
        btnResetView.update(uiMousePos);
        btnFullscreen.update(uiMousePos);
        btnSolve.update(uiMousePos);

        btnStepBack.update(uiMousePos);
        btnPlayPause.update(uiMousePos);
        btnStepFwd.update(uiMousePos);
        btnReset.update(uiMousePos);

        btnSpeed1.update(uiMousePos);
        btnSpeed2.update(uiMousePos);
        btnSpeed5.update(uiMousePos);

        // Update node visual states
        const DijkstraSnapshot& snapshot = solver.getCurrentSnapshot();
        std::vector<std::string> shortestPath = solver.getShortestPath();

        for (Vertex* v : manager.getVertices()) {
            if (v == startVertex) {
                v->setState(NodeState::START);
            } else if (v == endVertex) {
                v->setState(NodeState::END);
            } else if (std::find(shortestPath.begin(), shortestPath.end(), v->vertexName) != shortestPath.end()) {
                v->setState(NodeState::PATH);
            } else if (v->vertexName == snapshot.currentNode) {
                v->setState(NodeState::CURRENT);
            } else if (std::find(snapshot.visitedNodes.begin(), snapshot.visitedNodes.end(), v->vertexName) != snapshot.visitedNodes.end()) {
                v->setState(NodeState::VISITED);
            } else {
                v->setState(NodeState::DEFAULT);
            }
        }

        // --- RENDER PASS ---
        window.clear(COLOR_BG);

        // A. RENDER CANVAS SCENE (World Space using canvasView)
        window.setView(canvasView);

        // Draw Canvas Background rect
        sf::RectangleShape canvasWorldBg(sf::Vector2f(4000.f, 4000.f));
        canvasWorldBg.setOrigin(2000.f, 2000.f);
        canvasWorldBg.setPosition(canvasView.getCenter());
        canvasWorldBg.setFillColor(COLOR_CANVAS);
        window.draw(canvasWorldBg);

        // Draw Canvas Grid Lines
        sf::VertexArray gridLines(sf::Lines);
        const float gridSpacing = 50.f;
        sf::Vector2f center = canvasView.getCenter();
        sf::Vector2f size = canvasView.getSize();
        float left = center.x - size.x / 2.f - gridSpacing;
        float right = center.x + size.x / 2.f + gridSpacing;
        float top = center.y - size.y / 2.f - gridSpacing;
        float bottom = center.y + size.y / 2.f + gridSpacing;

        float startX = std::floor(left / gridSpacing) * gridSpacing;
        for (float x = startX; x <= right; x += gridSpacing) {
            gridLines.append(sf::Vertex(sf::Vector2f(x, top), sf::Color(30, 41, 59, 100)));
            gridLines.append(sf::Vertex(sf::Vector2f(x, bottom), sf::Color(30, 41, 59, 100)));
        }

        float startY = std::floor(top / gridSpacing) * gridSpacing;
        for (float y = startY; y <= bottom; y += gridSpacing) {
            gridLines.append(sf::Vertex(sf::Vector2f(left, y), sf::Color(30, 41, 59, 100)));
            gridLines.append(sf::Vertex(sf::Vector2f(right, y), sf::Color(30, 41, 59, 100)));
        }
        window.draw(gridLines);

        // Draw Edges & Directed Arrows
        std::vector<Vertex*>& allVertices = manager.getVertices();
        for (Vertex* u : allVertices) {
            sf::Vector2f uPos = u->getCenterPos();
            for (const auto& neighbor : u->neighbors) {
                Vertex* v = manager.getOneVertex(neighbor.first);
                if (!v) continue;

                sf::Vector2f vPos = v->getCenterPos();
                EdgeDirection dir = manager.getEdgeDirection(u->vertexName, v->vertexName);

                if (dir == EdgeDirection::BOTH && u->vertexName > v->vertexName) {
                    continue;
                }

                sf::Color edgeColor = COLOR_EDGE_DEFAULT;
                float lineThickness = 3.f;

                bool inPath = false;
                for (size_t i = 0; i + 1 < shortestPath.size(); i++) {
                    if ((shortestPath[i] == u->vertexName && shortestPath[i + 1] == v->vertexName) ||
                        (dir == EdgeDirection::BOTH && shortestPath[i] == v->vertexName && shortestPath[i + 1] == u->vertexName)) {
                        inPath = true;
                        break;
                    }
                }

                if (inPath) {
                    edgeColor = COLOR_EDGE_PATH;
                    lineThickness = 5.f;
                } else if ((u->vertexName == snapshot.currentNode && v->vertexName == snapshot.examiningNeighbor) ||
                           (dir == EdgeDirection::BOTH && v->vertexName == snapshot.currentNode && u->vertexName == snapshot.examiningNeighbor)) {
                    edgeColor = COLOR_EDGE_ACTIVE;
                    lineThickness = 5.f;
                }

                drawThickLine(window, uPos, vPos, lineThickness, edgeColor);

                if (dir == EdgeDirection::FORWARD) {
                    drawArrowHead(window, uPos, vPos, edgeColor);
                } else if (dir == EdgeDirection::BACKWARD) {
                    drawArrowHead(window, vPos, uPos, edgeColor);
                } else if (dir == EdgeDirection::BOTH) {
                    drawArrowHead(window, uPos, vPos, edgeColor);
                    drawArrowHead(window, vPos, uPos, edgeColor);
                }

                sf::Vector2f midPos = uPos + (vPos - uPos) / 2.f;
                sf::Color badgeBg = (edgeColor == COLOR_EDGE_PATH) ? sf::Color(6, 78, 59) :
                                    (edgeColor == COLOR_EDGE_ACTIVE) ? sf::Color(120, 53, 15) : sf::Color(30, 41, 59);
                drawEdgeWeightBadge(window, midPos, neighbor.second, font, badgeBg);
            }
        }

        // Draw Rubber-Band Line
        if (currentMode == InteractionMode::ADD_EDGE && edgeSourceVertex) {
            drawThickLine(window, edgeSourceVertex->getCenterPos(), canvasMousePos, 3.f, COLOR_EDGE_ACTIVE);
            drawArrowHead(window, edgeSourceVertex->getCenterPos(), canvasMousePos, COLOR_EDGE_ACTIVE);
        }

        // Draw Vertices & Labels
        for (Vertex* v : allVertices) {
            window.draw(v->vertexCircle);

            sf::Text text;
            text.setFont(font);
            text.setString(v->vertexName);
            text.setCharacterSize(16);
            text.setFillColor(sf::Color::White);
            sf::FloatRect textBounds = text.getLocalBounds();
            text.setOrigin(std::floor(textBounds.left + textBounds.width / 2.f), std::floor(textBounds.top + textBounds.height / 2.f));
            text.setPosition(std::floor(v->getCenterPos().x), std::floor(v->getCenterPos().y));
            window.draw(text);

            auto distIt = snapshot.distances.find(v->vertexName);
            if (distIt != snapshot.distances.end() && distIt->second != INF) {
                sf::Text distText;
                distText.setFont(font);
                distText.setString("d=" + std::to_string(distIt->second));
                distText.setCharacterSize(12);
                distText.setFillColor(sf::Color(226, 232, 240));
                sf::FloatRect dBounds = distText.getLocalBounds();
                distText.setOrigin(std::floor(dBounds.left + dBounds.width / 2.f), std::floor(dBounds.top + dBounds.height / 2.f));
                distText.setPosition(std::floor(v->getCenterPos().x), std::floor(v->getCenterPos().y + Vertex::RADIUS + 12.f));
                window.draw(distText);
            }
        }

        // B. RENDER UI OVERLAY SCENE (Screen Space using uiView matched 1:1 to window size)
        window.setView(uiView);

        // Left Sidebar Background
        sf::RectangleShape sidebarBg(sf::Vector2f(SIDEBAR_WIDTH, static_cast<float>(windowHeight)));
        sidebarBg.setPosition(0.f, 0.f);
        sidebarBg.setFillColor(COLOR_SIDEBAR);
        window.draw(sidebarBg);

        sf::RectangleShape sidebarDivider(sf::Vector2f(2.f, static_cast<float>(windowHeight)));
        sidebarDivider.setPosition(SIDEBAR_WIDTH, 0.f);
        sidebarDivider.setFillColor(COLOR_BORDER);
        window.draw(sidebarDivider);

        auto renderHeader = [&](const std::string& title, float posY) {
            sf::Text headerText;
            headerText.setFont(font);
            headerText.setString(title);
            headerText.setCharacterSize(13);
            headerText.setFillColor(COLOR_MUTED);
            headerText.setPosition(15.f, posY);
            window.draw(headerText);
        };

        renderHeader("MODES & GESTURES", 18.f);
        renderHeader("PRESET TEMPLATES", 310.f);
        renderHeader("VIEW & DISPLAY", 480.f);
        renderHeader("ALGORITHM SOLVER", 545.f);

        for (Button* btn : modeButtons) window.draw(*btn);
        window.draw(btnPresetDefault);
        window.draw(btnPresetGrid);
        window.draw(btnPresetRandom);
        window.draw(btnPresetClear);
        window.draw(btnResetView);
        window.draw(btnFullscreen);
        window.draw(btnSolve);

        // Bottom Playbar Panel
        sf::RectangleShape playbarBg(sf::Vector2f(currentCanvasWidth, PLAYBAR_HEIGHT));
        playbarBg.setPosition(SIDEBAR_WIDTH, windowHeight - PLAYBAR_HEIGHT);
        playbarBg.setFillColor(COLOR_SIDEBAR);
        window.draw(playbarBg);

        sf::RectangleShape playbarDivider(sf::Vector2f(currentCanvasWidth, 2.f));
        playbarDivider.setPosition(SIDEBAR_WIDTH, windowHeight - PLAYBAR_HEIGHT);
        playbarDivider.setFillColor(COLOR_BORDER);
        window.draw(playbarDivider);

        window.draw(btnStepBack);
        window.draw(btnPlayPause);
        window.draw(btnStepFwd);
        window.draw(btnReset);

        window.draw(btnSpeed1);
        window.draw(btnSpeed2);
        window.draw(btnSpeed5);

        float playbarYPos = windowHeight - PLAYBAR_HEIGHT + 18.f;
        float playbarXPos = SIDEBAR_WIDTH + 15.f;

        sf::Text stepText;
        stepText.setFont(font);
        stepText.setString("Step: " + std::to_string(solver.getCurrentStepIndex()) + " / " + (solver.getTotalSteps() > 0 ? std::to_string(solver.getTotalSteps() - 1) : "0"));
        stepText.setCharacterSize(12);
        stepText.setFillColor(COLOR_TEXT);
        stepText.setPosition(playbarXPos + 540.f, playbarYPos);
        window.draw(stepText);

        // Right Inspector Panel Background (Positioned relative to dynamic windowWidth)
        float inspX = windowWidth - INSPECTOR_WIDTH + 15.f;

        sf::RectangleShape inspectorBg(sf::Vector2f(INSPECTOR_WIDTH, static_cast<float>(windowHeight)));
        inspectorBg.setPosition(windowWidth - INSPECTOR_WIDTH, 0.f);
        inspectorBg.setFillColor(COLOR_SIDEBAR);
        window.draw(inspectorBg);

        sf::RectangleShape inspectorDivider(sf::Vector2f(2.f, static_cast<float>(windowHeight)));
        inspectorDivider.setPosition(windowWidth - INSPECTOR_WIDTH, 0.f);
        inspectorDivider.setFillColor(COLOR_BORDER);
        window.draw(inspectorDivider);

        sf::Text inspHeader;
        inspHeader.setFont(font);
        inspHeader.setString("INSPECTOR & STATE");
        inspHeader.setCharacterSize(13);
        inspHeader.setFillColor(COLOR_MUTED);
        inspHeader.setPosition(inspX, 20.f);
        window.draw(inspHeader);

        sf::Text msgText;
        msgText.setFont(font);
        msgText.setString(snapshot.message.empty() ? "Click 'RUN DIJKSTRA' to begin visualization." : snapshot.message);
        msgText.setCharacterSize(13);
        msgText.setFillColor(sf::Color(253, 230, 138));

        std::string rawMsg = msgText.getString();
        std::string wrappedMsg = "";
        float currentLineWidth = 0.f;
        std::istringstream wordsStream(rawMsg);
        std::string word;
        while (wordsStream >> word) {
            sf::Text dummyText;
            dummyText.setFont(font);
            dummyText.setCharacterSize(13);
            dummyText.setString(word + " ");
            float w = dummyText.getLocalBounds().width;
            if (currentLineWidth + w > INSPECTOR_WIDTH - 30.f) {
                wrappedMsg += "\n";
                currentLineWidth = 0.f;
            }
            wrappedMsg += word + " ";
            currentLineWidth += w;
        }
        msgText.setString(wrappedMsg);
        msgText.setPosition(inspX, 45.f);
        window.draw(msgText);

        sf::Text tableHeader;
        tableHeader.setFont(font);
        tableHeader.setString("DISTANCE TABLE");
        tableHeader.setCharacterSize(13);
        tableHeader.setFillColor(COLOR_MUTED);
        tableHeader.setPosition(inspX, 150.f);
        window.draw(tableHeader);

        sf::Text colHeader;
        colHeader.setFont(font);
        colHeader.setString("Node    Min Dist    Parent");
        colHeader.setCharacterSize(12);
        colHeader.setFillColor(COLOR_BORDER);
        colHeader.setPosition(inspX, 175.f);
        window.draw(colHeader);

        float rowY = 198.f;
        for (Vertex* v : allVertices) {
            if (rowY > windowHeight - 160.f) break;

            std::string nameStr = v->vertexName;
            auto dIt = snapshot.distances.find(nameStr);
            std::string distStr = (dIt != snapshot.distances.end() && dIt->second != INF) ? std::to_string(dIt->second) : "INF";

            auto pIt = snapshot.parents.find(nameStr);
            std::string parentStr = (pIt != snapshot.parents.end() && !pIt->second.empty()) ? pIt->second : "-";

            std::stringstream ss;
            ss << std::left << std::setw(8) << nameStr
               << std::setw(12) << distStr
               << parentStr;

            sf::Text rowText;
            rowText.setFont(font);
            rowText.setString(ss.str());
            rowText.setCharacterSize(12);
            rowText.setFillColor(v->vertexName == snapshot.currentNode ? sf::Color(253, 230, 138) : COLOR_TEXT);
            rowText.setPosition(inspX, rowY);
            window.draw(rowText);

            rowY += 19.f;
        }

        if (snapshot.isFinished) {
            sf::Text resHeader;
            resHeader.setFont(font);
            resHeader.setString("SHORTEST PATH");
            resHeader.setCharacterSize(13);
            resHeader.setFillColor(COLOR_MUTED);
            resHeader.setPosition(inspX, windowHeight - 145.f);
            window.draw(resHeader);

            sf::Text resText;
            resText.setFont(font);
            resText.setCharacterSize(12);
            if (snapshot.pathFound && endVertex) {
                std::string pathStr = "";
                for (size_t i = 0; i < shortestPath.size(); i++) {
                    pathStr += shortestPath[i] + (i + 1 < shortestPath.size() ? " -> " : "");
                }
                resText.setString("Path: " + pathStr + "\nCost: " + std::to_string(snapshot.distances.at(endVertex->vertexName)));
                resText.setFillColor(sf::Color(52, 211, 153));
            } else {
                resText.setString("No path exists between\nStart and End node.");
                resText.setFillColor(sf::Color(248, 113, 113));
            }
            resText.setPosition(inspX, windowHeight - 123.f);
            window.draw(resText);
        }

        sf::Text helpText;
        helpText.setFont(font);
        helpText.setString("Scroll: Zoom | Drag/Middle: Pan\nF11: Fullscreen | Right-Click: Toggle Dir");
        helpText.setCharacterSize(11);
        helpText.setFillColor(COLOR_MUTED);
        helpText.setPosition(inspX, windowHeight - 45.f);
        window.draw(helpText);

        window.display();
    }
};

int main() {
    AppState app;
    app.init();

#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop_arg([](void* arg) {
        auto* state = static_cast<AppState*>(arg);
        state->updateAndRender();
    }, &app, 0, 1);
#else
    while (app.window.isOpen()) {
        app.updateAndRender();
    }
#endif

    return 0;
}