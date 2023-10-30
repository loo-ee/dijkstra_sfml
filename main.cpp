#include <SFML/Graphics.hpp>
#include <iostream>
#include <cmath>
#include <string>
#include <stack>

#include "include/GraphManager.h"
#include "include/Button.h"

#define WHITE sf::Color::White
#define BLACK sf::Color::Black
#define GREEN sf::Color::Green
#define BLUE sf::Color::Blue
#define RED sf::Color::Red

void addVertexFromUser(GraphManager& manager, std::string& inputText);
void addVerticesToManager(GraphManager& manager);
void addVertexToManager(
    GraphManager& manager, std::string vertexName,
    std::vector<std::pair<std::string, int>> neighbors
);

int main(int argc, char* argv[]) {
    sf::RenderWindow window(sf::VideoMode(1280, 720), "Dijkstra Visualizer");
    sf::Font font1;
    GraphManager manager;

    window.setFramerateLimit(60);
    font1.loadFromFile("fonts/Meslo/Meslo LG L Bold Nerd Font Complete.ttf");

    std::string inputText;
    std::string messageInfo;

    bool appLoopRunning = true;
    bool isCalculating = false;
    bool dragging = false;
    bool isReceivingInput = false;
    bool isAddingNewLine = false;
    bool isSelectingStartVertex = false;
    bool isSelectingEndVertex = false;
    bool isFirstRun = true;
    bool vertexLoopRunning = true;
    bool isGraphFilled = true;
    bool showCursor = true;

    void (*createVertexFromUser)(GraphManager& manager, std::string& inputText) = addVertexFromUser;
    void (*addVerticesToGraph)(GraphManager& manager) = addVerticesToManager;

    Vertex* verticesForNewLine[2] = { nullptr };
    sf::Vector2f tempNewLineCoords[2];
    sf::Vector2f oldMousePos;

    sf::Clock cursorClock;
    sf::Clock delayClock;

    sf::Time elapsed;
    const sf::Time targetFrameTime = sf::seconds(1.f / 2);

    const int userInputTextFont = 28;
    const int messageTextFont = 20;

    sf::Text userInputText;
    userInputText.setPosition(sf::Vector2f(500.f, 650.f));
    userInputText.setFont(font1);
    userInputText.setString("Vertex name: " + inputText);
    userInputText.setCharacterSize(userInputTextFont);
    userInputText.setFillColor(WHITE);

    sf::Text messageText;
    messageText.setPosition(sf::Vector2f(10.f, 650.f));
    messageText.setFont(font1);
    messageText.setCharacterSize(messageTextFont);
    messageText.setFillColor(WHITE);

    sf::Vector2f cursorPos(userInputText.getPosition().x, userInputText.getPosition().y);
    sf::RectangleShape cursor(sf::Vector2f(2.f, 30.f));
    cursor.setFillColor(WHITE);
    cursor.setPosition(cursorPos);

    Vertex* startVertex = nullptr;
    Vertex* endVertex = nullptr;
    Vertex* topUnexplored = nullptr;
    Vertex* topExplored = nullptr;
    Vertex* dijkstraResultVertex = nullptr;

    std::vector<Vertex* >* vertices = manager.getVertices();
    std::vector<Vertex* > unexplored;
    std::vector<Vertex* > explored;

    std::vector<Vertex* >::iterator unexploredIterator = unexplored.begin();
    std::vector<Vertex* >::iterator exploredIterator = explored.begin();

    sf::RectangleShape* movedRect = nullptr;
    sf::RectangleShape* srcRect = nullptr;
    sf::RectangleShape* dstRect = nullptr;
    sf::RectangleShape* srcRectForCalculating = nullptr;
    sf::RectangleShape* dstRectForCalculating = nullptr;

    Button calculateBtn(sf::Vector2f(10.f, 10.f), sf::Vector2f(200.f, 50.f));
    calculateBtn.setButtonColor(GREEN);
    calculateBtn.setButtonText(font1, "Calculate", 24);
    calculateBtn.setButtonTextColor(BLACK);
    calculateBtn.setCallback([&isCalculating, &isFirstRun, &vertexLoopRunning]() {
        isCalculating = !isCalculating;
        isFirstRun = true;
        vertexLoopRunning = true;
        });

    Button addVertexBtn(sf::Vector2f(10.f, 70.f), sf::Vector2f(200.f, 50.f));
    addVertexBtn.setButtonColor(BLUE);
    addVertexBtn.setButtonText(font1, "Add Vertex", 24);
    addVertexBtn.setButtonTextColor(WHITE);
    addVertexBtn.setCallback([createVertexFromUser, &manager, &isReceivingInput, &inputText]() {

        if (isReceivingInput) {
            createVertexFromUser(manager, inputText);
        }

        isReceivingInput = !isReceivingInput;
    });

    Button addEdgeBtn(sf::Vector2f(10.f, 130.f), sf::Vector2f(200.f, 50.f));
    addEdgeBtn.setButtonColor(BLUE);
    addEdgeBtn.setButtonText(font1, "Add Edge", 24);
    addEdgeBtn.setButtonTextColor(WHITE);
    addEdgeBtn.setCallback([&isAddingNewLine]() {
        isAddingNewLine = !isAddingNewLine;
    });

    Button selectStartBtn(sf::Vector2f(10.f, 190.f), sf::Vector2f(200.f, 50.f));
    selectStartBtn.setButtonColor(GREEN);
    selectStartBtn.setButtonText(font1, "Select Start", 24);
    selectStartBtn.setButtonTextColor(BLACK);
    selectStartBtn.setCallback([&isSelectingStartVertex, &isSelectingEndVertex]() {
        isSelectingEndVertex = false;
        isSelectingStartVertex = true;
    });

    Button selectEndBtn(sf::Vector2f(10.f, 250.f), sf::Vector2f(200.f, 50.f));
    selectEndBtn.setButtonColor(RED);
    selectEndBtn.setButtonText(font1, "Select End", 24);
    selectEndBtn.setButtonTextColor(WHITE);
    selectEndBtn.setCallback([&isSelectingEndVertex, &isSelectingStartVertex]() {
        isSelectingStartVertex = false;
        isSelectingEndVertex = true;
    });

    Button clearVerticesBtn(sf::Vector2f(10.f, 350.f), sf::Vector2f(200.f, 50.f));
    clearVerticesBtn.setButtonColor(RED);
    clearVerticesBtn.setButtonText(font1, "Clear Graph", 24);
    clearVerticesBtn.setButtonTextColor(WHITE);
    clearVerticesBtn.setCallback([&isGraphFilled, &manager]() {
        manager.clearVertices();
        isGraphFilled = false;
    });

    Button addVerticesBtn(sf::Vector2f(10.f, 410.f), sf::Vector2f(200.f, 50.f));
    addVerticesBtn.setButtonColor(GREEN);
    addVerticesBtn.setButtonText(font1, "Fill Graph", 24);
    addVerticesBtn.setButtonTextColor(WHITE);
    addVerticesBtn.setCallback([addVerticesToGraph, &isGraphFilled, &manager]() {
        if (!isGraphFilled) {
            addVerticesToGraph(manager);
            isGraphFilled = true;
        }
    });

    addVerticesToManager(manager);

    while (window.isOpen()) {
        sf::Event event;
        window.clear();

        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    for (int i = 0; i < vertices->size(); i++) {
                        Vertex* currentVertex = vertices->at(i);
                        sf::RectangleShape* currentRect = &currentVertex->vertexRect;
                        sf::Vector2f mousePos = window.mapPixelToCoords(sf::Vector2i(event.mouseButton.x, event.mouseButton.y));

                        if (currentRect->getGlobalBounds().contains(mousePos)) {
                            if (isSelectingStartVertex) {
                                startVertex = currentVertex;
                                dijkstraResultVertex = nullptr;
                            }

                            if (isSelectingEndVertex) {
                                endVertex = currentVertex;
                                dijkstraResultVertex = nullptr;
                            }

                            if (isAddingNewLine) {
                                verticesForNewLine[0] = currentVertex;
                                tempNewLineCoords[0] = currentRect->getPosition();
                            } else {
                                oldMousePos = currentRect->getPosition();
                                movedRect = currentRect;
                            }

                            dragging = true;
                        }
                    }
                }
            }

            if (event.type == sf::Event::MouseButtonReleased) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    if (!isCalculating) {
                        if (isAddingNewLine) {
                            for (int i = 0; i < vertices->size(); i++) {
                                Vertex* currentVertex = vertices->at(i);
                                sf::RectangleShape* currentRect = &currentVertex->vertexRect;

                                if (currentRect->getGlobalBounds().contains(tempNewLineCoords[1])) {
                                    verticesForNewLine[1] = currentVertex;
                                }
                            }
                        }

                        movedRect = nullptr;
                        dragging = false;
                    }
                }
            }

            if (event.type == sf::Event::MouseMoved) {
                if (dragging && !isCalculating) {
                    sf::Vector2f mousePos = window.mapPixelToCoords(sf::Vector2i(event.mouseMove.x, event.mouseMove.y));

                    if (isAddingNewLine) {
                        tempNewLineCoords[1] = mousePos;
                    } else {
                        sf::Vector2f difference = mousePos - oldMousePos;
                        movedRect->move(difference);
                        oldMousePos = movedRect->getPosition();
                    }
                }
            }

            if (event.type == sf::Event::TextEntered && isReceivingInput && !isCalculating) {
                if (event.text.unicode < 128) {
                    if (event.text.unicode == '\b' && inputText.length() != 0) {
                        inputText.pop_back();
                    } else if (event.text.unicode != '\r' && event.text.unicode != '\n') {
                        inputText += static_cast<char>(event.text.unicode);
                    }
                }
            }

            calculateBtn.handleEvent(event, window);
            addVertexBtn.handleEvent(event, window);
            addEdgeBtn.handleEvent(event, window);
            selectStartBtn.handleEvent(event, window);
            selectEndBtn.handleEvent(event, window);
            clearVerticesBtn.handleEvent(event, window);
            addVerticesBtn.handleEvent(event, window);
        }

        if (isCalculating && startVertex != nullptr && endVertex != nullptr) {
            if (isFirstRun) {
                isFirstRun = false;
                unexplored.clear();
                explored.clear();
                manager.resetParentVertices();
                manager.resetMinDistancesFromSrc();

                for (int i = 0; i < vertices->size(); i++) {
                    unexplored.push_back(vertices->at(i));
                }

                std::vector<Vertex* >::iterator startElement = std::find_if(unexplored.begin(), unexplored.end(), [&startVertex](const Vertex* node) {
                    return node == startVertex;
                    });

                std::iter_swap(unexplored.begin(), startElement);
                unexplored.front()->minDistanceFromSrc = 0;

                system("CLS");
            }

            if (vertexLoopRunning) {
                manager.sort(unexplored);
                topUnexplored = unexplored.front();
                explored.push_back(topUnexplored);
                topExplored = explored.back();

                unexploredIterator = unexplored.begin();
                exploredIterator = explored.begin();
                vertexLoopRunning = false;

                srcRectForCalculating = &topExplored->vertexRect;
                messageInfo = "[GET LOWEST COST UNVISITED NODE] -> " + topExplored->vertexName;
            } else {
                if (unexploredIterator == unexplored.end()) {
                    unexplored.erase(unexplored.begin());
                    vertexLoopRunning = true;
                } else {
                    const std::string& target = (*unexploredIterator)->vertexName;

                    auto foundRelatedVertex = std::find_if(topExplored->neighbors.begin(), topExplored->neighbors.end(), [target](const std::pair<std::string, int>& neighbor) {
                        return neighbor.first == target;
                        });

                    if (foundRelatedVertex != topExplored->neighbors.end()) {
                        int totalDistance = topExplored->minDistanceFromSrc + foundRelatedVertex->second;

                        std::cout << "[CALCULATE " << topExplored->vertexName << " -> " << foundRelatedVertex->first << ": (" << totalDistance;
                        messageInfo = "[CALCULATE " + topExplored->vertexName + " -> " + foundRelatedVertex->first + ": (" + std::to_string(totalDistance);

                        if (totalDistance < (*unexploredIterator)->minDistanceFromSrc) {
                            std::cout << " < " << (*unexploredIterator)->minDistanceFromSrc << "), UPDATING " << foundRelatedVertex->first << " COST]\n";
                            messageInfo += " < " + std::to_string((*unexploredIterator)->minDistanceFromSrc) + "), UPDATING " + foundRelatedVertex->first + " COST]";

                            (*unexploredIterator)->parent = topExplored;
                            (*unexploredIterator)->minDistanceFromSrc = totalDistance;
                        } else {
                            std::cout << " > " << (*unexploredIterator)->minDistanceFromSrc << "), MAINTAIN " << foundRelatedVertex->first << " COST]\n";
                            messageInfo += " > " + std::to_string((*unexploredIterator)->minDistanceFromSrc) + "), MAINTAIN " + foundRelatedVertex->first + " COST]";
                        }

                        dstRectForCalculating = &(*unexploredIterator)->vertexRect;

                        const int labelFontSize = 20;
                        int srcPosX = srcRectForCalculating->getPosition().x;
                        int srcPosY = srcRectForCalculating->getPosition().y;
                        int dstPosX = dstRectForCalculating->getPosition().x;
                        int dstPosY = dstRectForCalculating->getPosition().y;
                        float length = std::sqrt(std::pow(dstPosX - srcPosX, 2) + std::pow(dstPosY - srcPosY, 2));

                        std::string distanceStr = std::to_string(length).substr(0, 6);
                        sf::Vector2f labelPos(srcPosX + (dstPosX - srcPosX) / 2, srcPosY + (dstPosY - srcPosY) / 2);
                        sf::VertexArray line(sf::Lines, 2);

                        foundRelatedVertex->second = length;

                        line[0].position = sf::Vector2f(srcPosX, srcPosY);
                        line[1].position = sf::Vector2f(dstPosX, dstPosY);
                        window.draw(line);
                        manager.renderText(distanceStr, labelPos, font1, labelFontSize, WHITE, window);
                    }

                    unexploredIterator++;
                }
            }

            for (int i = 0; i < explored.size(); i++) {
                Vertex* currentVertex = explored.at(i);

                if (topExplored == currentVertex)
                    continue;

                srcRect = &currentVertex->vertexRect;
                int srcPosX = srcRect->getPosition().x;
                int srcPosY = srcRect->getPosition().y;

                for (std::pair<std::string, int>& currentNeighbor : currentVertex->neighbors) {
                    Vertex* neighborRelatedVertex = manager.getOneVertex(currentNeighbor.first);
                    dstRect = &neighborRelatedVertex->vertexRect;

                    const int labelFontSize = 20;
                    int dstPosX = dstRect->getPosition().x;
                    int dstPosY = dstRect->getPosition().y;
                    float length = std::sqrt(std::pow(dstPosX - srcPosX, 2) + std::pow(dstPosY - srcPosY, 2));

                    std::string distanceStr = std::to_string(length).substr(0, 6);
                    sf::Vector2f labelPos(srcPosX + (dstPosX - srcPosX) / 2, srcPosY + (dstPosY - srcPosY) / 2);
                    sf::VertexArray line(sf::Lines, 2);

                    currentNeighbor.second = length;

                    line[0].position = sf::Vector2f(srcPosX, srcPosY);
                    line[1].position = sf::Vector2f(dstPosX, dstPosY);

                    window.draw(line);
                    manager.renderText(distanceStr, labelPos, font1, labelFontSize, WHITE, window);
                }
            }
        }

        for (int i = 0; i < vertices->size(); i++) {
            Vertex* currentVertex = vertices->at(i);
            std::string vertexName = currentVertex->vertexName;
            sf::RectangleShape* vertexRect = &currentVertex->vertexRect;
            sf::Vector2f rectPos(vertexRect->getPosition().x, vertexRect->getPosition().y - 25);
            const int vertexFontSize = 20;

            if (!isCalculating) {
                Vertex* pathPtr = dijkstraResultVertex;

                while (pathPtr) {
                    if (pathPtr == currentVertex)
                        break;

                    pathPtr = pathPtr->parent;
                }

                if (startVertex != nullptr && endVertex != nullptr && startVertex == endVertex)
                    vertexRect->setFillColor(BLUE);
                else if (startVertex != nullptr && &startVertex->vertexRect == vertexRect)
                    vertexRect->setFillColor(GREEN);
                else if (endVertex != nullptr && &endVertex->vertexRect == vertexRect)
                    vertexRect->setFillColor(RED);
                else if (pathPtr != nullptr && startVertex != nullptr && endVertex != nullptr && currentVertex != startVertex && currentVertex!= endVertex)
                    vertexRect->setFillColor(BLUE);
                else
                    vertexRect->setFillColor(WHITE);
            } else {
                if (topExplored == currentVertex)
                    vertexRect->setFillColor(GREEN);
                else
                    vertexRect->setFillColor(WHITE);
            }

            window.draw(*vertexRect);
            manager.renderText(vertexName, rectPos, font1, vertexFontSize, WHITE, window);

            srcRect = &currentVertex->vertexRect;
            int srcPosX = srcRect->getPosition().x;
            int srcPosY = srcRect->getPosition().y;

            for (std::pair<std::string, int>& currentNeighbor : currentVertex->neighbors) {
                Vertex* neighborRelatedVertex = manager.getOneVertex(currentNeighbor.first);
                dstRect = &neighborRelatedVertex->vertexRect;

                const int labelFontSize = 20;
                int dstPosX = dstRect->getPosition().x;
                int dstPosY = dstRect->getPosition().y;
                float length = std::sqrt(std::pow(dstPosX - srcPosX, 2) + std::pow(dstPosY - srcPosY, 2));

                std::string distanceStr = std::to_string(length).substr(0, 6);
                sf::Vector2f labelPos(srcPosX + (dstPosX - srcPosX) / 2, srcPosY + (dstPosY - srcPosY) / 2);
                sf::VertexArray line(sf::Lines, 2);

                currentNeighbor.second = length;

                line[0].position = sf::Vector2f(srcPosX, srcPosY);
                line[1].position = sf::Vector2f(dstPosX, dstPosY);

                if (!isCalculating) {
                    window.draw(line);
                    manager.renderText(distanceStr, labelPos, font1, labelFontSize, WHITE, window);
                }
            }
        }

        if (unexplored.empty() && isCalculating) {
            isCalculating = false;
            messageInfo = "";

            auto foundDijkstraResult = std::find_if(explored.begin(), explored.end(), [&endVertex](const Vertex* currentVertex) {
                return currentVertex == endVertex;
                });

            if (foundDijkstraResult != explored.end()) {
                dijkstraResultVertex = (*foundDijkstraResult);
                Vertex* pathPtr = dijkstraResultVertex;

                while (pathPtr) {
                    messageInfo = pathPtr->vertexName + (messageInfo == "" ? "" : " -> " + messageInfo);
                    pathPtr = pathPtr->parent;
                }

                messageInfo = "Path: " + messageInfo;
            }
        }

        userInputText.setString("Vertex name: " + inputText);

        float lastLetterCursosPos = userInputText.findCharacterPos(userInputText.getString().getSize()).x;
        cursorPos.x = lastLetterCursosPos;
        cursor.setPosition(cursorPos);


        if (isAddingNewLine) {
            addEdgeBtn.setButtonText(font1, "Done", 24);

            if (dragging) {
                if (
                    (tempNewLineCoords[0].x != 0 && tempNewLineCoords[0].y != 0) &&
                    (tempNewLineCoords[1].x != 0 && tempNewLineCoords[1].y != 0)
                    ) {
                    sf::VertexArray line(sf::Lines, 2);
                    line[0].position = tempNewLineCoords[0];
                    line[1].position = tempNewLineCoords[1];

                    window.draw(line);
                }
            }
        } else {
            addEdgeBtn.setButtonText(font1, "Add Edge", 24);
        }

        if (verticesForNewLine[0] != nullptr && verticesForNewLine[1] != nullptr) {
            std::string vertex0Name = verticesForNewLine[0]->vertexName;
            std::string vertex1Name = verticesForNewLine[1]->vertexName;

            verticesForNewLine[0]->neighbors.push_back(std::pair<std::string, int>(vertex1Name, 0));
            verticesForNewLine[1]->neighbors.push_back(std::pair<std::string, int>(vertex0Name, 0));

            verticesForNewLine[0] = nullptr;
            verticesForNewLine[1] = nullptr;

            tempNewLineCoords[0] = sf::Vector2f(0, 0);
            tempNewLineCoords[1] = sf::Vector2f(0, 0);
        }

        if (cursorClock.getElapsedTime().asSeconds() >= 0.5f) {
            showCursor = !showCursor;
            cursorClock.restart();
        }

        if (isReceivingInput) {
            addVertexBtn.setButtonText(font1, "Done", 24);
            window.draw(userInputText);

            if (showCursor) {
                window.draw(cursor);
            }
        } else {
            addVertexBtn.setButtonText(font1, "Add Vertex", 24);
            inputText = "";
        }

        if (!isCalculating || !startVertex || !endVertex) {
            window.draw(calculateBtn);
            window.draw(addVertexBtn);
            window.draw(addEdgeBtn);
            window.draw(selectStartBtn);
            window.draw(selectEndBtn);
            window.draw(clearVerticesBtn);
            window.draw(addVerticesBtn);
        }

        messageText.setString(messageInfo);
        window.draw(messageText);
        window.display();

        if (isCalculating && elapsed < targetFrameTime) {
           sf::sleep(targetFrameTime - elapsed);
        }
    }

    return 0;
}

void addVertexFromUser(GraphManager& manager, std::string& inputText) {
    const int newVertexPosX = 100;
    const int newVertexPosY = 500;

    manager.createVertex(inputText, std::vector<std::pair<std::string, int>>{}, newVertexPosX, newVertexPosY);
}

void addVerticesToManager(GraphManager& manager) {
    std::vector<std::pair<std::string, int>> vertexA = {
        std::pair<std::string, int>("B", 0),
        std::pair<std::string, int>("C", 0),
        std::pair<std::string, int>("D", 0),
        std::pair<std::string, int>("G", 0)
    };

    std::vector<std::pair<std::string, int>> vertexB = {
        std::pair<std::string, int>("A", 0),
        std::pair<std::string, int>("C", 0),
        std::pair<std::string, int>("F", 0),
        std::pair<std::string, int>("G", 0)
    };

    std::vector<std::pair<std::string, int>> vertexC = {
        std::pair<std::string, int>("A", 0),
        std::pair<std::string, int>("B", 0),
        std::pair<std::string, int>("D", 0),
        std::pair<std::string, int>("E", 0)
    };

    std::vector<std::pair<std::string, int>> vertexD = {
        std::pair<std::string, int>("A", 0),
        std::pair<std::string, int>("C", 0),
        std::pair<std::string, int>("E", 0),
        std::pair<std::string, int>("H", 0)
    };

    std::vector<std::pair<std::string, int>> vertexE = {
        std::pair<std::string, int>("C", 0),
        std::pair<std::string, int>("D", 0),
        std::pair<std::string, int>("F", 0)
    };

    std::vector<std::pair<std::string, int>> vertexF = {
        std::pair<std::string, int>("E", 0),
        std::pair<std::string, int>("B", 0),
        std::pair<std::string, int>("H", 0)
    };

    std::vector<std::pair<std::string, int>> vertexG = {
        std::pair<std::string, int>("A", 0),
        std::pair<std::string, int>("B", 0)
    };
    
    std::vector<std::pair<std::string, int>> vertexH = {
        std::pair<std::string, int>("F", 0),
        std::pair<std::string, int>("D", 0)
    };

    manager.createVertex("A", vertexA, 500, 400);
    manager.createVertex("B", vertexB, 540, 80);
    manager.createVertex("C", vertexC, 800, 200);
    manager.createVertex("D", vertexD, 840, 450);
    manager.createVertex("E", vertexE, 900, 175);
    manager.createVertex("F", vertexF, 1000, 100);
    manager.createVertex("G", vertexG, 300, 260);
    manager.createVertex("H", vertexH, 1050, 300);
}

void addVertexToManager(
    GraphManager& manager, std::string vertexName,
    std::vector<std::pair<std::string, int>> neighbors
) {
    manager.createVertex(vertexName, neighbors, 10, 200);
}