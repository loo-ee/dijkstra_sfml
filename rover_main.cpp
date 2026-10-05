#include <raylib.h>
#include <raymath.h>
#include <vector>
#include <string>
#include <memory>
#include <iostream>

#include "Vertex3D.h"
#include "GraphRenderer3D.h"
#include "OrbitCameraController.h"
#include "TerrainHeightfield.h"
#include "RoverNavGraph.h"

int main() {
    // 1. Window Initialization
    const int screenWidth = 1280;
    const int screenHeight = 720;

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(screenWidth, screenHeight, "3D Planetary Rover Simulator - Terrain & NavGraph (Phase 2)");
    SetTargetFPS(60);

    // 2. Camera Setup (Positioned for panoramic overview of 200m x 200m terrain)
    OrbitCameraController cameraController(
        Vector3{ 0.0f, 75.0f, 115.0f }, // Initial camera position
        Vector3{ 0.0f, 0.0f, 0.0f },     // Target center
        45.0f                           // Perspective FOV
    );

    // 3. Generate Procedural Martian Heightfield (128x128 across 200m x 200m)
    TerrainHeightfield terrain(128, 200.0f);
    terrain.generate();

    // 4. Drape 3D NavGraph over Terrain (26x26 = 676 nodes, > 500 nodes criteria)
    const int navCols = 26;
    const int navRows = 26;
    const float navSpacing = 7.0f; // 25 intervals * 7m = 175m coverage within 200m terrain

    RoverNavGraph navGraph;
    navGraph.generateTerrainGrid(terrain, navCols, navRows, navSpacing);

    // Display options
    bool showTerrain = true;
    bool showEdges = true;
    bool showNodes = true;
    bool showWireframe = false;

    // 5. Main Simulation Loop
    while (!WindowShouldClose()) {
        // Update Orbital Camera
        cameraController.update();

        // 3D Raycast Mouse Picking for Start / End Nodes
        Ray mouseRay = GetMouseRay(GetMousePosition(), cameraController.getCamera());
        Vertex3D* hoveredNode = navGraph.pickNodeFromRay(mouseRay, 1.8f);

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            if (hoveredNode) {
                if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                    navGraph.setEndNode(hoveredNode);
                } else {
                    navGraph.setStartNode(hoveredNode);
                }
            }
        }

        // Keyboard Controls
        if (IsKeyPressed(KEY_R)) {
            cameraController.reset(Vector3{ 0.0f, 75.0f, 115.0f }, Vector3{ 0.0f, 0.0f, 0.0f });
        }
        if (IsKeyPressed(KEY_T)) {
            showTerrain = !showTerrain;
        }
        if (IsKeyPressed(KEY_E)) {
            showEdges = !showEdges;
        }
        if (IsKeyPressed(KEY_N)) {
            showNodes = !showNodes;
        }
        if (IsKeyPressed(KEY_W)) {
            showWireframe = !showWireframe;
        }

        // Render Frame
        BeginDrawing();
        ClearBackground(Color{ 18, 20, 26, 255 }); // Dark space sky

        // 3D Scene Rendering
        BeginMode3D(cameraController.getCamera());
        {
            // A. Draw Procedural Martian Terrain Mesh
            if (showTerrain && terrain.isLoaded()) {
                DrawModel(terrain.getModel(), Vector3{ 0.0f, 0.0f, 0.0f }, 1.0f, WHITE);

                if (showWireframe) {
                    DrawModelWires(terrain.getModel(), Vector3{ 0.0f, 0.0f, 0.0f }, 1.0f, ColorAlpha(BLACK, 0.2f));
                }
            }

            // B. Draw Draped NavGraph Edges (continuous 3D cylinders)
            if (showEdges) {
                for (const auto& edge : navGraph.getEdges()) {
                    GraphRenderer3D::drawEdge(edge.start, edge.end, 0.09f, edge.color);
                }
            }

            // C. Draw Draped NavGraph Nodes (3D colored spheres)
            if (showNodes) {
                for (const Vertex3D* v : navGraph.getVertices()) {
                    float radius = (v == navGraph.getStartNode() || v == navGraph.getEndNode()) ? 1.6f : 0.75f;
                    GraphRenderer3D::drawNode(*v, radius);
                }
            }

            // D. Highlight Hovered Node
            if (hoveredNode) {
                DrawSphereWires(hoveredNode->position, 2.2f, 10, 10, GOLD);
                DrawCircle3D(hoveredNode->position, 2.5f, Vector3{ 0.0f, 1.0f, 0.0f }, 90.0f, ColorAlpha(YELLOW, 0.6f));
            }
        }
        EndMode3D();

        // 2D HUD & Telemetry Overlay
        DrawRectangle(16, 16, 440, 310, ColorAlpha(Color{ 12, 16, 24, 255 }, 0.88f));
        DrawRectangleLines(16, 16, 440, 310, Color{ 55, 70, 95, 255 });

        DrawText("Phase 2: Procedural Terrain & NavGraph", 28, 26, 18, RAYWHITE);
        DrawText(TextFormat("FPS: %i (Target: 60)", GetFPS()), 28, 52, 14, GREEN);

        DrawText(TextFormat("Terrain: %dx%d (%i verts) | Area: 200m x 200m",
            terrain.getResolution(), terrain.getResolution(),
            terrain.getResolution() * terrain.getResolution()), 28, 70, 13, LIGHTGRAY);

        DrawText(TextFormat("NavGraph: %d Draped Nodes | %d Edges (8-way)",
            (int)navGraph.getVertices().size(), (int)navGraph.getEdges().size()), 28, 88, 13, SKYBLUE);

        // Terrain Slope Color Legend
        DrawText("Terrain Slope Shading:", 28, 110, 13, RAYWHITE);
        DrawRectangle(32, 130, 14, 14, Color{ 195, 92, 60, 255 });
        DrawText("Dust (<15 deg)", 52, 130, 12, RAYWHITE);

        DrawRectangle(162, 130, 14, 14, Color{ 110, 68, 55, 255 });
        DrawText("Bedrock (15-30 deg)", 182, 130, 12, RAYWHITE);

        DrawRectangle(322, 130, 14, 14, Color{ 60, 50, 48, 255 });
        DrawText("Basalt (>30 deg)", 342, 130, 12, RAYWHITE);

        // Picking Telemetry
        DrawText("Interactive 3D Picking:", 28, 154, 13, RAYWHITE);
        DrawCircle(38, 178, 6, GraphRenderer3D::getNodeColor(NodeState::START));
        std::string startInfo = navGraph.getStartNode() 
            ? TextFormat("Start: %s (y=%.1fm, slope=%.1f deg)", 
                navGraph.getStartNode()->name.c_str(), 
                navGraph.getStartNode()->position.y,
                navGraph.getStartNode()->slopeAngleRad * RAD2DEG)
            : "Start: None";
        DrawText(startInfo.c_str(), 52, 172, 12, RAYWHITE);

        DrawCircle(38, 198, 6, GraphRenderer3D::getNodeColor(NodeState::END));
        std::string endInfo = navGraph.getEndNode()
            ? TextFormat("End:   %s (y=%.1fm, slope=%.1f deg)",
                navGraph.getEndNode()->name.c_str(),
                navGraph.getEndNode()->position.y,
                navGraph.getEndNode()->slopeAngleRad * RAD2DEG)
            : "End: None";
        DrawText(endInfo.c_str(), 52, 192, 12, RAYWHITE);

        // Hovered Node Telemetry
        if (hoveredNode) {
            DrawText(TextFormat("Hover: %s | Alt: %.1fm | Slope: %.1f deg %s",
                hoveredNode->name.c_str(),
                hoveredNode->position.y,
                hoveredNode->slopeAngleRad * RAD2DEG,
                hoveredNode->isWalkable ? "[Walkable]" : "[IMPASSABLE CLIFF]"),
                28, 218, 12, hoveredNode->isWalkable ? GOLD : RED);
        } else {
            DrawText("Hover: [Hover mouse over any 3D node]", 28, 218, 12, GRAY);
        }

        // Instructions
        DrawText("Picking: [Left Click] Set Start | [Shift + Left Click] Set End", 28, 246, 12, GOLD);
        DrawText("Camera:  [RMB Drag] Orbit | [MMB Drag] Pan | [Wheel] Zoom", 28, 266, 12, LIGHTGRAY);
        DrawText("Toggles: [T] Terrain | [E] Edges | [N] Nodes | [W] Wireframe | [R] Reset", 28, 286, 11, GRAY);

        EndDrawing();
    }

    // Cleanup & Exit
    terrain.unload();
    navGraph.clear();
    CloseWindow();
    return 0;
}
