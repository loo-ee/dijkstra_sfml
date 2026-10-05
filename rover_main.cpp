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
#include "PhysicsWorld.h"

int main() {
    // 1. Window Initialization
    const int screenWidth = 1280;
    const int screenHeight = 720;

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(screenWidth, screenHeight, "3D Planetary Rover Simulator - Jolt Physics Integration (Phase 3)");
    SetTargetFPS(60);

    // 2. Camera Setup (Panoramic overview of Martian terrain)
    OrbitCameraController cameraController(
        Vector3{ 0.0f, 75.0f, 115.0f },
        Vector3{ 0.0f, 0.0f, 0.0f },
        45.0f
    );

    // 3. Jolt Physics System Initialization
    PhysicsWorld physics;
    physics.init();

    // 4. Generate Procedural Martian Heightfield (128x128 across 200m x 200m)
    TerrainHeightfield terrain(128, 200.0f);
    terrain.generate();

    // 5. Create Physics Heightfield Collision Shape in Jolt
    float terrainSpacing = terrain.getSize() / (terrain.getResolution() - 1);
    physics.createTerrainHeightfield(
        terrain.getHeightData().data(),
        terrain.getResolution(),
        terrain.getResolution(),
        terrainSpacing
    );

    // 6. Spawn Static Boulders & Hazard Obstacles
    struct BoulderPreset {
        Vector2 pos;
        float radius;
    };
    std::vector<BoulderPreset> boulderPresets = {
        { { 18.0f, -10.0f }, 3.5f },  // Inside primary crater
        { { 32.0f, -22.0f }, 2.8f },  // On crater rim
        { { -15.0f, 12.0f }, 3.2f },  // On open plain
        { { -30.0f, -25.0f }, 4.0f }, // Large obstacle
        { { 5.0f, 35.0f }, 2.5f },
        { { -40.0f, 28.0f }, 3.0f },  // Near secondary crater
        { { -55.0f, -10.0f }, 2.6f },
        { { 45.0f, 20.0f }, 3.4f },
        { { 10.0f, -45.0f }, 3.0f },
        { { -10.0f, -60.0f }, 3.8f },
        { { 60.0f, -30.0f }, 2.9f },
        { { 25.0f, 50.0f }, 3.1f },
        { { -25.0f, 45.0f }, 2.7f },
        { { 0.0f, -15.0f }, 2.4f },
        { { -8.0f, 2.0f }, 2.2f }
    };

    for (const auto& bp : boulderPresets) {
        float h = terrain.getHeight(bp.pos.x, bp.pos.y);
        Vector3 boulderPos = { bp.pos.x, h + bp.radius * 0.75f, bp.pos.y };
        physics.spawnBoulder(boulderPos, bp.radius);
    }

    // 7. Drape 3D NavGraph over Terrain (26x26 = 676 nodes, > 500 nodes criteria)
    const int navCols = 26;
    const int navRows = 26;
    const float navSpacing = 7.0f;

    RoverNavGraph navGraph;
    navGraph.generateTerrainGrid(terrain, navCols, navRows, navSpacing);

    // 8. Line-of-Sight Edge Validation using Physical Raycasts
    // Cuts graph edges that intersect with boulders or sharp ground ridges
    navGraph.validateEdgesWithPhysics(physics, 0.6f);

    // Count blocked edges
    int blockedEdgeCount = 0;
    for (const auto& e : navGraph.getEdges()) {
        if (e.isBlocked) blockedEdgeCount++;
    }

    // 9. Initial Rolling Test Sphere (placed high on crater rim slope to demonstrate gravity)
    float startX = 22.0f;
    float startZ = -12.0f;
    float sphereSpawnY = terrain.getHeight(startX, startZ) + 5.0f;
    physics.spawnDynamicSphere(Vector3{ startX, sphereSpawnY, startZ }, 1.2f, 50.0f);

    // Display options
    bool showTerrain = true;
    bool showEdges = true;
    bool showNodes = true;
    bool showBoulders = true;
    bool showWireframe = false;

    // 10. Main Simulation Loop
    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt > 0.05f) dt = 0.05f; // Clamp delta time for stable physics step

        // Step Jolt Physics (60 Hz multi-threaded)
        physics.step(dt);

        // Update Orbital Camera
        cameraController.update();

        // 3D Raycast Mouse Picking for Nodes
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

        // Spawn dynamic test sphere at hovered node or above terrain
        if (IsKeyPressed(KEY_B)) {
            Vector3 dropPos;
            if (hoveredNode) {
                dropPos = Vector3Add(hoveredNode->position, Vector3{ 0.0f, 4.0f, 0.0f });
            } else {
                dropPos = Vector3{ 0.0f, terrain.getHeight(0.0f, 0.0f) + 8.0f, 0.0f };
            }
            physics.spawnDynamicSphere(dropPos, 1.2f, 50.0f);
        }

        // Spacebar: Cascade multiple spheres down crater
        if (IsKeyPressed(KEY_SPACE)) {
            for (int i = 0; i < 3; ++i) {
                float ox = static_cast<float>((rand() % 20) - 10);
                float oz = static_cast<float>((rand() % 20) - 10);
                float sx = 25.0f + ox;
                float sz = -15.0f + oz;
                float sy = terrain.getHeight(sx, sz) + 6.0f + i * 2.0f;
                physics.spawnDynamicSphere(Vector3{ sx, sy, sz }, 1.0f + 0.3f * i, 40.0f + 20.0f * i);
            }
        }

        // Clear dynamic spheres
        if (IsKeyPressed(KEY_C)) {
            physics.clearDynamicSpheres();
        }

        // Keyboard Controls
        if (IsKeyPressed(KEY_R)) {
            cameraController.reset(Vector3{ 0.0f, 75.0f, 115.0f }, Vector3{ 0.0f, 0.0f, 0.0f });
        }
        if (IsKeyPressed(KEY_T)) showTerrain = !showTerrain;
        if (IsKeyPressed(KEY_E)) showEdges = !showEdges;
        if (IsKeyPressed(KEY_N)) showNodes = !showNodes;
        if (IsKeyPressed(KEY_O)) showBoulders = !showBoulders;
        if (IsKeyPressed(KEY_W)) showWireframe = !showWireframe;

        // Render Frame
        BeginDrawing();
        ClearBackground(Color{ 18, 20, 26, 255 }); // Martian night sky

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

            // B. Draw Static Boulders (Physical obstacles)
            if (showBoulders) {
                for (const auto& b : physics.getBoulders()) {
                    DrawSphere(b.pos, b.radius, b.color);
                    DrawSphereWires(b.pos, b.radius, 10, 10, ColorAlpha(BLACK, 0.35f));
                }
            }

            // C. Draw Draped NavGraph Edges
            if (showEdges) {
                for (const auto& edge : navGraph.getEdges()) {
                    float radius = edge.isBlocked ? 0.16f : 0.08f;
                    GraphRenderer3D::drawEdge(edge.start, edge.end, radius, edge.color);
                }
            }

            // D. Draw Draped NavGraph Nodes
            if (showNodes) {
                for (const Vertex3D* v : navGraph.getVertices()) {
                    float radius = (v == navGraph.getStartNode() || v == navGraph.getEndNode()) ? 1.6f : 0.70f;
                    GraphRenderer3D::drawNode(*v, radius);
                }
            }

            // E. Draw Dynamic Rolling Test Spheres (Synchronized with Jolt Physics rigid bodies)
            for (auto sphereId : physics.getDynamicSpheres()) {
                Vector3 pos = physics.getBodyPosition(sphereId);
                // Vivid cyan/blue rolling sphere
                DrawSphere(pos, 1.2f, Color{ 46, 170, 240, 255 });
                DrawSphereWires(pos, 1.2f, 10, 10, ColorAlpha(WHITE, 0.7f));
            }

            // F. Highlight Hovered Node
            if (hoveredNode) {
                DrawSphereWires(hoveredNode->position, 2.2f, 10, 10, GOLD);
                DrawCircle3D(hoveredNode->position, 2.5f, Vector3{ 0.0f, 1.0f, 0.0f }, 90.0f, ColorAlpha(YELLOW, 0.6f));
            }
        }
        EndMode3D();

        // 2D HUD & Telemetry Overlay
        DrawRectangle(16, 16, 450, 340, ColorAlpha(Color{ 10, 14, 22, 255 }, 0.90f));
        DrawRectangleLines(16, 16, 450, 340, Color{ 55, 75, 105, 255 });

        DrawText("Phase 3: Jolt Physics & Terrain Collision", 28, 26, 18, RAYWHITE);
        DrawText(TextFormat("FPS: %i (Target: 60) | Step: 60Hz", GetFPS()), 28, 52, 14, GREEN);

        // Physics Telemetry
        DrawText(TextFormat("Jolt Physics: ACTIVE | Gravity: Martian (g = -3.71 m/s^2)"), 28, 72, 13, SKYBLUE);
        DrawText(TextFormat("Static Boulders: %d | Dynamic Spheres: %d", 
            (int)physics.getBoulders().size(), (int)physics.getDynamicSpheres().size()), 28, 90, 13, RAYWHITE);

        DrawText(TextFormat("NavGraph: %d Nodes | %d Edges (%d CUT by Raycast)",
            (int)navGraph.getVertices().size(), 
            (int)navGraph.getEdges().size(), 
            blockedEdgeCount), 28, 108, 13, (blockedEdgeCount > 0) ? Color{ 255, 100, 100, 255 } : LIGHTGRAY);

        // Picking Telemetry
        DrawText("Interactive 3D Picking:", 28, 132, 13, RAYWHITE);
        DrawCircle(38, 154, 6, GraphRenderer3D::getNodeColor(NodeState::START));
        std::string startInfo = navGraph.getStartNode() 
            ? TextFormat("Start: %s (y=%.1fm, slope=%.1f deg)", 
                navGraph.getStartNode()->name.c_str(), 
                navGraph.getStartNode()->position.y,
                navGraph.getStartNode()->slopeAngleRad * RAD2DEG)
            : "Start: None";
        DrawText(startInfo.c_str(), 52, 148, 12, RAYWHITE);

        DrawCircle(38, 174, 6, GraphRenderer3D::getNodeColor(NodeState::END));
        std::string endInfo = navGraph.getEndNode() 
            ? TextFormat("End:   %s (y=%.1fm, slope=%.1f deg)", 
                navGraph.getEndNode()->name.c_str(), 
                navGraph.getEndNode()->position.y,
                navGraph.getEndNode()->slopeAngleRad * RAD2DEG)
            : "End: None";
        DrawText(endInfo.c_str(), 52, 168, 12, RAYWHITE);

        // Edge Legend
        DrawText("NavGraph Edge Status:", 28, 194, 13, RAYWHITE);
        DrawRectangle(32, 212, 14, 8, Color{ 90, 115, 145, 255 });
        DrawText("Clear Line of Sight", 52, 209, 12, RAYWHITE);

        DrawRectangle(212, 212, 14, 8, Color{ 220, 45, 45, 255 });
        DrawText("Blocked (Boulder Cut)", 232, 209, 12, Color{ 255, 120, 120, 255 });

        // Physics Action Keys
        DrawText("Physics Actions:", 28, 234, 13, GOLD);
        DrawText("[B] Drop Sphere at Hovered Node | [SPACE] Cascade Crater Spheres", 28, 252, 12, RAYWHITE);
        DrawText("[C] Clear Dynamic Spheres | [O] Toggle Boulders", 28, 270, 12, RAYWHITE);

        // General Controls
        DrawText("Picking: [Left Click] Start | [Shift + Left Click] End", 28, 294, 11, LIGHTGRAY);
        DrawText("Camera:  [RMB Drag] Orbit | [MMB Drag] Pan | [Wheel] Zoom | [R] Reset", 28, 312, 11, GRAY);

        EndDrawing();
    }

    // Cleanup & Exit
    physics.shutdown();
    terrain.unload();
    navGraph.clear();
    CloseWindow();
    return 0;
}
