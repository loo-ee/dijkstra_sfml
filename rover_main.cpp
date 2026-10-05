#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
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
#include "DijkstraSolver3D.h"
#include "PlanetaryRover.h"
#include "RoverTelemetryHUD.h"

// Helper: Render a modern, high-contrast keybinding badge pill with text
static void DrawKeyBind(int x, int y, const char* key, const char* label, bool active = true) {
    int keyW = MeasureText(key, 11);
    int padX = 5;
    int h = 18;
    Rectangle badgeRect = { (float)x, (float)y, (float)(keyW + padX * 2), (float)h };
    Color badgeBg = active ? Color{ 26, 36, 52, 255 } : Color{ 18, 22, 30, 180 };
    Color badgeBorder = active ? Color{ 70, 110, 165, 255 } : Color{ 40, 50, 68, 200 };
    Color keyColor = active ? Color{ 225, 238, 255, 255 } : Color{ 110, 120, 135, 255 };
    Color labelColor = active ? Color{ 210, 220, 230, 255 } : Color{ 110, 120, 130, 255 };

    DrawRectangleRounded(badgeRect, 0.35f, 4, badgeBg);
    DrawRectangleRoundedLines(badgeRect, 0.35f, 4, badgeBorder);
    DrawText(key, x + padX, y + 4, 11, keyColor);

    int labelX = x + keyW + padX * 2 + 6;
    DrawText(label, labelX, y + 4, 11, labelColor);
}

// Helper: Load/Switch Terrain Mode Presets with customized physical landscapes & obstacles
static void ApplyTerrainPreset(
    TerrainPreset preset,
    TerrainHeightfield& terrain,
    PhysicsWorld& physics,
    RoverNavGraph& navGraph,
    DijkstraSolver3D& dijkstra,
    PlanetaryRover& rover,
    int& blockedEdgeCount
) {
    terrain.setPreset(preset);
    terrain.generate();

    float terrainSpacing = terrain.getSize() / (terrain.getResolution() - 1);
    physics.createTerrainHeightfield(
        terrain.getHeightData().data(),
        terrain.getResolution(),
        terrain.getResolution(),
        terrainSpacing
    );

    physics.clearBoulders();
    physics.clearDynamicSpheres();

    struct BoulderPreset {
        Vector2 pos;
        float radius;
    };
    std::vector<BoulderPreset> boulders;

    if (preset == TerrainPreset::OLYMPUS_CRATER) {
        boulders = {
            { { 18.0f, -10.0f }, 3.5f },  // Inside primary crater
            { { 32.0f, -22.0f }, 2.8f },  // On crater rim
            { { -15.0f, 12.0f }, 3.2f },  // On open plain
            { { -30.0f, -25.0f }, 4.0f }, // Large obstacle
            { { 5.0f, 35.0f }, 2.5f },
            { { -40.0f, 28.0f }, 3.0f },  // Near secondary crater
            { { -55.0f, -10.0f }, 2.6f },
            { { 45.0f, 20.0f }, 3.4f },
            { { 10.0f, -45.0f }, 3.0f },
            { { -10.0f, -60.0f }, 3.8f }
        };
    } else if (preset == TerrainPreset::SCREE_SLOPE) {
        boulders = {
            { { -20.0f, 10.0f }, 3.0f },
            { { -10.0f, -20.0f }, 3.5f },
            { { 15.0f, -5.0f }, 2.8f },
            { { 25.0f, 25.0f }, 3.2f },
            { { -35.0f, 30.0f }, 2.6f },
            { { 0.0f, 40.0f }, 3.4f },
            { { 40.0f, -30.0f }, 2.9f }
        };
    } else if (preset == TerrainPreset::BOULDER_SLALOM) {
        // Natural slalom gates along the canyon floor
        boulders = {
            { { -10.0f, -50.0f }, 3.2f },
            { { 12.0f, -30.0f }, 3.5f },
            { { -8.0f, -10.0f }, 3.2f },
            { { 14.0f, 10.0f }, 3.6f },
            { { -12.0f, 30.0f }, 3.4f },
            { { 8.0f, 50.0f }, 3.5f },
            { { -25.0f, 0.0f }, 4.0f },
            { { 28.0f, -20.0f }, 4.0f }
        };
    } else { // ACIDALIA_PLANITIA
        boulders = {
            { { 20.0f, 20.0f }, 2.4f },
            { { -30.0f, -25.0f }, 2.8f },
            { { 40.0f, -40.0f }, 2.2f },
            { { -15.0f, 45.0f }, 2.5f }
        };
    }

    for (const auto& bp : boulders) {
        float h = terrain.getHeight(bp.pos.x, bp.pos.y);
        Vector3 boulderPos = { bp.pos.x, h + bp.radius * 0.70f, bp.pos.y };
        physics.spawnBoulder(boulderPos, bp.radius);
    }

    // Drape 3D NavGraph (26x26 = 676 nodes)
    navGraph.generateTerrainGrid(terrain, 26, 26, 7.0f);
    navGraph.validateEdgesWithPhysics(physics, 0.6f);

    blockedEdgeCount = 0;
    for (const auto& e : navGraph.getEdges()) {
        if (e.isBlocked) blockedEdgeCount++;
    }

    // Solve Dijkstra route with history
    dijkstra.solveWithHistory(navGraph.getStartNode(), navGraph.getEndNode(),
                              navGraph.getVertices(), navGraph.getBlockedEdgesMap());

    // Reset Rover at valid start node
    if (navGraph.getStartNode()) {
        rover.reset(physics, navGraph.getStartNode()->position);
        rover.setPath(dijkstra.getShortestPathNodes());
    }
}

int main() {
    // 1. High-DPI Window Initialization (Native Retina resolution on Apple Silicon)
    const int screenWidth = 1280;
    const int screenHeight = 720;

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI);
    InitWindow(screenWidth, screenHeight, "3D Planetary Rover Simulator - Martian Terrain & Physics");
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

    // 4. Procedural Martian Terrain Heightfield
    TerrainHeightfield terrain(128, 200.0f);

    // 5. 3D NavGraph
    RoverNavGraph navGraph;

    // 6. Physics-Weighted 3D Dijkstra Solver with Snapshot History
    DijkstraSolver3D dijkstra;
    int currentPresetIndex = 0;
    dijkstra.applyPreset(currentPresetIndex);

    // 7. Planetary Rover Rig
    PlanetaryRover rover;

    // 8. Initialize Default Scenario (The Olympus Crater)
    int blockedEdgeCount = 0;
    ApplyTerrainPreset(TerrainPreset::OLYMPUS_CRATER, terrain, physics, navGraph, dijkstra, rover, blockedEdgeCount);

    // Initial rolling test sphere on slope
    physics.spawnDynamicSphere(Vector3{ 20.0f, terrain.getHeight(20.0f, -10.0f) + 6.0f, -10.0f }, 1.2f, 50.0f);

    // Display options
    bool showTerrain = true;
    bool showEdges = true;
    bool showNodes = true;
    bool showBoulders = true;
    bool showWireframe = false;
    bool showHUD = true;

    // Distant Martian pale blue sun position
    Vector3 sunPosition = { 160.0f, 110.0f, -130.0f };

    // 9. Main Simulation Loop
    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt > 0.05f) dt = 0.05f;

        // Step Jolt Physics (60 Hz multi-threaded)
        physics.step(dt);

        // Update Autonomous Planetary Rover & Pure Pursuit Navigation
        rover.update(physics, dt);

        // Active Camera (Smoothly blends Orbit, Chase, or Mast Camera)
        Camera3D activeCamera = rover.getCamera(cameraController.getCamera());

        // Check if cursor is over any active 2D HUD cards
        Vector2 mousePos = GetMousePosition();
        int screenW = GetScreenWidth();
        int screenH = GetScreenHeight();

        bool isOverUI = false;
        if (showHUD) {
            Rectangle hudRect = { 16.0f, 16.0f, 440.0f, 326.0f };
            Rectangle legRect = { 16.0f, 326.0f + 16.0f + 8.0f, 440.0f, 88.0f };
            int deckH = 92;
            Rectangle deckRect = { 16.0f, (float)(screenH - deckH - 16), (float)(screenW - 32), (float)deckH };
            Rectangle telemRect = { (float)(screenW - 340 - 16), 16.0f, 340.0f, 505.0f };

            if (CheckCollisionPointRec(mousePos, hudRect) ||
                CheckCollisionPointRec(mousePos, legRect) ||
                CheckCollisionPointRec(mousePos, deckRect) ||
                CheckCollisionPointRec(mousePos, telemRect)) {
                isOverUI = true;
            }
        } else {
            Rectangle btnRect = { (float)(screenW - 150 - 16), 16.0f, 150.0f, 32.0f };
            if (CheckCollisionPointRec(mousePos, btnRect)) {
                isOverUI = true;
            }
        }

        // Update Orbital Camera (only when in Orbit mode, ignoring inputs over UI cards)
        if (rover.getCameraMode() == RoverCameraMode::ORBIT) {
            cameraController.update(isOverUI);
        }

        // Update Dijkstra Step-by-Step Playback
        dijkstra.update(dt);

        // 3D Raycast Mouse Picking for Nodes (prevent picking if clicking on UI cards)
        Vertex3D* hoveredNode = nullptr;
        if (!isOverUI) {
            Ray mouseRay = GetMouseRay(mousePos, activeCamera);
            hoveredNode = navGraph.pickNodeFromRay(mouseRay, 1.8f);

            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                if (hoveredNode) {
                    if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                        navGraph.setEndNode(hoveredNode);
                    } else {
                        navGraph.setStartNode(hoveredNode);
                    }
                    dijkstra.solveWithHistory(navGraph.getStartNode(), navGraph.getEndNode(), 
                                              navGraph.getVertices(), navGraph.getBlockedEdgesMap());
                    rover.setPath(dijkstra.getShortestPathNodes());
                }
            }
        }

        // Cost Preset Selection Keys: 1, 2, 3, 4
        if (IsKeyPressed(KEY_ONE)) {
            currentPresetIndex = 0;
            dijkstra.applyPreset(0);
            dijkstra.solveWithHistory(navGraph.getStartNode(), navGraph.getEndNode(), 
                                      navGraph.getVertices(), navGraph.getBlockedEdgesMap());
            rover.setPath(dijkstra.getShortestPathNodes());
        }
        if (IsKeyPressed(KEY_TWO)) {
            currentPresetIndex = 1;
            dijkstra.applyPreset(1);
            dijkstra.solveWithHistory(navGraph.getStartNode(), navGraph.getEndNode(), 
                                      navGraph.getVertices(), navGraph.getBlockedEdgesMap());
            rover.setPath(dijkstra.getShortestPathNodes());
        }
        if (IsKeyPressed(KEY_THREE)) {
            currentPresetIndex = 2;
            dijkstra.applyPreset(2);
            dijkstra.solveWithHistory(navGraph.getStartNode(), navGraph.getEndNode(), 
                                      navGraph.getVertices(), navGraph.getBlockedEdgesMap());
            rover.setPath(dijkstra.getShortestPathNodes());
        }
        if (IsKeyPressed(KEY_FOUR)) {
            currentPresetIndex = 3;
            dijkstra.applyPreset(3);
            dijkstra.solveWithHistory(navGraph.getStartNode(), navGraph.getEndNode(), 
                                      navGraph.getVertices(), navGraph.getBlockedEdgesMap());
            rover.setPath(dijkstra.getShortestPathNodes());
        }

        // Phase 5: Planetary Rover Driving Controls & Camera Focus
        if (IsKeyPressed(KEY_TAB)) {
            rover.toggleAutonomous();
        }
        if (IsKeyPressed(KEY_F)) {
            cameraController.focusOn(rover.getPosition(), 22.0f);
        }
        if (IsKeyPressed(KEY_M)) {
            terrain.cyclePreset();
            ApplyTerrainPreset(terrain.getPreset(), terrain, physics, navGraph, dijkstra, rover, blockedEdgeCount);
        }
        if (IsKeyPressed(KEY_V)) {
            rover.cycleCameraMode();
        }
        if (IsKeyPressed(KEY_R)) {
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                cameraController.reset(Vector3{ 0.0f, 75.0f, 115.0f }, Vector3{ 0.0f, 0.0f, 0.0f });
            } else if (navGraph.getStartNode()) {
                rover.reset(physics, navGraph.getStartNode()->position);
                rover.setPath(dijkstra.getShortestPathNodes());
            }
        }
        if (IsKeyPressed(KEY_U)) {
            rover.selfRight(physics);
        }
        if (IsKeyPressed(KEY_G)) {
            if (physics.getGravity() < -6.0f) {
                physics.setGravity(-3.71f); // Martian Gravity
            } else {
                physics.setGravity(-9.81f); // Earth Gravity
            }
        }

        // Manual Driving Overrides (when Autonomous Pure Pursuit is paused)
        if (!rover.isAutonomous()) {
            float manualThrottle = 0.0f;
            float manualSteer = 0.0f;
            float manualBrake = 0.0f;
            if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) manualThrottle += 1.0f;
            if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) manualThrottle -= 0.6f;
            if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) manualSteer -= 0.60f;
            if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) manualSteer += 0.60f;
            if (IsKeyDown(KEY_SPACE)) manualBrake = 1.0f;

            rover.setThrottleInput(manualThrottle);
            rover.setSteeringInput(manualSteer);
            rover.setBrakeInput(manualBrake);
        }

        // Dijkstra Algorithm Playback & Scrubbing Keys
        if (IsKeyPressed(KEY_P)) {
            dijkstra.togglePlay();
        }
        if (IsKeyPressed(KEY_RIGHT)) {
            dijkstra.stepForward();
        }
        if (IsKeyPressed(KEY_LEFT)) {
            dijkstra.stepBackward();
        }
        if (IsKeyPressed(KEY_ENTER)) {
            dijkstra.jumpToEnd();
            rover.setPath(dijkstra.getShortestPathNodes());
        }
        if (IsKeyPressed(KEY_BACKSPACE)) {
            dijkstra.jumpToStart();
        }

        // Physics Actions
        if (IsKeyPressed(KEY_B)) {
            Vector3 dropPos;
            if (hoveredNode) {
                dropPos = Vector3Add(hoveredNode->position, Vector3{ 0.0f, 4.0f, 0.0f });
            } else {
                dropPos = Vector3{ 0.0f, terrain.getHeight(0.0f, 0.0f) + 8.0f, 0.0f };
            }
            physics.spawnDynamicSphere(dropPos, 1.2f, 50.0f);
        }

        // X key: Cascade multiple spheres down crater
        if (IsKeyPressed(KEY_X)) {
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

        // Keyboard Display Controls
        if (IsKeyPressed(KEY_T)) showTerrain = !showTerrain;
        if (IsKeyPressed(KEY_E)) showEdges = !showEdges;
        if (IsKeyPressed(KEY_N)) showNodes = !showNodes;
        if (IsKeyPressed(KEY_O)) showBoulders = !showBoulders;
        if (IsKeyPressed(KEY_W)) showWireframe = !showWireframe;
        if (IsKeyPressed(KEY_H)) showHUD = !showHUD;

        // Render Frame
        BeginDrawing();

        // 1. Explicitly clear Color and Depth buffers (essential for 3D camera rotation)
        ClearBackground(Color{ 12, 14, 24, 255 });

        // 2. Atmospheric Sky Gradient (2D background with depth testing disabled)
        rlDisableDepthMask();
        rlDisableDepthTest();
        DrawRectangleGradientV(0, 0, GetScreenWidth(), GetScreenHeight(), 
            Color{ 12, 14, 24, 255 }, 
            Color{ 72, 38, 30, 255 }
        );
        rlEnableDepthTest();
        rlEnableDepthMask();

        float sceneTime = static_cast<float>(GetTime());

        // 3D Scene Rendering
        BeginMode3D(activeCamera);
        {
            // A. Distant Martian Sun (pale blue disk with soft halo)
            DrawSphere(sunPosition, 6.0f, Color{ 190, 225, 255, 255 });
            DrawSphereWires(sunPosition, 9.0f, 6, 6, ColorAlpha(Color{ 150, 200, 255, 255 }, 0.4f));

            // B. Draw Procedural Martian Terrain Mesh with High-Definition Surface Texture
            if (showTerrain && terrain.isLoaded()) {
                rlDisableBackfaceCulling();
                DrawModel(terrain.getModel(), Vector3{ 0.0f, 0.0f, 0.0f }, 1.0f, WHITE);
                rlEnableBackfaceCulling();

                if (showWireframe) {
                    DrawModelWires(terrain.getModel(), Vector3{ 0.0f, 0.0f, 0.0f }, 1.0f, ColorAlpha(BLACK, 0.2f));
                }
            }

            // C. Draw Static Boulders (Craggy rock shading with sunlit facets)
            if (showBoulders) {
                for (const auto& b : physics.getBoulders()) {
                    DrawSphere(b.pos, b.radius, Color{ 78, 56, 48, 255 });
                    DrawSphere(Vector3Add(b.pos, Vector3{ 0.0f, b.radius * 0.18f, 0.0f }), b.radius * 0.88f, Color{ 115, 88, 76, 255 });
                    DrawSphereWires(b.pos, b.radius, 6, 6, ColorAlpha(Color{ 35, 24, 20, 255 }, 0.45f));
                }
            }

            // D. Draw GPU-Batched Draped NavGraph Edges
            if (showEdges) {
                navGraph.renderEdges();
            }

            // E. Phase 4: Draw Dijkstra Step-by-Step Search State (Current Node u, Examining Cyan Beam, Visited Nodes)
            GraphRenderer3D::drawSearchState(dijkstra.getCurrentSnapshot(), navGraph.getVertices(), sceneTime);

            // F. Phase 4: Draw Brilliant Glowing Emerald Shortest Path along Terrain Surface
            GraphRenderer3D::drawShortestPath(dijkstra.getShortestPathNodes(), sceneTime);

            // G. Draw Draped NavGraph Nodes (Sleek compact dots for regular nodes, beacons for Start/End)
            if (showNodes) {
                for (const Vertex3D* v : navGraph.getVertices()) {
                    if (v == navGraph.getStartNode() || v == navGraph.getEndNode()) {
                        continue; // Drawn prominently below
                    }
                    Color nodeCol = GraphRenderer3D::getNodeColor(v->state);
                    float r = (v->state == NodeState::IMPASSABLE) ? 0.30f : 0.38f;
                    DrawSphere(v->position, r, nodeCol);
                }

                // Prominent START Beacon with vertical light pillar and pulsating ground ring
                if (const Vertex3D* s = navGraph.getStartNode()) {
                    Vector3 pillarTop = Vector3Add(s->position, Vector3{ 0.0f, 16.0f, 0.0f });
                    DrawCylinderEx(s->position, pillarTop, 0.25f, 0.02f, 8, ColorAlpha(GREEN, 0.75f));
                    DrawSphere(s->position, 1.4f, Color{ 46, 204, 113, 255 });
                    DrawSphereWires(s->position, 1.4f, 8, 8, ColorAlpha(WHITE, 0.85f));
                    float pulseR = 2.4f + sinf(sceneTime * 4.0f) * 0.5f;
                    DrawCircle3D(s->position, pulseR, Vector3{ 0, 1, 0 }, 90.0f, ColorAlpha(GREEN, 0.7f));
                }

                // Prominent END Beacon with vertical light pillar and pulsating ground ring
                if (const Vertex3D* e = navGraph.getEndNode()) {
                    Vector3 pillarTop = Vector3Add(e->position, Vector3{ 0.0f, 16.0f, 0.0f });
                    DrawCylinderEx(e->position, pillarTop, 0.25f, 0.02f, 8, ColorAlpha(RED, 0.75f));
                    DrawSphere(e->position, 1.4f, Color{ 231, 76, 60, 255 });
                    DrawSphereWires(e->position, 1.4f, 8, 8, ColorAlpha(WHITE, 0.85f));
                    float pulseR = 2.4f + sinf(sceneTime * 4.0f + 1.5f) * 0.5f;
                    DrawCircle3D(e->position, pulseR, Vector3{ 0, 1, 0 }, 90.0f, ColorAlpha(RED, 0.7f));
                }
            }

            // H. Phase 5: Autonomous Planetary Rover Physical Rig & 3D Model
            rover.render(sceneTime);

            // I. Draw Dynamic Rolling Test Spheres (Synchronized with Jolt Physics rigid bodies)
            for (auto sphereId : physics.getDynamicSpheres()) {
                Vector3 pos = physics.getBodyPosition(sphereId);
                DrawSphere(pos, 1.2f, Color{ 0, 185, 255, 255 });
                DrawSphereWires(pos, 1.2f, 8, 8, ColorAlpha(WHITE, 0.75f));
            }

            // J. Highlight Hovered Node with Targeting Ring
            if (hoveredNode) {
                DrawSphereWires(hoveredNode->position, 1.8f, 10, 10, GOLD);
                DrawCircle3D(hoveredNode->position, 2.2f, Vector3{ 0.0f, 1.0f, 0.0f }, 90.0f, ColorAlpha(YELLOW, 0.8f));
            }
        }
        EndMode3D();

        // 2D HUD & Mission Control Overlay
        if (showHUD) {
            int screenW = GetScreenWidth();
            int screenH = GetScreenHeight();
            Vector2 mousePos = GetMousePosition();

            // ----------------------------------------------------
            // 1. TOP-LEFT: Mission Status & Telemetry HUD Card
            // ----------------------------------------------------
            int hudX = 16;
            int hudY = 16;
            int hudW = 440;
            int hudH = 326;
            Rectangle hudRect = { (float)hudX, (float)hudY, (float)hudW, (float)hudH };
            DrawRectangleRounded(hudRect, 0.04f, 4, ColorAlpha(Color{ 10, 14, 24, 255 }, 0.92f));
            DrawRectangleRoundedLines(hudRect, 0.04f, 4, Color{ 48, 68, 98, 255 });
            DrawRectangle(hudX + 1, hudY + 1, hudW - 2, 3, Color{ 230, 95, 45, 255 }); // Martian Ochre accent

            DrawText("MARTIAN ROVER MISSION TELEMETRY", hudX + 16, hudY + 14, 13, RAYWHITE);
            DrawText("Phase 5: Autonomous Planetary Rover & Telemetry HUD", hudX + 16, hudY + 32, 11, Color{ 145, 175, 205, 255 });

            // Clickable [H] Hide HUD button
            Rectangle hideBtnRect = { (float)(hudX + hudW - 85), (float)(hudY + 12), 70.0f, 18.0f };
            bool hideHovered = CheckCollisionPointRec(mousePos, hideBtnRect);
            DrawRectangleRounded(hideBtnRect, 0.35f, 4, hideHovered ? Color{ 40, 56, 80, 255 } : Color{ 24, 34, 50, 220 });
            DrawRectangleRoundedLines(hideBtnRect, 0.35f, 4, hideHovered ? Color{ 100, 160, 240, 255 } : Color{ 60, 85, 120, 200 });
            DrawText("[H] Hide", hudX + hudW - 77, hudY + 16, 10, Color{ 200, 225, 255, 255 });
            if (hideHovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                showHUD = false;
            }

            // Row 1: Interactive Terrain Mode Switcher [M]
            Rectangle tmRect = { (float)(hudX + 16), (float)(hudY + 52), (float)(hudW - 32), 22.0f };
            bool tmHovered = CheckCollisionPointRec(mousePos, tmRect);
            DrawRectangleRounded(tmRect, 0.25f, 4, tmHovered ? Color{ 36, 52, 78, 255 } : Color{ 20, 28, 44, 255 });
            DrawRectangleRoundedLines(tmRect, 0.25f, 4, tmHovered ? Color{ 90, 170, 255, 255 } : Color{ 50, 75, 110, 255 });
            DrawText(TextFormat("TERRAIN MODE [M]: %s", terrain.getPresetName()), hudX + 24, hudY + 57, 11, Color{ 100, 215, 255, 255 });
            DrawText("[Click / M to Cycle]", hudX + hudW - 130, hudY + 58, 9, Color{ 150, 175, 205, 255 });
            if (tmHovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                terrain.cyclePreset();
                ApplyTerrainPreset(terrain.getPreset(), terrain, physics, navGraph, dijkstra, rover, blockedEdgeCount);
            }

            DrawLine(hudX + 16, hudY + 82, hudX + hudW - 16, hudY + 82, Color{ 35, 48, 70, 255 });

            // Row 2: Active Cost Preset & Parameters
            const auto& w = dijkstra.getWeights();
            DrawText(TextFormat("Cost Preset [%d]: %s", currentPresetIndex + 1, w.name.c_str()), 
                hudX + 16, hudY + 90, 11, Color{ 240, 200, 80, 255 });
            DrawText(TextFormat("Weights: alpha=%.1f (Work) | beta=%.1f (Slip) | gamma=%.1f | delta=%.1f", 
                w.alpha, w.beta, w.gamma, w.delta), hudX + 16, hudY + 106, 10, Color{ 160, 175, 195, 255 });

            DrawLine(hudX + 16, hudY + 122, hudX + hudW - 16, hudY + 122, Color{ 35, 48, 70, 255 });

            // Row 3: Physics & NavGraph Infrastructure Telemetry
            DrawText(TextFormat("Jolt Engine: %d Boulders | %d Cut Edges | Gravity: %.2f m/s²",
                (int)physics.getBoulders().size(), blockedEdgeCount, physics.getGravity()),
                hudX + 16, hudY + 130, 11, RAYWHITE);

            // Row 4: Dijkstra Navigation Solution Status
            size_t stepIdx = dijkstra.getCurrentStepIndex();
            size_t totalSteps = dijkstra.getTotalSteps();
            const auto& stats = dijkstra.getPathStats();

            if (dijkstra.isPlaying()) {
                float pct = (totalSteps > 0) ? (static_cast<float>(stepIdx) / totalSteps * 100.0f) : 0.0f;
                DrawText(TextFormat("STATUS: [REPLAYING STEP %zu / %zu (%.0f%%)]", stepIdx + 1, totalSteps, pct), 
                    hudX + 16, hudY + 150, 12, GOLD);
            } else if (dijkstra.isPathFound()) {
                DrawText(TextFormat("STATUS: [OPTIMAL ROUTE SOLVED in %.2f ms]", stats.computeTimeMs), 
                    hudX + 16, hudY + 150, 12, Color{ 46, 204, 113, 255 });
            } else {
                DrawText("STATUS: [NO TRAVERSABLE PATH - SLIP / BOULDER BLOCKED]", 
                    hudX + 16, hudY + 150, 12, Color{ 255, 95, 95, 255 });
            }

            // Row 5: Route Metrics
            if (stats.isValid) {
                DrawText(TextFormat("3D Distance: %.1f m   |   Physical Energy: %.1f J-equiv", 
                    stats.totalDistance, stats.totalEnergyCost), hudX + 16, hudY + 172, 11, RAYWHITE);
                DrawText(TextFormat("Waypoints: %d nodes    |   Max Route Slope: %.1f°", 
                    stats.waypointCount, stats.maxSlopeDeg), hudX + 16, hudY + 190, 11, RAYWHITE);
                DrawText(TextFormat("Elevation Profile: +%.1fm climb  /  -%.1fm descent", 
                    stats.elevationGain, stats.elevationLoss), hudX + 16, hudY + 208, 11, Color{ 180, 200, 220, 255 });
            } else {
                DrawText("Target destination is unreachable with current physical constraints.", 
                    hudX + 16, hudY + 172, 11, Color{ 255, 140, 140, 255 });
            }

            DrawLine(hudX + 16, hudY + 226, hudX + hudW - 16, hudY + 226, Color{ 35, 48, 70, 255 });

            // Waypoints & Interactive Picking Telemetry
            DrawCircle(hudX + 22, hudY + 240, 5, GraphRenderer3D::getNodeColor(NodeState::START));
            std::string startInfo = navGraph.getStartNode() 
                ? TextFormat("Start: %s  (y=%.1fm, slope=%.1f°)", 
                    navGraph.getStartNode()->name.c_str(), 
                    navGraph.getStartNode()->position.y,
                    navGraph.getStartNode()->slopeAngleRad * RAD2DEG)
                : "Start: None [Left Click to set]";
            DrawText(startInfo.c_str(), hudX + 34, hudY + 235, 11, RAYWHITE);

            DrawCircle(hudX + 22, hudY + 260, 5, GraphRenderer3D::getNodeColor(NodeState::END));
            std::string endInfo = navGraph.getEndNode() 
                ? TextFormat("End:   %s  (y=%.1fm, slope=%.1f°)", 
                    navGraph.getEndNode()->name.c_str(), 
                    navGraph.getEndNode()->position.y,
                    navGraph.getEndNode()->slopeAngleRad * RAD2DEG)
                : "End:   None [Shift + Left Click to set]";
            DrawText(endInfo.c_str(), hudX + 34, hudY + 255, 11, RAYWHITE);

            // Replay Scrubbing Progress Bar
            int barW = hudW - 32;
            int barH = 6;
            int barX = hudX + 16;
            int barY = hudY + 284;
            DrawRectangle(barX, barY, barW, barH, Color{ 25, 35, 52, 255 });
            float progress = (totalSteps > 1) ? (static_cast<float>(stepIdx) / (totalSteps - 1)) : 1.0f;
            DrawRectangle(barX, barY, static_cast<int>(barW * progress), barH, Color{ 46, 204, 113, 255 });
            DrawCircle(barX + static_cast<int>(barW * progress), barY + 3, 5, WHITE);
            DrawText(TextFormat("Step %zu of %zu  |  FPS: %i", stepIdx + 1, totalSteps, GetFPS()), 
                hudX + 16, hudY + 298, 10, Color{ 140, 155, 175, 255 });

            // ----------------------------------------------------
            // 2. BOTTOM-LEFT: NavGraph & Routing Legend Card
            // ----------------------------------------------------
            int legW = 440;
            int legH = 88;
            int legX = 16;
            int legY = hudY + hudH + 8;
            Rectangle legRect = { (float)legX, (float)legY, (float)legW, (float)legH };
            DrawRectangleRounded(legRect, 0.05f, 4, ColorAlpha(Color{ 10, 14, 24, 255 }, 0.92f));
            DrawRectangleRoundedLines(legRect, 0.05f, 4, Color{ 48, 68, 98, 255 });
            DrawRectangle(legX + 1, legY + 1, legW - 2, 2, Color{ 46, 204, 113, 255 });

            DrawText("NAVGRAPH & DIJKSTRA ROUTING LEGEND", legX + 16, legY + 9, 11, RAYWHITE);
            DrawLine(legX + 16, legY + 24, legX + legW - 16, legY + 24, Color{ 35, 48, 70, 255 });

            // Column 1
            DrawRectangle(legX + 16, legY + 34, 14, 4, Color{ 46, 204, 113, 255 });
            DrawText("Shortest Path Conduit", legX + 36, legY + 30, 10, Color{ 180, 255, 200, 255 });

            DrawCircle(legX + 23, legY + 52, 4, GOLD);
            DrawText("Active Node u (Settling)", legX + 36, legY + 47, 10, GOLD);

            DrawRectangle(legX + 16, legY + 68, 14, 3, Color{ 0, 240, 255, 255 });
            DrawText("Examining Edge (u -> v)", legX + 36, legY + 63, 10, Color{ 0, 240, 255, 255 });

            // Column 2
            int legCol2X = legX + 225;
            DrawCircle(legCol2X + 7, legY + 34, 4, Color{ 52, 152, 219, 255 });
            DrawText("Settled Node", legCol2X + 20, legY + 30, 10, RAYWHITE);

            DrawRectangle(legCol2X, legY + 50, 14, 3, Color{ 240, 45, 45, 255 });
            DrawText("Blocked by Boulder Cut", legCol2X + 20, legY + 47, 10, Color{ 255, 120, 120, 255 });

            DrawCircle(legCol2X + 7, legY + 68, 4, GraphRenderer3D::getNodeColor(NodeState::START));
            DrawText("Start / End Beacons", legCol2X + 20, legY + 63, 10, RAYWHITE);

            // ----------------------------------------------------
            // 3. TOP-RIGHT: Planetary Rover Flight Telemetry HUD
            // ----------------------------------------------------
            RoverTelemetryHUD::draw(rover, screenW, screenH, sceneTime);

            // ----------------------------------------------------
            // 4. BOTTOM COMMAND DECK & KEYBINDINGS BAR
            // ----------------------------------------------------
            int deckH = 92;
            int deckW = screenW - 32;
            int deckX = 16;
            int deckY = screenH - deckH - 16;
            Rectangle deckRect = { (float)deckX, (float)deckY, (float)deckW, (float)deckH };
            DrawRectangleRounded(deckRect, 0.04f, 4, ColorAlpha(Color{ 10, 14, 24, 255 }, 0.94f));
            DrawRectangleRoundedLines(deckRect, 0.04f, 4, Color{ 48, 68, 98, 255 });
            DrawRectangle(deckX + 1, deckY + 1, deckW - 2, 2, Color{ 70, 115, 175, 255 }); // Slate blue accent

            float colW = (deckW - 32.0f) / 4.0f;

            // --- Column 1: Camera Controls ---
            int c1X = deckX + 16;
            DrawText("CAMERA CONTROLS", c1X, deckY + 10, 10, Color{ 100, 185, 255, 255 });
            DrawKeyBind(c1X, deckY + 28, "RMB Drag", "Orbit");
            DrawKeyBind(c1X + 105, deckY + 28, "Wheel", "Zoom to Cursor");
            DrawKeyBind(c1X, deckY + 54, "MMB / Shift+RMB", "Pan");
            DrawKeyBind(c1X + 145, deckY + 54, "F", "Focus Rover");

            // --- Column 2: Rover Navigation ---
            int c2X = deckX + 16 + static_cast<int>(colW);
            DrawLine(c2X - 12, deckY + 10, c2X - 12, deckY + deckH - 10, Color{ 35, 48, 70, 255 });
            DrawText("ROVER NAVIGATION", c2X, deckY + 10, 10, Color{ 46, 204, 113, 255 });
            DrawKeyBind(c2X, deckY + 28, "Tab", rover.isAutonomous() ? "Pause Auto" : "Auto Drive");
            DrawKeyBind(c2X + 115, deckY + 28, "WASD", "Manual Steer");
            DrawKeyBind(c2X, deckY + 54, "R", "Reset");
            DrawKeyBind(c2X + 80, deckY + 54, "U", "Self-Right");
            DrawKeyBind(c2X + 165, deckY + 54, "G", (physics.getGravity() < -6.0f) ? "Earth G" : "Mars G");

            // --- Column 3: Terrain & Dijkstra ---
            int c3X = deckX + 16 + static_cast<int>(colW * 2);
            DrawLine(c3X - 12, deckY + 10, c3X - 12, deckY + deckH - 10, Color{ 35, 48, 70, 255 });
            DrawText("TERRAIN & DIJKSTRA", c3X, deckY + 10, 10, Color{ 241, 196, 15, 255 });
            DrawKeyBind(c3X, deckY + 28, "M", "Terrain Mode");
            DrawKeyBind(c3X + 115, deckY + 28, "1-4", "Cost Presets");
            DrawKeyBind(c3X, deckY + 54, "P", dijkstra.isPlaying() ? "Pause" : "Play Replay");
            DrawKeyBind(c3X + 115, deckY + 54, "Enter", "Finish Path");

            // --- Column 4: View & HUD Toggles ---
            int c4X = deckX + 16 + static_cast<int>(colW * 3);
            DrawLine(c4X - 12, deckY + 10, c4X - 12, deckY + deckH - 10, Color{ 35, 48, 70, 255 });
            DrawText("VIEW & HUD TOGGLES", c4X, deckY + 10, 10, Color{ 190, 160, 240, 255 });
            DrawKeyBind(c4X, deckY + 28, "H", "Toggle HUD");
            DrawKeyBind(c4X + 105, deckY + 28, "V", "Cam Mode");
            DrawKeyBind(c4X, deckY + 54, "B / X", "Drop Rocks");
            DrawKeyBind(c4X + 90, deckY + 54, "C", "Clear");
            DrawKeyBind(c4X + 155, deckY + 54, "T", "Terrain");
        } else {
            // Interactive floating button when HUD is hidden
            int btnW = 150;
            int btnH = 32;
            int btnX = GetScreenWidth() - btnW - 16;
            int btnY = 16;
            Rectangle btnRect = { (float)btnX, (float)btnY, (float)btnW, (float)btnH };
            Vector2 mPos = GetMousePosition();
            bool isHovered = CheckCollisionPointRec(mPos, btnRect);

            Color bg = isHovered ? Color{ 36, 52, 78, 240 } : Color{ 16, 22, 34, 210 };
            Color border = isHovered ? Color{ 80, 160, 255, 255 } : Color{ 48, 70, 105, 220 };
            DrawRectangleRounded(btnRect, 0.35f, 4, bg);
            DrawRectangleRoundedLines(btnRect, 0.35f, 4, border);

            DrawCircle(btnX + 16, btnY + 16, 4, Color{ 46, 204, 113, 255 });
            DrawText("[H] SHOW HUD", btnX + 28, btnY + 10, 11, RAYWHITE);

            if (isHovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                showHUD = true;
            }
        }

        EndDrawing();
    }

    // Cleanup & Exit
    physics.shutdown();
    terrain.unload();
    navGraph.clear();
    CloseWindow();
    return 0;
}
