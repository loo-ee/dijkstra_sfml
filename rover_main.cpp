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
        Vector3 boulderPos = { bp.pos.x, h + bp.radius * 0.70f, bp.pos.y };
        physics.spawnBoulder(boulderPos, bp.radius);
    }

    // 7. Drape 3D NavGraph over Terrain (26x26 = 676 nodes, > 500 nodes criteria)
    const int navCols = 26;
    const int navRows = 26;
    const float navSpacing = 7.0f;

    RoverNavGraph navGraph;
    navGraph.generateTerrainGrid(terrain, navCols, navRows, navSpacing);

    // 8. Line-of-Sight Edge Validation using Physical Raycasts
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

    // 10. Phase 4: Physics-Weighted 3D Dijkstra Solver with Snapshot History
    DijkstraSolver3D dijkstra;
    int currentPresetIndex = 0;
    dijkstra.applyPreset(currentPresetIndex);
    dijkstra.solveWithHistory(navGraph.getStartNode(), navGraph.getEndNode(), 
                              navGraph.getVertices(), navGraph.getBlockedEdgesMap());

    // Display options
    bool showTerrain = true;
    bool showEdges = true;
    bool showNodes = true;
    bool showBoulders = true;
    bool showWireframe = false;
    bool showHUD = true;

    // Distant Martian pale blue sun position
    Vector3 sunPosition = { 160.0f, 110.0f, -130.0f };

    // 11. Main Simulation Loop
    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt > 0.05f) dt = 0.05f;

        // Step Jolt Physics (60 Hz multi-threaded)
        physics.step(dt);

        // Update Orbital Camera
        cameraController.update();

        // Update Dijkstra Step-by-Step Playback
        dijkstra.update(dt);

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
                dijkstra.solveWithHistory(navGraph.getStartNode(), navGraph.getEndNode(), 
                                          navGraph.getVertices(), navGraph.getBlockedEdgesMap());
            }
        }

        // Cost Preset Selection Keys: 1, 2, 3, 4
        if (IsKeyPressed(KEY_ONE)) {
            currentPresetIndex = 0;
            dijkstra.applyPreset(0);
            dijkstra.solveWithHistory(navGraph.getStartNode(), navGraph.getEndNode(), 
                                      navGraph.getVertices(), navGraph.getBlockedEdgesMap());
        }
        if (IsKeyPressed(KEY_TWO)) {
            currentPresetIndex = 1;
            dijkstra.applyPreset(1);
            dijkstra.solveWithHistory(navGraph.getStartNode(), navGraph.getEndNode(), 
                                      navGraph.getVertices(), navGraph.getBlockedEdgesMap());
        }
        if (IsKeyPressed(KEY_THREE)) {
            currentPresetIndex = 2;
            dijkstra.applyPreset(2);
            dijkstra.solveWithHistory(navGraph.getStartNode(), navGraph.getEndNode(), 
                                      navGraph.getVertices(), navGraph.getBlockedEdgesMap());
        }
        if (IsKeyPressed(KEY_FOUR)) {
            currentPresetIndex = 3;
            dijkstra.applyPreset(3);
            dijkstra.solveWithHistory(navGraph.getStartNode(), navGraph.getEndNode(), 
                                      navGraph.getVertices(), navGraph.getBlockedEdgesMap());
        }

        // Dijkstra Algorithm Playback & Scrubbing Keys
        if (IsKeyPressed(KEY_SPACE) || IsKeyPressed(KEY_P)) {
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
        if (IsKeyPressed(KEY_R)) {
            cameraController.reset(Vector3{ 0.0f, 75.0f, 115.0f }, Vector3{ 0.0f, 0.0f, 0.0f });
        }
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

        // 3D Scene Rendering
        BeginMode3D(cameraController.getCamera());
        {
            float sceneTime = static_cast<float>(GetTime());

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

            // H. Draw Dynamic Rolling Test Spheres (Synchronized with Jolt Physics rigid bodies)
            for (auto sphereId : physics.getDynamicSpheres()) {
                Vector3 pos = physics.getBodyPosition(sphereId);
                DrawSphere(pos, 1.2f, Color{ 0, 185, 255, 255 });
                DrawSphereWires(pos, 1.2f, 8, 8, ColorAlpha(WHITE, 0.75f));
            }

            // I. Highlight Hovered Node with Targeting Ring
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

            // ----------------------------------------------------
            // 1. TOP-LEFT: Mission Status & Telemetry HUD Card
            // ----------------------------------------------------
            int hudX = 16;
            int hudY = 16;
            int hudW = 440;
            int hudH = 310;
            Rectangle hudRect = { (float)hudX, (float)hudY, (float)hudW, (float)hudH };
            DrawRectangleRounded(hudRect, 0.04f, 4, ColorAlpha(Color{ 10, 14, 24, 255 }, 0.92f));
            DrawRectangleRoundedLines(hudRect, 0.04f, 4, Color{ 48, 68, 98, 255 });
            DrawRectangle(hudX + 1, hudY + 1, hudW - 2, 3, Color{ 230, 95, 45, 255 }); // Martian Ochre accent

            DrawText("MARTIAN ROVER MISSION TELEMETRY", hudX + 16, hudY + 14, 13, RAYWHITE);
            DrawText("Phase 4: Physics-Weighted Dijkstra & Snapshots", hudX + 16, hudY + 32, 11, Color{ 145, 175, 205, 255 });

            // FPS & Physics Sub-step Badges
            int curFPS = GetFPS();
            Color fpsColor = (curFPS >= 55) ? Color{ 46, 204, 113, 255 } : Color{ 241, 196, 15, 255 };
            DrawText(TextFormat("FPS: %i", curFPS), hudX + hudW - 75, hudY + 14, 12, fpsColor);
            DrawText("60Hz Step", hudX + hudW - 75, hudY + 30, 10, Color{ 52, 152, 219, 255 });

            DrawLine(hudX + 16, hudY + 48, hudX + hudW - 16, hudY + 48, Color{ 35, 48, 70, 255 });

            // Row 1: Active Cost Preset & Parameters
            const auto& w = dijkstra.getWeights();
            DrawText(TextFormat("Preset [%d]: %s", currentPresetIndex + 1, w.name.c_str()), 
                hudX + 16, hudY + 56, 11, Color{ 100, 200, 255, 255 });
            DrawText(TextFormat("Weights: alpha=%.1f (Work) | beta=%.1f (Slip) | gamma=%.1f | delta=%.1f", 
                w.alpha, w.beta, w.gamma, w.delta), hudX + 16, hudY + 72, 10, Color{ 160, 175, 195, 255 });

            DrawLine(hudX + 16, hudY + 88, hudX + hudW - 16, hudY + 88, Color{ 35, 48, 70, 255 });

            // Row 2: Physics & NavGraph Infrastructure Telemetry
            DrawText(TextFormat("Jolt Engine: %d Boulders | %d Cut Edges | Martian Gravity (-3.71 m/s²)",
                (int)physics.getBoulders().size(), blockedEdgeCount),
                hudX + 16, hudY + 96, 11, RAYWHITE);

            // Row 3: Dijkstra Navigation Solution Status
            size_t stepIdx = dijkstra.getCurrentStepIndex();
            size_t totalSteps = dijkstra.getTotalSteps();
            const auto& stats = dijkstra.getPathStats();

            if (dijkstra.isPlaying()) {
                float pct = (totalSteps > 0) ? (static_cast<float>(stepIdx) / totalSteps * 100.0f) : 0.0f;
                DrawText(TextFormat("STATUS: [REPLAYING STEP %zu / %zu (%.0f%%)]", stepIdx + 1, totalSteps, pct), 
                    hudX + 16, hudY + 116, 12, GOLD);
            } else if (dijkstra.isPathFound()) {
                DrawText(TextFormat("STATUS: [OPTIMAL ROUTE SOLVED in %.2f ms]", stats.computeTimeMs), 
                    hudX + 16, hudY + 116, 12, Color{ 46, 204, 113, 255 });
            } else {
                DrawText("STATUS: [NO TRAVERSABLE PATH - SLIP / BOULDER BLOCKED]", 
                    hudX + 16, hudY + 116, 12, Color{ 255, 95, 95, 255 });
            }

            // Row 4: Route Metrics
            if (stats.isValid) {
                DrawText(TextFormat("3D Distance: %.1f m   |   Physical Energy: %.1f J-equiv", 
                    stats.totalDistance, stats.totalEnergyCost), hudX + 16, hudY + 138, 11, RAYWHITE);
                DrawText(TextFormat("Waypoints: %d nodes    |   Max Route Slope: %.1f°", 
                    stats.waypointCount, stats.maxSlopeDeg), hudX + 16, hudY + 156, 11, RAYWHITE);
                DrawText(TextFormat("Elevation Profile: +%.1fm climb  /  -%.1fm descent", 
                    stats.elevationGain, stats.elevationLoss), hudX + 16, hudY + 174, 11, Color{ 180, 200, 220, 255 });
            } else {
                DrawText("Target destination is unreachable with current physical constraints.", 
                    hudX + 16, hudY + 138, 11, Color{ 255, 140, 140, 255 });
            }

            DrawLine(hudX + 16, hudY + 194, hudX + hudW - 16, hudY + 194, Color{ 35, 48, 70, 255 });

            // Waypoints & Interactive Picking Telemetry
            DrawCircle(hudX + 22, hudY + 208, 5, GraphRenderer3D::getNodeColor(NodeState::START));
            std::string startInfo = navGraph.getStartNode() 
                ? TextFormat("Start: %s  (y=%.1fm, slope=%.1f°)", 
                    navGraph.getStartNode()->name.c_str(), 
                    navGraph.getStartNode()->position.y,
                    navGraph.getStartNode()->slopeAngleRad * RAD2DEG)
                : "Start: None [Left Click to set]";
            DrawText(startInfo.c_str(), hudX + 34, hudY + 203, 11, RAYWHITE);

            DrawCircle(hudX + 22, hudY + 228, 5, GraphRenderer3D::getNodeColor(NodeState::END));
            std::string endInfo = navGraph.getEndNode() 
                ? TextFormat("End:   %s  (y=%.1fm, slope=%.1f°)", 
                    navGraph.getEndNode()->name.c_str(), 
                    navGraph.getEndNode()->position.y,
                    navGraph.getEndNode()->slopeAngleRad * RAD2DEG)
                : "End:   None [Shift + Left Click to set]";
            DrawText(endInfo.c_str(), hudX + 34, hudY + 223, 11, RAYWHITE);

            // Row 5: Current Replay Step Message
            const auto& snap = dijkstra.getCurrentSnapshot();
            DrawText(snap.message.c_str(), hudX + 16, hudY + 252, 11, GOLD);

            // Row 6: Replay Scrubbing Progress Bar
            int barW = hudW - 32;
            int barH = 6;
            int barX = hudX + 16;
            int barY = hudY + 278;
            DrawRectangle(barX, barY, barW, barH, Color{ 25, 35, 52, 255 });
            float progress = (totalSteps > 1) ? (static_cast<float>(stepIdx) / (totalSteps - 1)) : 1.0f;
            DrawRectangle(barX, barY, static_cast<int>(barW * progress), barH, Color{ 46, 204, 113, 255 });
            DrawCircle(barX + static_cast<int>(barW * progress), barY + 3, 5, WHITE);
            DrawText(TextFormat("Step %zu of %zu", stepIdx + 1, totalSteps), 
                hudX + 16, hudY + 290, 10, Color{ 140, 155, 175, 255 });

            // ----------------------------------------------------
            // 2. TOP-RIGHT: NavGraph & Dijkstra Legend Card
            // ----------------------------------------------------
            int legW = 265;
            int legH = 185;
            int legX = screenW - legW - 16;
            int legY = 16;
            Rectangle legRect = { (float)legX, (float)legY, (float)legW, (float)legH };
            DrawRectangleRounded(legRect, 0.05f, 4, ColorAlpha(Color{ 10, 14, 24, 255 }, 0.92f));
            DrawRectangleRoundedLines(legRect, 0.05f, 4, Color{ 48, 68, 98, 255 });
            DrawRectangle(legX + 1, legY + 1, legW - 2, 3, Color{ 46, 204, 113, 255 }); // Emerald Green accent

            DrawText("NAVGRAPH & DIJKSTRA LEGEND", legX + 16, legY + 14, 12, RAYWHITE);
            DrawLine(legX + 16, legY + 32, legX + legW - 16, legY + 32, Color{ 35, 48, 70, 255 });

            // Legend Items
            DrawRectangle(legX + 16, legY + 44, 16, 5, Color{ 46, 204, 113, 255 });
            DrawText("Optimal Shortest Path Conduit", legX + 40, legY + 40, 11, Color{ 180, 255, 200, 255 });

            DrawCircle(legX + 24, legY + 68, 5, GOLD);
            DrawText("Active Node u (Settling)", legX + 40, legY + 63, 11, GOLD);

            DrawRectangle(legX + 16, legY + 88, 16, 4, Color{ 0, 240, 255, 255 });
            DrawText("Examining Edge (u -> v)", legX + 40, legY + 83, 11, Color{ 0, 240, 255, 255 });

            DrawCircle(legX + 24, legY + 110, 5, Color{ 52, 152, 219, 255 });
            DrawText("Visited / Settled Node", legX + 40, legY + 105, 11, RAYWHITE);

            DrawRectangle(legX + 16, legY + 130, 16, 4, Color{ 240, 45, 45, 255 });
            DrawText("Blocked by Boulder Cut", legX + 40, legY + 125, 11, Color{ 255, 120, 120, 255 });

            DrawCircle(legX + 24, legY + 152, 5, GraphRenderer3D::getNodeColor(NodeState::START));
            DrawText("Start Origin / End Target", legX + 40, legY + 147, 11, RAYWHITE);

            // ----------------------------------------------------
            // 3. BOTTOM COMMAND DECK & KEYBINDINGS BAR
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
            DrawText("CAMERA NAVIGATION", c1X, deckY + 10, 10, Color{ 100, 185, 255, 255 });
            DrawKeyBind(c1X, deckY + 28, "RMB Drag", "Orbit View");
            DrawKeyBind(c1X + 130, deckY + 28, "Wheel", "Zoom");
            DrawKeyBind(c1X, deckY + 54, "MMB / Shift+RMB", "Pan");
            DrawKeyBind(c1X + 175, deckY + 54, "R", "Reset");

            // --- Column 2: Dijkstra Playback & Scrubbing ---
            int c2X = deckX + 16 + static_cast<int>(colW);
            DrawLine(c2X - 12, deckY + 10, c2X - 12, deckY + deckH - 10, Color{ 35, 48, 70, 255 });
            DrawText("DIJKSTRA REPLAY & SCRUB", c2X, deckY + 10, 10, Color{ 46, 204, 113, 255 });
            DrawKeyBind(c2X, deckY + 28, "Space / P", dijkstra.isPlaying() ? "Pause" : "Play Replay");
            DrawKeyBind(c2X + 130, deckY + 28, "< / >", "Step Back / Fwd");
            DrawKeyBind(c2X, deckY + 54, "Enter", "Finish Path");
            DrawKeyBind(c2X + 130, deckY + 54, "Bksp", "Rewind Start");

            // --- Column 3: Cost Presets & Physics ---
            int c3X = deckX + 16 + static_cast<int>(colW * 2);
            DrawLine(c3X - 12, deckY + 10, c3X - 12, deckY + deckH - 10, Color{ 35, 48, 70, 255 });
            DrawText("COST PRESETS & PHYSICS", c3X, deckY + 10, 10, Color{ 241, 196, 15, 255 });
            DrawKeyBind(c3X, deckY + 28, "1-4", "Presets: Std/Direct/Energy/Tract");
            DrawKeyBind(c3X, deckY + 54, "B", "Drop Sphere");
            DrawKeyBind(c3X + 115, deckY + 54, "X", "Cascade");
            DrawKeyBind(c3X + 205, deckY + 54, "C", "Clear");

            // --- Column 4: Display & Visual Toggles ---
            int c4X = deckX + 16 + static_cast<int>(colW * 3);
            DrawLine(c4X - 12, deckY + 10, c4X - 12, deckY + deckH - 10, Color{ 35, 48, 70, 255 });
            DrawText("VIEW & HUD TOGGLES", c4X, deckY + 10, 10, Color{ 190, 160, 240, 255 });
            DrawKeyBind(c4X, deckY + 28, "T", "Terrain", showTerrain);
            DrawKeyBind(c4X + 85, deckY + 28, "E", "Edges", showEdges);
            DrawKeyBind(c4X + 165, deckY + 28, "N", "Nodes", showNodes);
            DrawKeyBind(c4X, deckY + 54, "O", "Boulders", showBoulders);
            DrawKeyBind(c4X + 85, deckY + 54, "W", "Wire", showWireframe);
            DrawKeyBind(c4X + 165, deckY + 54, "H", "HUD", showHUD);
        } else {
            // Minimalist badge when HUD is hidden
            DrawKeyBind(GetScreenWidth() - 140, GetScreenHeight() - 32, "H", "Show HUD", true);
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
