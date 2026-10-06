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
#include "ChunkManager.h"
#include "RoverNavGraph.h"
#include "PhysicsWorld.h"
#include "DijkstraSolver3D.h"
#include "PlanetaryRover.h"
#include "RoverTelemetryHUD.h"

// Helper: Render a modern, high-contrast keybinding badge pill with text, returning total width
static int DrawKeyBind(int x, int y, const char* key, const char* label, bool active = true) {
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

    int labelX = x + keyW + padX * 2 + 5;
    DrawText(label, labelX, y + 4, 11, labelColor);

    return (keyW + padX * 2 + 5 + MeasureText(label, 11));
}

static const TerrainHeightfield* s_activeTerrain = nullptr;
static float SampleActiveTerrainHeight(float x, float z) {
    return s_activeTerrain ? s_activeTerrain->getHeight(x, z) : 0.0f;
}

// Helper: Load/Switch Terrain Mode Presets with customized physical landscapes & obstacles
static void ApplyTerrainPreset(
    TerrainPreset preset,
    TerrainHeightfield& terrain,
    ChunkManager& chunkMgr,
    bool infiniteWorldMode,
    PhysicsWorld& physics,
    RoverNavGraph& navGraph,
    DijkstraSolver3D& dijkstra,
    PlanetaryRover& rover,
    int& blockedEdgeCount
) {
    s_activeTerrain = &terrain;
    terrain.setPreset(preset);
    terrain.generate();

    // Set realistic gravity for this planetary environment (Mars: 3.71, Moon: 1.62, Earth: 9.81)
    physics.setGravity(terrain.getPresetGravity());

    physics.clearBoulders();
    physics.clearDynamicSpheres();

    if (infiniteWorldMode) {
        chunkMgr.clear(physics);
        chunkMgr.init(terrain, physics);
        chunkMgr.update(Vector3{ 0.0f, 0.0f, 0.0f }, terrain, physics);
    } else {
        float terrainSpacing = terrain.getSize() / (terrain.getResolution() - 1);
        physics.createTerrainHeightfield(
            terrain.getHeightData().data(),
            terrain.getResolution(),
            terrain.getResolution(),
            terrainSpacing
        );
    }

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

    // Spawn preset landmark boulders
    for (const auto& bp : boulders) {
        float h = terrain.getHeight(bp.pos.x, bp.pos.y);
        Vector3 boulderPos = { bp.pos.x, h + bp.radius * 0.70f, bp.pos.y };
        physics.spawnBoulder(boulderPos, bp.radius);
    }

    // Procedural Planetary Rock Fields (Spanning radius up to 620m across the entire planetary surface)
    uint32_t seed = 42 + static_cast<uint32_t>(preset) * 1337;
    auto pseudoRand = [&seed]() {
        seed = seed * 1664525u + 1013904223u;
        return static_cast<float>(seed & 0xFFFF) / 65535.0f;
    };

    const int numProceduralBoulders = 140;
    for (int i = 0; i < numProceduralBoulders; ++i) {
        float angle = pseudoRand() * 2.0f * PI;
        float dist = 20.0f + sqrtf(pseudoRand()) * 600.0f;
        float bx = dist * cosf(angle);
        float bz = dist * sinf(angle);

        // Don't spawn rocks directly on top of the rover start point
        if (sqrtf(bx * bx + bz * bz) < 14.0f) continue;

        float slope = terrain.getSlopeAngleRad(bx, bz);
        if (slope < 22.0f * DEG2RAD) {
            float radius = 1.6f + pseudoRand() * 2.6f; // Radii from 1.6m to 4.2m
            float by = terrain.getHeight(bx, bz);
            Vector3 bPos = { bx, by + radius * 0.70f, bz };
            physics.spawnBoulder(bPos, radius);
        }
    }

    // Lazy-Loaded Discovery: Unveil local exploration network around landing site (radius 110m, spacing 12m)
    navGraph.generatePersistentPlanetaryGrid(terrain, Vector3{ 0.0f, 0.0f, 0.0f }, 110.0f, 12.0f);
    navGraph.validateEdgesWithPhysics(physics, 0.6f);

    blockedEdgeCount = 0;
    for (const auto& e : navGraph.getEdges()) {
        if (e.isBlocked) blockedEdgeCount++;
    }

    // Solve Dijkstra route with history
    dijkstra.solveWithHistory(navGraph.getStartNode(), navGraph.getEndNode(),
                              navGraph.getVertices(), navGraph.getBlockedEdgesMap());

    // Reset Rover at valid start node facing first path segment
    if (navGraph.getStartNode()) {
        const auto& path = dijkstra.getShortestPathNodes();
        float startYaw = 0.0f;
        if (path.size() >= 2 && path[0] && path[1]) {
            startYaw = atan2f(path[1]->position.x - path[0]->position.x, path[1]->position.z - path[0]->position.z);
        }
        rover.reset(physics, navGraph.getStartNode()->position, startYaw);
        rover.setPath(path);
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

    // 4. Procedural Martian Terrain Heightfield (Full 1,440m planetary globe physics coverage)
    TerrainHeightfield terrain(256, 1440.0f);

    // 5. Infinite Procedural Chunk Manager
    ChunkManager chunkMgr;
    bool infiniteWorldMode = false;

    // 6. 3D NavGraph
    RoverNavGraph navGraph;

    // 7. Physics-Weighted 3D Dijkstra Solver with Snapshot History
    DijkstraSolver3D dijkstra;
    int currentPresetIndex = 0;
    dijkstra.applyPreset(currentPresetIndex);

    // 8. Planetary Rover Rig
    PlanetaryRover rover;

    // 9. Initialize Default Scenario (The Olympus Crater)
    int blockedEdgeCount = 0;
    ApplyTerrainPreset(TerrainPreset::OLYMPUS_CRATER, terrain, chunkMgr, infiniteWorldMode, physics, navGraph, dijkstra, rover, blockedEdgeCount);
    Vector3 lastNavGraphCenter = rover.getPosition();

    // Initial rolling test sphere on slope
    physics.spawnDynamicSphere(Vector3{ 20.0f, terrain.getHeight(20.0f, -10.0f) + 6.0f, -10.0f }, 1.2f, 50.0f);

    // Display options
    bool showTerrain = true;
    bool showEdges = true;
    bool showNodes = true;
    bool showBoulders = true;
    bool showSpheres = true;
    bool showWireframe = false;
    bool showHUD = true;

    // Distant Martian pale blue sun position
    Vector3 sunPosition = { 160.0f, 110.0f, -130.0f };

    // 10. Main Simulation Loop
    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt > 0.05f) dt = 0.05f;

        // Step Jolt Physics (60 Hz multi-threaded)
        physics.step(dt);

        // Update Autonomous Planetary Rover & Pure Pursuit Navigation
        rover.update(physics, dt);

        // Lazy-Loaded Dynamic Exploration Discovery:
        // As rover or camera moves into undiscovered areas, lazily unveil new nodes (radius 96m)
        // while preserving all previously discovered nodes and areas with 60 FPS performance!
        Vector3 explorationCenter = (rover.getCameraMode() == RoverCameraMode::ORBIT)
                                    ? cameraController.getCamera().target
                                    : rover.getPosition();

        static Vector3 lastLazyDiscoveryPos = Vector3{ 0.0f, 0.0f, 0.0f };
        static float lastDiscoveryTime = 0.0f;
        float now = static_cast<float>(GetTime());

        float distFromLastLazy = Vector3Distance(explorationCenter, lastLazyDiscoveryPos);
        if (distFromLastLazy > 18.0f && (now - lastDiscoveryTime > 0.06f)) {
            lastLazyDiscoveryPos = explorationCenter;
            lastDiscoveryTime = now;
            navGraph.generatePersistentPlanetaryGrid(terrain, explorationCenter, 96.0f, 12.0f);
            if (infiniteWorldMode) {
                chunkMgr.update(explorationCenter, terrain, physics);
            }
        }

        // Handle Autonomous Dynamic Route Recalculation (Hazard / Boulder Avoidance)
        if (rover.isReplanRequested()) {
            Vector3 roverPos = rover.getPosition();
            Vertex3D* currentNearest = navGraph.getClosestWalkableNode(roverPos);

            // Block the impassable edge if a specific hazard position was probed
            Vector3 hazardPos = rover.getHazardPos();
            if (Vector3LengthSqr(hazardPos) > 0.1f) {
                if (navGraph.blockEdgeBetweenPositions(roverPos, hazardPos)) {
                    // Update blocked edge count
                    blockedEdgeCount = 0;
                    for (const auto& e : navGraph.getEdges()) {
                        if (e.isBlocked) blockedEdgeCount++;
                    }
                }
            }

            if (currentNearest && navGraph.getEndNode() && currentNearest != navGraph.getEndNode()) {
                dijkstra.solveWithHistory(currentNearest, navGraph.getEndNode(),
                                          navGraph.getVertices(), navGraph.getBlockedEdgesMap());
                const auto& newPath = dijkstra.getShortestPathNodes();
                if (!newPath.empty()) {
                    Vector3 goalPos = navGraph.getEndNode() ? navGraph.getEndNode()->position : Vector3{ 0, 0, 0 };
                    rover.setPath(newPath, dijkstra.isPartialPath(), dijkstra.getDistanceToGoal(), goalPos);
                }
            }
            rover.clearReplanRequest();
        }

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
                    if (navGraph.getStartNode() && navGraph.getEndNode()) {
                        navGraph.ensureCorridor(terrain, navGraph.getStartNode()->position, navGraph.getEndNode()->position, navGraph.getSpacing());
                    }
                    dijkstra.solveWithHistory(navGraph.getStartNode(), navGraph.getEndNode(), 
                                              navGraph.getVertices(), navGraph.getBlockedEdgesMap());
                    Vector3 goalPos = navGraph.getEndNode() ? navGraph.getEndNode()->position : Vector3{ 0, 0, 0 };
                    rover.setPath(dijkstra.getShortestPathNodes(), dijkstra.isPartialPath(), dijkstra.getDistanceToGoal(), goalPos);
                }
            }
        }

        // Track Left-drag for globe surface rotation (when dragging across terrain, not clicking a node)
        static Vector2 leftClickStart = { 0.0f, 0.0f };
        static bool isLeftDragging = false;

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            leftClickStart = mousePos;
            isLeftDragging = false;
        } else if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            float dragDist = Vector2Distance(mousePos, leftClickStart);
            if (dragDist > 4.0f && !hoveredNode && !isOverUI) {
                isLeftDragging = true;
            }
        } else if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            isLeftDragging = false;
        }

        // Update Orbital Camera with Globe Surface Rotation (anchored to physical planetary sphere)
        if (rover.getCameraMode() == RoverCameraMode::ORBIT) {
            cameraController.update(isOverUI, SampleActiveTerrainHeight, isLeftDragging);
        }

        // Update Dijkstra Step-by-Step Playback
        dijkstra.update(dt);

        // Cost Preset Selection Keys: 1, 2, 3, 4, 5
        auto applyPresetAndRoute = [&](int presetIdx) {
            currentPresetIndex = presetIdx;
            dijkstra.applyPreset(presetIdx);
            dijkstra.solveWithHistory(navGraph.getStartNode(), navGraph.getEndNode(), 
                                      navGraph.getVertices(), navGraph.getBlockedEdgesMap());
            Vector3 goalPos = navGraph.getEndNode() ? navGraph.getEndNode()->position : Vector3{ 0, 0, 0 };
            rover.setPath(dijkstra.getShortestPathNodes(), dijkstra.isPartialPath(), dijkstra.getDistanceToGoal(), goalPos);
        };

        if (IsKeyPressed(KEY_ONE))   applyPresetAndRoute(0);
        if (IsKeyPressed(KEY_TWO))   applyPresetAndRoute(1);
        if (IsKeyPressed(KEY_THREE)) applyPresetAndRoute(2);
        if (IsKeyPressed(KEY_FOUR))  applyPresetAndRoute(3);
        if (IsKeyPressed(KEY_FIVE))  applyPresetAndRoute(4);

        // Phase 5: Planetary Rover Driving Controls & Camera Focus
        if (IsKeyPressed(KEY_TAB)) {
            rover.toggleAutonomous();
        }
        if (IsKeyPressed(KEY_T)) {
            // Direct Off-Road Infiltration / Traverse to Goal Beacon
            if (navGraph.getEndNode()) {
                rover.engageDirectHoming(navGraph.getEndNode()->position);
            }
        }
        if (IsKeyPressed(KEY_F)) {
            cameraController.focusOn(rover.getPosition(), 22.0f);
        }
        if (IsKeyPressed(KEY_M)) {
            terrain.cyclePreset();
            ApplyTerrainPreset(terrain.getPreset(), terrain, chunkMgr, infiniteWorldMode, physics, navGraph, dijkstra, rover, blockedEdgeCount);
            lastNavGraphCenter = rover.getPosition();
        }
        if (IsKeyPressed(KEY_I)) {
            infiniteWorldMode = !infiniteWorldMode;
            ApplyTerrainPreset(terrain.getPreset(), terrain, chunkMgr, infiniteWorldMode, physics, navGraph, dijkstra, rover, blockedEdgeCount);
            lastNavGraphCenter = rover.getPosition();
        }
        if (IsKeyPressed(KEY_V)) {
            rover.cycleCameraMode();
        }
        if (IsKeyPressed(KEY_R)) {
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                cameraController.reset(Vector3{ 0.0f, 75.0f, 115.0f }, Vector3{ 0.0f, 0.0f, 0.0f });
            } else if (navGraph.getStartNode()) {
                const auto& path = dijkstra.getShortestPathNodes();
                float startYaw = 0.0f;
                if (path.size() >= 2 && path[0] && path[1]) {
                    startYaw = atan2f(path[1]->position.x - path[0]->position.x, path[1]->position.z - path[0]->position.z);
                }
                rover.reset(physics, navGraph.getStartNode()->position, startYaw);
                rover.setPath(path);
            }
        }
        if (IsKeyPressed(KEY_U)) {
            rover.selfRight(physics);
        }
        if (IsKeyPressed(KEY_G)) {
            // Cycle between realistic planetary gravities: Mars (-3.71) -> Moon (-1.62) -> Earth (-9.81)
            float g = physics.getGravity();
            if (fabsf(g + 3.71f) < 0.2f) {
                physics.setGravity(-1.62f); // Switch to Moon Gravity
            } else if (fabsf(g + 1.62f) < 0.2f) {
                physics.setGravity(-9.81f); // Switch to Earth Gravity
            } else {
                physics.setGravity(-3.71f); // Switch to Mars Gravity
            }
        }

        // Manual Driving Overrides (when Autonomous Pure Pursuit is paused)
        if (!rover.isAutonomous()) {
            float manualThrottle = 0.0f;
            float manualSteer = 0.0f;
            float manualBrake = 0.0f;
            if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) manualThrottle += 1.0f;
            if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) manualThrottle -= 0.6f;
            if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) manualSteer += 0.60f;
            if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) manualSteer -= 0.60f;
            if (IsKeyDown(KEY_SPACE)) manualBrake = 1.0f;

            rover.setThrottleInput(manualThrottle);
            rover.setSteeringInput(manualSteer);
            rover.setBrakeInput(manualBrake);
        }

        // Dijkstra Algorithm Playback & Scrubbing Keys (scrub with Left/Right when autonomous, or [ / ] anytime)
        if (IsKeyPressed(KEY_P)) {
            dijkstra.togglePlay();
        }
        if (IsKeyPressed(KEY_RIGHT) || IsKeyPressed(KEY_RIGHT_BRACKET)) {
            if (rover.isAutonomous() || IsKeyPressed(KEY_RIGHT_BRACKET)) {
                dijkstra.stepForward();
            }
        }
        if (IsKeyPressed(KEY_LEFT) || IsKeyPressed(KEY_LEFT_BRACKET)) {
            if (rover.isAutonomous() || IsKeyPressed(KEY_LEFT_BRACKET)) {
                dijkstra.stepBackward();
            }
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

        // Keyboard Display Controls (All conflict-free hotkeys)
        if (IsKeyPressed(KEY_T)) showTerrain = !showTerrain;
        if (IsKeyPressed(KEY_E)) showEdges = !showEdges;
        if (IsKeyPressed(KEY_N)) showNodes = !showNodes;
        if (IsKeyPressed(KEY_O)) showBoulders = !showBoulders;
        if (IsKeyPressed(KEY_Z)) showSpheres = !showSpheres;
        if (IsKeyPressed(KEY_K)) showWireframe = !showWireframe; // K toggle (avoids conflict with W gas pedal)
        if (IsKeyPressed(KEY_H)) showHUD = !showHUD;

        // Render Frame
        BeginDrawing();

        // 1. Explicitly clear Color and Depth buffers (essential for 3D camera rotation)
        ClearBackground(Color{ 12, 14, 24, 255 });

        // 2. Atmospheric Sky Gradient (2D background with depth testing disabled)
        rlDisableDepthMask();
        rlDisableDepthTest();
        DrawRectangleGradientV(0, 0, GetRenderWidth(), GetRenderHeight(), 
            Color{ 10, 12, 22, 255 }, 
            Color{ 118, 62, 45, 255 }
        );
        rlEnableDepthTest();
        rlEnableDepthMask();

        float sceneTime = static_cast<float>(GetTime());

        // 3D Scene Rendering
        BeginMode3D(activeCamera);
        {
            // A. Distant Martian Sun (pale blue disk with soft atmospheric halo)
            DrawSphere(sunPosition, 6.5f, Color{ 195, 230, 255, 255 });
            DrawSphereWires(sunPosition, 10.0f, 8, 8, ColorAlpha(Color{ 150, 205, 255, 255 }, 0.45f));

            // B. Draw Procedural Martian Planetary Globe Mesh (1,440m circular sphere)
            if (showTerrain && terrain.isLoaded()) {
                rlDisableBackfaceCulling();
                DrawModel(terrain.getModel(), Vector3{ 0.0f, 0.0f, 0.0f }, 1.0f, WHITE);
                rlEnableBackfaceCulling();

                if (showWireframe) {
                    DrawModelWires(terrain.getModel(), Vector3{ 0.0f, 0.0f, 0.0f }, 1.0f, ColorAlpha(BLACK, 0.2f));
                }

                if (infiniteWorldMode) {
                    chunkMgr.draw(showWireframe);
                }

                // Curved Atmospheric Horizon Glow Ring (Spherical Horizon Silhouette)
                DrawCircle3D(Vector3{ 0.0f, -220.0f, 0.0f }, 725.0f, Vector3{ 0, 1, 0 }, 90.0f, ColorAlpha(Color{ 210, 115, 75, 255 }, 0.40f));
                DrawCircle3D(Vector3{ 0.0f, -224.0f, 0.0f }, 738.0f, Vector3{ 0, 1, 0 }, 90.0f, ColorAlpha(Color{ 180, 85, 55, 255 }, 0.20f));
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

            // Standoff Vantage Conduit: If goal is physically blocked, draw laser line-of-sight from standoff node to goal
            if (dijkstra.isPartialPath() && navGraph.getEndNode()) {
                const auto& pathNodes = dijkstra.getShortestPathNodes();
                if (!pathNodes.empty()) {
                    Vector3 standoffPos = pathNodes.back()->position;
                    Vector3 goalPos = navGraph.getEndNode()->position;
                    float pulse = 0.5f + 0.5f * sinf(sceneTime * 6.0f);
                    Color standoffCol = ColorAlpha(Color{ 255, 175, 45, 255 }, 0.55f + 0.45f * pulse);
                    
                    // Standoff targeting sightline beam and rings (elevated above ground)
                    DrawLine3D(Vector3Add(standoffPos, Vector3{ 0, 1.2f, 0 }), Vector3Add(goalPos, Vector3{ 0, 1.5f, 0 }), standoffCol);
                    DrawSphereWires(standoffPos, 1.6f + 0.4f * pulse, 6, 6, standoffCol);
                    DrawCircle3D(standoffPos, 3.0f, Vector3{ 0, 1, 0 }, 90.0f, standoffCol);

                    // When rover arrives at standoff vantage point: prominent arrival beacon indicator
                    if (rover.isAtStandoffVantage()) {
                        Vector3 poleTop = Vector3Add(standoffPos, Vector3{ 0.0f, 6.0f, 0.0f });
                        DrawLine3D(standoffPos, poleTop, Color{ 255, 195, 50, 255 });
                        DrawSphereWires(poleTop, 0.7f + 0.2f * pulse, 6, 6, Color{ 255, 215, 60, 255 });
                        DrawCircle3D(standoffPos, 4.5f + sinf(sceneTime * 3.0f) * 0.8f, Vector3{ 0, 1, 0 }, 90.0f, ColorAlpha(GOLD, 0.5f));
                    }
                }
            }

            // Direct Off-Road Homing Line (when user forces direct traverse toward blocked goal)
            if (rover.isDirectHoming() && navGraph.getEndNode()) {
                Vector3 roverPos = rover.getPosition();
                Vector3 goalPos = navGraph.getEndNode()->position;
                float pulse = 0.5f + 0.5f * sinf(sceneTime * 10.0f);
                Color homingCol = ColorAlpha(Color{ 255, 90, 30, 255 }, 0.7f + 0.3f * pulse);
                DrawLine3D(Vector3Add(roverPos, Vector3{ 0, 0.8f, 0 }), Vector3Add(goalPos, Vector3{ 0, 1.2f, 0 }), homingCol);
            }

            // G. Draw Draped NavGraph Nodes (Prominent Glowing 3D Spheres with Distance LOD)
            if (showNodes) {
                Vector3 camPos = activeCamera.position;
                for (const Vertex3D* v : navGraph.getVertices()) {
                    if (v == navGraph.getStartNode() || v == navGraph.getEndNode()) {
                        continue; // Drawn prominently below
                    }

                    float distToCam = Vector3Distance(camPos, v->position);

                    if (v->state == NodeState::IMPASSABLE) {
                        // Prominent Hazard Node on steep slopes / cliffs / boulder hazards
                        DrawSphere(v->position, 0.70f, Color{ 235, 65, 50, 220 });
                        if (distToCam < 260.0f) {
                            DrawSphereWires(v->position, 0.90f, 4, 4, ColorAlpha(RED, 0.50f));
                        }
                        continue;
                    }

                    Color nodeCol = GraphRenderer3D::getNodeColor(v->state);
                    float r = 0.90f; // Prominently visible from panoramic orbit camera!
                    DrawSphere(v->position, r, nodeCol);
                    if (distToCam < 260.0f) {
                        DrawSphereWires(v->position, r * 1.25f, 6, 6, ColorAlpha(nodeCol, 0.60f));
                        DrawLine3D(v->position, Vector3{ v->position.x, v->position.y - 0.5f, v->position.z }, ColorAlpha(nodeCol, 0.8f));
                    }
                }

                // Prominent START Beacon with 26m vertical laser beam and pulsating radar ground rings
                if (const Vertex3D* s = navGraph.getStartNode()) {
                    Vector3 pillarTop = Vector3Add(s->position, Vector3{ 0.0f, 26.0f, 0.0f });
                    DrawCylinderEx(s->position, pillarTop, 0.40f, 0.05f, 10, ColorAlpha(Color{ 46, 230, 113, 255 }, 0.85f));
                    DrawSphere(s->position, 1.8f, Color{ 46, 230, 113, 255 });
                    DrawSphereWires(s->position, 2.3f, 8, 8, WHITE);
                    float pulseR = 3.5f + sinf(sceneTime * 4.0f) * 0.8f;
                    DrawCircle3D(s->position, pulseR, Vector3{ 0, 1, 0 }, 90.0f, ColorAlpha(Color{ 46, 230, 113, 255 }, 0.75f));
                    DrawCircle3D(s->position, pulseR * 1.5f, Vector3{ 0, 1, 0 }, 90.0f, ColorAlpha(Color{ 46, 230, 113, 255 }, 0.35f));
                }

                // Prominent END Beacon with 26m vertical laser beam and pulsating radar ground rings
                if (const Vertex3D* e = navGraph.getEndNode()) {
                    Vector3 pillarTop = Vector3Add(e->position, Vector3{ 0.0f, 26.0f, 0.0f });
                    DrawCylinderEx(e->position, pillarTop, 0.40f, 0.05f, 10, ColorAlpha(Color{ 235, 60, 60, 255 }, 0.85f));
                    DrawSphere(e->position, 1.8f, Color{ 235, 60, 60, 255 });
                    DrawSphereWires(e->position, 2.3f, 8, 8, WHITE);
                    float pulseR = 3.5f + sinf(sceneTime * 4.0f + 1.5f) * 0.8f;
                    DrawCircle3D(e->position, pulseR, Vector3{ 0, 1, 0 }, 90.0f, ColorAlpha(Color{ 235, 60, 60, 255 }, 0.75f));
                    DrawCircle3D(e->position, pulseR * 1.5f, Vector3{ 0, 1, 0 }, 90.0f, ColorAlpha(Color{ 235, 60, 60, 255 }, 0.35f));
                }
            }

            // H. Phase 5: Autonomous Planetary Rover Physical Rig & 3D Model
            rover.render(sceneTime);

            // I. Draw Dynamic Rolling Test Spheres (Synchronized with Jolt Physics rigid bodies)
            if (showSpheres) {
                for (auto sphereId : physics.getDynamicSpheres()) {
                    Vector3 pos = physics.getBodyPosition(sphereId);
                    DrawSphere(pos, 1.2f, Color{ 0, 185, 255, 255 });
                    DrawSphereWires(pos, 1.2f, 8, 8, ColorAlpha(WHITE, 0.75f));
                }
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

            // Row 1: Interactive Terrain Mode Switcher [M] & Infinite World Toggle [I]
            Rectangle tmRect = { (float)(hudX + 16), (float)(hudY + 52), (float)(hudW - 160), 22.0f };
            bool tmHovered = CheckCollisionPointRec(mousePos, tmRect);
            DrawRectangleRounded(tmRect, 0.25f, 4, tmHovered ? Color{ 36, 52, 78, 255 } : Color{ 20, 28, 44, 255 });
            DrawRectangleRoundedLines(tmRect, 0.25f, 4, tmHovered ? Color{ 90, 170, 255, 255 } : Color{ 50, 75, 110, 255 });
            DrawText(TextFormat("TERRAIN [M]: %s", terrain.getPresetName()), hudX + 24, hudY + 57, 11, Color{ 100, 215, 255, 255 });

            Rectangle infRect = { (float)(hudX + hudW - 136), (float)(hudY + 52), 120.0f, 22.0f };
            bool infHovered = CheckCollisionPointRec(mousePos, infRect);
            Color infBg = infiniteWorldMode ? Color{ 24, 60, 48, 255 } : Color{ 36, 36, 44, 255 };
            if (infHovered) infBg = infiniteWorldMode ? Color{ 32, 80, 64, 255 } : Color{ 50, 50, 60, 255 };
            Color infLine = infiniteWorldMode ? Color{ 46, 204, 113, 255 } : Color{ 120, 130, 150, 255 };
            DrawRectangleRounded(infRect, 0.25f, 4, infBg);
            DrawRectangleRoundedLines(infRect, 0.25f, 4, infLine);
            DrawText(infiniteWorldMode ? TextFormat("[I] INF (%d Chk)", chunkMgr.getActiveChunkCount()) : "[I] BOUNDED", 
                hudX + hudW - 128, hudY + 57, 10, infiniteWorldMode ? Color{ 60, 230, 175, 255 } : Color{ 170, 180, 195, 255 });

            if (tmHovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                terrain.cyclePreset();
                ApplyTerrainPreset(terrain.getPreset(), terrain, chunkMgr, infiniteWorldMode, physics, navGraph, dijkstra, rover, blockedEdgeCount);
                lastNavGraphCenter = rover.getPosition();
            }
            if (infHovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                infiniteWorldMode = !infiniteWorldMode;
                ApplyTerrainPreset(terrain.getPreset(), terrain, chunkMgr, infiniteWorldMode, physics, navGraph, dijkstra, rover, blockedEdgeCount);
                lastNavGraphCenter = rover.getPosition();
            }

            DrawLine(hudX + 16, hudY + 82, hudX + hudW - 16, hudY + 82, Color{ 35, 48, 70, 255 });

            // Row 2: Active Cost Preset & Parameters
            const auto& w = dijkstra.getWeights();
            if (dijkstra.isNeuralMode()) {
                DrawText(TextFormat("Cost Preset [5]: %s", w.name.c_str()), 
                    hudX + 16, hudY + 90, 11, Color{ 60, 230, 175, 255 });
                DrawText("Engine: 3-Layer PINN MLP (8 -> 32 -> 16 -> 1) | <0.05 us/edge | Zero-Alloc", 
                    hudX + 16, hudY + 106, 10, Color{ 140, 235, 205, 255 });
            } else {
                DrawText(TextFormat("Cost Preset [%d]: %s", currentPresetIndex + 1, w.name.c_str()), 
                    hudX + 16, hudY + 90, 11, Color{ 240, 200, 80, 255 });
                DrawText(TextFormat("Weights: alpha=%.1f (Work) | beta=%.1f (Slip) | gamma=%.1f | delta=%.1f", 
                    w.alpha, w.beta, w.gamma, w.delta), hudX + 16, hudY + 106, 10, Color{ 160, 175, 195, 255 });
            }

            DrawLine(hudX + 16, hudY + 122, hudX + hudW - 16, hudY + 122, Color{ 35, 48, 70, 255 });

            // Row 3: Physics & NavGraph Infrastructure Telemetry
            const char* gravEnv = "Mars";
            float gVal = fabsf(physics.getGravity());
            if (fabsf(gVal - 1.62f) < 0.2f) gravEnv = "Moon";
            else if (fabsf(gVal - 9.81f) < 0.5f) gravEnv = "Earth";

            DrawText(TextFormat("Jolt Engine: %d Boulders | %d Cut Edges | Gravity: %.2f m/s² (%s) [G]",
                (int)physics.getBoulders().size(), blockedEdgeCount, physics.getGravity(), gravEnv),
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
                if (dijkstra.isPartialPath()) {
                    DrawText(TextFormat("STATUS: [STANDOFF VANTAGE - %.1fm FROM BLOCKED GOAL]", stats.distanceToGoal), 
                        hudX + 16, hudY + 150, 12, Color{ 255, 175, 45, 255 });
                } else {
                    DrawText(TextFormat("STATUS: [OPTIMAL ROUTE SOLVED in %.2f ms]", stats.computeTimeMs), 
                        hudX + 16, hudY + 150, 12, Color{ 46, 204, 113, 255 });
                }
            } else {
                DrawText("STATUS: [NO TRAVERSABLE PATH - SLIP / BOULDER BLOCKED]", 
                    hudX + 16, hudY + 150, 12, Color{ 255, 95, 95, 255 });
            }

            // Row 5: Route Metrics
            if (stats.isValid) {
                if (stats.isPartial) {
                    DrawText(TextFormat("Safe Standoff: %.1fm  |  Standoff Gap: %.1fm to target", 
                        stats.totalDistance, stats.distanceToGoal), hudX + 16, hudY + 172, 11, Color{ 255, 215, 100, 255 });
                } else {
                    DrawText(TextFormat("3D Distance: %.1f m   |   Physical Energy: %.1f J-equiv", 
                        stats.totalDistance, stats.totalEnergyCost), hudX + 16, hudY + 172, 11, RAYWHITE);
                }
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
            int x1 = c1X;
            x1 += DrawKeyBind(x1, deckY + 28, "RMB Drag", "Orbit") + 8;
            DrawKeyBind(x1, deckY + 28, "Wheel", "Zoom");
            int x2 = c1X;
            x2 += DrawKeyBind(x2, deckY + 54, "MMB", "Pan") + 8;
            x2 += DrawKeyBind(x2, deckY + 54, "F", "Focus") + 8;
            DrawKeyBind(x2, deckY + 54, "Sh+R", "Reset");

            // --- Column 2: Rover Navigation ---
            int c2X = deckX + 16 + static_cast<int>(colW);
            DrawLine(c2X - 12, deckY + 10, c2X - 12, deckY + deckH - 10, Color{ 35, 48, 70, 255 });
            DrawText("ROVER NAVIGATION (WASD)", c2X, deckY + 10, 10, Color{ 46, 204, 113, 255 });
            int x3 = c2X;
            x3 += DrawKeyBind(x3, deckY + 28, "Tab", rover.isAutonomous() ? "Manual" : "Auto Drive") + 8;
            x3 += DrawKeyBind(x3, deckY + 28, "T", "Traverse") + 8;
            x3 += DrawKeyBind(x3, deckY + 28, "WASD", "Drive") + 8;
            DrawKeyBind(x3, deckY + 28, "Space", "Brake");
            int x4 = c2X;
            x4 += DrawKeyBind(x4, deckY + 54, "R", "Reset") + 8;
            x4 += DrawKeyBind(x4, deckY + 54, "U", "Self-Right") + 8;
            const char* gKeyName = "Mars G";
            if (fabsf(physics.getGravity() + 1.62f) < 0.2f) gKeyName = "Moon G";
            else if (fabsf(physics.getGravity() + 9.81f) < 0.5f) gKeyName = "Earth G";
            DrawKeyBind(x4, deckY + 54, "G", gKeyName);

            // --- Column 3: Terrain & Dijkstra ---
            int c3X = deckX + 16 + static_cast<int>(colW * 2);
            DrawLine(c3X - 12, deckY + 10, c3X - 12, deckY + deckH - 10, Color{ 35, 48, 70, 255 });
            DrawText("TERRAIN & DIJKSTRA", c3X, deckY + 10, 10, Color{ 241, 196, 15, 255 });
            int x5 = c3X;
            x5 += DrawKeyBind(x5, deckY + 28, "M", "Preset") + 6;
            x5 += DrawKeyBind(x5, deckY + 28, "I", infiniteWorldMode ? "Infinite" : "Bounded", infiniteWorldMode) + 6;
            DrawKeyBind(x5, deckY + 28, "1-5", "Cost");
            int x6 = c3X;
            x6 += DrawKeyBind(x6, deckY + 54, "P", dijkstra.isPlaying() ? "Pause" : "Play") + 8;
            x6 += DrawKeyBind(x6, deckY + 54, "Left/Right", "Step") + 8;
            DrawKeyBind(x6, deckY + 54, "Enter", "Finish");

            // --- Column 4: View & Simulation Toggles ---
            int c4X = deckX + 16 + static_cast<int>(colW * 3);
            DrawLine(c4X - 12, deckY + 10, c4X - 12, deckY + deckH - 10, Color{ 35, 48, 70, 255 });
            DrawText("VIEW & SIMULATION TOGGLES", c4X, deckY + 10, 10, Color{ 190, 160, 240, 255 });
            int x7 = c4X;
            x7 += DrawKeyBind(x7, deckY + 28, "H", "HUD") + 6;
            x7 += DrawKeyBind(x7, deckY + 28, "V", "Cam") + 6;
            x7 += DrawKeyBind(x7, deckY + 28, "T", "Terrain", showTerrain) + 6;
            x7 += DrawKeyBind(x7, deckY + 28, "K", "Wire", showWireframe) + 6;
            DrawKeyBind(x7, deckY + 28, "E", "Edges", showEdges);

            int x8 = c4X;
            x8 += DrawKeyBind(x8, deckY + 54, "N", "Nodes", showNodes) + 6;
            x8 += DrawKeyBind(x8, deckY + 54, "O", "Rocks", showBoulders) + 6;
            x8 += DrawKeyBind(x8, deckY + 54, "Z", "Spheres", showSpheres) + 6;
            x8 += DrawKeyBind(x8, deckY + 54, "B/X", "Drop") + 6;
            DrawKeyBind(x8, deckY + 54, "C", "Clear");
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

        static int s_frameCounter = 0;
        s_frameCounter++;
        const char* screenshotPath = getenv("ROVER_SCREENSHOT_PATH");
        const char* frameTarget = getenv("ROVER_SCREENSHOT_FRAMES");
        if (screenshotPath && frameTarget && s_frameCounter == atoi(frameTarget)) {
            TakeScreenshot(screenshotPath);
            break;
        }

        EndDrawing();
    }

    // Cleanup & Exit
    chunkMgr.clear(physics);
    physics.shutdown();
    terrain.unload();
    navGraph.clear();
    CloseWindow();
    return 0;
}
