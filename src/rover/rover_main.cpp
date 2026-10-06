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
#include "SmartCostSelector.h"

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

    navGraph.setPhysicsWorld(&physics);

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

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
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
    bool isSmartAutoCost = true;
    SmartCostDecision activeSmartDecision = SmartCostSelector::evaluateStrategy(100.0f, TerrainPreset::OLYMPUS_CRATER);
    if (activeSmartDecision.isNeural) {
        dijkstra.applyPreset(4);
    } else {
        dijkstra.setWeights(activeSmartDecision.weights);
    }
    currentPresetIndex = activeSmartDecision.selectedPresetIndex;

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
        rover.update(physics, dt, &terrain);

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
        if (distFromLastLazy > 24.0f && (now - lastDiscoveryTime > 0.20f)) {
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
            Vector3 roverFwd = rover.getForward();

            // Block the impassable edge near the probed hazard position
            Vector3 hazardPos = rover.getHazardPos();
            if (Vector3LengthSqr(hazardPos) > 0.1f) {
                if (navGraph.blockEdgeNearPosition(hazardPos, 8.0f)) {
                    // Update blocked edge count
                    blockedEdgeCount = 0;
                    for (const auto& e : navGraph.getEdges()) {
                        if (e.isBlocked) blockedEdgeCount++;
                    }
                }
            }

            // Ensure grid around rover and goal is populated with nodes so detour paths can be found
            if (navGraph.getEndNode()) {
                navGraph.ensureCorridor(terrain, roverPos, navGraph.getEndNode()->position, navGraph.getSpacing());
            }
            navGraph.generatePersistentPlanetaryGrid(terrain, roverPos, 130.0f, navGraph.getSpacing());

            // Exclude visited standoff vantage points to prevent 2-node ping-pong looping
            const auto& visitedVantage = rover.getVisitedVantageHistory();
            std::unordered_set<std::string> excludedSet(visitedVantage.begin(), visitedVantage.end());
            dijkstra.setExcludedVantageNodes(excludedSet);

            // Find candidate walkable nodes near the rover (including forward, left flank, and right flank)
            struct CandidateNode {
                Vertex3D* node;
                float score;
            };
            std::vector<CandidateNode> candidates;
            for (Vertex3D* v : navGraph.getVertices()) {
                if (!v || !v->isWalkable) continue;
                Vector3 toV = Vector3Subtract(v->position, roverPos);
                float dot = Vector3DotProduct(toV, roverFwd);
                float dsq = Vector3DistanceSqr(roverPos, v->position);
                if (dsq > 48.0f * 48.0f) continue;
                bool isExcluded = (excludedSet.find(v->name) != excludedSet.end());
                // Prioritize forward and lateral flanking unvisited nodes
                float score = (dot > -0.2f ? dsq : (dsq + 50.0f)) + (isExcluded ? 1000.0f : 0.0f);
                candidates.push_back({ v, score });
            }
            std::sort(candidates.begin(), candidates.end(), [](const CandidateNode& a, const CandidateNode& b) {
                return a.score < b.score;
            });

            if (candidates.empty()) {
                Vertex3D* walkable = navGraph.getClosestWalkableNode(roverPos);
                if (walkable) candidates.push_back({ walkable, 0.0f });
            }

            if (!candidates.empty() && navGraph.getEndNode()) {
                std::vector<const Vertex3D*> bestPath;
                bool bestIsPartial = true;
                float bestDistToGoal = 1e9f;

                // Evaluate candidate starting nodes around the rover to discover flank detours
                for (size_t i = 0; i < std::min<size_t>(candidates.size(), 8); ++i) {
                    Vertex3D* candNode = candidates[i].node;
                    if (!candNode || candNode == navGraph.getEndNode()) continue;

                    dijkstra.solveWithHistory(candNode, navGraph.getEndNode(),
                                              navGraph.getVertices(), navGraph.getBlockedEdgesMap());
                    const auto& path = dijkstra.getShortestPathNodes();
                    if (!path.empty()) {
                        bool isPartial = dijkstra.isPartialPath();
                        float distToGoal = dijkstra.getDistanceToGoal();
                        if (!isPartial) {
                            // Complete route around obstacle discovered!
                            bestPath = path;
                            bestIsPartial = false;
                            bestDistToGoal = 0.0f;
                            break;
                        } else if (distToGoal < bestDistToGoal) {
                            bestPath = path;
                            bestIsPartial = true;
                            bestDistToGoal = distToGoal;
                        }
                    }
                }

                if (!bestPath.empty()) {
                    Vector3 goalPos = navGraph.getEndNode() ? navGraph.getEndNode()->position : Vector3{ 0, 0, 0 };
                    rover.setPath(bestPath, bestIsPartial, bestDistToGoal, goalPos);
                } else {
                    rover.engageDirectHoming(navGraph.getEndNode()->position);
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
        bool isOverUI = RoverTelemetryHUD::isMouseOverUI(mousePos, screenW, screenH, showHUD);

        // 3D Raycast Mouse Picking for Nodes (prevent picking if clicking on UI cards)
        Vertex3D* hoveredNode = nullptr;
        if (!isOverUI) {
            Ray mouseRay = GetMouseRay(mousePos, activeCamera);
            hoveredNode = navGraph.pickNodeFromRay(mouseRay, 1.8f);

            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                if (hoveredNode) {
                    rover.clearVisitedVantageHistory();
                    dijkstra.clearExcludedVantageNodes();
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

        // Real-Time Smart Cost Selector Engine (Adapts weights dynamically to Battery % and Terrain Topography)
        if (isSmartAutoCost) {
            SmartCostDecision newDec = SmartCostSelector::evaluateStrategy(
                rover.getBatteryPercent(),
                terrain.getPreset()
            );
            if (newDec.strategyName != activeSmartDecision.strategyName || newDec.isNeural != activeSmartDecision.isNeural) {
                activeSmartDecision = newDec;
                currentPresetIndex = activeSmartDecision.selectedPresetIndex;
                if (activeSmartDecision.isNeural) {
                    dijkstra.applyPreset(4);
                } else {
                    dijkstra.setWeights(activeSmartDecision.weights);
                }
                if (navGraph.getStartNode() && navGraph.getEndNode()) {
                    dijkstra.solveWithHistory(navGraph.getStartNode(), navGraph.getEndNode(), 
                                              navGraph.getVertices(), navGraph.getBlockedEdgesMap());
                    Vector3 goalPos = navGraph.getEndNode() ? navGraph.getEndNode()->position : Vector3{ 0, 0, 0 };
                    rover.setPath(dijkstra.getShortestPathNodes(), dijkstra.isPartialPath(), dijkstra.getDistanceToGoal(), goalPos);
                }
            }
        }

        // Update Dijkstra Step-by-Step Playback
        dijkstra.update(dt);

        // Cost Preset Selection Keys: 1, 2, 3, 4, 5 & Smart Auto Toggle: 0
        auto applyPresetAndRoute = [&](int presetIdx) {
            isSmartAutoCost = false;
            currentPresetIndex = presetIdx;
            dijkstra.applyPreset(presetIdx);
            dijkstra.solveWithHistory(navGraph.getStartNode(), navGraph.getEndNode(), 
                                      navGraph.getVertices(), navGraph.getBlockedEdgesMap());
            Vector3 goalPos = navGraph.getEndNode() ? navGraph.getEndNode()->position : Vector3{ 0, 0, 0 };
            rover.setPath(dijkstra.getShortestPathNodes(), dijkstra.isPartialPath(), dijkstra.getDistanceToGoal(), goalPos);
        };

        if (IsKeyPressed(KEY_ZERO)) {
            isSmartAutoCost = !isSmartAutoCost;
            if (isSmartAutoCost) {
                activeSmartDecision = SmartCostSelector::evaluateStrategy(rover.getBatteryPercent(), terrain.getPreset());
                currentPresetIndex = activeSmartDecision.selectedPresetIndex;
                if (activeSmartDecision.isNeural) {
                    dijkstra.applyPreset(4);
                } else {
                    dijkstra.setWeights(activeSmartDecision.weights);
                }
                if (navGraph.getStartNode() && navGraph.getEndNode()) {
                    dijkstra.solveWithHistory(navGraph.getStartNode(), navGraph.getEndNode(), 
                                              navGraph.getVertices(), navGraph.getBlockedEdgesMap());
                    Vector3 goalPos = navGraph.getEndNode() ? navGraph.getEndNode()->position : Vector3{ 0, 0, 0 };
                    rover.setPath(dijkstra.getShortestPathNodes(), dijkstra.isPartialPath(), dijkstra.getDistanceToGoal(), goalPos);
                }
            }
        }
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
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                // Direct Off-Road Infiltration / Traverse to Goal Beacon (Shift+T)
                if (navGraph.getEndNode()) {
                    rover.engageDirectHoming(navGraph.getEndNode()->position);
                }
            } else {
                showTerrain = !showTerrain;
            }
        }
        if (IsKeyPressed(KEY_F)) {
            cameraController.focusOn(rover.getPosition(), 22.0f);
        }
        if (IsKeyPressed(KEY_M)) {
            terrain.cyclePreset();
            ApplyTerrainPreset(terrain.getPreset(), terrain, chunkMgr, infiniteWorldMode, physics, navGraph, dijkstra, rover, blockedEdgeCount);
            lastNavGraphCenter = rover.getPosition();
            if (isSmartAutoCost) {
                activeSmartDecision = SmartCostSelector::evaluateStrategy(rover.getBatteryPercent(), terrain.getPreset());
                currentPresetIndex = activeSmartDecision.selectedPresetIndex;
                if (activeSmartDecision.isNeural) dijkstra.applyPreset(4);
                else dijkstra.setWeights(activeSmartDecision.weights);
            }
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
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                rover.refuel();
            } else {
                rover.selfRight(physics);
            }
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

            // C. Draw Static Boulders (Craggy rock shading with sunlit facets and distance LOD)
            if (showBoulders) {
                Vector3 camPos = activeCamera.position;
                for (const auto& b : physics.getBoulders()) {
                    float distToCam = Vector3Distance(camPos, b.pos);
                    if (distToCam > 280.0f) continue;
                    DrawSphere(b.pos, b.radius, Color{ 78, 56, 48, 255 });
                    if (distToCam < 160.0f) {
                        DrawSphere(Vector3Add(b.pos, Vector3{ 0.0f, b.radius * 0.18f, 0.0f }), b.radius * 0.88f, Color{ 115, 88, 76, 255 });
                        DrawSphereWires(b.pos, b.radius, 6, 6, ColorAlpha(Color{ 35, 24, 20, 255 }, 0.45f));
                    }
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

            // G. Draw Draped NavGraph Nodes (Prominent Glowing 3D Spheres with Distance LOD Culling)
            if (showNodes) {
                Vector3 camPos = activeCamera.position;
                for (const Vertex3D* v : navGraph.getVertices()) {
                    if (v == navGraph.getStartNode() || v == navGraph.getEndNode()) {
                        continue; // Drawn prominently below
                    }

                    float distToCam = Vector3Distance(camPos, v->position);
                    if (distToCam > 175.0f) {
                        continue; // Skip rendering distant nodes
                    }

                    float surfY = terrain.getHeight(v->position.x, v->position.z);
                    Vector3 baseAnchor = { v->position.x, surfY + 0.05f, v->position.z };

                    if (v->state == NodeState::IMPASSABLE) {
                        // Prominent Hazard Node on steep slopes / cliffs / boulder hazards
                        DrawSphere(v->position, 0.75f, Color{ 235, 65, 50, 220 });
                        DrawLine3D(baseAnchor, v->position, ColorAlpha(Color{ 235, 65, 50, 255 }, 0.70f));
                        if (distToCam < 90.0f) {
                            DrawSphereWires(v->position, 0.95f, 4, 4, ColorAlpha(RED, 0.50f));
                            DrawCircle3D(baseAnchor, 0.45f, Vector3{ 0, 1, 0 }, 90.0f, ColorAlpha(RED, 0.40f));
                        }
                        continue;
                    }

                    Color nodeCol = GraphRenderer3D::getNodeColor(v->state);
                    float r = 0.85f; // Prominently visible from panoramic orbit camera!
                    DrawSphere(v->position, r, nodeCol);
                    DrawLine3D(baseAnchor, v->position, ColorAlpha(nodeCol, 0.75f));
                    if (distToCam < 95.0f) {
                        DrawSphereWires(v->position, r * 1.25f, 6, 6, ColorAlpha(nodeCol, 0.60f));
                        DrawCircle3D(baseAnchor, 0.50f, Vector3{ 0, 1, 0 }, 90.0f, ColorAlpha(nodeCol, 0.45f));
                    }
                }

                // Prominent START Beacon with 26m vertical laser beam and pulsating radar ground rings
                if (const Vertex3D* s = navGraph.getStartNode()) {
                    float sSurfY = terrain.getHeight(s->position.x, s->position.z);
                    Vector3 sBase = { s->position.x, sSurfY + 0.05f, s->position.z };
                    Vector3 pillarTop = Vector3Add(s->position, Vector3{ 0.0f, 26.0f, 0.0f });
                    DrawCylinderEx(sBase, pillarTop, 0.40f, 0.05f, 10, ColorAlpha(Color{ 46, 230, 113, 255 }, 0.85f));
                    DrawSphere(s->position, 1.8f, Color{ 46, 230, 113, 255 });
                    DrawSphereWires(s->position, 2.3f, 8, 8, WHITE);
                    float pulseR = 3.5f + sinf(sceneTime * 4.0f) * 0.8f;
                    DrawCircle3D(sBase, pulseR, Vector3{ 0, 1, 0 }, 90.0f, ColorAlpha(Color{ 46, 230, 113, 255 }, 0.75f));
                    DrawCircle3D(sBase, pulseR * 1.5f, Vector3{ 0, 1, 0 }, 90.0f, ColorAlpha(Color{ 46, 230, 113, 255 }, 0.35f));
                }

                // Prominent END Beacon with 26m vertical laser beam and pulsating radar ground rings
                if (const Vertex3D* e = navGraph.getEndNode()) {
                    float eSurfY = terrain.getHeight(e->position.x, e->position.z);
                    Vector3 eBase = { e->position.x, eSurfY + 0.05f, e->position.z };
                    Vector3 pillarTop = Vector3Add(e->position, Vector3{ 0.0f, 26.0f, 0.0f });
                    DrawCylinderEx(eBase, pillarTop, 0.40f, 0.05f, 10, ColorAlpha(Color{ 235, 60, 60, 255 }, 0.85f));
                    DrawSphere(e->position, 1.8f, Color{ 235, 60, 60, 255 });
                    DrawSphereWires(e->position, 2.3f, 8, 8, WHITE);
                    float pulseR = 3.5f + sinf(sceneTime * 4.0f + 1.5f) * 0.8f;
                    DrawCircle3D(eBase, pulseR, Vector3{ 0, 1, 0 }, 90.0f, ColorAlpha(Color{ 235, 60, 60, 255 }, 0.75f));
                    DrawCircle3D(eBase, pulseR * 1.5f, Vector3{ 0, 1, 0 }, 90.0f, ColorAlpha(Color{ 235, 60, 60, 255 }, 0.35f));
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

            bool togglePreset = false;
            bool toggleInf = false;
            bool hideHUD = false;

            RoverTelemetryHUD::drawMissionCard(
                screenW, screenH, mousePos,
                terrain, infiniteWorldMode, chunkMgr.getActiveChunkCount(),
                physics, blockedEdgeCount, dijkstra, navGraph,
                activeSmartDecision, isSmartAutoCost, currentPresetIndex,
                togglePreset, toggleInf, hideHUD
            );

            if (hideHUD) showHUD = false;
            if (togglePreset) {
                terrain.cyclePreset();
                ApplyTerrainPreset(terrain.getPreset(), terrain, chunkMgr, infiniteWorldMode, physics, navGraph, dijkstra, rover, blockedEdgeCount);
                lastNavGraphCenter = rover.getPosition();
            }
            if (toggleInf) {
                infiniteWorldMode = !infiniteWorldMode;
                ApplyTerrainPreset(terrain.getPreset(), terrain, chunkMgr, infiniteWorldMode, physics, navGraph, dijkstra, rover, blockedEdgeCount);
                lastNavGraphCenter = rover.getPosition();
            }

            RoverTelemetryHUD::drawLegendCard(screenW, screenH);
            RoverTelemetryHUD::draw(rover, screenW, screenH, sceneTime, &activeSmartDecision, isSmartAutoCost);
            RoverTelemetryHUD::drawCommandDeck(
                screenW, screenH, rover, physics, infiniteWorldMode, isSmartAutoCost,
                showTerrain, showWireframe, showEdges, showNodes, showBoulders, showSpheres, showHUD
            );
        } else {
            bool requestShowHUD = false;
            RoverTelemetryHUD::drawShowHUDButton(screenW, screenH, mousePos, requestShowHUD);
            if (requestShowHUD) {
                showHUD = true;
            }
        }

        EndDrawing();

        static int s_frameCounter = 0;
        s_frameCounter++;
        const char* screenshotPath = getenv("ROVER_SCREENSHOT_PATH");
        const char* frameTarget = getenv("ROVER_SCREENSHOT_FRAMES");
        if (screenshotPath && frameTarget && s_frameCounter == atoi(frameTarget)) {
            TakeScreenshot(screenshotPath);
            break;
        }
    }

    // Cleanup & Exit
    chunkMgr.clear(physics);
    physics.shutdown();
    terrain.unload();
    navGraph.clear();
    CloseWindow();
    return 0;
}
