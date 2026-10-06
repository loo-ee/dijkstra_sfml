#pragma once
#include <raylib.h>

class PlanetaryRover;
class DijkstraSolver3D;
class RoverNavGraph;
class TerrainHeightfield;
class PhysicsWorld;
struct SmartCostDecision;

class RoverTelemetryHUD {
public:
    static bool isMouseOverUI(Vector2 mousePos, int screenW, int screenH, bool showHUD);

    static void draw(const PlanetaryRover& rover, int screenW, int screenH, float sceneTime, 
                     const SmartCostDecision* smartDecision = nullptr, bool isSmartAuto = false);

    static void drawMissionCard(int screenW, int screenH, Vector2 mousePos,
                                const TerrainHeightfield& terrain,
                                bool infiniteWorldMode, int activeChunkCount,
                                const PhysicsWorld& physics, int blockedEdgeCount,
                                const DijkstraSolver3D& dijkstra,
                                const RoverNavGraph& navGraph,
                                const SmartCostDecision& activeDecision,
                                bool isSmartAuto, int currentPresetIdx,
                                bool& outTogglePreset, bool& outToggleInfinite, bool& outHideHUD);

    static void drawLegendCard(int screenW, int screenH);

    static void drawCommandDeck(int screenW, int screenH,
                                const PlanetaryRover& rover,
                                const PhysicsWorld& physics,
                                bool infiniteWorldMode, bool isSmartAutoCost,
                                bool showTerrain, bool showWireframe,
                                bool showEdges, bool showNodes,
                                bool showBoulders, bool showSpheres, bool showHUD);

    static void drawShowHUDButton(int screenW, int screenH, Vector2 mousePos, bool& outShowHUD);

    static int drawKeyBind(int x, int y, const char* key, const char* label, bool active = true);

private:
    static void drawArtificialHorizon(int centerX, int centerY, int radius, float pitchDeg, float rollDeg, bool criticalRollover, float sceneTime);
    static void drawSpeedometer(int centerX, int centerY, int radius, float speedMps, float maxSpeedMps);
    static void drawSlipMonitor(int x, int y, int width, int height, const PlanetaryRover& rover);
    static void drawEnergyBar(int x, int y, int width, int height, const PlanetaryRover& rover);
    static void drawSmartCostCard(int x, int y, int width, int height, const SmartCostDecision* dec, bool isSmartAuto);
};


