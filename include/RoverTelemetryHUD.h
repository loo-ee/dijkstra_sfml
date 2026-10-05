#pragma once
#include <raylib.h>

class PlanetaryRover;
class DijkstraSolver3D;
class RoverNavGraph;

class RoverTelemetryHUD {
public:
    static void draw(const PlanetaryRover& rover, int screenW, int screenH, float sceneTime);

private:
    static void drawArtificialHorizon(int centerX, int centerY, int radius, float pitchDeg, float rollDeg, bool criticalRollover, float sceneTime);
    static void drawSpeedometer(int centerX, int centerY, int radius, float speedMps, float maxSpeedMps);
    static void drawSlipMonitor(int x, int y, int width, int height, const PlanetaryRover& rover);
    static void drawEnergyBar(int x, int y, int width, int height, float powerKw, float energyKj);
};
