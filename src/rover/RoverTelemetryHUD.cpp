#include "RoverTelemetryHUD.h"
#include "PlanetaryRover.h"
#include "SmartCostSelector.h"
#include "TerrainHeightfield.h"
#include "PhysicsWorld.h"
#include "DijkstraSolver3D.h"
#include "RoverNavGraph.h"
#include "GraphRenderer3D.h"
#include <raymath.h>
#include <cmath>
#include <string>

void RoverTelemetryHUD::draw(const PlanetaryRover& rover, int screenW, int screenH, float sceneTime, 
                             const SmartCostDecision* smartDecision, bool isSmartAuto) {
    if (!rover.isInitialized()) return;

    int cardW = 340;
    int cardH = 600;
    int cardX = screenW - cardW - 16;
    int cardY = 16;

    // Outer Glassmorphic Card Container
    Rectangle hudRect = { (float)cardX, (float)cardY, (float)cardW, (float)cardH };
    DrawRectangleRounded(hudRect, 0.04f, 4, ColorAlpha(Color{ 10, 14, 24, 255 }, 0.94f));
    DrawRectangleRoundedLines(hudRect, 0.04f, 4, Color{ 48, 68, 98, 255 });

    // Top Martian Copper Accent Stripe
    DrawRectangle(cardX + 1, cardY + 1, cardW - 2, 3, Color{ 220, 110, 50, 255 });

    // -----------------------------------------------------------------
    // 1. HEADER: Title & Autonomy Status Badge
    // -----------------------------------------------------------------
    DrawText("PLANETARY ROVER TELEMETRY", cardX + 16, cardY + 12, 13, RAYWHITE);

    // Camera Mode Pill (Clickable/Toggleable with V)
    const char* camStr = "CAM: ORBIT";
    if (rover.getCameraMode() == RoverCameraMode::CHASE) camStr = "CAM: CHASE (3P)";
    else if (rover.getCameraMode() == RoverCameraMode::MAST) camStr = "CAM: MAST (1P)";

    int camW = MeasureText(camStr, 10) + 12;
    Rectangle camRect = { (float)(cardX + cardW - camW - 14), (float)(cardY + 11), (float)camW, 18.0f };
    DrawRectangleRounded(camRect, 0.35f, 4, Color{ 26, 38, 56, 255 });
    DrawRectangleRoundedLines(camRect, 0.35f, 4, Color{ 70, 110, 160, 255 });
    DrawText(camStr, cardX + cardW - camW - 8, cardY + 15, 10, Color{ 140, 200, 255, 255 });

    // Autonomy Status Indicator Pill
    Color badgeBg, badgeBorder, badgeText;
    std::string statusStr;
    if (rover.isBatteryDepleted()) {
        badgeBg = Color{ 58, 16, 16, 255 };
        badgeBorder = RED;
        badgeText = Color{ 255, 120, 120, 255 };
        statusStr = "STATUS: BATTERY DEPLETED [RTG Trickle / Sh+U]";
    } else if (rover.hasReachedGoal()) {
        badgeBg = Color{ 16, 48, 52, 255 };
        badgeBorder = Color{ 0, 220, 240, 255 };
        badgeText = Color{ 0, 240, 255, 255 };
        statusStr = "STATUS: MISSION OBJECTIVE REACHED";
    } else if (rover.isSeekingDetour()) {
        badgeBg = Color{ 58, 40, 12, 255 };
        badgeBorder = Color{ 255, 185, 45, 255 };
        badgeText = Color{ 255, 205, 60, 255 };
        statusStr = TextFormat("STATUS: FLANKING DETOUR [Gap: %.1fm] (Seeking Path)", rover.getStandoffDistance());
    } else if (rover.isAtStandoffVantage()) {
        badgeBg = Color{ 58, 38, 12, 255 };
        badgeBorder = Color{ 255, 175, 45, 255 };
        badgeText = Color{ 255, 195, 65, 255 };
        statusStr = TextFormat("VANTAGE SIGHTLINE: %.1fm [Routing Around]", rover.getStandoffDistance());
    } else if (rover.isDirectHoming()) {
        badgeBg = Color{ 64, 22, 12, 255 };
        badgeBorder = Color{ 255, 100, 35, 255 };
        badgeText = Color{ 255, 125, 45, 255 };
        statusStr = "STATUS: OFF-ROAD DIRECT HOMING [Bumper Avoid]";
    } else if (rover.isReversing()) {
        badgeBg = Color{ 54, 28, 12, 255 };
        badgeBorder = Color{ 245, 130, 32, 255 };
        badgeText = Color{ 255, 160, 50, 255 };
        statusStr = "STATUS: REVERSE GEAR (OBSTACLE/SLOPE RECOVERY)";
    } else if (rover.isAutonomous()) {
        badgeBg = Color{ 16, 48, 32, 255 };
        badgeBorder = Color{ 46, 204, 113, 255 };
        badgeText = Color{ 46, 204, 113, 255 };
        statusStr = "STATUS: AUTONOMOUS PURE PURSUIT [Tab/F]";
    } else {
        badgeBg = Color{ 48, 38, 16, 255 };
        badgeBorder = Color{ 241, 196, 15, 255 };
        badgeText = Color{ 241, 196, 15, 255 };
        statusStr = "STATUS: MANUAL [WASD] (Auto: Tab/F)";
    }

    Rectangle statusRect = { (float)(cardX + 16), (float)(cardY + 32), (float)(cardW - 32), 20.0f };
    DrawRectangleRounded(statusRect, 0.25f, 4, badgeBg);
    DrawRectangleRoundedLines(statusRect, 0.25f, 4, badgeBorder);
    DrawText(statusStr.c_str(), cardX + 24, cardY + 37, 10, badgeText);

    DrawLine(cardX + 16, cardY + 58, cardX + cardW - 16, cardY + 58, Color{ 35, 48, 70, 255 });

    // -----------------------------------------------------------------
    // 2. DUAL INSTRUMENT GAUGES: Artificial Horizon & Speedometer
    // -----------------------------------------------------------------
    int gaugeRadius = 40;
    int horizonCenterX = cardX + 85;
    int gaugeCenterY  = cardY + 108;
    int speedCenterX   = cardX + cardW - 85;

    // Artificial Horizon (Attitude Gyro)
    drawArtificialHorizon(horizonCenterX, gaugeCenterY, gaugeRadius, 
                          rover.getPitchDeg(), rover.getRollDeg(), 
                          rover.isCriticalRollover(), sceneTime);

    // Speedometer Arc
    drawSpeedometer(speedCenterX, gaugeCenterY, gaugeRadius, rover.getSpeed(), 8.0f);

    // Readout Labels below gauges
    DrawText(TextFormat("P: %+4.1f°  R: %+4.1f°", rover.getPitchDeg(), rover.getRollDeg()), 
        cardX + 28, gaugeCenterY + gaugeRadius + 5, 10, Color{ 175, 195, 215, 255 });

    DrawText(TextFormat("Odom: %.1f m", rover.getOdometer()), 
        speedCenterX - 34, gaugeCenterY + gaugeRadius + 5, 10, Color{ 175, 195, 215, 255 });

    // Critical Rollover Alert Banner
    if (rover.isCriticalRollover()) {
        float blink = fmodf(sceneTime * 4.0f, 1.0f);
        Color warnBg = (blink > 0.5f) ? Color{ 180, 20, 20, 240 } : Color{ 120, 10, 10, 240 };
        Rectangle warnRect = { (float)(cardX + 16), (float)(gaugeCenterY + gaugeRadius + 18), (float)(cardW - 32), 20.0f };
        DrawRectangleRounded(warnRect, 0.25f, 4, warnBg);
        DrawRectangleRoundedLines(warnRect, 0.25f, 4, RED);
        DrawText("CRITICAL ROLLOVER HAZARD (>28°)", cardX + 32, gaugeCenterY + gaugeRadius + 22, 10, WHITE);
    }

    int nextY = cardY + 184;
    DrawLine(cardX + 16, nextY, cardX + cardW - 16, nextY, Color{ 35, 48, 70, 255 });

    // -----------------------------------------------------------------
    // 3. FUEL / BATTERY STATE OF CHARGE & RTG POWER
    // -----------------------------------------------------------------
    drawEnergyBar(cardX + 16, nextY + 6, cardW - 32, 60, rover);

    nextY += 72;
    DrawLine(cardX + 16, nextY, cardX + cardW - 16, nextY, Color{ 35, 48, 70, 255 });

    // -----------------------------------------------------------------
    // 4. SMART COST SELECTION & ADAPTIVE STRATEGY
    // -----------------------------------------------------------------
    drawSmartCostCard(cardX + 16, nextY + 6, cardW - 32, 52, smartDecision, isSmartAuto);

    nextY += 64;
    DrawLine(cardX + 16, nextY, cardX + cardW - 16, nextY, Color{ 35, 48, 70, 255 });

    // -----------------------------------------------------------------
    // 5. TRACTION CONTROL SYSTEM & 4-WHEEL SLIP MONITOR
    // -----------------------------------------------------------------
    drawSlipMonitor(cardX + 16, nextY + 6, cardW - 32, 92, rover);

    nextY += 106;
    DrawLine(cardX + 16, nextY, cardX + cardW - 16, nextY, Color{ 35, 48, 70, 255 });

    // -----------------------------------------------------------------
    // 6. AUTONOMOUS PURE PURSUIT WAYPOINT TRACKING
    // -----------------------------------------------------------------
    DrawText("WAYPOINT TRACKING & GUIDANCE", cardX + 16, nextY + 6, 10, Color{ 100, 190, 255, 255 });

    int curWp = rover.getCurrentWaypointIndex() + 1;
    int totalWp = rover.getTotalWaypoints();
    float progress = (totalWp > 0) ? (static_cast<float>(curWp) / totalWp * 100.0f) : 0.0f;

    DrawText(TextFormat("Waypoint %d of %d (%.0f%%)", curWp, totalWp, progress), 
        cardX + 16, nextY + 20, 11, RAYWHITE);
    DrawText(TextFormat("Cross-Track: %+4.2fm  |  Lookahead: %.1fm", 
        rover.getCrossTrackError(), rover.getLookaheadDistance()), 
        cardX + 16, nextY + 34, 10, Color{ 160, 180, 205, 255 });

    // Waypoint progress bar
    int barW = cardW - 32;
    int barH = 4;
    int barX = cardX + 16;
    int barY = nextY + 48;
    DrawRectangle(barX, barY, barW, barH, Color{ 25, 35, 52, 255 });
    float progressFrac = (totalWp > 1) ? (static_cast<float>(curWp - 1) / (totalWp - 1)) : 1.0f;
    DrawRectangle(barX, barY, static_cast<int>(barW * progressFrac), barH, Color{ 46, 204, 113, 255 });
    DrawCircle(barX + static_cast<int>(barW * progressFrac), barY + 2, 3, WHITE);
}

void RoverTelemetryHUD::drawArtificialHorizon(int centerX, int centerY, int radius, 
                                              float pitchDeg, float rollDeg, 
                                              bool criticalRollover, float sceneTime) {
    // Outer Dial Ring
    Color dialBorder = criticalRollover ? RED : Color{ 70, 95, 130, 255 };
    DrawCircle(centerX, centerY, radius + 2, Color{ 18, 24, 34, 255 });
    DrawCircleLines(centerX, centerY, radius + 2, dialBorder);

    // Sky upper circle background
    DrawCircle(centerX, centerY, radius, Color{ 25, 42, 68, 255 });

    // Pitch shift (pixels per degree)
    float pitchOffset = -pitchDeg * 0.75f;
    pitchOffset = Clamp(pitchOffset, -radius * 0.75f, radius * 0.75f);

    // Horizon line tilt angle
    float rollRad = rollDeg * DEG2RAD;
    float cosR = cosf(rollRad);
    float sinR = sinf(rollRad);

    // Ground sector tilted by roll
    float startGroundAngle = rollDeg;
    float endGroundAngle = rollDeg + 180.0f;
    Vector2 hCenter = { centerX - sinR * pitchOffset, centerY + cosR * pitchOffset };
    DrawCircleSector(hCenter, (float)radius, startGroundAngle, endGroundAngle, 20, Color{ 85, 45, 32, 255 });

    // Horizon Line
    Vector2 leftPt  = { hCenter.x - cosR * (float)radius, hCenter.y - sinR * (float)radius };
    Vector2 rightPt = { hCenter.x + cosR * (float)radius, hCenter.y + sinR * (float)radius };
    DrawLineEx(leftPt, rightPt, 2.0f, WHITE);

    // Pitch Ladder Lines (+15 deg, -15 deg)
    for (float deg : { 15.0f, -15.0f }) {
        float pOff = -(pitchDeg - deg) * 0.75f;
        Vector2 pCenter = { centerX - sinR * pOff, centerY + cosR * pOff };
        Vector2 p1 = { pCenter.x - cosR * 10.0f, pCenter.y - sinR * 10.0f };
        Vector2 p2 = { pCenter.x + cosR * 10.0f, pCenter.y + sinR * 10.0f };
        DrawLineEx(p1, p2, 1.5f, ColorAlpha(WHITE, 0.6f));
    }

    // Center Rover Reticle Crosshair (fixed with respect to chassis)
    DrawCircle(centerX, centerY, 3, GOLD);
    DrawLine(centerX - 14, centerY, centerX - 4, centerY, GOLD);
    DrawLine(centerX + 4, centerY, centerX + 14, centerY, GOLD);
    DrawLine(centerX, centerY - 2, centerX, centerY + 5, GOLD);

    // Outer Rim boundary
    DrawCircleLines(centerX, centerY, radius, ColorAlpha(WHITE, 0.4f));
}

void RoverTelemetryHUD::drawSpeedometer(int centerX, int centerY, int radius, float speedMps, float maxSpeedMps) {
    // Outer Dial Ring
    DrawCircle(centerX, centerY, radius + 2, Color{ 18, 24, 34, 255 });
    DrawCircleLines(centerX, centerY, radius + 2, Color{ 70, 95, 130, 255 });

    // Speedometer Sweep Arc: from 135 deg to 405 deg (270 deg span)
    const float startAngle = 135.0f;
    const float endAngle   = 405.0f;
    const float span = endAngle - startAngle;

    // Draw background track arc
    DrawRing(Vector2{ (float)centerX, (float)centerY }, radius - 7, radius - 2, startAngle, endAngle, 24, Color{ 30, 42, 58, 255 });

    // Active speed arc
    float currentFraction = Clamp(fabsf(speedMps) / maxSpeedMps, 0.0f, 1.0f);
    float activeEndAngle = startAngle + span * currentFraction;

    Color speedColor = Color{ 46, 204, 113, 255 }; // Green cruising
    if (currentFraction > 0.75f) {
        speedColor = Color{ 231, 76, 60, 255 }; // Red high
    } else if (currentFraction > 0.50f) {
        speedColor = Color{ 241, 196, 15, 255 }; // Yellow moderate
    }

    if (currentFraction > 0.01f) {
        DrawRing(Vector2{ (float)centerX, (float)centerY }, radius - 7, radius - 2, startAngle, activeEndAngle, 24, speedColor);
    }

    // Digital Numeric Speedometer Readout in Center
    float speedKmH = fabsf(speedMps) * 3.6f;
    std::string speedStr = TextFormat("%.1f", fabsf(speedMps));
    int textW = MeasureText(speedStr.c_str(), 16);
    DrawText(speedStr.c_str(), centerX - textW / 2, centerY - 14, 16, RAYWHITE);
    DrawText("m/s", centerX - 9, centerY + 3, 9, Color{ 140, 165, 190, 255 });
    DrawText(TextFormat("%.1f km/h", speedKmH), centerX - 24, centerY + 14, 9, speedColor);
}

void RoverTelemetryHUD::drawEnergyBar(int x, int y, int width, int height, const PlanetaryRover& rover) {
    DrawText("FUEL & RTG POWER STORAGE", x, y, 10, Color{ 100, 190, 255, 255 });

    float fuelPercent = rover.getBatteryPercent();
    float remKJ = rover.getBatteryRemainingKJ();
    float capKJ = rover.getBatteryCapacityKJ();
    float powerKw = rover.getCurrentPowerKW();
    float rangeM = rover.getEstimatedRangeMeters();

    // Fuel State of Charge text
    Color socColor = (fuelPercent > 50.0f) ? Color{ 46, 204, 113, 255 } :
                     (fuelPercent > 25.0f) ? Color{ 241, 196, 15, 255 } : Color{ 231, 76, 60, 255 };

    DrawText(TextFormat("SOC: %.1f%%  (%.0f / %.0f kJ)", fuelPercent, remKJ, capKJ), x, y + 14, 11, socColor);
    DrawText(TextFormat("Range: %.1f km  |  Draw: %.2f kW", rangeM * 0.001f, powerKw), x, y + 28, 10, Color{ 175, 195, 215, 255 });

    // Battery State of Charge Bar with Glow
    int barY = y + 43;
    int barH = 7;
    DrawRectangle(x, barY, width, barH, Color{ 25, 35, 52, 255 });

    float fuelFrac = Clamp(fuelPercent / 100.0f, 0.0f, 1.0f);
    int fillW = static_cast<int>(width * fuelFrac);
    DrawRectangle(x, barY, fillW, barH, socColor);
    DrawRectangleLines(x, barY, width, barH, Color{ 48, 68, 98, 255 });

    // RTG Charge indicator pip
    float netWatts = rover.getNetChargeWatts();
    if (netWatts > 0.0f) {
        DrawText(TextFormat("+%.0fW RTG", netWatts), x + width - 68, y, 9, Color{ 46, 204, 113, 255 });
    } else {
        DrawText(TextFormat("%.1fkW Net", -netWatts * 0.001f), x + width - 68, y, 9, Color{ 255, 160, 60, 255 });
    }
}

void RoverTelemetryHUD::drawSmartCostCard(int x, int y, int width, int height, const SmartCostDecision* dec, bool isSmartAuto) {
    DrawText("SMART PATHFINDING STRATEGY", x, y, 10, Color{ 100, 190, 255, 255 });

    // Auto / Manual Badge
    if (isSmartAuto) {
        DrawText("[AUTO SMART MGR]", x + width - 96, y, 9, Color{ 46, 230, 113, 255 });
    } else {
        DrawText("[MANUAL COST]", x + width - 82, y, 9, Color{ 170, 185, 205, 255 });
    }

    if (dec) {
        DrawText(dec->strategyName.c_str(), x, y + 14, 11, Color{ 255, 210, 80, 255 });
        
        // Rationale text (truncated if needed)
        DrawText(dec->rationale.c_str(), x, y + 28, 9, Color{ 160, 185, 210, 255 });

        // Weights: alpha (elevation), beta (slope), gamma (roughness), delta (turn)
        DrawText(TextFormat("Weights: a=%.1f  b=%.1f  g=%.1f  d=%.1f", 
            dec->weights.alpha, dec->weights.beta, dec->weights.gamma, dec->weights.delta), 
            x, y + 40, 9, Color{ 140, 165, 195, 255 });
    }
}

void RoverTelemetryHUD::drawSlipMonitor(int x, int y, int width, int height, const PlanetaryRover& rover) {
    DrawText("TRACTION CONTROL & TIRE SLIP", x, y, 10, Color{ 100, 190, 255, 255 });

    // Active Safety Badges: ESP (Stability), HDC (Descent Brake), TCS (Traction)
    int badgeRight = x + width;

    // 1. ESP Badge (Electronic Stability Program / Anti-Rollover)
    if (rover.isESPActive()) {
        int espW = 48;
        Rectangle espRect = { (float)(badgeRight - espW), (float)y - 2, (float)espW, 16.0f };
        DrawRectangleRounded(espRect, 0.3f, 4, Color{ 60, 20, 75, 255 });
        DrawRectangleRoundedLines(espRect, 0.3f, 4, Color{ 210, 110, 255, 255 });
        DrawText("ESP ACT", badgeRight - espW + 6, y + 1, 9, Color{ 235, 150, 255, 255 });
        badgeRight -= (espW + 4);
    }

    // 2. HDC Badge (Hill Descent Control / Downhill Brake)
    if (rover.isHDCActive()) {
        int hdcW = 54;
        Rectangle hdcRect = { (float)(badgeRight - hdcW), (float)y - 2, (float)hdcW, 16.0f };
        DrawRectangleRounded(hdcRect, 0.3f, 4, Color{ 75, 25, 20, 255 });
        DrawRectangleRoundedLines(hdcRect, 0.3f, 4, Color{ 255, 90, 80, 255 });
        DrawText("HDC BRK", badgeRight - hdcW + 6, y + 1, 9, Color{ 255, 120, 100, 255 });
        badgeRight -= (hdcW + 4);
    }

    // 3. Unstuck routine badge
    if (rover.isUnstuckActive()) {
        int ustW = 58;
        Rectangle ustRect = { (float)(badgeRight - ustW), (float)y - 2, (float)ustW, 16.0f };
        DrawRectangleRounded(ustRect, 0.3f, 4, Color{ 20, 50, 70, 255 });
        DrawRectangleRoundedLines(ustRect, 0.3f, 4, Color{ 80, 200, 255, 255 });
        DrawText("UNSTUCK", badgeRight - ustW + 6, y + 1, 9, Color{ 100, 220, 255, 255 });
        badgeRight -= (ustW + 4);
    }

    // 4. TCS Indicator Badge
    if (rover.isTCSActive()) {
        int tcsW = 58;
        Rectangle tcsRect = { (float)(badgeRight - tcsW), (float)y - 2, (float)tcsW, 16.0f };
        DrawRectangleRounded(tcsRect, 0.3f, 4, Color{ 80, 45, 10, 255 });
        DrawRectangleRoundedLines(tcsRect, 0.3f, 4, Color{ 255, 165, 0, 255 });
        DrawText("TCS ACT", badgeRight - tcsW + 8, y + 1, 9, Color{ 255, 195, 50, 255 });
    } else {
        DrawText("TCS RDY", badgeRight - 48, y + 1, 9, Color{ 100, 120, 140, 255 });
    }

    // 4 Tire Slip Bar Gauges: FL, FR, RL, RR
    const char* tireNames[4] = { "FL", "FR", "RL", "RR" };
    int colWidth = (width - 24) / 4;
    int barMaxH = 50;

    for (int i = 0; i < 4; ++i) {
        int bx = x + i * (colWidth + 8);
        int by = y + 20;

        const auto& wheel = rover.getWheel(i);
        float slip = wheel.slipRatio;
        float slipFrac = Clamp(slip, 0.0f, 1.0f);

        // Background track
        DrawRectangle(bx, by, colWidth, barMaxH, Color{ 22, 28, 40, 255 });
        DrawRectangleLines(bx, by, colWidth, barMaxH, Color{ 40, 52, 72, 255 });

        // Slip fill from bottom up
        int fillH = static_cast<int>(barMaxH * slipFrac);
        Color slipCol = Color{ 46, 204, 113, 255 }; // Normal grip (< 15%)
        if (slip > 0.25f) {
            slipCol = Color{ 231, 76, 60, 255 }; // Critical spin (> 25%)
        } else if (slip > 0.15f) {
            slipCol = Color{ 241, 196, 15, 255 }; // Moderate slip
        }

        DrawRectangle(bx + 1, by + barMaxH - fillH, colWidth - 2, fillH, slipCol);

        // Grounded contact indicator dot
        Color contactCol = wheel.isGrounded ? Color{ 46, 204, 113, 255 } : Color{ 100, 110, 125, 255 };
        DrawCircle(bx + colWidth / 2, by + barMaxH + 8, 3, contactCol);

        // Label
        DrawText(tireNames[i], bx + colWidth / 2 - 6, by + barMaxH + 14, 10, RAYWHITE);
        DrawText(TextFormat("%.0f%%", slip * 100.0f), bx + 2, by + barMaxH + 26, 9, slipCol);
    }
}

int RoverTelemetryHUD::drawKeyBind(int x, int y, const char* key, const char* label, bool active) {
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

bool RoverTelemetryHUD::isMouseOverUI(Vector2 mousePos, int screenW, int screenH, bool showHUD) {
    if (showHUD) {
        Rectangle hudRect = { 16.0f, 16.0f, 440.0f, 326.0f };
        Rectangle legRect = { 16.0f, 326.0f + 16.0f + 8.0f, 440.0f, 88.0f };
        int deckH = 92;
        Rectangle deckRect = { 16.0f, (float)(screenH - deckH - 16), (float)(screenW - 32), (float)deckH };
        Rectangle telemRect = { (float)(screenW - 340 - 16), 16.0f, 340.0f, 600.0f };

        return CheckCollisionPointRec(mousePos, hudRect) ||
               CheckCollisionPointRec(mousePos, legRect) ||
               CheckCollisionPointRec(mousePos, deckRect) ||
               CheckCollisionPointRec(mousePos, telemRect);
    } else {
        Rectangle btnRect = { (float)(screenW - 150 - 16), 16.0f, 150.0f, 32.0f };
        return CheckCollisionPointRec(mousePos, btnRect);
    }
}

void RoverTelemetryHUD::drawShowHUDButton(int screenW, int screenH, Vector2 mousePos, bool& outShowHUD) {
    (void)screenH;
    int btnW = 150;
    int btnH = 32;
    int btnX = screenW - btnW - 16;
    int btnY = 16;
    Rectangle btnRect = { (float)btnX, (float)btnY, (float)btnW, (float)btnH };
    bool isHovered = CheckCollisionPointRec(mousePos, btnRect);

    Color bg = isHovered ? Color{ 36, 52, 78, 240 } : Color{ 16, 22, 34, 210 };
    Color border = isHovered ? Color{ 80, 160, 255, 255 } : Color{ 48, 70, 105, 220 };
    DrawRectangleRounded(btnRect, 0.35f, 4, bg);
    DrawRectangleRoundedLines(btnRect, 0.35f, 4, border);

    DrawCircle(btnX + 16, btnY + 16, 4, Color{ 46, 204, 113, 255 });
    DrawText("[H] SHOW HUD", btnX + 28, btnY + 10, 11, RAYWHITE);

    if (isHovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        outShowHUD = true;
    }
}

void RoverTelemetryHUD::drawMissionCard(
    int screenW, int screenH, Vector2 mousePos,
    const TerrainHeightfield& terrain,
    bool infiniteWorldMode, int activeChunkCount,
    const PhysicsWorld& physics, int blockedEdgeCount,
    const DijkstraSolver3D& dijkstra,
    const RoverNavGraph& navGraph,
    const SmartCostDecision& activeDecision,
    bool isSmartAuto, int currentPresetIdx,
    bool& outTogglePreset, bool& outToggleInfinite, bool& outHideHUD)
{
    (void)screenW;
    (void)screenH;
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
        outHideHUD = true;
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
    DrawText(infiniteWorldMode ? TextFormat("[I] INF (%d Chk)", activeChunkCount) : "[I] BOUNDED", 
        hudX + hudW - 128, hudY + 57, 10, infiniteWorldMode ? Color{ 60, 230, 175, 255 } : Color{ 170, 180, 195, 255 });

    if (tmHovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        outTogglePreset = true;
    }
    if (infHovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        outToggleInfinite = true;
    }

    DrawLine(hudX + 16, hudY + 82, hudX + hudW - 16, hudY + 82, Color{ 35, 48, 70, 255 });

    // Row 2: Active Cost Preset & Parameters
    const auto& w = dijkstra.getWeights();
    if (isSmartAuto) {
        DrawText(TextFormat("Smart Auto-Cost [0]: %s", activeDecision.strategyName.c_str()), 
            hudX + 16, hudY + 90, 11, Color{ 0, 240, 255, 255 });
        DrawText(TextFormat("Rationale: %s", activeDecision.rationale.c_str()), 
            hudX + 16, hudY + 106, 10, Color{ 140, 215, 245, 255 });
    } else if (dijkstra.isNeuralMode()) {
        DrawText(TextFormat("Manual Preset [5]: %s (Auto: [0])", w.name.c_str()), 
            hudX + 16, hudY + 90, 11, Color{ 60, 230, 175, 255 });
        DrawText("Engine: 3-Layer PINN MLP (8 -> 32 -> 16 -> 1) | <0.05 us/edge | Zero-Alloc", 
            hudX + 16, hudY + 106, 10, Color{ 140, 235, 205, 255 });
    } else {
        DrawText(TextFormat("Manual Preset [%d]: %s (Auto: [0])", currentPresetIdx + 1, w.name.c_str()), 
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
}

void RoverTelemetryHUD::drawLegendCard(int screenW, int screenH) {
    (void)screenW;
    (void)screenH;
    int hudY = 16;
    int hudH = 326;
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
}

void RoverTelemetryHUD::drawCommandDeck(
    int screenW, int screenH,
    const PlanetaryRover& rover,
    const PhysicsWorld& physics,
    bool infiniteWorldMode, bool isSmartAutoCost,
    bool showTerrain, bool showWireframe,
    bool showEdges, bool showNodes,
    bool showBoulders, bool showSpheres, bool showHUD)
{
    (void)showHUD;
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
    x1 += drawKeyBind(x1, deckY + 28, "RMB Drag", "Orbit") + 8;
    drawKeyBind(x1, deckY + 28, "Wheel", "Zoom");
    int x2 = c1X;
    x2 += drawKeyBind(x2, deckY + 54, "MMB", "Pan") + 8;
    x2 += drawKeyBind(x2, deckY + 54, "F", "Focus") + 8;
    drawKeyBind(x2, deckY + 54, "Sh+R", "Reset");

    // --- Column 2: Rover Navigation ---
    int c2X = deckX + 16 + static_cast<int>(colW);
    DrawLine(c2X - 12, deckY + 10, c2X - 12, deckY + deckH - 10, Color{ 35, 48, 70, 255 });
    DrawText("ROVER NAVIGATION (WASD)", c2X, deckY + 10, 10, Color{ 46, 204, 113, 255 });
    int x3 = c2X;
    x3 += drawKeyBind(x3, deckY + 28, "Tab", rover.isAutonomous() ? "Manual" : "Auto Drive") + 6;
    x3 += drawKeyBind(x3, deckY + 28, "WASD", "Drive") + 6;
    drawKeyBind(x3, deckY + 28, "Space", "Brake");
    int x4 = c2X;
    x4 += drawKeyBind(x4, deckY + 54, "T", "Traverse") + 6;
    x4 += drawKeyBind(x4, deckY + 54, "R", "Reset") + 6;
    x4 += drawKeyBind(x4, deckY + 54, "U/Sh+U", "Right/Fuel") + 6;
    const char* gKeyName = "Mars G";
    if (fabsf(physics.getGravity() + 1.62f) < 0.2f) gKeyName = "Moon G";
    else if (fabsf(physics.getGravity() + 9.81f) < 0.5f) gKeyName = "Earth G";
    drawKeyBind(x4, deckY + 54, "G", gKeyName);

    // --- Column 3: Terrain & Dijkstra ---
    int c3X = deckX + 16 + static_cast<int>(colW * 2);
    DrawLine(c3X - 12, deckY + 10, c3X - 12, deckY + deckH - 10, Color{ 35, 48, 70, 255 });
    DrawText("TERRAIN & DIJKSTRA", c3X, deckY + 10, 10, Color{ 241, 196, 15, 255 });
    int x5 = c3X;
    x5 += drawKeyBind(x5, deckY + 28, "M", "Preset") + 6;
    x5 += drawKeyBind(x5, deckY + 28, "I", infiniteWorldMode ? "Inf" : "Bound", infiniteWorldMode) + 6;
    drawKeyBind(x5, deckY + 28, "0", isSmartAutoCost ? "AutoCost" : "Manual", isSmartAutoCost);
    int x6 = c3X;
    x6 += drawKeyBind(x6, deckY + 54, "1-5", "Preset") + 6;
    x6 += drawKeyBind(x6, deckY + 54, "P", "Play") + 6;
    drawKeyBind(x6, deckY + 54, "Enter", "Finish");

    // --- Column 4: View & Simulation Toggles ---
    int c4X = deckX + 16 + static_cast<int>(colW * 3);
    DrawLine(c4X - 12, deckY + 10, c4X - 12, deckY + deckH - 10, Color{ 35, 48, 70, 255 });
    DrawText("VIEW & SIMULATION TOGGLES", c4X, deckY + 10, 10, Color{ 190, 160, 240, 255 });
    int x7 = c4X;
    x7 += drawKeyBind(x7, deckY + 28, "H", "HUD") + 6;
    x7 += drawKeyBind(x7, deckY + 28, "V", "Cam") + 6;
    x7 += drawKeyBind(x7, deckY + 28, "T", "Terrain", showTerrain) + 6;
    x7 += drawKeyBind(x7, deckY + 28, "K", "Wire", showWireframe) + 6;
    drawKeyBind(x7, deckY + 28, "E", "Edges", showEdges);

    int x8 = c4X;
    x8 += drawKeyBind(x8, deckY + 54, "N", "Nodes", showNodes) + 6;
    x8 += drawKeyBind(x8, deckY + 54, "O", "Rocks", showBoulders) + 6;
    x8 += drawKeyBind(x8, deckY + 54, "Z", "Spheres", showSpheres) + 6;
    x8 += drawKeyBind(x8, deckY + 54, "B/X", "Drop") + 6;
    drawKeyBind(x8, deckY + 54, "C", "Clear");
}
