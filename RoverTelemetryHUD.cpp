#include "RoverTelemetryHUD.h"
#include "PlanetaryRover.h"
#include "SmartCostSelector.h"
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
