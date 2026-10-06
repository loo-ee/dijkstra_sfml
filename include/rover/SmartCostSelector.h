#pragma once
#include <string>
#include <algorithm>
#include "TerrainHeightfield.h"
#include "DijkstraSolver3D.h"

enum class SmartCostMode {
    MANUAL = 0,             // User manually selects preset 1..5
    SMART_AUTO = 1          // Automatically selects optimal cost profile based on fuel % and terrain
};

struct SmartCostDecision {
    int selectedPresetIndex = 0;
    std::string strategyName = "Balanced Exploration";
    std::string rationale = "Standard Martian exploration profile";
    CostWeights weights = {};
    bool isNeural = false;
};

class SmartCostSelector {
public:
    static inline SmartCostDecision evaluateStrategy(
        float batteryPercent,
        TerrainPreset terrainPreset,
        float avgSlopeDeg = 0.0f,
        bool forceEco = false
    ) {
        SmartCostDecision dec;

        // 1. Critical Low Fuel (< 25%): Strict Energy Saver / Contour Routing
        if (batteryPercent < 25.0f || forceEco) {
            dec.selectedPresetIndex = 2; // Energy Saver (Contour Routing)
            dec.strategyName = "ECO-CONTOUR (LOW FUEL)";
            dec.rationale = "Battery < 25%: Navigating along flat contours to prevent energy depletion.";
            dec.weights.alpha = 5.2f; // High uphill penalty
            dec.weights.beta = 2.5f;
            dec.weights.gamma = 1.8f;
            dec.weights.delta = 1.0f;
            dec.weights.name = "Smart: Eco-Contour";
            dec.isNeural = false;
            return dec;
        }

        // 2. High Ruggedness / Canyon / Scree Slope: Traction Safety First / Neural Physics
        if (terrainPreset == TerrainPreset::BOULDER_SLALOM || terrainPreset == TerrainPreset::SCREE_SLOPE || avgSlopeDeg > 10.0f) {
            if (batteryPercent > 50.0f) {
                dec.selectedPresetIndex = 4; // Neural Network Traversability (MLP)
                dec.strategyName = "NEURAL-MLP TRACTION";
                dec.rationale = "Rugged terrain: Evaluating 8D physics terramechanics via trained neural network.";
                dec.weights.alpha = 2.0f;
                dec.weights.beta = 2.5f;
                dec.weights.gamma = 1.8f;
                dec.weights.delta = 1.0f;
                dec.weights.name = "Smart: Neural MLP";
                dec.isNeural = true;
            } else {
                dec.selectedPresetIndex = 3; // High-Traction Safety First
                dec.strategyName = "TRACTION SAFETY";
                dec.rationale = "Moderate fuel on rocks/scree: Heavily penalizing wheel slip and steep grades.";
                dec.weights.alpha = 2.2f;
                dec.weights.beta = 4.8f;
                dec.weights.gamma = 3.2f;
                dec.weights.delta = 1.2f;
                dec.weights.name = "Smart: Traction Safety";
                dec.isNeural = false;
            }
            return dec;
        }

        // 3. Flat Plains (Acidalia Planitia) & High Fuel (> 65%): Direct Express Route
        if (terrainPreset == TerrainPreset::ACIDALIA_PLANITIA && batteryPercent > 65.0f) {
            dec.selectedPresetIndex = 1; // Direct / Euclidean
            dec.strategyName = "DIRECT EXPRESS (FAST)";
            dec.rationale = "High battery & gentle plains: Prioritizing minimal distance for fast transit.";
            dec.weights.alpha = 0.5f;
            dec.weights.beta = 0.6f;
            dec.weights.gamma = 0.3f;
            dec.weights.delta = 0.3f;
            dec.weights.name = "Smart: Direct Express";
            dec.isNeural = false;
            return dec;
        }

        // 4. Default Balanced Exploration (Olympus Crater / Nominal Fuel)
        dec.selectedPresetIndex = 0; // Standard Martian Rover
        dec.strategyName = "BALANCED EXPLORATION";
        dec.rationale = "Nominal fuel & topography: Balanced trade-off of battery, distance, and slope safety.";
        dec.weights.alpha = 2.0f;
        dec.weights.beta = 1.5f;
        dec.weights.gamma = 1.0f;
        dec.weights.delta = 0.8f;
        dec.weights.name = "Smart: Balanced";
        dec.isNeural = false;
        return dec;
    }
};
