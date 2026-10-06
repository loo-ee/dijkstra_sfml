#pragma once
#include <raylib.h>
#include <raymath.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <queue>
#include <memory>
#include <limits>
#include <chrono>

#include "Vertex3D.h"

struct CostWeights {
    float alpha = 2.0f;  // Uphill gravitational work penalty multiplier
    float beta = 1.5f;   // Slope slip penalty multiplier
    float gamma = 1.0f;  // Surface friction roughness penalty multiplier
    float delta = 0.8f;  // Directional heading change penalty multiplier
    std::string name = "Standard Martian Rover";
};

struct DijkstraSnapshot3D {
    std::string currentNode;
    std::string examiningNeighbor;
    std::string message;

    std::unordered_map<std::string, float> distances;
    std::unordered_map<std::string, std::string> parents;
    std::vector<std::string> visitedNodes;
    std::vector<std::pair<std::string, float>> priorityQueue;

    bool isFinished = false;
    bool pathFound = false;
    float candidateEdgeCost = 0.0f;
};

struct PathStats {
    float totalDistance = 0.0f;
    float totalEnergyCost = 0.0f;
    float elevationGain = 0.0f;
    float elevationLoss = 0.0f;
    float maxSlopeDeg = 0.0f;
    int waypointCount = 0;
    float computeTimeMs = 0.0f;
    bool isValid = false;
    bool isPartial = false;
    float distanceToGoal = 0.0f;
    std::string closestApproachNodeName = "";
};

class DijkstraSolver3D {
public:
    DijkstraSolver3D();
    ~DijkstraSolver3D();

    // Prevent accidental copying
    DijkstraSolver3D(const DijkstraSolver3D&) = delete;
    DijkstraSolver3D& operator=(const DijkstraSolver3D&) = delete;

    // Physical Cost Calculation
    float computeEdgeCost(const Vertex3D* u, const Vertex3D* v, 
                          const Vertex3D* parentOfU, bool isEdgeBlocked) const;

    // Solve and record full step-by-step history
    void solveWithHistory(Vertex3D* start, Vertex3D* end, 
                          const std::vector<Vertex3D*>& allVertices,
                          const std::unordered_map<std::string, bool>& blockedEdgesMap);

    // Fast instant solver without snapshot overhead (<1ms)
    void solveInstant(Vertex3D* start, Vertex3D* end, 
                      const std::vector<Vertex3D*>& allVertices,
                      const std::unordered_map<std::string, bool>& blockedEdgesMap);

    // Interactive Playback Controls
    void update(float dt);
    bool stepForward();
    bool stepBackward();
    void jumpToStart();
    void jumpToEnd();
    void reset();
    void togglePlay();
    void setPlaying(bool play) { m_isPlaying = play; }
    bool isPlaying() const { return m_isPlaying; }

    // Cost Weights & Presets
    void setWeights(const CostWeights& weights);
    const CostWeights& getWeights() const { return m_weights; }
    void applyPreset(int presetIndex);

    // State Queries
    bool isFinished() const;
    bool isPathFound() const;
    size_t getCurrentStepIndex() const { return m_currentStepIndex; }
    size_t getTotalSteps() const { return m_history.size(); }
    const DijkstraSnapshot3D& getCurrentSnapshot() const;
    
    // Result Path & Telemetry
    std::vector<const Vertex3D*> getShortestPathNodes() const;
    std::vector<const Vertex3D*> getSnapshotPathNodes() const;
    const PathStats& getPathStats() const { return m_pathStats; }
    bool isPartialPath() const { return m_pathStats.isPartial; }
    float getDistanceToGoal() const { return m_pathStats.distanceToGoal; }
    bool isNeuralMode() const { return m_isNeuralMode; }

private:
    void computePathStats();

    CostWeights m_weights;
    bool m_isNeuralMode = false;
    Vertex3D* m_startNode = nullptr;
    Vertex3D* m_endNode = nullptr;
    std::vector<Vertex3D*> m_allVertices;
    std::unordered_map<std::string, Vertex3D*> m_vertexMap;

    std::vector<DijkstraSnapshot3D> m_history;
    size_t m_currentStepIndex = 0;
    bool m_isPlaying = false;
    float m_playbackTimer = 0.0f;
    float m_stepInterval = 0.04f; // 25 steps per second

    PathStats m_pathStats;
    std::vector<std::string> m_cachedPathNames;
    DijkstraSnapshot3D m_emptySnapshot;
};
