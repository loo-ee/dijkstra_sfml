#pragma once
#include <vector>
#include <string>
#include <utility>
#include <unordered_map>
#include <raylib.h>
#include "Vertex3D.h"
#include "TerrainHeightfield.h"

class PhysicsWorld;

struct GraphEdge3D {
    Vector3 start;
    Vector3 end;
    Color color;
    bool isBlocked = false;
    std::string startNode;
    std::string endNode;
};

class RoverNavGraph {
public:
    RoverNavGraph();
    ~RoverNavGraph();

    // Prevent accidental copying
    RoverNavGraph(const RoverNavGraph&) = delete;
    RoverNavGraph& operator=(const RoverNavGraph&) = delete;

    void setPhysicsWorld(PhysicsWorld* physics) { m_physics = physics; }
    PhysicsWorld* getPhysicsWorld() const { return m_physics; }

    void clear();
    void generateTerrainGrid(const TerrainHeightfield& terrain, int gridCols, int gridRows, float spacing);
    void generateCenteredGrid(const TerrainHeightfield& terrain, Vector3 centerPos, int gridCols, int gridRows, float spacing);
    void generatePersistentPlanetaryGrid(const TerrainHeightfield& terrain, Vector3 centerPos, float radius = 96.0f, float spacing = 12.0f);
    void ensureCorridor(const TerrainHeightfield& terrain, Vector3 startPos, Vector3 endPos, float spacing = 12.0f);
    void validateEdgesWithPhysics(PhysicsWorld& physics, float clearanceOffset = 0.6f);
    
    Vertex3D* getClosestNode(Vector3 worldPos);
    Vertex3D* getClosestWalkableNode(Vector3 worldPos);
    bool blockEdge(const std::string& nodeA, const std::string& nodeB);
    bool blockEdgeBetweenPositions(Vector3 posA, Vector3 posB);
    bool blockEdgeNearPosition(Vector3 hazardPos, float maxDist = 8.0f);
    Vertex3D* pickNodeFromRay(Ray mouseRay, float pickRadius = 2.0f);

    void setStartNode(Vertex3D* node);
    void setEndNode(Vertex3D* node);

    void buildEdgeMeshes();
    void renderEdges() const;
    void unloadEdgeMeshes();

    const std::vector<Vertex3D*>& getVertices() const { return m_vertices; }
    const std::vector<GraphEdge3D>& getEdges() const { return m_edges; }
    const std::unordered_map<std::string, bool>& getBlockedEdgesMap() const { return m_blockedEdgesMap; }
    Vertex3D* getStartNode() const { return m_startNode; }
    Vertex3D* getEndNode() const { return m_endNode; }

    int getGridCols() const { return m_gridCols; }
    int getGridRows() const { return m_gridRows; }
    float getSpacing() const { return m_spacing; }

private:
    static inline int64_t getCellKey(int gx, int gz) {
        return (static_cast<int64_t>(gx) << 32) | (static_cast<int64_t>(gz) & 0xFFFFFFFF);
    }

    std::vector<Vertex3D*> m_vertices;
    std::unordered_map<int64_t, Vertex3D*> m_spatialNodes;
    std::vector<GraphEdge3D> m_edges;
    std::unordered_map<std::string, bool> m_blockedEdgesMap;
    Vertex3D* m_startNode = nullptr;
    Vertex3D* m_endNode = nullptr;
    PhysicsWorld* m_physics = nullptr;

    Model m_walkableEdgesModel = {};
    Model m_blockedEdgesModel = {};
    bool m_edgesModelsLoaded = false;

    int m_gridCols = 0;
    int m_gridRows = 0;
    float m_spacing = 0.0f;
};
