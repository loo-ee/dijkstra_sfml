#pragma once
#include <vector>
#include <string>
#include <utility>
#include <raylib.h>
#include "Vertex3D.h"
#include "TerrainHeightfield.h"

class PhysicsWorld;

struct GraphEdge3D {
    Vector3 start;
    Vector3 end;
    Color color;
    bool isBlocked = false;
};

class RoverNavGraph {
public:
    RoverNavGraph();
    ~RoverNavGraph();

    // Prevent accidental copying
    RoverNavGraph(const RoverNavGraph&) = delete;
    RoverNavGraph& operator=(const RoverNavGraph&) = delete;

    void clear();
    void generateTerrainGrid(const TerrainHeightfield& terrain, int gridCols, int gridRows, float spacing);
    void validateEdgesWithPhysics(PhysicsWorld& physics, float clearanceOffset = 0.6f);
    
    Vertex3D* getClosestNode(Vector3 worldPos);
    Vertex3D* pickNodeFromRay(Ray mouseRay, float pickRadius = 2.0f);

    void setStartNode(Vertex3D* node);
    void setEndNode(Vertex3D* node);

    void buildEdgeMeshes();
    void renderEdges() const;
    void unloadEdgeMeshes();

    const std::vector<Vertex3D*>& getVertices() const { return m_vertices; }
    const std::vector<GraphEdge3D>& getEdges() const { return m_edges; }
    Vertex3D* getStartNode() const { return m_startNode; }
    Vertex3D* getEndNode() const { return m_endNode; }

    int getGridCols() const { return m_gridCols; }
    int getGridRows() const { return m_gridRows; }
    float getSpacing() const { return m_spacing; }

private:
    std::vector<Vertex3D*> m_vertices;
    std::vector<GraphEdge3D> m_edges;
    Vertex3D* m_startNode = nullptr;
    Vertex3D* m_endNode = nullptr;

    Model m_walkableEdgesModel = {};
    Model m_blockedEdgesModel = {};
    bool m_edgesModelsLoaded = false;

    int m_gridCols = 0;
    int m_gridRows = 0;
    float m_spacing = 0.0f;
};
