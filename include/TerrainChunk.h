#pragma once
#include <raylib.h>
#include <raymath.h>
#include <vector>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>

class TerrainHeightfield;
class PhysicsWorld;
struct BoulderInfo;

class TerrainChunk {
public:
    static constexpr float CHUNK_SIZE = 64.0f;
    static constexpr int CHUNK_RESOLUTION = 33; // 32 quad cells, 33x33 vertices = 1,089 verts

    TerrainChunk(int cx, int cz);
    ~TerrainChunk();

    TerrainChunk(const TerrainChunk&) = delete;
    TerrainChunk& operator=(const TerrainChunk&) = delete;

    void generate(const TerrainHeightfield& generator, PhysicsWorld& physics, Texture2D sharedTexture);
    void unload(PhysicsWorld& physics);
    void draw(bool wireframe = false) const;

    int getChunkX() const { return m_chunkX; }
    int getChunkZ() const { return m_chunkZ; }
    float getOriginX() const { return m_originX; }
    float getOriginZ() const { return m_originZ; }
    bool isLoaded() const { return m_isLoaded; }

    float getHeight(float worldX, float worldZ) const;
    const std::vector<float>& getHeightData() const { return m_heightData; }

private:
    int m_chunkX = 0;
    int m_chunkZ = 0;
    float m_originX = 0.0f;
    float m_originZ = 0.0f;
    float m_spacing = 2.0f;

    std::vector<float> m_heightData;
    Mesh m_mesh = {};
    Model m_model = {};
    JPH::BodyID m_physicsBodyId;
    std::vector<JPH::BodyID> m_boulderBodyIds;
    bool m_isLoaded = false;
};
