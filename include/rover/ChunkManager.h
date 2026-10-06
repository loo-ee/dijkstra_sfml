#pragma once
#include <raylib.h>
#include <raymath.h>
#include <unordered_map>
#include <memory>
#include <vector>

#include "TerrainChunk.h"

class TerrainHeightfield;
class PhysicsWorld;

class ChunkManager {
public:
    ChunkManager();
    ~ChunkManager();

    ChunkManager(const ChunkManager&) = delete;
    ChunkManager& operator=(const ChunkManager&) = delete;

    void init(const TerrainHeightfield& generator, PhysicsWorld& physics);
    void update(Vector3 roverPos, const TerrainHeightfield& generator, PhysicsWorld& physics);
    void draw(bool wireframe = false) const;
    void clear(PhysicsWorld& physics);

    float getHeight(float worldX, float worldZ, const TerrainHeightfield& generator) const;
    Vector3 getNormal(float worldX, float worldZ, const TerrainHeightfield& generator) const;

    int getActiveChunkCount() const { return static_cast<int>(m_activeChunks.size()); }
    int getCurrentCenterCX() const { return m_currentCenterCX; }
    int getCurrentCenterCZ() const { return m_currentCenterCZ; }

    bool isInitialized() const { return m_initialized; }

private:
    static inline int64_t getChunkKey(int cx, int cz) {
        return (static_cast<int64_t>(cx) << 32) | (static_cast<int64_t>(cz) & 0xFFFFFFFF);
    }

    std::unordered_map<int64_t, std::unique_ptr<TerrainChunk>> m_activeChunks;
    Texture2D m_sharedTexture = {};
    int m_currentCenterCX = 999999;
    int m_currentCenterCZ = 999999;
    int m_radius = 3; // 7x7 active grid (49 chunks = 448m x 448m coverage)
    bool m_initialized = false;
};
