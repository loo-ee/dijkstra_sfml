#include "ChunkManager.h"
#include "TerrainHeightfield.h"
#include "PhysicsWorld.h"
#include <iostream>
#include <cmath>

ChunkManager::ChunkManager() {
}

ChunkManager::~ChunkManager() {
}

void ChunkManager::init(const TerrainHeightfield& generator, PhysicsWorld& physics) {
    clear(physics);
    m_sharedTexture = generator.getTexture();
    m_currentCenterCX = 999999;
    m_currentCenterCZ = 999999;
    m_initialized = true;
}

void ChunkManager::update(Vector3 roverPos, const TerrainHeightfield& generator, PhysicsWorld& physics) {
    if (!m_initialized) {
        init(generator, physics);
    }

    int cx = static_cast<int>(floorf(roverPos.x / TerrainChunk::CHUNK_SIZE));
    int cz = static_cast<int>(floorf(roverPos.z / TerrainChunk::CHUNK_SIZE));

    if (cx == m_currentCenterCX && cz == m_currentCenterCZ) {
        return; // Still in the same chunk, no streaming required
    }

    m_currentCenterCX = cx;
    m_currentCenterCZ = cz;

    // 1. Stream in Chunks within Active Radius
    for (int rz = -m_radius; rz <= m_radius; ++rz) {
        for (int rx = -m_radius; rx <= m_radius; ++rx) {
            int targetCX = cx + rx;
            int targetCZ = cz + rz;
            int64_t key = getChunkKey(targetCX, targetCZ);

            if (m_activeChunks.find(key) == m_activeChunks.end()) {
                auto chunk = std::make_unique<TerrainChunk>(targetCX, targetCZ);
                chunk->generate(generator, physics, m_sharedTexture);
                m_activeChunks[key] = std::move(chunk);
            }
        }
    }

    // 2. Stream out Chunks outside Active Radius
    for (auto it = m_activeChunks.begin(); it != m_activeChunks.end(); ) {
        int chX = it->second->getChunkX();
        int chZ = it->second->getChunkZ();

        if (std::abs(chX - cx) > m_radius || std::abs(chZ - cz) > m_radius) {
            it->second->unload(physics);
            it = m_activeChunks.erase(it);
        } else {
            ++it;
        }
    }
}

void ChunkManager::draw(bool wireframe) const {
    if (!m_initialized) return;

    for (const auto& pair : m_activeChunks) {
        if (pair.second && pair.second->isLoaded()) {
            pair.second->draw(wireframe);
        }
    }
}

void ChunkManager::clear(PhysicsWorld& physics) {
    for (auto& pair : m_activeChunks) {
        if (pair.second) {
            pair.second->unload(physics);
        }
    }
    m_activeChunks.clear();
    m_currentCenterCX = 999999;
    m_currentCenterCZ = 999999;
    m_initialized = false;
}

float ChunkManager::getHeight(float worldX, float worldZ, const TerrainHeightfield& generator) const {
    int cx = static_cast<int>(floorf(worldX / TerrainChunk::CHUNK_SIZE));
    int cz = static_cast<int>(floorf(worldZ / TerrainChunk::CHUNK_SIZE));
    int64_t key = getChunkKey(cx, cz);

    auto it = m_activeChunks.find(key);
    if (it != m_activeChunks.end() && it->second && it->second->isLoaded()) {
        return it->second->getHeight(worldX, worldZ);
    }

    return generator.getHeight(worldX, worldZ);
}

Vector3 ChunkManager::getNormal(float worldX, float worldZ, const TerrainHeightfield& generator) const {
    const float delta = 0.5f;
    float hL = getHeight(worldX - delta, worldZ, generator);
    float hR = getHeight(worldX + delta, worldZ, generator);
    float hD = getHeight(worldX, worldZ - delta, generator);
    float hU = getHeight(worldX, worldZ + delta, generator);

    float dhdx = (hR - hL) / (2.0f * delta);
    float dhdz = (hU - hD) / (2.0f * delta);

    Vector3 n = { -dhdx, 1.0f, -dhdz };
    return Vector3Normalize(n);
}
