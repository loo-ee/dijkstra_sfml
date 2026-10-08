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
    m_currentRoverCX = 999999;
    m_currentRoverCZ = 999999;
    m_initialized = true;
}

void ChunkManager::update(Vector3 roverPos, const TerrainHeightfield& generator, PhysicsWorld& physics) {
    if (!m_initialized) {
        init(generator, physics);
    }

    int roverCX = static_cast<int>(floorf(roverPos.x / TerrainChunk::CHUNK_SIZE));
    int roverCZ = static_cast<int>(floorf(roverPos.z / TerrainChunk::CHUNK_SIZE));

    // CRITICAL PERFORMANCE GUARD:
    // If vehicle remains within the same chunk cell, exit immediately!
    // Takes < 1 microsecond. Completely stops per-frame GPU buffer thrashing & stutter!
    if (roverCX == m_currentRoverCX && roverCZ == m_currentRoverCZ) {
        return;
    }

    m_currentRoverCX = roverCX;
    m_currentRoverCZ = roverCZ;

    // 1. Stream in 7x7 chunks around vehicle (49 chunks = 448m x 448m)
    for (int rz = -m_radius; rz <= m_radius; ++rz) {
        for (int rx = -m_radius; rx <= m_radius; ++rx) {
            int targetCX = roverCX + rx;
            int targetCZ = roverCZ + rz;
            int64_t key = getChunkKey(targetCX, targetCZ);

            if (m_activeChunks.find(key) == m_activeChunks.end()) {
                auto chunk = std::make_unique<TerrainChunk>(targetCX, targetCZ);
                chunk->generate(generator, m_sharedTexture);
                m_activeChunks[key] = std::move(chunk);
            }
        }
    }

    // 2. Dynamic Physics Management: Only maintain Jolt physics on 3x3 chunks immediately around the rover
    for (auto& pair : m_activeChunks) {
        if (!pair.second || !pair.second->isLoaded()) continue;
        int cx = pair.second->getChunkX();
        int cz = pair.second->getChunkZ();
        bool nearRover = (std::abs(cx - roverCX) <= 1 && std::abs(cz - roverCZ) <= 1);
        if (nearRover) {
            pair.second->ensurePhysics(physics, generator);
        } else if (pair.second->hasPhysics()) {
            pair.second->removePhysics(physics);
        }
    }

    // 3. Unload Chunks outside Active Radius + 1 (only when rover drives into a new chunk cell!)
    int maxDist = m_radius + 1;
    for (auto it = m_activeChunks.begin(); it != m_activeChunks.end(); ) {
        int targetCX = it->second->getChunkX();
        int targetCZ = it->second->getChunkZ();

        if (std::abs(targetCX - roverCX) > maxDist || std::abs(targetCZ - roverCZ) > maxDist) {
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
    m_currentRoverCX = 999999;
    m_currentRoverCZ = 999999;
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
