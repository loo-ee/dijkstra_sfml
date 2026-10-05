#include "TerrainChunk.h"
#include "TerrainHeightfield.h"
#include "PhysicsWorld.h"
#include <rlgl.h>
#include <algorithm>
#include <cmath>

namespace {
    // Deterministic spatial hash for procedural features (craters/boulders) per chunk
    uint32_t chunkHash(int cx, int cz, uint32_t seed) {
        uint32_t h = seed ^ (static_cast<uint32_t>(cx) * 73856093u) ^ (static_cast<uint32_t>(cz) * 19349663u);
        h = (h ^ (h >> 16)) * 0x85ebca6bu;
        h = (h ^ (h >> 13)) * 0xc2b2ae35u;
        return h ^ (h >> 16);
    }

    float hashToFloat(uint32_t h) {
        return static_cast<float>(h & 0xFFFF) / 65535.0f;
    }
}

TerrainChunk::TerrainChunk(int cx, int cz)
    : m_chunkX(cx), m_chunkZ(cz)
{
    m_originX = m_chunkX * CHUNK_SIZE;
    m_originZ = m_chunkZ * CHUNK_SIZE;
    m_spacing = CHUNK_SIZE / (CHUNK_RESOLUTION - 1);
    m_heightData.resize(CHUNK_RESOLUTION * CHUNK_RESOLUTION, 0.0f);
    m_physicsBodyId = JPH::BodyID();
}

TerrainChunk::~TerrainChunk() {
}

void TerrainChunk::generate(const TerrainHeightfield& generator, PhysicsWorld& physics, Texture2D sharedTexture) {
    if (m_isLoaded) return;

    m_originX = m_chunkX * CHUNK_SIZE;
    m_originZ = m_chunkZ * CHUNK_SIZE;
    m_spacing = CHUNK_SIZE / (CHUNK_RESOLUTION - 1);

    int numVertices = CHUNK_RESOLUTION * CHUNK_RESOLUTION;
    int numQuads = (CHUNK_RESOLUTION - 1) * (CHUNK_RESOLUTION - 1);
    int numTriangles = numQuads * 2;

    m_mesh = {};
    m_mesh.vertexCount = numVertices;
    m_mesh.triangleCount = numTriangles;

    m_mesh.vertices = static_cast<float*>(MemAlloc(numVertices * 3 * sizeof(float)));
    m_mesh.normals = static_cast<float*>(MemAlloc(numVertices * 3 * sizeof(float)));
    m_mesh.texcoords = static_cast<float*>(MemAlloc(numVertices * 2 * sizeof(float)));
    m_mesh.colors = static_cast<unsigned char*>(MemAlloc(numVertices * 4 * sizeof(unsigned char)));
    m_mesh.indices = static_cast<unsigned short*>(MemAlloc(numTriangles * 3 * sizeof(unsigned short)));

    // 1. Populate Height Data and Vertex Attributes
    for (int gz = 0; gz < CHUNK_RESOLUTION; ++gz) {
        for (int gx = 0; gx < CHUNK_RESOLUTION; ++gx) {
            int vIdx = gz * CHUNK_RESOLUTION + gx;
            float worldX = m_originX + gx * m_spacing;
            float worldZ = m_originZ + gz * m_spacing;
            float worldY = generator.getHeight(worldX, worldZ);

            m_heightData[vIdx] = worldY;

            m_mesh.vertices[vIdx * 3 + 0] = worldX;
            m_mesh.vertices[vIdx * 3 + 1] = worldY;
            m_mesh.vertices[vIdx * 3 + 2] = worldZ;

            Vector3 norm = generator.getNormal(worldX, worldZ);
            m_mesh.normals[vIdx * 3 + 0] = norm.x;
            m_mesh.normals[vIdx * 3 + 1] = norm.y;
            m_mesh.normals[vIdx * 3 + 2] = norm.z;

            // Continuous world-space texture mapping (no seams across chunk edges)
            m_mesh.texcoords[vIdx * 2 + 0] = worldX / 10.0f;
            m_mesh.texcoords[vIdx * 2 + 1] = worldZ / 10.0f;

            float slopeRad = acosf(Clamp(norm.y, -1.0f, 1.0f));
            Color vertexColor = generator.getSlopeColor(slopeRad, norm);
            m_mesh.colors[vIdx * 4 + 0] = vertexColor.r;
            m_mesh.colors[vIdx * 4 + 1] = vertexColor.g;
            m_mesh.colors[vIdx * 4 + 2] = vertexColor.b;
            m_mesh.colors[vIdx * 4 + 3] = vertexColor.a;
        }
    }

    // 2. Populate Triangles (CCW Winding)
    int tIdx = 0;
    for (int gz = 0; gz < CHUNK_RESOLUTION - 1; ++gz) {
        for (int gx = 0; gx < CHUNK_RESOLUTION - 1; ++gx) {
            unsigned short topLeft = static_cast<unsigned short>(gz * CHUNK_RESOLUTION + gx);
            unsigned short topRight = static_cast<unsigned short>(topLeft + 1);
            unsigned short bottomLeft = static_cast<unsigned short>((gz + 1) * CHUNK_RESOLUTION + gx);
            unsigned short bottomRight = static_cast<unsigned short>(bottomLeft + 1);

            // Triangle 1
            m_mesh.indices[tIdx++] = topLeft;
            m_mesh.indices[tIdx++] = bottomLeft;
            m_mesh.indices[tIdx++] = topRight;

            // Triangle 2
            m_mesh.indices[tIdx++] = bottomLeft;
            m_mesh.indices[tIdx++] = bottomRight;
            m_mesh.indices[tIdx++] = topRight;
        }
    }

    // 3. Upload Mesh & Setup Raylib Model
    UploadMesh(&m_mesh, false);
    m_model = LoadModelFromMesh(m_mesh);
    m_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = sharedTexture;

    // 4. Jolt Physics HeightField Collider
    m_physicsBodyId = physics.createChunkHeightField(
        m_heightData.data(),
        CHUNK_RESOLUTION,
        m_spacing,
        Vector3{ m_originX, 0.0f, m_originZ }
    );

    // 5. Deterministic Procedural Boulder Placement
    // Each chunk deterministically generates 1 to 3 boulders based on coordinate hash
    uint32_t seed = chunkHash(m_chunkX, m_chunkZ, 1337);
    int numBoulders = 1 + (seed % 3);

    for (int b = 0; b < numBoulders; ++b) {
        uint32_t bh = chunkHash(m_chunkX, m_chunkZ, seed + b * 997);
        float bx = m_originX + 8.0f + hashToFloat(bh) * (CHUNK_SIZE - 16.0f);
        uint32_t bzHash = chunkHash(m_chunkX, m_chunkZ, bh + 12345);
        float bz = m_originZ + 8.0f + hashToFloat(bzHash) * (CHUNK_SIZE - 16.0f);
        float by = generator.getHeight(bx, bz);

        float radius = 1.8f + hashToFloat(bh ^ bzHash) * 1.6f;
        float slope = generator.getSlopeAngleRad(bx, bz);

        // Only spawn boulders on reasonably flat or rolling ground (< 20 deg)
        if (slope < 20.0f * DEG2RAD) {
            Vector3 bPos = { bx, by + radius * 0.70f, bz };
            physics.spawnBoulder(bPos, radius);
        }
    }

    m_isLoaded = true;
}

void TerrainChunk::unload(PhysicsWorld& physics) {
    if (!m_isLoaded) return;

    if (!m_physicsBodyId.IsInvalid()) {
        physics.removeChunkHeightField(m_physicsBodyId);
        m_physicsBodyId = JPH::BodyID();
    }

    UnloadModel(m_model);
    m_isLoaded = false;
}

void TerrainChunk::draw(bool wireframe) const {
    if (!m_isLoaded) return;

    rlDisableBackfaceCulling();
    DrawModel(m_model, Vector3{ 0.0f, 0.0f, 0.0f }, 1.0f, WHITE);
    rlEnableBackfaceCulling();

    if (wireframe) {
        DrawModelWires(m_model, Vector3{ 0.0f, 0.0f, 0.0f }, 1.0f, ColorAlpha(BLACK, 0.25f));
    }
}

float TerrainChunk::getHeight(float worldX, float worldZ) const {
    float gx = (worldX - m_originX) / m_spacing;
    float gz = (worldZ - m_originZ) / m_spacing;

    if (gx < 0.0f || gx >= CHUNK_RESOLUTION - 1 || gz < 0.0f || gz >= CHUNK_RESOLUTION - 1) {
        return 0.0f;
    }

    int x0 = static_cast<int>(floorf(gx));
    int z0 = static_cast<int>(floorf(gz));
    int x1 = std::min(x0 + 1, CHUNK_RESOLUTION - 1);
    int z1 = std::min(z0 + 1, CHUNK_RESOLUTION - 1);

    float fx = gx - x0;
    float fz = gz - z0;

    float h00 = m_heightData[z0 * CHUNK_RESOLUTION + x0];
    float h10 = m_heightData[z0 * CHUNK_RESOLUTION + x1];
    float h01 = m_heightData[z1 * CHUNK_RESOLUTION + x0];
    float h11 = m_heightData[z1 * CHUNK_RESOLUTION + x1];

    float h0 = Lerp(h00, h10, fx);
    float h1 = Lerp(h01, h11, fx);

    return Lerp(h0, h1, fz);
}
