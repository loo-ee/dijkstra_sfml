#include "TerrainHeightfield.h"
#include <algorithm>
#include <cstdlib>

// Self-contained 2D Perlin Noise Implementation
namespace {
    const int P[512] = {
        151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,
        8,99,37,240,21,10,23,190,6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,
        35,11,32,57,177,33,88,237,149,56,87,174,20,125,136,171,168,68,175,74,165,71,
        134,139,48,27,166,77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,
        55,46,245,40,244,102,143,54,65,25,63,161,1,216,80,73,209,76,132,187,208,89,
        18,169,200,196,135,130,116,188,159,86,164,100,109,198,173,186,3,64,52,217,226,
        250,124,123,5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,
        189,28,42,223,183,170,213,119,248,152,2,44,154,163,70,221,153,101,155,167,43,
        172,9,129,22,39,253,19,98,108,110,79,113,224,232,178,185,112,104,218,246,97,
        228,251,34,242,193,238,210,144,12,191,179,162,241,81,51,145,235,249,14,239,
        107,49,192,214,31,181,199,106,157,184,84,204,176,115,121,50,45,127,4,150,254,
        138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180,
        // Repeated table for overflow
        151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,
        8,99,37,240,21,10,23,190,6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,
        35,11,32,57,177,33,88,237,149,56,87,174,20,125,136,171,168,68,175,74,165,71,
        134,139,48,27,166,77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,
        55,46,245,40,244,102,143,54,65,25,63,161,1,216,80,73,209,76,132,187,208,89,
        18,169,200,196,135,130,116,188,159,86,164,100,109,198,173,186,3,64,52,217,226,
        250,124,123,5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,
        189,28,42,223,183,170,213,119,248,152,2,44,154,163,70,221,153,101,155,167,43,
        172,9,129,22,39,253,19,98,108,110,79,113,224,232,178,185,112,104,218,246,97,
        228,251,34,242,193,238,210,144,12,191,179,162,241,81,51,145,235,249,14,239,
        107,49,192,214,31,181,199,106,157,184,84,204,176,115,121,50,45,127,4,150,254,
        138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180
    };

    inline float fade(float t) {
        return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
    }

    inline float grad(int hash, float x, float y) {
        int h = hash & 7;
        float u = (h < 4) ? x : y;
        float v = (h < 4) ? y : x;
        return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
    }
}

TerrainHeightfield::TerrainHeightfield(int resolution, float size)
    : m_resolution(resolution), m_size(size), m_isLoaded(false)
{
    m_spacing = m_size / (m_resolution - 1);
    m_heightData.resize(m_resolution * m_resolution, 0.0f);

    // Setup distinct Martian craters
    // Primary crater: prominent impact basin with raised rim wall
    m_craters.push_back({
        Vector2{ 25.0f, -15.0f }, // center
        32.0f,                    // radius R
        9.0f,                     // depth d
        4.0f,                     // rimHeight
        5.5f                      // rimWidth sigma
    });

    // Secondary crater: smaller satellite crater
    m_craters.push_back({
        Vector2{ -45.0f, 35.0f },
        20.0f,
        5.5f,
        2.8f,
        4.0f
    });

    // Third shallow crater
    m_craters.push_back({
        Vector2{ -20.0f, -40.0f },
        16.0f,
        3.5f,
        1.8f,
        3.2f
    });
}

TerrainHeightfield::~TerrainHeightfield() {
    unload();
}

void TerrainHeightfield::unload() {
    if (m_isLoaded) {
        UnloadModel(m_model);
        m_isLoaded = false;
    }
}

float TerrainHeightfield::samplePerlin(float x, float z) const {
    int X = static_cast<int>(floorf(x)) & 255;
    int Y = static_cast<int>(floorf(z)) & 255;

    float xf = x - floorf(x);
    float yf = z - floorf(z);

    float u = fade(xf);
    float v = fade(yf);

    int A = P[X] + Y;
    int B = P[X + 1] + Y;

    float g00 = grad(P[A], xf, yf);
    float g10 = grad(P[B], xf - 1.0f, yf);
    float g01 = grad(P[A + 1], xf, yf - 1.0f);
    float g11 = grad(P[B + 1], xf - 1.0f, yf - 1.0f);

    float x1 = Lerp(g00, g10, u);
    float x2 = Lerp(g01, g11, u);

    return Lerp(x1, x2, v);
}

float TerrainHeightfield::evaluateRawHeight(float x, float z) const {
    // 1. Fractal Brownian Motion (fBm)
    // h(x, z) = sum_{i=0}^{N-1} A * gamma^i * Noise(f * 2^i * x, f * 2^i * z)
    float baseAmp = 14.0f;
    float baseFreq = 0.012f;
    float persistence = 0.5f;
    float height = 0.0f;

    for (int i = 0; i < 4; ++i) {
        float freq = baseFreq * (1 << i);
        float amp = baseAmp * powf(persistence, static_cast<float>(i));
        height += amp * samplePerlin(x * freq, z * freq);
    }

    // 2. Martian Impact Crater Displacements
    for (const auto& c : m_craters) {
        float dx = x - c.center.x;
        float dz = z - c.center.y;
        float r = sqrtf(dx * dx + dz * dz);

        if (r <= 1.5f * c.radius) {
            float normR = r / c.radius;
            // Bowl depression (creates deep crater floor)
            float bowl = -c.depth * std::max(0.0f, 1.0f - normR * normR);
            // Raised rim wall typical of Martian impact craters
            float rimDiff = r - c.radius;
            float rim = c.rimHeight * expf(-(rimDiff * rimDiff) / (2.0f * c.rimWidth * c.rimWidth));

            height += (bowl + rim);
        }
    }

    return height;
}

void TerrainHeightfield::generate() {
    unload();

    const float halfSize = m_size * 0.5f;

    // 1. Precalculate 2D Height Grid
    for (int gz = 0; gz < m_resolution; ++gz) {
        for (int gx = 0; gx < m_resolution; ++gx) {
            float worldX = -halfSize + gx * m_spacing;
            float worldZ = -halfSize + gz * m_spacing;
            m_heightData[gz * m_resolution + gx] = evaluateRawHeight(worldX, worldZ);
        }
    }

    // 2. Construct Raylib Mesh with Normal Vectors and Slope-based Vertex Shading
    int numVertices = m_resolution * m_resolution;
    int numQuads = (m_resolution - 1) * (m_resolution - 1);
    int numTriangles = numQuads * 2;

    Mesh mesh = {};
    mesh.vertexCount = numVertices;
    mesh.triangleCount = numTriangles;

    mesh.vertices = static_cast<float*>(MemAlloc(numVertices * 3 * sizeof(float)));
    mesh.normals = static_cast<float*>(MemAlloc(numVertices * 3 * sizeof(float)));
    mesh.texcoords = static_cast<float*>(MemAlloc(numVertices * 2 * sizeof(float)));
    mesh.colors = static_cast<unsigned char*>(MemAlloc(numVertices * 4 * sizeof(unsigned char)));
    mesh.indices = static_cast<unsigned short*>(MemAlloc(numTriangles * 3 * sizeof(unsigned short)));

    // Populate Vertices, Normals, and Colors
    for (int gz = 0; gz < m_resolution; ++gz) {
        for (int gx = 0; gx < m_resolution; ++gx) {
            int vIdx = gz * m_resolution + gx;
            float worldX = -halfSize + gx * m_spacing;
            float worldZ = -halfSize + gz * m_spacing;
            float worldY = m_heightData[vIdx];

            mesh.vertices[vIdx * 3 + 0] = worldX;
            mesh.vertices[vIdx * 3 + 1] = worldY;
            mesh.vertices[vIdx * 3 + 2] = worldZ;

            // Surface normal & slope angle via central differences
            Vector3 norm = getNormal(worldX, worldZ);
            mesh.normals[vIdx * 3 + 0] = norm.x;
            mesh.normals[vIdx * 3 + 1] = norm.y;
            mesh.normals[vIdx * 3 + 2] = norm.z;

            mesh.texcoords[vIdx * 2 + 0] = static_cast<float>(gx) / (m_resolution - 1);
            mesh.texcoords[vIdx * 2 + 1] = static_cast<float>(gz) / (m_resolution - 1);

            float slopeRad = acosf(Clamp(norm.y, -1.0f, 1.0f));
            Color vertexColor = getSlopeColor(slopeRad, norm);

            mesh.colors[vIdx * 4 + 0] = vertexColor.r;
            mesh.colors[vIdx * 4 + 1] = vertexColor.g;
            mesh.colors[vIdx * 4 + 2] = vertexColor.b;
            mesh.colors[vIdx * 4 + 3] = vertexColor.a;
        }
    }

    // Populate Triangles (CCW winding)
    int tIdx = 0;
    for (int gz = 0; gz < m_resolution - 1; ++gz) {
        for (int gx = 0; gx < m_resolution - 1; ++gx) {
            unsigned short topLeft = static_cast<unsigned short>(gz * m_resolution + gx);
            unsigned short topRight = static_cast<unsigned short>(topLeft + 1);
            unsigned short bottomLeft = static_cast<unsigned short>((gz + 1) * m_resolution + gx);
            unsigned short bottomRight = static_cast<unsigned short>(bottomLeft + 1);

            // Triangle 1
            mesh.indices[tIdx++] = topLeft;
            mesh.indices[tIdx++] = bottomLeft;
            mesh.indices[tIdx++] = topRight;

            // Triangle 2
            mesh.indices[tIdx++] = topRight;
            mesh.indices[tIdx++] = bottomLeft;
            mesh.indices[tIdx++] = bottomRight;
        }
    }

    // Upload Mesh to GPU
    UploadMesh(&mesh, false);
    m_model = LoadModelFromMesh(mesh);
    m_isLoaded = true;
}

float TerrainHeightfield::getHeight(float x, float z) const {
    const float halfSize = m_size * 0.5f;
    float gx = (x + halfSize) / m_spacing;
    float gz = (z + halfSize) / m_spacing;

    if (gx < 0.0f || gx >= m_resolution - 1 || gz < 0.0f || gz >= m_resolution - 1) {
        return evaluateRawHeight(x, z);
    }

    int x0 = static_cast<int>(floorf(gx));
    int z0 = static_cast<int>(floorf(gz));
    int x1 = std::min(x0 + 1, m_resolution - 1);
    int z1 = std::min(z0 + 1, m_resolution - 1);

    float fx = gx - x0;
    float fz = gz - z0;

    // Bilinear interpolation across grid quad
    float h00 = m_heightData[z0 * m_resolution + x0];
    float h10 = m_heightData[z0 * m_resolution + x1];
    float h01 = m_heightData[z1 * m_resolution + x0];
    float h11 = m_heightData[z1 * m_resolution + x1];

    float h0 = Lerp(h00, h10, fx);
    float h1 = Lerp(h01, h11, fx);

    return Lerp(h0, h1, fz);
}

Vector3 TerrainHeightfield::getNormal(float x, float z) const {
    // Central finite differences: delta = 0.5m
    const float delta = 0.5f;
    float hL = getHeight(x - delta, z);
    float hR = getHeight(x + delta, z);
    float hD = getHeight(x, z - delta);
    float hU = getHeight(x, z + delta);

    float dhdx = (hR - hL) / (2.0f * delta);
    float dhdz = (hU - hD) / (2.0f * delta);

    // Normal vector n = (-dh/dx, 1, -dh/dz) / ||...||
    Vector3 n = { -dhdx, 1.0f, -dhdz };
    return Vector3Normalize(n);
}

float TerrainHeightfield::getSlopeAngleRad(float x, float z) const {
    Vector3 norm = getNormal(x, z);
    return acosf(Clamp(norm.y, -1.0f, 1.0f));
}

Color TerrainHeightfield::getSlopeColor(float slopeRad, Vector3 normal) const {
    float slopeDeg = slopeRad * RAD2DEG;

    // Section 2.3 Color Specifications:
    // Slope < 15 deg: Reddish Martian dust (195, 92, 60)
    // Slope 15 - 30 deg: Dark exposed bedrock (110, 68, 55)
    // Slope > 30 deg: Charcoal basalt (60, 50, 48)
    Color base;
    if (slopeDeg < 15.0f) {
        base = Color{ 195, 92, 60, 255 };
    } else if (slopeDeg <= 30.0f) {
        float t = (slopeDeg - 15.0f) / 15.0f;
        base = Color{
            static_cast<unsigned char>(Lerp(195, 110, t)),
            static_cast<unsigned char>(Lerp(92, 68, t)),
            static_cast<unsigned char>(Lerp(60, 55, t)),
            255
        };
    } else {
        float t = std::min(1.0f, (slopeDeg - 30.0f) / 15.0f);
        base = Color{
            static_cast<unsigned char>(Lerp(110, 60, t)),
            static_cast<unsigned char>(Lerp(68, 50, t)),
            static_cast<unsigned char>(Lerp(55, 48, t)),
            255
        };
    }

    // Directional solar lighting for crisp topography relief
    Vector3 sunDir = Vector3Normalize(Vector3{ 0.4f, 0.85f, 0.35f });
    float diffuse = std::max(0.0f, Vector3DotProduct(normal, sunDir));
    float lightFactor = 0.35f + 0.65f * diffuse; // Ambient + Diffuse

    return Color{
        static_cast<unsigned char>(Clamp(base.r * lightFactor, 0.0f, 255.0f)),
        static_cast<unsigned char>(Clamp(base.g * lightFactor, 0.0f, 255.0f)),
        static_cast<unsigned char>(Clamp(base.b * lightFactor, 0.0f, 255.0f)),
        255
    };
}
