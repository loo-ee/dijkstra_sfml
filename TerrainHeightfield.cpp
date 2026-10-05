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
    : m_resolution(resolution), m_size(size), m_preset(TerrainPreset::OLYMPUS_CRATER), m_isLoaded(false)
{
    m_spacing = m_size / (m_resolution - 1);
    m_heightData.resize(m_resolution * m_resolution, 0.0f);
    configureCratersForPreset();
}

TerrainHeightfield::~TerrainHeightfield() {
    unload();
}

void TerrainHeightfield::setPreset(TerrainPreset preset) {
    m_preset = preset;
    configureCratersForPreset();
}

const char* TerrainHeightfield::getPresetName() const {
    switch (m_preset) {
        case TerrainPreset::OLYMPUS_CRATER:   return "The Olympus Crater [Mars]";
        case TerrainPreset::SCREE_SLOPE:      return "Lunar Scree Slope [Moon]";
        case TerrainPreset::BOULDER_SLALOM:   return "Martian Canyon Slalom [Mars]";
        case TerrainPreset::ACIDALIA_PLANITIA: return "Terrestrial Proving Ground [Earth]";
        default: return "Unknown";
    }
}

float TerrainHeightfield::getPresetGravity() const {
    switch (m_preset) {
        case TerrainPreset::OLYMPUS_CRATER:   return -3.71f; // Martian Gravity
        case TerrainPreset::SCREE_SLOPE:      return -1.62f; // Lunar Gravity
        case TerrainPreset::BOULDER_SLALOM:   return -3.71f; // Martian Gravity
        case TerrainPreset::ACIDALIA_PLANITIA: return -9.81f; // Earth Gravity
        default: return -3.71f;
    }
}

const char* TerrainHeightfield::getEnvironmentName() const {
    switch (m_preset) {
        case TerrainPreset::OLYMPUS_CRATER:   return "Mars (g = 3.71 m/s²)";
        case TerrainPreset::SCREE_SLOPE:      return "Moon (g = 1.62 m/s²)";
        case TerrainPreset::BOULDER_SLALOM:   return "Mars (g = 3.71 m/s²)";
        case TerrainPreset::ACIDALIA_PLANITIA: return "Earth (g = 9.81 m/s²)";
        default: return "Mars (g = 3.71 m/s²)";
    }
}

void TerrainHeightfield::cyclePreset() {
    int next = (static_cast<int>(m_preset) + 1) % 4;
    setPreset(static_cast<TerrainPreset>(next));
}

void TerrainHeightfield::configureCratersForPreset() {
    m_craters.clear();

    if (m_preset == TerrainPreset::OLYMPUS_CRATER) {
        // Primary crater: prominent impact basin with raised rim wall
        m_craters.push_back({
            Vector2{ 20.0f, -10.0f },
            32.0f,
            9.0f,
            4.0f,
            5.5f
        });
        m_craters.push_back({
            Vector2{ -45.0f, 35.0f },
            20.0f,
            5.5f,
            2.8f,
            4.0f
        });
        m_craters.push_back({
            Vector2{ -20.0f, -40.0f },
            16.0f,
            3.5f,
            1.8f,
            3.2f
        });
    } else if (m_preset == TerrainPreset::SCREE_SLOPE) {
        m_craters.push_back({
            Vector2{ -35.0f, -25.0f },
            18.0f,
            4.5f,
            2.0f,
            3.5f
        });
    } else if (m_preset == TerrainPreset::BOULDER_SLALOM) {
        m_craters.push_back({
            Vector2{ 50.0f, 40.0f },
            18.0f,
            5.0f,
            2.5f,
            3.5f
        });
    }
    // ACIDALIA_PLANITIA has 0 craters (pure rolling dune plains)
}

void TerrainHeightfield::unload() {
    if (m_isLoaded) {
        UnloadModel(m_model);
        if (m_texture.id != 0) {
            UnloadTexture(m_texture);
            m_texture = {};
        }
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
    float height = 0.0f;

    if (m_preset == TerrainPreset::OLYMPUS_CRATER) {
        // Fractal Brownian Motion (fBm)
        float baseAmp = 12.0f;
        float baseFreq = 0.012f;
        float persistence = 0.5f;

        for (int i = 0; i < 4; ++i) {
            float freq = baseFreq * (1 << i);
            float amp = baseAmp * powf(persistence, static_cast<float>(i));
            height += amp * samplePerlin(x * freq, z * freq);
        }

        // Martian Impact Crater Displacements
        for (const auto& c : m_craters) {
            float dx = x - c.center.x;
            float dz = z - c.center.y;
            float r = sqrtf(dx * dx + dz * dz);

            if (r <= 1.5f * c.radius) {
                float normR = r / c.radius;
                float bowl = -c.depth * std::max(0.0f, 1.0f - normR * normR);
                float rimDiff = r - c.radius;
                float rim = c.rimHeight * expf(-(rimDiff * rimDiff) / (2.0f * c.rimWidth * c.rimWidth));
                height += (bowl + rim);
            }
        }
    } else if (m_preset == TerrainPreset::SCREE_SLOPE) {
        // High-grade mountain slope with stepped terracing and loose scree
        height += (x * 0.15f + z * 0.08f);
        height += sinf(x * 0.055f) * 3.8f + cosf(z * 0.045f) * 2.2f;
        height += samplePerlin(x * 0.035f, z * 0.035f) * 3.5f;
        height += samplePerlin(x * 0.09f, z * 0.09f) * 1.0f;

        for (const auto& c : m_craters) {
            float dx = x - c.center.x;
            float dz = z - c.center.y;
            float r = sqrtf(dx * dx + dz * dz);
            if (r <= 1.5f * c.radius) {
                float normR = r / c.radius;
                float bowl = -c.depth * std::max(0.0f, 1.0f - normR * normR);
                float rimDiff = r - c.radius;
                float rim = c.rimHeight * expf(-(rimDiff * rimDiff) / (2.0f * c.rimWidth * c.rimWidth));
                height += (bowl + rim);
            }
        }
    } else if (m_preset == TerrainPreset::BOULDER_SLALOM) {
        // Winding Martian Canyon Corridor with sheer wall flanks
        float canyonCenter = sinf(z * 0.032f) * 32.0f;
        float distFromCanyon = fabsf(x - canyonCenter);

        float canyonDepression = -8.0f * (1.0f - Clamp(distFromCanyon / 34.0f, 0.0f, 1.0f));
        float wallFactor = Clamp((distFromCanyon - 16.0f) / 28.0f, 0.0f, 1.0f);
        float canyonWalls = wallFactor * wallFactor * 16.0f;

        height += (canyonDepression + canyonWalls);
        height += samplePerlin(x * 0.018f, z * 0.018f) * 2.8f;
    } else { // ACIDALIA_PLANITIA
        // Smooth flowing low-gradient sand dunes (fast rover cruising)
        float baseFreq = 0.007f;
        height += samplePerlin(x * baseFreq, z * baseFreq) * 5.2f;
        height += sinf((x * 0.7f + z * 0.3f) * 0.04f) * 1.5f;
        height += samplePerlin(x * 0.02f, z * 0.02f) * 0.8f;
    }

    // Planetary Spherical Globe Curvature (Mars Planetary Body Radius R ~ 1400m)
    // Curvature equation: y = -(R - sqrt(max(0, R^2 - (x^2 + z^2))))
    const float planetRadius = 1400.0f;
    float distSq = x * x + z * z;
    float globeDrop = 0.0f;
    if (distSq < planetRadius * planetRadius) {
        globeDrop = planetRadius - sqrtf(planetRadius * planetRadius - distSq);
    } else {
        globeDrop = planetRadius + (sqrtf(distSq) - planetRadius) * 1.5f;
    }
    height -= globeDrop;

    return height;
}

void TerrainHeightfield::generate() {
    unload();

    const float halfPhys = m_size * 0.5f;

    // 1. Precalculate 2D Height Grid for Jolt Physics Static Collider
    for (int gz = 0; gz < m_resolution; ++gz) {
        for (int gx = 0; gx < m_resolution; ++gx) {
            float worldX = -halfPhys + gx * m_spacing;
            float worldZ = -halfPhys + gz * m_spacing;
            m_heightData[gz * m_resolution + gx] = evaluateRawHeight(worldX, worldZ);
        }
    }

    // 2. Construct Circular Planetary Globe Mesh (Diameter = 1,440m with 360-degree curved horizon)
    const int numRings = 64;
    const int numSectors = 72; // 5 degrees per sector
    const float horizonRadius = 720.0f;

    int numVertices = 1 + numRings * numSectors;
    int numTriangles = numSectors + (numRings - 1) * numSectors * 2;

    Mesh mesh = {};
    mesh.vertexCount = numVertices;
    mesh.triangleCount = numTriangles;

    mesh.vertices = static_cast<float*>(MemAlloc(numVertices * 3 * sizeof(float)));
    mesh.normals = static_cast<float*>(MemAlloc(numVertices * 3 * sizeof(float)));
    mesh.texcoords = static_cast<float*>(MemAlloc(numVertices * 2 * sizeof(float)));
    mesh.colors = static_cast<unsigned char*>(MemAlloc(numVertices * 4 * sizeof(unsigned char)));
    mesh.indices = static_cast<unsigned short*>(MemAlloc(numTriangles * 3 * sizeof(unsigned short)));

    // Vertex 0: Planetary Zenith Center (0, Y, 0)
    float centerY = evaluateRawHeight(0.0f, 0.0f);
    mesh.vertices[0] = 0.0f;
    mesh.vertices[1] = centerY;
    mesh.vertices[2] = 0.0f;

    Vector3 centerNorm = getNormal(0.0f, 0.0f);
    mesh.normals[0] = centerNorm.x;
    mesh.normals[1] = centerNorm.y;
    mesh.normals[2] = centerNorm.z;

    mesh.texcoords[0] = 0.0f;
    mesh.texcoords[1] = 0.0f;

    float centerSlope = acosf(Clamp(centerNorm.y, -1.0f, 1.0f));
    Color centerCol = getSlopeColor(centerSlope, centerNorm);
    mesh.colors[0] = centerCol.r;
    mesh.colors[1] = centerCol.g;
    mesh.colors[2] = centerCol.b;
    mesh.colors[3] = centerCol.a;

    // Concentric Globe Rings: Dense near rover (r < 120m), sweeping out to circular planetary horizon (r = 720m)
    for (int rIdx = 0; rIdx < numRings; ++rIdx) {
        float u = static_cast<float>(rIdx + 1) / static_cast<float>(numRings);
        // Non-linear power distribution allocates high polygon density to exploration core
        float radius = powf(u, 1.25f) * horizonRadius;

        for (int sIdx = 0; sIdx < numSectors; ++sIdx) {
            int vIdx = 1 + rIdx * numSectors + sIdx;
            float angle = static_cast<float>(sIdx) * (2.0f * PI / static_cast<float>(numSectors));

            float wx = radius * cosf(angle);
            float wz = radius * sinf(angle);
            float wy = evaluateRawHeight(wx, wz);

            // Outermost horizon rim skirt curves downward below the horizon line
            if (rIdx == numRings - 1) {
                wy -= 36.0f;
            }

            mesh.vertices[vIdx * 3 + 0] = wx;
            mesh.vertices[vIdx * 3 + 1] = wy;
            mesh.vertices[vIdx * 3 + 2] = wz;

            Vector3 norm = getNormal(wx, wz);
            mesh.normals[vIdx * 3 + 0] = norm.x;
            mesh.normals[vIdx * 3 + 1] = norm.y;
            mesh.normals[vIdx * 3 + 2] = norm.z;

            // Continuous world-space UV texture mapping
            mesh.texcoords[vIdx * 2 + 0] = wx / 12.0f;
            mesh.texcoords[vIdx * 2 + 1] = wz / 12.0f;

            float slopeRad = acosf(Clamp(norm.y, -1.0f, 1.0f));
            Color vertexColor = getSlopeColor(slopeRad, norm);

            // Soft atmospheric darkening near the circular horizon edge
            if (rIdx >= numRings - 8) {
                float fade = static_cast<float>(numRings - 1 - rIdx) / 8.0f;
                vertexColor.r = static_cast<unsigned char>(vertexColor.r * (0.6f + 0.4f * fade));
                vertexColor.g = static_cast<unsigned char>(vertexColor.g * (0.6f + 0.4f * fade));
                vertexColor.b = static_cast<unsigned char>(vertexColor.b * (0.6f + 0.4f * fade));
            }

            mesh.colors[vIdx * 4 + 0] = vertexColor.r;
            mesh.colors[vIdx * 4 + 1] = vertexColor.g;
            mesh.colors[vIdx * 4 + 2] = vertexColor.b;
            mesh.colors[vIdx * 4 + 3] = vertexColor.a;
        }
    }

    // Populate Triangles (CCW Winding)
    int tIdx = 0;

    // 1. Central Fan: Center vertex 0 connected to Ring 0
    for (int s = 0; s < numSectors; ++s) {
        int nextS = (s + 1) % numSectors;
        mesh.indices[tIdx++] = 0;
        mesh.indices[tIdx++] = static_cast<unsigned short>(1 + nextS);
        mesh.indices[tIdx++] = static_cast<unsigned short>(1 + s);
    }

    // 2. Concentric Ring Quad Strips
    for (int r = 0; r < numRings - 1; ++r) {
        int currBase = 1 + r * numSectors;
        int nextBase = 1 + (r + 1) * numSectors;

        for (int s = 0; s < numSectors; ++s) {
            int nextS = (s + 1) % numSectors;
            unsigned short v00 = static_cast<unsigned short>(currBase + s);
            unsigned short v01 = static_cast<unsigned short>(currBase + nextS);
            unsigned short v10 = static_cast<unsigned short>(nextBase + s);
            unsigned short v11 = static_cast<unsigned short>(nextBase + nextS);

            // Triangle 1
            mesh.indices[tIdx++] = v00;
            mesh.indices[tIdx++] = v11;
            mesh.indices[tIdx++] = v01;

            // Triangle 2
            mesh.indices[tIdx++] = v00;
            mesh.indices[tIdx++] = v10;
            mesh.indices[tIdx++] = v11;
        }
    }

    // Upload Mesh to GPU
    UploadMesh(&mesh, false);
    m_model = LoadModelFromMesh(mesh);

    // Generate and bind high-resolution procedural Martian detail texture
    generateSurfaceTexture();
    m_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = m_texture;

    m_isLoaded = true;
}

void TerrainHeightfield::generateSurfaceTexture() {
    const int texW = 1024;
    const int texH = 1024;

    Image img = GenImageColor(texW, texH, Color{ 210, 115, 75, 255 });
    Color* pixels = static_cast<Color*>(img.data);

    for (int y = 0; y < texH; ++y) {
        for (int x = 0; x < texW; ++x) {
            float u = static_cast<float>(x) / texW;
            float v = static_cast<float>(y) / texH;

            // Multi-frequency noise for sand grain and dunes
            float n1 = samplePerlin(u * 12.0f, v * 12.0f);
            float n2 = samplePerlin(u * 36.0f, v * 36.0f);
            float n3 = samplePerlin(u * 128.0f, v * 128.0f);

            // Sand dune ripples
            float ripples = sinf(u * 60.0f + n1 * 4.0f) * 0.08f;
            float grain = n1 * 0.45f + n2 * 0.35f + n3 * 0.20f + ripples;

            // Natural Martian ochre palette with micro-contrast
            float r = Clamp(205.0f + grain * 45.0f, 160.0f, 245.0f);
            float g = Clamp(105.0f + grain * 35.0f, 75.0f, 145.0f);
            float b = Clamp(68.0f + grain * 25.0f, 48.0f, 100.0f);

            // Scattered regolith pebbles / basalt flecks
            if (((x * 7919 + y * 65537) & 0x7F) < 3) {
                r *= 0.6f;
                g *= 0.6f;
                b *= 0.6f;
            }

            pixels[y * texW + x] = Color{
                static_cast<unsigned char>(r),
                static_cast<unsigned char>(g),
                static_cast<unsigned char>(b),
                255
            };
        }
    }

    m_texture = LoadTextureFromImage(img);
    GenTextureMipmaps(&m_texture);
    SetTextureFilter(m_texture, TEXTURE_FILTER_TRILINEAR);
    SetTextureWrap(m_texture, TEXTURE_WRAP_REPEAT);
    UnloadImage(img);
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

    // Slope modulation tint for texture
    // Slope < 15 deg: Vibrant Martian dust highlights
    // Slope 15 - 30 deg: Exposed mineral bedrock
    // Slope > 30 deg: Darker basalt ridge
    Color base;
    if (slopeDeg < 15.0f) {
        base = Color{ 255, 245, 235, 255 };
    } else if (slopeDeg <= 30.0f) {
        float t = (slopeDeg - 15.0f) / 15.0f;
        base = Color{
            static_cast<unsigned char>(Lerp(255, 195, t)),
            static_cast<unsigned char>(Lerp(245, 165, t)),
            static_cast<unsigned char>(Lerp(235, 150, t)),
            255
        };
    } else {
        float t = std::min(1.0f, (slopeDeg - 30.0f) / 15.0f);
        base = Color{
            static_cast<unsigned char>(Lerp(195, 135, t)),
            static_cast<unsigned char>(Lerp(165, 115, t)),
            static_cast<unsigned char>(Lerp(150, 105, t)),
            255
        };
    }

    // Directional solar lighting for crisp topography relief
    Vector3 sunDir = Vector3Normalize(Vector3{ 0.4f, 0.85f, 0.35f });
    float diffuse = std::max(0.0f, Vector3DotProduct(normal, sunDir));
    float lightFactor = 0.45f + 0.55f * diffuse; // Ambient + Diffuse

    return Color{
        static_cast<unsigned char>(Clamp(base.r * lightFactor, 0.0f, 255.0f)),
        static_cast<unsigned char>(Clamp(base.g * lightFactor, 0.0f, 255.0f)),
        static_cast<unsigned char>(Clamp(base.b * lightFactor, 0.0f, 255.0f)),
        255
    };
}
