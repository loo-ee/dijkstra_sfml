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
        case TerrainPreset::OLYMPUS_CRATER:   return "Valley Circuit & Rolling Hills";
        case TerrainPreset::SCREE_SLOPE:      return "Alpine Mountain Pass";
        case TerrainPreset::BOULDER_SLALOM:   return "Rally Canyon & Rhythm Slalom";
        case TerrainPreset::ACIDALIA_PLANITIA: return "Velodrome Bowl & Desert Dunes";
        default: return "Unknown";
    }
}

float TerrainHeightfield::getPresetGravity() const {
    return -9.81f; // Standard Earth gravity for driving simulator
}

const char* TerrainHeightfield::getEnvironmentName() const {
    return "Earth Proving Ground (g = 9.81 m/s²)";
}

void TerrainHeightfield::cyclePreset() {
    int next = (static_cast<int>(m_preset) + 1) % 4;
    setPreset(static_cast<TerrainPreset>(next));
}

void TerrainHeightfield::configureCratersForPreset() {
    m_craters.clear();
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
        // Valley Circuit & Rolling Hills:
        // Broad sweeping hills with crest jumps and a natural banked raceway corridor
        float hills = samplePerlin(x * 0.007f, z * 0.007f) * 7.5f;
        float subWaves = samplePerlin((x + 120.0f) * 0.015f, (z + 80.0f) * 0.015f) * 3.2f;

        // Winding natural raceway corridor: track center oscillates smoothly
        float trackX = sinf(z * 0.012f) * 45.0f;
        float distToTrack = fabsf(x - trackX);
        float trackFactor = Clamp(distToTrack / 30.0f, 0.0f, 1.0f);

        // Valley road bed is smooth, with banked turns on the outside of bends
        float valleyDip = -3.2f * (1.0f - trackFactor);
        float curvature = -cosf(z * 0.012f) * 0.012f;
        float banking = (x - trackX) * curvature * 3.0f * (1.0f - trackFactor);

        height = (hills + subWaves) * (0.35f + 0.65f * trackFactor) + valleyDip + banking;
    } else if (m_preset == TerrainPreset::SCREE_SLOPE) {
        // Alpine Mountain Pass & Terraced Ridges:
        // Panoramic mountain pass ascent with sweeping terrace benches
        float passAscent = sinf(z * 0.006f) * 12.0f + cosf(x * 0.007f) * 7.0f;
        float terraceBenches = samplePerlin(x * 0.009f, z * 0.009f) * 4.8f;
        float smoothRollers = sinf(x * 0.022f + z * 0.016f) * 1.6f;
        height = passAscent + terraceBenches + smoothRollers;
    } else if (m_preset == TerrainPreset::BOULDER_SLALOM) {
        // Rally Canyon & Rhythm Slalom:
        // S-curve canyon valley with gentle rhythm waves for dynamic suspension feedback
        float canyonCenter = sinf(z * 0.016f) * 36.0f;
        float distFromCenter = fabsf(x - canyonCenter);
        float valley = -5.5f * (1.0f - Clamp(distFromCenter / 38.0f, 0.0f, 1.0f));
        float rhythmWaves = sinf(z * 0.055f) * 1.5f * (1.0f - Clamp(distFromCenter / 24.0f, 0.0f, 1.0f));
        float canyonFlanks = samplePerlin(x * 0.008f, z * 0.008f) * 5.2f;
        height = valley + rhythmWaves + canyonFlanks;
    } else { // ACIDALIA_PLANITIA
        // Velodrome Bowl & Desert Dunes:
        // Expansive gentle dunes with a huge banked outer bowl for flat-out top speed
        float dunes = samplePerlin(x * 0.005f, z * 0.005f) * 4.2f;
        float bowlCurve = ((x * x + z * z) / 220000.0f) * 6.5f;
        float subtleRollers = sinf(x * 0.018f + z * 0.018f) * 1.0f;
        height = dunes + Clamp(bowlCurve, 0.0f, 8.5f) + subtleRollers;
    }

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

    // 2. Construct Planar Square Grid Mesh for Flat World
    const int gridRes = 96;
    float cellSpacing = m_size / (gridRes - 1);

    int numVertices = gridRes * gridRes;
    int numQuads = (gridRes - 1) * (gridRes - 1);
    int numTriangles = numQuads * 2;

    Mesh mesh = {};
    mesh.vertexCount = numVertices;
    mesh.triangleCount = numTriangles;

    mesh.vertices = static_cast<float*>(MemAlloc(numVertices * 3 * sizeof(float)));
    mesh.normals = static_cast<float*>(MemAlloc(numVertices * 3 * sizeof(float)));
    mesh.texcoords = static_cast<float*>(MemAlloc(numVertices * 2 * sizeof(float)));
    mesh.colors = static_cast<unsigned char*>(MemAlloc(numVertices * 4 * sizeof(unsigned char)));
    mesh.indices = static_cast<unsigned short*>(MemAlloc(numTriangles * 3 * sizeof(unsigned short)));

    for (int gz = 0; gz < gridRes; ++gz) {
        for (int gx = 0; gx < gridRes; ++gx) {
            int vIdx = gz * gridRes + gx;
            float wx = -halfPhys + gx * cellSpacing;
            float wz = -halfPhys + gz * cellSpacing;
            float wy = evaluateRawHeight(wx, wz);

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

            mesh.colors[vIdx * 4 + 0] = vertexColor.r;
            mesh.colors[vIdx * 4 + 1] = vertexColor.g;
            mesh.colors[vIdx * 4 + 2] = vertexColor.b;
            mesh.colors[vIdx * 4 + 3] = vertexColor.a;
        }
    }

    // Populate Triangles (CCW Winding)
    int tIdx = 0;
    for (int gz = 0; gz < gridRes - 1; ++gz) {
        for (int gx = 0; gx < gridRes - 1; ++gx) {
            unsigned short topLeft = static_cast<unsigned short>(gz * gridRes + gx);
            unsigned short topRight = static_cast<unsigned short>(topLeft + 1);
            unsigned short bottomLeft = static_cast<unsigned short>((gz + 1) * gridRes + gx);
            unsigned short bottomRight = static_cast<unsigned short>(bottomLeft + 1);

            // Triangle 1
            mesh.indices[tIdx++] = topLeft;
            mesh.indices[tIdx++] = bottomLeft;
            mesh.indices[tIdx++] = topRight;

            // Triangle 2
            mesh.indices[tIdx++] = bottomLeft;
            mesh.indices[tIdx++] = bottomRight;
            mesh.indices[tIdx++] = topRight;
        }
    }

    // Upload Mesh to GPU
    UploadMesh(&mesh, false);
    m_model = LoadModelFromMesh(mesh);

    // Generate and bind high-resolution procedural detail texture
    generateSurfaceTexture();
    m_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = m_texture;

    m_isLoaded = true;
}

void TerrainHeightfield::generateSurfaceTexture() {
    const int texW = 2048;
    const int texH = 2048;

    Image img = GenImageColor(texW, texH, Color{ 60, 130, 65, 255 });
    Color* pixels = static_cast<Color*>(img.data);

    for (int y = 0; y < texH; ++y) {
        for (int x = 0; x < texW; ++x) {
            float u = static_cast<float>(x) / texW;
            float v = static_cast<float>(y) / texH;

            float n1 = samplePerlin(u * 16.0f, v * 16.0f);
            float n2 = samplePerlin(u * 48.0f, v * 48.0f);
            float n3 = samplePerlin(u * 144.0f, v * 144.0f);
            float detailGrain = n1 * 0.45f + n2 * 0.35f + n3 * 0.20f;

            float r = 60.0f, g = 130.0f, b = 65.0f;

            if (m_preset == TerrainPreset::OLYMPUS_CRATER) {
                // LUSH GRASSLAND CIRCUIT (Vibrant turf with asphalt track veins and dirt margins)
                float turfBaseG = 135.0f + detailGrain * 35.0f;
                r = Clamp(55.0f + detailGrain * 25.0f, 35.0f, 105.0f);
                g = Clamp(turfBaseG, 95.0f, 175.0f);
                b = Clamp(50.0f + detailGrain * 20.0f, 30.0f, 95.0f);

                // Procedural road / raceway strip across texture
                float trackDist = fabsf(u - 0.5f - sinf(v * 4.0f * PI) * 0.18f);
                if (trackDist < 0.06f) {
                    // Asphalt raceway surface
                    float tarmac = 48.0f + detailGrain * 18.0f;
                    r = tarmac + 4.0f;
                    g = tarmac + 5.0f;
                    b = tarmac + 8.0f;

                    // Track edge rumble stripe
                    if (trackDist > 0.052f) {
                        float curb = sinf(v * 160.0f);
                        if (curb > 0.0f) { r = 220.0f; g = 50.0f; b = 45.0f; } // Red curb
                        else { r = 240.0f; g = 240.0f; b = 240.0f; }           // White curb
                    }
                }
            } else if (m_preset == TerrainPreset::SCREE_SLOPE) {
                // ALPINE MOUNTAIN PASS (Granite stone, slate grey rock, cool moss)
                float stone = 150.0f + detailGrain * 45.0f;
                r = Clamp(stone + 5.0f, 95.0f, 215.0f);
                g = Clamp(stone + 8.0f, 100.0f, 220.0f);
                b = Clamp(stone + 15.0f, 105.0f, 230.0f);

                // Granite flecks
                uint32_t pHash = (x * 7919 + y * 65537);
                if ((pHash & 0x7F) < 4) { r = 240.0f; g = 242.0f; b = 245.0f; }
                else if ((pHash & 0x7F) > 123) { r = 60.0f; g = 65.0f; b = 70.0f; }
            } else if (m_preset == TerrainPreset::BOULDER_SLALOM) {
                // RALLY CANYON (Warm terracotta, packed red clay raceway)
                float clay = 185.0f + detailGrain * 40.0f;
                r = Clamp(clay + 25.0f, 140.0f, 245.0f);
                g = Clamp(clay * 0.55f, 70.0f, 140.0f);
                b = Clamp(clay * 0.35f, 40.0f, 95.0f);
            } else { // ACIDALIA_PLANITIA
                // GOLDEN VELODROME DUNES (Warm sand ripples & desert varnish)
                float sand = 210.0f + detailGrain * 35.0f;
                r = Clamp(sand + 15.0f, 175.0f, 255.0f);
                g = Clamp(sand * 0.78f, 130.0f, 210.0f);
                b = Clamp(sand * 0.52f, 85.0f, 150.0f);
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
    return evaluateRawHeight(x, z);
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

    Color base;
    if (m_preset == TerrainPreset::OLYMPUS_CRATER) {
        // Lush Grassland Circuit: Green turf on flats, warm earth on banks, stone on crests
        if (slopeDeg < 12.0f) {
            base = Color{ 220, 250, 220, 255 };
        } else if (slopeDeg <= 25.0f) {
            float t = (slopeDeg - 12.0f) / 13.0f;
            base = Color{
                static_cast<unsigned char>(Lerp(220, 190, t)),
                static_cast<unsigned char>(Lerp(250, 175, t)),
                static_cast<unsigned char>(Lerp(220, 140, t)),
                255
            };
        } else {
            base = Color{ 165, 160, 150, 255 }; // Rocky limestone
        }
    } else if (m_preset == TerrainPreset::SCREE_SLOPE) {
        // Alpine Stone Pass
        if (slopeDeg < 14.0f) {
            base = Color{ 235, 240, 250, 255 };
        } else {
            float t = std::min(1.0f, (slopeDeg - 14.0f) / 18.0f);
            base = Color{
                static_cast<unsigned char>(Lerp(235, 130, t)),
                static_cast<unsigned char>(Lerp(240, 135, t)),
                static_cast<unsigned char>(Lerp(250, 145, t)),
                255
            };
        }
    } else if (m_preset == TerrainPreset::BOULDER_SLALOM) {
        // Terracotta Rally Canyon
        if (slopeDeg < 14.0f) {
            base = Color{ 255, 225, 205, 255 };
        } else {
            float t = std::min(1.0f, (slopeDeg - 14.0f) / 18.0f);
            base = Color{
                static_cast<unsigned char>(Lerp(255, 160, t)),
                static_cast<unsigned char>(Lerp(225, 95, t)),
                static_cast<unsigned char>(Lerp(205, 65, t)),
                255
            };
        }
    } else { // ACIDALIA_PLANITIA
        // Golden Velodrome Dunes
        if (slopeDeg < 15.0f) {
            base = Color{ 255, 245, 225, 255 };
        } else {
            float t = std::min(1.0f, (slopeDeg - 15.0f) / 18.0f);
            base = Color{
                static_cast<unsigned char>(Lerp(255, 180, t)),
                static_cast<unsigned char>(Lerp(245, 145, t)),
                static_cast<unsigned char>(Lerp(225, 105, t)),
                255
            };
        }
    }

    // Solar lighting for crisp topography relief
    Vector3 sunDir = Vector3Normalize(Vector3{ 0.4f, 0.85f, 0.35f });
    float diffuse = std::max(0.0f, Vector3DotProduct(normal, sunDir));
    float lightFactor = 0.44f + 0.56f * diffuse;

    return Color{
        static_cast<unsigned char>(Clamp(base.r * lightFactor, 0.0f, 255.0f)),
        static_cast<unsigned char>(Clamp(base.g * lightFactor, 0.0f, 255.0f)),
        static_cast<unsigned char>(Clamp(base.b * lightFactor, 0.0f, 255.0f)),
        255
    };
}
