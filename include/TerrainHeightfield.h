#pragma once
#include <raylib.h>
#include <raymath.h>
#include <vector>
#include <cmath>

struct CraterParam {
    Vector2 center;
    float radius;
    float depth;
    float rimHeight;
    float rimWidth;
};

enum class TerrainPreset {
    OLYMPUS_CRATER = 0,   // Impact basin with raised rim wall & central depression
    SCREE_SLOPE,          // High-grade mountain slope with stepped terracing and dynamic scree
    BOULDER_SLALOM,       // Winding Martian canyon pass with rock hazards
    ACIDALIA_PLANITIA     // Smooth rolling dunes & low-gradient plains for high-speed cruising
};

class TerrainHeightfield {
public:
    TerrainHeightfield(int resolution = 128, float size = 200.0f);
    ~TerrainHeightfield();

    // Prevent accidental copying of GPU model
    TerrainHeightfield(const TerrainHeightfield&) = delete;
    TerrainHeightfield& operator=(const TerrainHeightfield&) = delete;

    void generate();
    void unload();

    // Terrain Preset & Realistic Environment Gravity
    void setPreset(TerrainPreset preset);
    TerrainPreset getPreset() const { return m_preset; }
    const char* getPresetName() const;
    float getPresetGravity() const;
    const char* getEnvironmentName() const;
    void cyclePreset();

    // Queries
    float getHeight(float x, float z) const;
    Vector3 getNormal(float x, float z) const;
    float getSlopeAngleRad(float x, float z) const;
    Color getSlopeColor(float slopeRad, Vector3 normal) const;

    // Accessors
    int getResolution() const { return m_resolution; }
    float getSize() const { return m_size; }
    const Model& getModel() const { return m_model; }
    const std::vector<float>& getHeightData() const { return m_heightData; }
    bool isLoaded() const { return m_isLoaded; }

private:
    float evaluateRawHeight(float x, float z) const;
    float samplePerlin(float x, float z) const;
    void generateSurfaceTexture();
    void configureCratersForPreset();

    int m_resolution;
    float m_size;
    float m_spacing;
    TerrainPreset m_preset = TerrainPreset::OLYMPUS_CRATER;
    std::vector<float> m_heightData;
    std::vector<CraterParam> m_craters;
    Model m_model = {};
    Texture2D m_texture = {};
    bool m_isLoaded = false;
};
