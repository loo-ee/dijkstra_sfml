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

class TerrainHeightfield {
public:
    TerrainHeightfield(int resolution = 128, float size = 200.0f);
    ~TerrainHeightfield();

    // Prevent accidental copying of GPU model
    TerrainHeightfield(const TerrainHeightfield&) = delete;
    TerrainHeightfield& operator=(const TerrainHeightfield&) = delete;

    void generate();
    void unload();

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

    int m_resolution;
    float m_size;
    float m_spacing;
    std::vector<float> m_heightData;
    std::vector<CraterParam> m_craters;
    Model m_model = {};
    bool m_isLoaded = false;
};
