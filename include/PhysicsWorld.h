#pragma once
#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <raylib.h>
#include <memory>
#include <vector>

struct BoulderInfo {
    Vector3 pos;
    float radius;
    Color color;
    JPH::BodyID bodyId;
};

class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();

    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    void init();
    void step(float deltaTime);
    void shutdown();

    // Terrain & Obstacles
    void createTerrainHeightfield(const float* heightData, int cols, int rows, float spacing);
    void spawnBoulder(Vector3 pos, float radius);
    void clearBoulders();

    // Dynamic Test Spheres
    JPH::BodyID spawnDynamicSphere(Vector3 pos, float radius = 1.0f, float mass = 50.0f);
    Vector3 getBodyPosition(JPH::BodyID bodyId) const;
    void clearDynamicSpheres();

    // Vehicle Rigid Body & Dynamics
    JPH::BodyID createChassisBody(Vector3 pos, Vector3 halfExtents, float mass);
    void destroyBody(JPH::BodyID id);
    void applyForceAtPosition(JPH::BodyID id, Vector3 force, Vector3 worldPos);
    void applyTorque(JPH::BodyID id, Vector3 torque);
    void getBodyTransform(JPH::BodyID id, Vector3& outPos, Quaternion& outRot) const;
    void setBodyTransform(JPH::BodyID id, Vector3 pos, Quaternion rot);
    Vector3 getBodyLinearVelocity(JPH::BodyID id) const;
    void setBodyLinearVelocity(JPH::BodyID id, Vector3 vel);
    Vector3 getBodyAngularVelocity(JPH::BodyID id) const;
    void setBodyAngularVelocity(JPH::BodyID id, Vector3 angVel);
    Vector3 getPointVelocity(JPH::BodyID id, Vector3 worldPos) const;
    void setGravity(float g);
    float getGravity() const;

    // Queries
    bool raycast(Vector3 from, Vector3 to, Vector3* hitPoint = nullptr, Vector3* hitNormal = nullptr, JPH::BodyID ignoreBody = JPH::BodyID());
    bool checkSphereClearance(Vector3 center, float radius);

    // Accessors
    JPH::PhysicsSystem& getSystem() { return m_physicsSystem; }
    const std::vector<BoulderInfo>& getBoulders() const { return m_boulders; }
    const std::vector<JPH::BodyID>& getDynamicSpheres() const { return m_dynamicSpheres; }
    bool isInitialized() const { return m_initialized; }

private:
    JPH::PhysicsSystem m_physicsSystem;
    std::unique_ptr<JPH::TempAllocator> m_tempAllocator;
    std::unique_ptr<JPH::JobSystemThreadPool> m_jobSystem;
    JPH::BodyID m_terrainBodyId;

    class BPLayerInterfaceImpl;
    class ObjectVsBroadPhaseLayerFilterImpl;
    class ObjectLayerPairFilterImpl;

    std::unique_ptr<BPLayerInterfaceImpl> m_bpLayerInterface;
    std::unique_ptr<ObjectVsBroadPhaseLayerFilterImpl> m_objectVsBpFilter;
    std::unique_ptr<ObjectLayerPairFilterImpl> m_objectPairFilter;

    std::vector<BoulderInfo> m_boulders;
    std::vector<JPH::BodyID> m_dynamicSpheres;
    bool m_initialized = false;
};
