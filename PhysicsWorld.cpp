#include "PhysicsWorld.h"

#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>

#include <iostream>
#include <thread>
#include <algorithm>
#include <cstdarg>

#ifdef JPH_ENABLE_ASSERTS
static bool AssertFailedImpl(const char* inExpression, const char* inMessage, const char* inFile, JPH::uint inLine) {
    std::cerr << inFile << ":" << inLine << ": (" << inExpression << ") " << (inMessage != nullptr ? inMessage : "") << std::endl;
    return true;
}
namespace JPH {
    AssertFailedFunction AssertFailed = AssertFailedImpl;
}
#endif

static void TraceImpl(const char* inFMT, ...) {
    va_list list;
    va_start(list, inFMT);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), inFMT, list);
    va_end(list);
    std::cout << buffer << std::endl;
}

namespace Layers {
    static constexpr JPH::ObjectLayer NON_MOVING = 0;
    static constexpr JPH::ObjectLayer MOVING = 1;
}

namespace BroadPhaseLayers {
    static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
    static constexpr JPH::BroadPhaseLayer MOVING(1);
    static constexpr JPH::uint NUM_LAYERS(2);
}

class PhysicsWorld::BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    virtual JPH::uint GetNumBroadPhaseLayers() const override {
        return BroadPhaseLayers::NUM_LAYERS;
    }

    virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override {
        switch (inLayer) {
            case Layers::NON_MOVING: return BroadPhaseLayers::NON_MOVING;
            case Layers::MOVING: return BroadPhaseLayers::MOVING;
            default: return BroadPhaseLayers::NON_MOVING;
        }
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    virtual const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override {
        switch ((JPH::BroadPhaseLayer::Type)inLayer) {
            case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING: return "NON_MOVING";
            case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::MOVING: return "MOVING";
            default: return "INVALID";
        }
    }
#endif
};

class PhysicsWorld::ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override {
        switch (inLayer1) {
            case Layers::NON_MOVING:
                return inLayer2 == BroadPhaseLayers::MOVING;
            case Layers::MOVING:
                return true;
            default:
                return false;
        }
    }
};

class PhysicsWorld::ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter {
public:
    virtual bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override {
        switch (inObject1) {
            case Layers::NON_MOVING:
                return inObject2 == Layers::MOVING;
            case Layers::MOVING:
                return true;
            default:
                return false;
        }
    }
};

PhysicsWorld::PhysicsWorld()
    : m_terrainBodyId(JPH::BodyID()), m_initialized(false)
{
}

PhysicsWorld::~PhysicsWorld() {
    shutdown();
}

void PhysicsWorld::init() {
    if (m_initialized) return;

    // Register Jolt allocators and system types
    JPH::RegisterDefaultAllocator();

    // Install trace and assert callbacks
    JPH::Trace = TraceImpl;
    JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = AssertFailedImpl;)

    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();

    // 10 MB temporary allocator for physics collision calculations
    m_tempAllocator = std::make_unique<JPH::TempAllocatorImpl>(10 * 1024 * 1024);

    // Job system using hardware thread pool
    unsigned int numThreads = std::max(1u, std::thread::hardware_concurrency() - 1);
    m_jobSystem = std::make_unique<JPH::JobSystemThreadPool>(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, numThreads);

    m_bpLayerInterface = std::make_unique<BPLayerInterfaceImpl>();
    m_objectVsBpFilter = std::make_unique<ObjectVsBroadPhaseLayerFilterImpl>();
    m_objectPairFilter = std::make_unique<ObjectLayerPairFilterImpl>();

    const JPH::uint cMaxBodies = 2048;
    const JPH::uint cNumBodyMutexes = 0;
    const JPH::uint cMaxBodyPairs = 2048;
    const JPH::uint cMaxContactConstraints = 1024;

    m_physicsSystem.Init(
        cMaxBodies,
        cNumBodyMutexes,
        cMaxBodyPairs,
        cMaxContactConstraints,
        *m_bpLayerInterface,
        *m_objectVsBpFilter,
        *m_objectPairFilter
    );

    // Martian simulated gravity (-3.71 m/s^2)
    m_physicsSystem.SetGravity(JPH::Vec3(0.0f, -3.71f, 0.0f));

    m_initialized = true;
}

void PhysicsWorld::step(float deltaTime) {
    if (!m_initialized) return;
    m_physicsSystem.Update(deltaTime, 1, m_tempAllocator.get(), m_jobSystem.get());
}

void PhysicsWorld::shutdown() {
    if (!m_initialized) return;

    clearDynamicSpheres();

    JPH::BodyInterface& bodyInterface = m_physicsSystem.GetBodyInterface();
    for (const auto& b : m_boulders) {
        if (!b.bodyId.IsInvalid()) {
            bodyInterface.RemoveBody(b.bodyId);
            bodyInterface.DestroyBody(b.bodyId);
        }
    }
    m_boulders.clear();

    if (!m_terrainBodyId.IsInvalid()) {
        bodyInterface.RemoveBody(m_terrainBodyId);
        bodyInterface.DestroyBody(m_terrainBodyId);
        m_terrainBodyId = JPH::BodyID();
    }

    m_bpLayerInterface.reset();
    m_objectVsBpFilter.reset();
    m_objectPairFilter.reset();
    m_jobSystem.reset();
    m_tempAllocator.reset();

    JPH::UnregisterTypes();
    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;

    m_initialized = false;
}

void PhysicsWorld::createTerrainHeightfield(const float* heightData, int cols, int rows, float spacing) {
    if (!m_initialized) return;

    float halfSize = (cols - 1) * spacing * 0.5f;

    JPH::HeightFieldShapeSettings shapeSettings(
        heightData,
        JPH::Vec3(-halfSize, 0.0f, -halfSize),
        JPH::Vec3(spacing, 1.0f, spacing),
        static_cast<JPH::uint32>(cols)
    );

    auto shapeResult = shapeSettings.Create();
    if (!shapeResult.IsValid()) {
        std::cerr << "Failed to create Jolt HeightFieldShape: " << shapeResult.GetError() << std::endl;
        return;
    }

    JPH::ShapeRefC shape = shapeResult.Get();
    JPH::BodyCreationSettings bodySettings(
        shape,
        JPH::RVec3(0.0f, 0.0f, 0.0f),
        JPH::Quat::sIdentity(),
        JPH::EMotionType::Static,
        Layers::NON_MOVING
    );
    bodySettings.mFriction = 0.70f;
    bodySettings.mRestitution = 0.05f;

    JPH::BodyInterface& bodyInterface = m_physicsSystem.GetBodyInterface();
    m_terrainBodyId = bodyInterface.CreateAndAddBody(bodySettings, JPH::EActivation::DontActivate);
}

void PhysicsWorld::spawnBoulder(Vector3 pos, float radius) {
    if (!m_initialized) return;

    JPH::ShapeRefC sphereShape = new JPH::SphereShape(radius);
    JPH::BodyCreationSettings settings(
        sphereShape,
        JPH::RVec3(pos.x, pos.y, pos.z),
        JPH::Quat::sIdentity(),
        JPH::EMotionType::Static,
        Layers::NON_MOVING
    );
    settings.mFriction = 0.85f;
    settings.mRestitution = 0.1f;

    JPH::BodyInterface& bodyInterface = m_physicsSystem.GetBodyInterface();
    JPH::BodyID id = bodyInterface.CreateAndAddBody(settings, JPH::EActivation::DontActivate);

    Color color = Color{ 85, 65, 55, 255 };
    m_boulders.push_back({ pos, radius, color, id });
}

JPH::BodyID PhysicsWorld::spawnDynamicSphere(Vector3 pos, float radius, float mass) {
    if (!m_initialized) return JPH::BodyID();

    JPH::ShapeRefC sphereShape = new JPH::SphereShape(radius);
    JPH::BodyCreationSettings settings(
        sphereShape,
        JPH::RVec3(pos.x, pos.y, pos.z),
        JPH::Quat::sIdentity(),
        JPH::EMotionType::Dynamic,
        Layers::MOVING
    );
    settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    settings.mMassPropertiesOverride.mMass = mass;
    settings.mFriction = 0.55f;
    settings.mRestitution = 0.35f;

    JPH::BodyInterface& bodyInterface = m_physicsSystem.GetBodyInterface();
    JPH::BodyID id = bodyInterface.CreateAndAddBody(settings, JPH::EActivation::Activate);
    m_dynamicSpheres.push_back(id);
    return id;
}

Vector3 PhysicsWorld::getBodyPosition(JPH::BodyID bodyId) const {
    if (bodyId.IsInvalid() || !m_initialized) return Vector3{ 0, 0, 0 };
    const JPH::BodyInterface& bodyInterface = m_physicsSystem.GetBodyInterface();
    JPH::RVec3 p = bodyInterface.GetCenterOfMassPosition(bodyId);
    return Vector3{ static_cast<float>(p.GetX()), static_cast<float>(p.GetY()), static_cast<float>(p.GetZ()) };
}

void PhysicsWorld::clearDynamicSpheres() {
    if (!m_initialized) return;
    JPH::BodyInterface& bodyInterface = m_physicsSystem.GetBodyInterface();
    for (auto id : m_dynamicSpheres) {
        if (!id.IsInvalid()) {
            bodyInterface.RemoveBody(id);
            bodyInterface.DestroyBody(id);
        }
    }
    m_dynamicSpheres.clear();
}

bool PhysicsWorld::raycast(Vector3 from, Vector3 to, Vector3* hitPoint, Vector3* hitNormal) {
    if (!m_initialized) return false;

    JPH::RVec3 origin(from.x, from.y, from.z);
    JPH::Vec3 dir(to.x - from.x, to.y - from.y, to.z - from.z);
    float len = dir.Length();
    if (len < 1e-4f) return false;

    JPH::RRayCast ray(origin, dir);
    JPH::RayCastResult hit;
    bool hasHit = m_physicsSystem.GetNarrowPhaseQuery().CastRay(ray, hit);
    if (hasHit && hit.mFraction <= 1.0f) {
        if (hitPoint) {
            JPH::RVec3 p = ray.GetPointOnRay(hit.mFraction);
            *hitPoint = Vector3{ static_cast<float>(p.GetX()), static_cast<float>(p.GetY()), static_cast<float>(p.GetZ()) };
        }
        if (hitNormal) {
            JPH::BodyLockRead lock(m_physicsSystem.GetBodyLockInterface(), hit.mBodyID);
            if (lock.Succeeded()) {
                const JPH::Body& body = lock.GetBody();
                JPH::Vec3 normal = body.GetWorldSpaceSurfaceNormal(hit.mSubShapeID2, ray.GetPointOnRay(hit.mFraction));
                *hitNormal = Vector3{ static_cast<float>(normal.GetX()), static_cast<float>(normal.GetY()), static_cast<float>(normal.GetZ()) };
            } else {
                *hitNormal = Vector3{ 0.0f, 1.0f, 0.0f };
            }
        }
        return true;
    }
    return false;
}

bool PhysicsWorld::checkSphereClearance(Vector3 center, float radius) {
    if (!m_initialized) return true;
    Vector3 hitPoint;
    if (raycast(center, Vector3{ center.x, center.y - radius, center.z }, &hitPoint)) {
        return false;
    }
    return true;
}
