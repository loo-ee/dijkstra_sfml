#include "PlanetaryRover.h"
#include "PhysicsWorld.h"
#include "Vertex3D.h"
#include <rlgl.h>
#include <cmath>
#include <algorithm>

PlanetaryRover::PlanetaryRover()
    : m_chassisBodyId()
    , m_halfExtents{ 0.70f, 0.35f, 1.15f } // width 1.4m, height 0.7m, length 2.3m
    , m_mass(800.0f)
    , m_position{ 0, 0, 0 }
    , m_rotation(QuaternionIdentity())
    , m_forward{ 0, 0, 1 }
    , m_right{ 1, 0, 0 }
    , m_up{ 0, 1, 0 }
    , m_linearVelocity{ 0, 0, 0 }
    , m_speed(0.0f)
    , m_pitchDeg(0.0f)
    , m_rollDeg(0.0f)
    , m_yawDeg(0.0f)
    , m_isRolloverWarning(false)
    , m_odometerMeters(0.0f)
    , m_batteryJoules(0.0f)
    , m_currentPowerWatts(0.0f)
    , m_suspensionRestLength(0.45f)
    , m_springStiffness(36000.0f)
    , m_springDamping(3600.0f)
    , m_wheelRadius(0.45f)
    , m_wheelWidth(0.35f)
    , m_antiRollBarStiffness(7500.0f)
    , m_throttleInput(0.0f)
    , m_steerInput(0.0f)
    , m_brakeInput(0.0f)
    , m_tcsEngagedOverall(false)
    , m_currentWaypointIndex(0)
    , m_targetWaypoint{ 0, 0, 0 }
    , m_lookaheadDist(4.0f)
    , m_crossTrackError(0.0f)
    , m_isAutonomous(false)
    , m_hasReachedGoal(false)
    , m_cameraMode(RoverCameraMode::ORBIT)
    , m_chaseCamPos{ 0, 10, -10 }
    , m_chaseCamTarget{ 0, 0, 0 }
{
    // Mount positions in local chassis coordinate system:
    // Local: +X = Right, +Y = Up, +Z = Forward
    m_wheels[0].mountOffset = Vector3{ -0.80f, -0.10f, +0.95f }; // Front-Left (FL)
    m_wheels[1].mountOffset = Vector3{ +0.80f, -0.10f, +0.95f }; // Front-Right (FR)
    m_wheels[2].mountOffset = Vector3{ -0.80f, -0.10f, -0.95f }; // Rear-Left (RL)
    m_wheels[3].mountOffset = Vector3{ +0.80f, -0.10f, -0.95f }; // Rear-Right (RR)

    for (int i = 0; i < 4; ++i) {
        m_wheels[i].worldMountPos = Vector3{ 0, 0, 0 };
        m_wheels[i].worldWheelPos = Vector3{ 0, 0, 0 };
        m_wheels[i].contactPoint  = Vector3{ 0, 0, 0 };
        m_wheels[i].contactNormal = Vector3{ 0, 1, 0 };
        m_wheels[i].suspensionLength = m_suspensionRestLength;
        m_wheels[i].suspensionCompression = 0.0f;
        m_wheels[i].spinAngle = 0.0f;
        m_wheels[i].angularVelocity = 0.0f;
        m_wheels[i].steerAngle = 0.0f;
        m_wheels[i].slipRatio = 0.0f;
        m_wheels[i].isGrounded = false;
        m_wheels[i].tcsActive = false;
    }
}

PlanetaryRover::~PlanetaryRover() {}

void PlanetaryRover::init(PhysicsWorld& physics, Vector3 spawnPos, float yawAngleRad) {
    reset(physics, spawnPos, yawAngleRad);
}

void PlanetaryRover::reset(PhysicsWorld& physics, Vector3 spawnPos, float yawAngleRad) {
    if (!m_chassisBodyId.IsInvalid()) {
        physics.destroyBody(m_chassisBodyId);
        m_chassisBodyId = JPH::BodyID();
    }

    // Spawn slightly elevated so wheels cleanly drop onto suspension
    Vector3 elevatedPos = spawnPos;
    elevatedPos.y += 0.85f;

    m_chassisBodyId = physics.createChassisBody(elevatedPos, m_halfExtents, m_mass);
    Quaternion initRot = QuaternionFromAxisAngle(Vector3{ 0, 1, 0 }, yawAngleRad);
    physics.setBodyTransform(m_chassisBodyId, elevatedPos, initRot);
    physics.setBodyLinearVelocity(m_chassisBodyId, Vector3{ 0, 0, 0 });
    physics.setBodyAngularVelocity(m_chassisBodyId, Vector3{ 0, 0, 0 });

    m_position = elevatedPos;
    m_rotation = initRot;
    m_forward  = Vector3RotateByQuaternion(Vector3{ 0, 0, 1 }, m_rotation);
    m_right    = Vector3RotateByQuaternion(Vector3{ 1, 0, 0 }, m_rotation);
    m_up       = Vector3RotateByQuaternion(Vector3{ 0, 1, 0 }, m_rotation);
    m_linearVelocity = Vector3{ 0, 0, 0 };
    m_speed = 0.0f;

    m_throttleInput = 0.0f;
    m_steerInput    = 0.0f;
    m_brakeInput    = 0.0f;
    m_tcsEngagedOverall = false;
    m_hasReachedGoal = false;
    m_currentWaypointIndex = 0;

    for (int i = 0; i < 4; ++i) {
        m_wheels[i].suspensionLength = m_suspensionRestLength;
        m_wheels[i].suspensionCompression = 0.0f;
        m_wheels[i].angularVelocity = 0.0f;
        m_wheels[i].spinAngle = 0.0f;
        m_wheels[i].steerAngle = 0.0f;
        m_wheels[i].slipRatio = 0.0f;
        m_wheels[i].isGrounded = false;
        m_wheels[i].tcsActive = false;
    }

    m_chaseCamPos = Vector3Subtract(m_position, Vector3Scale(m_forward, 6.5f));
    m_chaseCamPos.y += 3.0f;
    m_chaseCamTarget = m_position;
}

void PlanetaryRover::setPath(const std::vector<const Vertex3D*>& pathNodes) {
    m_waypoints.clear();
    m_waypoints.reserve(pathNodes.size());
    for (const Vertex3D* node : pathNodes) {
        if (node) {
            m_waypoints.push_back(node->position);
        }
    }
    m_currentWaypointIndex = 0;
    m_hasReachedGoal = false;
    if (!m_waypoints.empty()) {
        m_targetWaypoint = m_waypoints.front();
    }
}

void PlanetaryRover::cycleCameraMode() {
    if (m_cameraMode == RoverCameraMode::ORBIT) {
        m_cameraMode = RoverCameraMode::CHASE;
    } else if (m_cameraMode == RoverCameraMode::CHASE) {
        m_cameraMode = RoverCameraMode::MAST;
    } else {
        m_cameraMode = RoverCameraMode::ORBIT;
    }
}

Camera3D PlanetaryRover::getCamera(const Camera3D& orbitCamera) const {
    if (m_cameraMode == RoverCameraMode::ORBIT) {
        return orbitCamera;
    }

    Camera3D cam = {};
    cam.up = Vector3{ 0.0f, 1.0f, 0.0f };

    if (m_cameraMode == RoverCameraMode::CHASE) {
        Vector3 desiredPos = Vector3Subtract(m_position, Vector3Scale(m_forward, 6.2f));
        desiredPos.y += 2.8f;
        Vector3 desiredTarget = Vector3Add(m_position, Vector3Scale(m_forward, 2.5f));
        desiredTarget.y += 0.8f;

        m_chaseCamPos = Vector3Lerp(m_chaseCamPos, desiredPos, 0.12f);
        m_chaseCamTarget = Vector3Lerp(m_chaseCamTarget, desiredTarget, 0.15f);

        cam.position = m_chaseCamPos;
        cam.target = m_chaseCamTarget;
        cam.fovy = 52.0f;
        cam.projection = CAMERA_PERSPECTIVE;
    } else { // MAST CAM (1st-person forward viewing head sensor)
        Vector3 mastEye = Vector3Add(m_position, Vector3Scale(m_up, 1.30f));
        mastEye = Vector3Add(mastEye, Vector3Scale(m_forward, 0.50f));
        Vector3 mastTarget = Vector3Add(mastEye, Vector3Scale(m_forward, 15.0f));
        mastTarget.y -= 1.2f;

        cam.position = mastEye;
        cam.target = mastTarget;
        cam.fovy = 65.0f;
        cam.projection = CAMERA_PERSPECTIVE;
    }
    return cam;
}

void PlanetaryRover::update(PhysicsWorld& physics, float dt) {
    if (m_chassisBodyId.IsInvalid()) return;

    // 1. Fetch latest physical transform and velocity from Jolt
    physics.getBodyTransform(m_chassisBodyId, m_position, m_rotation);
    m_forward = Vector3RotateByQuaternion(Vector3{ 0, 0, 1 }, m_rotation);
    m_right   = Vector3RotateByQuaternion(Vector3{ 1, 0, 0 }, m_rotation);
    m_up      = Vector3RotateByQuaternion(Vector3{ 0, 1, 0 }, m_rotation);

    m_linearVelocity = physics.getBodyLinearVelocity(m_chassisBodyId);
    m_speed = Vector3DotProduct(m_linearVelocity, m_forward);

    // 2. Attitude calculation & rollover hazard detection
    updateAttitudeAndSensors(dt);

    // 3. Autonomous Pure Pursuit guidance (if enabled)
    if (m_isAutonomous) {
        updatePurePursuit(dt);
    }

    // 4. Raycast suspension, tire dynamics, TCS, and drive forces
    updateSuspensionAndTires(physics, dt);
}

void PlanetaryRover::updateAttitudeAndSensors(float dt) {
    // Pitch: elevation of forward vector (nose up = +pitch)
    m_pitchDeg = asinf(Clamp(m_forward.y, -1.0f, 1.0f)) * RAD2DEG;

    // Roll: elevation of right vector (right tilted down = -roll)
    m_rollDeg = asinf(Clamp(m_right.y, -1.0f, 1.0f)) * RAD2DEG;

    // Yaw: heading angle in horizontal plane
    m_yawDeg = atan2f(m_forward.x, m_forward.z) * RAD2DEG;

    // Critical rollover safety threshold (> 28 deg inclination)
    m_isRolloverWarning = (fabsf(m_pitchDeg) > 28.0f || fabsf(m_rollDeg) > 28.0f);

    // Integrate odometer
    float frameDist = fabsf(m_speed) * dt;
    m_odometerMeters += frameDist;
}

void PlanetaryRover::updatePurePursuit(float dt) {
    if (m_waypoints.empty()) {
        m_throttleInput = 0.0f;
        m_brakeInput = 1.0f;
        return;
    }

    // Check distance to goal (last waypoint)
    Vector3 goalPos = m_waypoints.back();
    float distToGoal = Vector2Distance(Vector2{ m_position.x, m_position.z }, Vector2{ goalPos.x, goalPos.z });

    if (distToGoal < 1.6f) {
        m_hasReachedGoal = true;
        m_throttleInput = 0.0f;
        m_brakeInput = 1.0f;
        m_steerInput = 0.0f;
        return;
    }

    // 1. Advance waypoint index if rover has arrived within 2.8m of current waypoint
    int totalWp = static_cast<int>(m_waypoints.size());
    while (m_currentWaypointIndex < totalWp - 1) {
        float d = Vector2Distance(Vector2{ m_position.x, m_position.z }, 
                                  Vector2{ m_waypoints[m_currentWaypointIndex].x, m_waypoints[m_currentWaypointIndex].z });
        if (d < 3.2f) {
            m_currentWaypointIndex++;
        } else {
            break;
        }
    }

    // 2. Select lookahead waypoint w_k at distance ~ L_d (4.0m)
    m_lookaheadDist = 4.0f;
    int targetIdx = m_currentWaypointIndex;
    for (int i = m_currentWaypointIndex; i < totalWp; ++i) {
        float d = Vector2Distance(Vector2{ m_position.x, m_position.z }, 
                                  Vector2{ m_waypoints[i].x, m_waypoints[i].z });
        targetIdx = i;
        if (d >= m_lookaheadDist) {
            break;
        }
    }
    m_targetWaypoint = m_waypoints[targetIdx];

    // 3. Compute vector to lookahead target in rover local frame
    Vector3 toTarget = Vector3Subtract(m_targetWaypoint, m_position);
    float localX = Vector3DotProduct(toTarget, m_right);
    float localZ = Vector3DotProduct(toTarget, m_forward);
    m_crossTrackError = localX;

    // Angle alpha between heading and target
    float alpha = atan2f(localX, fmaxf(0.1f, localZ));

    // Pure Pursuit Steering Curvature: kappa = 2*sin(alpha)/Ld
    // Steering angle: delta = atan(kappa * L) where L = wheelbase ~ 1.9m
    const float wheelbase = 1.90f;
    float targetSteer = atan2f(2.0f * wheelbase * sinf(alpha), m_lookaheadDist);
    targetSteer = Clamp(targetSteer, -0.62f, 0.62f); // Max ~35 deg steering lock

    // Smooth steering rate limiter (prevents sudden tire jerk)
    float steerRate = 3.5f;
    m_steerInput += Clamp(targetSteer - m_steerInput, -steerRate * dt, steerRate * dt);

    // 4. Target speed regulation & Incline assist
    float cruiseSpeed = 4.2f; // ~15.1 km/h cruise

    // Turn slowdown: modulate speed inversely with steer magnitude
    float steerPenalty = 1.0f - 0.45f * (fabsf(m_steerInput) / 0.62f);
    cruiseSpeed *= steerPenalty;

    // Deceleration ramp as we near the final destination
    if (distToGoal < 7.0f) {
        cruiseSpeed *= fmaxf(0.2f, distToGoal / 7.0f);
    }

    // Incline Assist: if climbing uphill, boost torque to overcome gravity
    float inclineFactor = 1.0f;
    if (m_pitchDeg > 2.0f) {
        inclineFactor += 1.8f * fminf(1.0f, sinf(m_pitchDeg * DEG2RAD));
    }

    // Speed error control
    float speedError = cruiseSpeed - m_speed;
    if (speedError > 0.0f) {
        m_throttleInput = Clamp(speedError * 0.45f * inclineFactor, 0.20f, 1.0f);
        m_brakeInput = 0.0f;
    } else {
        m_throttleInput = 0.0f;
        m_brakeInput = Clamp(-speedError * 0.5f, 0.0f, 0.8f);
    }
}

void PlanetaryRover::updateSuspensionAndTires(PhysicsWorld& physics, float dt) {
    const float maxRayDist = m_suspensionRestLength + m_wheelRadius;
    m_tcsEngagedOverall = false;

    // Apply front steering angles
    m_wheels[0].steerAngle = m_steerInput; // FL
    m_wheels[1].steerAngle = m_steerInput; // FR
    m_wheels[2].steerAngle = 0.0f;         // RL
    m_wheels[3].steerAngle = 0.0f;         // RR

    float totalWorkRate = 0.0f;

    // Step 1: Compute Raycast Suspension & Contact for all 4 wheels
    for (int i = 0; i < 4; ++i) {
        WheelState& w = m_wheels[i];
        w.tcsActive = false;

        // World mount position of strut top
        Vector3 localMount = w.mountOffset;
        w.worldMountPos = Vector3Add(m_position, Vector3RotateByQuaternion(localMount, m_rotation));

        // Downward raycast along chassis negative up vector
        Vector3 rayDir = Vector3Scale(m_up, -1.0f);
        Vector3 rayEnd = Vector3Add(w.worldMountPos, Vector3Scale(rayDir, maxRayDist));

        Vector3 hitPoint, hitNormal;
        bool grounded = physics.raycast(w.worldMountPos, rayEnd, &hitPoint, &hitNormal);
        w.isGrounded = grounded;

        if (grounded) {
            float hitDist = Vector3Distance(w.worldMountPos, hitPoint);
            w.contactPoint  = hitPoint;
            w.contactNormal = hitNormal;

            // Wheel hub sits at radius R above ground along contact normal/suspension axis
            w.suspensionLength = hitDist - m_wheelRadius;
            w.suspensionCompression = Clamp(m_suspensionRestLength - w.suspensionLength, 0.0f, m_suspensionRestLength);
            w.worldWheelPos = Vector3Add(w.worldMountPos, Vector3Scale(rayDir, w.suspensionLength));

            // Spring-damper force calculation
            // Relative velocity of mount point along suspension ray
            Vector3 mountVel = physics.getPointVelocity(m_chassisBodyId, w.worldMountPos);
            float vRel = Vector3DotProduct(mountVel, rayDir); // positive when compressing

            float springForceMag = (m_springStiffness * w.suspensionCompression) + (m_springDamping * vRel);
            springForceMag = Clamp(springForceMag, 0.0f, 28000.0f);

            // Apply suspension upward force to chassis
            Vector3 suspForce = Vector3Scale(m_up, springForceMag);
            physics.applyForceAtPosition(m_chassisBodyId, suspForce, w.worldMountPos);

            // ----------------------------------------------------
            // Tire Contact Forces (Lateral Cornering & Longitudinal Drive)
            // ----------------------------------------------------
            Vector3 wheelHubVel = physics.getPointVelocity(m_chassisBodyId, w.worldWheelPos);

            // Calculate tire orientation vectors based on steering angle
            float cosS = cosf(w.steerAngle);
            float sinS = sinf(w.steerAngle);
            Vector3 tireForward = Vector3Add(Vector3Scale(m_forward, cosS), Vector3Scale(m_right, sinS));
            Vector3 tireRight   = Vector3Subtract(Vector3Scale(m_right, cosS), Vector3Scale(m_forward, sinS));

            float vLong = Vector3DotProduct(wheelHubVel, tireForward);
            float vLat  = Vector3DotProduct(wheelHubVel, tireRight);

            // Lateral Friction (Anti-skid cornering force)
            float muLateral = 0.85f;
            float maxLatForce = springForceMag * muLateral;
            float latGripMag = -vLat * 5500.0f; // Cornering stiffness
            latGripMag = Clamp(latGripMag, -maxLatForce, maxLatForce);
            Vector3 lateralForceVec = Vector3Scale(tireRight, latGripMag);
            physics.applyForceAtPosition(m_chassisBodyId, lateralForceVec, w.contactPoint);

            // Longitudinal Drive & Traction Control System (TCS)
            float nominalTireSpeed = w.angularVelocity * m_wheelRadius;
            float slipRatio = fabsf(nominalTireSpeed - vLong) / fmaxf(0.3f, fabsf(vLong));
            w.slipRatio = Clamp(slipRatio, 0.0f, 1.5f);

            // Traction Control: Suppress drive force if slip ratio exceeds 0.20
            float tcsFactor = 1.0f;
            if (w.slipRatio > 0.20f && m_throttleInput > 0.05f) {
                tcsFactor = 1.0f - Clamp((w.slipRatio - 0.20f) * 3.2f, 0.0f, 0.85f);
                w.tcsActive = true;
                m_tcsEngagedOverall = true;
            }

            // Motor Drive Force (All-Wheel Drive 4WD)
            const float maxMotorForcePerWheel = 2800.0f; // Newtons
            float driveForceMag = m_throttleInput * maxMotorForcePerWheel * tcsFactor;

            // Braking Force
            if (m_brakeInput > 0.02f) {
                float brakeMag = -vLong * 5000.0f * m_brakeInput;
                brakeMag = Clamp(brakeMag, -springForceMag * 0.9f, springForceMag * 0.9f);
                driveForceMag += brakeMag;
            }

            // Coulomb friction clamp
            float maxLongForce = springForceMag * 0.95f;
            driveForceMag = Clamp(driveForceMag, -maxLongForce, maxLongForce);

            Vector3 longForceVec = Vector3Scale(tireForward, driveForceMag);
            physics.applyForceAtPosition(m_chassisBodyId, longForceVec, w.contactPoint);

            // Update wheel rotation
            float targetAngVel = (vLong / m_wheelRadius) + (m_throttleInput * (1.0f - tcsFactor) * 8.0f);
            w.angularVelocity = Lerp(w.angularVelocity, targetAngVel, 0.25f);
            w.spinAngle += w.angularVelocity * dt;

            // Power calculation: P = F * v
            totalWorkRate += fabsf(driveForceMag * vLong);
        } else {
            // In the air (unloaded)
            w.suspensionLength = m_suspensionRestLength;
            w.suspensionCompression = 0.0f;
            w.worldWheelPos = Vector3Add(w.worldMountPos, Vector3Scale(rayDir, m_suspensionRestLength));
            w.contactPoint  = w.worldWheelPos;
            w.slipRatio = 0.0f;
            w.angularVelocity = Lerp(w.angularVelocity, 0.0f, 0.05f);
            w.spinAngle += w.angularVelocity * dt;
        }
    }

    // Step 2: Anti-Roll Bar Stabilization (Front axle: 0-1, Rear axle: 2-3)
    float diffFront = m_wheels[0].suspensionCompression - m_wheels[1].suspensionCompression;
    float arbForceF = diffFront * m_antiRollBarStiffness;
    physics.applyForceAtPosition(m_chassisBodyId, Vector3Scale(m_up, -arbForceF), m_wheels[0].worldMountPos);
    physics.applyForceAtPosition(m_chassisBodyId, Vector3Scale(m_up, +arbForceF), m_wheels[1].worldMountPos);

    float diffRear = m_wheels[2].suspensionCompression - m_wheels[3].suspensionCompression;
    float arbForceR = diffRear * m_antiRollBarStiffness;
    physics.applyForceAtPosition(m_chassisBodyId, Vector3Scale(m_up, -arbForceR), m_wheels[2].worldMountPos);
    physics.applyForceAtPosition(m_chassisBodyId, Vector3Scale(m_up, +arbForceR), m_wheels[3].worldMountPos);

    // Energy tracking
    m_currentPowerWatts = totalWorkRate;
    m_batteryJoules += totalWorkRate * dt;
}

void PlanetaryRover::render(float sceneTime) const {
    if (m_chassisBodyId.IsInvalid()) return;

    // -----------------------------------------------------------------
    // 1. CHASSIS RIGID BODY & SCIENTIFIC PAYLOAD
    // -----------------------------------------------------------------
    // Construct 4x4 transform matrix from position and quaternion
    Matrix matRot = QuaternionToMatrix(m_rotation);
    Matrix matTrans = MatrixTranslate(m_position.x, m_position.y, m_position.z);
    Matrix matWorld = MatrixMultiply(matRot, matTrans);

    rlPushMatrix();
    rlMultMatrixf(MatrixToFloat(matWorld));
    {
        // A. Primary Avionics Hull (Titanium aerospace silver with bevel)
        Vector3 hullSize = { m_halfExtents.x * 2.0f, m_halfExtents.y * 1.5f, m_halfExtents.z * 1.8f };
        DrawCube(Vector3{ 0.0f, 0.05f, 0.0f }, hullSize.x, hullSize.y, hullSize.z, Color{ 220, 226, 235, 255 });
        DrawCubeWires(Vector3{ 0.0f, 0.05f, 0.0f }, hullSize.x, hullSize.y, hullSize.z, Color{ 70, 85, 105, 255 });

        // B. Golden Thermal Multi-Layer Insulation (MLI) Foil Shielding on sides
        DrawCube(Vector3{ -hullSize.x * 0.51f, 0.05f, 0.0f }, 0.04f, hullSize.y * 0.82f, hullSize.z * 0.85f, Color{ 212, 175, 55, 255 });
        DrawCube(Vector3{ +hullSize.x * 0.51f, 0.05f, 0.0f }, 0.04f, hullSize.y * 0.82f, hullSize.z * 0.85f, Color{ 212, 175, 55, 255 });

        // C. Solar Array Top Panels (Dark photovoltaic blue with metallic borders)
        DrawCube(Vector3{ 0.0f, hullSize.y * 0.52f, 0.0f }, hullSize.x * 0.94f, 0.05f, hullSize.z * 0.90f, Color{ 20, 36, 68, 255 });
        DrawCubeWires(Vector3{ 0.0f, hullSize.y * 0.52f, 0.0f }, hullSize.x * 0.94f, 0.05f, hullSize.z * 0.90f, Color{ 65, 130, 220, 255 });

        // D. Rear RTG (Radioisotope Thermoelectric Generator) Power Unit with Cooling Fins
        Vector3 rtgCenter = { 0.0f, 0.15f, -m_halfExtents.z * 0.95f };
        DrawCube(rtgCenter, 0.50f, 0.40f, 0.45f, Color{ 45, 50, 60, 255 });
        DrawCubeWires(rtgCenter, 0.50f, 0.40f, 0.45f, Color{ 200, 100, 40, 255 }); // Warm thermal glow

        // E. High-Gain Parabolic Communications Dish (Mounted on rear right deck)
        Vector3 dishBase = { 0.40f, hullSize.y * 0.55f, -0.60f };
        DrawCylinderEx(dishBase, Vector3Add(dishBase, Vector3{ 0.0f, 0.35f, 0.0f }), 0.04f, 0.04f, 6, Color{ 160, 170, 185, 255 });
        DrawSphere(Vector3Add(dishBase, Vector3{ 0.0f, 0.35f, 0.0f }), 0.22f, Color{ 240, 240, 245, 255 });
        DrawSphereWires(Vector3Add(dishBase, Vector3{ 0.0f, 0.35f, 0.0f }), 0.22f, 6, 6, Color{ 100, 115, 130, 255 });

        // F. Forward Remote Sensing Mast (Mastcam & LIDAR Head)
        Vector3 mastBase = { -0.35f, hullSize.y * 0.52f, 0.70f };
        Vector3 mastTop  = Vector3Add(mastBase, Vector3{ 0.0f, 1.10f, 0.0f });
        DrawCylinderEx(mastBase, mastTop, 0.045f, 0.035f, 8, Color{ 175, 185, 200, 255 });

        // Mastcam stereo optics head
        Vector3 headPos = Vector3Add(mastTop, Vector3{ 0.0f, 0.10f, 0.05f });
        DrawCube(headPos, 0.28f, 0.12f, 0.18f, Color{ 35, 42, 52, 255 });
        // Dual stereo camera lenses
        DrawSphere(Vector3Add(headPos, Vector3{ -0.08f, 0.0f, 0.10f }), 0.04f, Color{ 0, 200, 255, 255 });
        DrawSphere(Vector3Add(headPos, Vector3{ +0.08f, 0.0f, 0.10f }), 0.04f, Color{ 0, 200, 255, 255 });

        // Spinning LIDAR sensor dome on masthead
        Vector3 lidarPos = Vector3Add(mastTop, Vector3{ 0.0f, 0.22f, 0.0f });
        DrawCylinderEx(lidarPos, Vector3Add(lidarPos, Vector3{ 0.0f, 0.08f, 0.0f }), 0.09f, 0.09f, 10, Color{ 20, 25, 32, 255 });
        // Glowing laser emitter slit rotating with scene time
        float lidarRot = sceneTime * 8.0f;
        Vector3 lidarPulse = Vector3Add(lidarPos, Vector3{ sinf(lidarRot) * 0.10f, 0.04f, cosf(lidarRot) * 0.10f });
        DrawSphere(lidarPulse, 0.025f, Color{ 255, 40, 40, 255 });

        // G. Dual Navigation Headlights (projecting forward warm beam)
        Vector3 lightL = { -0.45f, -0.05f, hullSize.z * 0.51f };
        Vector3 lightR = { +0.45f, -0.05f, hullSize.z * 0.51f };
        DrawSphere(lightL, 0.06f, Color{ 255, 240, 180, 255 });
        DrawSphere(lightR, 0.06f, Color{ 255, 240, 180, 255 });
    }
    rlPopMatrix();

    // -----------------------------------------------------------------
    // 2. SUSPENSION LINKAGES & 4 CLEATED WHEELS
    // -----------------------------------------------------------------
    for (int i = 0; i < 4; ++i) {
        const WheelState& w = m_wheels[i];

        // Draw physical suspension strut/damper piston from mount to wheel hub
        DrawCylinderEx(w.worldMountPos, w.worldWheelPos, 0.045f, 0.035f, 6, Color{ 85, 95, 110, 255 });
        // Spring coil visual sleeve
        Vector3 midStrut = Vector3Lerp(w.worldMountPos, w.worldWheelPos, 0.5f);
        DrawSphere(midStrut, 0.08f, Color{ 190, 130, 40, 255 });

        // Transform into individual wheel hub coordinate frame
        // Wheel heading combines chassis rotation + wheel steering angle
        Quaternion steerQuat = QuaternionFromAxisAngle(m_up, w.steerAngle);
        Quaternion totalWheelRot = QuaternionMultiply(steerQuat, m_rotation);

        Matrix matWheelRot = QuaternionToMatrix(totalWheelRot);
        Matrix matWheelTrans = MatrixTranslate(w.worldWheelPos.x, w.worldWheelPos.y, w.worldWheelPos.z);
        Matrix matWheelWorld = MatrixMultiply(matWheelRot, matWheelTrans);

        rlPushMatrix();
        rlMultMatrixf(MatrixToFloat(matWheelWorld));
        {
            // Spin rotation around wheel axle (local X axis)
            rlRotatef(w.spinAngle * RAD2DEG, 1.0f, 0.0f, 0.0f);

            // Wheel cylinder is aligned along local X axis
            // Draw main cleated tire body (outer cylinder)
            Vector3 axleStart = { -m_wheelWidth * 0.5f, 0.0f, 0.0f };
            Vector3 axleEnd   = { +m_wheelWidth * 0.5f, 0.0f, 0.0f };
            Color tireCol = w.tcsActive ? Color{ 230, 130, 40, 255 } : Color{ 42, 46, 54, 255 };

            DrawCylinderEx(axleStart, axleEnd, m_wheelRadius, m_wheelRadius, 14, tireCol);
            DrawCylinderWiresEx(axleStart, axleEnd, m_wheelRadius, m_wheelRadius, 14, Color{ 80, 90, 105, 255 });

            // Machined titanium center hubcap
            DrawCylinderEx(Vector3{ -m_wheelWidth * 0.52f, 0.0f, 0.0f }, 
                           Vector3{ +m_wheelWidth * 0.52f, 0.0f, 0.0f }, 
                           m_wheelRadius * 0.45f, m_wheelRadius * 0.45f, 8, Color{ 160, 175, 195, 255 });

            // Raised chevron traction grousers / cleats (6 around the circumference)
            for (int c = 0; c < 6; ++c) {
                float cleatAngle = c * (2.0f * PI / 6.0f);
                float cy = cosf(cleatAngle) * (m_wheelRadius * 0.98f);
                float cz = sinf(cleatAngle) * (m_wheelRadius * 0.98f);
                Vector3 cleatPos = { 0.0f, cy, cz };
                DrawCube(cleatPos, m_wheelWidth * 0.92f, 0.04f, 0.06f, Color{ 120, 135, 155, 255 });
            }
        }
        rlPopMatrix();

        // If tire contact patch is grounded, draw contact footprint ring
        if (w.isGrounded) {
            Color ringCol = w.tcsActive ? Color{ 255, 165, 0, 200 } : Color{ 46, 204, 113, 140 };
            DrawCircle3D(w.contactPoint, 0.28f, w.contactNormal, 90.0f, ringCol);
        }
    }
}
