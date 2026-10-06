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
    , m_suspensionRestLength(0.48f)
    , m_wheelRadius(0.40f)
    , m_wheelWidth(0.35f)
    , m_throttleInput(0.0f)
    , m_steerInput(0.0f)
    , m_brakeInput(0.0f)
    , m_tcsEngagedOverall(false)
    , m_hdcActive(false)
    , m_espActive(false)
    , m_unstuckActive(false)
    , m_stuckTimer(0.0f)
    , m_unstuckTimer(0.0f)
    , m_isReversing(false)
    , m_reverseTimer(0.0f)
    , m_reverseCooldown(0.0f)
    , m_replanRequested(false)
    , m_replanReason("")
    , m_hazardPos{ 0, 0, 0 }
    , m_replanCooldown(0.0f)
    , m_currentWaypointIndex(0)
    , m_targetWaypoint{ 0, 0, 0 }
    , m_lookaheadDist(4.0f)
    , m_crossTrackError(0.0f)
    , m_isAutonomous(false)
    , m_hasReachedGoal(false)
    , m_isPartialPath(false)
    , m_standoffDist(0.0f)
    , m_isAtStandoffVantage(false)
    , m_standoffTimer(0.0f)
    , m_isDirectHoming(false)
    , m_cameraMode(RoverCameraMode::ORBIT)
    , m_chaseCamPos{ 0, 10, -10 }
    , m_chaseCamTarget{ 0, 0, 0 }
{
    // Mount positions in local chassis coordinate system:
    // Local: +X = Right, +Y = Up, +Z = Forward
    m_wheels[0].mountOffset = Vector3{ -0.92f, -0.10f, +0.95f }; // Front-Left (FL)
    m_wheels[1].mountOffset = Vector3{ +0.92f, -0.10f, +0.95f }; // Front-Right (FR)
    m_wheels[2].mountOffset = Vector3{ -0.92f, -0.10f, -0.95f }; // Rear-Left (RL)
    m_wheels[3].mountOffset = Vector3{ +0.92f, -0.10f, -0.95f }; // Rear-Right (RR)

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

    // Raycast down to find exact terrain surface and slope normal under spawn point
    Vector3 hitGround = spawnPos;
    Vector3 groundNormal = Vector3{ 0.0f, 1.0f, 0.0f };
    Vector3 rayStart = Vector3{ spawnPos.x, spawnPos.y + 10.0f, spawnPos.z };
    Vector3 rayEnd   = Vector3{ spawnPos.x, spawnPos.y - 10.0f, spawnPos.z };
    physics.raycast(rayStart, rayEnd, &hitGround, &groundNormal);

    // Ride height: chassis half-extent Y = 0.35m, mount offset Y = -0.10m,
    // rest length = 0.48m, wheel radius = 0.40m, nominal sag = 0.15m.
    // Uncompressed clearance: ~0.88m above terrain along normal ensures no ground penetration
    Vector3 elevatedPos = Vector3Add(hitGround, Vector3Scale(groundNormal, 0.88f));

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
    m_hdcActive     = false;
    m_espActive     = false;
    m_unstuckActive = false;
    m_stuckTimer    = 0.0f;
    m_unstuckTimer  = 0.0f;
    m_isReversing   = false;
    m_reverseTimer  = 0.0f;
    m_reverseCooldown = 0.0f;
    m_replanRequested = false;
    m_replanReason  = "";
    m_hazardPos     = Vector3{ 0, 0, 0 };
    m_replanCooldown = 0.0f;
    m_hasReachedGoal = false;
    m_isPartialPath = false;
    m_standoffDist = 0.0f;
    m_isAtStandoffVantage = false;
    m_standoffTimer = 0.0f;
    m_isDirectHoming = false;
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

void PlanetaryRover::setPath(const std::vector<const Vertex3D*>& pathNodes, bool isPartial, float standoffDist, Vector3 finalGoalPos) {
    m_waypoints.clear();
    m_waypoints.reserve(pathNodes.size());
    for (const Vertex3D* node : pathNodes) {
        if (node) {
            m_waypoints.push_back(node->position);
        }
    }
    m_isPartialPath = isPartial;
    m_standoffDist = standoffDist;
    m_finalGoalPos = finalGoalPos;
    m_isAtStandoffVantage = false;
    m_standoffTimer = 0.0f;
    m_isDirectHoming = false;
    m_hasReachedGoal = false;

    // Intelligent Waypoint Progression on new/updated path:
    // If the rover is already moving, find the best forward waypoint along the rover's forward vector
    // to prevent the rover from turning around 180 deg to reach path[0] if path[0] is behind it.
    m_currentWaypointIndex = 0;
    if (m_waypoints.size() > 1 && Vector3LengthSqr(m_linearVelocity) > 0.05f) {
        float bestDist = 1e9f;
        int bestIdx = 0;
        for (size_t i = 0; i < std::min<size_t>(m_waypoints.size(), 5); ++i) {
            Vector3 toWp = Vector3Subtract(m_waypoints[i], m_position);
            float dotFwd = Vector3DotProduct(toWp, m_forward);
            float d = Vector2Distance(Vector2{ m_position.x, m_position.z }, 
                                      Vector2{ m_waypoints[i].x, m_waypoints[i].z });
            if (dotFwd > -0.2f && d < bestDist) {
                bestDist = d;
                bestIdx = static_cast<int>(i);
            }
        }
        m_currentWaypointIndex = bestIdx;
    }

    if (!m_waypoints.empty()) {
        m_targetWaypoint = m_waypoints[m_currentWaypointIndex];
    }
}

void PlanetaryRover::engageDirectHoming(Vector3 targetPos) {
    m_waypoints.clear();
    m_waypoints.push_back(targetPos);
    m_currentWaypointIndex = 0;
    m_targetWaypoint = targetPos;
    m_isPartialPath = false;
    m_standoffDist = 0.0f;
    m_isAtStandoffVantage = false;
    m_standoffTimer = 0.0f;
    m_isDirectHoming = true;
    m_hasReachedGoal = false;
    m_isAutonomous = true;
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
        updatePurePursuit(physics, dt);
    }

    // 4. Raycast suspension, tire dynamics, TCS, and drive forces
    updateSuspensionAndTires(physics, dt);

    // 5. Anti-Subsurface Floor Clamp: Guarantee chassis never sinks or clips beneath physical terrain
    Vector3 gNormal = Vector3{ 0, 1, 0 };
    Vector3 gHit = m_position;
    Vector3 rStart = Vector3{ m_position.x, m_position.y + 6.0f, m_position.z };
    Vector3 rEnd   = Vector3{ m_position.x, m_position.y - 6.0f, m_position.z };
    if (physics.raycast(rStart, rEnd, &gHit, &gNormal, m_chassisBodyId)) {
        float minSafeY = gHit.y + m_halfExtents.y + 0.12f;
        if (m_position.y < minSafeY) {
            m_position.y = minSafeY;
            physics.setBodyTransform(m_chassisBodyId, m_position, m_rotation);
            Vector3 vel = m_linearVelocity;
            if (vel.y < 0.0f) vel.y = 0.0f;
            physics.setBodyLinearVelocity(m_chassisBodyId, vel);
        }
    }
}

void PlanetaryRover::triggerReplan(const std::string& reason, Vector3 hazardPos) {
    if (m_replanCooldown <= 0.0f) {
        m_replanRequested = true;
        m_replanReason = reason;
        m_hazardPos = hazardPos;
        m_replanCooldown = 3.0f;
    }
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

void PlanetaryRover::updatePurePursuit(PhysicsWorld& physics, float dt) {
    if (m_replanCooldown > 0.0f) m_replanCooldown -= dt;
    if (m_reverseCooldown > 0.0f) m_reverseCooldown -= dt;

    if (m_waypoints.empty()) {
        m_throttleInput = 0.0f;
        m_brakeInput = 1.0f;
        m_isReversing = false;
        return;
    }

    // Check distance to goal (last waypoint)
    Vector3 goalPos = m_waypoints.back();
    float distToGoal = Vector2Distance(Vector2{ m_position.x, m_position.z }, Vector2{ goalPos.x, goalPos.z });

    if (distToGoal < 1.6f) {
        if (m_isPartialPath && !m_isDirectHoming) {
            m_isAtStandoffVantage = true;
            m_hasReachedGoal = false;
            m_standoffTimer += dt;

            // When reaching standoff vantage point, pause briefly (0.5s) to scan terrain,
            // then trigger an active detour replan and attempt safe drive towards final goal/nearest node
            if (m_standoffTimer >= 0.5f) {
                m_standoffTimer = 0.0f;
                triggerReplan("STANDOFF PROBING SAFE DETOUR", m_finalGoalPos);

                float distToFinal = Vector2Distance(Vector2{ m_position.x, m_position.z }, Vector2{ m_finalGoalPos.x, m_finalGoalPos.z });
                if (distToFinal > 1.8f) {
                    engageDirectHoming(m_finalGoalPos);
                    return;
                }
            }

            m_throttleInput = 0.0f;
            m_brakeInput = 0.6f;
            m_steerInput = 0.0f;
            m_isReversing = false;
            return;
        } else {
            m_hasReachedGoal = true;
            m_isAtStandoffVantage = false;
            m_standoffTimer = 0.0f;
            m_throttleInput = 0.0f;
            m_brakeInput = 1.0f;
            m_steerInput = 0.0f;
            m_isReversing = false;
            return;
        }
    } else {
        m_standoffTimer = 0.0f;
    }

    // -------------------------------------------------------------
    // Step 0: Real-Time Hazard & Blockade Detection
    // -------------------------------------------------------------
    bool hasBlockade = false;
    bool hasSteepUphill = false;
    Vector3 detectedHazardPos = { 0, 0, 0 };

    // A. Forward bumper obstacle raycast (detects boulders directly in path)
    Vector3 nosePos = Vector3Add(m_position, Vector3Scale(m_forward, m_halfExtents.z + 0.15f));
    nosePos.y += 0.12f;
    Vector3 bumperEnd = Vector3Add(nosePos, Vector3Scale(m_forward, 2.5f));
    bumperEnd.y -= 0.08f;

    Vector3 hitBumper, normBumper;
    if (physics.raycast(nosePos, bumperEnd, &hitBumper, &normBumper, m_chassisBodyId)) {
        float hitDist = Vector3Distance(nosePos, hitBumper);
        if (hitDist < 2.2f && normBumper.y < 0.38f) {
            hasBlockade = true;
            detectedHazardPos = hitBumper;
        }
    }

    // B. Terrain probe 2.6m ahead: check for impassable slope / flip hazard (>= 20 deg)
    Vector3 probeAhead = Vector3Add(m_position, Vector3Scale(m_forward, 2.6f));
    Vector3 probeHit, probeNormal;
    if (physics.raycast(Vector3{ probeAhead.x, probeAhead.y + 4.0f, probeAhead.z },
                        Vector3{ probeAhead.x, probeAhead.y - 4.0f, probeAhead.z },
                        &probeHit, &probeNormal, m_chassisBodyId)) {
        float deltaY = probeHit.y - m_position.y;
        float slopeAngleDeg = atan2f(deltaY, 2.6f) * RAD2DEG;
        float normalTiltDeg = acosf(Clamp(probeNormal.y, -1.0f, 1.0f)) * RAD2DEG;

        // Uphill slope >= 20 deg or ground surface normal tilt >= 20 deg threatens rollover/stall
        if (slopeAngleDeg >= 20.0f || (deltaY > 0.55f && normalTiltDeg >= 20.0f)) {
            hasSteepUphill = true;
            detectedHazardPos = probeHit;
        }
    }

    // C. Current pitch rollover hazard: climbing steep slope (> 18 deg) and stalled
    if (m_pitchDeg > 18.0f && fabsf(m_speed) < 0.15f && m_throttleInput > 0.25f) {
        hasSteepUphill = true;
        detectedHazardPos = Vector3Add(m_position, Vector3Scale(m_forward, 1.8f));
    }

    // -------------------------------------------------------------
    // Step 1: Autonomous Reverse Gear Maneuver
    // -------------------------------------------------------------
    if ((hasBlockade || hasSteepUphill) && !m_isReversing && m_reverseCooldown <= 0.0f) {
        m_isReversing = true;
        m_reverseTimer = 1.8f; // Reverse for 1.8 seconds (~2-3m back)
        m_reverseCooldown = 4.5f;
    }

    if (m_isReversing) {
        m_reverseTimer -= dt;
        m_throttleInput = -0.75f; // Active reverse gear
        m_brakeInput = 0.0f;
        m_steerInput = -0.30f;    // Counter-steer to pivot away from obstacle

        if (m_reverseTimer <= 0.0f) {
            m_isReversing = false;
            m_throttleInput = 0.0f;
            m_brakeInput = 1.0f;
            // Backed up safely into clear terrain: trigger route recalculation!
            triggerReplan(hasSteepUphill ? "UPHILL ROLLOVER RISK" : "OBSTACLE BLOCKADE", detectedHazardPos);
        }
        return;
    }

    // -------------------------------------------------------------
    // Step 2: Intelligent Waypoint Progression & Skipping Missed Nodes
    // -------------------------------------------------------------
    int totalWp = static_cast<int>(m_waypoints.size());
    while (m_currentWaypointIndex < totalWp - 1) {
        Vector3 curWp = m_waypoints[m_currentWaypointIndex];
        Vector3 nextWp = m_waypoints[m_currentWaypointIndex + 1];

        Vector2 curToRover = { m_position.x - curWp.x, m_position.z - curWp.z };
        Vector2 segVec = { nextWp.x - curWp.x, nextWp.z - curWp.z };
        float segLenSq = segVec.x * segVec.x + segVec.y * segVec.y;

        float distToCur = Vector2Length(curToRover);
        float distToNext = Vector2Distance(Vector2{ m_position.x, m_position.z }, Vector2{ nextWp.x, nextWp.z });

        bool shouldAdvance = false;

        // Condition A: Inside arrival tolerance
        if (distToCur < 3.2f) {
            shouldAdvance = true;
        }
        // Condition B: Rover has passed perpendicular plane of waypoint along trajectory segment
        else if (segLenSq > 0.01f) {
            float proj = (curToRover.x * segVec.x + curToRover.y * segVec.y) / segLenSq;
            // If forward progress along segment exceeds 60%, or rover is closer to next waypoint:
            if (proj > 0.60f || distToNext < distToCur * 0.85f) {
                shouldAdvance = true;
            }
        }

        if (shouldAdvance) {
            m_currentWaypointIndex++;
        } else {
            break;
        }
    }

    // Forward Proximity Shortcut: if displaced off-path and closer to an upcoming forward waypoint
    for (int j = m_currentWaypointIndex + 1; j < std::min(m_currentWaypointIndex + 4, totalWp); ++j) {
        Vector3 fw = m_waypoints[j];
        Vector3 toFw = Vector3Subtract(fw, m_position);
        float dotFwd = Vector3DotProduct(toFw, m_forward);
        float distFw = Vector2Distance(Vector2{ m_position.x, m_position.z }, Vector2{ fw.x, fw.z });
        float distCur = Vector2Distance(Vector2{ m_position.x, m_position.z }, 
                                        Vector2{ m_waypoints[m_currentWaypointIndex].x, m_waypoints[m_currentWaypointIndex].z });
        if (distFw < distCur && dotFwd > 0.5f) {
            m_currentWaypointIndex = j;
            break;
        }
    }

    // -------------------------------------------------------------
    // Step 3: Pure Pursuit Lookahead Target & Guidance
    // -------------------------------------------------------------
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

    // Compute target vector in local rover frame
    Vector3 toTarget = Vector3Subtract(m_targetWaypoint, m_position);
    float localX = Vector3DotProduct(toTarget, m_right);
    float localZ = Vector3DotProduct(toTarget, m_forward);
    m_crossTrackError = localX;

    // Off-path trigger: if cross track error is severe (> 6.5m), trigger replan
    if (fabsf(m_crossTrackError) > 6.5f && distToGoal > 8.0f) {
        triggerReplan("OFF-PATH DEVIATION", m_targetWaypoint);
    }

    // Prevent backtracking: if lookahead target is behind rover, advance index
    if (localZ < -1.0f && m_currentWaypointIndex < totalWp - 1) {
        m_currentWaypointIndex++;
        m_targetWaypoint = m_waypoints[m_currentWaypointIndex];
        toTarget = Vector3Subtract(m_targetWaypoint, m_position);
        localX = Vector3DotProduct(toTarget, m_right);
        localZ = Vector3DotProduct(toTarget, m_forward);
    }

    // Pure Pursuit Steering Curvature: kappa = 2*sin(alpha)/Ld
    float alpha = atan2f(localX, fmaxf(0.1f, localZ));
    const float wheelbase = 1.90f;
    float targetSteer = atan2f(2.0f * wheelbase * sinf(alpha), m_lookaheadDist);
    targetSteer = Clamp(targetSteer, -0.62f, 0.62f); // Max ~35 deg steering lock

    // Speed-dependent steering lock reduction
    float speedFactor = Clamp(fabsf(m_speed) / 5.0f, 0.0f, 1.0f);
    float maxSteerAtSpeed = Lerp(0.62f, 0.22f, speedFactor);
    targetSteer = Clamp(targetSteer, -maxSteerAtSpeed, maxSteerAtSpeed);

    // Smooth steering rate limiter
    float steerRate = 2.8f;
    m_steerInput += Clamp(targetSteer - m_steerInput, -steerRate * dt, steerRate * dt);

    // -------------------------------------------------------------
    // Step 4: Speed Regulation, Downhill HDC, & Anti-Stuck
    // -------------------------------------------------------------
    float cruiseSpeed = 3.8f; // ~13.7 km/h cruise

    // Turn slowdown
    float steerPenalty = 1.0f - 0.55f * (fabsf(m_steerInput) / 0.62f);
    cruiseSpeed *= steerPenalty;

    // Deceleration ramp as we near destination
    if (distToGoal < 7.0f) {
        cruiseSpeed *= fmaxf(0.2f, distToGoal / 7.0f);
    }

    // Downhill Speed Governor & Slope Adaptation
    m_hdcActive = false;
    if (m_pitchDeg < -2.0f) {
        float descentAngle = -m_pitchDeg;
        float descentFactor = Clamp(1.0f - (descentAngle / 28.0f) * 0.58f, 0.40f, 1.0f);
        cruiseSpeed *= descentFactor;
    }

    // Incline Assist: Generates dynamic torque boost when ascending slopes
    float inclineFactor = 1.0f;
    if (m_pitchDeg > 1.5f) {
        inclineFactor += 2.5f * fminf(1.0f, sinf(m_pitchDeg * DEG2RAD));
    }

    // Speed error control with Active Hill Descent Control (HDC)
    float speedError = cruiseSpeed - m_speed;
    if (speedError > 0.0f) {
        m_throttleInput = Clamp(speedError * 0.45f * inclineFactor, 0.20f, 1.0f);
        m_brakeInput = 0.0f;
    } else {
        m_throttleInput = 0.0f;
        float brakeGain = (m_pitchDeg < -2.0f) ? 0.95f : 0.65f;
        float downhillExtra = (m_pitchDeg < -2.0f) ? (-m_pitchDeg * 0.025f) : 0.0f;
        m_brakeInput = Clamp(-speedError * brakeGain + downhillExtra, 0.0f, 1.0f);
        if (m_pitchDeg < -2.5f || m_speed > 4.5f) {
            m_hdcActive = true;
        }
    }

    // Anti-Stuck Detection & Autonomous Recovery Routine
    if (m_throttleInput > 0.25f && fabsf(m_speed) < 0.18f && !m_hasReachedGoal) {
        m_stuckTimer += dt;
        if (m_stuckTimer > 1.2f) {
            m_unstuckActive = true;
            m_unstuckTimer += dt;
            // Wiggle steering left and right to gain traction on rocks
            float wiggle = sinf(m_unstuckTimer * 9.0f) * 0.48f;
            m_steerInput = Clamp(m_steerInput + wiggle, -0.62f, 0.62f);
            m_throttleInput = 1.0f; // Maximum torque burst
            m_brakeInput = 0.0f;
            // If stuck continues, initiate reverse gear
            if (m_unstuckTimer > 2.2f && !m_isReversing && m_reverseCooldown <= 0.0f) {
                m_isReversing = true;
                m_reverseTimer = 1.8f;
                m_reverseCooldown = 4.0f;
                m_stuckTimer = 0.0f;
                m_unstuckTimer = 0.0f;
                m_unstuckActive = false;
            }
        }
    } else if (fabsf(m_speed) > 0.35f) {
        m_stuckTimer = 0.0f;
        m_unstuckTimer = 0.0f;
        m_unstuckActive = false;
    }
}

void PlanetaryRover::updateSuspensionAndTires(PhysicsWorld& physics, float dt) {
    const float maxRayDist = m_suspensionRestLength + m_wheelRadius + 0.35f;
    m_tcsEngagedOverall = false;

    // Speed-dependent steering lock reduction for manual mode too
    float speedFactor = Clamp(fabsf(m_speed) / 5.0f, 0.0f, 1.0f);
    float maxSteerAtSpeed = Lerp(0.62f, 0.22f, speedFactor);
    float effectiveSteer = Clamp(m_steerInput, -maxSteerAtSpeed, maxSteerAtSpeed);

    // Apply front steering angles
    m_wheels[0].steerAngle = effectiveSteer; // FL
    m_wheels[1].steerAngle = effectiveSteer; // FR
    m_wheels[2].steerAngle = 0.0f;           // RL
    m_wheels[3].steerAngle = 0.0f;           // RR

    // Dynamically calculate corner weight and stiffness for current environment gravity
    float currentGravity = fabsf(physics.getGravity());
    if (currentGravity < 0.2f) currentGravity = 3.71f;

    // Corner static weight: F_c = (m * g) / 4
    float cornerWeight = (m_mass * currentGravity) * 0.25f;
    // Spring stiffness k tuned for ~0.15m natural sag under environment gravity
    float springStiffness = cornerWeight / 0.15f;
    // Near-critical damping: c = 2 * zeta * sqrt(k * m_corner) with zeta = 0.85
    float mCorner = m_mass * 0.25f;
    float springDamping = 2.0f * 0.85f * sqrtf(springStiffness * mCorner);
    // Anti-roll bar stiffness dynamically scaled to corner weight
    float arbStiffness = springStiffness * 0.65f;

    // Maximum upward suspension force clamp (never exceeds 2.2x corner weight!)
    // This strictly prevents the suspension from launching the rover into the air.
    float maxSpringForce = cornerWeight * 2.2f;

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
        bool grounded = physics.raycast(w.worldMountPos, rayEnd, &hitPoint, &hitNormal, m_chassisBodyId);
        w.isGrounded = grounded;

        if (grounded) {
            float hitDist = Vector3Distance(w.worldMountPos, hitPoint);
            w.contactPoint  = hitPoint;
            w.contactNormal = hitNormal;

            // Distance along suspension ray to place wheel hub so tire contacts ground
            float desiredHubDist = hitDist - m_wheelRadius;
            w.suspensionLength = Clamp(desiredHubDist, 0.12f, m_suspensionRestLength + 0.25f);
            w.suspensionCompression = Clamp(m_suspensionRestLength - w.suspensionLength, 0.0f, m_suspensionRestLength);
            w.worldWheelPos = Vector3Add(w.worldMountPos, Vector3Scale(rayDir, w.suspensionLength));

            // Spring-damper force calculation
            // Relative velocity of mount point along suspension ray
            Vector3 mountVel = physics.getPointVelocity(m_chassisBodyId, w.worldMountPos);
            float vRel = Vector3DotProduct(mountVel, rayDir); // positive when compressing

            float springForceMag = (springStiffness * w.suspensionCompression) + (springDamping * vRel);
            springForceMag = Clamp(springForceMag, 0.0f, maxSpringForce);

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
            // Use proper tire cornering stiffness (~6x corner weight) instead of suspension spring rate
            // to prevent excessive lateral force that generates rollover torque
            float muLateral = 0.75f;
            float maxLatForce = springForceMag * muLateral;
            float corneringStiffness = cornerWeight * 6.0f; // Realistic tire cornering coefficient
            float latGripMag = -vLat * corneringStiffness;
            latGripMag = Clamp(latGripMag, -maxLatForce, maxLatForce);
            Vector3 lateralForceVec = Vector3Scale(tireRight, latGripMag);
            // Apply lateral cornering force at wheel hub/axle to eliminate artificial rollover moment
            physics.applyForceAtPosition(m_chassisBodyId, lateralForceVec, w.worldWheelPos);

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

            // Motor Drive Force (All-Wheel Drive 4WD scaled with gravity & incline capability)
            const float maxMotorForcePerWheel = cornerWeight * 3.4f;
            float driveForceMag = m_throttleInput * maxMotorForcePerWheel * tcsFactor;

            // Braking Force
            if (m_brakeInput > 0.02f) {
                float brakeMag = -vLong * (cornerWeight * 4.0f) * m_brakeInput;
                brakeMag = Clamp(brakeMag, -springForceMag * 0.95f, springForceMag * 0.95f);
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
            // In the air (unloaded - reaches downward to seek terrain)
            w.suspensionLength = m_suspensionRestLength + 0.15f;
            w.suspensionCompression = 0.0f;
            w.worldWheelPos = Vector3Add(w.worldMountPos, Vector3Scale(rayDir, w.suspensionLength));
            w.contactPoint  = w.worldWheelPos;
            w.slipRatio = 0.0f;
            w.angularVelocity = Lerp(w.angularVelocity, 0.0f, 0.05f);
            w.spinAngle += w.angularVelocity * dt;
        }
    }

    // Step 2: Anti-Roll Bar Stabilization (Front axle: 0-1, Rear axle: 2-3)
    float diffFront = m_wheels[0].suspensionCompression - m_wheels[1].suspensionCompression;
    float arbForceF = diffFront * arbStiffness;
    arbForceF = Clamp(arbForceF, -cornerWeight * 0.75f, cornerWeight * 0.75f);
    // Correct sign: if Left (0) is compressed more than Right (1), diffFront > 0.
    // Left mount MUST be pushed UP (+m_up) and Right mount pulled DOWN (-m_up) to counteract roll!
    physics.applyForceAtPosition(m_chassisBodyId, Vector3Scale(m_up, +arbForceF), m_wheels[0].worldMountPos);
    physics.applyForceAtPosition(m_chassisBodyId, Vector3Scale(m_up, -arbForceF), m_wheels[1].worldMountPos);

    float diffRear = m_wheels[2].suspensionCompression - m_wheels[3].suspensionCompression;
    float arbForceR = diffRear * arbStiffness;
    arbForceR = Clamp(arbForceR, -cornerWeight * 0.75f, cornerWeight * 0.75f);
    physics.applyForceAtPosition(m_chassisBodyId, Vector3Scale(m_up, +arbForceR), m_wheels[2].worldMountPos);
    physics.applyForceAtPosition(m_chassisBodyId, Vector3Scale(m_up, -arbForceR), m_wheels[3].worldMountPos);

    // Step 3: Active Electronic Stability Program (ESP)
    // Progressive angular velocity damping to prevent turn-induced rollover
    m_espActive = false;
    Vector3 angVel = physics.getBodyAngularVelocity(m_chassisBodyId);
    float angSpeed = Vector3Length(angVel);
    if (angSpeed > 0.65f) {
        // Progressive damping: stronger as angular velocity increases
        float espGain = Clamp((angSpeed - 0.65f) / 1.5f, 0.0f, 1.0f);
        float dampCoeff = m_mass * (2.0f + espGain * 6.0f);
        Vector3 dampTorque = Vector3Scale(angVel, -dampCoeff);
        physics.applyTorque(m_chassisBodyId, dampTorque);
        m_espActive = true;
    }

    // Step 4: Active Dynamic Anti-Rollover Assist & Self-Righting
    // A. Active counter-torque when roll angle exceeds hazardous threshold (> 26 deg)
    if (fabsf(m_rollDeg) > 26.0f) {
        float rollRad = m_rollDeg * DEG2RAD;
        float assistGain = Clamp((fabsf(m_rollDeg) - 26.0f) / 15.0f, 0.0f, 1.0f);
        Vector3 rollRestore = Vector3Scale(m_forward, -sinf(rollRad) * (m_mass * 16.0f) * assistGain);
        physics.applyTorque(m_chassisBodyId, rollRestore);
        m_espActive = true;
    }

    // B. Critical Rollover Recovery: automatically rights rover if tipped onto side or upside down
    if (m_up.y < 0.40f) {
        Vector3 worldUp = Vector3{ 0.0f, 1.0f, 0.0f };
        Vector3 rightingTorque = Vector3CrossProduct(m_up, worldUp);
        float tLen = Vector3Length(rightingTorque);
        if (tLen > 1e-3f) {
            physics.applyTorque(m_chassisBodyId, Vector3Scale(rightingTorque, (m_mass * 35.0f) / tLen));
            m_espActive = true;
        }
    }

    // Step 5: Manual Mode Overspeed Governor (Intervenes if speed is unsafe down slope)
    if (!m_isAutonomous) {
        if (m_speed > 5.5f || (m_pitchDeg < -6.0f && m_speed > 3.8f)) {
            m_hdcActive = true;
            m_brakeInput = fmaxf(m_brakeInput, 0.85f);
        }
    }

    // Energy tracking
    m_currentPowerWatts = totalWorkRate;
    m_batteryJoules += totalWorkRate * dt;
}

void PlanetaryRover::selfRight(PhysicsWorld& physics) {
    if (m_chassisBodyId.IsInvalid()) return;
    Vector3 hitGround = m_position;
    Vector3 groundNormal = Vector3{ 0.0f, 1.0f, 0.0f };
    physics.raycast(Vector3{ m_position.x, m_position.y + 10.0f, m_position.z },
                    Vector3{ m_position.x, m_position.y - 10.0f, m_position.z },
                    &hitGround, &groundNormal, m_chassisBodyId);
    Vector3 rightedPos = Vector3Add(hitGround, Vector3Scale(groundNormal, 0.88f));
    float currentYaw = m_yawDeg * DEG2RAD;
    Quaternion uprightRot = QuaternionFromAxisAngle(Vector3{ 0, 1, 0 }, currentYaw);
    physics.setBodyTransform(m_chassisBodyId, rightedPos, uprightRot);
    physics.setBodyLinearVelocity(m_chassisBodyId, Vector3{ 0, 0, 0 });
    physics.setBodyAngularVelocity(m_chassisBodyId, Vector3{ 0, 0, 0 });
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
