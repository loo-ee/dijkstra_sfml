#pragma once
#include <raylib.h>
#include <raymath.h>
#include <vector>
#include <string>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>

class PhysicsWorld;
struct Vertex3D;

enum class RoverCameraMode {
    ORBIT,
    CHASE,
    MAST
};

struct WheelState {
    Vector3 mountOffset;       // Local chassis mount coordinate
    Vector3 worldMountPos;     // Current world position of suspension top
    Vector3 worldWheelPos;     // Current visual wheel hub center
    Vector3 contactPoint;      // Surface contact point
    Vector3 contactNormal;     // Surface normal
    
    float suspensionLength;    // Current spring length
    float suspensionCompression;// Current compression in meters
    float spinAngle;           // Wheel rotational angle for visual rolling
    float angularVelocity;     // Wheel rotational speed (rad/s)
    float steerAngle;          // Steering deflection in radians (+ = right, - = left)
    float slipRatio;           // Dynamic slip ratio [0.0, 1.0+]
    
    bool isGrounded;           // True if tire contact patch hits terrain
    bool tcsActive;            // True if traction control intervened
};

class PlanetaryRover {
public:
    PlanetaryRover();
    ~PlanetaryRover();

    // Prevent accidental copying
    PlanetaryRover(const PlanetaryRover&) = delete;
    PlanetaryRover& operator=(const PlanetaryRover&) = delete;

    void init(PhysicsWorld& physics, Vector3 spawnPos, float yawAngleRad = 0.0f);
    void reset(PhysicsWorld& physics, Vector3 spawnPos, float yawAngleRad = 0.0f);
    void update(PhysicsWorld& physics, float dt);
    void render(float sceneTime) const;

    // Autonomous Pure Pursuit Waypoint Navigation
    void setPath(const std::vector<const Vertex3D*>& pathNodes);
    void setAutonomous(bool autoDrive) { m_isAutonomous = autoDrive; }
    bool isAutonomous() const { return m_isAutonomous; }
    bool hasReachedGoal() const { return m_hasReachedGoal; }
    void toggleAutonomous() { m_isAutonomous = !m_isAutonomous; }

    // Manual Drive Overrides (when autonomous is paused)
    void setThrottleInput(float throttle) { m_throttleInput = throttle; }
    void setSteeringInput(float steer) { m_steerInput = steer; }
    void setBrakeInput(float brake) { m_brakeInput = brake; }
    float getThrottle() const { return m_throttleInput; }
    float getSteering() const { return m_steerInput; }
    float getBrake() const { return m_brakeInput; }
    bool isInitialized() const { return !m_chassisBodyId.IsInvalid(); }

    // Camera Views
    void cycleCameraMode();
    void setCameraMode(RoverCameraMode mode) { m_cameraMode = mode; }
    RoverCameraMode getCameraMode() const { return m_cameraMode; }
    Camera3D getCamera(const Camera3D& orbitCamera) const;

    // Vehicle Telemetry Accessors
    Vector3 getPosition() const { return m_position; }
    Quaternion getRotation() const { return m_rotation; }
    Vector3 getForward() const { return m_forward; }
    Vector3 getRight() const { return m_right; }
    Vector3 getUp() const { return m_up; }
    
    float getSpeed() const { return m_speed; }                 // m/s
    float getSpeedKmH() const { return m_speed * 3.6f; }       // km/h
    float getPitchDeg() const { return m_pitchDeg; }
    float getRollDeg() const { return m_rollDeg; }
    float getYawDeg() const { return m_yawDeg; }
    bool isCriticalRollover() const { return m_isRolloverWarning; }

    float getOdometer() const { return m_odometerMeters; }
    float getBatteryExpendedKJ() const { return m_batteryJoules * 0.001f; }
    float getCurrentPowerKW() const { return m_currentPowerWatts * 0.001f; }
    
    // Pure Pursuit Metrics
    int getCurrentWaypointIndex() const { return m_currentWaypointIndex; }
    int getTotalWaypoints() const { return static_cast<int>(m_waypoints.size()); }
    Vector3 getCurrentTargetWaypoint() const { return m_targetWaypoint; }
    float getCrossTrackError() const { return m_crossTrackError; }
    float getLookaheadDistance() const { return m_lookaheadDist; }

    // Wheel States (FL, FR, RL, RR)
    const WheelState& getWheel(int idx) const { return m_wheels[idx]; }
    bool isTCSActive() const { return m_tcsEngagedOverall; }
    bool isHDCActive() const { return m_hdcActive; }
    bool isESPActive() const { return m_espActive; }
    bool isUnstuckActive() const { return m_unstuckActive; }

    // Instant recovery / self-righting
    void selfRight(PhysicsWorld& physics);

private:
    void updatePurePursuit(float dt);
    void updateSuspensionAndTires(PhysicsWorld& physics, float dt);
    void updateAttitudeAndSensors(float dt);

    JPH::BodyID m_chassisBodyId;
    Vector3 m_halfExtents;
    float m_mass;

    Vector3 m_position;
    Quaternion m_rotation;
    Vector3 m_forward;
    Vector3 m_right;
    Vector3 m_up;
    Vector3 m_linearVelocity;
    float m_speed;

    float m_pitchDeg;
    float m_rollDeg;
    float m_yawDeg;
    bool m_isRolloverWarning;

    float m_odometerMeters;
    float m_batteryJoules;
    float m_currentPowerWatts;

    // Suspension & 4 Wheels
    WheelState m_wheels[4];
    float m_suspensionRestLength;
    float m_springStiffness;
    float m_springDamping;
    float m_wheelRadius;
    float m_wheelWidth;
    float m_antiRollBarStiffness;

    // Controls & Active Safety Systems
    float m_throttleInput;
    float m_steerInput;
    float m_brakeInput;
    bool m_tcsEngagedOverall;
    bool m_hdcActive;        // Hill Descent Control (active braking on downhill)
    bool m_espActive;        // Electronic Stability Program (anti-rollover torque)
    bool m_unstuckActive;    // Anti-stuck wheel sweep & high-torque burst
    float m_stuckTimer;
    float m_unstuckTimer;

    // Autonomous Pure Pursuit State
    std::vector<Vector3> m_waypoints;
    int m_currentWaypointIndex;
    Vector3 m_targetWaypoint;
    float m_lookaheadDist;
    float m_crossTrackError;
    bool m_isAutonomous;
    bool m_hasReachedGoal;

    // Camera Mode
    RoverCameraMode m_cameraMode;
    mutable Vector3 m_chaseCamPos;
    mutable Vector3 m_chaseCamTarget;
};
