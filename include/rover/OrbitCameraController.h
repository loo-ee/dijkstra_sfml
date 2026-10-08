#pragma once
#include <raylib.h>
#include <raymath.h>
#include <cmath>

inline Vector3 rotateVectorAroundAxis(Vector3 v, Vector3 axis, float angleRad) {
    float c = cosf(angleRad);
    float s = sinf(angleRad);
    Vector3 a = Vector3Normalize(axis);
    Vector3 cross = Vector3CrossProduct(a, v);
    float dot = Vector3DotProduct(a, v);
    return Vector3Add(
        Vector3Scale(v, c),
        Vector3Add(
            Vector3Scale(cross, s),
            Vector3Scale(a, dot * (1.0f - c))
        )
    );
}

class OrbitCameraController {
public:
    OrbitCameraController(Vector3 position = { 0.0f, 40.0f, 25.0f },
                          Vector3 target = { 0.0f, 0.0f, 0.0f },
                          float fovy = 45.0f)
    {
        m_camera.position = position;
        m_camera.target = target;
        m_camera.up = Vector3{ 0.0f, 1.0f, 0.0f };
        m_camera.fovy = fovy;
        m_camera.projection = CAMERA_PERSPECTIVE;
    }

    void update(bool isMouseOverUI = false, float (*getHeightFunc)(float, float) = nullptr, bool isDraggingLeft = false) {
        if (isMouseOverUI) return;

        Vector2 mouseDelta = GetMouseDelta();
        float wheel = GetMouseWheelMove();

        // 1. Zoom to Mouse Cursor (zooms directly into any terrain point under cursor)
        // Hard-locked between 4.0m (close inspection) and 75.0m (elevated scenic overview)
        if (wheel != 0.0f) {
            Vector3 camToTarget = Vector3Subtract(m_camera.target, m_camera.position);
            float dist = Vector3Length(camToTarget);

            if (dist > 0.001f) {
                Vector3 viewDir = Vector3Normalize(camToTarget);
                Vector2 mousePos = GetMousePosition();
                Ray mouseRay = GetMouseRay(mousePos, m_camera);

                // Find 3D intersection point on the plane passing through target perpendicular to view direction
                float denom = Vector3DotProduct(viewDir, mouseRay.direction);
                Vector3 focusPoint = m_camera.target;

                if (fabsf(denom) > 0.01f) {
                    float t = dist / denom;
                    if (t > 0.1f && t < 2000.0f) {
                        focusPoint = Vector3Add(mouseRay.position, Vector3Scale(mouseRay.direction, t));
                    }
                }

                // Smooth exponential zoom factor based on wheel delta
                float zoomFactor = powf(0.85f, wheel);
                zoomFactor = Clamp(zoomFactor, 0.4f, 2.5f);
                float newDist = Clamp(dist * zoomFactor, 4.0f, 75.0f);
                float actualScale = newDist / dist;

                m_camera.position = Vector3Add(focusPoint, Vector3Scale(Vector3Subtract(m_camera.position, focusPoint), actualScale));
                m_camera.target   = Vector3Add(focusPoint, Vector3Scale(Vector3Subtract(m_camera.target, focusPoint), actualScale));

                // Keep camera safely above target height
                if (m_camera.position.y < m_camera.target.y + 0.8f) {
                    m_camera.position.y = m_camera.target.y + 0.8f;
                }
            }
        }

        // 2. Pan / Ground Surface Translation:
        // Triggered by Middle-click drag, Shift+RMB drag, or Left-drag across terrain
        bool isPanning = IsMouseButtonDown(MOUSE_BUTTON_MIDDLE) || 
                         (IsKeyDown(KEY_LEFT_SHIFT) && IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) ||
                         isDraggingLeft;

        if (isPanning && (fabsf(mouseDelta.x) > 0.001f || fabsf(mouseDelta.y) > 0.001f)) {
            Vector3 forward = Vector3Subtract(m_camera.target, m_camera.position);
            forward.y = 0.0f;
            if (Vector3LengthSqr(forward) < 0.001f) forward = Vector3{ 0.0f, 0.0f, -1.0f };
            forward = Vector3Normalize(forward);
            Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, Vector3{ 0.0f, 1.0f, 0.0f }));

            float dist = Vector3Distance(m_camera.position, m_camera.target);
            float panSpeed = 0.0015f * dist;

            Vector3 translation = Vector3Add(
                Vector3Scale(right, -mouseDelta.x * panSpeed),
                Vector3Scale(forward, mouseDelta.y * panSpeed)
            );

            Vector3 newTarget = Vector3Add(m_camera.target, translation);
            Vector3 newPos = Vector3Add(m_camera.position, translation);

            // Anchor target to terrain elevation
            if (getHeightFunc) {
                float terrainY = getHeightFunc(newTarget.x, newTarget.z);
                float deltaY = terrainY - newTarget.y;
                newTarget.y = terrainY;
                newPos.y += deltaY;
            }

            if (newPos.y < newTarget.y + 1.2f) {
                newPos.y = newTarget.y + 1.2f;
            }

            m_camera.target = newTarget;
            m_camera.position = newPos;
        }
        // 3. Orbit Rotation: Right-click drag rotates pitch and yaw around target center
        else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector3 toCamera = Vector3Subtract(m_camera.position, m_camera.target);
            float radius = Vector3Length(toCamera);
            if (radius > 0.001f) {
                radius = Clamp(radius, 4.0f, 75.0f);
                float yaw = atan2f(toCamera.x, toCamera.z);
                float pitch = asinf(Clamp(toCamera.y / radius, -0.999f, 0.999f));

                const float rotSpeed = 0.005f;
                yaw -= mouseDelta.x * rotSpeed;
                pitch -= mouseDelta.y * rotSpeed;

                // Elevated pitch clamp between ~20 deg and ~88 deg prevents grazing angle into unrendered horizon
                const float minPitch = 20.0f * DEG2RAD;  // ~20 degrees above horizontal plane
                const float maxPitch = 88.0f * DEG2RAD;  // ~88 degrees overhead
                pitch = Clamp(pitch, minPitch, maxPitch);

                m_camera.position = Vector3Add(m_camera.target, Vector3{
                    radius * cosf(pitch) * sinf(yaw),
                    radius * sinf(pitch),
                    radius * cosf(pitch) * cosf(yaw)
                });

                // Extra safety: enforce camera Y is always above target Y
                if (m_camera.position.y < m_camera.target.y + 1.0f) {
                    m_camera.position.y = m_camera.target.y + 1.0f;
                }
            }
        }
    }

    Camera3D& getCamera() { return m_camera; }
    const Camera3D& getCamera() const { return m_camera; }

    void focusOn(Vector3 newTarget, float distance = 30.0f) {
        distance = Clamp(distance, 4.0f, 75.0f);
        Vector3 viewDir = Vector3Normalize(Vector3{ 0.0f, 0.85f, 0.53f }); // ~58 deg high-angle overview
        m_camera.target = newTarget;
        m_camera.position = Vector3Add(newTarget, Vector3Scale(viewDir, distance));
        if (m_camera.position.y < m_camera.target.y + 1.5f) {
            m_camera.position.y = m_camera.target.y + 1.5f;
        }
    }

    void reset(Vector3 position = { 0.0f, 40.0f, 25.0f }, Vector3 target = { 0.0f, 0.0f, 0.0f }) {
        m_camera.position = position;
        m_camera.target = target;
        m_camera.up = Vector3{ 0.0f, 1.0f, 0.0f };
    }

private:
    Camera3D m_camera = {};
};
