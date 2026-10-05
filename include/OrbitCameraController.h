#pragma once
#include <raylib.h>
#include <raymath.h>
#include <cmath>

class OrbitCameraController {
public:
    OrbitCameraController(Vector3 position = { 0.0f, 30.0f, 40.0f },
                          Vector3 target = { 0.0f, 0.0f, 0.0f },
                          float fovy = 45.0f)
    {
        m_camera.position = position;
        m_camera.target = target;
        m_camera.up = Vector3{ 0.0f, 1.0f, 0.0f };
        m_camera.fovy = fovy;
        m_camera.projection = CAMERA_PERSPECTIVE;
    }

    void update(bool isMouseOverUI = false) {
        if (isMouseOverUI) return;

        Vector2 mouseDelta = GetMouseDelta();
        float wheel = GetMouseWheelMove();

        // 1. Zoom to Mouse Cursor (zooms directly into any terrain point under cursor)
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
                zoomFactor = Clamp(zoomFactor, 0.3f, 3.0f);
                float newDist = dist * zoomFactor;

                if (newDist >= 3.0f && newDist <= 600.0f) {
                    // Scale position and target relative to the cursor focal point
                    // This keeps the 3D point under the mouse cursor stationary on the screen!
                    m_camera.position = Vector3Add(focusPoint, Vector3Scale(Vector3Subtract(m_camera.position, focusPoint), zoomFactor));
                    m_camera.target   = Vector3Add(focusPoint, Vector3Scale(Vector3Subtract(m_camera.target, focusPoint), zoomFactor));

                    // Keep camera safely above target height
                    if (m_camera.position.y < m_camera.target.y + 0.8f) {
                        m_camera.position.y = m_camera.target.y + 0.8f;
                    }
                }
            }
        }

        // 2. Pan: Middle-click drag or Shift+RMB shifts target and position in camera screen plane
        if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE) || 
            (IsKeyDown(KEY_LEFT_SHIFT) && IsMouseButtonDown(MOUSE_BUTTON_RIGHT))) {
            Vector3 forward = Vector3Normalize(Vector3Subtract(m_camera.target, m_camera.position));
            Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, m_camera.up));
            Vector3 screenUp = Vector3Normalize(Vector3CrossProduct(right, forward));

            float dist = Vector3Distance(m_camera.position, m_camera.target);
            float panSpeed = dist * 0.0015f;

            Vector3 panDelta = Vector3Add(
                Vector3Scale(right, -mouseDelta.x * panSpeed),
                Vector3Scale(screenUp, mouseDelta.y * panSpeed)
            );

            m_camera.target = Vector3Add(m_camera.target, panDelta);
            m_camera.position = Vector3Add(m_camera.position, panDelta);
        }
        // 3. Orbit Rotation: Right-click drag rotates pitch and yaw around target center
        else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector3 toCamera = Vector3Subtract(m_camera.position, m_camera.target);
            float radius = Vector3Length(toCamera);
            if (radius > 0.001f) {
                float yaw = atan2f(toCamera.x, toCamera.z);
                float pitch = asinf(Clamp(toCamera.y / radius, -0.999f, 0.999f));

                const float rotSpeed = 0.005f;
                yaw -= mouseDelta.x * rotSpeed;
                pitch -= mouseDelta.y * rotSpeed;

                // Clamp pitch between ~2.5 deg and ~88 deg so camera stays above terrain plane
                const float minPitch = 0.045f;           // ~2.5 degrees above horizontal plane
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

    void focusOn(Vector3 newTarget, float distance = 25.0f) {
        Vector3 toCam = Vector3Subtract(m_camera.position, m_camera.target);
        Vector3 viewDir = Vector3Normalize(toCam);
        if (Vector3Length(viewDir) < 0.1f) viewDir = Vector3{ 0.0f, 0.6f, 0.8f };

        m_camera.target = newTarget;
        m_camera.position = Vector3Add(newTarget, Vector3Scale(viewDir, distance));
        if (m_camera.position.y < m_camera.target.y + 1.0f) {
            m_camera.position.y = m_camera.target.y + 1.0f;
        }
    }

    void reset(Vector3 position = { 0.0f, 65.0f, 100.0f }, Vector3 target = { 0.0f, 0.0f, 0.0f }) {
        m_camera.position = position;
        m_camera.target = target;
        m_camera.up = Vector3{ 0.0f, 1.0f, 0.0f };
    }

private:
    Camera3D m_camera = {};
};
