#pragma once

#include <glm/glm.hpp>

namespace Runtime { 
namespace Graphics {

class Camera {
public:
    Camera();
    ~Camera() = default;

    void SetPerspective(float fovRadians, float aspect, float nearPlane, float farPlane);
    void SetOrthographic(float left, float right, float bottom, float top, float nearPlane, float farPlane);
    void SetPosition(const glm::vec3& pos);
    void SetTarget(const glm::vec3& target);

    glm::mat4 GetView() const;
    glm::mat4 GetProjection() const;
    glm::vec3 GetPosition() const { return m_position; }

private:
    glm::vec3 m_position{0.0f, 0.0f, 3.0f};
    glm::vec3 m_target{0.0f, 0.0f, 0.0f};
    glm::mat4 m_projection{1.0f};
};

} // namespace Graphics
} // namespace Runtime
