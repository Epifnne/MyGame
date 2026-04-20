#include "Graphics/Camera.h"
#include <glm/gtc/matrix_transform.hpp>

namespace Runtime { namespace Graphics {

Camera::Camera() {
    SetPerspective(glm::radians(45.0f), 4.0f/3.0f, 0.1f, 100.0f);
}

void Camera::SetPerspective(float fovRadians, float aspect, float nearPlane, float farPlane) {
    m_projection = glm::perspective(fovRadians, aspect, nearPlane, farPlane);
}

void Camera::SetOrthographic(float left, float right, float bottom, float top, float nearPlane, float farPlane) {
    m_projection = glm::ortho(left, right, bottom, top, nearPlane, farPlane);
}

void Camera::SetPosition(const glm::vec3& pos) {
    m_position = pos;
}

void Camera::SetTarget(const glm::vec3& target) {
    m_target = target;
}

glm::mat4 Camera::GetView() const {
    return glm::lookAt(m_position, m_target, glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::mat4 Camera::GetProjection() const {
    return m_projection;
}

} }
