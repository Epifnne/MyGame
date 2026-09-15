#pragma once

#include <glm/glm.hpp>

namespace Runtime {
namespace Graphics {

enum class LightType {
    Directional,
    Point,
    Spot
};

struct DirectionalLight {
    glm::vec3 direction = glm::normalize(glm::vec3(-1.0f, -1.0f, 0.5f));
    glm::vec3 color = glm::vec3(1.0f);
    float intensity = 1.0f;
    bool castShadows = true;
};

} // namespace Graphics
} // namespace Runtime
