#pragma once

#include <glm/glm.hpp>

namespace Game {
namespace Components {

struct CubeRenderComponent {
    glm::vec3 baseColor = glm::vec3(0.95f, 0.95f, 0.98f);
    float metallic = 1.0f;
    float roughness = 0.08f;
};

} // namespace Components
} // namespace Game
