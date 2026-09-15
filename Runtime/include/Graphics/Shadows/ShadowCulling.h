#pragma once

#include <cstddef>
#include <vector>
#include <glm/glm.hpp>

namespace Runtime {
namespace Graphics {

struct ShadowCasterBounds {
    glm::vec3 center = glm::vec3(0.0f);
    float radius = 0.0f;
};

class ShadowCulling {
public:
    ShadowCulling() = default;
    ~ShadowCulling() = default;

    void CullCasters(
        const std::vector<ShadowCasterBounds>& casters,
        const glm::mat4& lightViewProj,
        std::vector<std::size_t>& outVisibleIndices) const;
};

} // namespace Graphics
} // namespace Runtime
