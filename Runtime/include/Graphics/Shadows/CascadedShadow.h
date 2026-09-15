#pragma once

#include <array>
#include <glm/glm.hpp>
#include "Graphics/Shadows/ShadowTypes.h"

namespace Runtime {
namespace Graphics {

class CascadedShadow {
public:
    static void ComputeSplitDistances(
        float nearPlane,
        float farPlane,
        int cascadeCount,
        float distributionExponent,
        std::array<float, kMaxShadowCascades>& outSplitFar);

    static glm::mat4 BuildDirectionalLightViewProj(
        const glm::vec3& lightDirection,
        const glm::vec3& cascadeCenter,
        float cascadeRadius);

    static void StabilizeToTexelGrid(glm::mat4& inOutLightViewProj, float shadowMapResolution);
};

} // namespace Graphics
} // namespace Runtime
