#pragma once

#include <array>
#include <glm/glm.hpp>

namespace Runtime {
namespace Graphics {

constexpr int kMaxShadowCascades = 4;

struct ShadowCascadeSettings {
    float splitNear = 0.1f;
    float splitFar = 100.0f;
    float splitNearFadeRegion = 0.0f;
    float splitFarFadeRegion = 0.0f;
    float fadePlaneOffset = 0.0f;
    float fadePlaneLength = 0.0f;
    glm::mat4 lightViewProj = glm::mat4(1.0f);
    glm::vec4 atlasUvRect = glm::vec4(0.0f);
};

struct ShadowQualitySettings {
    int cascadeCount = 4;
    int shadowMapResolution = 2048;
    float shadowDistance = 40000.0f;
    float cascadeDistributionExponent = 3.0f;
    float cascadeTransitionFraction = 0.1f;
    float shadowDistanceFadeoutFraction = 0.1f;
    float constantBias = 0.0015f;
    float slopeBias = 2.0f;
    bool enablePCF = true;
    int pcfKernel = 3;
};

struct ShadowFrameConstants {
    int activeCascadeCount = 0;
    std::array<ShadowCascadeSettings, kMaxShadowCascades> cascades{};
};

} // namespace Graphics
} // namespace Runtime
