#pragma once

#include "Graphics/Camera.h"
#include "Graphics/Shadows/Light.h"
#include "Graphics/Shadows/ShadowAtlas.h"
#include "Graphics/Shadows/ShadowTypes.h"

namespace Runtime {
namespace Graphics {

class ShadowRenderer {
public:
    ShadowRenderer() = default;
    ~ShadowRenderer() = default;

    bool Initialize(const ShadowQualitySettings& quality);
    void Shutdown();

    bool BuildFrameConstants(
        const DirectionalLight& light,
        const Camera& camera,
        ShadowFrameConstants& outFrameConstants);

    const ShadowQualitySettings& GetQuality() const { return m_quality; }
    void SetQuality(const ShadowQualitySettings& quality) { m_quality = quality; }

    const ShadowAtlas& GetAtlas() const { return m_atlas; }
    ShadowAtlas& GetAtlas() { return m_atlas; }

private:
    ShadowQualitySettings m_quality;
    ShadowAtlas m_atlas;
};

} // namespace Graphics
} // namespace Runtime
