#pragma once

#include <vector>
#include <glad/gl.h>
#include "Graphics/Shadows/ShadowMap.h"

namespace Runtime {
namespace Graphics {

struct ShadowViewport {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
};

class ShadowAtlas {
public:
    ShadowAtlas() = default;
    ~ShadowAtlas() = default;

    bool Initialize(int cascadeCount, int resolutionPerCascade);
    void Shutdown();

    int GetCascadeCount() const { return m_cascadeCount; }
    int GetResolutionPerCascade() const { return m_resolutionPerCascade; }
    const ShadowMap& GetCascadeMap(int cascadeIndex) const;
    ShadowMap& GetCascadeMap(int cascadeIndex);
    ShadowViewport GetCascadeViewport(int cascadeIndex) const;

private:
    int m_cascadeCount = 0;
    int m_resolutionPerCascade = 0;
    std::vector<ShadowMap> m_cascadeMaps;
};

} // namespace Graphics
} // namespace Runtime
