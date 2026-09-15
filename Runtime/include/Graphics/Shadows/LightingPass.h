#pragma once

#include <glad/gl.h>
#include "Graphics/Shadows/ShadowTypes.h"

namespace Runtime {
namespace Graphics {

class LightingPass {
public:
    LightingPass() = default;
    ~LightingPass() = default;

    void BindShadowTextures(GLuint firstTextureUnit, const ShadowFrameConstants& frameConstants) const;

    void ApplyShadowUniforms(
        GLuint program,
        const ShadowFrameConstants& frameConstants,
        const ShadowQualitySettings& quality) const;
};

} // namespace Graphics
} // namespace Runtime
