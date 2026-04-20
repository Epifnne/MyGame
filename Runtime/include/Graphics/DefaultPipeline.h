#pragma once

#include "Graphics/RenderPipeline.h"
#include <memory>

namespace Runtime { 
namespace Graphics {
    
class Shader;
class ShaderManager;

class DefaultPipeline : public RenderPipeline {
public:
    DefaultPipeline() = default;
    ~DefaultPipeline() = default;

    bool Initialize(ShaderManager* shaderMgr) override;
    void Execute() override;

private:
    std::shared_ptr<Shader> m_defaultShader;
};

} // namespace Graphics
} // namespace Runtime
