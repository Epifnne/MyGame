#pragma once

namespace Runtime { 
namespace Graphics {
    
class ShaderManager;

class RenderPipeline {
public:
    virtual ~RenderPipeline() = default;
    virtual bool Initialize(ShaderManager* shaderMgr) = 0;
    virtual void Execute() = 0;
};

} // namespace Graphics
} // namespace Runtime
