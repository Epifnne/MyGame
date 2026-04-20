#pragma once

#include "Graphics/RenderPipeline.h"
#include <memory>
#include <string>

namespace Runtime { 
namespace Graphics {
    
class ShaderManager;
class Shader;

class SimplePipeline : public RenderPipeline {
public:
    explicit SimplePipeline(const std::string& shaderName);
    ~SimplePipeline() override = default;

    bool Initialize(ShaderManager* shaderMgr) override;
    void Execute() override;

private:
    std::string m_shaderName;
    std::shared_ptr<Shader> m_shader;
};

} // namespace Graphics
} // namespace Runtime
