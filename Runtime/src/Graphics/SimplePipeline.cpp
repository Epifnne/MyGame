#include "Graphics/SimplePipeline.h"
#include "Graphics/ShaderManager.h"
#include "Graphics/Shader.h"

namespace Runtime { namespace Graphics {

SimplePipeline::SimplePipeline(const std::string& shaderName)
    : m_shaderName(shaderName) {}

bool SimplePipeline::Initialize(ShaderManager* shaderMgr) {
    if (!shaderMgr) return false;
    m_shader = shaderMgr->Get(m_shaderName);
    return (m_shader != nullptr);
}

void SimplePipeline::Execute() {
    if (m_shader) m_shader->Use();
}

} }
