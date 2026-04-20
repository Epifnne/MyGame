#include "Graphics/DefaultPipeline.h"
#include "Graphics/ShaderManager.h"
#include "Graphics/Shader.h"

namespace Runtime { namespace Graphics {

static const char* s_simpleVertex = R"glsl(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 u_ModelViewProj;
void main() {
    gl_Position = u_ModelViewProj * vec4(aPos, 1.0);
}
)glsl";

static const char* s_simpleFragment = R"glsl(
#version 330 core
out vec4 FragColor;
void main() {
    FragColor = vec4(1.0, 0.8, 0.2, 1.0);
}
)glsl";

bool DefaultPipeline::Initialize(ShaderManager* shaderMgr) {
    if (!shaderMgr) return false;
    m_defaultShader = shaderMgr->CreateFromSource("default", s_simpleVertex, s_simpleFragment);
    return (m_defaultShader != nullptr);
}

void DefaultPipeline::Execute() {
    if (m_defaultShader) m_defaultShader->Use();
    // For now the pipeline only binds shader. Actual draw calls are issued elsewhere.
}

} }
