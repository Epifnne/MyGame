#include "Graphics/ShaderManager.h"
#include "Graphics/Shader.h"

namespace Runtime { namespace Graphics {

std::shared_ptr<Shader> ShaderManager::CreateFromSource(const std::string& name, const std::string& vertSrc, const std::string& fragSrc) {
    auto it = m_shaders.find(name);
    if (it != m_shaders.end()) return it->second;

    auto sh = std::make_shared<Shader>();
    if (!sh->CompileFromSource(vertSrc, fragSrc)) return nullptr;
    m_shaders[name] = sh;
    return sh;
}

std::shared_ptr<Shader> ShaderManager::Get(const std::string& name) const {
    auto it = m_shaders.find(name);
    if (it == m_shaders.end()) return nullptr;
    return it->second;
}

} }
