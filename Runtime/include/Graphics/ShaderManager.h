#pragma once

#include <string>
#include <memory>
#include <unordered_map>

namespace Runtime { 
namespace Graphics {

class Shader;

class ShaderManager {
public:
    ShaderManager() = default;
    ~ShaderManager() = default;

    std::shared_ptr<Shader> CreateFromSource(const std::string& name, const std::string& vertSrc, const std::string& fragSrc);
    std::shared_ptr<Shader> Get(const std::string& name) const;

private:
    std::unordered_map<std::string, std::shared_ptr<Shader>> m_shaders;
};

} // namespace Graphics
} // namespace Runtime
