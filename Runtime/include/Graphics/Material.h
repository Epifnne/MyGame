#pragma once

#include <memory>
#include <string>
#include <vector>
#include "Graphics/RenderState.h"

namespace Runtime { 
namespace Graphics {
    
class Shader;
class Texture;

class Material {
public:
    struct Pass {
        std::shared_ptr<Shader> shader;
        RenderState state;
        std::string name;
        bool enabled = true;
    };

    Material() = default;
    ~Material() = default;

    // Replace all passes with a single main pass using given shader
    void SetShader(const std::shared_ptr<Shader>& shader) {
        m_passes.clear();
        if (shader) m_passes.push_back(Pass{shader, RenderState{}, "Main", true});
    }

    // Add a custom pass
    void AddPass(const Pass& pass) { m_passes.push_back(pass); }

    size_t GetPassCount() const { return m_passes.size(); }
    const Pass* GetPass(size_t i) const { return (i < m_passes.size()) ? &m_passes[i] : nullptr; }

    void SetAlbedo(const std::shared_ptr<Texture>& tex) { m_albedo = tex; }
    void SetNormal(const std::shared_ptr<Texture>& tex) { m_normal = tex; }
    void SetOrm(const std::shared_ptr<Texture>& tex) { m_orm = tex; }

    // Legacy: return shader of first pass
    std::shared_ptr<Shader> GetShader() const { return m_passes.empty() ? nullptr : m_passes[0].shader; }
    std::shared_ptr<Texture> GetAlbedo() const { return m_albedo; }
    std::shared_ptr<Texture> GetNormal() const { return m_normal; }
    std::shared_ptr<Texture> GetOrm() const { return m_orm; }

private:
    std::vector<Pass> m_passes;
    std::shared_ptr<Texture> m_albedo;
    std::shared_ptr<Texture> m_normal;
    std::shared_ptr<Texture> m_orm;
};

} // namespace Graphics
} // namespace Runtime
