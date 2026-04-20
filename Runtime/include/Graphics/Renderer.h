#pragma once

#include <glm/glm.hpp>
#include <memory>
#include "Graphics/ShaderManager.h"
#include "Graphics/RenderPipeline.h"
#include "Graphics/Mesh.h"
#include "Graphics/Material.h"
#include "Graphics/Camera.h"
#include <vector>
#include <tuple>
#include <memory>
#include <unordered_map>

namespace Runtime {
namespace Graphics {

class Renderer {
public:
    Renderer() = default;
    ~Renderer();

    bool Initialize();
    void Shutdown();

    // Set a custom render pipeline (takes ownership)
    void SetPipeline(std::unique_ptr<Graphics::RenderPipeline> pipeline);
    // Load shader source strings into ShaderManager and register by name
    bool LoadShaderFromSource(const std::string& name, const std::string& vertSource, const std::string& fragSource);
    // Get shader by name from internal ShaderManager
    std::shared_ptr<Graphics::Shader> GetShader(const std::string& name) const;

    // Submit a render command (mesh + material + model matrix)
    void Submit(const Graphics::Mesh& mesh, const Graphics::Material& material, const glm::mat4& model);
    // Flush queued commands using provided camera (does sorting/batching)
    void Flush(const Graphics::Camera& camera);

    void SetClearColor(const glm::vec4& color);
    void Clear();

    void SetViewport(int x, int y, int width, int height);
    
private:
    glm::vec4 m_clearColor = {0.1f, 0.1f, 0.1f, 1.0f};
    std::unique_ptr<Graphics::ShaderManager> m_shaderManager;
    std::unique_ptr<Graphics::RenderPipeline> m_pipeline;
    std::vector<std::tuple<const Graphics::Mesh*, const Graphics::Material*, glm::mat4>> m_renderQueue;
};

} // namespace Graphics
} // namespace Runtime
