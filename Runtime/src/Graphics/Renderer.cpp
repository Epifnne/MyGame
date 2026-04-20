#include "Graphics/Renderer.h"
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include "Graphics/ShaderManager.h"
#include "Graphics/DefaultPipeline.h"
#include "Graphics/Shader.h"
#include "Graphics/Texture.h"
#include "Graphics/RenderState.h"
#include <iostream>

namespace Runtime {
namespace Graphics {

bool Renderer::Initialize() {
    if (!gladLoadGL((GLADloadfunc)glfwGetProcAddress)) {
        return false;
    }

    // Print GL context info for debugging shader issues
    const char* vendor = (const char*)glGetString(GL_VENDOR);
    const char* renderer = (const char*)glGetString(GL_RENDERER);
    const char* version = (const char*)glGetString(GL_VERSION);
    const char* glsl = (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION);
    if (vendor) std::cerr << "GL_VENDOR: " << vendor << std::endl;
    if (renderer) std::cerr << "GL_RENDERER: " << renderer << std::endl;
    if (version) std::cerr << "GL_VERSION: " << version << std::endl;
    if (glsl) std::cerr << "GLSL_VERSION: " << glsl << std::endl;

    // Reserve render queue
    m_renderQueue.reserve(256);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);

    glClearColor(m_clearColor.r, m_clearColor.g, m_clearColor.b, m_clearColor.a);
    // Initialize shader manager and default pipeline
    m_shaderManager = std::make_unique<Graphics::ShaderManager>();
    m_pipeline = std::make_unique<Graphics::DefaultPipeline>();
    if (!m_pipeline->Initialize(m_shaderManager.get())) {
        return false;
    }
    return true;
}

void Renderer::Shutdown() {
    // Release graphics subsystems
    m_pipeline.reset();
    m_shaderManager.reset();
}

    Renderer::~Renderer() {
        Shutdown();
    }

void Renderer::SetPipeline(std::unique_ptr<Graphics::RenderPipeline> pipeline) {
    if (!pipeline) return;
    pipeline->Initialize(m_shaderManager.get());
    m_pipeline = std::move(pipeline);
}

bool Renderer::LoadShaderFromSource(const std::string& name, const std::string& vertSource, const std::string& fragSource) {
    if (!m_shaderManager) return false;
    auto sh = m_shaderManager->CreateFromSource(name, vertSource, fragSource);
    return (sh != nullptr);
}

std::shared_ptr<Graphics::Shader> Renderer::GetShader(const std::string& name) const {
    if (!m_shaderManager) return nullptr;
    return m_shaderManager->Get(name);
}

void Renderer::Submit(const Graphics::Mesh& mesh, const Graphics::Material& material, const glm::mat4& model) {
    m_renderQueue.emplace_back(&mesh, &material, model);
}

void Renderer::Flush(const Graphics::Camera& camera) {
    if (m_renderQueue.empty()) return;

    // Sequentially process render queue and support multi-pass materials
    glm::mat4 vp = camera.GetProjection() * camera.GetView();
    glm::vec3 camPos = camera.GetPosition();

    for (auto& t : m_renderQueue) {
        const Graphics::Mesh* mesh = std::get<0>(t);
        const Graphics::Material* material = std::get<1>(t);
        glm::mat4 model = std::get<2>(t);

        size_t passCount = material->GetPassCount();
        for (size_t p = 0; p < passCount; ++p) {
            const Material::Pass* pass = material->GetPass(p);
            if (!pass || !pass->enabled || !pass->shader) continue;
            auto shader = pass->shader;
            GLuint prog = shader->GetProgram();
            if (prog == 0) continue;

            // Save previous GL state we will modify
            struct StateSnapshot {
                GLboolean depthTestEnabled;
                GLboolean depthMask;
                GLboolean blendEnabled;
                GLint prevBlendSrc;
                GLint prevBlendDst;
                GLboolean cullEnabled;
                GLint prevCullFaceMode;
                GLboolean colorMask[4];
            } snap{};

            glGetBooleanv(GL_DEPTH_TEST, &snap.depthTestEnabled);
            glGetBooleanv(GL_DEPTH_WRITEMASK, &snap.depthMask);
            glGetBooleanv(GL_BLEND, &snap.blendEnabled);
            glGetIntegerv(GL_BLEND_SRC_RGB, &snap.prevBlendSrc);
            glGetIntegerv(GL_BLEND_DST_RGB, &snap.prevBlendDst);
            glGetBooleanv(GL_CULL_FACE, &snap.cullEnabled);
            glGetIntegerv(GL_CULL_FACE_MODE, &snap.prevCullFaceMode);
            glGetBooleanv(GL_COLOR_WRITEMASK, snap.colorMask);

            // Apply pass render state
            const RenderState& rs = pass->state;
            if (rs.depthTest) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
            glDepthMask(rs.depthWrite ? GL_TRUE : GL_FALSE);
            if (rs.blend) { glEnable(GL_BLEND); glBlendFunc(rs.blendSrc, rs.blendDst); } else { glDisable(GL_BLEND); }
            if (rs.cullFace) { glEnable(GL_CULL_FACE); glCullFace(rs.cullMode); } else { glDisable(GL_CULL_FACE); }
            glColorMask(rs.colorWriteR ? GL_TRUE : GL_FALSE, rs.colorWriteG ? GL_TRUE : GL_FALSE, rs.colorWriteB ? GL_TRUE : GL_FALSE, rs.colorWriteA ? GL_TRUE : GL_FALSE);

            glUseProgram(prog);

            // set common uniforms
            GLint locMVP = glGetUniformLocation(prog, "u_ModelViewProj");
            if (locMVP >= 0) {
                glm::mat4 mvp = vp * model;
                glUniformMatrix4fv(locMVP, 1, GL_FALSE, &mvp[0][0]);
            }
            GLint locModel = glGetUniformLocation(prog, "u_Model");
            if (locModel >= 0) {
                glUniformMatrix4fv(locModel, 1, GL_FALSE, &model[0][0]);
            }
            GLint locCam = glGetUniformLocation(prog, "u_CamPos");
            if (locCam >= 0) {
                glUniform3fv(locCam, 1, &camPos[0]);
            }

            GLint locAlbedoSampler = glGetUniformLocation(prog, "u_AlbedoMap");
            if (locAlbedoSampler >= 0) {
                glUniform1i(locAlbedoSampler, 0);
            }
            GLint locNormalSampler = glGetUniformLocation(prog, "u_NormalMap");
            if (locNormalSampler >= 0) {
                glUniform1i(locNormalSampler, 1);
            }
            GLint locOrmSampler = glGetUniformLocation(prog, "u_ORMMap");
            if (locOrmSampler >= 0) {
                glUniform1i(locOrmSampler, 2);
            }

            const auto albedo = material->GetAlbedo();
            const auto normal = material->GetNormal();
            const auto orm = material->GetOrm();

            GLint locHasAlbedo = glGetUniformLocation(prog, "u_HasAlbedoMap");
            if (locHasAlbedo >= 0) {
                glUniform1i(locHasAlbedo, albedo ? 1 : 0);
            }
            GLint locHasNormal = glGetUniformLocation(prog, "u_HasNormalMap");
            if (locHasNormal >= 0) {
                glUniform1i(locHasNormal, normal ? 1 : 0);
            }
            GLint locHasOrm = glGetUniformLocation(prog, "u_HasORMMap");
            if (locHasOrm >= 0) {
                glUniform1i(locHasOrm, orm ? 1 : 0);
            }

            if (albedo) albedo->Bind(0);
            if (normal) normal->Bind(1);
            if (orm) orm->Bind(2);

            mesh->Draw();

            glActiveTexture(GL_TEXTURE2);
            glBindTexture(GL_TEXTURE_2D, 0);
            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_2D, 0);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, 0);

            glUseProgram(0);

            // Restore previous GL state
            if (snap.depthTestEnabled) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
            glDepthMask(snap.depthMask);
            if (snap.blendEnabled) { glEnable(GL_BLEND); glBlendFunc(snap.prevBlendSrc, snap.prevBlendDst); } else { glDisable(GL_BLEND); }
            if (snap.cullEnabled) { glEnable(GL_CULL_FACE); glCullFace(snap.prevCullFaceMode); } else { glDisable(GL_CULL_FACE); }
            glColorMask(snap.colorMask[0], snap.colorMask[1], snap.colorMask[2], snap.colorMask[3]);
        }
    }

    m_renderQueue.clear();
}

void Renderer::SetClearColor(const glm::vec4& color) {
    m_clearColor = color;
    glClearColor(color.r, color.g, color.b, color.a);
}

void Renderer::Clear() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::SetViewport(int x, int y, int width, int height) {
    glViewport(x, y, width, height);
}

} // namespace Graphics
} // namespace Runtime
