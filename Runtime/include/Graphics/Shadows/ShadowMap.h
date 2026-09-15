#pragma once

#include <glad/gl.h>

namespace Runtime {
namespace Graphics {

class ShadowMap {
public:
    ShadowMap() = default;
    ~ShadowMap() = default;

    bool Initialize(int width, int height);
    void Shutdown();

    bool IsValid() const { return m_fbo != 0 && m_depthTexture != 0; }
    int GetWidth() const { return m_width; }
    int GetHeight() const { return m_height; }
    GLuint GetFBO() const { return m_fbo; }
    GLuint GetDepthTexture() const { return m_depthTexture; }

private:
    GLuint m_fbo = 0;
    GLuint m_depthTexture = 0;
    int m_width = 0;
    int m_height = 0;
};

} // namespace Graphics
} // namespace Runtime
