#pragma once

#include <cstddef>
#include <string>
#include <glad/gl.h>

namespace Runtime { 
namespace Graphics {

class Texture {
public:
    Texture() = default;
    ~Texture();

    bool LoadFromMemory(const unsigned char* bytes, size_t length, bool flipVertically = true, bool useSrgb = false);
    void Bind(int unit = 0) const;
    void Unbind() const;

    GLuint GetHandle() const { return m_handle; }

private:
    GLuint m_handle = 0;
    int m_width = 0, m_height = 0, m_channels = 0;
};

} // namespace Graphics
} // namespace Runtime
