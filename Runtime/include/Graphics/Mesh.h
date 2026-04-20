#pragma once

#include <vector>
#include <glad/gl.h>

namespace Runtime { 
namespace Graphics {

class Mesh {
public:
    Mesh() = default;
    ~Mesh();

    // Create a simple static mesh from float vertex data (positions only)
    bool Create(const float* vertices, size_t vertexCount);
    void Destroy();

    void Bind() const;
    void Unbind() const;
    void Draw() const;

private:
    GLuint m_vao = 0;
    GLuint m_vbo = 0;
    size_t m_vertexCount = 0;
};

} // namespace Graphics
} // namespace Runtime
