#include "Graphics/Mesh.h"
#include <cassert>

namespace Runtime { namespace Graphics {

Mesh::~Mesh() {
    Destroy();
}

bool Mesh::Create(const float* vertices, size_t vertexCount) {
    if (!vertices || vertexCount == 0) return false;
    // support either positions-only (3 floats) or interleaved pos+normal (6 floats)
    size_t strideFloats = 3;
    if (vertexCount % 6 == 0) strideFloats = 6;
    m_vertexCount = vertexCount / strideFloats;

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);

    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, vertexCount * sizeof(float), vertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, strideFloats * sizeof(float), (void*)0);
    if (strideFloats == 6) {
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, strideFloats * sizeof(float), (void*)(3 * sizeof(float)));
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    return true;
}

void Mesh::Destroy() {
    if (m_vbo) { glDeleteBuffers(1, &m_vbo); m_vbo = 0; }
    if (m_vao) { glDeleteVertexArrays(1, &m_vao); m_vao = 0; }
    m_vertexCount = 0;
}

void Mesh::Bind() const {
    glBindVertexArray(m_vao);
}

void Mesh::Unbind() const {
    glBindVertexArray(0);
}

void Mesh::Draw() const {
    if (m_vao == 0) return;
    glBindVertexArray(m_vao);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(m_vertexCount));
    glBindVertexArray(0);
}

} }
