#pragma once

#include <string>
#include <glad/gl.h>

namespace Runtime {
namespace Graphics {

class Shader {
public:
    Shader() = default;
    ~Shader();

    bool CompileFromSource(const std::string& vertexSrc, const std::string& fragmentSrc);
    void Use() const;
    GLuint GetProgram() const { return m_program; }

private:
    bool CompileShader(GLuint shader, const std::string& src);
    GLuint m_program = 0;
};

} // namespace Graphics
} // namespace Runtime
