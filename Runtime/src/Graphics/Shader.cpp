#include "Graphics/Shader.h"
#include <iostream>

namespace Runtime {
namespace Graphics {

Shader::~Shader() {
    if (m_program) {
        glDeleteProgram(m_program);
        m_program = 0;
    }
}

bool Shader::CompileShader(GLuint shader, const std::string& src) {
    const char* cstr = src.c_str();
    glShaderSource(shader, 1, &cstr, nullptr);
    glCompileShader(shader);

    GLint compiled = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (!compiled) {
        GLint len = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);
        std::string log(len, '\0');
        glGetShaderInfoLog(shader, len, &len, &log[0]);
        std::cerr << "Shader compile error: " << log << std::endl;
        // Print the shader source to aid debugging
        std::cerr << "--- Shader source start ---\n" << src << "\n--- Shader source end ---\n";
        return false;
    }
    return true;
}

bool Shader::CompileFromSource(const std::string& vertexSrc, const std::string& fragmentSrc) {
    GLuint vert = glCreateShader(GL_VERTEX_SHADER);
    if (!CompileShader(vert, vertexSrc)) { glDeleteShader(vert); return false; }

    GLuint frag = glCreateShader(GL_FRAGMENT_SHADER);
    if (!CompileShader(frag, fragmentSrc)) { glDeleteShader(vert); glDeleteShader(frag); return false; }

    m_program = glCreateProgram();
    glAttachShader(m_program, vert);
    glAttachShader(m_program, frag);
    glLinkProgram(m_program);

    GLint linked = 0;
    glGetProgramiv(m_program, GL_LINK_STATUS, &linked);
    if (!linked) {
        GLint len = 0;
        glGetProgramiv(m_program, GL_INFO_LOG_LENGTH, &len);
        std::string log(len, '\0');
        glGetProgramInfoLog(m_program, len, &len, &log[0]);
        std::cerr << "Program link error: " << log << std::endl;
        // Print provided shader sources for debugging
        std::cerr << "--- Vertex shader source ---\n" << vertexSrc << "\n--- End vertex shader ---\n";
        std::cerr << "--- Fragment shader source ---\n" << fragmentSrc << "\n--- End fragment shader ---\n";
        glDeleteShader(vert);
        glDeleteShader(frag);
        glDeleteProgram(m_program);
        m_program = 0;
        return false;
    }

    glDetachShader(m_program, vert);
    glDetachShader(m_program, frag);
    glDeleteShader(vert);
    glDeleteShader(frag);

    return true;
}

void Shader::Use() const {
    if (m_program) glUseProgram(m_program);
}

} // namespace Graphics
} // namespace Runtime
