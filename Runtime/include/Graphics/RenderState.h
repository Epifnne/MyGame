#pragma once

#include <glad/gl.h>

namespace Runtime { 
namespace Graphics {

struct RenderState {
    bool depthTest = true;
    bool depthWrite = true;
    bool blend = false;
    GLint blendSrc = GL_SRC_ALPHA;
    GLint blendDst = GL_ONE_MINUS_SRC_ALPHA;
    bool cullFace = true;
    GLint cullMode = GL_BACK; // GL_BACK or GL_FRONT
    bool colorWriteR = true;
    bool colorWriteG = true;
    bool colorWriteB = true;
    bool colorWriteA = true;
};

} // namespace Graphics
} // namespace Runtime
