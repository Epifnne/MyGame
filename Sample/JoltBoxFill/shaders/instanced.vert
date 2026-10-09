#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=2) in mat4 aModel;
layout(location=6) in vec3 aColor;
uniform mat4 uViewProj;
out vec3 vNormal;
out vec3 vColor;
void main() {
    vNormal = mat3(aModel) * aNormal;
    vColor = aColor;
    gl_Position = uViewProj * aModel * vec4(aPos, 1.0);
}
