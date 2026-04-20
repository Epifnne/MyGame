#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

uniform mat4 u_ModelViewProj;
uniform mat4 u_Model;

out vec3 vNormal;
out vec3 vWorldPos;

void main() {
    vNormal = mat3(transpose(inverse(u_Model))) * aNormal;
    vWorldPos = (u_Model * vec4(aPos, 1.0)).xyz;
    gl_Position = u_ModelViewProj * vec4(aPos, 1.0);
}
