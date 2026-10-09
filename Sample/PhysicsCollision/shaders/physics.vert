#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
uniform mat4 u_Model;
uniform mat4 u_ModelViewProj;
out vec3 vNormal;
void main(){vNormal=mat3(transpose(inverse(u_Model)))*aNormal;gl_Position=u_ModelViewProj*vec4(aPos,1.0);}
