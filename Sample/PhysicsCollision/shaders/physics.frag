#version 330 core
in vec3 vNormal;
out vec4 FragColor;
void main(){float l=0.25+0.75*max(dot(normalize(vNormal),normalize(vec3(0.4,1.0,0.3))),0.0);FragColor=vec4(vec3(0.2,0.65,0.9)*l,1.0);}
