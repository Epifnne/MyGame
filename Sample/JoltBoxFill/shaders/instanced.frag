#version 330 core
in vec3 vNormal;
in vec3 vColor;
out vec4 FragColor;
void main() {
    float light = 0.30 + 0.70 * max(dot(normalize(vNormal), normalize(vec3(0.4, 1.0, 0.3))), 0.0);
    FragColor = vec4(vColor * light, 1.0);
}
