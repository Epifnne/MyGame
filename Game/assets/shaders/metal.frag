#version 330 core
in vec3 vNormal;
in vec3 vWorldPos;
out vec4 FragColor;

uniform vec3 u_CamPos;
uniform sampler2D u_AlbedoMap;
uniform sampler2D u_NormalMap;
uniform sampler2D u_ORMMap;
uniform int u_HasAlbedoMap;
uniform int u_HasNormalMap;
uniform int u_HasORMMap;
const float PI = 3.14159265359;

vec3 safeNormalize(vec3 v) {
    float len = max(length(v), 1e-6);
    return v / len;
}

float DistributionGGX(vec3 N, vec3 H, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float NdotH = max(dot(N, H), 0.0);
    float NdotH2 = NdotH * NdotH;

    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    return a2 / (PI * denom * denom + 1e-7);
}

float GeometrySchlickGGX(float NdotV, float k) {
    return NdotV / (NdotV * (1.0 - k) + k + 1e-7);
}

float GeometrySmith(vec3 N, vec3 V, vec3 L, float k) {
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    float ggx1 = GeometrySchlickGGX(NdotV, k);
    float ggx2 = GeometrySchlickGGX(NdotL, k);
    return ggx1 * ggx2;
}

vec3 FresnelSchlick(float cosTheta, vec3 F0) {
    float c = clamp(cosTheta, 0.0, 1.0);
    float base = max(1.0 - c, 0.0);
    return F0 + (1.0 - F0) * pow(base, 5.0);
}

void main() {
    vec3 N = safeNormalize(vNormal);
    vec3 V = safeNormalize(u_CamPos - vWorldPos);
    vec2 uv = vWorldPos.xz * 0.35;

    vec3 albedo = vec3(0.95, 0.95, 0.98);
    if (u_HasAlbedoMap == 1) {
        albedo = texture(u_AlbedoMap, uv).rgb;
    }

    float metallic = 1.0;
    float roughness = 0.08;
    float ao = 1.0;
    if (u_HasORMMap == 1) {
        vec3 orm = texture(u_ORMMap, uv).rgb;
        ao = clamp(orm.r, 0.0, 1.0);
        roughness = clamp(orm.g, 0.04, 1.0);
        metallic = clamp(orm.b, 0.0, 1.0);
    }

    if (u_HasNormalMap == 1) {
        vec3 mapN = texture(u_NormalMap, uv).xyz * 2.0 - 1.0;
        vec3 up = (abs(N.y) < 0.999) ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
        vec3 T = safeNormalize(cross(up, N));
        vec3 B = safeNormalize(cross(N, T));
        mat3 tbn = mat3(T, B, N);
        N = safeNormalize(tbn * mapN);
    }

    vec3 lightDir = safeNormalize(vec3(-1.0, -1.0, 0.5));
    vec3 L = -lightDir;
    vec3 H = safeNormalize(V + L);

    vec3 backLightDir = safeNormalize(vec3(1.0, 0.5, -0.5));
    vec3 L2 = -backLightDir;
    vec3 H2 = safeNormalize(V + L2);

    vec3 F0 = mix(vec3(0.04), albedo, metallic);

    float D = DistributionGGX(N, H, roughness);
    float k = (roughness + 1.0);
    k = (k * k) / 8.0;
    float G = GeometrySmith(N, V, L, k);
    vec3 F = FresnelSchlick(max(dot(H, V), 0.0), F0);

    float D2 = DistributionGGX(N, H2, roughness);
    float G2 = GeometrySmith(N, V, L2, k);
    vec3 F2 = FresnelSchlick(max(dot(H2, V), 0.0), F0);

    vec3 numerator = D * G * F;
    float denom = max(4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0), 1e-4);
    vec3 specular = numerator / denom;

    vec3 numerator2 = D2 * G2 * F2;
    float denom2 = max(4.0 * max(dot(N, V), 0.0) * max(dot(N, L2), 0.0), 1e-4);
    vec3 specular2 = numerator2 / denom2;

    vec3 kS = F;
    vec3 kD = (1.0 - kS) * (1.0 - metallic);
    vec3 diffuse = kD * albedo / PI;

    vec3 kS2 = F2;
    vec3 kD2 = (1.0 - kS2) * (1.0 - metallic);
    vec3 diffuse2 = kD2 * albedo / PI;

    float NdotL = max(dot(N, L), 0.0);
    vec3 Lo = (diffuse + specular) * vec3(1.5) * NdotL;

    float NdotL2 = max(dot(N, L2), 0.0);
    vec3 Lo2 = (diffuse2 + specular2) * vec3(0.5) * NdotL2;

    vec3 ambient = vec3(0.15) * ao * albedo;

    vec3 color = ambient + Lo + Lo2;

    float fresnel = pow(1.0 - max(dot(N, V), 0.0), 3.0);
    color += vec3(0.2) * fresnel * albedo;

    color = color / (color + vec3(1.0));
    color = pow(max(color, 0.0), vec3(1.0 / 2.2));

    FragColor = vec4(color, 1.0);
}
