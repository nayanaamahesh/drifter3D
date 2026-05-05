#version 330 core
in vec3  vColor;
in vec3  vNormal;
in vec3  vFragPos;
in vec4  vFragPosLightSpace;

out vec4 FragColor;

uniform vec3  uColor;
uniform vec3  uMaterialKd;
uniform int   uUseMaterial;
uniform float uAlpha;

uniform sampler2D uShadowMap;
uniform vec3      uSunDir;       // normalised, points TOWARD sun
uniform vec3      uSunColor;     // e.g. (1.0, 0.95, 0.80)
uniform float     uAmbient;      // e.g. 0.25

float shadowFactor(vec4 fragPosLS)
{
    
    vec3 proj = fragPosLS.xyz / fragPosLS.w;
    // Map [-1,1] ? [0,1]
    proj = proj * 0.5 + 0.5;

    // no shadow 
    if (proj.z > 1.0) return 1.0;

    float closestDepth = texture(uShadowMap, proj.xy).r;
    float currentDepth = proj.z;

    // Bias
    float cosTheta = max(dot(normalize(vNormal), uSunDir), 0.0);
    float bias = mix(0.005, 0.0005, cosTheta);

    // PCF — 3×3 kernel for soft edges
    float shadow = 0.0;
    vec2 texelSize = 1.0 / textureSize(uShadowMap, 0);
    for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y)
        {
            float pcfDepth = texture(uShadowMap,
                proj.xy + vec2(x, y) * texelSize).r;
            shadow += (currentDepth - bias > pcfDepth) ? 0.0 : 1.0;
        }
    shadow /= 9.0;
    return shadow;
}

void main()
{
    vec3 baseColor = (uUseMaterial == 1) ? uMaterialKd : uColor;

    vec3  N    = normalize(vNormal);
    float diff = max(dot(N, uSunDir), 0.0);

    float lit  = shadowFactor(vFragPosLightSpace);

    // Combine: ambient always on, diffuse only where not in shadow
    vec3 finalColor = baseColor * (uAmbient + (1.0 - uAmbient) * diff * lit * uSunColor);

    FragColor = vec4(finalColor, uAlpha);
}