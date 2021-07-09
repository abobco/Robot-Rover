#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 1) uniform sampler2D texSampler;
layout(binding = 2) uniform UniformLighting {
  vec3 color;
  vec3 viewPos;
  vec3 pos;
}
light;

layout(location = 0) in vec3 fragColor;
layout(location = 1) in vec2 fragTexCoord;
layout(location = 2) in vec3 fragNormal;
layout(location = 3) in vec3 fragPos;

layout(location = 0) out vec4 outColor;

#define SPEC_POW 32
vec3 lightDir = vec3(1.0, 1.0, 3.0);
const float diffuse_strength = 1.0;

void main() {
  // ambient
  float ambientStrength = 0.1;
  vec3 ambient = ambientStrength * light.color;

  // diffuse
  vec3 norm = normalize(fragNormal);
  lightDir = normalize(light.pos - fragPos);
  //   lightDir = normalize(lightDir);
  float diff = max(dot(norm, lightDir), 0.0);
  vec3 diffuse = diff * light.color * diffuse_strength;

  // specular
  float specularStrength = 1.0;
  vec3 viewDir = normalize(light.viewPos - fragPos);
  vec3 reflectDir = reflect(-lightDir, norm);
  float spec = pow(max(dot(viewDir, reflectDir), 0.0), SPEC_POW);
  vec3 specular = specularStrength * spec * light.color;

  vec3 result = (ambient + diffuse + specular) *
                texture(texSampler, fragTexCoord * 2.0).rgb * fragColor;
  outColor = vec4(result, 1.0);
  //   outColor =
  //       vec4(abs(fragNormal) * texture(texSampler, fragTexCoord
  //       * 2.0).rgb, 1.0);

  //   outColor = vec4(fragNormal, 1.0);
}