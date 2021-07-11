#version 330 core

#define SPEC_POW 32

in vec3 Normal;
in vec3 FragPos;

out vec4 FragColor;

uniform vec3 objectColor;
uniform vec3 lightColor;
uniform vec3 lightPos;
uniform vec3 viewPos;

vec3 lightDir = vec3(1.0, 3.0, 1.0);

void main() {
  // ambient
  float ambientStrength = 0.35;
  vec3 ambient = ambientStrength * lightColor;

  // diffuse
  vec3 norm = normalize(Normal);
  // lightDir = normalize(lightPos - FragPos);
  lightDir = normalize(lightDir);
  float diff = max(dot(norm, lightDir), 0.0);
  vec3 diffuse = diff * lightColor * 0.65;

  // specular
  float specularStrength = 0.1;
  vec3 viewDir = normalize(viewPos - FragPos);
  vec3 reflectDir = reflect(-lightDir, norm);
  float spec = pow(max(dot(viewDir, reflectDir), 0.0), SPEC_POW);
  vec3 specular = specularStrength * spec * lightColor;

  vec3 result = (ambient + diffuse + specular) * objectColor;
  FragColor = vec4(result, 1.0);
}
