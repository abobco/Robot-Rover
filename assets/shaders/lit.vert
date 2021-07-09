#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec2 inTexCoord;
layout(location = 3) in vec3 inNormal;

layout(location = 0) out vec3 fragColor;
layout(location = 1) out vec2 fragTexCoord;
layout(location = 2) out vec3 fragNormal;
layout(location = 3) out vec3 fragPos;

layout(binding = 0) uniform UniformBufferObject {
  mat4 view;
  mat4 proj;
}
ubo;

layout(push_constant) uniform PushConstants { mat4 model; }
constants;

void main() {
  gl_Position = ubo.proj * ubo.view * constants.model * vec4(inPosition, 1.0);
  fragColor = inColor;
  // fragColor = constants.color;
  fragTexCoord = inTexCoord;
  fragNormal = mat3(transpose(inverse(constants.model))) * inNormal;
  fragPos = vec3(constants.model * vec4(inPosition, 1.0));
}