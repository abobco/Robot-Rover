#version 300 es
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aTexCoord0;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec2 v_texCoords;

void main() {
    v_texCoords = a_texCoord0;
    gl_Position = projection * view * model * vec4(aPos, 1.0f);
}