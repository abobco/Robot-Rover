#version 300 es
precision mediump float;
out vec4 FragColor;
in vec3 vColor;

void main() { FragColor = vec4(vColor, 1.0); }