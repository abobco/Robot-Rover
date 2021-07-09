#version 300 es
layout (location = 0) in vec3 aPos;

uniform vec2 scale;
uniform vec2 position;
void main()
{
    gl_Position = vec4(position.x+aPos.x*scale.x, position.y+aPos.y*scale.y, aPos.z, 1.0);

}