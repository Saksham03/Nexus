#version 330 core
in vec3 nor;
in vec3 lightDir;

layout(location = 0) out vec4 fCol;

uniform vec3 color = vec3(0.7, 0.7, 0.6);
uniform float uAlpha = 1.0;

void main()
{
    //fCol = vec4(0.5 * (nor + 1), 1.0);
    fCol = vec4(color, 1.0);
} 