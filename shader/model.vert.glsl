#version 330 core
layout (location = 0) in vec3 vPos;
layout (location = 1) in vec3 vNor;

uniform mat4 uModel;
uniform mat4 uProjView;
uniform vec3 uLightPos;

out vec3 nor;
out vec3 lightDir;

void main()
{
    vec4 modelPos = uModel * vec4(vPos, 1.0);
    lightDir = uLightPos - vec3(modelPos);

    mat3 uModelInvTr = mat3(transpose(inverse(uModel)));
    nor = normalize(uModelInvTr * vNor);

    gl_Position = uProjView * modelPos;
}