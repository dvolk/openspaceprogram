#version 450

in vec3 texcoord0;

out vec4 fragColor;

uniform samplerCube skybox;

void main()
{    
    fragColor = texture(skybox, texcoord0);
}