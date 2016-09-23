#version 120

varying vec3 texcoord0;

uniform samplerCube skybox;

void main()
{    
    gl_FragColor = textureCube(skybox, texcoord0);
}