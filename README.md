# Open Space Program

Open source space simulator

Written in C++ with SDL2, GLM, ImGui, Bullet physics, AssImp.

Code licence: GPL 3
Content licence: CC-BY-SA 3.0

Some textures and code are from Pioneer space sim: https://github.com/pioneerspacesim/pioneer

Lately, some of the code is written by qwen code.

<img src="https://i.imgur.com/HM02Gd7.png"/>
<img src="https://i.imgur.com/eKhFz34.png"/>
<img src="https://i.imgur.com/1xzE4Fo.png"/>

## building

system deps (Ubuntu; the game still links SDL2 + GLEW + zlib from the
system -- bullet3, assimp and the header-only libs are vendored in
middleware/):

    sudo apt-get install g++ cmake make libsdl2-dev libsdl2-image-dev libglew-dev zlib1g-dev --no-install-recommends

    git clone --recurse-submodules https://github.com/dvolk/openspaceprogram
    cd openspaceprogram
    ./bootstrap.sh   # builds the vendored bullet3 + assimp into middleware/*/build
    make             # -> ./osp

<!-- TODO maybe vendor SDL, glew? -->

start osp with

    ./osp

<!-- TODO some docs for cli options -->
