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

## building on Ubuntu 16.04+

<!-- TODO can we build bullet with make? -->
<!-- TODO maybe vendor SDL, assimp, glew? -->

    sudo apt-get install g++ libsdl2-dev libsdl2-image-dev libassimp-dev libglew-dev --no-install-recommends

    git clone --recurse-submodules https://github.com/dvolk/openspaceprogram

    cd openspaceprogram
    mkdir obj/
    cd middleware/bullet3
    mkdir build
    cd build
    cmake -DUSE_DOUBLE_PRECISION=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ..
    make
    cd ..
    ln -s src bullet

    cd ..

finally

    make

<!-- TODO some docs for cli options -->

start osp with

    ./osp
