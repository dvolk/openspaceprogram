# Open Space Program

Open source space simulator

Written in C++ with SDL2, GLM, ImGui, Bullet physics, AssImp.

Code licence: GPL 3
Content licence: CC-BY-SA 3.0

Some textures and code are from Pioneer space sim: https://github.com/pioneerspacesim/pioneer

<img src="https://i.imgur.com/HM02Gd7.png"/>
<img src="https://i.imgur.com/eKhFz34.png"/>
<img src="https://i.imgur.com/1xzE4Fo.png"/>

## building on Ubuntu 16.04+

    sudo apt-get install g++ libsdl2-dev libsdl2-image-dev libassimp-dev libglew-dev --no-install-recommends
    
    git clone --recurse-submodules https://github.com/dvolk/openspaceprogram
    
    cd openspaceprogram
    mkdir obj/
    cd middleware/bullet3
    mkdir build
    cd build
    cmake -DUSE_DOUBLE_PRECISION=ON ..
    make
    cd ..
    ln -s src bullet

    cd ..

(imgui is a submodule and is built by the top-level Makefile, no manual steps)

finally

    make
    
start osp with

    ./osp

