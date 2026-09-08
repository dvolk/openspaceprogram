# Open Space Program

Open source space sim

Written in C++ with SDL2, GLM, ImGui, Bullet physics, AssImp.

Code licence: GPL 3
Content licence: CC-BY-SA 3.0

Some textures and code are from Pioneer space sim: https://github.com/pioneerspacesim/pioneer

Lately, some of the code is written by qwen code.

<img src="https://i.imgur.com/HM02Gd7.png"/>
<img src="https://i.imgur.com/eKhFz34.png"/>
<img src="https://i.imgur.com/1xzE4Fo.png"/>

## Build from source/run

    sudo apt-get install g++ cmake make libsdl2-dev libsdl2-image-dev libglew-dev zlib1g-dev --no-install-recommends

    git clone https://github.com/dvolk/openspaceprogram
    cd openspaceprogram
    ./bootstrap.sh
    make

start OSP with

    ./osp

## Cli options

OSP comes with lots of cli options. eg:

    ./osp --body Laythe --scenario rot-orbit --ship res/ships/stager.json

Use`--help` to see all.
