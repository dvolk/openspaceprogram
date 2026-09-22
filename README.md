# Open Space Program

Open source space sim

Written in C++ with SDL3, GLM, ImGui, Bullet physics, AssImp.

Code licence: AGPL-3.0
Content licence: CC-BY-SA 3.0

Lately, some of the code is written by qwen code.

<img src="https://i.imgur.com/HM02Gd7.png"/>
<img src="https://i.imgur.com/eKhFz34.png"/>
<img src="https://i.imgur.com/1xzE4Fo.png"/>

## Build from source/run

    sudo apt-get install g++ cmake make curl libgl1-mesa-dev libx11-dev libxext-dev libxcursor-dev libxi-dev libxfixes-dev libxrandr-dev libxrender-dev libxss-dev --no-install-recommends

    git clone https://github.com/dvolk/openspaceprogram
    cd openspaceprogram
    ./bootstrap.sh
    make

then start osp with:

    ./osp
