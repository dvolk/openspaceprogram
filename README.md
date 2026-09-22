# Open Space Program

Open source space sim inspired by Pioneer and Kerbal Space Program.

Written in C++ using SDL3, GLM, ImGui, and Bullet physics.

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
