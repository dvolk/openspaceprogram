# Open Space Program

Open source space sim

Written in C++ with SDL3, GLM, ImGui, Bullet physics, AssImp.

Code licence: GPL 3
Content licence: CC-BY-SA 3.0

Some textures and code are from Pioneer space sim: https://github.com/pioneerspacesim/pioneer

Lately, some of the code is written by qwen code.

<img src="https://i.imgur.com/HM02Gd7.png"/>
<img src="https://i.imgur.com/eKhFz34.png"/>
<img src="https://i.imgur.com/1xzE4Fo.png"/>

## Build from source/run

    sudo apt-get install g++ cmake make libgl1-mesa-dev libx11-dev libxext-dev libxcursor-dev libxi-dev libxfixes-dev libxrandr-dev libxrender-dev libxss-dev --no-install-recommends

    git clone https://github.com/dvolk/openspaceprogram
    cd openspaceprogram
    ./bootstrap.sh
    make

`make` builds the release binary into `build/linux-v2-znver3/release/osp`;
`./osp` is a symlink to it, so start OSP with

    ./osp

Other configs: `make debug`, `make asan`, `make tsan` (each builds into its
own directory and re-points `./osp` at the result). `make test` runs the
unit tests; `make e2e` the e2e battery (Xvfb for headless machines).

## Cli options

OSP comes with lots of cli options. eg:

    ./osp --body Laythe --scenario rot-orbit --ship res/ships/stager.json

Use`--help` to see all.
