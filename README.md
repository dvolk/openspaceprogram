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

SDL2, SDL_image, GLEW, Bullet, AssImp, glm, imgui, implot are vendored in
`middleware/` (git submodules; GLEW is fetched as the official 2.2.0 source
tarball because its git repo ships only the code generator) -- `bootstrap.sh`
checks them out and builds them static. The system packages below are the
toolchain + the dev headers those builds need.

    sudo apt-get install g++ cmake make zlib1g-dev libpng-dev libgl1-mesa-dev \
        libx11-dev libxext-dev libxcursor-dev libxi-dev libxfixes-dev \
        libxrandr-dev libxrender-dev libxss-dev --no-install-recommends

    git clone https://github.com/dvolk/openspaceprogram
    cd openspaceprogram
    ./bootstrap.sh
    make

The build defaults to `-march=native` (code for the machine it's built on).
To target another ISA, set `MARCH` for *both* bootstrap and make (keep them
matching so the whole binary targets one ISA), e.g.
`MARCH=x86-64-v3 ./bootstrap.sh && MARCH=x86-64-v3 make`, or `MARCH=` for
plain x86-64. A binary built for a newer ISA won't run on older CPUs.
After changing `MARCH`, run `make clean` first (make can't detect a flag
change).

Release build (PGO, ~10% faster on heavy fleet scenarios):

    make clean && make PGO=gen        # instrumented build
    # the heavy scenario (tmp/pgo-fleet100.json: 100 heavy_asp in one
    # orbit) plus a couple of the usual launch/orbit/EVA scenarios:
    ./osp --fleet tmp/pgo-fleet100.json --terrain-px 32 --time-accel 10 --timeout 60
    make clean && make PGO=use        # optimized build from the profile

Profile data lives in `tmp/pgo`. After big code changes, repeat the cycle
(stale data just warns "no data for counter"). Plain `make` stays the
dev build (no PGO).

start OSP with

    ./osp

## Cli options

OSP comes with lots of cli options. eg:

    ./osp --body Laythe --scenario rot-orbit --ship res/ships/stager.json

Use`--help` to see all.
