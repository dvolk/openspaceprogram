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

    sudo apt-get install g++ cmake make curl libgl1-mesa-dev libx11-dev libxext-dev libxcursor-dev libxi-dev libxfixes-dev libxrandr-dev libxrender-dev libxss-dev --no-install-recommends

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

## Windows

Cross-built from the same Linux box — no Windows toolchain needed:

    sudo apt-get install g++-mingw-w64-x86-64 wine64
    OS=windows ./bootstrap.sh   # the windows middleware (the plain ./bootstrap.sh is linux)
    make OS=windows

The result lands in `build/windows-v2-znver3/release/osp.exe` with all
libraries statically linked (including the C runtime); copy it to a
Windows machine and run it as-is. To try it without leaving the box:

    wine build/windows-v2-znver3/release/osp.exe --timeout 5

Linux-only bits (the Makefile refuses them with a message):

- `make OS=windows asan` — this mingw cross package has no
  AddressSanitizer runtime for the Windows target; use the linux asan
  config, or validate on a real Windows box.
- `tsan` — no ThreadSanitizer runtime for mingw at all.
- `make test` — unit tests run native (linux).
- `make e2e` — with the .exe it runs the battery under wine, serially
  (two concurrent software-GL instances are flaky).

## Cli options

OSP comes with lots of cli options. eg:

    ./osp --body Laythe --scenario rot-orbit --ship res/ships/stager.json

Use`--help` to see all.
