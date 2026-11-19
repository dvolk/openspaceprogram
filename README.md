<h1><img src="https://raw.githubusercontent.com/dvolk/openspaceprogram/refs/heads/master/release/openspaceprogram.png" height=24> Open Space Program</h1>

Space sim inspired by [Pioneer](https://pioneerspacesim.net/) and Kerbal Space Program.

| <img src="https://oxfordfun.com/2026_09_23-18_19_23-ubbu-osp.png" alt="osp screenshot showing ship around phobos"> | <img src="https://oxfordfun.com/2026_09_23-00_19_57-ubbu-osp.png" alt="osp screenshot showing jupiter and its moons"> |
|------|------|
| <img src="https://oxfordfun.com/2026_09_23-18_53_42-ubbu-osp.png" alt="osp screenshot showing the VAB"> | <img src="https://oxfordfun.com/2026_09_24-00_32_48-ubbu-osp.png" alt="osp screenshot showing the space center menu"> |

## Build from source/run

    sudo apt-get install --no-install-recommends g++ cmake make curl pkg-config git \
                         libgl1-mesa-dev libx11-dev libxext-dev \
                         libxcursor-dev libxi-dev libxfixes-dev libxrandr-dev libxrender-dev libxss-dev \
                         libpulse-dev libasound2-dev

`libpulse-dev` and `libasound2-dev` are build deps, not runtime ones: bootstrap builds SDL3 with
`SDL_DEPS_SHARED=OFF`, so the PulseAudio and ALSA backends are compiled into `libSDL3.a`, and the Makefile
links them directly (`SDL3_SYS`, `Makefile:84`). The `-l` targets (`libpulse.so`, `libasound.so`) come only
from the `-dev` packages -- the runtime ones ship `.so.0`/`.so.2` -- so without them `make` stops at
`/usr/bin/ld: cannot find -lpulse`, while SDL's own configure merely says "Unable to find the alsa
development library" and carries on building SDL without that backend. The authoritative list is
`utils/debian/control`, which is what `make deb` builds against.

    git clone https://github.com/dvolk/openspaceprogram
    cd openspaceprogram
    ./bootstrap.sh
    make
    make test         # unit tests; `make` alone does not build them, `make tests` builds without running

then start osp with:

    ./osp
