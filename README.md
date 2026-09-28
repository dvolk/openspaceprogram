<h1><img src="https://raw.githubusercontent.com/dvolk/openspaceprogram/refs/heads/master/release/openspaceprogram.png" height=24> Open Space Program</h1>

Space sim inspired by [Pioneer](https://pioneerspacesim.net/) and Kerbal Space Program.

| <img src="https://oxfordfun.com/2026_09_23-18_19_23-ubbu-osp.png" alt="osp screenshot showing ship around phobos"> | <img src="https://oxfordfun.com/2026_09_23-00_19_57-ubbu-osp.png" alt="osp screenshot showing jupiter and its moons"> |
|------|------|
| <img src="https://oxfordfun.com/2026_09_23-18_53_42-ubbu-osp.png" alt="osp screenshot showing the VAB"> | <img src="https://oxfordfun.com/2026_09_24-00_32_48-ubbu-osp.png" alt="osp screenshot showing the space center menu"> |

## Build from source/run

    sudo apt-get install --no-install-recommends g++ cmake make curl libgl1-mesa-dev libx11-dev libxext-dev \
                         libxcursor-dev libxi-dev libxfixes-dev libxrandr-dev libxrender-dev libxss-dev

    git clone https://github.com/dvolk/openspaceprogram
    cd openspaceprogram
    ./bootstrap.sh
    make

then start osp with:

    ./osp
