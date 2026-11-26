# Clean-room build of the GitHub repository, mirroring .github/workflows/release.yml.
# The source is cloned inside the image, so the build context is just this directory
# and a dirty working tree here cannot leak in.
#
#   sudo docker build -t osp-build utils/docker
#   sudo docker run --rm osp-build                      # build + package, print dist/
#   sudo docker run --rm -v "$PWD/dist:/out" osp-build \
#        sh -c 'cp -a dist/. /out/'                     # keep the artifacts
#
# ARGs pick the commit (default: master) and bust the layer cache -- a cached
# clone does not notice that the branch moved:
#   --build-arg REF=v0.0.1
#   --build-arg CACHEBUST=$(date +%s)
#
# Stops on the way, for narrowing a failure: --target source (deps + clone),
# --target linux (native build), --target windows (cross build too).
# The default target is package: ./bootstrap.sh twice, make artifacts.
ARG REPO=https://github.com/dvolk/openspaceprogram.git
ARG REF=master
ARG CACHEBUST=0

FROM ubuntu:24.04 AS deps

ENV DEBIAN_FRONTEND=noninteractive

# The same list as the release workflow: the README's build deps plus zip/xz for
# the tarballs, dpkg-dev/debhelper for the .deb, and the mingw cross toolchain.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake make curl pkg-config git ca-certificates python3 \
        zip xz-utils dpkg-dev debhelper \
        libgl1-mesa-dev libglu1-mesa-dev libx11-dev libxext-dev libxcursor-dev libxi-dev \
        libxfixes-dev libxrandr-dev libxrender-dev libxss-dev \
        libpulse-dev libasound2-dev \
        g++-mingw-w64-x86-64 binutils-mingw-w64-x86-64 \
    && rm -rf /var/lib/apt/lists/*

FROM deps AS source
ARG REPO
ARG REF
ARG CACHEBUST
# Full history: the version counts commits from the root, and the tags it carries
# name its base. A shallow clone would misname the build.
RUN echo "$CACHEBUST" >/dev/null \
    && git clone "$REPO" /src/osp \
    && git -C /src/osp checkout "$REF" \
    && git -C /src/osp log -1 --oneline \
    && echo "commits: $(git -C /src/osp rev-list --count HEAD)"
WORKDIR /src/osp

FROM source AS linux
RUN ./bootstrap.sh && make

FROM linux AS windows
RUN OS=windows ./bootstrap.sh && make OS=windows all

FROM windows AS package
RUN make artifacts
CMD ["sh", "-c", "cd dist && sha256sum -c SHA256SUMS && ls -lh"]
