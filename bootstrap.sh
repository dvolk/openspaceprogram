#!/usr/bin/env bash
# bootstrap.sh -- fetch and build the cmake-built middleware so `make` works:
#   bullet3  static libs, double precision (the game's physics precision)
#   assimp   static lib (assimp 6 defaults to SHARED, so force it off)
# The header-only submodules (glm, imgui, implot, CLI11, nlohmann) need no
# build step. Idempotent: the cmake steps are incremental, so re-running
# after a submodule update only rebuilds what changed.
set -euo pipefail
cd "$(dirname "$0")"

JOBS=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

for tool in g++ cmake make; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "error: $tool not found -- install the system deps (see README) and re-run" >&2
        exit 1
    fi
done

# check the submodules out at their pinned commits (a no-op if the clone
# already used --recurse-submodules)
git submodule update --init --recursive

# the game includes bullet3's headers as <bullet/...>, so bullet3/ has a
# symlink bullet -> src
ln -sfn src middleware/bullet3/bullet

echo "=== building bullet3 (static, double precision) ==="
# CMAKE_POLICY_VERSION_MINIMUM: bullet3 declares a pre-3.5 cmake policy,
# which cmake 4 rejects without this
# demos/extras/tests are not linked by the game, so keep them out
cmake -S middleware/bullet3 -B middleware/bullet3/build \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DUSE_DOUBLE_PRECISION=ON \
    -DBUILD_BULLET2_DEMOS=OFF -DBUILD_EXTRAS=OFF -DBUILD_UNIT_TESTS=OFF
cmake --build middleware/bullet3/build -j"$JOBS"

echo "=== building assimp (static, OBJ-only) ==="
# We only ever load .obj meshes, so build just the OBJ importer (and no
# exporters). The default all-importers build pulls in ~30 format loaders
# (FBX, glTF, STEP, IFC, ...) that add ~11 MB to the game binary.
cmake -S middleware/assimp -B middleware/assimp/build \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF \
    -DASSIMP_BUILD_TESTS=OFF -DASSIMP_BUILD_SAMPLES=OFF -DASSIMP_INSTALL=OFF \
    -DASSIMP_BUILD_ALL_IMPORTERS_BY_DEFAULT=OFF \
    -DASSIMP_BUILD_OBJ_IMPORTER=ON \
    -DASSIMP_BUILD_ALL_EXPORTERS_BY_DEFAULT=OFF
cmake --build middleware/assimp/build -j"$JOBS"

echo
echo "middleware ready. Now:  make   (then ./osp)"
