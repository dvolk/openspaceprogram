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

# Section flags for the cmake-built static libs: put every function/global in
# its own section so the game link's -Wl,--gc-sections can drop the ones we
# don't reference (saves ~200 KB on bullet3, ~140 KB on assimp).
# -fvisibility=hidden (what the game's own build does in the Makefile, and
# what SDL2 already does internally): the libs are statically linked, so
# their symbols never need to be exported -- hiding them keeps them out of
# the dynamic symbol table and lets the LTO pass at the game link inline /
# eliminate unreferenced code more aggressively (a hidden symbol can't be
# called from outside the binary).
SECT="-ffunction-sections -fdata-sections -fvisibility=hidden"
# LTO: emit GIMPLE bytecode instead of machine code, so the game link's
# -flto (the LTO var in the Makefile) runs the optimizer across the game +
# these libs too. Requires the same compiler version as the game link (a
# compiler upgrade means a bootstrap re-run); changing this flag re-runs
# cmake, which rebuilds the libs (bytecode objects are not interchangeable
# with the old machine-code ones).
LTO="-flto"
# -march: target ISA (keep it the same as the Makefile's MARCH, same
# default). Default native (AVX2 etc. on this machine); MARCH=x86-64-v3
# for a portable-but-modern ISA, MARCH= (empty) for plain x86-64. The
# binary is locked to the ISA it was built with (older CPU -> SIGILL).
# Changing it re-runs cmake, which rebuilds all the libs.
MARCH="${MARCH-native}"
ARCH=""
if [ -n "$MARCH" ]; then ARCH="-march=$MARCH"; fi

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

echo "=== building bullet3 (static, double precision, Release) ==="
# CMAKE_POLICY_VERSION_MINIMUM: bullet3 declares a pre-3.5 cmake policy,
# which cmake 4 rejects without this
# demos/extras/tests are not linked by the game, so keep them out
# Release (-O3 -DNDEBUG): bullet3's own CMakeLists also defaults to
# Release, but set it explicitly like the other libs; the build type owns
# the optimization flags, so nothing else is passed.
cmake -S middleware/bullet3 -B middleware/bullet3/build \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DUSE_DOUBLE_PRECISION=ON \
    -DBUILD_BULLET2_DEMOS=OFF -DBUILD_EXTRAS=OFF -DBUILD_UNIT_TESTS=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_FLAGS="$SECT $LTO $ARCH" -DCMAKE_CXX_FLAGS="$SECT $LTO $ARCH"
cmake --build middleware/bullet3/build -j"$JOBS"

echo "=== building SDL2 (static, X11) ==="
# Static lib (SDL_SHARED=OFF), X11 video driver linked in (X11_SHARED=OFF,
# so the game link carries the -lX11... libs). Wayland/Vulkan stay off:
# the game runs on X11 (e2e under Xvfb) and uses GL 4.5 via GLEW.
# SDL2's own EGL API declares SDL_EGL_CreateSurface with a mismatched
# type (NativeWindowType vs void*) in SDL_egl_c.h vs SDL_egl.c; LTO is
# the first thing to see both TUs together and warn. The game uses the
# GLX/SDL_GL path, never the EGL API, so silence it there.
cmake -S middleware/sdl2 -B middleware/sdl2/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DSDL_SHARED=OFF -DSDL_STATIC=ON -DSDL_TESTS=OFF \
    -DSDL_X11=ON -DSDL_X11_SHARED=OFF -DSDL_X11_XTEST=OFF \
    -DSDL_WAYLAND=OFF -DSDL_VULKAN=OFF \
    -DCMAKE_C_FLAGS="$SECT $LTO $ARCH -Wno-lto-type-mismatch" \
    -DCMAKE_CXX_FLAGS="$SECT $LTO $ARCH -Wno-lto-type-mismatch"
cmake --build middleware/sdl2/build -j"$JOBS"

echo "=== building SDL_image (static, PNG-only) ==="
# The game only loads/saves PNG (textures, skybox, screenshots), so build
# just the PNG loader + saver (like the OBJ-only assimp build). Links the
# system libpng + the SDL2 we built above (SDL2_DIR -> its build dir, so
# find_package picks ours even if the system SDL2 dev files exist).
cmake -S middleware/sdl2-image -B middleware/sdl2-image/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=OFF \
    -DSDL2_DIR="$PWD/middleware/sdl2/build/SDL2" \
    -DSDL2IMAGE_DEPS_SHARED=OFF -DSDL2IMAGE_VENDORED=OFF \
    -DSDL2IMAGE_SAMPLES=OFF -DSDL2IMAGE_BACKEND_STB=OFF -DSDL2IMAGE_PNG_SAVE=ON \
    -DSDL2IMAGE_AVIF=OFF -DSDL2IMAGE_BMP=OFF -DSDL2IMAGE_GIF=OFF \
    -DSDL2IMAGE_JPG=OFF -DSDL2IMAGE_LBM=OFF -DSDL2IMAGE_PCX=OFF \
    -DSDL2IMAGE_PNM=OFF -DSDL2IMAGE_QOI=OFF -DSDL2IMAGE_SVG=OFF \
    -DSDL2IMAGE_TGA=OFF -DSDL2IMAGE_TIF=OFF -DSDL2IMAGE_WEBP=OFF \
    -DSDL2IMAGE_XCF=OFF -DSDL2IMAGE_XPM=OFF -DSDL2IMAGE_XV=OFF \
    -DCMAKE_C_FLAGS="$SECT $LTO $ARCH" -DCMAKE_CXX_FLAGS="$SECT $LTO $ARCH"
cmake --build middleware/sdl2-image/build -j"$JOBS"

echo "=== building GLEW (static, 2.2.0) ==="
# GLEW's git repo contains only the generator (src/glew.c is generated from
# the Khronos registry), so vendor the official 2.2.0 source tarball
# (pre-generated; the same artifact distros build from). Fetched once, kept
# in tmp/, and only re-downloaded if the extracted source is missing.
if [ ! -f middleware/glew/src/glew.c ]; then
    mkdir -p tmp
    curl -fsSL -o tmp/glew_2.2.0.orig.tar.xz \
        'http://archive.ubuntu.com/ubuntu/pool/universe/g/glew/glew_2.2.0.orig.tar.xz'
    mkdir -p middleware/glew
    tar xJf tmp/glew_2.2.0.orig.tar.xz -C middleware/glew --strip-components=1
fi
# The cmake project lives in build/cmake (there is no root CMakeLists.txt),
# and the static target glew_s -> <builddir>/lib/libGLEW.a. The build dir is
# build-cmake (NOT build/, which is a SOURCE directory of this tree).
# Release: GLEW's CMakeLists also defaults to it; set explicitly for
# uniformity with the other libs.
cmake -S middleware/glew/build/cmake -B middleware/glew/build-cmake \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DBUILD_UTILS=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_FLAGS="$SECT $LTO $ARCH" -DCMAKE_CXX_FLAGS="$SECT $LTO $ARCH"
cmake --build middleware/glew/build-cmake -j"$JOBS"

echo "=== building assimp (static, OBJ-only) ==="
# We only ever load .obj meshes, so build just the OBJ importer (and no
# exporters). The default all-importers build pulls in ~30 format loaders
# (FBX, glTF, STEP, IFC, ...) that add ~11 MB to the game binary.
cmake -S middleware/assimp -B middleware/assimp/build \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF \
    -DASSIMP_BUILD_TESTS=OFF -DASSIMP_BUILD_SAMPLES=OFF -DASSIMP_INSTALL=OFF \
    -DASSIMP_BUILD_ALL_IMPORTERS_BY_DEFAULT=OFF \
    -DASSIMP_BUILD_OBJ_IMPORTER=ON \
    -DASSIMP_BUILD_ALL_EXPORTERS_BY_DEFAULT=OFF \
    -DCMAKE_C_FLAGS="$SECT $LTO $ARCH" -DCMAKE_CXX_FLAGS="$SECT $LTO $ARCH"
cmake --build middleware/assimp/build -j"$JOBS"

echo
echo "middleware ready. Now:  make   (then ./osp)"
