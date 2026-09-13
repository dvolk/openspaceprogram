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

echo "=== building SDL3 (static, X11) ==="
# Static lib (SDL_SHARED=OFF), X11 video driver linked in (SDL_X11_SHARED=OFF,
# so the game link carries the -lX11... libs). Wayland/Vulkan stay off:
# the game runs on X11 (e2e under Xvfb) and uses GL 4.5 via GLEW.
# SDL_TESTS defaults ON for the main project, so force it off (we never link
# the testsuite). SDL3 ships a proper CMake config in the build dir that
# SDL_image3's find_package(SDL3) consumes below.
# The game renders with raw GL (GLEW) + imgui and only uses SDL's video/
# events/keyboard/mouse/surface APIs, so compile out every subsystem it never
# touches. The 2D renderer is the big one -- its software blit/blend backend
# (~1 MB) is pure dead weight here. Joystick/haptic/HIDAPI/sensor/power/GPU,
# camera, native dialogs, tray, KMSDRM (X11-only), and the offscreen + dummy
# drivers are likewise unused. Audio and GLES stay on -- both are coming
# later (desktop GL, SDL_OPENGL, stays too).
cmake -S middleware/sdl3 -B middleware/sdl3/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DSDL_SHARED=OFF -DSDL_STATIC=ON -DSDL_DEPS_SHARED=OFF \
    -DSDL_TESTS=OFF \
    -DSDL_RENDER=OFF -DSDL_GPU=OFF \
    -DSDL_JOYSTICK=OFF -DSDL_HIDAPI=OFF -DSDL_HAPTIC=OFF \
    -DSDL_SENSOR=OFF -DSDL_POWER=OFF \
    -DSDL_CAMERA=OFF -DSDL_DIALOG=OFF -DSDL_TRAY=OFF \
    -DSDL_KMSDRM=OFF -DSDL_OFFSCREEN=OFF \
    -DSDL_OPENGL=ON -DSDL_OPENGLES=ON -DSDL_LIBUDEV=OFF \
    -DSDL_DUMMYVIDEO=OFF -DSDL_DUMMYCAMERA=OFF \
    -DSDL_X11=ON -DSDL_X11_SHARED=OFF -DSDL_X11_XTEST=OFF \
    -DSDL_WAYLAND=OFF -DSDL_VULKAN=OFF \
    -DCMAKE_C_FLAGS="$SECT $LTO $ARCH" \
    -DCMAKE_CXX_FLAGS="$SECT $LTO $ARCH"
cmake --build middleware/sdl3/build -j"$JOBS"

echo "=== building SDL_image3 (static, PNG-only) ==="
# The game only loads/saves PNG (textures, skybox, screenshots), so build
# just the PNG loader + saver (like the OBJ-only assimp build). Links the
# system libpng + the SDL3 we built above (SDL3_DIR -> its build dir, so
# find_package picks ours even if the system SDL3 dev files exist).
cmake -S middleware/sdl3-image -B middleware/sdl3-image/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=OFF \
    -DSDL3_DIR="$PWD/middleware/sdl3/build" \
    -DSDLIMAGE_DEPS_SHARED=OFF -DSDLIMAGE_VENDORED=OFF \
    -DSDLIMAGE_SAMPLES=OFF -DSDLIMAGE_TESTS=OFF -DSDLIMAGE_BACKEND_STB=OFF \
    -DSDLIMAGE_PNG=ON -DSDLIMAGE_PNG_SAVE=ON \
    -DSDLIMAGE_AVIF=OFF -DSDLIMAGE_BMP=OFF -DSDLIMAGE_GIF=OFF \
    -DSDLIMAGE_JPG=OFF -DSDLIMAGE_JXL=OFF -DSDLIMAGE_LBM=OFF \
    -DSDLIMAGE_PCX=OFF -DSDLIMAGE_ANI=OFF -DSDLIMAGE_PNM=OFF \
    -DSDLIMAGE_QOI=OFF -DSDLIMAGE_SVG=OFF \
    -DSDLIMAGE_TGA=OFF -DSDLIMAGE_TIF=OFF -DSDLIMAGE_WEBP=OFF \
    -DSDLIMAGE_XCF=OFF -DSDLIMAGE_XPM=OFF -DSDLIMAGE_XV=OFF \
    -DCMAKE_C_FLAGS="$SECT $LTO $ARCH" -DCMAKE_CXX_FLAGS="$SECT $LTO $ARCH"
cmake --build middleware/sdl3-image/build -j"$JOBS"

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
