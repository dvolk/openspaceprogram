#!/usr/bin/env bash
# bootstrap.sh -- fetch and build the cmake-built static middleware so `make` works.
set -euo pipefail
cd "$(dirname "$0")"

JOBS=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

# Sections for the game link's --gc-sections; hidden visibility (static, no exports).
SECT="-ffunction-sections -fdata-sections -fvisibility=hidden"
# LTO: must share compiler version with the game link; changing it re-runs cmake.
LTO="-flto"
# Must match the Makefile's MARCH/MTUNE (same defaults); changing either re-runs cmake.
MARCH="${MARCH-x86-64-v2}"
MTUNE="${MTUNE-znver3}"
ARCH=""
if [ -n "$MARCH" ]; then ARCH="-march=$MARCH"; fi
if [ -n "$MTUNE" ]; then ARCH="${ARCH:+$ARCH }-mtune=$MTUNE"; fi

# OS: matches the Makefile's OS. windows = mingw-w64 cross-compile.
OS="${OS-linux}"
# Windows cross: target triplet + mingw compilers (else CMake picks native gcc).
CROSS=""
if [ "$OS" = windows ]; then
    CROSS="-DCMAKE_SYSTEM_NAME=Windows \
           -DCMAKE_C_COMPILER=x86_64-w64-mingw32-gcc \
           -DCMAKE_CXX_COMPILER=x86_64-w64-mingw32-g++"
fi

# Per-OS video/audio drivers (see the cmake invocation for the toggles).
if [ "$OS" = windows ]; then
    SDL3_DRIVERS="-DSDL_OPENGL=ON -DSDL_OPENGLES=OFF \
                  -DSDL_X11=OFF -DSDL_WAYLAND=OFF -DSDL_VULKAN=OFF"
else
    # SDL3 auto-detects optional system libs and, with SDL_DEPS_SHARED=OFF,
    # links the ones it does not dlopen directly into libSDL3.a -- deps the
    # Makefile's SDL3_SYS does not carry, so a host that happens to have the
    # -dev package fails to link osp. Pin the ones we don't want OFF:
    # fribidi/libthai shape RTL/Thai text in SDL's messagebox (the X11 driver's
    # vtable always pulls that TU in, even though the game never calls
    # SDL_ShowMessageBox); pipewire is a second audio backend and PulseAudio
    # already talks to PipeWire's compat server.
    SDL3_DRIVERS="-DSDL_OPENGL=ON -DSDL_OPENGLES=ON -DSDL_LIBUDEV=OFF \
                  -DSDL_DUMMYVIDEO=OFF -DSDL_DUMMYCAMERA=OFF \
                  -DSDL_X11=ON -DSDL_X11_SHARED=OFF -DSDL_X11_XTEST=OFF \
                  -DSDL_FRIBIDI=OFF -DSDL_LIBTHAI=OFF \
                  -DSDL_WAYLAND=OFF -DSDL_VULKAN=OFF -DSDL_OFFSCREEN=ON \
                  -DSDL_ALSA=ON -DSDL_PULSEAUDIO=ON -DSDL_PIPEWIRE=OFF \
                  -DSDL_SNDIO=OFF -DSDL_JACK=OFF"
fi

# Mirrors the Makefile's MWROOT so middleware is shared across game configs.
MARCH_TOK="${MARCH#x86-64-}"
[ -z "$MARCH" ] && MARCH_TOK=base
MTUNE_TOK="${MTUNE:-untuned}"
MWROOT="build/${OS}-${MARCH_TOK}-${MTUNE_TOK}/middleware"

for tool in g++ cmake make; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "error: $tool not found -- install the system deps (see README) and re-run" >&2
        exit 1
    fi
done
if [ "$OS" = windows ]; then
    command -v x86_64-w64-mingw32-g++ >/dev/null 2>&1 || {
        echo "error: x86_64-w64-mingw32-g++ not found (apt install g++-mingw-w64-x86-64)" >&2
        exit 1
    }
    # release/osp.rc (the exe's icon + version info) is compiled by windres.
    command -v x86_64-w64-mingw32-windres >/dev/null 2>&1 || {
        echo "error: x86_64-w64-mingw32-windres not found (apt install binutils-mingw-w64-x86-64)" >&2
        exit 1
    }
fi

# Pinned submodules + sdl3-image's nested libpng/zlib (PNG build needs those only).
git submodule update --init
git -C middleware/sdl3-image submodule update --init external/libpng external/zlib

echo "=== building bullet3 (static, double precision, Release) ==="
# POLICY_VERSION_MINIMUM: bullet3 declares a pre-3.5 cmake policy (cmake 4 rejects).
cmake -S middleware/bullet3 -B "$MWROOT/bullet3" \
    $CROSS \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DUSE_DOUBLE_PRECISION=ON \
    -DBUILD_BULLET2_DEMOS=OFF -DBUILD_EXTRAS=OFF -DBUILD_UNIT_TESTS=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_FLAGS="$SECT $LTO $ARCH" -DCMAKE_CXX_FLAGS="$SECT $LTO $ARCH"
cmake --build "$MWROOT/bullet3" -j"$JOBS"

echo "=== building SDL3 (static, $OS drivers) ==="
# Static SDL3; drivers from $SDL3_DRIVERS. SDL_TESTS defaults ON -- force off.
# PulseAudio primary + ALSA fallback: ALSA-only left the mix callback no slack
# (crackles on start/tap). Offscreen video is the e2e EGL path. Keep audio on
# (SDL3_mixer needs it).
cmake -S middleware/sdl3 -B "$MWROOT/sdl3" \
    $CROSS \
    -DCMAKE_BUILD_TYPE=Release \
    -DSDL_SHARED=OFF -DSDL_STATIC=ON -DSDL_DEPS_SHARED=OFF \
    -DSDL_TESTS=OFF \
    -DSDL_RENDER=OFF -DSDL_GPU=OFF \
    -DSDL_JOYSTICK=OFF -DSDL_HIDAPI=OFF -DSDL_HAPTIC=OFF \
    -DSDL_SENSOR=OFF -DSDL_POWER=OFF \
    -DSDL_CAMERA=OFF -DSDL_DIALOG=OFF -DSDL_TRAY=OFF \
    -DSDL_KMSDRM=OFF \
    $SDL3_DRIVERS \
    -DCMAKE_C_FLAGS="$SECT $LTO $ARCH" \
    -DCMAKE_CXX_FLAGS="$SECT $LTO $ARCH"
cmake --build "$MWROOT/sdl3" -j"$JOBS"

# SDL3 auto-detects more optional system libs than the pins above cover, and
# SDL_DEPS_SHARED=OFF makes the ones it does not dlopen direct link deps that
# SDL3_SYS lacks. Catch a new one here, not at the osp link (or, worse, deep
# inside `make deb`'s dpkg-buildpackage).
if [ "$OS" != windows ]; then
    SDL3_STATIC="$MWROOT/sdl3/libSDL3.a"
    if [ ! -f "$SDL3_STATIC" ]; then
        echo "error: $SDL3_STATIC missing -- cmake built no static archive" >&2
        exit 1
    fi
    stray=$(nm -u "$SDL3_STATIC" 2>/dev/null | awk '$1 == "U" {print $2}' |
            { grep -E '^(fribidi_|th_|pw_|decor_)' || true; } | sort -u) ||
        { echo "error: nm -u $SDL3_STATIC failed (binutils installed?)" >&2; exit 1; }
    if [ -n "$stray" ]; then
        echo "error: libSDL3.a needs system libs the Makefile does not link:" >&2
        sed 's/^/  /' <<<"$stray" >&2
        echo "  pin the matching SDL_* option OFF, or add the lib to SDL3_SYS" >&2
        exit 1
    fi
fi

echo "=== building SDL_image3 (static, PNG-only) ==="
# PNG-only (the game loads/saves PNG). Vendored libpng+zlib; SDL3_DIR pins our SDL3.
cmake -S middleware/sdl3-image -B "$MWROOT/sdl3-image" \
    $CROSS \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=OFF \
    -DSDL3_DIR="$PWD/$MWROOT/sdl3" \
    -DSDLIMAGE_DEPS_SHARED=OFF -DSDLIMAGE_VENDORED=ON \
    -DSDLIMAGE_SAMPLES=OFF -DSDLIMAGE_TESTS=OFF -DSDLIMAGE_BACKEND_STB=OFF \
    -DSDLIMAGE_PNG=ON -DSDLIMAGE_PNG_SAVE=ON \
    -DSDLIMAGE_AVIF=OFF -DSDLIMAGE_BMP=OFF -DSDLIMAGE_GIF=OFF \
    -DSDLIMAGE_JPG=OFF -DSDLIMAGE_JXL=OFF -DSDLIMAGE_LBM=OFF \
    -DSDLIMAGE_PCX=OFF -DSDLIMAGE_ANI=OFF -DSDLIMAGE_PNM=OFF \
    -DSDLIMAGE_QOI=OFF -DSDLIMAGE_SVG=OFF \
    -DSDLIMAGE_TGA=OFF -DSDLIMAGE_TIF=OFF -DSDLIMAGE_WEBP=OFF \
    -DSDLIMAGE_XCF=OFF -DSDLIMAGE_XPM=OFF -DSDLIMAGE_XV=OFF \
    -DCMAKE_C_FLAGS="$SECT $LTO $ARCH" -DCMAKE_CXX_FLAGS="$SECT $LTO $ARCH"
cmake --build "$MWROOT/sdl3-image" -j"$JOBS"
# zlib/libpng rewrite in-tree files during an out-of-source build -- restore them.
git -C middleware/sdl3-image/external/zlib checkout -- zconf.h
git -C middleware/sdl3-image/external/libpng checkout -- config.guess config.sub

echo "=== building SDL3_mixer (static, WAV + stb_vorbis only) ==="
# WAV + stb_vorbis only (the game's two formats); all else off, so no external deps.
cmake -S middleware/sdl-mixer -B "$MWROOT/sdl-mixer" \
    $CROSS \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=OFF \
    -DSDL3_DIR="$PWD/$MWROOT/sdl3" \
    -DSDLMIXER_DEPS_SHARED=OFF \
    -DSDLMIXER_AIFF=OFF \
    -DSDLMIXER_AU=OFF \
    -DSDLMIXER_VOC=OFF \
    -DSDLMIXER_FLAC=OFF \
    -DSDLMIXER_GME=OFF \
    -DSDLMIXER_MOD=OFF \
    -DSDLMIXER_MP3=OFF \
    -DSDLMIXER_MIDI=OFF \
    -DSDLMIXER_OPUS=OFF \
    -DSDLMIXER_WAVPACK=OFF \
    -DSDLMIXER_VORBIS_VORBISFILE=OFF \
    -DSDLMIXER_VORBIS_STB=ON \
    -DSDLMIXER_TESTS=OFF \
    -DSDLMIXER_EXAMPLES=OFF \
    -DSDLMIXER_INSTALL=OFF \
    -DCMAKE_C_FLAGS="$SECT $LTO $ARCH" -DCMAKE_CXX_FLAGS="$SECT $LTO $ARCH"
cmake --build "$MWROOT/sdl-mixer" -j"$JOBS"

# GLEW's git tree has no generated glew.c (the release process builds it from
# core/*.c), so vendor the official release tarball. SourceForge is where upstream
# publishes it; 2.2.0 used to come from Ubuntu's universe pool, which made our GL
# loader's version a distro choice served off a mirror that can vanish (#999).
# Cached in tmp/ as before.
GLEW_VER=2.3.1
GLEW_SHA=b64790f94b926acd7e8f84c5d6000a86cb43967bd1e688b03089079799c9e889
echo "=== building GLEW (static, $GLEW_VER) ==="
# The guard is upstream's own version file (GLEW's cmake reads config/version the
# same way), not "does src/glew.c exist" -- that check let this script keep saying
# 2.2.0 while the tree on disk had quietly moved to 2.3.1. Never use
# GLEW_VERSION_MAJOR/MINOR/MICRO from glew.h: those are glewGetString() error codes
# (1..4), identical in every release.
glew_have=$(awk -F' *= *' '/^GLEW_(MAJOR|MINOR|MICRO) /{v[n++]=$2}
                         END{printf "%s.%s.%s", v[0], v[1], v[2]}' \
             middleware/glew/config/version 2>/dev/null)
if [ "$glew_have" != "$GLEW_VER" ]; then
    glew_tgz=tmp/glew-$GLEW_VER.tgz
    if [ ! -f "$glew_tgz" ]; then
        # tmp/ is gitignored, so a fresh clone has no such directory and curl dies
        # with (23) writing into it. --remove-on-error matters because a truncated
        # file left in the cache makes every later run fail the sha check below,
        # and only a manual rm recovers.
        mkdir -p tmp
        curl -fsSL --remove-on-error -o "$glew_tgz" \
            "https://sourceforge.net/projects/glew/files/glew/$GLEW_VER/glew-$GLEW_VER.tgz"
    fi
    # SourceForge redirects to a mirror picked for us, so verify before extracting,
    # and delete a bad cache -- the next run cannot fix it on its own.
    echo "$GLEW_SHA  $glew_tgz" | sha256sum -c - \
        || { rm -f "$glew_tgz"
             echo "error: $glew_tgz is not the GLEW $GLEW_VER release (removed; re-run to refetch)" >&2
             exit 1; }
    # Extract into an empty tree rather than over the old one: a GLEW tree with
    # files from two releases is a phantom-extension bug waiting to happen. Old tree
    # goes to tmp/, it does not get deleted -- and the $$ suffix is load bearing,
    # because `mv src dst` into an existing dst nests instead of replacing, and the
    # move after that fails outright.
    if [ -d middleware/glew ]; then
        glew_old=tmp/glew-pre-$GLEW_VER
        [ -e "$glew_old" ] && glew_old="$glew_old.$$"
        mv middleware/glew "$glew_old"
    fi
    mkdir -p middleware/glew
    tar xzf "$glew_tgz" -C middleware/glew --strip-components=1
fi
# cmake project is build/cmake (no root CMakeLists); static target -> libGLEW.a.
# BUILD_SHARED_LIBS defaults ON in 2.3.1 -- we link libglew32.a, so do not build a
# .so nobody uses (and do not leave old-version .so files lying in the build dir).
cmake -S middleware/glew/build/cmake -B "$MWROOT/glew" \
    $CROSS \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DBUILD_UTILS=OFF -DBUILD_SHARED_LIBS=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_FLAGS="$SECT $LTO $ARCH" -DCMAKE_CXX_FLAGS="$SECT $LTO $ARCH"
cmake --build "$MWROOT/glew" -j"$JOBS"

echo "=== building assimp (static, OBJ-only) ==="
# OBJ-only (the all-importers default adds ~11 MB of unused loaders).
# windows: assimp's MINGW branch + mingw LTO pe-bigobj objects break `ar`
# indexing -- force -fno-lto there (the game uses 4 assimp symbols).
ASSIMP_LTO="$LTO"
if [ "$OS" = windows ]; then
    ASSIMP_LTO="-fno-lto"
fi
cmake -S middleware/assimp -B "$MWROOT/assimp" \
    $CROSS \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF \
    -DASSIMP_BUILD_TESTS=OFF -DASSIMP_BUILD_SAMPLES=OFF -DASSIMP_INSTALL=OFF \
    -DASSIMP_BUILD_ALL_IMPORTERS_BY_DEFAULT=OFF \
    -DASSIMP_BUILD_OBJ_IMPORTER=ON \
    -DASSIMP_BUILD_ALL_EXPORTERS_BY_DEFAULT=OFF \
    -DCMAKE_C_FLAGS="$SECT $ASSIMP_LTO $ARCH" -DCMAKE_CXX_FLAGS="$SECT $ASSIMP_LTO $ARCH"
cmake --build "$MWROOT/assimp" -j"$JOBS"

echo
if [ "$OS" = windows ]; then
    echo "windows middleware ready. Now:  make OS=windows   (then: wine .../release/osp.exe)"
    echo "  (archives under $MWROOT/)"
else
    echo "middleware ready. Now:  make   (then ./osp)"
    echo "  (the binary is ${MWROOT%/middleware}/release/osp; ./osp is a symlink to it)"
fi
