#CXX_OPT=-march=x86-64-v2 -mtune=znver3 -flto -O2 -fprofile-arcs -ftest-coverage -fprofile-dir=/home/dv/src/my/openspaceprogram/data/pgo -fprofile-generate=/home/dv/src/my/openspaceprogram/data/pgo
#LD_OPT=-O2 -flto -fprofile-arcs
#SANITIZE=-g3 -fsanitize=address -fsanitize=leak -fsanitize=undefined

# LTO: at compile time -flto emits GIMPLE bytecode instead of machine code;
# the optimizer + codegen then run once at the link (see LFLAGS), across all
# TUs we build here (src/, imgui, implot) and the cmake-built middleware --
# bootstrap.sh passes -flto to those too, so the whole binary is one LTO
# program. Bytecode is compiler-version-locked: a compiler upgrade means a
# bootstrap re-run. A `make clean` is required after toggling this (bytecode
# objects are not interchangeable with machine-code ones).
# -flto=N runs the final codegen on N threads; plain -flto uses 1. N only
# schedules the link -- the bytecode is unchanged, so a different N needs
# no clean (toggling LTO on/off does).
LTO=-flto=$(shell nproc)

# -march: target ISA (the compatibility contract). Default x86-64-v2 (runs
# on ~2010+ CPUs); MARCH=x86-64-v3 for a faster-but-less-portable ISA,
# MARCH=native for "my box only", MARCH= (empty) for plain x86-64.
# -mtune: which core to SCHEDULE for, without changing the ISA (code still
# runs on the -march baseline). Default znver3; MTUNE=generic for a
# family-neutral tune, MTUNE= (empty) to skip it.
# The binary is locked to the -march ISA (older CPU -> SIGILL). Both must
# match bootstrap.sh (same defaults) so the MIDDLEWARE is built at the same
# baseline -- otherwise the game is v2 but the libs are native and the
# whole thing is not actually v2-portable. Each config (release/asan/tsan)
# is a separate make invocation that applies these defaults independently,
# so pass the same MARCH/MTUNE to every config you build. Changing either
# does NOT auto-rebuild: make tracks file times, not recipe flags, and LTO
# bytecode carries each function's target across relinks -- `make clean` is
# required
# (same as for LTO), and a bootstrap.sh re-run re-bases the middleware.
MARCH ?= x86-64-v2
MTUNE ?= znver3
ARCH = $(if $(MARCH),-march=$(MARCH)) $(if $(MTUNE),-mtune=$(MTUNE))

# PGO: profile-guided optimization (the "release build" lever), two phases:
#   1. make clean && make PGO=gen     (instrumented build, a bit slower)
#   2. run the game across the hot paths (launch, orbit, EVA, dock, spin)
#   3. make clean && make PGO=use     (reads the profile data, optimizes)
# Profile data lands in tmp/pgo (survives `make clean`; the dir is baked
# into the binary as an absolute path, so it works from any cwd).
# `make clean` is mandatory between phases: make can't detect a flag
# change (see MARCH). Stale data after a big code change warns "no data
# for counter" -- repeat 1-3. Scope: the game + imgui/implot (CXXFLAGS)
# and the link; the middleware stays PGO-free, so bootstrap.sh never
# depends on profile data existing.
PGO ?=
ifeq ($(PGO),gen)
PGOFLAGS = -fprofile-generate -fprofile-dir=$(CURDIR)/tmp/pgo
else
ifeq ($(PGO),use)
PGOFLAGS = -fprofile-use -fprofile-dir=$(CURDIR)/tmp/pgo
endif
endif

# Binary-size trim (dead code + symbol table):
#   -ffunction-sections / -fdata-sections   each function / global gets its
#                                           own section, so the linker can
#                                           drop the unused ones individually
#   -Wl,--gc-sections                       drop every section not reachable
#                                           from a root (main, .ctors, the
#                                           dynamic symbol table)
#   -fvisibility=hidden                     default-hide OUR symbols so they
#                                           don't bloat the dynamic symbol
#                                           table (the C/C++ libs we link opt
#                                           their own API in, so calls still
#                                           resolve)
#   -Wl,--as-needed                         only record DT_NEEDED for a shared
#                                           lib that actually resolves a used
#                                           symbol (drops unneeded deps)
SECT=-ffunction-sections -fdata-sections -fvisibility=hidden
LDFLAGS=-Wl,--gc-sections -Wl,--as-needed

# OS: which platform we build (bootstrap.sh's OS, same default). windows =
# cross-compile from Linux with mingw-w64 (see
# reports/build-tree2026_09_22/phase1-windows.md). The game source is
# platform-clean (all OS contact is SDL3/SDL_image/SDL_mixer), so this only
# swaps the compiler, the link closure, the artifact name, and the ./osp
# symlink (a linux convenience).
OS ?= linux
ifeq ($(OS),windows)
CXX= x86_64-w64-mingw32-g++
else
CXX= g++
endif
# -MMD -MP emit a .d dependency file per object so a changed header (e.g. frame.h)
# forces a recompile of every TU that includes it. Without this, make only sees
# the .cpp prerequisite and silently links stale .o files with a mismatched
# struct layout -> heap corruption / segfault. The .d files are -included below.
# assimp's include order: the BUILD dir (config.h is cmake-generated, never in
# the source tree) before the source headers. (The linux build used to resolve
# config.h from a system libassimp-dev install by accident -- the cross build
# has no such system headers and exposed it.)
# GLEW_STATIC: we link GLEW as a static lib, so GLEWAPI must not mark its
# symbols __declspec(dllimport) -- on windows that makes the compiler emit
# __imp_ import-thunk references a real static archive can't satisfy (the
# non-LTO debug config dies at link; LTO happens to resolve it, don't rely
# on that). Linux ignores the declspec, so the define is a no-op there.
CXXFLAGS=$(CFGFLAGS) -MMD -MP $(CFGLTO) $(SECT) $(ARCH) $(PGOFLAGS) $(CXX_OPT) -Wall -Wextra -Wpedantic -Wno-unused-variable -Wno-unused-parameter -Wno-unused-but-set-variable -std=c++20 -DGLEW_STATIC -I./middleware/glm/ -I./middleware/bullet3/ -I./middleware/bullet3/bullet -I./middleware/imgui/ -I./middleware/ -I$(MWROOT)/assimp/include/ -I./middleware/assimp/include/ -I./middleware/sdl3/include -I./middleware/sdl3-image/include -I./middleware/sdl-mixer/include -I./middleware/glew/include

LINKER=$(CXX) $(CFGFLAGS) $(LD_OPT) -o
LDLIBS=$(GL_LIBS) $(ASSIMP_LIB)

# Default to all cores: a plain `make` runs parallel (verified: MAKEFLAGS
# set in-file takes effect, and a command-line -jN still overrides it).
# $(shell ...) -- not $(nproc) -- because make 4.4 does not run the shell
# for an undefined variable; it would expand to empty, leaving a bare
# -j, which means *unlimited* jobs.
MAKEFLAGS += -j$(shell nproc)

# imgui submodule (pinned to a tagged release), built like everything else
# (the build rules are near the bottom, after the main target). Their objects
# live under $(OBJDIR), not a hardcoded obj/, so every build variant (plain,
# asan, tsan -- see those targets at the bottom) compiles its own copy with
# its own flags instead of clobbering the shared one. Recursive (=)
# assignment: $(OBJDIR) is defined further down, and a command-line
# OBJDIR=... must win over it.
IMGUI_DIR=./middleware/imgui
IMGUI_OBJS=$(OBJDIR)/imgui/imgui.o $(OBJDIR)/imgui/imgui_draw.o $(OBJDIR)/imgui/imgui_widgets.o $(OBJDIR)/imgui/imgui_tables.o $(OBJDIR)/imgui/imgui_impl_sdl3.o $(OBJDIR)/imgui/imgui_impl_opengl3.o
# implot submodule (pinned to a tagged release): plotting plugin that draws
# through the existing imgui renderer, so only its two .cpp files are built.
IMPLLOT_DIR=./middleware/implot
IMPLLOT_OBJS=$(OBJDIR)/implot/implot.o $(OBJDIR)/implot/implot_items.o
# assimp submodule (pinned to a tagged release), built static via cmake like
# bullet3 (assimp 6 defaults to shared, so force -DBUILD_SHARED_LIBS=OFF).
# Two assimp objects (Compression.cpp, unzip.c) reference zlib -- compressed
# textures / zip-packed formats we never load, so plain-text .obj pulls
# neither in and no -lz is needed (verified by linking without it).
# $(ASSIMP_A) is the archive file (used as a relink prerequisite); ASSIMP_LIB
# is what goes on the link line.
ASSIMP_A=$(MWROOT)/assimp/lib/libassimp.a
ASSIMP_LIB=$(ASSIMP_A)
# SDL3 + SDL_image + GLEW: vendored in middleware/ like bullet3/assimp
# (bootstrap.sh builds them static; GLEW from the official 2.2.0 tarball --
# its git repo ships only the generator, see bootstrap.sh).
SDL3_A=$(MWROOT)/sdl3/libSDL3.a
SDLIMG_A=$(MWROOT)/sdl3-image/libSDL3_image.a
# SDL3_mixer (vendored like SDL_image; bootstrap.sh builds it static with
# WAV + the bundled stb_vorbis only): the short SFX chunks play on the
# regular mixer channels (Mix_Chunk is WAV-only), the OGG ambient music
# streams on the music channel. A dependent of SDL3, so it links BEFORE
# it (static link order: dependents first).
SDLMIXER_A=$(MWROOT)/sdl-mixer/libSDL3_mixer.a
# SDL_image's PNG loader/saver use the vendored libpng + zlib (sdl3-image's
# nested submodules, built under its build dir's external/ -- no system
# libpng/zlib packages needed).
PNG_A=$(MWROOT)/sdl3-image/external/libpng-build/libpng16.a
# zlib's and GLEW's static targets rename themselves per-OS (zlibstatic /
# glew32 on windows) -- the archive NAME, not just its location, is
# platform-dependent.
ifeq ($(OS),windows)
ZLIB_A=$(MWROOT)/sdl3-image/external/zlib-build/libzlibstatic.a
GLEW_A=$(MWROOT)/glew/lib/libglew32.a
else
ZLIB_A=$(MWROOT)/sdl3-image/external/zlib-build/libz.a
GLEW_A=$(MWROOT)/glew/lib/libGLEW.a
endif
# The system closure behind the static SDL3 build (per-OS drivers:
# bootstrap.sh's SDL3_DRIVERS).
#  linux:   SDL3 is built with the X11 driver linked in (not dlopen'd), so
#           the X11 stack rides along. Audio: PulseAudio (primary) + ALSA
#           (fallback) -- see bootstrap.sh for why (direct ALSA cracks the
#           engine track; the server's queue absorbs the callback jitter).
#  windows: SDL3's static-link closure (the set its docs list for Windows)
#           + opengl32 (backing GLEW's wgl) + what SDL3's own code actually
#           references: uuid (the COM IIDs it uses live in uuid.lib, not
#           ole32), imm32 (IME), setupapi (audio device enumeration). Audio
#           is WASAPI (built into SDL3 on Windows, no extra lib).
ifeq ($(OS),windows)
SDL3_SYS=-lopengl32 -lwinmm -lversion -luser32 -lgdi32 -ladvapi32 -lshell32 -lole32 -luuid -limm32 -lsetupapi
else
SDL3_SYS=-lX11 -lXext -lXcursor -lXi -lXfixes -lXrandr -lXss -lasound -lpulse -ldl -lm -lpthread
endif
# Static link order matters (dependents before dependencies):
# SDL_image -> SDL3, GLEW -> GL, PNG loader/saver -> libpng -> zlib.
# GL: -lGL (libGL.so) on linux; windows' opengl32 is already in SDL3_SYS
# (GLEW's + SDL3's wgl references resolve against it).
ifeq ($(OS),windows)
GL_LIBS=$(SDLIMG_A) $(SDLMIXER_A) $(SDL3_A) $(GLEW_A) $(PNG_A) $(ZLIB_A) $(SDL3_SYS)
else
GL_LIBS=$(SDLIMG_A) $(SDLMIXER_A) $(SDL3_A) $(GLEW_A) -lGL $(PNG_A) $(ZLIB_A) $(SDL3_SYS)
endif
# bullet3's cmake scatters its libs per-component on linux
# (src/<Lib>/) but groups them into lib/ on windows -- per-OS layout.
ifeq ($(OS),windows)
BULLET3_OBJS=$(MWROOT)/bullet3/lib/libBulletDynamics.a $(MWROOT)/bullet3/lib/libBulletCollision.a $(MWROOT)/bullet3/lib/libBulletSoftBody.a $(MWROOT)/bullet3/lib/libBullet3Geometry.a $(MWROOT)/bullet3/lib/libBulletInverseDynamics.a $(MWROOT)/bullet3/lib/libBullet3Common.a $(MWROOT)/bullet3/lib/libBullet3Collision.a $(MWROOT)/bullet3/lib/libLinearMath.a $(MWROOT)/bullet3/lib/libBullet2FileLoader.a $(MWROOT)/bullet3/lib/libBullet3OpenCL_clew.a $(MWROOT)/bullet3/lib/libBullet3Dynamics.a
else
BULLET3_OBJS=$(MWROOT)/bullet3/src/BulletDynamics/libBulletDynamics.a $(MWROOT)/bullet3/src/BulletCollision/libBulletCollision.a $(MWROOT)/bullet3/src/BulletSoftBody/libBulletSoftBody.a $(MWROOT)/bullet3/src/Bullet3Geometry/libBullet3Geometry.a $(MWROOT)/bullet3/src/BulletInverseDynamics/libBulletInverseDynamics.a $(MWROOT)/bullet3/src/Bullet3Common/libBullet3Common.a $(MWROOT)/bullet3/src/Bullet3Collision/libBullet3Collision.a $(MWROOT)/bullet3/src/LinearMath/libLinearMath.a $(MWROOT)/bullet3/src/Bullet3Serialize/Bullet2FileLoader/libBullet2FileLoader.a $(MWROOT)/bullet3/src/Bullet3OpenCL/libBullet3OpenCL_clew.a $(MWROOT)/bullet3/src/Bullet3Dynamics/libBullet3Dynamics.a
endif

# windows: fully static (D5 in the phase 1 report) -- MinGW's libgcc/
# libstdc++/winpthread go INTO the exe, so it ships as one portable file
# with no runtime DLLs.
ifeq ($(OS),windows)
STATIC_LD=-static
else
STATIC_LD=
endif

# -Wno-lto-type-mismatch: SDL2's own EGL API (SDL_egl_c.h vs SDL_egl.c)
# declares SDL_EGL_CreateSurface with mismatched types, and the LTO pass
# here is the first thing to see both TUs together and warn. The game uses
# the GLX/SDL_GL path, never the EGL API (bootstrap.sh silences the same
# warning in SDL2's own compile).
LFLAGS=$(CFGLTO) $(ARCH) $(PGOFLAGS) $(LDFLAGS) $(STATIC_LD) -Wall -Wno-lto-type-mismatch $(LDLIBS) $(IMGUI_LIBS) $(BULLET3_OBJS)

# Build tree (reports/build-tree2026_09_22): build/<os>-<march>-<mtune>/<config>/.
# Each (march, mtune, config) combo gets its own dir with its own objects, so
# variants can't relink each other's stale LTO bytecode and an ISA change
# lands in a different tree instead of silently reusing old objects.
# Tokens: -march x86-64-v2 -> v2, native -> native, (empty) -> base;
# MTUNE as-is (znver3), or untuned when empty.
BUILDROOT=build
OSTOK=$(OS)
MARCH_TOK=$(if $(MARCH),$(subst x86-64-,,$(MARCH)),base)
MTUNE_TOK=$(if $(MTUNE),$(MTUNE),untuned)
CONFIG ?= release
ARCHDIR=$(BUILDROOT)/$(OSTOK)-$(MARCH_TOK)-$(MTUNE_TOK)
SRCDIR=src
OBJDIR=$(ARCHDIR)/$(CONFIG)/obj
BINDIR=$(ARCHDIR)/$(CONFIG)
# The unit tests are one -O2 build at the baseline level (like
# middleware/): shared by every config, not a config of its own.
TESTDIR=$(ARCHDIR)/tests
# The cmake-built middleware (bootstrap.sh) is likewise shared at the
# (os, march, mtune) level: each <name> is that library's cmake build dir
# (the archives keep the layout each CMakeLists chooses inside it).
MWROOT=$(ARCHDIR)/middleware
# Per-config binary name: the sanitizer builds keep their suffix (their dirs
# are already separate, but the suffix keeps `ls` self-explanatory);
# release/debug share the name osp, their dirs separate them.
ifeq ($(CONFIG),asan)
TARGET=osp_asan
else ifeq ($(CONFIG),tsan)
TARGET=osp_tsan
else
TARGET=osp
endif
# windows artifacts are .exe (the dir already names the config)
ifeq ($(OS),windows)
TARGET:=$(TARGET).exe
endif
# Per-config flags. release: O2 + LTO (the old default). debug: O0 + debug
# info, no LTO (plain per-file debugging). asan/tsan: keep O2 + LTO and add
# -g3 + the sanitizer (same flags as before, now selected by CONFIG). The
# link line gets the same: LTO finalizes at the link, and the sanitizer must
# be on the link line too.
ifeq ($(CONFIG),debug)
CFGFLAGS=-O0 -g3
CFGLTO=
else ifeq ($(CONFIG),asan)
ifeq ($(OS),windows)
# mingw has no LeakSanitizer runtime; address+undefined are supported.
CFGFLAGS=-O2 -g3 -fsanitize=address,undefined
else
CFGFLAGS=-O2 -g3 -fsanitize=address,leak,undefined
endif
CFGLTO=$(LTO)
else ifeq ($(CONFIG),tsan)
CFGFLAGS=-O2 -g3 -fsanitize=thread,undefined
CFGLTO=$(LTO)
else
CFGFLAGS=-O2
CFGLTO=$(LTO)
endif

SOURCES := $(wildcard $(SRCDIR)/*.cpp)
INCLUDES := $(wildcard $(SRCDIR)/*.h)
OBJECTS  := $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
# Header dependency files (generated by -MMD). -include is silent when absent
# (first build), and pulls each .o's full include list into the dependency graph.
DEPS     := $(OBJECTS:.o=.d) $(IMGUI_OBJS:.o=.d) $(IMPLLOT_OBJS:.o=.d)
rm = rm -f

# Default entry point: build this config's binary, then (linux only) point
# ./osp at it. `all` is phony, so the symlink refreshes on EVERY invocation
# even when the binary is already up to date -- ./osp always tracks the last
# `make` you ran, whatever config (release/debug/asan/tsan). The link target
# is relative (resolved from the repo root), so the tree can be moved. A
# windows build must not touch the linux ./osp symlink: its entry point is
# the .exe itself (run it with wine).
.PHONY: all
all: $(BINDIR)/$(TARGET)
ifeq ($(OS),windows)
	@echo "run: wine $(BINDIR)/$(TARGET)"
else
	ln -sfn $(BINDIR)/$(TARGET) osp
endif

# The static libs (assimp + bullet) are prerequisites too: they're built by
# bootstrap.sh (cmake), not this make, so a middleware rebuild doesn't show up
# as a changed .o -- listing the .a files makes make relink when they're
# newer than the binary. (On a fresh checkout before bootstrap, make reports
# the missing .a instead of failing at the link.)
$(BINDIR)/$(TARGET): $(OBJECTS) $(IMGUI_OBJS) $(IMPLLOT_OBJS) $(ASSIMP_A) $(SDL3_A) $(SDLIMG_A) $(SDLMIXER_A) $(GLEW_A) $(BULLET3_OBJS)
	$(LINKER) $@ $(IMGUI_OBJS) $(IMPLLOT_OBJS) $(OBJECTS) $(LFLAGS)

$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# The git version string into src/version.h (embedded in the main menu by
# gameui.cpp). `version` is phony, so its check re-runs on every build; it
# rewrites the header only when the string actually changed, so a quiet
# `make` stays quiet and gameui.o (which depends on the header file
# itself) is recompiled only when the version changes. A file target with
# no prerequisites (the old form) would be seen as up-to-date forever
# once the header existed, and the menu would keep the stale string.
# Note: make fixes its rebuild plan before recipes run, so a changed
# version lands on the build after the one that rewrote the header;
# `make clean && make` (the release flow) picks it up immediately.
# `make version` re-runs the check on demand (e.g. after adding a tag);
# `make version VERSION=1.0` overrides it outside a git checkout.
.PHONY: version
version:
	@( VERSION_STRING="$(VERSION)"; \
       [ -e "./.git" ] && GITVERSION=$$( git describe --tags --always --dirty --match "v*.*" ) && VERSION_STRING=$$GITVERSION ; \
       [ -e "src/version.h" ] && OLDVERSION=$$(grep VERSION src/version.h|cut -d '"' -f2) ; \
       if [ "x$$VERSION_STRING" != "x$$OLDVERSION" ]; then echo "#define VERSION \"$$VERSION_STRING\"" | tee src/version.h ; fi \
     )

src/version.h: version

$(OBJDIR)/gameui.o: src/version.h

# imgui / implot objects (the paths come from IMGUI_OBJS / IMPLLOT_OBJS, so
# they follow $(OBJDIR) and each variant builds its own). Three pattern rules
# replace the eight per-file ones; the two imgui rules are order-insensitive
# because make picks the rule whose prerequisite exists -- the backend files
# live in backends/, the core ones in the submodule root.
$(OBJDIR)/imgui/%.o: $(IMGUI_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJDIR)/imgui/%.o: $(IMGUI_DIR)/backends/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJDIR)/implot/%.o: $(IMPLLOT_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Unit tests. Every binary is its own file target, so make builds the stale
# ones in parallel (MAKEFLAGS -j$(nproc)) and skips the fresh ones -- the
# old single phony recipe recompiled + relinked all ~30 serially on every
# run. `test` builds whatever is stale, then runs every binary (always,
# even when fresh, so runtime inputs like res/ get re-checked).
#
# All TUs -- the test files and the shared src files -- compile once into
# $(TESTDIR)/obj/ with -MMD (a changed header rebuilds every dependent test).
# $(TESTDIR)/obj/ is separate from obj/: the game's objects are LTO bytecode,
# the tests' are machine code, and the two must not mix.
#
# The heavy tests link the static libs' LTO bytecode, so their link line
# carries $(LTO) (-flto=$(nproc)): the per-object LTRANS jobs run on N
# threads instead of serially (measured ~53 s -> ~8 s here).
#
# The two pattern rules are order-sensitive: if a stem ever exists in both
# src/ and tests/ (none do today), the src/ rule wins -- keep it first.
# If you delete a test source but keep its target, make reuses the stale
# $(TESTDIR)/obj/ object and binary happily -- run `make clean` after removing one.

TCC   = -O2 -std=c++20
TINC  = -I./src -I./middleware/glm/ -I./middleware/bullet3/ -I./middleware/bullet3/bullet \
        -I./middleware/imgui/ -I./middleware/ -I./middleware/sdl3/include \
        -I./middleware/sdl3-image/include -I./middleware/sdl-mixer/include -I./middleware/glew/include
TLIBS = $(BULLET3_OBJS) $(GL_LIBS) $(ASSIMP_LIB)
# The real-Bullet tests share these TUs (compiled once, not once per test).
TCOMMON_OBJS = $(TESTDIR)/obj/physics.o $(TESTDIR)/obj/body.o $(TESTDIR)/obj/vehicle.o \
               $(TESTDIR)/obj/shipdef.o $(TESTDIR)/obj/frame.o $(TESTDIR)/obj/terrain.o \
               $(TESTDIR)/obj/shader.o $(TESTDIR)/obj/camera.o $(TESTDIR)/obj/mesh.o \
               $(TESTDIR)/obj/texture.o $(TESTDIR)/obj/gldebug.o

$(TESTDIR)/obj/%.o: src/%.cpp
	@mkdir -p $(TESTDIR)/obj
	$(CXX) $(TCC) -MMD -MP $(TINC) -c $< -o $@

$(TESTDIR)/obj/%.o: tests/%.cpp
	@mkdir -p $(TESTDIR)/obj
	$(CXX) $(TCC) -MMD -MP $(TINC) -c $< -o $@

# reference frames + orbital spawn math (src/frame.cpp): pure math, no
# rendering/Bullet needed at runtime.
$(TESTDIR)/test_frames: $(TESTDIR)/obj/test_frames.o $(TESTDIR)/obj/frame.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_spawn: $(TESTDIR)/obj/test_spawn.o $(TESTDIR)/obj/frame.o
	$(CXX) -o $@ $^

# attitude law (pure C, no Bullet): the per-substep braking law
# main.cpp runs, pinned across the authority/warp grid.
$(TESTDIR)/test_attitude: $(TESTDIR)/obj/test_attitude.o
	$(CXX) -o $@ $^

# slew law in 3 DOF (pure C++, no Bullet/GL): the full-transverse law
# damps the "third-axis" spin the old slew-axis-only law left undamped
# (the prograde wobble), across spin magnitudes and warps; pins the
# non-vacuous guard (the old law must wobble) + the authority bound.
$(TESTDIR)/test_slew3d: $(TESTDIR)/obj/test_slew3d.o
	$(CXX) -o $@ $^

# thrust fixes (substep delivery, fuel flow, SetMass inertia): links the
# real src/physics.cpp, so it pulls in the render chain + Bullet + GL libs.
$(TESTDIR)/test_thrust: $(TESTDIR)/obj/test_thrust.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

# fuel drain (the real Vehicle::consumeResourceMass from src/vehicle.cpp
# + the real SetMass): pro-rata across the active stage's tanks (not
# first-tank-first), stage gating, no stranded fuel, no partial drain.
$(TESTDIR)/test_fuel: $(TESTDIR)/obj/test_fuel.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

# electrical (KSP-style EC): the powerTick gate (wheels need power
# left over after life support; no-EC ships ungated) + the pool balance
# (RTG charges, life support + active wheels drain, clamped) + EC has
# no mass. Calls powerTick/drainEC/chargeEC directly, so headless.
$(TESTDIR)/test_power: $(TESTDIR)/obj/test_power.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

# staging topology (Vehicle::droppedPartsAtStage from src/vehicle.cpp): a
# decoupler drops itself + its whole child-side subtree, a sibling branch
# sharing the stage NUMBER survives (the heavy_two rule), and nested /
# same-stage decouplers compose. Pure graph logic over Part::parent -- it
# reads no Bullet state -- but it links like test_fuel because ~Vehicle
# (src/vehicle.cpp) references the physics teardown symbols.
$(TESTDIR)/test_staging: $(TESTDIR)/obj/test_staging.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

# docking merge/split (Vehicle::absorbShip + extractSubtreeAsShip from
# src/vehicle.cpp): absorbShip is a rigid merge -- every absorbed part keeps
# its exact world pose, the seam is recorded, the absorbed root rehangs off
# the survivor's port -- and extractSubtreeAsShip is its exact inverse (the
# undock round-trip restores both ships' geometry). This is the same general
# primitive a future "dropped stage becomes a ship" will call. Headless:
# init() runs rebuildCompound (the one hull body) but NOT enterWorld, and
# extractSubtreeAsShip leaves enterWorld to its caller, so no physics world.
$(TESTDIR)/test_dock: $(TESTDIR)/obj/test_dock.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

# containment edge (Part::owner/container/contents, part.h) + the invariant
# check (Vehicle::checkPartInvariants, src/vehicle.cpp): owner is wired by
# the attach primitives and re-pointed by merge/split; the container/
# contents back-references hold and only a character's part (isEva) may be
# contained, in a real capsule. The game populates no containment yet (step
# 2.4 does), so this test wires a chain by hand -- the way the transitions
# will -- and checks the invariant passes, and fails where it must fail.
# A local TestCrew : Vehicle stands in for Kerbal (eva.cpp links game.h,
# too heavy for a headless test); the invariant is keyed on the isEva()
# virtual, so the stand-in exercises the same path.
$(TESTDIR)/test_contain: $(TESTDIR)/obj/test_contain.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

# ship mass properties (Vehicle::get_center_of_mass / getInertia from
# src/vehicle.cpp): golden values against an independent analytic
# parallel-axis assembly, incl. the products of inertia and rotated
# non-cubic parts. This is the tensor a reaction wheel's authority and
# the autopilot slew law divide by; nothing else pins it (test_attitude
# and test_slew3d simulate their own hardcoded Ix/Iz, and e2e 22 only
# checks ratios). Also pins the ship's single compound rigid body
# (Vehicle::rebuildCompound): the principal-axis transform's COM origin,
# its diagonalized inertia against that same analytic reference, the
# re-based child poses, and the part poses derived back out of the body
# at an arbitrary world pose. Headless: no world, no GL context.
$(TESTDIR)/test_inertia: $(TESTDIR)/obj/test_inertia.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

# inventory transfer (phase 4.3): re-parenting between containers,
# capacity enforcement, ownership (~Part deletes ownedContents).
# Headless: no Game / GL (links Body + Bullet for the ~Part -> ~Body chain).
$(TESTDIR)/test_inventory: $(TESTDIR)/obj/test_inventory.o $(TESTDIR)/obj/inventory.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

# rotation model (physical wheel torque, per-substep law, torque
# delivery).
$(TESTDIR)/test_rotation: $(TESTDIR)/obj/test_rotation.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

# ship/part JSON data model (GL-free: catalog + ship-def parse/validate,
# part resolution, aggregates). Runs from the repo root (needs res/).
$(TESTDIR)/test_shipload: $(TESTDIR)/obj/test_shipload.o $(TESTDIR)/obj/shipdef.o
	$(CXX) -o $@ $^

# save/load JSON (de)serialization (src/save.h, header-only): the
# SaveMeta + SaveShip round trip (every field), permissive reads (an
# absent/wrong-typed key keeps the default), mat3/vec3 serialization.
$(TESTDIR)/test_save: $(TESTDIR)/obj/test_save.o
	$(CXX) -o $@ $^

# crew_capacity on the part catalog (GL-free: which parts are capsules
# and their seat count, the default-0 for everything else, error path).
$(TESTDIR)/test_crew: $(TESTDIR)/obj/test_crew.o $(TESTDIR)/obj/shipdef.o
	$(CXX) -o $@ $^

# fleet JSON (GL-free: entry parse + defaults + error paths).
$(TESTDIR)/test_fleet: $(TESTDIR)/obj/test_fleet.o $(TESTDIR)/obj/fleet.o
	$(CXX) -o $@ $^

# home-planet calendar (src/calendar.h, header-only pure math): day/year
# from spin/orbit rates, 427-day snapped year, months, epoch year,
# tidally-locked + star edge cases. Pinned to the Eerbon JSON rates.
$(TESTDIR)/test_calendar: $(TESTDIR)/obj/test_calendar.o
	$(CXX) -o $@ $^

# two-body orbital elements + time-to-apsis (src/orbit.h, header-only
# pure math): elements, anomaly conversions, the ApT/PeT countdown fix,
# hyperbolic/parabolic handling, degenerate-plane guards.
$(TESTDIR)/test_orbit: $(TESTDIR)/obj/test_orbit.o
	$(CXX) -o $@ $^

# orbit-sampling cache (src/orbitsample.h, header-only pure math): the
# map's per-orbit points, cached on the elements so coasting orbits are
# propagated once. Circular/eccentric radii, cache hit + invalidation,
# hyperbolic -> empty.
$(TESTDIR)/test_orbitsample: $(TESTDIR)/obj/test_orbitsample.o
	$(CXX) -o $@ $^

# Lambert solver + min-dv planner (src/transfer.h, header-only pure
# math): Hohmann analytic reference, round-trip, hyperbolic leg.
$(TESTDIR)/test_transfer: $(TESTDIR)/obj/test_transfer.o
	$(CXX) -o $@ $^

# porkchop 2-D sweep (src/transfer.h, header-only pure math): pinned to
# planTransfer (t_dep = 0 row) + the Hohmann analytic min + the no-
# solution (all-NaN) path + grid bookkeeping.
$(TESTDIR)/test_porkchop: $(TESTDIR)/obj/test_porkchop.o
	$(CXX) -o $@ $^

# surface map projection + terminator (src/surfmap.h, header-only pure
# math): the equirectangular pixel <-> direction round-trip, the
# lon/lat convention (lon 0 = +Z, north = +Y -- the same atan2(x, z) /
# asin(y) render.cpp uses), the shade range, the antimeridian wrap.
$(TESTDIR)/test_surfmap: $(TESTDIR)/obj/test_surfmap.o
	$(CXX) -o $@ $^

# EVA control-law geometry (src/evamath.h, header-only pure math): the
# Rodrigues rotation + axis-angle round-trip (incl. the 180-deg
# fallback), the camera/upright target bases, the screen-axis helpers.
$(TESTDIR)/test_eva: $(TESTDIR)/obj/test_eva.o
	$(CXX) -o $@ $^

# terrain core (src/terragen.h, header-only pure math): the height
# model (bounds, sea floor, the LOD band-limit fade), the surface
# color (sea, palette, gas-giant bands), and the grid builder
# (vertex/index counts, band-limited on-surface vertices, the skirt
# ring below the terrain).
$(TESTDIR)/test_terrain: $(TESTDIR)/obj/test_terrain.o
	$(CXX) -o $@ $^

# atmospheric drag law (src/drag.h, header-only pure math): the
# exponential density (rho(H)=rho0/e, monotone, below-surface -> 0,
# degenerate atmo -> 0) and the force (opposite v, |F|=0.5 rho cd A v^2,
# 4x at 2x speed, zero on any degenerate input).
$(TESTDIR)/test_drag: $(TESTDIR)/obj/test_drag.o
	$(CXX) -o $@ $^

# audio positional math (src/audio.h, inline pure math): the world->listener
# frame conversion (listener at the origin, looking down -z, +x right, +y up)
# for axis-aligned, offset, yawed and tilted listeners; the up||forward
# degenerate case stays finite; the rotation preserves length.
$(TESTDIR)/test_audio: $(TESTDIR)/obj/test_audio.o
	$(CXX) -o $@ $^

# jet engine thrust factor (src/drag.h, header-only pure math): the
# air-breathing multiplier -- the speed ramp (the VTOL floor: f0 at
# rest, linear to 1 at v_rated, saturating above) times the density
# falloff (linear in rho/rho_sea, ZERO in vacuum, clamped at 1),
# degenerate inputs -> 0 (or the clamped floor).
$(TESTDIR)/test_jet: $(TESTDIR)/obj/test_jet.o
	$(CXX) -o $@ $^

# background job runner (src/job.cpp): the worker/main-thread handoff --
# the body runs off the calling thread, the returned continuation runs on
# the poll() thread, jobs land in posted order, a throwing body does not
# kill the worker, busy()/poll() report the state + the running label.
$(TESTDIR)/test_jobs: $(TESTDIR)/obj/test_jobs.o $(TESTDIR)/obj/job.o
	$(CXX) -o $@ $^

# orbital map projection (src/orbitmap.h, pure-math part): project() drops
# the map normal (+Y) and scales XZ by meters-per-pixel. Header-only, so
# the imgui include is headers-only (no imgui/Bullet/GL link needed).
$(TESTDIR)/test_orbitmap: $(TESTDIR)/obj/test_orbitmap.o
	$(CXX) -o $@ $^

# orbit camera (src/camera.cpp, pure math): pitching past the pole must
# keep the up vector continuous (no sudden roll) and the view NaN-free.
$(TESTDIR)/test_orbitcam: $(TESTDIR)/obj/test_orbitcam.o $(TESTDIR)/obj/camera.o
	$(CXX) -o $@ $^

# picking (src/pick.cpp): pixel->ray round-trip through the camera's
# own view/projection (a point on the ray projects back to the pixel),
# then the real Bullet convex-cast hull ray-test (hit point/distance,
# a miss, translated + rotated bodies). pick.cpp includes game.h (the
# fleet), so the imgui include dir is needed for ui.h; and pickShipPart
# casts against a ship's compound children through Vehicle's pose
# accessors, so vehicle.cpp + physics.cpp + body.cpp + shipdef.cpp link in.
$(TESTDIR)/test_pick: $(TESTDIR)/obj/test_pick.o $(TESTDIR)/obj/pick.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

# settings.json mapping (src/settings.cpp, nlohmann): the
# SettingsData <-> JSON round trip, absent-key tolerance (a field the
# file does not mention keeps the current value), mistyped-key
# tolerance, and the window-mode name mapping.
$(TESTDIR)/test_settings: $(TESTDIR)/obj/test_settings.o $(TESTDIR)/obj/settings.o $(TESTDIR)/obj/keys.o
	$(CXX) -o $@ $^

# key map (src/keys.cpp): the exact-modifier lookup (a plain binding
# fires only with no Shift/Ctrl/Alt held; a combo only with exactly its
# modifiers), the default map (the previously-hardcoded keys, cam/eva
# up-down on R/F), --sim-press plain-key compatibility, naming.
# Pure logic -- no SDL link (no SDL calls).
$(TESTDIR)/test_keys: $(TESTDIR)/obj/test_keys.o $(TESTDIR)/obj/keys.o
	$(CXX) -o $@ $^

# cli (src/cli.cpp): the --version / --help short-circuits (exit 0,
# correct output), a valid parse, and an invalid flag's nonzero exit.
# cli.cpp includes version.h (the --version string), which is generated
# by the phony version target -- wire it in like gameui.o's dep.
# siminput.o: the --sim-press key/button name maps cli.cpp folds into.
$(TESTDIR)/obj/cli.o: src/version.h
$(TESTDIR)/test_cli: $(TESTDIR)/obj/test_cli.o $(TESTDIR)/obj/cli.o $(TESTDIR)/obj/siminput.o
	$(CXX) -o $@ $^

TESTS = test_frames test_spawn test_attitude test_slew3d test_thrust test_fuel \
        test_power test_staging test_dock test_contain test_inertia test_inventory \
        test_rotation test_shipload test_save test_crew test_fleet test_calendar \
        test_orbit test_orbitsample test_transfer test_porkchop test_surfmap test_eva \
        test_terrain test_drag test_audio test_jet test_jobs test_orbitmap test_orbitcam \
        test_pick test_settings test_keys test_cli

.PHONY: test
test: $(addprefix $(TESTDIR)/,$(TESTS))
	$(TESTDIR)/test_frames
	$(TESTDIR)/test_spawn
	$(TESTDIR)/test_attitude
	$(TESTDIR)/test_slew3d
	$(TESTDIR)/test_thrust
	$(TESTDIR)/test_fuel
	$(TESTDIR)/test_power
	$(TESTDIR)/test_staging
	$(TESTDIR)/test_dock
	$(TESTDIR)/test_contain
	$(TESTDIR)/test_inertia
	$(TESTDIR)/test_inventory
	$(TESTDIR)/test_rotation
	$(TESTDIR)/test_shipload
	$(TESTDIR)/test_save
	$(TESTDIR)/test_crew
	$(TESTDIR)/test_fleet
	$(TESTDIR)/test_calendar
	$(TESTDIR)/test_orbit
	$(TESTDIR)/test_orbitsample
	$(TESTDIR)/test_transfer
	$(TESTDIR)/test_porkchop
	$(TESTDIR)/test_surfmap
	$(TESTDIR)/test_eva
	$(TESTDIR)/test_terrain
	$(TESTDIR)/test_drag
	$(TESTDIR)/test_audio
	$(TESTDIR)/test_jet
	$(TESTDIR)/test_jobs
	$(TESTDIR)/test_orbitmap
	$(TESTDIR)/test_orbitcam
	$(TESTDIR)/test_pick
	$(TESTDIR)/test_settings
	$(TESTDIR)/test_keys
	$(TESTDIR)/test_cli

# E2E battery: launch the built game under Xvfb and run the pass/fail cases
# in e2e/cases/ (see e2e/run.py). Needs the game binary, so it depends on
# $(TARGET). Runs headless via xvfb-run; on a machine with a real display it
# uses that instead. Stdlib Python only.
# Cases run in parallel (run.py --jobs, default 2); override with JOBS,
# e.g. `make e2e JOBS=4` or `make e2e JOBS=1` for serial.
JOBS ?=
E2E_JOBS = $(if $(JOBS),--jobs $(JOBS),)
# --force: the e2e target IS the "run the whole battery" intent, so it opts
# into run.py's full-battery guard (a bare run.py refuses it as a safety
# nudge).
.PHONY: e2e
e2e: all
	python3 e2e/run.py $(E2E_JOBS) --force --game $(BINDIR)/$(TARGET)

# Release artifacts (phase 2, reports/build-tree2026_09_22/phase2-artifacts.md):
# per-OS archives + a combined one, in dist/ (gitignored, uploaded manually).
# The per-OS trees are named by the same <os>-<march>-<mtune> tokens, so the
# binary paths follow the ARCHDIR rule with the other OS substituted.
DISTDIR=dist
LINUX_BIN=$(BUILDROOT)/linux-$(MARCH_TOK)-$(MTUNE_TOK)/release/osp
WINDOWS_BIN=$(BUILDROOT)/windows-$(MARCH_TOK)-$(MTUNE_TOK)/release/osp.exe
# The wine gate is a parity smoke, not the full matrix (the native battery
# above is the full gate -- 47 serial wine cases would take half an hour on
# a desktop box). Default = the phase 1.4 acceptance set; extend with
# WINE_CASES="...". run.py auto-serializes .exe games.
WINE_CASES ?= smoke vab-launch vab-launch-orbit vab-launch-body

# Fast path: build + package, NO test/e2e gates ("testing releases" mode).
# The version string comes from the version target (same logic as the build's
# embedded VERSION), so the archive name always matches the game's --version
# output. A dirty tree is allowed but named as-is (-dirty) and warned.
.PHONY: artifacts
artifacts:
	@$(MAKE) --no-print-directory version
	@$(MAKE) --no-print-directory all
	@$(MAKE) --no-print-directory OS=windows all
	@VER=$$(sed -n 's/^#define VERSION "\(.*\)"/\1/p' src/version.h); \
	if [ -z "$$VER" ]; then \
		echo "error: no version string (src/version.h missing?)" >&2; exit 1; \
	fi; \
	case "$$VER" in *-dirty*) \
		echo "warning: version '$$VER' marks a dirty tree (tag it for a clean name)";; \
	esac; \
	STAGE=tmp/release; \
	rm -rf "$$STAGE"; \
	for layout in osp-$$VER-linux osp-$$VER-windows osp-$$VER-linux+windows; do \
		mkdir -p "$$STAGE/$$layout"; \
		cp -r res "$$STAGE/$$layout/"; \
		cp LICENSE.md release/README.md "$$STAGE/$$layout/"; \
	done; \
	cp "$(LINUX_BIN)"   "$$STAGE/osp-$$VER-linux/"; \
	cp "$(WINDOWS_BIN)" "$$STAGE/osp-$$VER-windows/"; \
	cp "$(LINUX_BIN)"   "$$STAGE/osp-$$VER-linux+windows/"; \
	cp "$(WINDOWS_BIN)" "$$STAGE/osp-$$VER-linux+windows/"; \
	mkdir -p $(DISTDIR); \
	(cd "$$STAGE" && tar -cJf ../../$(DISTDIR)/osp-$$VER-linux.tar.xz osp-$$VER-linux) && \
	(cd "$$STAGE" && tar -cJf ../../$(DISTDIR)/osp-$$VER-linux+windows.tar.xz osp-$$VER-linux+windows) && \
	(cd "$$STAGE" && zip -r -q ../../$(DISTDIR)/osp-$$VER-windows.zip osp-$$VER-windows) && \
	(cd $(DISTDIR) && sha256sum osp-$$VER-linux.tar.xz osp-$$VER-windows.zip osp-$$VER-linux+windows.tar.xz > SHA256SUMS) && \
	rm -rf "$$STAGE"; \
	ls -lh $(DISTDIR)

# Real-release path: full gates first (native unit tests + the full native
# e2e battery + the wine parity set), then package. Any gate failing stops
# the flow before dist/ is touched.
.PHONY: release
release:
	@$(MAKE) --no-print-directory test
	@$(MAKE) --no-print-directory e2e
	@$(MAKE) --no-print-directory OS=windows all
	@python3 e2e/run.py --force --game $(WINDOWS_BIN) $(WINE_CASES)
	@$(MAKE) --no-print-directory artifacts

# GL-context probe: on this Mesa 26 stack any draw (or vertex-attribute
# setup) made in the default VAO 0 fails with GL_INVALID_OPERATION — draws
# must happen inside a real glGenVertexArrays VAO (even an empty one for a
# vertex-less gl_VertexID quad). Needs an X display:
#     DISPLAY=:99 make test-gl
.PHONY: test-gl
test-gl:
	$(CXX) -O2 -std=c++20 -I./middleware/sdl3/include $(LTO) tests/test_vertexless.c $(GL_LIBS) -o $(TESTDIR)/test_gl_vao
	$(TESTDIR)/test_gl_vao

# Config variants. Each is a separate recursive make with its own config
# dir (see OBJDIR/BINDIR above), so the builds share no file at all:
#
#     make          -> build/linux-v2-znver3/release/osp   (the default: O2 + LTO)
#     make debug    -> build/linux-v2-znver3/debug/osp     (-O0 -g3, no LTO)
#     make asan     -> build/linux-v2-znver3/asan/osp_asan (+ AddressSanitizer)
#     make tsan     -> build/linux-v2-znver3/tsan/osp_tsan (+ ThreadSanitizer)
#
# No `make clean` is needed between them, and they can run concurrently in
# separate shells. The imgui/implot objects follow OBJDIR as well, so a
# variant's middleware TUs are instrumented like its src/ ones -- TSan only
# reports races in instrumented code, so an uninstrumented imgui would hide
# the UI side of any race.
# NOT per-variant: the cmake-built static libs (bullet3, assimp, SDL3,
# SDL_image, GLEW). bootstrap.sh builds them once, uninstrumented, and every
# variant links the same archives.
#
# Run a variant exactly like osp, e.g.
#     xvfb-run -a build/linux-v2-znver3/asan/osp_asan --selftest-spawn --timeout 5
# ASan aborts on the first error. LeakSanitizer also runs at exit and reports
# the intentional leaks (the shader/mesh/texture registries are never freed),
# so ASAN_OPTIONS=detect_leaks=0 keeps the output to real memory errors.
# TSan needs ./tsan.supp to be usable at all (Mesa's software renderer is
# noisy under it) -- use `make tsan-run` rather than remembering the env var.
.PHONY: debug
debug:
	@$(MAKE) --no-print-directory CONFIG=debug all

# windows asan: probe the toolchain first -- this Ubuntu mingw cross package
# compiles -fsanitize= but ships NO ASan runtime for the Windows target (no
# libsanitizer.spec, no libasan under /usr/lib/gcc/x86_64-w64-mingw32/), so
# a real attempt dies at link after a full compile. The one-line probe
# links in ~1s and says whether asan is possible here at all.
.PHONY: asan
asan:
ifeq ($(OS),windows)
	@printf 'int main() { return 0; }\n' > tmp/asan_probe.cpp && \
	  x86_64-w64-mingw32-g++ -static -fsanitize=address tmp/asan_probe.cpp -o tmp/asan_probe.exe 2>tmp/asan_probe.err \
	  && { rm -f tmp/asan_probe.cpp tmp/asan_probe.exe tmp/asan_probe.err; } \
	  || { echo "error: windows asan unavailable here: this mingw toolchain has no ASan runtime for x86_64-w64-mingw32 (probe: $$(head -1 tmp/asan_probe.err 2>/dev/null))" >&2; \
	       echo "       use the linux asan config, or a toolchain that ships a mingw ASan runtime (full asan validation was deferred to a real Windows box anyway -- phase 1, Risks)" >&2; \
	       rm -f tmp/asan_probe.cpp tmp/asan_probe.err tmp/asan_probe.exe; exit 1; }
	@rm -f tmp/asan_probe.exe
	@$(MAKE) --no-print-directory CONFIG=asan all
else
	@$(MAKE) --no-print-directory CONFIG=asan all
endif

# tsan is linux-only: mingw has no ThreadSanitizer runtime (phase 1, D3 in
# reports/build-tree2026_09_22/phase1-windows.md) -- refuse clearly instead
# of dying deep in the cross-compile.
.PHONY: tsan
tsan:
ifeq ($(OS),windows)
	@echo "error: tsan is linux-only (mingw has no ThreadSanitizer runtime)" >&2; exit 1
else
	@$(MAKE) --no-print-directory CONFIG=tsan all
endif

# Build and run the TSan variant with tsan.supp applied (see that file for
# what it suppresses and why). Uses the real display when there is one and
# xvfb-run when there is not. Pass game args through GAME_ARGS:
#     make tsan-run GAME_ARGS="--timeout 10 --ship res/ships/racer.json"
# The exit code is TSan's: 0 = clean, 66 = it reported a race.
GAME_ARGS ?=
.PHONY: tsan-run
tsan-run: tsan
	@XVFB=""; if [ -z "$$DISPLAY" ]; then XVFB="xvfb-run -a"; fi; \
	 TSAN_OPTIONS="suppressions=$(CURDIR)/tsan.supp" $$XVFB $(ARCHDIR)/tsan/osp_tsan $(GAME_ARGS)

# Drop both sanitizer variants entirely -- objects and binaries.
.PHONY: san-clean
san-clean:
	rm -rf $(ARCHDIR)/asan $(ARCHDIR)/tsan

.PHONY: clean
# All objects (src/, imgui/implot, tests): imgui/implot are pinned submodules
# you rarely touch, but leaving their .o files across a clean is a trap -- an
# ISA/LTO/PGO change then relinks stale bytecode, or the build looks "up to
# date" and silently keeps the old ISA. Rebuilding them costs seconds, so
# clean always drops them.
clean:
	$(rm) $(OBJECTS) $(OBJECTS:.o=.d)
	rm -rf $(OBJDIR) $(TESTDIR)

.PHONY: remove
remove: clean
	$(rm) $(BINDIR)/$(TARGET)
	rm -rf $(TESTDIR)
ifeq ($(OS),windows)
	# ./osp is the linux convenience symlink -- a windows remove leaves it
	# (and its linux target) alone.
	@echo "note: left the linux ./osp symlink alone"
else
	# drop the ./osp symlink too (removing a link never touches its target)
	$(rm) osp
endif

# Pull in the generated header dependencies (see -MMD above). Silent if the
# .d files don't exist yet (fresh checkout / first build). The $(TESTDIR)/obj/
# objects (the unit tests' shared TUs) use the same -MMD mechanism; a
# wildcard keeps this list in sync with whatever has been compiled.
-include $(DEPS)
-include $(wildcard $(TESTDIR)/obj/*.d)
