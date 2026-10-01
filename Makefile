#CXX_OPT=-march=x86-64-v2 -mtune=znver3 -flto -O2 -fprofile-arcs -ftest-coverage -fprofile-dir=/home/dv/src/my/openspaceprogram/data/pgo -fprofile-generate=/home/dv/src/my/openspaceprogram/data/pgo
#LD_OPT=-O2 -flto -fprofile-arcs
#SANITIZE=-g3 -fsanitize=address -fsanitize=leak -fsanitize=undefined

# LTO: `make clean` required after toggling LTO on/off (and after a compiler
# upgrade: re-run bootstrap.sh). Changing -flto=N needs no clean.
LTO=-flto=$(shell nproc)

# -march ISA / -mtune schedule. Must match bootstrap.sh. Changing either
# needs `make clean` + a bootstrap re-run (make doesn't track flags).
MARCH ?= x86-64-v2
MTUNE ?= znver3
ARCH = $(if $(MARCH),-march=$(MARCH)) $(if $(MTUNE),-mtune=$(MTUNE))

# PGO: PGO=gen (instrument) -> run the game -> PGO=use (optimize). Profiles
# in tmp/pgo (survives clean). `make clean` between phases and after big
# code changes ("no data for counter" = repeat the run).
PGO ?=
ifeq ($(PGO),gen)
PGOFLAGS = -fprofile-generate -fprofile-dir=$(CURDIR)/tmp/pgo
else
ifeq ($(PGO),use)
PGOFLAGS = -fprofile-use -fprofile-dir=$(CURDIR)/tmp/pgo
endif
endif

# Dead-code + symbol-table trim (gc-sections, hidden visibility, as-needed).
SECT=-ffunction-sections -fdata-sections -fvisibility=hidden
LDFLAGS=-Wl,--gc-sections -Wl,--as-needed

# OS: linux | windows (mingw-w64 cross). Must match bootstrap.sh's OS.
OS ?= linux
ifeq ($(OS),windows)
CXX= x86_64-w64-mingw32-g++
else
CXX= g++
endif
# -MMD -MP: .d files (header changes must rebuild, else stale layout).
# MWROOT/assimp/include first (cmake config.h). GLEW_STATIC (static, not
# dllimport). -Wno-deprecated-enum-enum-conversion (implot, GCC).
CXXFLAGS=$(CFGFLAGS) -MMD -MP $(CFGLTO) $(SECT) $(ARCH) $(PGOFLAGS) $(CXX_OPT) -Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-deprecated-enum-enum-conversion -std=c++20 -DGLEW_STATIC -I./middleware/glm/ -I./middleware/bullet3/src -I./middleware/imgui/ -I./middleware/ -I$(MWROOT)/assimp/include/ -I./middleware/assimp/include/ -I./middleware/sdl3/include -I./middleware/sdl3-image/include -I./middleware/sdl-mixer/include -I./middleware/glew/include

LINKER=$(CXX) $(CFGFLAGS) $(LD_OPT) -o
LDLIBS=$(GL_LIBS) $(ASSIMP_LIB)

# Default -j$(nproc); $(shell) so make 4.4 expands it (bare -j = unlimited).
MAKEFLAGS += -j$(shell nproc)

# imgui (pinned submodule). Objects follow $(OBJDIR); recursive = so a
# command-line OBJDIR= wins.
IMGUI_DIR=./middleware/imgui
IMGUI_OBJS=$(OBJDIR)/imgui/imgui.o $(OBJDIR)/imgui/imgui_draw.o $(OBJDIR)/imgui/imgui_widgets.o $(OBJDIR)/imgui/imgui_tables.o $(OBJDIR)/imgui/imgui_impl_sdl3.o $(OBJDIR)/imgui/imgui_impl_opengl3.o
# implot (pinned submodule): two TUs through the imgui renderer.
IMPLLOT_DIR=./middleware/implot
IMPLLOT_OBJS=$(OBJDIR)/implot/implot.o $(OBJDIR)/implot/implot_items.o
# assimp (pinned, static cmake; -DBUILD_SHARED_LIBS=OFF). No -lz needed.
# ASSIMP_A = archive for relink prereqs.
ASSIMP_A=$(MWROOT)/assimp/lib/libassimp.a
ASSIMP_LIB=$(ASSIMP_A)
# SDL3 + SDL_image + GLEW (vendored, static via bootstrap.sh).
SDL3_A=$(MWROOT)/sdl3/libSDL3.a
SDLIMG_A=$(MWROOT)/sdl3-image/libSDL3_image.a
# SDL3_mixer (WAV + stb_vorbis). Links BEFORE SDL3 (dependents first).
SDLMIXER_A=$(MWROOT)/sdl-mixer/libSDL3_mixer.a
# libpng/zlib from sdl3-image's nested submodules (no system packages).
PNG_A=$(MWROOT)/sdl3-image/external/libpng-build/libpng16.a
# zlib/GLEW static target names differ per-OS.
ifeq ($(OS),windows)
ZLIB_A=$(MWROOT)/sdl3-image/external/zlib-build/libzlibstatic.a
GLEW_A=$(MWROOT)/glew/lib/libglew32.a
else
ZLIB_A=$(MWROOT)/sdl3-image/external/zlib-build/libz.a
GLEW_A=$(MWROOT)/glew/lib/libGLEW.a
endif
# Per-OS SDL3 system closure (see bootstrap.sh's SDL3_DRIVERS).
ifeq ($(OS),windows)
SDL3_SYS=-lopengl32 -lwinmm -lversion -luser32 -lgdi32 -ladvapi32 -lshell32 -lole32 -luuid -limm32 -lsetupapi
else
SDL3_SYS=-lX11 -lXext -lXcursor -lXi -lXfixes -lXrandr -lXss -lasound -lpulse -ldl -lm -lpthread
endif
# Static link order: dependents before dependencies.
ifeq ($(OS),windows)
GL_LIBS=$(SDLIMG_A) $(SDLMIXER_A) $(SDL3_A) $(GLEW_A) $(PNG_A) $(ZLIB_A) $(SDL3_SYS)
else
GL_LIBS=$(SDLIMG_A) $(SDLMIXER_A) $(SDL3_A) $(GLEW_A) -lGL $(PNG_A) $(ZLIB_A) $(SDL3_SYS)
endif
# bullet3 archives: per-component src/<Lib>/ on linux, lib/ on windows.
ifeq ($(OS),windows)
BULLET3_OBJS=$(MWROOT)/bullet3/lib/libBulletDynamics.a $(MWROOT)/bullet3/lib/libBulletCollision.a $(MWROOT)/bullet3/lib/libBulletSoftBody.a $(MWROOT)/bullet3/lib/libBullet3Geometry.a $(MWROOT)/bullet3/lib/libBulletInverseDynamics.a $(MWROOT)/bullet3/lib/libBullet3Common.a $(MWROOT)/bullet3/lib/libBullet3Collision.a $(MWROOT)/bullet3/lib/libLinearMath.a $(MWROOT)/bullet3/lib/libBullet2FileLoader.a $(MWROOT)/bullet3/lib/libBullet3OpenCL_clew.a $(MWROOT)/bullet3/lib/libBullet3Dynamics.a
else
BULLET3_OBJS=$(MWROOT)/bullet3/src/BulletDynamics/libBulletDynamics.a $(MWROOT)/bullet3/src/BulletCollision/libBulletCollision.a $(MWROOT)/bullet3/src/BulletSoftBody/libBulletSoftBody.a $(MWROOT)/bullet3/src/Bullet3Geometry/libBullet3Geometry.a $(MWROOT)/bullet3/src/BulletInverseDynamics/libBulletInverseDynamics.a $(MWROOT)/bullet3/src/Bullet3Common/libBullet3Common.a $(MWROOT)/bullet3/src/Bullet3Collision/libBullet3Collision.a $(MWROOT)/bullet3/src/LinearMath/libLinearMath.a $(MWROOT)/bullet3/src/Bullet3Serialize/Bullet2FileLoader/libBullet2FileLoader.a $(MWROOT)/bullet3/src/Bullet3OpenCL/libBullet3OpenCL_clew.a $(MWROOT)/bullet3/src/Bullet3Dynamics/libBullet3Dynamics.a
endif

# windows: fully static exe (no runtime DLLs).
ifeq ($(OS),windows)
STATIC_LD=-static
else
STATIC_LD=
endif

# -Wno-lto-type-mismatch: SDL2 EGL API vs EGL impl, only visible under LTO.
LFLAGS=$(CFGLTO) $(ARCH) $(PGOFLAGS) $(LDFLAGS) $(STATIC_LD) -Wall -Wno-lto-type-mismatch $(LDLIBS) $(IMGUI_LIBS) $(BULLET3_OBJS)

# build/<os>-<march>-<mtune>/<config>/: one tree per combo (no shared/stale
# LTO bytecode). Tokens: x86-64-v2 -> v2, (empty) -> base; MTUNE or untuned.
BUILDROOT=build
OSTOK=$(OS)
MARCH_TOK=$(if $(MARCH),$(subst x86-64-,,$(MARCH)),base)
MTUNE_TOK=$(if $(MTUNE),$(MTUNE),untuned)
CONFIG ?= release
ARCHDIR=$(BUILDROOT)/$(OSTOK)-$(MARCH_TOK)-$(MTUNE_TOK)
SRCDIR=src
OBJDIR=$(ARCHDIR)/$(CONFIG)/obj
BINDIR=$(ARCHDIR)/$(CONFIG)
# Unit tests: one shared -O2 build at the baseline ISA.
TESTDIR=$(ARCHDIR)/tests
# cmake middleware (bootstrap.sh) lives under $(MWROOT) at the (os,arch) level.
MWROOT=$(ARCHDIR)/middleware
# Sanitizer binaries keep a suffix in the name.
ifeq ($(CONFIG),asan)
TARGET=osp_asan
else ifeq ($(CONFIG),tsan)
TARGET=osp_tsan
else
TARGET=osp
endif
# windows artifacts are .exe
ifeq ($(OS),windows)
TARGET:=$(TARGET).exe
endif
# Per-config flags: release O2+LTO; debug O0 -g3 no LTO; asan/tsan O2+LTO
# + sanitizer (also on the link line -- LTO finalizes at the link).
ifeq ($(CONFIG),debug)
CFGFLAGS=-O0 -g3
CFGLTO=
else ifeq ($(CONFIG),asan)
ifeq ($(OS),windows)
# mingw has no LeakSanitizer runtime.
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
# Header deps from -MMD (-included below).
DEPS     := $(OBJECTS:.o=.d) $(IMGUI_OBJS:.o=.d) $(IMPLLOT_OBJS:.o=.d)
rm = rm -f

# Build this config's binary, then (linux) point ./osp at it. Phony so the
# symlink always tracks the last `make`. Windows leaves ./osp alone.
.PHONY: all
all: $(BINDIR)/$(TARGET)
ifeq ($(OS),windows)
	@echo "run: wine $(BINDIR)/$(TARGET)"
else
	ln -sfn $(BINDIR)/$(TARGET) osp
endif

# Middleware .a files are prereqs (built by bootstrap.sh, not this make) so a
# rebuilt lib forces a relink. Missing .a before bootstrap = a clear error.
$(BINDIR)/$(TARGET): $(OBJECTS) $(IMGUI_OBJS) $(IMPLLOT_OBJS) $(ASSIMP_A) $(SDL3_A) $(SDLIMG_A) $(SDLMIXER_A) $(GLEW_A) $(BULLET3_OBJS)
	$(LINKER) $@ $(IMGUI_OBJS) $(IMPLLOT_OBJS) $(OBJECTS) $(LFLAGS)

$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Embed git version in src/version.h (gameui.cpp). Phony re-check; rewrites
# only on change (a change lands one build later). VERSION=1.0 overrides.
.PHONY: version
version:
	@( VERSION_STRING="$(VERSION)"; \
       [ -e "./.git" ] && GITVERSION=$$( git describe --tags --always --dirty --match "v*.*" ) && VERSION_STRING=$$GITVERSION ; \
       [ -e "src/version.h" ] && OLDVERSION=$$(grep VERSION src/version.h|cut -d '"' -f2) ; \
       if [ "x$$VERSION_STRING" != "x$$OLDVERSION" ]; then echo "#define VERSION \"$$VERSION_STRING\"" | tee src/version.h ; fi \
     )

src/version.h: version

$(OBJDIR)/gameui.o: src/version.h

# imgui / implot pattern rules (order-insensitive: two stems share imgui/).
$(OBJDIR)/imgui/%.o: $(IMGUI_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJDIR)/imgui/%.o: $(IMGUI_DIR)/backends/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJDIR)/implot/%.o: $(IMPLLOT_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Unit tests: per-binary file targets (parallel, skip-fresh); `test` runs all.
# $(TESTDIR)/obj/ is machine-code (never mix with the game's LTO objects);
# heavy tests link with $(LTO). src/ pattern rule must precede tests/.
# After deleting a test source, `make clean` (stale .o is otherwise reused).

TCC   = -O2 -std=c++20 -DGLEW_STATIC
TINC  = -I./src -I./middleware/glm/ -I./middleware/bullet3/src \
        -I./middleware/imgui/ -I./middleware/ -I$(MWROOT)/assimp/include/ -I./middleware/assimp/include/ \
        -I./middleware/sdl3/include -I./middleware/sdl3-image/include -I./middleware/sdl-mixer/include -I./middleware/glew/include
TLIBS = $(BULLET3_OBJS) $(GL_LIBS) $(ASSIMP_LIB)
# Real-Bullet tests share these TUs.
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

$(TESTDIR)/test_frames: $(TESTDIR)/obj/test_frames.o $(TESTDIR)/obj/frame.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_spawn: $(TESTDIR)/obj/test_spawn.o $(TESTDIR)/obj/frame.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_attitude: $(TESTDIR)/obj/test_attitude.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_slew3d: $(TESTDIR)/obj/test_slew3d.o
	$(CXX) -o $@ $^

# Links the real physics chain (Bullet + GL).
$(TESTDIR)/test_thrust: $(TESTDIR)/obj/test_thrust.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

$(TESTDIR)/test_fuel: $(TESTDIR)/obj/test_fuel.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

$(TESTDIR)/test_power: $(TESTDIR)/obj/test_power.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

# Pure graph logic; links TCOMMON only for ~Vehicle teardown symbols.
$(TESTDIR)/test_staging: $(TESTDIR)/obj/test_staging.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

$(TESTDIR)/test_staging_dv: $(TESTDIR)/obj/test_staging_dv.o $(TESTDIR)/obj/staging.o $(TESTDIR)/obj/shipdef.o
	$(CXX) -o $@ $^

# Headless (init() rebuilds the compound, never enterWorld).
$(TESTDIR)/test_dock: $(TESTDIR)/obj/test_dock.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

# Local TestCrew : Vehicle stands in for Kerbal (eva.cpp is too heavy).
$(TESTDIR)/test_contain: $(TESTDIR)/obj/test_contain.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

$(TESTDIR)/test_inertia: $(TESTDIR)/obj/test_inertia.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

$(TESTDIR)/test_inventory: $(TESTDIR)/obj/test_inventory.o $(TESTDIR)/obj/inventory.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

$(TESTDIR)/test_rotation: $(TESTDIR)/obj/test_rotation.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

# Needs res/ (run from the repo root).
$(TESTDIR)/test_shipload: $(TESTDIR)/obj/test_shipload.o $(TESTDIR)/obj/shipdef.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_save: $(TESTDIR)/obj/test_save.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_crew: $(TESTDIR)/obj/test_crew.o $(TESTDIR)/obj/shipdef.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_calendar: $(TESTDIR)/obj/test_calendar.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_flightlog: $(TESTDIR)/obj/test_flightlog.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_science: $(TESTDIR)/obj/test_science.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_bodylimits: $(TESTDIR)/obj/test_bodylimits.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_fmt: $(TESTDIR)/obj/test_fmt.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_orbit: $(TESTDIR)/obj/test_orbit.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_orbitsample: $(TESTDIR)/obj/test_orbitsample.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_transfer: $(TESTDIR)/obj/test_transfer.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_porkchop: $(TESTDIR)/obj/test_porkchop.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_surfmap: $(TESTDIR)/obj/test_surfmap.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_eva: $(TESTDIR)/obj/test_eva.o
	$(CXX) -o $@ $^

# camera.o pins the LOD px measure against the real projection matrix.
$(TESTDIR)/test_terrain: $(TESTDIR)/obj/test_terrain.o $(TESTDIR)/obj/camera.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_drag: $(TESTDIR)/obj/test_drag.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_audio: $(TESTDIR)/obj/test_audio.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_jet: $(TESTDIR)/obj/test_jet.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_jobs: $(TESTDIR)/obj/test_jobs.o $(TESTDIR)/obj/job.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_orbitmap: $(TESTDIR)/obj/test_orbitmap.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_orbitcam: $(TESTDIR)/obj/test_orbitcam.o $(TESTDIR)/obj/camera.o
	$(CXX) -o $@ $^

# pick.cpp needs the full vehicle/physics chain (compound hull cast).
$(TESTDIR)/test_pick: $(TESTDIR)/obj/test_pick.o $(TESTDIR)/obj/pick.o $(TCOMMON_OBJS)
	$(CXX) -O2 $(LTO) -o $@ $^ $(TLIBS)

$(TESTDIR)/test_settings: $(TESTDIR)/obj/test_settings.o $(TESTDIR)/obj/settings.o $(TESTDIR)/obj/keys.o
	$(CXX) -o $@ $^

$(TESTDIR)/test_keys: $(TESTDIR)/obj/test_keys.o $(TESTDIR)/obj/keys.o
	$(CXX) -o $@ $^

# cli.o depends on generated version.h (same as gameui.o).
$(TESTDIR)/obj/cli.o: src/version.h
$(TESTDIR)/test_cli: $(TESTDIR)/obj/test_cli.o $(TESTDIR)/obj/cli.o $(TESTDIR)/obj/siminput.o
	$(CXX) -o $@ $^

TESTS = test_frames test_spawn test_attitude test_slew3d test_thrust test_fuel \
        test_power test_staging test_staging_dv test_dock test_contain test_inertia test_inventory \
        test_rotation test_shipload test_save test_crew test_calendar \
        test_flightlog test_science test_bodylimits \
        test_orbit test_orbitsample test_transfer test_porkchop test_surfmap test_eva \
        test_terrain test_drag test_audio test_jet test_jobs test_orbitmap test_orbitcam \
        test_pick test_settings test_keys test_cli test_fmt

# Short-name aliases: `make test_fuel` builds + runs one test (from the repo
# root -- some need res/). `make test` runs the whole set.
.PHONY: $(TESTS)
$(TESTS):
	$(MAKE) --no-print-directory $(TESTDIR)/$@
	$(TESTDIR)/$@

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
	$(TESTDIR)/test_staging_dv
	$(TESTDIR)/test_dock
	$(TESTDIR)/test_contain
	$(TESTDIR)/test_inertia
	$(TESTDIR)/test_inventory
	$(TESTDIR)/test_rotation
	$(TESTDIR)/test_shipload
	$(TESTDIR)/test_save
	$(TESTDIR)/test_crew
	$(TESTDIR)/test_calendar
	$(TESTDIR)/test_flightlog
	$(TESTDIR)/test_science
	$(TESTDIR)/test_bodylimits
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
	$(TESTDIR)/test_fmt

# E2E battery (e2e/run.py). Headless via xvfb-run when no display.
# JOBS=N overrides parallel cases (default 2).
JOBS ?=
E2E_JOBS = $(if $(JOBS),--jobs $(JOBS),)
# --force: this target IS the full-battery intent (run.py otherwise refuses).
.PHONY: e2e
e2e: all
	python3 e2e/run.py $(E2E_JOBS) --force --game $(BINDIR)/$(TARGET)

# Release artifacts in dist/ (gitignored). Per-OS trees use the same
# <os>-<march>-<mtune> ARCHDIR tokens.
DISTDIR=dist
LINUX_BIN=$(BUILDROOT)/linux-$(MARCH_TOK)-$(MTUNE_TOK)/release/osp
WINDOWS_BIN=$(BUILDROOT)/windows-$(MARCH_TOK)-$(MTUNE_TOK)/release/osp.exe
# Cached under tmp/. APPIMAGE_EXTRACT_AND_RUN so it works without FUSE.
APPIMAGETOOL=tmp/appimagetool-x86_64.AppImage
# Wine parity smoke (not the full matrix). run.py auto-serializes .exe games.
WINE_CASES ?= smoke vab-launch vab-launch-orbit vab-launch-body

# Package only (no test/e2e gates). Version from `version` target; dirty
# trees keep -dirty in the name.
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
	rm -rf "$$STAGE"
	@$(MAKE) --no-print-directory appimage
	@$(MAKE) --no-print-directory deb
	@VER=$$(sed -n 's/^#define VERSION "\(.*\)"/\1/p' src/version.h); \
	DEBVER=$$(printf '%s' "$$VER" | sed -e 's/^v//' -e 's/-/~/g')-1; \
	ARCH=$$(dpkg --print-architecture); \
	(cd $(DISTDIR) && sha256sum osp-$$VER-linux.tar.xz osp-$$VER-windows.zip osp-$$VER-linux+windows.tar.xz osp-$$VER-x86_64.AppImage openspaceprogram_$$DEBVER_$$ARCH.deb > SHA256SUMS); \
	ls -lh $(DISTDIR)

# AppImage via utils/make_appimage.sh. FHS AppDir so resdir::root() finds
# assets. Host-only libs (libGL, X11, Pulse, libstdc++) -- bundling libGL
# is how AppImages get black windows.
.PHONY: appimage
appimage:
	@$(MAKE) --no-print-directory version
	@$(MAKE) --no-print-directory all
	@VER=$$(sed -n 's/^#define VERSION "\(.*\)"/\1/p' src/version.h); \
	if [ -z "$$VER" ]; then \
		echo "error: no version string (src/version.h missing?)" >&2; exit 1; \
	fi; \
	bash utils/make_appimage.sh "$(LINUX_BIN)" "$$VER" $(DISTDIR) $(APPIMAGETOOL)

# Debian package via utils/make_deb.sh (also a fast path, no gates).
.PHONY: deb
deb:
	@$(MAKE) --no-print-directory version
	@$(MAKE) --no-print-directory all
	bash utils/make_deb.sh $(DISTDIR) "$(LINUX_BIN)"

# Full gates first (unit + native e2e + wine smoke), then package.
.PHONY: release
release:
	@$(MAKE) --no-print-directory test
	@$(MAKE) --no-print-directory e2e
	@$(MAKE) --no-print-directory OS=windows all
	@python3 e2e/run.py --force --game $(WINDOWS_BIN) $(WINE_CASES)
	@$(MAKE) --no-print-directory artifacts

# GL probe: Mesa 26 rejects draws in default VAO 0 -- need a real
# glGenVertexArrays VAO. Needs a display: `DISPLAY=:99 make test-gl`.
.PHONY: test-gl
test-gl:
	@mkdir -p $(TESTDIR)
	$(CXX) -O2 -std=c++20 -DGLEW_STATIC -I./middleware/sdl3/include -I./middleware/glew/include $(LTO) tests/test_vertexless.c $(GL_LIBS) -o $(TESTDIR)/test_gl_vao
	$(TESTDIR)/test_gl_vao

# Config variants (separate recursive makes, no clean between): make / debug /
# asan / tsan. imgui/implot follow OBJDIR (TSan needs them instrumented).
# Middleware is shared/uninstrumented. ASan: detect_leaks=0 hides registry
# leaks. TSan: use `make tsan-run` (needs ./tsan.supp).
.PHONY: debug
debug:
	@$(MAKE) --no-print-directory CONFIG=debug all

# windows asan: this mingw ships no ASan runtime -- probe first (the real
# attempt dies at link after a full compile).
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

# tsan is linux-only (mingw has no ThreadSanitizer runtime).
.PHONY: tsan
tsan:
ifeq ($(OS),windows)
	@echo "error: tsan is linux-only (mingw has no ThreadSanitizer runtime)" >&2; exit 1
else
	@$(MAKE) --no-print-directory CONFIG=tsan all
endif

# Run the TSan variant with tsan.supp. GAME_ARGS passes game args.
# Exit code is TSan's: 0 = clean, 66 = race reported.
GAME_ARGS ?=
.PHONY: tsan-run
tsan-run: tsan
	@XVFB=""; if [ -z "$$DISPLAY" ]; then XVFB="xvfb-run -a"; fi; \
	 TSAN_OPTIONS="suppressions=$(CURDIR)/tsan.supp" $$XVFB $(ARCHDIR)/tsan/osp_tsan $(GAME_ARGS)

.PHONY: san-clean
san-clean:
	rm -rf $(ARCHDIR)/asan $(ARCHDIR)/tsan

.PHONY: clean
# Also drops imgui/implot/test objects -- stale ones silently survive an
# ISA/LTO/PGO change otherwise. Rebuild cost is seconds.
clean:
	$(rm) $(OBJECTS) $(OBJECTS:.o=.d)
	rm -rf $(OBJDIR) $(TESTDIR)

.PHONY: remove
remove: clean
	$(rm) $(BINDIR)/$(TARGET)
	rm -rf $(TESTDIR)
ifeq ($(OS),windows)
	# windows remove leaves the linux ./osp symlink alone
	@echo "note: left the linux ./osp symlink alone"
else
	$(rm) osp
endif

# Pull in generated header deps (-MMD above). Silent when absent.
-include $(DEPS)
-include $(wildcard $(TESTDIR)/obj/*.d)
