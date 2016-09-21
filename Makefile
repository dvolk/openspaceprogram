TARGET=osp

#CXX_OPT=-march=native -flto -O2 -fprofile-arcs -ftest-coverage -fprofile-dir=/home/dv/src/my/openspaceprogram/data/pgo -fprofile-generate=/home/dv/src/my/openspaceprogram/data/pgo
#LD_OPT=-O2 -flto -fprofile-arcs
#SANITIZE=-g3 -fsanitize=address -fsanitize=leak -fsanitize=undefined

CXX= g++
CXXFLAGS=-O2 $(CXX_OPT) $(SANITIZE) -Wall -Wextra -Wpedantic -std=c++11 -I/usr/local/include/bullet -I/usr/include/SDL2 #-I../../../lib/imgui

LINKER=g++ -O2 $(LD_OPT) $(SANITIZE) -o
LDLIBS=-lSDL2 -lGLEW -lGL -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath -lassimp

# you'll need to build these manually
IMGUI_OBJS=src/imgui_impl_sdl/imgui_impl_sdl.o ../../lib/imgui/imgui.o ../../lib/imgui/imgui_draw.o

LFLAGS=$(LTO) -Wall $(LDLIBS) $(IMGUI_LIBS)

SRCDIR=src
OBJDIR=obj
BINDIR=.

SOURCES := $(wildcard $(SRCDIR)/*.cpp)
INCLUDES := $(wildcard $(SRCDIR)/*.h)
OBJECTS  := $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
rm = rm -f

$(BINDIR)/$(TARGET): $(OBJECTS)
	$(LINKER) $@ $(IMGUI_OBJS) $(OBJECTS) $(LFLAGS)

$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PHONEY: clean
clean:
	$(rm) $(OBJECTS)

.PHONEY: remove
remove: clean
	$(rm) $(BINDIR)/$(TARGET)
