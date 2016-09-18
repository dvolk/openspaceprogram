TARGET=osp

LTO=-flto # link-time optimization
CXX= g++
#SANITIZE=-g3 -fsanitize=address -fsanitize=leak -fsanitize=undefined
CXXFLAGS=$(LTO) -g3 -O2 -Wall -Wextra -Wpedantic -std=c++11 $(SANITIZE) -I/usr/local/include/bullet -I/usr/include/freetype2 -I/usr/include/SDL2 #-I../../../lib/imgui

LINKER=g++ -o
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
