TARGET=osp

#CXX_OPT=-march=native -flto -O2 -fprofile-arcs -ftest-coverage -fprofile-dir=/home/dv/src/my/openspaceprogram/data/pgo -fprofile-generate=/home/dv/src/my/openspaceprogram/data/pgo
#LD_OPT=-O2 -flto -fprofile-arcs
#SANITIZE=-g3 -fsanitize=address -fsanitize=leak -fsanitize=undefined

CXX= g++
CXXFLAGS=-O2 $(CXX_OPT) $(SANITIZE) -Wall -Wextra -Wpedantic -std=c++11 -I./middleware/glm/ -I./middleware/bullet3/ -I./middleware/bullet3/bullet -I./middleware/imgui/ -I/usr/include/SDL2

LINKER=g++ -O2 $(LD_OPT) $(SANITIZE) -o
LDLIBS=-lSDL2_image -lSDL2 -lGLEW -lGL -lassimp

# you'll need to build these manually

# clone imgui in ./middleware
# cd ./middleware/imgui/examples/sdl_opengl3_example/
# sed -i -- 's/#include <GL\/gl3w.h>/#include <GL\/glew.h>/g' imgui_impl_sdl_gl3.cpp
# c++ `sdl2-config --cflags` -I ../.. -c ../../imgui.cpp -fPIC  `sdl2-config --libs`
# c++ `sdl2-config --cflags` -I ../.. -c ../../imgui_draw.cpp -fPIC  `sdl2-config --libs`
# c++ `sdl2-config --cflags` -I ../.. -c imgui_impl_sdl_gl3.cpp -fPIC  `sdl2-config --libs` -lGL -lGLEW
IMGUI_OBJS=./middleware/imgui/examples/sdl_opengl3_example/imgui_impl_sdl_gl3.o ./middleware/imgui/examples/sdl_opengl3_example/imgui_draw.o ./middleware/imgui/examples/sdl_opengl3_example/imgui.o
# clone bullet3 in ./middleware
# cd ./middleware/bullet3
# ln -s bullet src
# build it with cmake with double precision enabled
BULLET3_OBJS=./middleware/bullet3/build/src/BulletDynamics/libBulletDynamics.a ./middleware/bullet3/build/src/BulletCollision/libBulletCollision.a ./middleware/bullet3/build/src/BulletSoftBody/libBulletSoftBody.a ./middleware/bullet3/build/src/Bullet3Geometry/libBullet3Geometry.a ./middleware/bullet3/build/src/BulletInverseDynamics/libBulletInverseDynamics.a ./middleware/bullet3/build/src/Bullet3Common/libBullet3Common.a ./middleware/bullet3/build/src/Bullet3Collision/libBullet3Collision.a ./middleware/bullet3/build/src/LinearMath/libLinearMath.a ./middleware/bullet3/build/src/Bullet3Serialize/Bullet2FileLoader/libBullet2FileLoader.a ./middleware/bullet3/build/src/Bullet3OpenCL/libBullet3OpenCL_clew.a ./middleware/bullet3/build/src/Bullet3Dynamics/libBullet3Dynamics.a

LFLAGS=$(LTO) -Wall $(LDLIBS) $(IMGUI_LIBS) $(BULLET3_OBJS)

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
