CC=gcc
CXX=g++
RM=rm -f
CPPFLAGS=-std=c++11 -g -I/usr/local/include/bullet -I/usr/include/freetype2 -I/usr/include/SDL2 
LDFLAGS=-g -std=c++11 -I/usr/local/include/bullet -I/usr/include/freetype2 -I/usr/include/SDL2
LDLIBS=-lSDL2 -lGLEW -lGL -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath

SRCS=body.cpp  display.cpp  gldebug.cpp  main.cpp  mesh.cpp  obj_loader.cpp  physics.cpp  shader.cpp  shader_utils.cpp  stb_image.cpp  texture.cpp
OBJS=$(subst .cpp,.o,$(SRCS))

all: tool

tool: $(OBJS)
	$(CXX) $(LDFLAGS) -o oglplay $(OBJS) $(LDLIBS) 

depend: .depend

.depend: $(SRCS)
	rm -f ./.depend
	$(CXX) $(CPPFLAGS) -MM $^>>./.depend;

clean:
	$(RM) $(OBJS)

dist-clean: clean
	$(RM) *~ .depend

include .depend

