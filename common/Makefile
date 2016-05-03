LDLIBS=-lm -lglut -lGLEW -lGL -lfreetype
CXXFLAGS=-I/usr/include/freetype2 -g -Wall
all: text
text: ../common/shader_utils.o
clean:
	rm -f *.o text
.PHONY: all clean
