#include "gldebug.h"
#include <stdio.h>
#include <stdlib.h>
#include <GL/glew.h>

void _check_gl_error(const char *file, int line) {
  GLenum err;

  while((err = glGetError()) != GL_NO_ERROR) {
    const char *error;

    switch(err) {
    case GL_INVALID_OPERATION:
      error = "INVALID_OPERATION";
      break;
    case GL_INVALID_ENUM:
      error = "INVALID_ENUM";
      break;
    case GL_INVALID_VALUE:
      error = "INVALID_VALUE";
      break;
    case GL_OUT_OF_MEMORY:
      error = "OUT_OF_MEMORY";
      break;
    case GL_INVALID_FRAMEBUFFER_OPERATION:
      error = "INVALID_FRAMEBUFFER_OPERATION";
      break;
    }

    fprintf(stdout, "GL_%s - %s:%d\n", error, file, line);
    exit(1);
  }
}
