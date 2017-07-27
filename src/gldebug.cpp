#include "gldebug.h"
#include <stdio.h>
#include <stdlib.h>
#include <GL/glew.h>
#include <iostream>

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

using namespace std;

void openglCallbackFunction(unsigned int source,
			    unsigned int type,
			    unsigned int id,
			    unsigned int severity,
			    int length,
			    const char* message,
			    const void* userParam)
{
  cout << "---------------------opengl-callback-start------------" << endl;
  cout << "message: "<< message << endl;
  cout << "type: ";
  switch (type) {
  case GL_DEBUG_TYPE_ERROR:
    cout << "ERROR";
    break;
  case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
    cout << "DEPRECATED_BEHAVIOR";
    break;
  case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
    cout << "UNDEFINED_BEHAVIOR";
    break;
  case GL_DEBUG_TYPE_PORTABILITY:
    cout << "PORTABILITY";
    break;
  case GL_DEBUG_TYPE_PERFORMANCE:
    cout << "PERFORMANCE";
    break;
  case GL_DEBUG_TYPE_OTHER:
    cout << "OTHER";
    break;
  }
  cout << endl;

  cout << "id: " << id << endl;
  cout << "severity: ";
  switch (severity){
  case GL_DEBUG_SEVERITY_LOW:
    cout << "LOW";
    break;
  case GL_DEBUG_SEVERITY_MEDIUM:
    cout << "MEDIUM";
    break;
  case GL_DEBUG_SEVERITY_HIGH:
    cout << "HIGH";
    break;
  }
  cout << endl;
  cout << "---------------------opengl-callback-end--------------" << endl;
}
