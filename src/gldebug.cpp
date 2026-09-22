#include "gldebug.h"
#include <stdio.h>
#include <stdlib.h>
#include <GL/glew.h>
#include <iostream>

void _check_gl_error(const char *file, int line) {
    GLenum err;
    int drained = 0;

    // Bounded drain: with no usable context (a call made before one exists,
    // or a wedged driver) some stacks return the same error from EVERY
    // glGetError -- wine's does -- and an unbounded loop here spins at 100%
    // CPU while flooding stdout. A healthy core app never queues more than
    // a couple of pending errors, so a cap of 16 is generous.
    while((err = glGetError()) != GL_NO_ERROR) {
        const char *error = "UNKNOWN_ERROR";

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
        fflush(stdout);
        if(++drained >= 16) {
            fprintf(stdout,
                    "error drain capped at %d (no usable context or "
                    "misbehaving driver?) - %s:%d\n", drained, file, line);
            fflush(stdout);
            break;
        }
        //exit(1);
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
