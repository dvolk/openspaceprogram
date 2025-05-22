#pragma once

#ifndef GLDEBUG
#define check_gl_error() _check_gl_error(__FILE__, __LINE__)
#else
#define check_gl_error()
#endif

void _check_gl_error(const char *file, int line);
void openglCallbackFunction(unsigned int source,
                            unsigned int type,
                            unsigned int id,
                            unsigned int severity,
                            int length,
                            const char* message,
                            const void* userParam);
