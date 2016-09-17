#pragma once

#ifdef GLDEBUG
#define check_gl_error() _check_gl_error(__FILE__, __LINE__)
#else
#define check_gl_error()
#endif

void _check_gl_error(const char *file, int line);
