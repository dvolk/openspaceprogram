#pragma once

#define check_gl_error() _check_gl_error(__FILE__, __LINE__)
void _check_gl_error(const char *file, int line);
