#pragma once

/* glGetError() drains flush the command buffer on several drivers, serialising
   the CPU against the GPU, and most call sites sit in the draw path -- so they
   are gated on a runtime flag rather than always live. Compiled in always
   (unlike a build-time define) so they can be enabled without a rebuild. Boot
   sets it from --gl-debug / --gl-error-checks; the e2e runner turns it on for
   cases that carry FORBID GL_. */
void set_gl_error_checks(bool on);
bool gl_error_checks_on();

void _check_gl_error(const char *file, int line);
#define check_gl_error() \
    do { if(gl_error_checks_on()) _check_gl_error(__FILE__, __LINE__); } while(0)

void openglCallbackFunction(unsigned int source,
                            unsigned int type,
                            unsigned int id,
                            unsigned int severity,
                            int length,
                            const char* message,
                            const void* userParam);
