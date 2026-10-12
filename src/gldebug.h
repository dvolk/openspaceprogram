#pragma once

/* glGetError() drains are not free: on several drivers the call flushes the
   command buffer, which serialises the CPU against the GPU. There are ~100
   call sites and most of them sit in the draw path (a 51-part ship at the pad
   hit 95 per frame), so they are gated on a runtime flag rather than always
   live. They stay compiled in -- unlike a build-time define, the flag can turn
   them on without a rebuild; a normal frame pays one predictable branch per
   site instead of a driver round-trip. The boot sets it from --gl-debug /
   --gl-error-checks (main.cpp). The e2e runner turns the drains on for every
   case that carries FORBID GL_: _check_gl_error's "GL_<ENUM> - file:line" line
   is the only GL_ token the game prints on purpose, so without that flag the
   assertion in 179 cases would check nothing. */
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
