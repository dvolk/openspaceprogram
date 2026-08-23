// Minimal probe: replicate the game's exact SDL/GL context setup and report
// what the context actually is, plus which of the reverse-Z setup calls fail.
#include <SDL.h>
#include <GL/glew.h>
#include <stdio.h>

static void check(const char *what) {
    GLenum e = glGetError();
    printf("%-28s -> %s\n", what, e == GL_NO_ERROR ? "OK" : "GL ERROR");
    while (e != GL_NO_ERROR) {
        printf("   0x%04x\n", (unsigned)e);
        e = glGetError();
    }
}

int main(void) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        printf("SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    SDL_Window *w = SDL_CreateWindow("probe", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                     800, 600, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!w) { printf("SDL_CreateWindow FAILED: %s\n", SDL_GetError()); return 1; }
    SDL_GLContext c = SDL_GL_CreateContext(w);
    if (!c) { printf("SDL_GL_CreateContext FAILED: %s\n", SDL_GetError()); return 1; }

    GLenum s = glewInit();
    printf("glewInit: %s\n", glewGetErrorString(s));
    printf("VENDOR:   %s\n", (const char *)glGetString(GL_VENDOR));
    printf("RENDERER: %s\n", (const char *)glGetString(GL_RENDERER));
    printf("VERSION:  %s\n", (const char *)glGetString(GL_VERSION));
    printf("GLSL:     %s\n", (const char *)glGetString(GL_SHADING_LANGUAGE_VERSION));
    printf("GLEW_VERSION_4_3=%d  GLEW_VERSION_4_6=%d\n", (int)GLEW_VERSION_4_3, (int)GLEW_VERSION_4_6);

    int bits = -1;
    glGetIntegerv(GL_DEPTH_BITS, &bits);
    printf("DEPTH_BITS=%d (glGetError=%u)\n", bits, (unsigned)glGetError());
    int sbits = -1;
    glGetIntegerv(GL_STENCIL_BITS, &sbits);
    printf("STENCIL_BITS=%d (glGetError=%u)\n", sbits, (unsigned)glGetError());

    check("glEnable(DEPTH_TEST)");
    check("glDepthFunc(GREATER)");
    check("glDepthRange(1.0, 0.0)");
    check("glClearDepth(0.0)");
    check("glEnable(CULL_FACE)");
    check("glFrontFace(GL_CCW)");
    check("glCullFace(GL_BACK)");

    return 0;
}
