#include <iostream>
#include <assert.h>
#include <cstring>

#include <GL/glew.h>
#include "SDL2/SDL.h"
#include "SDL2/SDL_image.h"

#include "display.h"
#include "gldebug.h"

using namespace std;

Renderer::Renderer(int width, int height, WindowMode mode, bool gl_debug)
{
    int gl_major = 4;
    int gl_minor = 5;
    bool gl_core = true;
    m_gl_debug = gl_debug;
    Uint32 window_flags = SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE;
    if (mode == WindowMode::Borderless) {
        window_flags |= SDL_WINDOW_BORDERLESS;
    } else if (mode == WindowMode::Fullscreen) {
        // Borderless fullscreen, not exclusive: no display mode change, so
        // leaving fullscreen doesn't reconfigure the monitor.
        window_flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
    } else if (mode == WindowMode::Exclusive) {
        // Exclusive fullscreen: ask the display for width x height (on X11
        // that's a CRTC mode change -- the only way to get a non-native
        // resolution); SDL falls back to covering the current mode if the
        // panel has no matching mode.
        window_flags |= SDL_WINDOW_FULLSCREEN;
    }
    char window_title[] = "Open Space Program";
    m_screen_width = width;
    m_screen_height = height;
  
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    check_gl_error();
    // 4x MSAA for geometry edges, when the stack has a multisample GLX
    // visual (window creation falls back below if it doesn't). Note the
    // --postfx path renders into a non-multisampled FBO, so only the
    // default path's 3D gets the window's MSAA.
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    check_gl_error();
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
    check_gl_error();
    // 32-bit float depth was tried (see git history): on this stack window
    // creation fails with DEPTH 32 + STENCIL 8, and it wouldn't have helped
    // anyway -- the log-z values come from a float32 varying, and the
    // skirt hiding uses the stencil, not depth precision.
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    check_gl_error();
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    check_gl_error();
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, gl_major);
    check_gl_error();
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, gl_minor);
    check_gl_error();
    if(gl_core == true)
        {
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
            check_gl_error();
        }
    if(m_gl_debug == true)
        {
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);
            check_gl_error();
        }

    SDL_DisplayMode current;
    SDL_GetCurrentDisplayMode(0, &current);
    check_gl_error();

    auto create_window = [&]() -> SDL_Window * {
        return SDL_CreateWindow(window_title, SDL_WINDOWPOS_CENTERED,
                                SDL_WINDOWPOS_CENTERED, m_screen_width,
                                m_screen_height, window_flags);
    };
    m_window = create_window();
    check_gl_error();
    if(m_window == NULL) {
        // e.g. Xvfb/llvmpipe: no multisample GLX visual (same family of
        // quirk as the DEPTH 32 + STENCIL 8 failure above). Retry without
        // MSAA so headless stacks still work.
        printf("MSAA window creation failed (%s); retrying without MSAA\n", SDL_GetError());
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 0);
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 0);
        m_window = create_window();
        check_gl_error();
    }
    assert(m_window);

    SDL_GLContext glcontext = SDL_GL_CreateContext(m_window);
    check_gl_error();
    assert(glcontext);

    GLenum glew_status = glewInit();
    check_gl_error();

    if (glew_status != GLEW_OK)
        {
            cerr << "Error: glewInit: " << glewGetErrorString(glew_status) << endl;
        }
    if (GLEW_VERSION_4_5 == false)
        {
            cerr << "Error: your graphic card does not support OpenGL " << gl_major << "." << gl_minor << endl;
        }

    {
        int granted = 0;
        SDL_GL_GetAttribute(SDL_GL_MULTISAMPLESAMPLES, &granted);
        printf("MSAA: %d sample(s)\n", granted);
    }

    // Trust the drawable size the compositor actually gave us: the WM may
    // have clamped a windowed size to the work area, and fullscreen uses the
    // display mode regardless of the requested size. Everything downstream
    // (viewport, camera aspect, screenshots) reads m_screen_width/height.
    int w = 0, h = 0;
    SDL_GL_GetDrawableSize(m_window, &w, &h); // void in SDL2
    if (w > 0 && h > 0) {
        m_screen_width = w;
        m_screen_height = h;
    }
    glViewport(0, 0, m_screen_width, m_screen_height);
    check_gl_error();
    static const char *mode_names[] = {"windowed", "borderless",
                                       "fullscreen", "exclusive"};
    printf("window: %dx%d (%s)\n", m_screen_width, m_screen_height,
           mode_names[static_cast<size_t>(mode)]);

    glEnable(GL_DEPTH_TEST);
    check_gl_error();
    glDepthFunc(GL_LESS);
    check_gl_error();
    glEnable(GL_CULL_FACE);
    check_gl_error();
    glFrontFace(GL_CCW);
    check_gl_error();
    glCullFace(GL_BACK);
    check_gl_error();

    if(m_gl_debug == true)
        {
            std::cout << "Register OpenGL debug callback " << endl;
            glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
            check_gl_error();

            GLuint unusedIds = 0;
            glDebugMessageControl(GL_DONT_CARE,
                                  GL_DONT_CARE,
                                  GL_DONT_CARE,
                                  0,
                                  &unusedIds,
                                  false);
            check_gl_error();
            glDebugMessageCallback(openglCallbackFunction, nullptr);
            check_gl_error();
        }
}

Renderer::~Renderer()
{
}

void Renderer::onResize(int width, int height) {
    printf("Renderer::onResize(): old size: %d %d new size: %d %d\n", m_screen_width, m_screen_height, width, height);
    m_screen_width = width;
    m_screen_height = height;
    check_gl_error();
    glViewport(0, 0, m_screen_width, m_screen_height);
    check_gl_error();

}
void Renderer::Clear(float r, float g, float b, float a)
{
    check_gl_error();
    glClearColor(r, g, b, a);
    check_gl_error();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    check_gl_error();
}

void Renderer::SwapBuffers()
{
    check_gl_error();
    SDL_GL_SwapWindow(m_window);
    check_gl_error();
}

bool Renderer::SaveScreenshot(const char *filename)
{
    const int w = m_screen_width;
    const int h = m_screen_height;

    unsigned char *pixels = new unsigned char[w * h * 4];
    // read the current draw buffer (what we just rendered)
    glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    check_gl_error();

    bool ok = false;
    SDL_Surface *surface = SDL_CreateRGBSurfaceWithFormat(0, w, h, 32, SDL_PIXELFORMAT_RGBA32);
    if (surface) {
        // glReadPixels is bottom-up; SDL surface is top-down. Flip vertically.
        for (int y = 0; y < h; y++) {
            const unsigned char *src = pixels + (h - 1 - y) * w * 4;
            unsigned char *dst = (unsigned char *)surface->pixels + y * surface->pitch;
            memcpy(dst, src, w * 4);
        }
        if (IMG_SavePNG(surface, filename) == 0) {
            printf("Screenshot saved: %s (%dx%d)\n", filename, w, h);
            ok = true;
        } else {
            printf("Failed to save screenshot %s: %s\n", filename, SDL_GetError());
        }
        SDL_FreeSurface(surface);
    }
    delete[] pixels;
    return ok;
}
