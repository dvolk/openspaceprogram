#include <iostream>
#include <algorithm>
#include <assert.h>
#include <cstring>
#include <string>

#include <GL/glew.h>
#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>

#include "display.h"
#include "gldebug.h"

using namespace std;

Renderer::Renderer(int width, int height, WindowMode mode, int msaa_samples,
                   bool gl_debug, int vsync)
{
    int gl_major = 4;
    int gl_minor = 5;
    bool gl_core = true;
    m_gl_debug = gl_debug;
    // SDL_WindowFlags is Uint64 in SDL3 (Uint32 in SDL2).
    SDL_WindowFlags window_flags = SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE;
    if (mode == WindowMode::Borderless) {
        window_flags |= SDL_WINDOW_BORDERLESS;
    } else if (mode == WindowMode::Fullscreen) {
        // SDL3's creation-time SDL_WINDOW_FULLSCREEN IS the desktop (borderless) mode.
        window_flags |= SDL_WINDOW_FULLSCREEN;
    } else if (mode == WindowMode::Exclusive) {
        // Creation flag alone only gets the desktop mode in SDL3; the
        // exclusive mode is requested after context creation (below).
        window_flags |= SDL_WINDOW_FULLSCREEN;
    }
    char window_title[] = "Open Space Program";
    m_screen_width = width;
    m_screen_height = height;
  
    // No check_gl_error() before a context exists: wine's WGL answers
    // glGetError with a spurious INVALID_OPERATION on every call.
    SDL_Init(SDL_INIT_VIDEO);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    // MSAA (falls back below if no multisample visual). --postfx renders into
    // a non-multisampled FBO, so only the default path's 3D gets this.
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, msaa_samples > 0 ? 1 : 0);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, msaa_samples);
    // 24-bit window depth is enough for the current reverse-Z setup.
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, gl_major);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, gl_minor);
    if(gl_core == true)
        {
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
        }
    if(m_gl_debug == true)
        {
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);
        }

    auto create_window = [&]() -> SDL_Window * {
        return SDL_CreateWindow(window_title, m_screen_width,
                                m_screen_height, window_flags);
    };
    // No check_gl_error() before MakeCurrent: no context is CURRENT yet.
    m_window = create_window();
    if(m_window == NULL) {
        // e.g. Xvfb/llvmpipe: no multisample GLX visual. Retry without MSAA
        // so headless stacks still work.
        printf("MSAA window creation failed (%s); retrying without MSAA\n", SDL_GetError());
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 0);
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 0);
        m_window = create_window();
    }
    assert(m_window);

    SDL_GLContext glcontext = SDL_GL_CreateContext(m_window);
    assert(glcontext);
    SDL_GL_MakeCurrent(m_window, glcontext);
    check_gl_error();

    if (mode == WindowMode::Exclusive) {
        // Request the display mode now that the context exists (X11 CRTC
        // change; the GL context survives it). Falls back to the closest mode.
        SDL_DisplayMode closest;
        if (SDL_GetClosestFullscreenDisplayMode(SDL_GetPrimaryDisplay(),
                                                m_screen_width,
                                                m_screen_height, 0.0f,
                                                false, &closest)) {
            SDL_SetWindowFullscreenMode(m_window, &closest);
        } else {
            printf("Exclusive %dx%d not available (%s); "
                   "falling back to desktop mode\n",
                   m_screen_width, m_screen_height, SDL_GetError());
        }
        check_gl_error();
    }

    /* Own the swap interval instead of inheriting the driver default. With
       vsync on, SwapBuffers blocking IS the render clock, so the fixed
       physics tick can only phase-lock to a rate we know; with it off, the
       --frame-cap sleep is the pacer and its accuracy is SDL_Delay's.
       After the exclusive-mode block: a CRTC mode change can reset it. */
    m_vsync = vsync;
    applySwapInterval();

    GLenum glew_status = glewInit();
    check_gl_error();
    // SDL offscreen creates an EGL context: glewInit's GLX pass returns
    // GLEW_ERROR_NO_GLX_DISPLAY, but the core GL entry points are already filled in.
    if (glew_status == GLEW_ERROR_NO_GLX_DISPLAY && GLEW_VERSION_4_5)
        {
            printf("GLEW: no GLX display (EGL/offscreen); core GL ok\n");
            glew_status = GLEW_OK;
        }
    if (glew_status != GLEW_OK)
        {
            cerr << "Error: glewInit: " << glewGetErrorString(glew_status) << endl;
        }
    if (GLEW_VERSION_4_5 == false)
        {
            cerr << "Error: your graphic card does not support OpenGL " << gl_major << "." << gl_minor << endl;
        }
    if (const GLubyte* r = glGetString(GL_RENDERER)) {
        // Not "GL_RENDERER": cases FORBID "GL_" (real GL error enums).
        printf("GL renderer: %s\n", (const char*)r);
    }

    {
        int granted = 0;
        SDL_GL_GetAttribute(SDL_GL_MULTISAMPLESAMPLES, &granted);
        printf("MSAA: %d sample(s)\n", granted);
    }

    // Trust the drawable size the compositor actually gave us (the WM may
    // have clamped a windowed size). Everything downstream reads these.
    int w = 0, h = 0;
    SDL_GetWindowSizeInPixels(m_window, &w, &h); // SDL3 rename of SDL_GL_GetDrawableSize
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
    // Reverse-Z: clip depth 1.0 (near) -> 0.0 (far). GEQUAL (not GREATER) so
    // a far body at depth 0.0 still passes against the 0.0-cleared buffer.
    glClipControl(GL_LOWER_LEFT, GL_ZERO_TO_ONE);
    check_gl_error();
    glDepthFunc(GL_GEQUAL);
    check_gl_error();
    glClearDepth(0.0);
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

void Renderer::setWindowMode(WindowMode mode, int width, int height) {
    if(mode == WindowMode::Windowed || mode == WindowMode::Borderless) {
        // Leave fullscreen first (X11: restore the previous display mode).
        SDL_SetWindowFullscreen(m_window, false);
        SDL_SetWindowBordered(m_window, mode == WindowMode::Windowed);
        SDL_SetWindowSize(m_window, width, height);
    } else if(mode == WindowMode::Fullscreen) {
        SDL_SetWindowBordered(m_window, false);
        // Borderless fullscreen at the display's native mode (SDL3 `true` = desktop).
        SDL_SetWindowFullscreen(m_window, true);
    } else { // WindowMode::Exclusive
        // Request the display mode (X11 CRTC change; falls back to closest).
        SDL_DisplayMode closest;
        if (SDL_GetClosestFullscreenDisplayMode(SDL_GetPrimaryDisplay(),
                                                width, height, 0.0f,
                                                false, &closest)) {
            SDL_SetWindowFullscreenMode(m_window, &closest);
        } else {
            printf("Exclusive %dx%d not available (%s); "
                   "staying in the current mode\n",
                   width, height, SDL_GetError());
        }
    }
    check_gl_error();
    // Trust the drawable the compositor actually gave (WM may clamp; exclusive
    // may have fallen back). The SIZE_CHANGED event finishes the resize.
    int w = 0, h = 0;
    SDL_GetWindowSizeInPixels(m_window, &w, &h);
    if(w > 0 && h > 0) {
        m_screen_width = w;
        m_screen_height = h;
    }
    glViewport(0, 0, m_screen_width, m_screen_height);
    check_gl_error();
    static const char *mode_names[] = {"windowed", "borderless",
                                       "fullscreen", "exclusive"};
    printf("display mode: %dx%d (%s)\n", m_screen_width, m_screen_height,
           mode_names[static_cast<size_t>(mode)]);
    // A mode change is what resets the swap interval, and the panel rate can
    // change with it (the Settings dropdown lists "%dx%d @ %dHz"), so re-apply
    // and re-print rather than leave the startup line stale.
    applySwapInterval();
    check_gl_error();
}

std::vector<Resolution> Renderer::displayModes() {
    std::vector<Resolution> out;
    // SDL3: SDL_GetFullscreenDisplayModes returns the whole list at once.
    const SDL_DisplayID did = SDL_GetPrimaryDisplay();
    int n = 0;
    SDL_DisplayMode **modes = SDL_GetFullscreenDisplayModes(did, &n);
    if(modes != NULL) {
        for(int i = 0; i < n; i++) {
            const SDL_DisplayMode &dm = *modes[i];
            if(dm.w > 0 && dm.h > 0) {
                const Resolution r{dm.w, dm.h, (int)dm.refresh_rate};
                // Dedup (the driver may list the same mode more than once).
                bool have = false;
                for(size_t j = 0; j < out.size(); j++) {
                    if(out[j].width == r.width && out[j].height == r.height
                       && out[j].refresh == r.refresh) {
                        have = true;
                        break;
                    }
                }
                if(!have) { out.push_back(r); }
            }
        }
        SDL_free(modes);
    }
    // Some stacks keep the current mode out of the list; the dropdown must
    // always offer what the display is actually running.
    const SDL_DisplayMode *cur = SDL_GetCurrentDisplayMode(did);
    if(cur != NULL && cur->w > 0 && cur->h > 0) {
        bool have = false;
        int same_wh_refresh = 0;
        for(size_t i = 0; i < out.size(); i++) {
            if(out[i].width == cur->w && out[i].height == cur->h) {
                if(out[i].refresh == (int)cur->refresh_rate) {
                    have = true;
                    break;
                }
                same_wh_refresh = out[i].refresh;
            }
        }
        if(!have) {
            // Some stacks report refresh as 0; don't add a duplicate-looking entry.
            out.push_back(Resolution{cur->w, cur->h,
                                     (int)cur->refresh_rate
                                     ? (int)cur->refresh_rate
                                     : same_wh_refresh});
        }
    }
    std::sort(out.begin(), out.end(),
              [](const Resolution &a, const Resolution &b) {
                  if(a.width != b.width) { return a.width < b.width; }
                  if(a.height != b.height) { return a.height < b.height; }
                  return a.refresh < b.refresh;
              });
    return out;
}

void Renderer::applySwapInterval() {
    // The driver may not grant the request, so read back what it actually
    // gave us rather than assume. A failed read-back gets its own sentinel:
    // -1 is a legitimate value (adaptive vsync), so it must not double as
    // "unknown".
    if(!SDL_GL_SetSwapInterval(m_vsync)) {
        printf("vsync: SDL_GL_SetSwapInterval(%d) failed (%s)\n",
               m_vsync, SDL_GetError());
    }
    const int kSwapUnknown = -999;
    int granted = kSwapUnknown;
    if(!SDL_GL_GetSwapInterval(&granted)) { granted = kSwapUnknown; }
    // A granted interval > 0 only means SwapBuffers BLOCKS where there is a
    // retrace to wait for: the headless/offscreen EGL path reports 1 and then
    // never waits (measured present ~0.02 ms). Claim the wait only when a
    // panel rate is actually known.
    const int hz = currentRefresh();
    printf("vsync: requested %d, granted %s (%s); display %s\n",
           m_vsync,
           granted == kSwapUnknown ? "unknown" : std::to_string(granted).c_str(),
           hz <= 0                 ? "no panel rate: pacer unverified" :
           granted >  0            ? "SwapBuffers blocks N refreshes" :
           granted == 0            ? "immediate, no retrace wait" :
           granted  <  0           ? "adaptive" :
                                     "read-back failed",
           hz > 0 ? std::string(std::to_string(hz) + " Hz").c_str() : "unknown");
}

int Renderer::currentRefresh() {
    const SDL_DisplayMode *cur = SDL_GetCurrentDisplayMode(SDL_GetPrimaryDisplay());
    if(cur != NULL) {
        return (int)cur->refresh_rate;
    }
    return 0;
}

int Renderer::msaaSamples() const {
    int granted = 0;
    SDL_GL_GetAttribute(SDL_GL_MULTISAMPLESAMPLES, &granted);
    return granted;
}

void Renderer::Clear(float r, float g, float b, float a)
{
    check_gl_error();
    glClearColor(r, g, b, a);
    check_gl_error();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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
    // glReadPixels yields [R,G,B,A]; SDL3's 8888 names are the reverse of
    // memory order, so ABGR8888 is the correct match (RGBA8888 would scramble).
    SDL_Surface *surface = SDL_CreateSurface(w, h, SDL_PIXELFORMAT_ABGR8888);
    if (surface) {
        // glReadPixels is bottom-up; SDL surface is top-down. Flip vertically.
        // Force alpha opaque so a viewer doesn't re-blend translucent layers.
        for (int y = 0; y < h; y++) {
            const unsigned char *src = pixels + (h - 1 - y) * w * 4;
            unsigned char *dst = (unsigned char *)surface->pixels + y * surface->pitch;
            memcpy(dst, src, w * 4);
            for (int x = 0; x < w; x++) {
                dst[x * 4 + 3] = 255;
            }
        }
        if (IMG_SavePNG(surface, filename)) {
            printf("Screenshot saved: %s (%dx%d)\n", filename, w, h);
            ok = true;
        } else {
            printf("Failed to save screenshot %s: %s\n", filename, SDL_GetError());
        }
        SDL_DestroySurface(surface);
    }
    delete[] pixels;
    return ok;
}
