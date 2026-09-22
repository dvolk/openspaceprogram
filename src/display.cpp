#include <iostream>
#include <algorithm>
#include <assert.h>
#include <cstring>

#include <GL/glew.h>
#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>

#include "display.h"
#include "gldebug.h"

using namespace std;

Renderer::Renderer(int width, int height, WindowMode mode, int msaa_samples,
                   bool gl_debug)
{
    int gl_major = 4;
    int gl_minor = 5;
    bool gl_core = true;
    m_gl_debug = gl_debug;
    // SDL_WindowFlags is Uint64 in SDL3 (Uint32 in SDL2): keep the native
    // type so a flag above bit 31 can never be silently truncated.
    SDL_WindowFlags window_flags = SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE;
    if (mode == WindowMode::Borderless) {
        window_flags |= SDL_WINDOW_BORDERLESS;
    } else if (mode == WindowMode::Fullscreen) {
        // Borderless fullscreen, not exclusive: no display mode change, so
        // leaving fullscreen doesn't reconfigure the monitor. SDL3's
        // creation-time SDL_WINDOW_FULLSCREEN IS the desktop (borderless)
        // mode -- the SDL2 SDL_WINDOW_FULLSCREEN_DESKTOP.
        window_flags |= SDL_WINDOW_FULLSCREEN;
    } else if (mode == WindowMode::Exclusive) {
        // Exclusive fullscreen: ask the display for width x height (on X11
        // that's a CRTC mode change -- the only way to get a non-native
        // resolution); SDL falls back to the closest available mode if the
        // panel has no matching one. The creation flag alone only gets the
        // desktop mode in SDL3, so the exclusive mode is requested after
        // context creation (below).
        window_flags |= SDL_WINDOW_FULLSCREEN;
    }
    char window_title[] = "Open Space Program";
    m_screen_width = width;
    m_screen_height = height;
  
    // No check_gl_error() in this section: no GL context exists yet, so
    // there is no GL error state to read. (SDL_GL_SetAttribute only sets
    // creation hints.) Some stacks -- wine's WGL over llvmpipe -- answer
    // glGetError with a fresh INVALID_OPERATION on EVERY call before a
    // context exists, which the (now bounded) drain loop would print.
    // SDL3 dropped the SDL_INIT_TIMER flag: the timer is always available.
    SDL_Init(SDL_INIT_VIDEO);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    // MSAA for geometry edges, when the stack has a multisample GLX
    // visual (window creation falls back below if it doesn't). Note the
    // --postfx path renders into a non-multisampled FBO, so only the
    // default path's 3D gets the window's MSAA.
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, msaa_samples > 0 ? 1 : 0);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, msaa_samples);
    // 24-bit window depth: enough for the current reverse-Z setup. 32-bit
    // float depth (GL_DEPTH_COMPONENT32F) is only available as an
    // FBO/renderbuffer attachment, not a window surface -- see
    // tmp/depth_migration_scope.txt (Option B) for the follow-up if
    // near-surface precision (launch-pad/terrain jitter) ever bites.
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
        // SDL3's SDL_CreateWindow takes no x/y (the window manager places
        // it); SDL2's SDL_WINDOWPOS_CENTERED pair is gone.
        return SDL_CreateWindow(window_title, m_screen_width,
                                m_screen_height, window_flags);
    };
    // The check_gl_error() calls between here and SDL_GL_MakeCurrent are
    // deliberately absent: no context is CURRENT yet, and a stack like
    // wine's answers glGetError with a fresh INVALID_OPERATION on EVERY
    // call until one is -- the bounded drain would just print 16 spurious
    // lines. The first meaningful check is after MakeCurrent (below).
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
        // Exclusive: request width x height as the display mode now that the
        // context exists (on X11 the CRTC mode change; the GL context
        // survives it). A panel without a matching mode gets the closest
        // one; if even that is too small, keep the desktop mode.
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
    // Reverse-Z (replaces the Outerra logZ hack -- see
    // tmp/depth_migration_scope.txt): clip depth is 1.0 (near) -> 0.0 (far),
    // so nearer fragments have the LARGER depth value. glClipControl is core
    // in GL 4.2 (we require 4.5).
    glClipControl(GL_LOWER_LEFT, GL_ZERO_TO_ONE);
    check_gl_error();
    // Depth test flipped to match. GEQUAL (not GREATER) so a far body at
    // depth 0.0 still passes against the 0.0-cleared buffer.
    glDepthFunc(GL_GEQUAL);
    check_gl_error();
    // "Far" is now the 0.0 extreme (was 1.0); clear depth to the far value.
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
        // Leave fullscreen first (X11: restore the previous display mode),
        // then the decorations, then the window size.
        SDL_SetWindowFullscreen(m_window, false);
        SDL_SetWindowBordered(m_window, mode == WindowMode::Windowed);
        SDL_SetWindowSize(m_window, width, height);
    } else if(mode == WindowMode::Fullscreen) {
        SDL_SetWindowBordered(m_window, false);
        // Borderless fullscreen at the display's native mode (SDL3's
        // `true` = desktop mode); `width` / `height` carry over to the
        // next sized-mode switch.
        SDL_SetWindowFullscreen(m_window, true);
    } else { // WindowMode::Exclusive
        // Exclusive: ask for `width` x `height` as the display mode (on
        // X11 the CRTC mode change; the GL context survives it). A panel
        // without a matching mode gets the closest one (the constructor's
        // note).
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
    // Trust the drawable the compositor actually gave: the WM may clamp
    // a windowed size, exclusive may have fallen back. The SIZE_CHANGED
    // event (events.cpp) finishes the resize (postfx, the camera aspect).
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
    check_gl_error();
}

std::vector<Resolution> Renderer::displayModes() {
    std::vector<Resolution> out;
    // SDL3: the per-index SDL_GetNumDisplayModes/SDL_GetDisplayMode pair is
    // gone; SDL_GetFullscreenDisplayModes hands back the whole list at once
    // (one allocation, freed with SDL_free).
    const SDL_DisplayID did = SDL_GetPrimaryDisplay();
    int n = 0;
    SDL_DisplayMode **modes = SDL_GetFullscreenDisplayModes(did, &n);
    if(modes != NULL) {
        for(int i = 0; i < n; i++) {
            const SDL_DisplayMode &dm = *modes[i];
            if(dm.w > 0 && dm.h > 0) {
                const Resolution r{dm.w, dm.h, (int)dm.refresh_rate};
                // The driver may list the same (w,h,refresh) more than once
                // (different pixel formats); one entry is enough.
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
    // Some stacks keep the current mode out of the list (or report an
    // empty one); the dropdown must always offer what the display is
    // actually running. (SDL3 returns a pointer to an internal struct --
    // read it, don't free it.)
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
            // Some stacks report the current mode's refresh as 0 even
            // when the list carries one; don't add a duplicate-looking
            // entry (w x h with no Hz next to w x h @ 60Hz).
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
    // SDL3: SDL_CreateSurface(w, h, format) replaces
    // SDL_CreateRGBSurfaceWithFormat; the pixels are 4 bytes top-down, so
    // each row is w*4 bytes. glReadPixels yields [R,G,B,A] bytes (R first);
    // in SDL3 the 8888 names are the reverse of the memory order, so that
    // byte order is SDL_PIXELFORMAT_ABGR8888 (not RGBA8888 = [A,B,G,R],
    // which would scramble the channels in the saved PNG).
    SDL_Surface *surface = SDL_CreateSurface(w, h, SDL_PIXELFORMAT_ABGR8888);
    if (surface) {
        // glReadPixels is bottom-up; SDL surface is top-down. Flip vertically.
        // Force alpha opaque: the window has no transparency, so the
        // on-screen image is fully opaque, and the screenshot should match
        // it. The scene's genuinely translucent layers (atmosphere, clouds)
        // still carry partial alpha in the framebuffer; saved as-is a
        // viewer would re-blend that coverage over its own background.
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
