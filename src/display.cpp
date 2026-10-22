#include <iostream>
#include <algorithm>
#include <assert.h>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <GL/glew.h>
#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>

#include "display.h"
#include "gldebug.h"

using namespace std;

/* PNG encoding of a 1080p frame costs ~200 ms (4x that at 4K), all of it
   compression: doing it on the main thread froze the game for the whole write.
   The grab itself stays on the GL thread (a sync we cannot avoid); the surface
   is then handed here to be flipped and encoded. RGB, not RGBA: the alpha byte
   of a read-back framebuffer is not meaningful and an alpha channel only slowed
   compression and bloated the file. */
class ScreenshotWriter
{
public:
    ScreenshotWriter() : worker_(&ScreenshotWriter::run, this) {}
    ~ScreenshotWriter() {
        {
            std::lock_guard<std::mutex> lk(mu_);
            stop_ = true;
        }
        cv_.notify_one();
        if(worker_.joinable()) { worker_.join(); }   // drains queued shots
    }
    ScreenshotWriter(const ScreenshotWriter &) = delete;
    ScreenshotWriter &operator=(const ScreenshotWriter &) = delete;

    // Takes ownership of the surface (bottom-up as read); it is flipped, encoded
    // and freed on the worker. False when the queue is full.
    bool push(SDL_Surface *surface, std::string path) {
        {
            std::lock_guard<std::mutex> lk(mu_);
            if(queue_.size() >= kMaxPending) { return false; }
            queue_.emplace_back(surface, std::move(path));
        }
        cv_.notify_one();
        return true;
    }

    Renderer::ShotState poll() {
        std::lock_guard<std::mutex> lk(mu_);
        if(saved_ > 0) { saved_--; return Renderer::ShotState::Saved; }
        if(failed_ > 0) { failed_--; return Renderer::ShotState::Failed; }
        return Renderer::ShotState::None;
    }

private:
    // Three queued plus the one being encoded: 4 full frames resident (8 MB
    // each at 1080p, 33 MB at 4K), plus the RGB copy IMG_SavePNG makes.
    static constexpr size_t kMaxPending = 3;

    void run() {
        for(;;) {
            SDL_Surface *surface = nullptr;
            std::string path;
            {
                std::unique_lock<std::mutex> lk(mu_);
                cv_.wait(lk, [this]{ return stop_ || !queue_.empty(); });
                // stop_ with an empty queue: shutdown, unless shots are still
                // queued (a user who pressed F12 just before quitting still
                // gets the file).
                if(queue_.empty()) { return; }
                surface = queue_.front().first;
                path = std::move(queue_.front().second);
                queue_.pop_front();
            }
            const int w = surface->w, h = surface->h;
            // glReadPixels is bottom-up, the PNG top-down. Row swaps: cheap, but
            // 33 MB at 4K, so not on the frame that took the shot.
            SDL_FlipSurface(surface, SDL_FLIP_VERTICAL);
            // IMG_SavePNG yields true or sets SDL_GetError.
            const bool ok = IMG_SavePNG(surface, path.c_str());
            if(ok) {
                printf("Screenshot saved: %s (%dx%d)\n", path.c_str(), w, h);
            } else {
                printf("Failed to save screenshot %s: %s\n", path.c_str(), SDL_GetError());
            }
            SDL_DestroySurface(surface);
            {
                std::lock_guard<std::mutex> lk(mu_);
                (ok ? saved_ : failed_)++;
            }
        }
    }

    std::mutex mu_;
    std::condition_variable cv_;
    std::deque<std::pair<SDL_Surface *, std::string>> queue_;
    int saved_ = 0;
    int failed_ = 0;
    bool stop_ = false;
    std::thread worker_;
};

/* Where the 3D pass renders (#263): multisampled RGBA8 colour over 32-bit FLOAT
   depth, resolved into a sampleable texture. The float depth is the point. The
   projection writes zNear/d (camera.cpp), which on the window's 24-bit unorm
   buffer has a uniform 6e-8 LSB, so every body past ~1.7e7 m underflowed onto
   the 0.0 clear value and occlusion fell back to g.sys.bodies order. Under
   float, 1/d keeps 2^-24 *relative* precision at any distance.

   The window never depth-tests again: it receives the resolved image plus the
   UI, so the window's depth size and sample count stop mattering. */
class SceneTarget
{
public:
    SceneTarget() : m_fbo(0), m_colorRB(0), m_depthRB(0),
                    m_resolveFbo(0), m_resolveTex(0),
                    m_width(0), m_height(0),
                    m_requestedSamples(-1), m_samples(0), m_depthBits(0) {}
    ~SceneTarget() { Destroy(); }

    void Resize(int width, int height, int samples)
    {
        if(width <= 0 || height <= 0) { return; }   // minimized: keep the old size
        if(m_fbo != 0 && width == m_width && height == m_height
           && samples == m_requestedSamples) {
            return;
        }
        Destroy();
        m_width = width;
        m_height = height;
        m_requestedSamples = samples;

        GLint maxSamples = 0;
        glGetIntegerv(GL_MAX_SAMPLES, &maxSamples);
        m_samples = std::clamp(samples, 0, (int)maxSamples);

        bool ok = Build();
        if(!ok && m_samples > 0) {
            // Some drivers refuse a multisampled float-depth renderbuffer.
            // Losing the AA is survivable; losing the depth is not.
            fprintf(stderr, "scene target: %d-sample incomplete; retrying "
                            "without MSAA\n", m_samples);
            Destroy();
            m_samples = 0;
            ok = Build();
        }
        assert(ok);   // nothing renders without this target
        check_gl_error();
    }

    void Bind()
    {
        glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
        glViewport(0, 0, m_width, m_height);
    }

    // MSAA colour -> the resolve texture. A no-op without MSAA, where the
    // texture is the colour attachment itself.
    void Resolve()
    {
        if(m_samples == 0) { return; }
        glBindFramebuffer(GL_READ_FRAMEBUFFER, m_fbo);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_resolveFbo);
        glBlitFramebuffer(0, 0, m_width, m_height,
                          0, 0, m_width, m_height,
                          GL_COLOR_BUFFER_BIT, GL_NEAREST);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        check_gl_error();
    }

    GLuint texture() const { return m_resolveTex; }
    int samples() const { return m_samples; }
    int depthBits() const { return m_depthBits; }

private:
    bool Build()
    {
        glGenTextures(1, &m_resolveTex);
        glBindTexture(GL_TEXTURE_2D, m_resolveTex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, m_width, m_height, 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);

        glGenFramebuffers(1, &m_fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
        if(m_samples > 0) {
            glGenRenderbuffers(1, &m_colorRB);
            glBindRenderbuffer(GL_RENDERBUFFER, m_colorRB);
            glRenderbufferStorageMultisample(GL_RENDERBUFFER, m_samples,
                                             GL_RGBA8, m_width, m_height);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                      GL_RENDERBUFFER, m_colorRB);
        } else {
            // No MSAA: draw straight into the resolve texture, so Resolve()
            // costs nothing instead of a full-screen copy.
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                   GL_TEXTURE_2D, m_resolveTex, 0);
        }

        glGenRenderbuffers(1, &m_depthRB);
        glBindRenderbuffer(GL_RENDERBUFFER, m_depthRB);
        if(m_samples > 0) {
            glRenderbufferStorageMultisample(GL_RENDERBUFFER, m_samples,
                                             GL_DEPTH_COMPONENT32F,
                                             m_width, m_height);
        } else {
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32F,
                                  m_width, m_height);
        }
        glBindRenderbuffer(GL_RENDERBUFFER, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                  GL_RENDERBUFFER, m_depthRB);

        GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if(status != GL_FRAMEBUFFER_COMPLETE) {
            fprintf(stderr, "scene target: framebuffer incomplete (%u) at "
                            "%dx%d, %d sample(s)\n",
                    status, m_width, m_height, m_samples);
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            return false;
        }

        GLint bits = 0;
        glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                              GL_FRAMEBUFFER_ATTACHMENT_DEPTH_SIZE,
                                              &bits);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        m_depthBits = bits;
        /* A driver that silently demotes 32F puts us back on the 1/d collapse
           with no symptom until a moon draws through its planet. */
        assert(m_depthBits >= 32);
        check_gl_error();
        if(m_samples == 0) { return true; }   // the texture IS the draw target

        // The blit destination. Only MSAA needs one: with no MSAA the scene
        // already draws into m_resolveTex, and Resolve() is a no-op.
        glGenFramebuffers(1, &m_resolveFbo);
        glBindFramebuffer(GL_FRAMEBUFFER, m_resolveFbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, m_resolveTex, 0);
        status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        if(status != GL_FRAMEBUFFER_COMPLETE) {
            fprintf(stderr, "scene target: resolve framebuffer incomplete (%u) "
                            "at %dx%d\n", status, m_width, m_height);
            return false;
        }
        check_gl_error();
        return true;
    }

    // Leaves the dimensions and sample count alone: Resize's no-MSAA retry
    // rebuilds at the size it already recorded.
    void Destroy()
    {
        if(m_fbo != 0) glDeleteFramebuffers(1, &m_fbo);
        if(m_resolveFbo != 0) glDeleteFramebuffers(1, &m_resolveFbo);
        if(m_colorRB != 0) glDeleteRenderbuffers(1, &m_colorRB);
        if(m_depthRB != 0) glDeleteRenderbuffers(1, &m_depthRB);
        if(m_resolveTex != 0) glDeleteTextures(1, &m_resolveTex);
        m_fbo = m_resolveFbo = m_colorRB = m_depthRB = m_resolveTex = 0;
    }

    GLuint m_fbo;         // the scene draws here
    GLuint m_colorRB;     // multisample colour (0 without MSAA)
    GLuint m_depthRB;     // 32F, matching the draw target's sample count
    GLuint m_resolveFbo;  // colour = m_resolveTex (0 without MSAA)
    GLuint m_resolveTex;
    int m_width;
    int m_height;
    int m_requestedSamples;   // what the caller asked for, before clamping
    int m_samples;            // what the driver gave (0 = no MSAA)
    int m_depthBits;
};

Renderer::Renderer(int width, int height, WindowMode mode, int msaa_samples,
                   bool gl_debug, int vsync)
{
    int gl_major = 4;
    int gl_minor = 5;
    bool gl_core = true;
    m_gl_debug = gl_debug;
    SDL_WindowFlags window_flags = SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE;
    if (mode == WindowMode::Borderless) {
        window_flags |= SDL_WINDOW_BORDERLESS;
    } else if (mode == WindowMode::Fullscreen) {
        // SDL3's creation-time SDL_WINDOW_FULLSCREEN IS the desktop (borderless) mode.
        window_flags |= SDL_WINDOW_FULLSCREEN;
    } else if (mode == WindowMode::Exclusive) {
        // The creation flag alone only gets desktop mode; exclusive is requested
        // after context creation (below).
        window_flags |= SDL_WINDOW_FULLSCREEN;
    }
    char window_title[] = "Open Space Program";
    m_screen_width = width;
    m_screen_height = height;

    // No check_gl_error() before a context exists: wine's WGL answers glGetError
    // with a spurious INVALID_OPERATION.
    SDL_Init(SDL_INIT_VIDEO);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    /* No SDL_GL_DEPTH_SIZE or MSAA here: the scene renders into SceneTarget
       (float depth, FBO multisampling), so the window is only where the
       composite and the UI land. It needs neither. */
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

    // No check_gl_error() before MakeCurrent: no context is CURRENT yet.
    m_window = SDL_CreateWindow(window_title, m_screen_width,
                                m_screen_height, window_flags);
    assert(m_window);

    SDL_GLContext glcontext = SDL_GL_CreateContext(m_window);
    assert(glcontext);
    SDL_GL_MakeCurrent(m_window, glcontext);
    check_gl_error();

    if (mode == WindowMode::Exclusive) {
        // Request the display mode now the context exists (X11 CRTC change; the
        // GL context survives it). Falls back to the closest mode.
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

    /* Own the swap interval rather than inherit the driver default. With vsync on,
       SwapBuffers blocking IS the render clock, so the fixed physics tick can only
       phase-lock to a rate we know; off, the --frame-cap sleep paces at
       SDL_Delay's accuracy. After the exclusive-mode block: a CRTC change can
       reset it. */
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

    // Trust the drawable size the compositor gave us (the WM may have clamped a
    // windowed size). Everything downstream reads these.
    int w = 0, h = 0;
    SDL_GetWindowSizeInPixels(m_window, &w, &h);
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

    // The scene target exists before the first frame, so nothing has to ask
    // whether it has been built yet.
    m_msaa = msaa_samples;
    m_scene = std::make_unique<SceneTarget>();
    m_scene->Resize(m_screen_width, m_screen_height, m_msaa);
    // Pinned by e2e 33: the sample count and depth width the scene actually got.
    printf("MSAA: %d sample(s), %d-bit depth\n",
           m_scene->samples(), m_scene->depthBits());

    check_gl_error();
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
            /* count=0 + all-GL_DONT_CARE covers EVERY message, and the last argument
               says whether to enable or DISABLE that set: false here silenced the
               whole debug stream, so --gl-debug registered a callback that could
               never fire. */
            glDebugMessageControl(GL_DONT_CARE,
                                  GL_DONT_CARE,
                                  GL_DONT_CARE,
                                  0,
                                  &unusedIds,
                                  true);
            check_gl_error();
            glDebugMessageCallback(openglCallbackFunction, nullptr);
            check_gl_error();
        }
}

Renderer::~Renderer()
{
    /* m_scene deletes GL objects, so this relies on the context still being
       current: main never destroys it (and nothing before here does either).
       A future SDL_GL_DestroyContext on the way out has to come AFTER the
       Renderer is gone. */
}

void Renderer::onResize(int width, int height) {
    printf("Renderer::onResize(): old size: %d %d new size: %d %d\n", m_screen_width, m_screen_height, width, height);
    m_screen_width = width;
    m_screen_height = height;
    check_gl_error();
    glViewport(0, 0, m_screen_width, m_screen_height);
    check_gl_error();
    if(m_scene) { m_scene->Resize(width, height, m_msaa); }
    check_gl_error();
}

void Renderer::BeginScene()
{
    // The 3D pass never touches the window's framebuffer (#263): it needs float
    // depth, and the window's depth is whatever the visual happened to give.
    assert(m_scene);
    m_scene->Bind();
}

void Renderer::EndScene()
{
    assert(m_scene);
    m_scene->Resolve();
    // Hand the window back: postfx composites into it and the UI draws on top.
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, m_screen_width, m_screen_height);
    check_gl_error();
}

unsigned int Renderer::sceneTexture() const
{
    assert(m_scene);
    return m_scene->texture();
}

int Renderer::grantedMsaaSamples() const
{
    return m_scene ? m_scene->samples() : 0;
}

void Renderer::setMsaaSamples(int samples)
{
    if(!m_scene) { return; }
    m_msaa = samples;
    m_scene->Resize(m_screen_width, m_screen_height, samples);
    // The scene target is an FBO, not the window's visual, so this is live.
    printf("MSAA: %d sample(s), %d-bit depth\n",
           m_scene->samples(), m_scene->depthBits());
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
    // Trust the drawable the compositor gave (WM may clamp; exclusive may have
    // fallen back). The SIZE_CHANGED event finishes the resize.
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
    // The SIZE_CHANGED event normally gets here first; this covers a mode change
    // the WM answers without an event.
    if(m_scene) { m_scene->Resize(m_screen_width, m_screen_height, m_msaa); }
    check_gl_error();
    // A mode change resets the swap interval, and the panel rate can change with
    // it (the Settings dropdown lists "%dx%d @ %dHz"), so re-apply rather than
    // leave the startup line stale.
    applySwapInterval();
    check_gl_error();
}

std::vector<Resolution> Renderer::displayModes() {
    std::vector<Resolution> out;
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
    // The driver may not grant the request: read back what it gave. A failed
    // read-back gets its own sentinel -- -1 is a legitimate value (adaptive
    // vsync), so it cannot double as "unknown".
    if(!SDL_GL_SetSwapInterval(m_vsync)) {
        printf("vsync: SDL_GL_SetSwapInterval(%d) failed (%s)\n",
               m_vsync, SDL_GetError());
    }
    const int kSwapUnknown = -999;
    int granted = kSwapUnknown;
    if(!SDL_GL_GetSwapInterval(&granted)) { granted = kSwapUnknown; }
    // A granted interval > 0 only means SwapBuffers BLOCKS where there is a
    // retrace to wait for: the headless/offscreen EGL path reports 1 and then
    // never waits. Claim the wait only when a panel rate is actually known.
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

bool Renderer::isHidden() const {
    const SDL_WindowFlags f = SDL_GetWindowFlags(m_window);
    return (f & (SDL_WINDOW_OCCLUDED | SDL_WINDOW_MINIMIZED)) != 0;
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
    /* No lazy-init guard here: SDL_image 3 has no IMG_Init, and its libpng
       loader initializes on first use without a lock. It is already loaded by
       then -- texture.cpp/skybox.cpp IMG_Load at startup, on this thread. */
    if(!m_shots) { m_shots = std::make_unique<ScreenshotWriter>(); }

    const int w = m_screen_width;
    const int h = m_screen_height;
    /* GL_BGRA is the framebuffer's native byte order: B,G,R,A in memory.
       SDL3's packed names run high bit to low bit, so the format naming THAT
       memory layout is XRGB8888 (its little-endian bytes are B,G,R,X). X =
       unused alpha: the framebuffer's alpha is meaningless, and an RGB PNG is
       quicker to encode and smaller than RGBA. */
    SDL_Surface *surface = SDL_CreateSurface(w, h, SDL_PIXELFORMAT_XRGB8888);
    if(!surface) { return false; }
    // Read straight into the surface: no second full-frame buffer, no per-pixel
    // copy. Safe because pitch is 4-aligned and a 4-byte pixel row always is.
    assert(surface->pitch == w * 4);
    glReadPixels(0, 0, w, h, GL_BGRA, GL_UNSIGNED_BYTE, surface->pixels);
    check_gl_error();

    if(!m_shots->push(surface, filename)) {
        // F12 spam against a slow encoder: the game reports the shot as failed,
        // say why on the console.
        printf("Failed to save screenshot %s: encoder still busy\n", filename);
        SDL_DestroySurface(surface);
        return false;
    }
    return true;
}

Renderer::ShotState Renderer::PollScreenshot()
{
    return m_shots ? m_shots->poll() : ShotState::None;
}
