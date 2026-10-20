#ifndef DISPLAY_INCLUDED_H
#define DISPLAY_INCLUDED_H

#include <memory>
#include <vector>

struct SDL_Window;
class ScreenshotWriter;

enum class WindowMode
{
    Windowed,    // decorated
    Borderless,  // no decorations, desktop mode
    Fullscreen,  // desktop-mode fullscreen at the native size
    Exclusive    // display-mode change to width x height
};

// One supported display mode. Refresh is informational -- exclusive matching
// uses width/height only.
struct Resolution {
    int width;
    int height;
    int refresh;   // Hz; 0 = unknown (omitted from the label)
};

class Renderer
{
public:
    Renderer(int width, int height,
             WindowMode mode = WindowMode::Windowed,
             int msaa_samples = 4, bool gl_debug = false, int vsync = 1);

    void Clear(float r, float g, float b, float a);
    void SwapBuffers();
    void onResize(int width, int height);
    // Grab the back buffer now and encode the PNG on a worker thread: the grab
    // still costs one GPU sync, but PNG compression (~200 ms at 1080p, 4x that
    // at 4K) would otherwise stall the frame. true = queued; poll ShotState.
    bool SaveScreenshot(const char *filename);
    // A queued shot that finished since the last call.
    enum class ShotState { None, Saved, Failed };
    ShotState PollScreenshot();
    // Reconfigure the live window; the fullscreen modes ignore width/height.
    void setWindowMode(WindowMode mode, int width, int height);
    // Modes sorted by width, height, refresh; the current one is always in.
    std::vector<Resolution> displayModes();
    // Current refresh rate (Hz); 0 if unknown.
    int currentRefresh();
    // Sample count actually granted (may be fewer than requested).
    int msaaSamples() const;
    // True when occluded or minimized: nobody can see the image. Polled from
    // the window flags, not latched from events, so a missed event cannot
    // wedge it.
    bool isHidden() const;

    SDL_Window *get_display() { return m_window; }
    int get_width() { return m_screen_width; }
    int get_height() { return m_screen_height; }

    virtual ~Renderer();
protected:
private:

    // Apply m_vsync and report what the driver granted (ctor + setWindowMode).
    void applySwapInterval();

    bool m_gl_debug;
    int m_vsync = 1;
    int m_screen_width;
    int m_screen_height;
    SDL_Window *m_window;
    // Declared last, so it is destroyed first: the worker is joined before the
    // rest of the Renderer goes away. Shots only touch their own surfaces.
    std::unique_ptr<ScreenshotWriter> m_shots;
};

#endif
