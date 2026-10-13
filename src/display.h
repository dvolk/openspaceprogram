#ifndef DISPLAY_INCLUDED_H
#define DISPLAY_INCLUDED_H

#include <vector>

struct SDL_Window;

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
    bool SaveScreenshot(const char *filename);
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
};

#endif
