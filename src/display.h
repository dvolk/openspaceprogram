#ifndef DISPLAY_INCLUDED_H
#define DISPLAY_INCLUDED_H

#include <vector>

struct SDL_Window;

enum class WindowMode
{
    Windowed,    // decorated window at width x height
    Borderless,  // no decorations, stays on the desktop
    Fullscreen,  // borderless fullscreen at the display's native mode
    Exclusive    // exclusive fullscreen: display mode change to width x height
};

// One supported display resolution (a "display mode" entry; the Settings
// dropdown's list item). The same width x height appears once per refresh
// rate; the refresh is informational -- this SDL has no
// SDL_SetWindowMode, so exclusive matches on width/height only.
struct Resolution {
    int width;
    int height;
    int refresh;   // Hz; 0 = unknown (omitted from the label)
};

class Renderer
{
public:
    Renderer(int width, int height,
             WindowMode mode = WindowMode::Windowed, bool gl_debug = false);

    void Clear(float r, float g, float b, float a);
    void SwapBuffers();
    void onResize(int width, int height);
    bool SaveScreenshot(const char *filename);
    // Reconfigure the live window to `mode` at `width` x `height` (the
    // Settings dropdowns and --sim-mode go through this): decorations,
    // desktop fullscreen, the display-mode change. Fullscreen runs at the
    // display's native mode, so `width`/`height` are ignored there. The
    // SIZE_CHANGED event (events.cpp) finishes the resize: the viewport
    // (onResize), postfx, the camera aspect.
    void setWindowMode(WindowMode mode, int width, int height);
    // The display's (display 0's) supported resolutions -- one entry per
    // width x height x refresh rate -- sorted by width, height, refresh,
    // with the current mode guaranteed to be in the list (some stacks
    // keep it out of the reported modes).
    std::vector<Resolution> displayModes();
    // The refresh rate (Hz) the display is currently running at; 0 if
    // unknown.
    int currentRefresh();

    SDL_Window *get_display() { return m_window; }
    int get_width() { return m_screen_width; }
    int get_height() { return m_screen_height; }

    virtual ~Renderer();
protected:
private:

    bool m_gl_debug;
    int m_screen_width;
    int m_screen_height;
    SDL_Window *m_window;
};

#endif
