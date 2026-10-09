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

// One supported display resolution (the Settings dropdown's list item).
// The refresh is informational -- exclusive matches on width/height only.
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
    // Reconfigure the live window (Settings dropdowns / --sim-mode).
    // Fullscreen runs at the display's native mode, so width/height are ignored.
    // The SIZE_CHANGED event finishes the resize (viewport, postfx, camera aspect).
    void setWindowMode(WindowMode mode, int width, int height);
    // Supported resolutions sorted by width, height, refresh; the current
    // mode is guaranteed to be in the list (some stacks keep it out).
    std::vector<Resolution> displayModes();
    // Current refresh rate (Hz); 0 if unknown.
    int currentRefresh();
    // Sample count the window was actually created with (the driver may
    // grant fewer than requested, or zero with no multisample visual).
    int msaaSamples() const;
    // True while the compositor reports the surface fully covered (occluded)
    // or the window minimized: nobody can see the image. Polled from the
    // window flags rather than latched from SDL_EVENT_WINDOW_OCCLUDED /
    // _MINIMIZED, so a window that starts hidden reads correctly and a missed
    // event cannot wedge the caller in either state.
    bool isHidden() const;

    SDL_Window *get_display() { return m_window; }
    int get_width() { return m_screen_width; }
    int get_height() { return m_screen_height; }

    virtual ~Renderer();
protected:
private:

    // Apply m_vsync and report what the driver actually granted. Called by the
    // constructor AND by setWindowMode: a display-mode change is exactly what
    // resets the swap interval, and the panel rate may change with it.
    void applySwapInterval();

    bool m_gl_debug;
    int m_vsync = 1;
    int m_screen_width;
    int m_screen_height;
    SDL_Window *m_window;
};

#endif
