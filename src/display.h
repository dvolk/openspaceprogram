#ifndef DISPLAY_INCLUDED_H
#define DISPLAY_INCLUDED_H

struct SDL_Window;

enum class WindowMode
{
    Windowed,    // decorated window at width x height
    Borderless,  // no decorations, stays on the desktop
    Fullscreen   // borderless fullscreen at the display's native mode
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
