#ifndef DISPLAY_INCLUDED_H
#define DISPLAY_INCLUDED_H

struct SDL_Window;

class Renderer
{
public:
    Renderer(int width, int height);

    void Clear(float r, float g, float b, float a);
    void SwapBuffers();
    void onResize(int width, int height);

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
