#include <iostream>

#include <GL/glew.h>
#include "SDL2/SDL.h"

#include "display.h"
#include "gldebug.h"

using namespace std;

Renderer::Renderer(int width, int height)
{
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
    SDL_DisplayMode current;
    SDL_GetCurrentDisplayMode(0, &current);
    m_window = SDL_CreateWindow("Open Space Program", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE);
    SDL_GLContext glcontext = SDL_GL_CreateContext(m_window);

    // m_screen_width = width;
    // m_screen_height = height;
    // SDL_DisplayMode current;
    // SDL_GetCurrentDisplayMode(0, &current);
    // m_window =
    //     SDL_CreateWindow("Hello?", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
    //                      width, height,
    //                      SDL_WINDOW_RESIZABLE | SDL_WINDOW_OPENGL);
    // if (m_window == NULL) {
    //     cerr << "Error: can't create window: " << SDL_GetError() << endl;
    // }

    // SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    // SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
    // //SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    // if (SDL_GL_CreateContext(m_window) == NULL) {
    //     cerr << "Error: SDL_GL_CreateContext: " << SDL_GetError() << endl;
    // }

    GLenum glew_status = glewInit();

    if (glew_status != GLEW_OK) {
        cerr << "Error: glewInit: " << glewGetErrorString(glew_status) << endl;
    }
    if (!GLEW_VERSION_2_0) {
        cerr << "Error: your graphic card does not support OpenGL 2.0" << endl    ;}
    
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glCullFace(GL_BACK);

    GLenum res = glewInit();
    if(res != GLEW_OK) {
      std::cerr << "Glew failed to initialize!" << std::endl;
    }
}

Renderer::~Renderer()
{
}

void Renderer::onResize(int width, int height) {
    m_screen_width = width;
    m_screen_height = height;
    check_gl_error();
    glViewport(0, 0, m_screen_width, m_screen_height);
    check_gl_error();

}
void Renderer::Clear(float r, float g, float b, float a)
{
    glClearColor(r, g, b, a);
    check_gl_error();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    check_gl_error();
    // glClearDepth(1.0f);
}

void Renderer::SwapBuffers()
{
    check_gl_error();
    SDL_GL_SwapWindow(m_window);
    check_gl_error();
}
