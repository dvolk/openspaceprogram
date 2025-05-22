#include <iostream>
#include <assert.h>

#include <GL/glew.h>
#include "SDL2/SDL.h"

#include "display.h"
#include "gldebug.h"

using namespace std;

Renderer::Renderer(int width, int height)
{
    int gl_major = 4;
    int gl_minor = 3;
    bool gl_core = true;
    m_gl_debug = false;
    Uint32 window_flags = SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE;
    char window_title[] = "Open Space Program";
    m_screen_width = 1920;
    m_screen_height = 1080;
  
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    check_gl_error();
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    check_gl_error();
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    check_gl_error();
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, gl_major);
    check_gl_error();
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, gl_minor);
    check_gl_error();
    if(gl_core == true)
        {
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
            check_gl_error();
        }
    if(m_gl_debug == true)
        {
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);
            check_gl_error();
        }

    SDL_DisplayMode current;
    SDL_GetCurrentDisplayMode(0, &current);
    check_gl_error();

    m_window = SDL_CreateWindow(window_title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1920, 1080, window_flags);
    check_gl_error();
    assert(m_window);

    SDL_GLContext glcontext = SDL_GL_CreateContext(m_window);
    check_gl_error();
    assert(glcontext);

    GLenum glew_status = glewInit();
    check_gl_error();

    if (glew_status != GLEW_OK)
        {
            cerr << "Error: glewInit: " << glewGetErrorString(glew_status) << endl;
        }
    if (GLEW_VERSION_4_3 == false)
        {
            cerr << "Error: your graphic card does not support OpenGL " << gl_major << "." << gl_minor << endl;
        }
    
    glEnable(GL_DEPTH_TEST);
    check_gl_error();
    glDepthFunc(GL_LESS);
    check_gl_error();
    glEnable(GL_CULL_FACE);
    check_gl_error();
    glFrontFace(GL_CCW);
    check_gl_error();
    glCullFace(GL_BACK);
    check_gl_error();

    GLenum res = glewInit();
    if(res != GLEW_OK) {
        std::cerr << "Glew failed to initialize!" << std::endl;
    }

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
