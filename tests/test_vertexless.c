// Standalone repro: vertex-less fullscreen triangle (gl_VertexID) in a
// 4.3 core context, same setup as display.cpp. Prints the exact GL error
// after use/draw so the llvmpipe complaint is unambiguous.
#include <stdio.h>
#include <GL/glew.h>
#include "SDL2/SDL.h"

static GLuint compile(GLenum type, const char *src)
{
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    GLint ok; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if(!ok) {
        char log[2048]; glGetShaderInfoLog(s, sizeof(log), NULL, log);
        printf("COMPILE FAIL (%s):\n%s\n", type == GL_VERTEX_SHADER ? "vs" : "fs", log);
    }
    return s;
}

int main(void)
{
    SDL_Init(SDL_INIT_VIDEO);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_Window *w = SDL_CreateWindow("t", 0, 0, 640, 480, SDL_WINDOW_OPENGL);
    SDL_GLContext c = SDL_GL_CreateContext(w);
    SDL_GL_MakeCurrent(w, c);
    glewInit();
    printf("GL: %s | %s\n",
           (const char *)glGetString(GL_VERSION),
           (const char *)glGetString(GL_RENDERER));

    // 1) vertex-less triangle (the pattern postfx uses)
    const char *vs =
        "#version 430\n"
        "out vec2 uv;\n"
        "void main(){\n"
        "  vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));\n"
        "  gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);\n"
        "  uv = p;\n"
        "}\n";
    const char *fs =
        "#version 430\n"
        "in vec2 uv;\n"
        "out vec4 c;\n"
        "void main(){ c = vec4(uv, 0.0, 1.0); }\n";

    GLuint prog = glCreateProgram();
    glAttachShader(prog, compile(GL_VERTEX_SHADER, vs));
    glAttachShader(prog, compile(GL_FRAGMENT_SHADER, fs));
    glLinkProgram(prog);
    GLint ok; glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if(!ok) {
        char log[2048]; glGetProgramInfoLog(prog, sizeof(log), NULL, log);
        printf("LINK FAIL:\n%s\n", log);
    }
    glUseProgram(prog);
    printf("vertexless: after glUseProgram 0x%x\n", glGetError());
    glDrawArrays(GL_TRIANGLES, 0, 3);
    printf("vertexless: after glDrawArrays (VAO 0) 0x%x\n", glGetError());

    // 1b) same vertex-less shader, but draw inside a real VAO
    GLuint vao0; glGenVertexArrays(1, &vao0);
    glBindVertexArray(vao0);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    printf("vertexless: after glDrawArrays (real VAO) 0x%x\n", glGetError());
    glBindVertexArray(0);

    // 2) control: VBO + attribute, same context
    const float verts[6] = { -1.f, -1.f, 1.f, -1.f, -1.f, 1.f };
    const char *vs2 =
        "#version 430\n"
        "layout(location=0) in vec2 pos;\n"
        "void main(){ gl_Position = vec4(pos, 0.0, 1.0); }\n";
    const char *fs2 =
        "#version 430\n"
        "out vec4 c;\n"
        "void main(){ c = vec4(1.0); }\n";

    GLuint prog2 = glCreateProgram();
    glAttachShader(prog2, compile(GL_VERTEX_SHADER, vs2));
    glAttachShader(prog2, compile(GL_FRAGMENT_SHADER, fs2));
    glLinkProgram(prog2);
    glUseProgram(prog2);
    printf("vbo:      after glUseProgram 0x%x\n", glGetError());
    GLuint vao2; glGenVertexArrays(1, &vao2);
    glBindVertexArray(vao2);
    GLuint vbo; glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
    printf("vbo:      after attrib setup 0x%x\n", glGetError());
    glDrawArrays(GL_TRIANGLES, 0, 3);
    printf("vbo:      after glDrawArrays (real VAO) 0x%x\n", glGetError());
    glBindVertexArray(0);

    SDL_Quit();
    return 0;
}
