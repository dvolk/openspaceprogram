// Which form of the 'far' constant does this GLSL compiler mangle?
// Vertex shader computes logz = log(w*11+1)/log(far*11+1) with w = 1e8,
// fragment outputs logz as color. Expected (far = 1e13): logz = 0.6384 -> (163,163,163).
#include <SDL.h>
#include <GL/glew.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static GLuint sh(GLenum t, const char *s) {
    GLuint o = glCreateShader(t); glShaderSource(o, 1, &s, NULL); glCompileShader(o);
    GLint ok = 0; glGetShaderiv(o, GL_COMPILE_STATUS, &ok);
    if (!ok) { char l[4096]; glGetShaderInfoLog(o, sizeof l, NULL, l); printf("  COMPILE FAIL: %s\n", l); return 0; }
    return o;
}
static GLuint prog(GLuint v, GLuint f) {
    GLuint p = glCreateProgram(); glAttachShader(p, v); glAttachShader(p, f); glLinkProgram(p);
    GLint ok = 0; glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) { char l[4096]; glGetProgramInfoLog(p, sizeof l, NULL, l); printf("  LINK FAIL: %s\n", l); exit(1); }
    return p;
}

static const char *VS_FMT =
"#version 120\n"
"attribute vec3 position;\n"
"varying float logz;\n"
"uniform mat4 MVP;\n"
"void main()\n"
"{\n"
"    gl_Position = MVP * vec4(position, 1.0);\n"
"    const float C=11;\n"
"    %s\n"
"    const float FC = 1.0/log(far*C + 1);\n"
"    logz = log(gl_Position.w * C + 1) * FC;\n"
"    gl_Position.z = (2 * logz - 1) * gl_Position.w;\n"
"}\n";

static const char *FS =
"#version 120\n"
"varying float logz;\n"
"void main()\n"
"{ gl_FragColor = vec4(logz, logz, logz, 1.0); }\n";

int main(void) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) { printf("SDL: %s\n", SDL_GetError()); return 1; }
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    const int W = 400, H = 300;
    SDL_Window *w = SDL_CreateWindow("fc", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, W, H, SDL_WINDOW_OPENGL);
    if (!w) return 1;
    SDL_GLContext c = SDL_GL_CreateContext(w);
    if (!c) return 1;
    glewInit();
    printf("%s | %s\n", (const char*)glGetString(GL_VENDOR), (const char*)glGetString(GL_RENDERER));
    glViewport(0, 0, W, H);

    // one quad: big on screen
    float quad[] = { -0.5f, -0.5f, 0.0f,  0.5f, -0.5f, 0.0f,  0.5f, 0.5f, 0.0f,
                     -0.5f, -0.5f, 0.0f,  0.5f, 0.5f, 0.0f,  -0.5f, 0.5f, 0.0f };
    float proj[16];
    {
        float t = 1.0f/tanf((float)M_PI/6.0f);
        memset(proj, 0, sizeof proj);
        proj[0] = t/(W/(float)H); proj[5] = t;
        proj[10] = -(1e13f+1.0f)/(1e13f-1.0f); proj[11] = -1.0f;
        proj[14] = -2.0f*1e13f/(1e13f-1.0f);
    }
    float mvp[16];
    {
        float D = 1e8f, R = 5e6f;
        float m[16]; memset(m, 0, sizeof m);
        m[0] = R; m[5] = R; m[10] = R; m[14] = -D; m[15] = 1;
        for (int col = 0; col < 4; col++)
            for (int row = 0; row < 4; row++) {
                float s = 0.0f;
                for (int k = 0; k < 4; k++) s += proj[k*4+row] * m[col*4+k];
                mvp[col*4+row] = s;
            }
    }
    GLuint vbo = 0, vao = 0;
    glGenVertexArrays(1, &vao); glBindVertexArray(vao);
    glGenBuffers(1, &vbo); glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof quad, quad, GL_STATIC_DRAW);
    glBindVertexArray(0);

    struct { const char *name; const char *constline; } cases[] = {
        { "int literal        10000000000000   (game form)", "const float far = 10000000000000;" },
        { "float literal      10000000000000.0", "const float far = 10000000000000.0;" },
        { "scientific         1e13            ", "const float far = 1e13;" },
        { "scientific         10.0e12         ", "const float far = 10.0e12;" },
        { "product            1e9 * 10000     ", "const float far = 1e9 * 10000.0;" },
        { "small              1e9             ", "const float far = 1e9;" },
        { "small              1000000000      ", "const float far = 1000000000;" },
    };
    unsigned char *px = malloc(W*H*4);
    int cx = W/2, cy = H/2;
    double expected13 = log(1.1e9) / log(1.1e14);
    printf("expected for far=1e13: logz = %.4f -> RGB %d\n", expected13, (int)(expected13*255+0.5));

    // control: trivial shader, no varyings, constant color
    {
        printf("\n== CONTROL: trivial shader, constant (255,0,0) ==\n");
        static const char *TVS =
        "#version 120\n"
        "attribute vec3 position;\n"
        "uniform mat4 MVP;\n"
        "void main()\n"
        "{ gl_Position = MVP * vec4(position, 1.0); }\n";
        static const char *TFS =
        "#version 120\n"
        "void main()\n"
        "{ gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0); }\n";
        GLuint v = sh(GL_VERTEX_SHADER, TVS);
        GLuint f = sh(GL_FRAGMENT_SHADER, TFS);
        if (v && f) {
            GLuint p = prog(v, f);
            GLint aPos = glGetAttribLocation(p, "position");
            GLint uMVP = glGetUniformLocation(p, "MVP");
            printf("   aPos=%d uMVP=%d\n", aPos, uMVP);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glUseProgram(p);
            glUniformMatrix4fv(uMVP, 1, GL_FALSE, mvp);
            glBindVertexArray(vao);
            glEnableVertexAttribArray(aPos); glVertexAttribPointer(aPos, 3, GL_FLOAT, GL_FALSE, 0, 0);
            glDrawArrays(GL_TRIANGLES, 0, 6);
            glBindVertexArray(0);
            printf("   draw err: 0x%04x\n", (unsigned)glGetError());
            SDL_GL_SwapWindow(w);
            glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
            const unsigned char *p2 = px + (cy*W+cx)*4;
            printf("   center = (%3d,%3d,%3d)\n", p2[0], p2[1], p2[2]);
        }
    }

    for (int i = 0; i < 7; i++) {
        printf("\n== %s ==\n", cases[i].name);
        char vs_src[512];
        snprintf(vs_src, sizeof vs_src, VS_FMT, cases[i].constline);
        GLuint v = sh(GL_VERTEX_SHADER, vs_src);
        if (!v) continue;
        GLuint f = sh(GL_FRAGMENT_SHADER, FS);
        GLuint p = prog(v, f);
        GLint aPos = glGetAttribLocation(p, "position");
        GLint uMVP = glGetUniformLocation(p, "MVP");
        printf("   aPos=%d uMVP=%d\n", aPos, uMVP);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glUseProgram(p);
        glUniformMatrix4fv(uMVP, 1, GL_FALSE, mvp);
        glBindVertexArray(vao);
        glEnableVertexAttribArray(aPos); glVertexAttribPointer(aPos, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glBindVertexArray(0);
        printf("   draw err: 0x%04x\n", (unsigned)glGetError());
        SDL_GL_SwapWindow(w);
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
        const unsigned char *p2 = px + (cy*W+cx)*4;
        double logz = p2[0]/255.0;
        // logz = log(1.1e9)/log(far*11+1)  =>  far = (exp(log(1.1e9)/logz) - 1)/11
        double implied_far = (exp(log(1.1e9)/logz) - 1.0)/11.0;
        printf("   center = (%3d,%3d,%3d)  logz=%.4f  => implied far = %.3e\n", p2[0], p2[1], p2[2], logz, implied_far);
    }
    free(px);
    SDL_Quit();
    return 0;
}
