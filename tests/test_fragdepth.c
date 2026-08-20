// Bisect: does gl_FragDepth corrupt color output on this driver?
// D=1e8, R=5e6 (big on screen), depth test ON, sphere only.
#include <SDL.h>
#include <GL/glew.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static GLuint sh(GLenum t, const char *s) {
    GLuint o = glCreateShader(t); glShaderSource(o, 1, &s, NULL); glCompileShader(o);
    GLint ok = 0; glGetShaderiv(o, GL_COMPILE_STATUS, &ok);
    if (!ok) { char l[2048]; glGetShaderInfoLog(o, sizeof l, NULL, l); printf("CFAIL: %s\n", l); exit(1); }
    return o;
}
static GLuint prog(GLuint v, GLuint f) {
    GLuint p = glCreateProgram(); glAttachShader(p, v); glAttachShader(p, f); glLinkProgram(p);
    GLint ok = 0; glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) { char l[2048]; glGetProgramInfoLog(p, sizeof l, NULL, l); printf("LFAIL: %s\n", l); exit(1); }
    return p;
}

static const char *VS_HACK =   // vertex shader WITH the logz hack (game's)
"#version 120\n"
"attribute vec3 position;\n"
"varying vec4 color0;\n"
"varying float logz;\n"
"uniform mat4 MVP;\n"
"void main()\n"
"{\n"
"    gl_Position = MVP * vec4(position, 1.0);\n"
"    color0 = vec4(0.9, 0.9, 0.2, 1.0);\n"
"    const float C=11;\n"
"    const float far = 10000000000000;\n"
"    const float FC = 1.0/log(far*C + 1);\n"
"    logz = log(gl_Position.w * C + 1) * FC;\n"
"    gl_Position.z = (2 * logz - 1) * gl_Position.w;\n"
"}\n";
static const char *VS_PLAIN =  // no hack
"#version 120\n"
"attribute vec3 position;\n"
"varying vec4 color0;\n"
"varying float logz;\n"
"uniform mat4 MVP;\n"
"void main()\n"
"{ gl_Position = MVP * vec4(position, 1.0); color0 = vec4(0.9, 0.9, 0.2, 1.0); logz = 0.5; }\n";

#define F(colorExpr, depthExpr) \
"#version 120\n" \
"varying vec4 color0;\n" \
"varying float logz;\n" \
"void main()\n" \
"{ gl_FragColor = " colorExpr "; " depthExpr " }\n"

static const char *F_CONST      = F("vec4(1.0)", "");
static const char *F_COLOR      = F("color0", "");
static const char *F_COLOR_D    = F("color0", "gl_FragDepth = logz;");
static const char *F_LOGZ_AS_C  = F("vec4(logz, logz, logz, 1.0)", "");
static const char *F_CONST_D    = F("vec4(1.0)", "gl_FragDepth = logz;");
static const char *F_COLOR_D05  = F("color0", "gl_FragDepth = 0.5;");
static const char *F_COLOR_DLOG = F("color0", "gl_FragDepth = log(gl_Position.w);"); // no w in fs; expect compile fail or 0

int main(void) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) { printf("SDL: %s\n", SDL_GetError()); return 1; }
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    const int W = 800, H = 600;
    SDL_Window *w = SDL_CreateWindow("fd", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, W, H, SDL_WINDOW_OPENGL);
    if (!w) return 1;
    SDL_GLContext c = SDL_GL_CreateContext(w);
    if (!c) return 1;
    glewInit();
    printf("%s | %s\n", (const char*)glGetString(GL_VENDOR), (const char*)glGetString(GL_RENDERER));
    glViewport(0, 0, W, H);

    int LAT = 32, LON = 32;
    int nvmax = (LAT+1)*LON;
    float *vb = malloc(sizeof(float)*nvmax*3);
    unsigned int *ib = malloc(sizeof(unsigned int)*LAT*LON*6);
    int nv = 0;
    for (int i = 0; i <= LAT; i++) {
        float phi = (float)M_PI*i/LAT;
        for (int j = 0; j < LON; j++) {
            float th = 2.0f*(float)M_PI*j/LON;
            vb[nv++]=sinf(phi)*cosf(th); vb[nv++]=cosf(phi); vb[nv++]=sinf(phi)*sinf(th);
        }
    }
    int ni = 0;
    for (int i = 0; i < LAT; i++) for (int j = 0; j < LON; j++) {
        int a=i*LON+j, b=i*LON+(j+1)%LON, c2=(i+1)*LON+j, d=(i+1)*LON+(j+1)%LON;
        ib[ni++]=a; ib[ni++]=b; ib[ni++]=d; ib[ni++]=a; ib[ni++]=d; ib[ni++]=c2;
    }
    GLuint vao = 0, vbo = 0, ebo = 0;
    glGenVertexArrays(1, &vao); glBindVertexArray(vao);
    glGenBuffers(1, &vbo); glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof vb[0]*nvmax*3, vb, GL_STATIC_DRAW);
    glGenBuffers(1, &ebo); glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof ib[0]*ni, ib, GL_STATIC_DRAW);
    glBindVertexArray(0);

    float proj[16];
    {
        float t = 1.0f/tanf((float)M_PI/6.0);
        memset(proj, 0, sizeof proj);
        proj[0] = t/(W/(float)H); proj[5] = t;
        proj[10] = -(1e13f+1.0f)/(1e13f-1.0f); proj[11] = -1.0f;
        proj[14] = -2.0f*1e13f*1.0f/(1e13f-1.0f);
    }
    float mvp[16];
    {
        float D = 1e8f, R = 5e6f;
        // M = translate(0,0,-D) * scale(R)
        float m[16];
        memset(m, 0, sizeof m);
        m[0] = R; m[5] = R; m[10] = R;
        m[14] = -D; m[15] = 1.0f;
        // mvp = P * M (full product)
        for (int col = 0; col < 4; col++)
            for (int row = 0; row < 4; row++) {
                float s = 0.0f;
                for (int k = 0; k < 4; k++) s += proj[k*4+row] * m[col*4+k];
                mvp[col*4+row] = s;
            }
    }

    struct { const char *name; const char *fs; } cases[] = {
        { "const color, no depth write",       F_CONST },
        { "color0, no depth write",            F_COLOR },
        { "color0 + gl_FragDepth=logz (game)", F_COLOR_D },
        { "logz as color (expect 163,163,163)",F_LOGZ_AS_C },
        { "const color + gl_FragDepth=logz",   F_CONST_D },
        { "color0 + gl_FragDepth=0.5",         F_COLOR_D05 },
    };
    unsigned char *px = malloc(W*H*4);
    int cx = W/2, cy = H/2;

    for (int i = 0; i < 6; i++) {
        printf("\n== %s ==\n", cases[i].name);
        GLuint p;
        if (i == 0) { // const color: use hack VS (logz defined)
            p = prog(sh(GL_VERTEX_SHADER, VS_HACK), sh(GL_FRAGMENT_SHADER, cases[i].fs));
        } else {
            p = prog(sh(GL_VERTEX_SHADER, VS_HACK), sh(GL_FRAGMENT_SHADER, cases[i].fs));
        }
        GLint aPos = glGetAttribLocation(p, "position");
        GLint uMVP = glGetUniformLocation(p, "MVP");
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LESS);
        glUseProgram(p); glUniformMatrix4fv(uMVP, 1, GL_FALSE, mvp);
        glBindVertexArray(vao);
        glEnableVertexAttribArray(aPos); glVertexAttribPointer(aPos, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glDrawElements(GL_TRIANGLES, ni, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
        GLenum e = glGetError();
        printf("   draw err: %s\n", e == GL_NO_ERROR ? "OK" : "ERROR");
        SDL_GL_SwapWindow(w);
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
        const unsigned char *p2 = px + (cy*W+cx)*4;
        printf("   center = (%3d,%3d,%3d)\n", p2[0], p2[1], p2[2]);
    }
    free(px);
    SDL_Quit();
    return 0;
}
