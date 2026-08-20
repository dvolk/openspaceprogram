// End-to-end check of the far-distance fix, replicating the game's exact
// render order: body (game sunShader pattern) with GL_LESS, THEN skybox
// (gl_Position = pos.xyww -> window depth 1.0) with GL_LEQUAL.
//
// The old bug: `const float far = 10000000000000;` (bare int literal) compiled
// to ~1.316e9 on Mesa llvmpipe, so any body beyond ~1.3e9 m got logz >= 1.0
// -> window depth 1.0 -> the LEQUAL skybox clobbered it.
//
// This test renders the body at D = 3e9 m (beyond the old broken cutoff)
// twice: once with the OLD constant form, once with the FIXED form (1e13).
// Expected: OLD -> skybox color (25,51,204); FIXED -> body color (230,230,51).
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

// body vertex shader, game pattern; FARLINE injected per case
static const char *BODY_VS_FMT =
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
"    %s\n"
"    const float FC = 1.0/log(far*C + 1);\n"
"    logz = log(gl_Position.w * C + 1) * FC;\n"
"    gl_Position.z = (2 * logz - 1) * gl_Position.w;\n"
"}\n";

static const char *BODY_FS =
"#version 120\n"
"varying vec4 color0;\n"
"varying float logz;\n"
"void main()\n"
"{ gl_FragColor = color0; gl_FragDepth = logz; }\n";

// skybox: exactly as the game (skyboxShader), window depth = 1.0
static const char *SKY_VS =
"#version 120\n"
"attribute vec3 position;\n"
"varying vec3 texcoord0;\n"
"uniform mat4 projectionview;\n"
"void main()\n"
"{\n"
"    vec4 pos = projectionview * vec4(position, 1.0);\n"
"    gl_Position = pos.xyww;\n"
"    texcoord0 = position;\n"
"}\n";
static const char *SKY_FS =
"#version 120\n"
"varying vec3 texcoord0;\n"
"void main()\n"
"{ gl_FragColor = vec4(0.1, 0.2, 0.8, 1.0); }\n";

static const float skyboxVertices[36*3] = {
    -1,1,-1,  -1,-1,-1,   1,-1,-1,   1,-1,-1,   1,1,-1,  -1,1,-1,
    -1,-1,1,  -1,-1,-1,  -1,1,-1,  -1,1,-1,  -1,-1,1,  -1,-1,-1,
     1,-1,-1,   1,-1, 1,   1, 1,  1,   1, 1,  1,   1,-1,-1,   1,-1,-1,
    -1,-1, 1,  -1, 1, 1,   1, 1,  1,   1, 1,  1,   1,-1,  1,  -1,-1,  1,
    -1, 1,-1,   1, 1,-1,   1, 1,  1,   1, 1,  1,  -1, 1,  1,  -1, 1,-1,
    -1,-1,-1,  -1,-1, 1,   1,-1, 1,   1,-1, 1,  -1,-1, 1,   1,-1,  1
};

int main(void) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) { printf("SDL: %s\n", SDL_GetError()); return 1; }
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    const int W = 800, H = 600;
    SDL_Window *w = SDL_CreateWindow("ff", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, W, H, SDL_WINDOW_OPENGL);
    if (!w) return 1;
    SDL_GLContext c = SDL_GL_CreateContext(w);
    if (!c) return 1;
    glewInit();
    printf("%s | %s\n", (const char*)glGetString(GL_VENDOR), (const char*)glGetString(GL_RENDERER));
    glViewport(0, 0, W, H);

    // body: a quad, big on screen (geometry is irrelevant to the depth question)
    float quad[] = { -0.5f, -0.5f, 0.0f,  0.5f, -0.5f, 0.0f,  0.5f, 0.5f, 0.0f,
                     -0.5f, -0.5f, 0.0f,  0.5f, 0.5f, 0.0f,  -0.5f, 0.5f, 0.0f };
    GLuint bodyvao = 0, bodyvbo = 0;
    glGenVertexArrays(1, &bodyvao); glBindVertexArray(bodyvao);
    glGenBuffers(1, &bodyvbo); glBindBuffer(GL_ARRAY_BUFFER, bodyvbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof quad, quad, GL_STATIC_DRAW);
    glBindVertexArray(0);

    GLuint skyvao = 0, skyvbo = 0;
    glGenVertexArrays(1, &skyvao); glBindVertexArray(skyvao);
    glGenBuffers(1, &skyvbo); glBindBuffer(GL_ARRAY_BUFFER, skyvbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof skyboxVertices, skyboxVertices, GL_STATIC_DRAW);
    glBindVertexArray(0);

    // projection with far = 1e13 (same as the game's glm::perspective call)
    float proj[16];
    {
        float t = 1.0f/tanf((float)M_PI/6.0f);
        memset(proj, 0, sizeof proj);
        proj[0] = t/(W/(float)H); proj[5] = t;
        proj[10] = -(1e13f+1.0f)/(1e13f-1.0f); proj[11] = -1.0f;
        proj[14] = -2.0f*1e13f/(1e13f-1.0f);
    }
    // body at D: M = translate(0,0,-D) * scale(R)
    float D = 3e9f, R = 5e8f; // angular radius ~0.33 rad -> fills most of the screen
    float mvp[16];
    {
        float m[16]; memset(m, 0, sizeof m);
        m[0] = R; m[5] = R; m[10] = R; m[14] = -D; m[15] = 1;
        for (int col = 0; col < 4; col++)
            for (int row = 0; row < 4; row++) {
                float s = 0.0f;
                for (int k = 0; k < 4; k++) s += proj[k*4+row] * m[col*4+k];
                mvp[col*4+row] = s;
            }
    }
    printf("body distance w = %.2e (old broken cutoff was ~1.3e9)\n", (double)D);

    struct { const char *name; const char *farline; int expect_body; } cases[] = {
        { "OLD form: const float far = 10000000000000;  (expect SKYBOX)", "const float far = 10000000000000;", 0 },
        { "FIXED:  const float far = 1e13;              (expect BODY)",   "const float far = 1e13;", 1 },
    };

    unsigned char *px = malloc(W*H*4);
    int cx = W/2, cy = H/2;

    for (int i = 0; i < 2; i++) {
        printf("\n== %s ==\n", cases[i].name);
        char vs_src[512];
        snprintf(vs_src, sizeof vs_src, BODY_VS_FMT, cases[i].farline);
        GLuint bv = sh(GL_VERTEX_SHADER, vs_src);
        if (!bv) continue;
        GLuint bp = prog(bv, sh(GL_FRAGMENT_SHADER, BODY_FS));
        GLuint sp = prog(sh(GL_VERTEX_SHADER, SKY_VS), sh(GL_FRAGMENT_SHADER, SKY_FS));
        GLint bPos = glGetAttribLocation(bp, "position"), bMVP = glGetUniformLocation(bp, "MVP");
        GLint sPos = glGetAttribLocation(sp, "position"), sPV = glGetUniformLocation(sp, "projectionview");

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LESS);

        // 1) body (game order)
        glUseProgram(bp);
        glBindVertexArray(bodyvao);
        glEnableVertexAttribArray(bPos); glVertexAttribPointer(bPos, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glUniformMatrix4fv(bMVP, 1, GL_FALSE, mvp);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glBindVertexArray(0);

        // 2) skybox LAST, with LEQUAL (game order)
        glDepthFunc(GL_LEQUAL);
        glUseProgram(sp);
        glBindVertexArray(skyvao);
        glEnableVertexAttribArray(sPos); glVertexAttribPointer(sPos, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glUniformMatrix4fv(sPV, 1, GL_FALSE, proj);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glBindVertexArray(0);
        glDepthFunc(GL_LESS);

        printf("   draw err: 0x%04x\n", (unsigned)glGetError());
        SDL_GL_SwapWindow(w);
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
        const unsigned char *p2 = px + (cy*W+cx)*4;
        const char *got = (p2[2] > p2[0]) ? "SKYBOX" : "BODY";
        const char *want = cases[i].expect_body ? "BODY" : "SKYBOX";
        printf("   center = (%3d,%3d,%3d) -> %s   [%s]\n", p2[0], p2[1], p2[2], got,
               strcmp(got, want) == 0 ? "PASS" : "FAIL");
    }
    free(px);
    SDL_Quit();
    return 0;
}
