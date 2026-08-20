// WARNING (2026-08-20): results of THIS file are NOT trustworthy — the
// SETUP_SUN/SETUP_PLAIN macros set attribute pointers while the current
// ARRAY_BUFFER is the COLOR buffer (cbo), so position/normal read color data.
// Use test_fragdepth.c / test_farconst.c / test_farfix.c instead.
//
// Depth-pipeline debug: which of the game's render patterns actually work
// on this stack? D = 1e8 m, R = 5e6 m (huge on screen).
#include <SDL.h>
#include <GL/glew.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static GLuint compile(GLenum type, const char *src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    GLint ok = 0; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) { char log[2048]; glGetShaderInfoLog(s, sizeof log, NULL, log); printf("FAIL: %s\n", log); exit(1); }
    return s;
}
static GLuint linkp(GLuint vs, GLuint fs) {
    GLuint p = glCreateProgram();
    glAttachShader(p, vs); glAttachShader(p, fs); glLinkProgram(p);
    GLint ok = 0; glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) { char log[2048]; glGetProgramInfoLog(p, sizeof log, NULL, log); printf("LINK FAIL: %s\n", log); exit(1); }
    return p;
}

static const char *SUN_VS =
"#version 120\n"
"attribute vec3 position;\n"
"attribute vec3 normal;\n"
"attribute vec3 color;\n"
"varying vec3 normal0;\n"
"varying vec4 color0;\n"
"varying float logz;\n"
"uniform mat4 MVP;\n"
"uniform mat4 Normal;\n"
"void main()\n"
"{\n"
"    gl_Position = MVP * vec4(position, 1.0);\n"
"    normal0 = (Normal * vec4(normal, 0.0)).xyz;\n"
"    color0 = vec4(color, 1.0);\n"
"    const float C=11;\n"
"    const float far = 10000000000000;\n"
"    const float FC = 1.0/log(far*C + 1);\n"
"    logz = log(gl_Position.w * C + 1) * FC;\n"
"    gl_Position.z = (2 * logz - 1) * gl_Position.w;\n"
"}\n";
static const char *SUN_FS =
"#version 120\n"
"varying vec3 normal0;\n"
"varying vec4 color0;\n"
"varying float logz;\n"
"uniform vec3 lightDirection;\n"
"uniform vec4 color;\n"
"void main()\n"
"{\n"
"    gl_FragColor = color0;\n"
"    gl_FragDepth = logz;\n"
"}\n";
static const char *PLAIN_VS =
"#version 120\n"
"attribute vec3 position;\n"
"attribute vec3 color;\n"
"varying vec4 color0;\n"
"uniform mat4 MVP;\n"
"void main()\n"
"{ gl_Position = MVP * vec4(position, 1.0); color0 = vec4(color, 1.0); }\n";
static const char *PLAIN_FS =
"#version 120\n"
"varying vec4 color0;\n"
"void main()\n"
"{ gl_FragColor = color0; }\n";
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
     1,-1,-1,   1,-1, 1,   1, 1, 1,   1, 1, 1,   1,-1,-1,   1,-1,-1,
    -1,-1, 1,  -1, 1, 1,   1, 1, 1,   1, 1, 1,   1,-1, 1,  -1,-1, 1,
    -1, 1,-1,   1, 1,-1,   1, 1, 1,   1, 1, 1,  -1, 1, 1,  -1, 1,-1,
    -1,-1,-1,  -1,-1, 1,   1,-1, 1,   1,-1, 1,  -1,-1, 1,   1,-1, 1
};

static void perspective(float fovy, float aspect, float n, float f, float out[16]) {
    float t = 1.0f / tanf(fovy / 2.0f);
    memset(out, 0, sizeof(float) * 16);
    out[0] = t / aspect; out[5] = t;
    out[10] = -(f + n) / (f - n); out[11] = -1.0f; out[14] = -2.0f * f * n / (f - n);
}
static void make_mvp(float P[16], float D, float R, float mvp[16]) {
    float m[16]; memset(m, 0, sizeof m);
    m[0] = R; m[5] = R; m[10] = R; m[14] = -(float)D; m[15] = 1.0f;
    for (int col = 0; col < 4; col++)
        for (int row = 0; row < 4; row++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) s += P[k*4+row] * m[col*4+k];
            mvp[col*4+row] = s;
        }
}

int main(void) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) { printf("SDL_Init failed: %s\n", SDL_GetError()); return 1; }
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    const int W = 800, H = 600;
    SDL_Window *w = SDL_CreateWindow("depth", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, W, H, SDL_WINDOW_OPENGL);
    if (!w) { printf("window FAILED: %s\n", SDL_GetError()); return 1; }
    SDL_GLContext c = SDL_GL_CreateContext(w);
    if (!c) { printf("context FAILED: %s\n", SDL_GetError()); return 1; }
    glewInit();
    printf("GL: %s | %s\n", (const char*)glGetString(GL_VENDOR), (const char*)glGetString(GL_RENDERER));
    int dbits = -1; glGetIntegerv(GL_DEPTH_BITS, &dbits); printf("DEPTH_BITS=%d (err=%u)\n", dbits, (unsigned)glGetError());
    glViewport(0, 0, W, H);

    // sphere VBO (unit) + VAO
    int LAT = 32, LON = 32;
    int nvmax = (LAT+1)*LON;
    float *vbuf = malloc(sizeof(float)*nvmax*3);
    unsigned int *ibuf = malloc(sizeof(unsigned int)*LAT*LON*6);
    int nv = 0;
    for (int i = 0; i <= LAT; i++) {
        float phi = (float)M_PI * i / LAT;
        for (int j = 0; j < LON; j++) {
            float th = 2.0f*(float)M_PI*j/LON;
            vbuf[nv++] = sinf(phi)*cosf(th); vbuf[nv++] = cosf(phi); vbuf[nv++] = sinf(phi)*sinf(th);
        }
    }
    int ni = 0;
    for (int i = 0; i < LAT; i++) for (int j = 0; j < LON; j++) {
        int a = i*LON+j, b = i*LON+(j+1)%LON, c2 = (i+1)*LON+j, d = (i+1)*LON+(j+1)%LON;
        ibuf[ni++]=a; ibuf[ni++]=b; ibuf[ni++]=d;
        ibuf[ni++]=a; ibuf[ni++]=d; ibuf[ni++]=c2;
    }
    float *cbuf = malloc(sizeof(float)*nvmax*3);
    for (int i = 0; i < nvmax; i++) { cbuf[3*i]=0.9f; cbuf[3*i+1]=0.9f; cbuf[3*i+2]=0.2f; }

    GLuint sunVs = compile(GL_VERTEX_SHADER, SUN_VS), sunFs = compile(GL_FRAGMENT_SHADER, SUN_FS);
    GLuint sunProg = linkp(sunVs, sunFs);
    GLint aPos = glGetAttribLocation(sunProg, "position"), aNor = glGetAttribLocation(sunProg, "normal"), aCol = glGetAttribLocation(sunProg, "color");
    GLint uMVP = glGetUniformLocation(sunProg, "MVP"), uNrm = glGetUniformLocation(sunProg, "Normal");
    GLuint plainVs = compile(GL_VERTEX_SHADER, PLAIN_VS), plainFs = compile(GL_FRAGMENT_SHADER, PLAIN_FS);
    GLuint plainProg = linkp(plainVs, plainFs);
    GLint paPos = glGetAttribLocation(plainProg, "position"), paCol = glGetAttribLocation(plainProg, "color");
    GLint puMVP = glGetUniformLocation(plainProg, "MVP");
    GLuint skyVs = compile(GL_VERTEX_SHADER, SKY_VS), skyFs = compile(GL_FRAGMENT_SHADER, SKY_FS);
    GLuint skyProg = linkp(skyVs, skyFs);
    GLint saPos = glGetAttribLocation(skyProg, "position");
    GLint suPV = glGetUniformLocation(skyProg, "projectionview");

    GLuint vao = 0, vbo = 0, cbo = 0, ebo = 0, skyvao = 0, skyvbo = 0;
    glGenVertexArrays(1, &vao); glBindVertexArray(vao);
    glGenBuffers(1, &vbo); glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof vbuf[0]*nvmax*3, vbuf, GL_STATIC_DRAW);
    glGenBuffers(1, &cbo); glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof cbuf[0]*nvmax*3, cbuf, GL_STATIC_DRAW);
    glGenBuffers(1, &ebo); glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(ibuf[0])*ni, ibuf, GL_STATIC_DRAW);
    glGenVertexArrays(1, &skyvao); glBindVertexArray(skyvao);
    glGenBuffers(1, &skyvbo); glBindBuffer(GL_ARRAY_BUFFER, skyvbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof skyboxVertices, skyboxVertices, GL_STATIC_DRAW);
    glBindVertexArray(0);

    float proj[16]; perspective((float)M_PI/3.0, (float)W/H, 1.0f, 1e13f, proj);
    float mvp[16]; make_mvp(proj, 1e8f, 5e6f, mvp);
    unsigned char *px = malloc(W*H*4);
    float *dpx = malloc(W*H*4);
    int cx = W/2, cy = H/2;

    // per-program attribute setup helpers
    #define SETUP_SUN() do { glBindVertexArray(vao); \
        glEnableVertexAttribArray(aPos); glVertexAttribPointer(aPos,3,GL_FLOAT,GL_FALSE,0,0); \
        glEnableVertexAttribArray(aNor); glVertexAttribPointer(aNor,3,GL_FLOAT,GL_FALSE,0,0); \
        glEnableVertexAttribArray(aCol); glVertexAttribPointer(aCol,3,GL_FLOAT,GL_FALSE,0,0); } while(0)
    #define SETUP_PLAIN() do { glBindVertexArray(vao); \
        glEnableVertexAttribArray(paPos); glVertexAttribPointer(paPos,3,GL_FLOAT,GL_FALSE,0,0); \
        if (paCol >= 0) { glEnableVertexAttribArray(paCol); glVertexAttribPointer(paCol,3,GL_FLOAT,GL_FALSE,0,0); } } while(0)
    #define SETUP_SKY() do { glBindVertexArray(skyvao); \
        glEnableVertexAttribArray(saPos); glVertexAttribPointer(saPos,3,GL_FLOAT,GL_FALSE,0,0); } while(0)
    #define TEARDOWN() do { for (int i = 0; i < 4; i++) glDisableVertexAttribArray(i); glBindVertexArray(0); } while(0)

    void report(const char *what, int depthTest, GLenum df) {
        glDepthFunc(GL_LESS);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (depthTest) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
        (void)df;
        // placeholder (unused); real cases below
        (void)what;
    }

    #define CHECK(e) do { GLenum e_ = glGetError(); if (e_ != GL_NO_ERROR) printf("    GL error 0x%04x after %s\n", (unsigned)e_, e); } while(0)

    float depthAtCenter(void) {
        glReadPixels(cx, cy, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, dpx);
        return dpx[0];
    }
    void readCenter(unsigned char *out) {
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
        memcpy(out, px + (cy*W+cx)*4, 4);
    }
    void swap_read(unsigned char *out) {
        SDL_GL_SwapWindow(w);
        readCenter(out);
        printf("    center = (%3d,%3d,%3d) depth=%.5f\n", out[0], out[1], out[2], depthAtCenter());
    }

    printf("\n--- case 1: sphere only, depth test ON, LESS, sunShader (logz) ---\n");
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LESS);
    glUseProgram(sunProg); glUniformMatrix4fv(uMVP,1,GL_FALSE,mvp); glUniformMatrix4fv(uNrm,1,GL_FALSE,mvp);
    SETUP_SUN(); glDrawElements(GL_TRIANGLES, ni, GL_UNSIGNED_INT, 0); TEARDOWN();
    CHECK("draw");
    swap_read((unsigned char[4]){0});

    printf("\n--- case 2: sphere only, depth test OFF, sunShader ---\n");
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);
    glUseProgram(sunProg);
    SETUP_SUN(); glDrawElements(GL_TRIANGLES, ni, GL_UNSIGNED_INT, 0); TEARDOWN();
    CHECK("draw");
    swap_read((unsigned char[4]){0});

    printf("\n--- case 3: sphere (plain shader) only, depth ON ---\n");
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LESS);
    glUseProgram(plainProg); glUniformMatrix4fv(puMVP,1,GL_FALSE,mvp);
    SETUP_PLAIN(); glDrawElements(GL_TRIANGLES, ni, GL_UNSIGNED_INT, 0); TEARDOWN();
    CHECK("draw");
    swap_read((unsigned char[4]){0});

    printf("\n--- case 4: sphere(logz) then skybox(LEQUAL), depth ON ---  [game order]\n");
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LESS);
    glUseProgram(sunProg);
    SETUP_SUN(); glDrawElements(GL_TRIANGLES, ni, GL_UNSIGNED_INT, 0); TEARDOWN();
    glDepthFunc(GL_LEQUAL);
    glUseProgram(skyProg); glUniformMatrix4fv(suPV,1,GL_FALSE,proj);
    SETUP_SKY(); glDrawArrays(GL_TRIANGLES, 0, 36); TEARDOWN();
    CHECK("draw");
    glDepthFunc(GL_LESS);
    swap_read((unsigned char[4]){0});

    printf("\n--- case 5: skybox(LESS) then sphere(logz, LESS), depth ON ---  [reversed order]\n");
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LESS);
    glUseProgram(skyProg); glUniformMatrix4fv(suPV,1,GL_FALSE,proj);
    SETUP_SKY(); glDrawArrays(GL_TRIANGLES, 0, 36); TEARDOWN();
    glUseProgram(sunProg);
    SETUP_SUN(); glDrawElements(GL_TRIANGLES, ni, GL_UNSIGNED_INT, 0); TEARDOWN();
    CHECK("draw");
    swap_read((unsigned char[4]){0});

    printf("\n--- case 6: skybox(LESS) then plain sphere(LESS), depth ON ---\n");
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LESS);
    glUseProgram(skyProg); glUniformMatrix4fv(suPV,1,GL_FALSE,proj);
    SETUP_SKY(); glDrawArrays(GL_TRIANGLES, 0, 36); TEARDOWN();
    glUseProgram(plainProg); glUniformMatrix4fv(puMVP,1,GL_FALSE,mvp);
    SETUP_PLAIN(); glDrawElements(GL_TRIANGLES, ni, GL_UNSIGNED_INT, 0); TEARDOWN();
    CHECK("draw");
    swap_read((unsigned char[4]){0});

    printf("\n--- case 7: sphere only, culling OFF, depth ON (isolate cull face?) ---\n");
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LESS);
    glDisable(GL_CULL_FACE);
    glUseProgram(sunProg);
    SETUP_SUN(); glDrawElements(GL_TRIANGLES, ni, GL_UNSIGNED_INT, 0); TEARDOWN();
    glEnable(GL_CULL_FACE);
    CHECK("draw");
    swap_read((unsigned char[4]){0});

    free(px); free(dpx);
    SDL_Quit();
    return 0;
}
