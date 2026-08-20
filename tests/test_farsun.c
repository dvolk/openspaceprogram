// WARNING (2026-08-20): results of THIS file are NOT trustworthy — GL_CULL_FACE
// is enabled and the generated sphere's winding may be culled, so the sweep
// read skybox everywhere. The question it targeted is now covered by
// test_farfix.c (body + skybox LEQUAL order, old vs fixed `far` literal),
// which passes. Fix culling/winding before relying on this file again.
//
// Far-body visibility sweep: replicates the game's exact render pipeline
// (log-depth sunShader, skybox cube drawn LAST with GL_LEQUAL at window
// depth 1.0) and sweeps the body distance to find where the body vanishes.
//
// For each distance D the sphere radius is scaled so the body subtends a
// constant ~100 px, isolating the depth-test question from rasterization.
// A second series uses the real sun radius (6e6 m).
//
// Build: see Makefile (test_farsun target)
#include <SDL.h>
#include <GL/glew.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static GLuint compile(GLenum type, const char *src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetShaderInfoLog(s, sizeof log, NULL, log);
        printf("SHADER COMPILE FAILED: %s\n", log);
        exit(1);
    }
    return s;
}

static GLuint link(GLuint vs, GLuint fs) {
    GLuint p = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glLinkProgram(p);
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetProgramInfoLog(p, sizeof log, NULL, log);
        printf("LINK FAILED: %s\n", log);
        exit(1);
    }
    return p;
}

// ---- sun (sphere) program: identical to res/sunShader.{vs,fs} ----
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

// ---- skybox: vertex identical to res/skyboxShader.vs, fragment constant ----
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
"{\n"
"    gl_FragColor = vec4(0.1, 0.2, 0.8, 1.0); // skybox blue\n"
"}\n";

static const float skyboxVertices[36 * 3] = {
    -1, 1,-1,  -1,-1,-1,   1,-1,-1,   1,-1,-1,   1, 1,-1,  -1, 1,-1,
    -1,-1, 1,  -1,-1,-1,  -1, 1,-1,  -1, 1,-1,  -1,-1, 1,  -1,-1,-1,
     1,-1,-1,   1,-1, 1,   1, 1, 1,   1, 1, 1,   1,-1,-1,   1,-1,-1,
    -1,-1, 1,  -1, 1, 1,   1, 1, 1,   1, 1, 1,   1,-1, 1,  -1,-1, 1,
    -1, 1,-1,   1, 1,-1,   1, 1, 1,   1, 1, 1,  -1, 1, 1,  -1, 1,-1,
    -1,-1,-1,  -1,-1, 1,   1,-1, 1,   1,-1, 1,  -1,-1, 1,   1,-1, 1
};

// Build a UV sphere (unit radius) with `lat` x `lon` segments, CCW,
// back-face culling on. Positions/indices are float32 like the game's meshes.
static GLuint build_sphere(int lat, int lon, float *vbuf, unsigned int *ibuf, unsigned int *nidx) {
    int nv = 0;
    for (int i = 0; i <= lat; i++) {
        float phi = (float)M_PI * i / lat;
        for (int j = 0; j < lon; j++) {
            float th = 2.0f * (float)M_PI * j / lon;
            float x = sinf(phi) * cosf(th);
            float y = cosf(phi);
            float z = sinf(phi) * sinf(th);
            vbuf[nv++] = x; vbuf[nv++] = y; vbuf[nv++] = z;
        }
    }
    int nquad = 0;
    for (int i = 0; i < lat; i++) {
        for (int j = 0; j < lon; j++) {
            int a = i * lon + j;
            int b = i * lon + (j + 1) % lon;
            int c = (i + 1) * lon + j;
            int d = (i + 1) * lon + (j + 1) % lon;
            ibuf[nquad++] = a; ibuf[nquad++] = b; ibuf[nquad++] = d;
            ibuf[nquad++] = a; ibuf[nquad++] = d; ibuf[nquad++] = c;
        }
    }
    *nidx = (unsigned int)nquad;
    return 0;
}

// glm::perspective equivalent (float), matches camera.cpp usage
static void perspective(float fovy, float aspect, float n, float f, float out[16]) {
    float t = 1.0f / tanf(fovy / 2.0f);
    memset(out, 0, sizeof(float) * 16);
    out[0] = t / aspect;
    out[5] = t;
    out[10] = -(f + n) / (f - n);
    out[11] = -1.0f;
    out[14] = -2.0f * f * n / (f - n);
}

// model = translate(0,0,-D) * scale(R); MVP = P * M  (view = identity)
static void make_mvp(float P[16], float D, float R, float mvp[16]) {
    float m[16];
    memset(m, 0, sizeof m);
    m[0] = R; m[5] = R; m[10] = R;
    m[12] = 0.0f; m[13] = 0.0f; m[14] = -(float)D; m[15] = 1.0f;
    // mvp = P * M
    float o[16];
    for (int col = 0; col < 4; col++) {
        for (int row = 0; row < 4; row++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) s += P[k * 4 + row] * m[col * 4 + k];
            o[col * 4 + row] = s;
        }
    }
    memcpy(mvp, o, sizeof m);
}

int main(int argc, char **argv) {
    (void)argc; (void)argv;
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        printf("SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    const int W = 1920, H = 1080;
    SDL_Window *w = SDL_CreateWindow("farsun", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                     W, H, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!w) { printf("window FAILED: %s\n", SDL_GetError()); return 1; }
    SDL_GLContext c = SDL_GL_CreateContext(w);
    if (!c) { printf("context FAILED: %s\n", SDL_GetError()); return 1; }
    glewInit();
    printf("GL: %s | %s\n", (const char *)glGetString(GL_VENDOR), (const char *)glGetString(GL_RENDERER));
    int dbits = 0; glGetIntegerv(GL_DEPTH_BITS, &dbits);
    printf("DEPTH_BITS=%d\n", dbits);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glCullFace(GL_BACK);
    glViewport(0, 0, W, H);

    // sphere program
    GLuint sunVs = compile(GL_VERTEX_SHADER, SUN_VS);
    GLuint sunFs = compile(GL_FRAGMENT_SHADER, SUN_FS);
    GLuint sunProg = link(sunVs, sunFs);
    GLint aPos = glGetAttribLocation(sunProg, "position");
    GLint aNor = glGetAttribLocation(sunProg, "normal");
    GLint aCol = glGetAttribLocation(sunProg, "color");
    GLint uMVP = glGetUniformLocation(sunProg, "MVP");
    GLint uNrm = glGetUniformLocation(sunProg, "Normal");

    int LAT = 64, LON = 64;
    int nvmax = (LAT + 1) * LON;
    float *vbuf = malloc(sizeof(float) * nvmax * 3);
    float *nbuf = malloc(sizeof(float) * nvmax * 3);
    float *cbuf = malloc(sizeof(float) * nvmax * 3);
    unsigned int *ibuf = malloc(sizeof(unsigned int) * LAT * LON * 6);
    unsigned int nidx = 0;
    build_sphere(LAT, LON, vbuf, ibuf, &nidx);
    for (int i = 0; i < nvmax; i++) {
        memcpy(nbuf + 3 * i, vbuf + 3 * i, sizeof(float) * 3); // unit sphere: normal = position
        cbuf[3 * i + 0] = 0.833f; cbuf[3 * i + 1] = 0.833f; cbuf[3 * i + 2] = 0.333f;
    }

    GLuint sunVao = 0, vbo = 0, nbo = 0, cbo = 0, ebo = 0;
    glGenVertexArrays(1, &sunVao);
    glBindVertexArray(sunVao);
    glGenBuffers(1, &vbo); glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof vbuf[0] * nvmax * 3, vbuf, GL_DYNAMIC_DRAW);
    glGenBuffers(1, &nbo); glBindBuffer(GL_ARRAY_BUFFER, nbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof nbuf[0] * nvmax * 3, nbuf, GL_DYNAMIC_DRAW);
    glGenBuffers(1, &cbo); glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof cbuf[0] * nvmax * 3, cbuf, GL_STATIC_DRAW);
    glGenBuffers(1, &ebo); glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(ibuf[0]) * nidx, ibuf, GL_STATIC_DRAW);
    glEnableVertexAttribArray(aPos); glVertexAttribPointer(aPos, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(aNor); glVertexAttribPointer(aNor, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(aCol); glVertexAttribPointer(aCol, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glBindVertexArray(0);

    // skybox program
    GLuint skyVs = compile(GL_VERTEX_SHADER, SKY_VS);
    GLuint skyFs = compile(GL_FRAGMENT_SHADER, SKY_FS);
    GLuint skyProg = link(skyVs, skyFs);
    GLint aSkyPos = glGetAttribLocation(skyProg, "position");
    GLint uSkyPV = glGetUniformLocation(skyProg, "projectionview");
    GLuint skyVao = 0, skyVbo = 0;
    glGenVertexArrays(1, &skyVao);
    glBindVertexArray(skyVao);
    glGenBuffers(1, &skyVbo); glBindBuffer(GL_ARRAY_BUFFER, skyVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof skyboxVertices, skyboxVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(aSkyPos);
    glVertexAttribPointer(aSkyPos, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glBindVertexArray(0);

    float proj[16];
    perspective((float)M_PI / 3.0, (float)W / H, 1.0f, 1e13f, proj);
    float skyPV[16];
    memcpy(skyPV, proj, sizeof proj); // view = identity (camera at origin)

    unsigned char *px = malloc(W * H * 4);

    // test pairs: (D, R). Series 1: constant ~100 px apparent size.
    // px per rad at 1080p, 60 deg: 1080 / (PI/3) = 1032 px/rad. 100 px dia ->
    // half-angle = 50/1032 = 0.04845 rad -> R = D * tan(0.04845) ~= 0.04847 D.
    const double RATIO = tan(50.0 / (1080.0 / (M_PI / 3.0)));
    double Ds[] = { 1e7, 3e7, 1e8, 3e8, 1e9, 3e9, 1e10, 3e10, 1e11, 3e11, 1e12, 3e12, 9e12 };
    int nd = sizeof Ds / sizeof Ds[0];

    printf("\n== Series 1: constant ~100 px apparent size (R = %.5f * D) ==\n", RATIO);
    printf("%12s %14s %10s  %s\n", "D (m)", "R (m)", "centerRGB", "verdict");
    for (int i = 0; i < nd; i++) {
        double D = Ds[i];
        double R = D * RATIO;
        if (D > 1e13) { printf("%12.0f %14.0f  -        beyond zFar, skip\n", D, R); continue; }

        // upload
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof vbuf[0] * nvmax * 3, vbuf);

        glClearColor(0, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // sphere (game draws bodies with LESS)
        float mvp[16];
        make_mvp(proj, (float)D, (float)R, mvp);
        glUseProgram(sunProg);
        glUniformMatrix4fv(uMVP, 1, GL_FALSE, mvp);
        glUniformMatrix4fv(uNrm, 1, GL_FALSE, mvp);
        glUniform3f(glGetUniformLocation(sunProg, "lightDirection"), 0, 0, -1);
        glUniform4f(glGetUniformLocation(sunProg, "color"), 0.8, 0.8, 0.8, 1);
        glBindVertexArray(sunVao);
        glDrawElements(GL_TRIANGLES, nidx, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

        // skybox last, LEQUAL (game order)
        glDepthFunc(GL_LEQUAL);
        glUseProgram(skyProg);
        glUniformMatrix4fv(uSkyPV, 1, GL_FALSE, skyPV);
        glBindVertexArray(skyVao);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glBindVertexArray(0);
        glDepthFunc(GL_LESS);

        SDL_GL_SwapWindow(w);
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
        // center pixel (bottom-up coords: y = H/2 is symmetric)
        int cx = W / 2, cy = H / 2;
        const unsigned char *p = px + (cy * W + cx) * 4;
        int r = p[0], g = p[1], b = p[2];
        const char *verdict;
        if (r > 100 && g > 100 && b < 150) verdict = "SUN (yellow)";
        else if (b > 100 && r < 80) verdict = "SKYBOX (clobbered or empty)";
        else verdict = "other/none";
        printf("%12.0f %14.0f %5d %5d %5d  %s\n", D, R, r, g, b, verdict);
    }

    // Series 2: real sun radius 6e6 m, varying distance (sub-pixel risk)
    printf("\n== Series 2: real sun radius R = 6e6 m ==\n");
    double Ds2[] = { 2e7, 5e7, 1e8, 3e8, 1e9, 3e9, 1e10 };
    for (int i = 0; i < (int)(sizeof Ds2 / sizeof Ds2[0]); i++) {
        double D = Ds2[i];
        double R = 6e6;
        glClearColor(0, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        float mvp[16];
        make_mvp(proj, (float)D, (float)R, mvp);
        glUseProgram(sunProg);
        glUniformMatrix4fv(uMVP, 1, GL_FALSE, mvp);
        glUniformMatrix4fv(uNrm, 1, GL_FALSE, mvp);
        glBindVertexArray(sunVao);
        glDrawElements(GL_TRIANGLES, nidx, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
        glDepthFunc(GL_LEQUAL);
        glUseProgram(skyProg);
        glUniformMatrix4fv(uSkyPV, 1, GL_FALSE, skyPV);
        glBindVertexArray(skyVao);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glBindVertexArray(0);
        glDepthFunc(GL_LESS);
        SDL_GL_SwapWindow(w);
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
        int cx = W / 2, cy = H / 2;
        const unsigned char *p = px + (cy * W + cx) * 4;
        printf("D = %8.0f m : center = (%3d,%3d,%3d)  %s\n", D, p[0], p[1], p[2],
               (p[0] > 100 && p[1] > 100) ? "SUN visible" : (p[2] > 100 ? "SKYBOX" : "black"));
    }

    free(px);
    SDL_Quit();
    return 0;
}
