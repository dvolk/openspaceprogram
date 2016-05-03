/**
 * From the OpenGL Programming wikibook: http://en.wikibooks.org/wiki/OpenGL_Programming
 * This file is in the public domain.
 * Contributors: Guus Sliepen, Sylvain Beucler
 */
#include <cstdlib>
#include <iostream>
using namespace std;

/* Use glew.h instead of gl.h to get all the GL prototypes declared */
#include <GL/glew.h>
/* Using SDL2 for the base window and OpenGL context init */
#include "SDL2/SDL.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "shader_utils.h"

#include <ft2build.h>
#include FT_FREETYPE_H

int screen_width=800, screen_height=600;
GLuint program;
GLint attribute_coord;
GLint uniform_tex;
GLint uniform_color;

struct point {
    GLfloat x;
    GLfloat y;
    GLfloat s;
    GLfloat t;
};

GLuint vbo;

FT_Library ft;
FT_Face face;

const char *fontfilename = "./FreeSans.ttf";

// Maximum texture width
#define MAXWIDTH 1024

/**
 * The atlas struct holds a texture that contains the visible US-ASCII characters
 * of a certain font rendered with a certain character height.
 * It also contains an array that contains all the information necessary to
 * generate the appropriate vertex and texture coordinates for each character.
 *
 * After the constructor is run, you don't need to use any FreeType functions anymore.
 */
struct atlas {
    GLuint tex;             // texture object

    unsigned int w;                 // width of texture in pixels
    unsigned int h;                 // height of texture in pixels

    struct {
        float ax;       // advance.x
        float ay;       // advance.y

        float bw;       // bitmap.width;
        float bh;       // bitmap.height;

        float bl;       // bitmap_left;
        float bt;       // bitmap_top;

        float tx;       // x offset of glyph in texture coordinates
        float ty;       // y offset of glyph in texture coordinates
    } c[128];               // character information

    atlas(FT_Face face, int height) {
        FT_Set_Pixel_Sizes(face, 0, height);
        FT_GlyphSlot g = face->glyph;

        unsigned int roww = 0;
        unsigned int rowh = 0;
        w = 0;
        h = 0;

        memset(c, 0, sizeof c);

        /* Find minimum size for a texture holding all visible ASCII characters */
        for (int i = 32; i < 128; i++) {
            if (FT_Load_Char(face, i, FT_LOAD_RENDER)) {
                fprintf(stderr, "Loading character %c failed!\n", i);
                continue;
            }
            if (roww + g->bitmap.width + 1 >= MAXWIDTH) {
                w = std::max(w, roww);
                h += rowh;
                roww = 0;
                rowh = 0;
            }
            roww += g->bitmap.width + 1;
            rowh = std::max(rowh, g->bitmap.rows);
        }
        w = std::max(w, roww);
        h += rowh;

        /* Create a texture that will be used to hold all ASCII glyphs */
        glActiveTexture(GL_TEXTURE0);
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glUniform1i(uniform_tex, 0);

        glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, w, h, 0, GL_ALPHA, GL_UNSIGNED_BYTE, 0);

        /* We require 1 byte alignment when uploading texture data */
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        /* Clamping to edges is important to prevent artifacts when scaling */
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        /* Linear filtering usually looks best for text */
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        /* Paste all glyph bitmaps into the texture, remembering the offset */
        int ox = 0;
        int oy = 0;

        rowh = 0;
        for (int i = 32; i < 128; i++) {
            if (FT_Load_Char(face, i, FT_LOAD_RENDER)) {
                fprintf(stderr, "Loading character %c failed!\n", i);
                continue;
            }

            if (ox + g->bitmap.width + 1 >= MAXWIDTH) {
                oy += rowh;
                rowh = 0;
                ox = 0;
            }

            glTexSubImage2D(GL_TEXTURE_2D, 0, ox, oy, g->bitmap.width, g->bitmap.rows, GL_ALPHA, GL_UNSIGNED_BYTE, g->bitmap.buffer);
            c[i].ax = g->advance.x >> 6;
            c[i].ay = g->advance.y >> 6;

            c[i].bw = g->bitmap.width;
            c[i].bh = g->bitmap.rows;

            c[i].bl = g->bitmap_left;
            c[i].bt = g->bitmap_top;

            c[i].tx = ox / (float)w;
            c[i].ty = oy / (float)h;

            rowh = std::max(rowh, g->bitmap.rows);
            ox += g->bitmap.width + 1;
        }

        fprintf(stderr, "Generated a %d x %d (%d kb) texture atlas\n", w, h, w * h / 1024);
    }

    ~atlas() {
        glDeleteTextures(1, &tex);
    }
};

atlas *a16;

bool text_init_resources() {
    /* Initialize the FreeType2 library */
    if (FT_Init_FreeType(&ft)) {
        cerr << "Could not init freetype library" << endl;
        return false;
    }

    /* Load a font */
    int fontsize;
    char* font = file_read(fontfilename, &fontsize);
    if (font == NULL) {
        cerr << "Could not load font file " << fontfilename << endl;
        return false;
    }
    FT_Error fterr = FT_New_Memory_Face(ft, (FT_Byte*)font, fontsize, 0, &face);
    if (fterr != FT_Err_Ok) {
        cerr << "Could not init font: error 0x" << hex << fterr << endl;
        return false;
    }

    program = create_program("res/text.v.glsl", "res/text.f.glsl");
    if(program == 0)
        return false;

    attribute_coord = get_attrib(program, "coord");
    uniform_tex = get_uniform(program, "tex");
    uniform_color = get_uniform(program, "color");

    if(attribute_coord == -1 || uniform_tex == -1 || uniform_color == -1)
        return false;

    // Create the vertex buffer object
    glGenBuffers(1, &vbo);

    a16 = new atlas(face, 16);

    return true;
}

void render_text(const char *text, atlas * a, float x, float y, float sx, float sy)
{
    const uint8_t *p;

    /* Use the texture containing the atlas */
    glBindTexture(GL_TEXTURE_2D, a->tex);
    glUniform1i(uniform_tex, 0);

    /* Set up the VBO for our vertex data */
    glEnableVertexAttribArray(attribute_coord);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexAttribPointer(attribute_coord, 4, GL_FLOAT, GL_FALSE, 0, 0);

    point coords[6 * strlen(text)];
    int c = 0;

    /* Loop through all characters */
    for (p = (const uint8_t *)text; *p; p++) {
        /* Calculate the vertex and texture coordinates */
        float x2 = x + a->c[*p].bl * sx;
        float y2 = -y - a->c[*p].bt * sy;
        float w = a->c[*p].bw * sx;
        float h = a->c[*p].bh * sy;

        /* Advance the cursor to the start of the next character */
        x += a->c[*p].ax * sx;
        y += a->c[*p].ay * sy;

        /* Skip glyphs that have no pixels */
        if (!w || !h)
            continue;

        coords[c++] = (point) {
            x2, -y2, a->c[*p].tx, a->c[*p].ty};
        coords[c++] = (point) {
            x2 + w, -y2, a->c[*p].tx + a->c[*p].bw / a->w, a->c[*p].ty};
        coords[c++] = (point) {
            x2, -y2 - h, a->c[*p].tx, a->c[*p].ty + a->c[*p].bh / a->h};
        coords[c++] = (point) {
            x2 + w, -y2, a->c[*p].tx + a->c[*p].bw / a->w, a->c[*p].ty};
        coords[c++] = (point) {
            x2, -y2 - h, a->c[*p].tx, a->c[*p].ty + a->c[*p].bh / a->h};
        coords[c++] = (point) {
            x2 + w, -y2 - h, a->c[*p].tx + a->c[*p].bw / a->w, a->c[*p].ty + a->c[*p].bh / a->h};
    }

    /* Draw all the character on the screen in one go */
    glBufferData(GL_ARRAY_BUFFER, sizeof coords, coords, GL_DYNAMIC_DRAW);
    glDrawArrays(GL_TRIANGLES, 0, c);

    glDisableVertexAttribArray(attribute_coord);
}

void drawtext(const char *format, ...) {

    va_list args;
    static char buffer[100];
    
    va_start(args, format);
    vsprintf(buffer, format, args);
    va_end(args);
    
    float sx = 2.0 / 1440;
    float sy = 2.0 / 900;

    glUseProgram(program);

    glDisable(GL_CULL_FACE); // grr!! fonts are CW
    glDisable(GL_DEPTH_TEST);
    
    /* Enable blending, necessary for our alpha texture */
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    GLfloat white[4] = { 1, 1, 1, 1 };

    // white
    glUniform4fv(uniform_color, 1, white);

    render_text(buffer, a16, -1 + 8 * sx, 1 - (16 + 8) * sy, sx, sy);

    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
}

void free_resources() {
    glDeleteProgram(program);
}
