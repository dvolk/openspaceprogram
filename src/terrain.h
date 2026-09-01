// terrain.h -- terrain data + geometry types.
//
//   COLOUR             rgb triplet (legacy name).
//   PaletteStop        one stop on a body's land-color ramp.
//   AtmosphereParams   per-body Fresnel limb-glow rim.
//   Surface            per-body terrain + color params (the "surface" JSON
//                      block); holds the palette + atmosphere.
//   GeoPatch           one subdividable spherical patch (terrain LOD node).
//   TerrainBody        a celestial body's terrain: 6 root patches, color
//                      ramp, atmosphere shell, height/shadow queries.

#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "model.h"
#include "mesh.h"
#include "shader.h"
#include "camera.h"
#include "frame.h"
#include "calendar.h"

class btRigidBody;

typedef struct {
    float r, g, b;
} COLOUR;

// One stop on a body's land-color ramp: elevation fraction t in [0,1]
// (0 = sea level / lowest land, 1 = highest relief) and the color there.
struct PaletteStop {
    float t;
    glm::vec3 color;
};

// Per-body atmosphere appearance (optional "surface.atmosphere" block).
// v1 is a single Fresnel limb-glow shell drawn over the terrain; see
// reports/atmosphere2026_08_25/atmosphere.md for the design + roadmap.
struct AtmosphereParams {
    bool enabled = false;
    glm::vec3 color = glm::vec3(0.3f, 0.5f, 1.0f);  // rim tint (N2/O2 blue)
    float thickness = 0.0f;   // [m] shell radius above radius + max_height
    float power = 3.0f;       // Fresnel falloff (higher = tighter rim)
    float intensity = 1.0f;   // overall alpha scale
};

// Per-body terrain + color parameters (the optional "surface" JSON block).
// Defaults reproduce the legacy hardcoded behavior.
struct Surface {
    float amplitude = 2500.0f;   // [m] peak noise height
    int octaves = 12;            // simplex octaves
    float persistence = 0.6f;    // octave falloff
    float frequency = 1.0f;      // noise-domain scale
    int power = 3;               // height-distribution exponent
    bool has_sea = false;
    float sea_level = 0.0f;      // [m] above base radius
    glm::vec3 sea_color = glm::vec3(0.1f, 0.1f, 0.8f);
    std::vector<PaletteStop> palette;  // empty => type-based default palette
    float max_height = 1.0f;     // [m] scaled relief above sea level (computed)
    glm::vec3 seed_offset = glm::vec3(0.0f);
    bool bands = false;          // gas giant: smooth sphere, latitude bands
    int band_count = 9;          // stripes pole to pole (odd => bright equator)
    AtmosphereParams atmosphere; // optional rim; enabled => body has air

    COLOUR PaletteColor(float t) const {
        const std::vector<PaletteStop> &s = palette;
        if (s.empty()) {
            return { 1.0f, 1.0f, 1.0f };
        }
        if (t <= s.front().t) {
            return { s.front().color.x, s.front().color.y, s.front().color.z };
        }
        if (t >= s.back().t) {
            return { s.back().color.x, s.back().color.y, s.back().color.z };
        }
        for (size_t i = 1; i < s.size(); i++) {
            if (t <= s[i].t) {
                float f = (t - s[i-1].t) / (s[i].t - s[i-1].t);
                glm::vec3 c = glm::mix(s[i-1].color, s[i].color, f);
                return { c.x, c.y, c.z };
            }
        }
        return { 1.0f, 1.0f, 1.0f };
    }

    // Gas-giant color at unit direction p: latitude runs through a triangle
    // wave so each stripe is dark at its edges, light at its center, sampled
    // through the palette (first stop = dark, last = light).
    COLOUR BandColor(const glm::vec3& p) const {
        float y = glm::clamp(p.y, -1.0f, 1.0f);
        float u = 0.5f + 0.5f * y;           // 0 = south pole, 1 = north
        float x = u * (float)band_count;
        float v = 1.0f - std::fabs(2.0f * (x - std::floor(x)) - 1.0f);
        return PaletteColor(v);
    }
};

struct TerrainBody;

struct GeoPatch {
    TerrainBody *body;
    Model *model;
    GeoPatch *parent_geopatch;
    btRigidBody *collision;
    static const int max_depth = 14;
    int quadrant;

    GeoPatch *kids[4];

    glm::vec3 centroid;
    glm::vec3 v0, v1, v2, v3;

    int depth;

    GeoPatch(TerrainBody *body, Shader *shader, int depth, glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 v3);
    ~GeoPatch();

    void Subdivide(void);
    // skirt_pass=false draws the terrain (stamping stencil), true draws
    // only the skirt ring where the stencil says no terrain was drawn
    void Draw(const Camera* camera, const glm::dmat4& transform, const glm::vec3 & sunlightVec, bool skirt_pass);
    // max_patch_px: subdivide while the patch projects wider than this
    // [screen px]; collapse below half (the hysteresis band)
    void Update(const Camera* camera, const glm::dmat4& transform, int max_patch_px);

    int CountChildren() {
        int ret = 1;
        for(auto&& kid : kids) {
            if(kid != NULL) {
                ret += kid->CountChildren();
            }
        }
        return ret;
    }
};

struct TerrainBody {
    GeoPatch *patches[6];
    Shader *shader;
    Model *atmosphere = nullptr; // Fresnel rim shell (built on demand)
    float atm_radius = 0.0f;     // shell radius [m]; 0 = no atmosphere
    float radius;
    double mu;
    double g; // [m/s^2]
    double soi; // [m]
    float mass;
    std::string name;
    double seed = 0;   // noise-domain offset; 0 = legacy pattern
    Surface surface;
    Frame *frame; // owner
    Frame *rot_frame; // owner
    Calendar cal; // this body's day/year (from its spin + orbit rates)
    glm::dmat4 transform = glm::dmat4(1.0);
    glm::vec3 sunlightVec;

    ~TerrainBody() {
        for(int i = 0; i < 6; i++) { delete patches[i]; }
        delete atmosphere;
        delete frame;
        delete rot_frame;
    }

    COLOUR (*colour_func)(float v, float vmin, float vmax);

    Mesh *create_grid_mesh(bool has_collision, bool has_skirt, glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4);
    Mesh *create_atmosphere_mesh(float radius); // defined below with the other mesh builders
    float GetTerrainHeight(const glm::vec3& p) const;
    float GetTerrainHeightUnscaled(const glm::vec3& p) const;
    float ScaleHeightNoise(float noise) const;
    // The surface color at a unit direction in the body's rotating frame:
    // the exact per-vertex color create_grid_mesh bakes into the terrain
    // (noise, palette / band, sea, contrast), so the 2-D surface map
    // (surfmap.cpp) matches the rendered surface pixel for pixel.
    glm::vec3 SurfaceColor(const glm::vec3& p) const;

    void Create(float radius, float mass) {
        this->radius = radius;
        this->mass = mass;
        const glm::vec3 p1 = glm::normalize(glm::vec3( 1, 1, 1));
        const glm::vec3 p2 = glm::normalize(glm::vec3(-1, 1, 1));
        const glm::vec3 p3 = glm::normalize(glm::vec3(-1,-1, 1));
        const glm::vec3 p4 = glm::normalize(glm::vec3( 1,-1, 1));
        const glm::vec3 p5 = glm::normalize(glm::vec3( 1, 1,-1));
        const glm::vec3 p6 = glm::normalize(glm::vec3(-1, 1,-1));
        const glm::vec3 p7 = glm::normalize(glm::vec3(-1,-1,-1));
        const glm::vec3 p8 = glm::normalize(glm::vec3( 1,-1,-1));

        patches[0] = new GeoPatch(this, shader, 1, p1, p2, p3, p4);
        patches[1] = new GeoPatch(this, shader, 1, p4, p3, p7, p8);
        patches[2] = new GeoPatch(this, shader, 1, p1, p4, p8, p5);
        patches[3] = new GeoPatch(this, shader, 1, p2, p1, p5, p6);
        patches[4] = new GeoPatch(this, shader, 1, p3, p2, p6, p7);
        patches[5] = new GeoPatch(this, shader, 1, p8, p7, p6, p5);
    }

    // Build the atmosphere rim shell on demand. It sits just above the
    // highest terrain so no peak pokes through: base radius + scaled relief
    // + the data thickness. See reports/atmosphere2026_08_25 for the model.
    void BuildAtmosphere(Shader *atmosphereshader) {
        if(atmosphere != nullptr || !surface.atmosphere.enabled) return;
        float shell_radius = radius + surface.max_height
                             + surface.atmosphere.thickness;
        if(shell_radius <= radius) shell_radius = radius * 1.02f;
        Mesh *m = create_atmosphere_mesh(shell_radius);
        atmosphere = new Model;
        atmosphere->FromData(m, atmosphereshader, NULL);
        atm_radius = shell_radius;
    }

    void DrawAtmosphere(const Camera *camera, TerrainBody *sun, Frame *renderFrame) {
        if(atmosphere == nullptr) return;

        // The shell's inner surface is back-face-culled once the camera is
        // inside it, so it's invisible from the surface anyway; skip the draw
        // call explicitly rather than issue a fully-culled one (a proper
        // interior sky-dome is the §8.5 enhancement).
        const glm::dvec3 center = glm::dvec3(transform[3]);
        if(glm::length(camera->GetPos() - center) < (double)atm_radius) return;

        const glm::dmat4 &View = camera->GetView();
        // double, then truncate (shifted into the render frame like the view;
        // the Normal/cameraPos uniforms below stay in world coordinates, a
        // consistent pair for the V = worldPos - cameraPos shader math)
        glm::dmat4 ModelView = View * glm::translate(-camera->GetRenderOrigin()) * transform;
        glm::mat4 ModelViewFloat = ModelView;
        const glm::mat4 &Projection = camera->GetProjection();
        const glm::mat4 &Model = transform;

        atmosphere->shader->Bind();
        atmosphere->shader->setUniform_mat4(0, Projection * ModelViewFloat);
        atmosphere->shader->setUniform_mat4(1, Model);            // world normal/pos
        atmosphere->shader->setUniform_vec3(2, glm::vec3(camera->GetPos()));
        atmosphere->shader->setUniform_vec3(3, surface.atmosphere.color);
        atmosphere->shader->setUniform_vec1(4, surface.atmosphere.intensity);
        atmosphere->shader->setUniform_vec1(5, surface.atmosphere.power);
        // Direction light travels (sun -> planet); the same value the terrain
        // lights with, so the rim brightens on the day side and fades on the
        // night side instead of glowing uniformly ("neon night-side" artifact).
        atmosphere->shader->setUniform_vec3(6,
            glm::vec3(SunlightDir(this, sun, renderFrame)));

        // Transparent: blend over whatever is behind (terrain haze / starfield
        // ring), keep depth test so closer opaque things still occlude us, but
        // don't write depth so we don't cull the plumes/HUD or each other.
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(false);
        atmosphere->mesh->Draw();
        glDepthMask(true);
        glDisable(GL_BLEND);
    }

    // Direction light travels (sun -> object) in renderFrame's axes, where
    // `object_root` is the lit object's position in universe (root) coords.
    // Computing it from the object's actual position -- rather than the SOI
    // body's center -- is what keeps this defined when the SOI body IS the
    // star: sun->center collapses to a zero vector and normalize(0) = NaN,
    // which turned the ship's fragments NaN and rendered it black.
    static glm::dvec3 LightDirFrom(const glm::dvec3 &object_root,
                                   TerrainBody *sun, Frame *renderFrame) {
        const glm::dvec3 d = object_root - sun->frame->root_pos; // sun -> object
        if(glm::length(d) < 1e-9) { return glm::dvec3(0, 1, 0); }
        return glm::normalize(d) * renderFrame->root_orient;
    }

    // The lit object is the SOI body's own center (terrain / atmosphere).
    static glm::dvec3 SunlightDir(TerrainBody *planet, TerrainBody *sun,
                                  Frame *renderFrame) {
        return LightDirFrom(planet->frame->root_pos, sun, renderFrame);
    }

    void Draw(const Camera* camera, TerrainBody *sun, Frame *renderFrame) {
        sunlightVec = glm::vec3(SunlightDir(this, sun, renderFrame));

        // two passes with a stencil mask: pass 1 draws the terrain and
        // stamps stencil=1; pass 2 draws the skirts only where stencil==0,
        // i.e. where no terrain fragment was drawn (the cracks between
        // patches at different subdivision depths, and the limb). Hiding
        // the skirt under the neighbouring surface this way needs no depth
        // comparison, which the float32 view transform can't resolve at
        // range (its rounding is of the same order as the skirt depth
        // margin, which z-fights).
        glEnable(GL_STENCIL_TEST);
        glStencilFunc(GL_ALWAYS, 1, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
        for(auto&& patch : patches) {
            patch->Draw(camera, transform, sunlightVec, false);
        }
        glStencilFunc(GL_EQUAL, 0, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
        for(auto&& patch : patches) {
            patch->Draw(camera, transform, sunlightVec, true);
        }
        glDisable(GL_STENCIL_TEST);
    }

    void Update(const Camera* camera, int max_patch_px) {
        for(auto&& patch : patches) {
            patch->Update(camera, transform, max_patch_px);
        }
    }

    int CountPatches() {
        int ret = 0;
        for(auto&& patch : patches) {
            ret += patch->CountChildren();
        }
        return ret;
    }
};

// Elevation palette samplers, one per body type (assigned to
// TerrainBody::colour_func by load_system()).
COLOUR GetColourMoon(float v, float vmin, float vmax);
COLOUR GetColourSun(float v, float vmin, float vmax);
COLOUR GetColourEarth(float v, float vmin, float vmax);

// Per-part terrain shadow factor: 1.0 = lit, <1.0 = the planet's terrain
// occludes the line to the sun.
float ComputeTerrainShadow(TerrainBody *planet, const Frame *posFrame,
                           const glm::dvec3 &posInFrame, TerrainBody *sun);
