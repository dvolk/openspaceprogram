// terrain.h -- terrain data + geometry types.
//
//   GeoPatch           one subdividable spherical patch (terrain LOD node).
//   TerrainBody        a celestial body's terrain: 6 root patches,
//                      atmosphere shell, height/shadow queries.
//
// The pure terrain math (per-body params, the height/color functions, the
// grid builder) lives in terragen.h (glm only, so tests can pin it
// without GL/Bullet). The GeoPatch ctor consumes its GridGeom on the main
// thread, and the async subdivision job snapshots a TerrainParams for the
// worker -- the same pure-work/publish split as the porkchop grid and the
// surface map (job.h).

#pragma once

#include <set>
#include <string>

#include <glm/glm.hpp>

#include "terragen.h"
#include "model.h"
#include "mesh.h"
#include "shader.h"
#include "camera.h"
#include "frame.h"
#include "calendar.h"
#include "job.h"

class btRigidBody;

struct TerrainBody;
class Vehicle;          // a ship (vehicle.h); a body owns its ship list
struct Body;            // a rigid body (body.h); StaticBuilding's physics body
class StaticBuilding;   // a space pad (defined at the end of this file)

struct GeoPatch {
    TerrainBody *body;
    Model *model;
    btRigidBody *collision;
    static const int max_depth = 14;

    GeoPatch *kids[4];

    glm::vec3 centroid;
    glm::vec3 v0, v1, v2, v3;

    int depth;

    // A requestSubdivide job is in flight: the worker is building the four
    // children's grids and the main-thread continuation will attach them
    // (or discard them if the collapse path cleared the flag first).
    bool subdivide_in_flight = false;

    GeoPatch(TerrainBody *body, Shader *shader, int depth,
             glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 v3,
             const GridGeom &geom);
    ~GeoPatch();

    // skirt_pass=false draws the terrain (stamping stencil), true draws
    // only the skirt ring where the stencil says no terrain was drawn
    void Draw(const Camera* camera, const glm::dmat4& transform, const glm::vec3 & sunlightVec, bool skirt_pass);
    // max_patch_px: subdivide while the patch projects wider than this
    // [screen px]; collapse below half (the hysteresis band). Subdivision
    // is async (requestSubdivide): the parent keeps drawing until its
    // children land, so there is never a hole.
    void Update(const Camera* camera, const glm::dmat4& transform, int max_patch_px, JobRunner &jobs);

    // Post the async subdivision job (main thread). The worker builds the
    // four children's GridGeoms (pure math, terragen.h); the main-thread
    // continuation does the GL upload + Bullet collision and attaches the
    // children.
    void requestSubdivide(JobRunner &jobs);

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

    // Main-thread-only liveness registry (the GeoPatch ctor inserts, the
    // dtor erases): an in-flight terrain job's continuation uses it to
    // check its target patch still exists before touching it -- a
    // grandparent's collapse can free the whole subtree while the job is
    // still building the grids.
    std::set<GeoPatch*> alive;
    bool patchAlive(GeoPatch *p) const { return alive.count(p) > 0; }

    // The body's terrain math as a value snapshot for the worker thread
    // (the same const data the mesh bakes; set once in load_system).
    TerrainParams params() const {
        TerrainParams t;
        t.surface = surface;
        t.radius = radius;
        t.colour_func = colour_func;
        return t;
    }

    // Defined in terrain.cpp (it also deletes the ships + space pads below,
    // which needs the complete Vehicle / StaticBuilding types).
    ~TerrainBody();

    // The ships currently in this body's SOI (vehicle.h). While a ship is
    // here, its Vehicle::m_parent is this body; a SoI crossing moves it
    // between bodies' lists (Vehicle::moveToFrame) instead of copying
    // anything. Aboard characters are NOT here -- they live on their ship
    // (Vehicle::crew); a free EVA character IS a ship in this list.
    std::vector<Vehicle *> ships;
    // The space pads on this body (one per pad site). Drawn by render.cpp;
    // StaticBuilding::Draw culls itself when the active ship is off-body.
    std::vector<StaticBuilding *> pads;

    COLOUR (*colour_func)(float v, float vmin, float vmax);

    // Thin delegates to the pure functions (terragen.h).
    float GetTerrainHeight(const glm::vec3& p) const {
        return terrainHeight(p, params());
    }
    float GetTerrainHeightUnscaled(const glm::vec3& p) const {
        return terrainHeightUnscaled(p, params());
    }
    float ScaleHeightNoise(float noise) const {
        return terrainScaleHeightNoise(noise, params());
    }
    // The surface color at a unit direction in the body's rotating frame:
    // the exact per-vertex color the grid bakes (noise, palette / band,
    // sea, contrast), so the 2-D surface map (surfmap.cpp) matches the
    // rendered surface pixel for pixel.
    glm::vec3 SurfaceColor(const glm::vec3& p) const {
        return terrainSurfaceColor(p, params());
    }

    // The atmosphere rim shell (defined in terrain.cpp with the other
    // mesh builders).
    Mesh *create_atmosphere_mesh(float radius);

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

        // The six root patches (depth 1) build synchronously at load time:
        // before the main loop starts there is nothing else to draw, so
        // blocking here is fine. Child patches are async (GeoPatch::
        // requestSubdivide).
        patches[0] = new GeoPatch(this, shader, 1, p1, p2, p3, p4, buildGridGeom(params(), false, p1, p2, p3, p4));
        patches[1] = new GeoPatch(this, shader, 1, p4, p3, p7, p8, buildGridGeom(params(), false, p4, p3, p7, p8));
        patches[2] = new GeoPatch(this, shader, 1, p1, p4, p8, p5, buildGridGeom(params(), false, p1, p4, p8, p5));
        patches[3] = new GeoPatch(this, shader, 1, p2, p1, p5, p6, buildGridGeom(params(), false, p2, p1, p5, p6));
        patches[4] = new GeoPatch(this, shader, 1, p3, p2, p6, p7, buildGridGeom(params(), false, p3, p2, p6, p7));
        patches[5] = new GeoPatch(this, shader, 1, p8, p7, p6, p5, buildGridGeom(params(), false, p8, p7, p6, p5));
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

    void Update(const Camera* camera, int max_patch_px, JobRunner &jobs) {
        for(auto&& patch : patches) {
            patch->Update(camera, transform, max_patch_px, jobs);
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

// Per-part terrain shadow factor: 1.0 = lit, <1.0 = the planet's terrain
// occludes the line to the sun.
float ComputeTerrainShadow(TerrainBody *planet, const Frame *posFrame,
                           const glm::dvec3 &posInFrame, TerrainBody *sun);

// A space pad -- a static building shared by every ship standing on the
// same (body, pad site). Drawn like terrain (culls itself when the active
// ship is not on the pad's body); the light source is the star.
//
// Owned by its body (TerrainBody::pads). Each pad owns its own model
// (like a ship part), so ~TerrainBody can free the rigid Body (it
// unregisters from the Bullet world first, then ~Body frees the model +
// rigid body) -- nothing to leak.
// Draw is defined in terrain.cpp (it needs the complete Body type).
class StaticBuilding {
public:
    TerrainBody *parent;
    TerrainBody *sun = nullptr; // the star (light source)
    Body *body;
    bool polar = false; // the pad site (default vs polar) -- the de-dup key

    void Draw(const Camera* camera, const TerrainBody *current, Frame *renderFrame);
};
