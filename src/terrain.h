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

#include <numbers>
#include <memory>
#include <set>
#include <string>

#include <glm/glm.hpp>

#include "terragen.h"
#include "mesh.h"
#include "shader.h"
#include "texture.h"
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
    Mesh *mesh;      // OWNED (procedural grid, unique per patch)
    Shader *shader;  // shared (the body's terrain shader)
    btRigidBody *collision;

    GeoPatch *kids[4];

    glm::vec3 centroid;
    glm::vec3 v0, v1, v2, v3;

    // Body-frame anchor point [m] the mesh vertices are baked relative to
    // (GridGeom::anchor -- the float32 precision fix; see buildGridGeom).
    // Draw folds it into the modelview in double; the collision rigid body
    // is translated by it.
    glm::dvec3 anchor;

    int depth;

    // Cached in the ctor (constant per patch): the Update() traversal
    // runs every frame over every alive patch, so it must not re-sample
    // the height function or re-measure the corners.
    double centroid_height;   // [m] terrain radius at `centroid`
    double width_m;           // [m] widest edge arc (v0-v3 chord * radius)

    // A requestSubdivide job is in flight: the worker is building the four
    // children's grids and the main-thread continuation will attach them
    // (or discard them if the collapse path cleared the flag first).
    bool subdivide_in_flight = false;

    GeoPatch(TerrainBody *body, Shader *shader, int depth,
             glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 v3,
             const GridGeom &geom);
    ~GeoPatch();

    // skirt_pass=false draws the terrain, true draws only the skirt ring
    // (which depth-tests against the terrain drawn first). The shader bind
    // + body-constant uniforms happen once per pass in TerrainBody::Draw;
    // this uploads the patch's OWN MVP (the anchor composition happens in
    // double, so the float32 uniform only ever holds patch-scale numbers)
    // and issues the mesh draws.
    void Draw(const Camera* camera, bool skirt_pass);
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
    // Value-initialized so ~TerrainBody (which unconditionally deletes all
    // six slots) is safe for a body whose AttachRoot never ran (e.g. an exit
    // while its deferred heavy phase is still queued): unattached slots are
    // nullptr, and deleting nullptr is a no-op.
    GeoPatch *patches[6] = { nullptr };
    Shader *shader;
    /* A demand-built shell (the atmosphere rim, the cloud deck): a
       PROCEDURAL mesh (unique to the body) drawn with a shared shader, and
       for the clouds a baked coverage texture. The body OWNS the mesh and
       the texture (~TerrainBody frees both); the shader is shared (the
       registry owns it). */
    struct Shell {
        Mesh *mesh;
        Shader *shader;
        Texture *texture;
    };
    Shell *atmosphere = nullptr; // Fresnel rim shell (built on demand)
    float atm_radius = 0.0f;     // shell radius [m]; 0 = no atmosphere
    Shell *clouds = nullptr;     // cloud deck shell (built on demand)
    float cloud_radius = 0.0f;   // deck radius [m]; 0 = no clouds
    Shell *ocean = nullptr;      // ocean surface shell (built on demand)
    float ocean_radius = 0.0f;   // shell radius [m]; 0 = no ocean
    float radius;
    double mu;
    double g; // [m/s^2]
    double soi; // [m]
    float mass;
    std::string name;
    double seed = 0;   // noise-domain offset; 0 = legacy pattern
    // Subdivision stop for this body's patch tree (set by AttachRoot from
    // the radius, via BuildRootGeoms). The patch angular size at a given
    // depth is the same on every body, so leaf-cell METRES scale with the
    // radius: one extra level per radius doubling keeps mesh density per
    // surface area comparable across bodies (the old constant 14 left big
    // bodies with a sparse mesh and tiny ones over-subdivided).
    int max_depth = 14;
    // Root terrain attached (drawable). False until the heavy phase lands:
    // the body still simulates (frames) but Draw/Update/CountPatches skip
    // it, so a deferred body simply isn't drawn until its mesh is ready.
    bool ready = false;
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
    // (the same const data the mesh bakes). CACHED, by reference: the
    // inputs are written in exactly two places -- load_system pass 1 and
    // AttachRoot (applying the computed max_height) -- and both refresh
    // the cache, so the hot per-frame queries (GetTerrainHeight via
    // ComputeTerrainShadow, the HUD readouts) don't deep-copy the palette
    // vector every call.
    const TerrainParams &params() const { return paramsCache_; }
    void refreshParamsCache() {
        paramsCache_.surface = surface;
        paramsCache_.radius = radius;
        paramsCache_.colour_func = colour_func;
    }
    TerrainParams paramsCache_;

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

    // Thin delegate to the pure height function (terragen.h): the full-
    // detail analytic surface (physics, spawning, shadows, the HUD).
    float GetTerrainHeight(const glm::vec3& p) const {
        return terrainHeight(p, params());
    }
    // The surface color at a unit direction in the body's rotating frame:
    // the exact per-vertex color the grid bakes (noise, palette / band,
    // sea, contrast), so the 2-D surface map (surfmap.cpp) matches the
    // rendered surface pixel for pixel.
    glm::vec3 SurfaceColor(const glm::vec3& p) const {
        return terrainSurfaceColor(p, params());
    }

    // The rim / deck shell sphere (defined in terrain.cpp with the other
    // mesh builders); res = latitude = longitude rings.
    Mesh *create_atmosphere_mesh(float radius, int res);

    // The heavy per-load work, split so the CPU-bound part can run on the
    // JobRunner worker and the GL/Bullet part on the main thread (the same
    // pure-math/publish split as GeoPatch::requestSubdivide). radius/mass
    // are already set by the light phase (load_system); `ready` flips on
    // when AttachRoot lands.
    struct RootGeoms {
        int max_depth;
        float max_height;
        std::array<GridGeom, 6> geoms;
    };

    // Worker-safe (const, pure math): the subdivision stop, the highest
    // relief, and the six root grids. No GL, no Bullet, no body-state write
    // (the result is returned), so it runs on the JobRunner worker.
    RootGeoms BuildRootGeoms() const {
        // Mesh density per surface area: one extra subdivision level per
        // radius doubling, anchored at 14 for a Kerbin-sized (600 km)
        // body. Floor of 8 so tiny moons don't build useless depth; 12 is
        // the ceiling (the skirt has a precision issue at depth >= 13).
        int md = 14 + (int)llround(std::log2((double)radius / 600.0e3));
        if (md < 8) { md = 8; }
        if (md > 12) { md = 12; }

        // Highest relief above sea level (moved out of load_system): sizes
        // the palette ramp + the atmosphere/cloud shells. terrainHeight
        // does not read max_height, so this is self-contained.
        float maxh;
        if (surface.bands) {
            maxh = 0.0f;   // gas giant: smooth sphere
        } else {
            const TerrainParams tp0 = params();
            const int N = 2048;
            const float golden = 2.39996322972865332f;   // golden angle
            float hi = 0.0f;
            for (int i = 0; i < N; i++) {
                const float y = 1.0f - 2.0f * (i + 0.5f) / (float)N;
                const float rr = std::sqrt(std::max(0.0f, 1.0f - y * y));
                const glm::vec3 d(rr * std::cos(i * golden), y,
                                  rr * std::sin(i * golden));
                hi = std::max(hi, terrainHeight(d, tp0) - radius);
            }
            maxh = std::max(1.0f, (hi - surface.sea_level) * 1.05f);
        }

        // Bake the six root grids with the COMPUTED max_height (the palette
        // ramp reads it); the body's surface.max_height is still its default
        // until AttachRoot applies this result.
        TerrainParams tp = params();
        tp.surface.max_height = maxh;
        const glm::vec3 p1 = glm::normalize(glm::vec3( 1, 1, 1));
        const glm::vec3 p2 = glm::normalize(glm::vec3(-1, 1, 1));
        const glm::vec3 p3 = glm::normalize(glm::vec3(-1,-1, 1));
        const glm::vec3 p4 = glm::normalize(glm::vec3( 1,-1, 1));
        const glm::vec3 p5 = glm::normalize(glm::vec3( 1, 1,-1));
        const glm::vec3 p6 = glm::normalize(glm::vec3(-1, 1,-1));
        const glm::vec3 p7 = glm::normalize(glm::vec3(-1,-1,-1));
        const glm::vec3 p8 = glm::normalize(glm::vec3( 1,-1,-1));

        RootGeoms r;
        r.max_depth = md;
        r.max_height = maxh;
        r.geoms[0] = buildGridGeom(tp, true, 1, p1, p2, p3, p4);
        r.geoms[1] = buildGridGeom(tp, true, 1, p4, p3, p7, p8);
        r.geoms[2] = buildGridGeom(tp, true, 1, p1, p4, p8, p5);
        r.geoms[3] = buildGridGeom(tp, true, 1, p2, p1, p5, p6);
        r.geoms[4] = buildGridGeom(tp, true, 1, p3, p2, p6, p7);
        r.geoms[5] = buildGridGeom(tp, true, 1, p8, p7, p6, p5);
        return r;
    }

    // Main thread: apply the worker-built root terrain (GL upload + Bullet
    // collision) and flip `ready` so the body becomes drawable.
    void AttachRoot(const RootGeoms &r) {
        max_depth = r.max_depth;
        surface.max_height = r.max_height;
        refreshParamsCache();   // the palette ramp reads max_height
        const glm::vec3 p1 = glm::normalize(glm::vec3( 1, 1, 1));
        const glm::vec3 p2 = glm::normalize(glm::vec3(-1, 1, 1));
        const glm::vec3 p3 = glm::normalize(glm::vec3(-1,-1, 1));
        const glm::vec3 p4 = glm::normalize(glm::vec3( 1,-1, 1));
        const glm::vec3 p5 = glm::normalize(glm::vec3( 1, 1,-1));
        const glm::vec3 p6 = glm::normalize(glm::vec3(-1, 1,-1));
        const glm::vec3 p7 = glm::normalize(glm::vec3(-1,-1,-1));
        const glm::vec3 p8 = glm::normalize(glm::vec3( 1,-1,-1));
        patches[0] = new GeoPatch(this, shader, 1, p1, p2, p3, p4, r.geoms[0]);
        patches[1] = new GeoPatch(this, shader, 1, p4, p3, p7, p8, r.geoms[1]);
        patches[2] = new GeoPatch(this, shader, 1, p1, p4, p8, p5, r.geoms[2]);
        patches[3] = new GeoPatch(this, shader, 1, p2, p1, p5, p6, r.geoms[3]);
        patches[4] = new GeoPatch(this, shader, 1, p3, p2, p6, p7, r.geoms[4]);
        patches[5] = new GeoPatch(this, shader, 1, p8, p7, p6, p5, r.geoms[5]);
        ready = true;
    }

    // Build the atmosphere rim shell on demand. It sits just above the
    // highest terrain so no peak pokes through: base radius + scaled relief
    // + the data thickness. See reports/atmosphere2026_08_25 for the model.
    void BuildAtmosphere(Shader *atmosphereshader) {
        if(atmosphere != nullptr || !surface.atmosphere.enabled) return;
        float shell_radius = radius + surface.max_height
                             + surface.atmosphere.thickness;
        if(shell_radius <= radius) shell_radius = radius * 1.02f;
        Mesh *m = create_atmosphere_mesh(shell_radius, 128);
        atmosphere = new Shell;
        atmosphere->mesh = m;
        atmosphere->shader = atmosphereshader;
        atmosphere->texture = nullptr;
        atm_radius = shell_radius;
    }

    // Build the cloud deck shell on demand. Like the atmosphere it sits
    // above the highest terrain (no peak pokes through): base radius +
    // scaled relief + the data height.
    //
    // The coverage is BAKED once (equirectangular R8, the cloudCover FBM
    // in terragen.h) -- the pattern is static in the body's frame, so the
    // shader fetches it instead of recomputing it per fragment. The bake
    // itself is ~0.4s per body of CPU work, so it runs on the JobRunner
    // worker: this call posts it and the deck immediately draws a solid
    // 1x1 placeholder (the "solid ceiling" read), then poll()'s
    // continuation uploads the real grid and the pattern refines in.
    void BuildClouds(Shader *cloudshader, int res, JobRunner &jobs) {
        if(clouds != nullptr || !surface.clouds.enabled) return;
        float shell_radius = radius + surface.max_height
                             + surface.clouds.height;
        if(shell_radius <= radius) shell_radius = radius * 1.02f;
        Mesh *m = create_atmosphere_mesh(shell_radius, res);
        // Solid placeholder (coverage 1): the deck reads as the solid
        // ceiling from the first frame until the bake lands.
        const unsigned char solid = 255;
        clouds = new Shell;
        // The Shell owns the texture (~TerrainBody frees it); the bake
        // re-uploads into it (upload_coverage_r8), so no texture swap.
        clouds->mesh = m;
        clouds->shader = cloudshader;
        clouds->texture = make_coverage_texture(1, 1, &solid, true);
        cloud_radius = shell_radius;

        // The bake layout MUST match the deck shader's UV (cloudShader.vs):
        // u = lon/2pi + 0.5 with lon = atan2(x, z) (the game's convention,
        // lon 0 = +Z); v = 0.5 - lat/pi, so row 0 (v=0) is the north pole.
        // Cell centers (px + 0.5) land on the same texel centers the
        // shader's UV hits.
        const int W = 2048, H = 1024;
        // Snapshots for the worker (job.h: the body may only touch its own
        // copy of the inputs -- not game state, GL or imgui).
        const glm::mat3 rot = surface.seed_rot;
        const CloudParams cp = surface.clouds;
        Texture *tex = clouds->texture;
        const std::string label = std::string("Clouds (") + name + ")";
        jobs.post(label, [tex, rot, cp, W, H]() -> std::function<void()> {
            // Worker thread: pure math over the snapshots above.
            std::vector<unsigned char> px((size_t)W * H);
            for(int py = 0; py < H; py++) {
                const float lat = (0.5f - (py + 0.5f) / (float)H) * (float)std::numbers::pi;
                const float cl = (float)std::cos(lat);
                const float sl = (float)std::sin(lat);
                for(int pxi = 0; pxi < W; pxi++) {
                    const float lon = ((pxi + 0.5f) / (float)W)
                                      * 2.0f * (float)std::numbers::pi - (float)std::numbers::pi;
                    const glm::vec3 dir(cl * std::sin(lon), sl,
                                        cl * std::cos(lon));
                    px[(size_t)py * W + pxi] =
                        (unsigned char)(cloudCover(dir, rot, cp) * 255.0f
                                        + 0.5f);
                }
            }
            // Main-thread continuation (JobRunner::poll): upload to the
            // texture the deck already draws. The shared_ptr lets the
            // buffer outlive this body (std::function needs copyable
            // captures -- the same idiom as the surface map job).
            std::shared_ptr<std::vector<unsigned char> > ppx =
                std::make_shared<std::vector<unsigned char> >(std::move(px));
            return [tex, W, H, ppx]() {
                upload_coverage_r8(tex, W, H, ppx->data());
            };
        });
    }

    // Build the ocean surface shell on demand. A UV sphere at sea level:
    // land pokes through via the depth test, sea floor is covered by the
    // transparent water surface.
    void BuildOcean(Shader *oceanshader) {
        if(ocean != nullptr || !surface.has_sea) return;
        // Slight offset above sea level: at the exact coastline the terrain
        // and ocean surfaces are coplanar, and without the offset the depth
        // test flickers between them (z-fighting).
        float shell_radius = radius + surface.sea_level + 0.1f;
        Mesh *m = create_atmosphere_mesh(shell_radius, 128);
        ocean = new Shell;
        ocean->mesh = m;
        ocean->shader = oceanshader;
        ocean->texture = nullptr;
        ocean_radius = shell_radius;
    }

    // Main thread: the full heavy phase -- attach the worker-built root
    // terrain (BuildRootGeoms) and the demand-built shells (atmosphere rim,
    // cloud deck, ocean). Used synchronously for the boot-critical bodies
    // (home, its moon, the star) and from the deferred bodies' JobRunner
    // continuations.
    void Finish(const RootGeoms &r, Shader *atmos, Shader *cloud, int cloudres,
                Shader *ocean, JobRunner &jobs) {
        AttachRoot(r);
        BuildAtmosphere(atmos);
        BuildClouds(cloud, cloudres, jobs);
        BuildOcean(ocean);
    }

    void DrawOcean(const Camera *camera, TerrainBody *sun,
                   Frame *renderFrame, double time) {
        if(ocean == nullptr) return;

        const glm::dvec3 center = glm::dvec3(transform[3]);
        const bool inside = glm::length(camera->GetPos() - center)
                            < (double)ocean_radius;

        const glm::dmat4 &View = camera->GetView();
        glm::dmat4 ModelView = View * glm::translate(-camera->GetRenderOrigin())
                               * transform;
        glm::mat4 ModelViewFloat = ModelView;
        const glm::mat4 &Projection = camera->GetProjection();

        ocean->shader->Bind();
        ocean->shader->setUniform_mat4(0, Projection * ModelViewFloat);
        ocean->shader->setUniform_mat4(1, glm::mat4(transform));
        ocean->shader->setUniform_vec3(2, glm::vec3(camera->GetPos()));
        ocean->shader->setUniform_vec3(3, surface.sea_color);
        ocean->shader->setUniform_vec3(4,
            glm::vec3(SunlightDir(this, sun, renderFrame)));
        ocean->shader->setUniform_vec1(5, (float)time);
        ocean->shader->setUniform_vec3(6, glm::vec3(center));

        // Transparent over the terrain: depth test so land occludes the
        // ocean, but no depth write so the sea floor shows through and
        // later shells (clouds, atmosphere) aren't clipped.
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(false);
        if(inside) glCullFace(GL_FRONT);
        ocean->mesh->Draw();
        if(inside) glCullFace(GL_BACK);
        glDepthMask(true);
        glDisable(GL_BLEND);
    }

    void DrawAtmosphere(const Camera *camera, TerrainBody *sun, Frame *renderFrame) {
        if(atmosphere == nullptr) return;

        // Inside the shell (on the surface, or below its top): draw the
        // inner surface as the sky dome. From inside the eye is on the
        // inner side of every face, so the visible faces are all back
        // faces -- cull FRONT instead of the global BACK, and let the
        // shader use the inside blend (horizon haze, not the Fresnel rim).
        // The near wall is closer than the far side, so depth keeps the
        // sky reading as the air above the camera.
        const glm::dvec3 center = glm::dvec3(transform[3]);
        const bool inside = glm::length(camera->GetPos() - center) < (double)atm_radius;

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
        atmosphere->shader->setUniform_vec1(7, inside ? 1.0f : 0.0f);
        atmosphere->shader->setUniform_vec3(8, glm::vec3(center));

        // Transparent: blend over whatever is behind (terrain haze / starfield
        // ring), keep depth test so closer opaque things still occlude us, but
        // don't write depth so we don't cull the plumes/HUD or each other.
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(false);
        if(inside) glCullFace(GL_FRONT);
        atmosphere->mesh->Draw();
        if(inside) glCullFace(GL_BACK);
        glDepthMask(true);
        glDisable(GL_BLEND);
    }

    void DrawClouds(const Camera *camera, TerrainBody *sun,
                    Frame *renderFrame, double time) {
        if(clouds == nullptr) return;

        // Same inside/outside flip as the atmosphere: the eye below the
        // deck (on the surface, or in the air under it) is inside the
        // shell, so the visible faces are back faces -- cull FRONT and
        // the near wall reads as the solid ceiling. From orbit the default
        // cull shows the near-side exterior (the deck top over the planet).
        const glm::dvec3 center = glm::dvec3(transform[3]);
        const bool inside = glm::length(camera->GetPos() - center) < (double)cloud_radius;

        const glm::dmat4 &View = camera->GetView();
        // double, then truncate (the view is built in the render frame,
        // like the atmosphere path; the Normal/cameraPos uniforms stay in
        // world coordinates, a consistent pair for the shader's V math)
        glm::dmat4 ModelView = View * glm::translate(-camera->GetRenderOrigin()) * transform;
        glm::mat4 ModelViewFloat = ModelView;
        const glm::mat4 &Projection = camera->GetProjection();

        clouds->shader->Bind();
        clouds->shader->setUniform_mat4(0, Projection * ModelViewFloat);
        clouds->shader->setUniform_mat4(1, glm::mat4(transform));
        clouds->shader->setUniform_vec3(2, glm::vec3(camera->GetPos()));
        clouds->shader->setUniform_vec3(3, surface.clouds.color);
        // Direction light travels (sun -> planet); the same value the
        // terrain lights with, so the deck's terminator matches the ground.
        clouds->shader->setUniform_vec3(4,
            glm::vec3(SunlightDir(this, sun, renderFrame)));
        // Drift as a horizontal UV offset (the data drift is pattern
        // units/s; a full longitude is 2*pi*freq pattern units = 1.0 UV).
        clouds->shader->setUniform_vec1(5,
            (float)(time * surface.clouds.drift)
            / (float)(2.0 * glm::pi<double>() * surface.clouds.freq));
        clouds->shader->setUniform_vec3(6, glm::vec3(center));
        clouds->shader->setUniform_i(7, 0);   // coverage_tex -> unit 0
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, clouds->texture->id);

        // Transparent over the terrain (and under the atmosphere rim,
        // which draws after): keep the depth test so the limb stays
        // correct, but don't write depth so nothing behind the deck gets
        // clobbered (plumes, the rim shell).
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(false);
        if(inside) glCullFace(GL_FRONT);
        clouds->mesh->Draw();
        if(inside) glCullFace(GL_BACK);
        glDepthMask(true);
        glDisable(GL_BLEND);
        glBindTexture(GL_TEXTURE_2D, 0);
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
        if(!ready) return;   // root terrain not attached yet (deferred)
        sunlightVec = glm::vec3(SunlightDir(this, sun, renderFrame));

        // Body-constant uniforms upload once per pass; the MVP is per
        // patch (its mesh is anchor-relative, so the patch folds its own
        // anchor into the double-precision modelview -- see GeoPatch::Draw).
        shader->Bind();
        shader->setUniform_mat4(1, glm::mat4(transform));
        shader->setUniform_vec3(2, sunlightVec);
        shader->setUniform_vec4(3, glm::vec4(0.8, 0.8, 0.8, 1.0));

        // Detail texture: shared by all bodies, loaded once via the
        // get_texture registry (mipmapped, REPEAT wrap, anisotropic).
        static Texture *detail_tex = get_texture("res/terrain_detail.png");
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, detail_tex->id);

        // Skirt pass fills the LOD cracks between patches at different
        // subdivision depths (and the limb). The skirt tail is drawn after the
        // terrain and depth-tests against it (mesh.h:DrawSkirt), so it hides
        // under the surface and shows only in the gaps.
        // Reverse-Z's front-loaded near-field precision resolves the
        // skirt/terrain boundary that a stencil mask used to paper over, so the
        // plain depth test is enough now.
        for(auto&& patch : patches) {
            patch->Draw(camera, false);
        }
        for(auto&& patch : patches) {
            patch->Draw(camera, true);
        }
    }

    void Update(const Camera* camera, int max_patch_px, JobRunner &jobs) {
        if(!ready) return;
        for(auto&& patch : patches) {
            patch->Update(camera, transform, max_patch_px, jobs);
        }
    }

    int CountPatches() {
        if(!ready) return 0;
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
// Owned by its body (TerrainBody::pads). The pad's render assets are
// shared (the get_mesh/get_texture registries own them), so
// ~TerrainBody only frees the rigid Body (it unregisters from the Bullet
// world first, then ~Body frees the rigid body + hull shape) -- nothing
// to leak.
// Draw is defined in terrain.cpp (it needs the complete Body type).
class StaticBuilding {
public:
    TerrainBody *parent;
    TerrainBody *sun = nullptr; // the star (light source)
    Body *body;
    bool polar = false; // the pad site (default vs polar) -- the de-dup key

    void Draw(const Camera* camera, const TerrainBody *current, Frame *renderFrame);
};
