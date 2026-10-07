// terrain.h -- terrain data + geometry types.
//
//   GeoPatch           one subdividable spherical patch (terrain LOD node).
//   TerrainBody        a celestial body's terrain: 6 root patches,
//                      atmosphere shell, height/shadow queries.
//   BodyType           planet / star / moon, from the system JSON's "type".
//
// Pure terrain math (height/color, grid builder, biomes) lives in
// terragen.h (glm only, testable without GL/Bullet). The GeoPatch ctor
// consumes its GridGeom on the main thread; async subdivision snapshots a
// TerrainParams for the worker (see job.h).

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

// What one body's patch tree looks like right now (TerrainBody::lodStats,
// printed by --terrain-log). `deep_off` should be a patch width or two.
// Read it only when `deepest > 0` (0 leaves would read as "at the camera").
struct TerrainLodStats {
    int patches = 0;        // alive patches (every level, not just leaves)
    int deepest = 0;        // the deepest leaf's depth
    int collision = 0;      // leaves carrying a Bullet body (depth == max)
    double deep_off = 0;    // [m] camera -> the nearest deepest leaf's point
    double cam_r = 0;       // [m] camera -> the body centre
};

struct GeoPatch {
    TerrainBody *body;
    Mesh *mesh;      // OWNED (procedural grid, unique per patch)
    Shader *shader;  // shared (the body's terrain shader)
    btRigidBody *collision;

    GeoPatch *kids[4];

    glm::vec3 centroid;
    glm::vec3 v0, v1, v2, v3;

    // Body-frame anchor [m] the mesh vertices are baked relative to
    // (the float32 precision fix; see buildGridGeom). Draw folds it into
    // the modelview in double; the collision rigid body is translated by it.
    glm::dvec3 anchor;

    int depth;

    // Cached in the ctor (constant per patch): Update() runs every frame
    // over every alive patch.
    double centroid_height;   // [m] terrain radius at `centroid`
    double width_m;           // [m] characteristic size (mean edge chord)

    // A requestSubdivide job is in flight: the worker is building the four
    // children's grids and the continuation will attach (or discard) them.
    bool subdivide_in_flight = false;

    GeoPatch(TerrainBody *body, Shader *shader, int depth,
             glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 v3,
             const GridGeom &geom);
    ~GeoPatch();

    // skirt_pass=false draws the terrain, true draws only the skirt ring
    // (which depth-tests against the terrain drawn first). Shader bind +
    // body-constant uniforms happen once per pass in TerrainBody::Draw.
    void Draw(const Camera* camera, bool skirt_pass);
    // cam_bf: the camera in BODY-FIXED axes (cameraInBodyFrame -- the body's
    // spin must be undone, not just its position subtracted). px_per_rad:
    // the camera's screen scale (lodPxPerRad).
    // max_patch_px: subdivide while the patch projects wider than this
    // [screen px]; collapse below half (the hysteresis band). Subdivision
    // is async: the parent keeps drawing until its children land.
    void Update(const glm::dvec3 &cam_bf, double px_per_rad,
                int max_patch_px, JobRunner &jobs);

    // Post the async subdivision job (main thread). The worker builds the
    // four children's GridGeoms (pure math, terragen.h); the main-thread
    // continuation does the GL upload + Bullet collision and attaches them.
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

/* What kind of body this is, from the system JSON's "type" (system.h). */
enum class BodyType : unsigned char {
    Planet,   // default: a body that is neither star nor moon
    Star,     // the system's light source / frame-tree root (System::root)
    Moon,     // orbits a planet
};

struct TerrainBody {
    // Value-initialized so ~TerrainBody (which unconditionally deletes all
    // six slots) is safe for a body whose AttachRoot never ran.
    GeoPatch *patches[6] = { nullptr };
    Shader *shader;
    /* A demand-built shell: a PROCEDURAL mesh (unique to the body) drawn
       with a shared shader, and for the clouds a baked coverage texture.
       The body OWNS the mesh and the texture; the shader is shared. */
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
    /* Planetary rings (built on demand): one flat annulus per band in
       surface.rings, in the body's equatorial plane. The shared ring shader
       + the per-band albedo / opacity live in surface.rings; the body owns
       only the meshes. */
    std::vector<Mesh *> ring_meshes;
    Shader *ring_shader = nullptr;
    float radius;
    double mu;
    double g; // [m/s^2]
    float mass;
    /* Science distance fields (system JSON, written by utils/sci_dist.py).
       science_mult is the score body weight (hand-editable); transfer_dv is
       the approach leg [m/s] for display: home->planet for planets,
       parent->moon for moons. */
    double science_mult = 1.0;
    double transfer_dv = 0.0;
    std::string name;
    BodyType type = BodyType::Planet;   // from the system JSON's "type"
    /* A star has no classifiable surface. Every shipped system also makes
       the star the frame-tree root, but the two are parsed independently,
       so nothing enforces it. */
    bool isStar() const { return type == BodyType::Star; }
    /* A surface you can stand on and classify into biomes. A star and a
       banded gas giant have none (can still be orbited; findings there are
       global "of the body", biome left empty). */
    bool hasClassifiableSurface() const { return !isStar() && !surface.bands; }
    double seed = 0;   // noise-domain offset; 0 = legacy pattern
    // Subdivision stop (set by AttachRoot from the radius). One extra level
    // per radius doubling keeps mesh density per surface area comparable.
    int max_depth = 14;
    // Root terrain attached (drawable). False until the heavy phase lands:
    // the body still simulates but Draw/Update skip it.
    bool ready = false;
    Surface surface;
    Frame *frame; // owner
    Frame *rot_frame; // owner
    Calendar cal; // this body's day/year (from its spin + orbit rates)
    glm::dmat4 transform = glm::dmat4(1.0);
    glm::vec3 sunlightVec;

    // Main-thread-only liveness registry (GeoPatch ctor inserts, dtor
    // erases): an in-flight terrain job's continuation checks its target
    // patch still exists before touching it.
    std::set<GeoPatch*> alive;
    bool patchAlive(GeoPatch *p) const { return alive.count(p) > 0; }

    // The body's terrain math as a value snapshot for the worker thread.
    // CACHED, by reference: both writers refresh the cache, so the hot
    // per-frame queries don't deep-copy the palette vector every call.
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

    // The ships currently in this body's SOI. A SoI crossing moves them
    // between bodies' lists (Vehicle::setSoi). Aboard characters are NOT
    // here -- they live on their ship (Vehicle::crew); a free EVA character
    // IS a ship in this list.
    std::vector<Vehicle *> ships;
    // The space pads on this body. Drawn by render.cpp; StaticBuilding::Draw
    // culls itself when the active ship is off-body.
    std::vector<StaticBuilding *> pads;

    COLOUR (*colour_func)(float v, float vmin, float vmax);

    // Thin delegate to the pure height function (terragen.h).
    float GetTerrainHeight(const glm::vec3& p) const {
        return terrainHeight(p, params());
    }
    // The surface color at a unit direction in the body's rotating frame:
    // the exact per-vertex color the grid bakes, so surfmap matches the
    // rendered surface.
    glm::vec3 SurfaceColor(const glm::vec3& p) const {
        return terrainSurfaceColor(p, params());
    }

    // The rim / deck shell sphere (defined in terrain.cpp); res = latitude
    // = longitude rings.
    Mesh *create_atmosphere_mesh(float radius, int res);
    // A flat annulus in the local XZ plane (normal +Y) between inner and
    // outer radius [m]; res = angular segments (defined in terrain.cpp).
    Mesh *create_ring_mesh(double inner, double outer, int res);

    // The heavy per-load work, split so the CPU-bound part can run on the
    // JobRunner worker and the GL/Bullet part on the main thread.
    // radius/mass are already set by the light phase (load_system).
    struct RootGeoms {
        int max_depth;
        float max_height;
        std::array<GridGeom, 6> geoms;
    };

    // Worker-safe (const, pure math): the subdivision stop, the highest
    // relief, and the six root grids. No GL, no Bullet, no body-state write.
    RootGeoms BuildRootGeoms() const {
        // Mesh density per surface area: one extra subdivision level per
        // radius doubling, anchored at 14 for a Kerbin-sized (600 km) body.
        // Floor of 8 so tiny moons don't build useless depth; 12 is the
        // ceiling (the skirt has a precision issue at depth >= 13).
        int md = 14 + (int)llround(std::log2((double)radius / 600.0e3));
        if (md < 8) { md = 8; }
        if (md > 12) { md = 12; }

        // Highest relief above sea level: sizes the palette ramp + the
        // atmosphere/cloud shells. terrainHeight does not read max_height,
        // so this is self-contained.
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
        // ramp reads it). surface.max_height keeps its default until
        // AttachRoot applies this result.
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

    // Main thread: apply the worker-built root terrain and flip `ready`.
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
    // highest terrain so no peak pokes through. See
    // reports/atmosphere2026_08_25 for the model.
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

    // Build the cloud deck shell on demand (above the highest terrain).
    //
    // The coverage is BAKED once (equirectangular R8, the cloudCover FBM in
    // terragen.h) on the JobRunner worker: this call posts it and the deck
    // immediately draws a solid 1x1 placeholder, then the continuation
    // uploads the real grid.
    void BuildClouds(Shader *cloudshader, int res, JobRunner &jobs) {
        if(clouds != nullptr || !surface.clouds.enabled) return;
        float shell_radius = radius + surface.max_height
                             + surface.clouds.height;
        if(shell_radius <= radius) shell_radius = radius * 1.02f;
        Mesh *m = create_atmosphere_mesh(shell_radius, res);
        // Solid placeholder (coverage 1) until the bake lands.
        const unsigned char solid = 255;
        clouds = new Shell;
        // The Shell owns the texture; the bake re-uploads into it (no swap).
        clouds->mesh = m;
        clouds->shader = cloudshader;
        clouds->texture = make_coverage_texture(1, 1, &solid, true);
        cloud_radius = shell_radius;

        // The bake layout MUST match the deck shader's UV (cloudShader.vs):
        // u = lon/2pi + 0.5 (lon 0 = +Z), v = 0.5 - lat/pi (row 0 = north).
        const int W = 2048, H = 1024;
        // Snapshots for the worker (job.h: no game state, GL or imgui).
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
            // Main-thread continuation: upload to the texture the deck
            // already draws. The shared_ptr lets the buffer outlive this
            // body (std::function needs copyable captures).
            std::shared_ptr<std::vector<unsigned char> > ppx =
                std::make_shared<std::vector<unsigned char> >(std::move(px));
            return [tex, W, H, ppx]() {
                upload_coverage_r8(tex, W, H, ppx->data());
            };
        });
    }

    // Build the ocean surface shell on demand: a UV sphere at sea level
    // (land pokes through via the depth test). Only the Mesh mode has a
    // shell: Flat paints the sea on the terrain itself, and a future mode
    // gets one only if it asks.
    void BuildOcean(Shader *oceanshader) {
        if(ocean != nullptr || !surface.has_sea
           || surface.ocean_mode != OceanMode::Mesh) return;
        // Slight offset above sea level: without it the terrain and ocean
        // surfaces are coplanar at the coastline and z-fight.
        float shell_radius = radius + surface.sea_level + 0.1f;
        Mesh *m = create_atmosphere_mesh(shell_radius, 128);
        ocean = new Shell;
        ocean->mesh = m;
        ocean->shader = oceanshader;
        ocean->texture = nullptr;
        ocean_radius = shell_radius;
    }

    // Build the ring annuli on demand: one flat mesh per band in
    // surface.rings. The per-band albedo / opacity stay in surface.rings
    // (DrawRings reads them); the body owns only the meshes + the shader.
    void BuildRings(Shader *ringshader) {
        if(ring_shader != nullptr || surface.rings.empty()) return;
        ring_shader = ringshader;
        for(const RingParams &rp : surface.rings) {
            ring_meshes.push_back(create_ring_mesh(rp.inner, rp.outer, 128));
        }
    }

    // Main thread: the full heavy phase -- attach the worker-built root
    // terrain and the demand-built shells. Used for the boot's sync set and
    // from the deferred bodies' JobRunner continuations.
    void Finish(const RootGeoms &r, Shader *atmos, Shader *cloud, int cloudres,
                Shader *ocean, Shader *rings, JobRunner &jobs) {
        AttachRoot(r);
        BuildAtmosphere(atmos);
        BuildClouds(cloud, cloudres, jobs);
        BuildOcean(ocean);
        BuildRings(rings);
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
        // later shells aren't clipped.
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

        // Inside the shell: draw the inner surface as the sky dome. Cull
        // FRONT (the visible faces are all back faces from inside) and let
        // the shader use the inside blend (horizon haze, not the Fresnel
        // rim).
        const glm::dvec3 center = glm::dvec3(transform[3]);
        const bool inside = glm::length(camera->GetPos() - center) < (double)atm_radius;

        const glm::dmat4 &View = camera->GetView();
        // double, then truncate (the Normal/cameraPos uniforms stay in world
        // coordinates, a consistent pair for the shader's V math)
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
        // lights with, so the rim brightens on the day side (not "neon
        // night-side").
        atmosphere->shader->setUniform_vec3(6,
            glm::vec3(SunlightDir(this, sun, renderFrame)));
        atmosphere->shader->setUniform_vec1(7, inside ? 1.0f : 0.0f);
        atmosphere->shader->setUniform_vec3(8, glm::vec3(center));

        // Transparent: blend over whatever is behind, keep depth test so
        // closer opaque things still occlude us, but don't write depth.
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

        // Same inside/outside flip as the atmosphere: cull FRONT under the
        // deck so the near wall reads as the solid ceiling.
        const glm::dvec3 center = glm::dvec3(transform[3]);
        const bool inside = glm::length(camera->GetPos() - center) < (double)cloud_radius;

        const glm::dmat4 &View = camera->GetView();
        // double, then truncate (the view is built in the render frame;
        // Normal/cameraPos stay in world coordinates).
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
        // Drift as a horizontal UV offset (data drift is pattern units/s).
        clouds->shader->setUniform_vec1(5,
            (float)(time * surface.clouds.drift)
            / (float)(2.0 * glm::pi<double>() * surface.clouds.freq));
        clouds->shader->setUniform_vec3(6, glm::vec3(center));
        clouds->shader->setUniform_i(7, 0);   // coverage_tex -> unit 0
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, clouds->texture->id);

        // Transparent over the terrain (under the atmosphere rim): keep
        // the depth test so the limb stays correct, but don't write depth.
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

    // Draw the ring annuli: transparent, over the opaque terrain (the depth
    // buffer hides the far arc). Cull the far face -- with culling off BOTH
    // coincident faces blend and the band renders at 2a-a^2 instead of its
    // stated opacity. The shader uses two-sided (abs) Lambert and a
    // body-local cylinder test for the planet's sun shadow.
    void DrawRings(const Camera *camera, TerrainBody *sun, Frame *renderFrame) {
        if(ring_meshes.empty()) return;

        // The ring plane passes through the body centre with normal = the
        // body's pole (local +Y in world).
        const glm::dvec3 center = glm::dvec3(transform[3]);
        const glm::dvec3 pole = glm::dvec3(transform[1]);
        const bool below =
            glm::dot(camera->GetPos() - center, pole) < 0.0;

        const glm::dmat4 &View = camera->GetView();
        // double, then truncate (same precision convention as the shells).
        glm::dmat4 ModelView = View * glm::translate(-camera->GetRenderOrigin())
                               * transform;
        glm::mat4 ModelViewFloat = ModelView;
        const glm::mat4 &Projection = camera->GetProjection();
        const glm::vec3 sunDir =
            glm::vec3(SunlightDir(this, sun, renderFrame));

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(false);
        if(below) glCullFace(GL_FRONT);
        for(size_t i = 0; i < ring_meshes.size(); i++) {
            ring_shader->Bind();
            ring_shader->setUniform_mat4(0, Projection * ModelViewFloat);
            ring_shader->setUniform_mat4(1, glm::mat4(transform));
            ring_shader->setUniform_vec3(2, sunDir);
            ring_shader->setUniform_vec1(3, surface.rings[i].albedo);
            ring_shader->setUniform_vec1(4, surface.rings[i].opacity);
            // Occluder for the sun-shadow test (body-local sphere at origin).
            ring_shader->setUniform_vec1(5, radius);
            ring_meshes[i]->Draw();
        }
        if(below) glCullFace(GL_BACK);
        glDepthMask(true);
        glDisable(GL_BLEND);
    }

    // Direction light travels (sun -> object) in renderFrame's axes, where
    // `object_root` is the lit object's position in universe (root) coords.
    // Computed from the object's actual position -- not the SOI body's
    // center -- so it stays defined when the SOI body IS the star
    // (normalize(0) = NaN rendered the ship black).
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

        // Body-constant uniforms upload once per pass; the MVP is per patch
        // (see GeoPatch::Draw).
        shader->Bind();
        shader->setUniform_mat4(1, glm::mat4(transform));
        shader->setUniform_vec3(2, sunlightVec);
        shader->setUniform_vec4(3, glm::vec4(0.8, 0.8, 0.8, 1.0));

        // Detail texture: shared by all bodies (the get_texture registry).
        static Texture *detail_tex = get_texture("res/textures/terrain_detail.png");
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, detail_tex->id);

        // Skirt pass fills the LOD cracks between patches at different
        // subdivision depths (and the limb). The skirt tail is drawn after
        // the terrain and depth-tests against it (mesh.h:DrawSkirt).
        for(auto&& patch : patches) {
            patch->Draw(camera, false);
        }
        for(auto&& patch : patches) {
            patch->Draw(camera, true);
        }
    }

    // The camera in this body's FIXED axes (cameraInBodyFrame): the LOD
    // measure and --terrain-log both need it.
    glm::dvec3 CameraInBodyFrame(const Camera *camera) const {
        return cameraInBodyFrame(transform, camera->GetPos());
    }

    void Update(const Camera* camera, int max_patch_px, JobRunner &jobs) {
        if(!ready) return;
        const glm::dvec3 cam_bf = CameraInBodyFrame(camera);
        // The screen scale depends only on the camera: do the projection
        // maths once here, not once per patch.
        const double px_per_rad = lodPxPerRad(camera->viewport_h, camera->fov);
        for(auto&& patch : patches) {
            patch->Update(cam_bf, px_per_rad, max_patch_px, jobs);
        }
    }

    // LOD telemetry (--terrain-log): the live tree relative to the camera.
    // Walks `alive` (every patch of this body), one pass per logged line.
    TerrainLodStats lodStats(const glm::dvec3 &cam_bf) const {
        TerrainLodStats s;
        s.cam_r = glm::length(cam_bf);
        for(const GeoPatch *p : alive) {
            s.patches++;
            if(p->kids[0] != NULL) { continue; }   // leaves carry the detail
            if(p->collision != NULL) { s.collision++; }
            const double off = glm::length(
                cam_bf - p->centroid_height * (glm::dvec3)p->centroid);
            if(p->depth > s.deepest) { s.deepest = p->depth; s.deep_off = off; }
            else if(p->depth == s.deepest and off < s.deep_off) { s.deep_off = off; }
        }
        return s;
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
// same (body, pad site). Drawn like terrain (culls itself off-body).
//
// Owned by its body (TerrainBody::pads). Render assets are shared (the
// get_mesh/get_texture registries own them), so ~TerrainBody only frees the
// rigid Body. Draw is defined in terrain.cpp.
class StaticBuilding {
public:
    TerrainBody *parent;
    TerrainBody *sun = nullptr; // the star (light source)
    Body *body;
    bool polar = false; // the pad site (default vs polar) -- the de-dup key

    void Draw(const Camera* camera, const TerrainBody *current, Frame *renderFrame);
};
