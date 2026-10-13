// terrain.h -- terrain data + geometry types: GeoPatch (a subdividable
// spherical patch, the terrain LOD node) and TerrainBody (a body's terrain).
// Pure terrain math (height/color, grid builder, biomes) lives in terragen.h.

#pragma once

#include <numbers>
#include <memory>
#include <set>
#include <string>
#include <vector>

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
class Vehicle;          // a ship (vehicle.h)
struct Body;            // a rigid body (body.h)
class StaticBuilding;   // a space pad (defined at the end of this file)

// Patch-tree snapshot (--terrain-log). Read deep_off only when `deepest > 0`.
struct TerrainLodStats {
    int patches = 0;        // alive patches (every level, not just leaves)
    int deepest = 0;        // the deepest leaf's depth
    int collision = 0;      // leaves carrying a Bullet body (depth == max)
    double deep_off = 0;    // [m] camera -> the nearest deepest leaf's point
    double cam_r = 0;       // [m] camera -> the body centre
};

struct GeoPatch {
    TerrainBody *body;
    Mesh *mesh;      // OWNED
    Shader *shader;  // shared
    btRigidBody *collision;

    GeoPatch *kids[4];

    glm::vec3 centroid;
    glm::vec3 v0, v1, v2, v3;

    // Body-frame anchor [m] the mesh vertices are baked relative to; Draw
    // folds it into the modelview in double.
    glm::dvec3 anchor;

    int depth;

    // Cached in the ctor (per-patch constant), read every frame by Update().
    double centroid_height;   // [m] terrain radius at `centroid`
    double width_m;           // [m] characteristic size (mean edge chord)

    // A subdivision job is in flight (its continuation attaches or discards
    // the children).
    bool subdivide_in_flight = false;

    GeoPatch(TerrainBody *body, Shader *shader, int depth,
             glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 v3,
             const GridGeom &geom);
    ~GeoPatch();

    // skirt_pass=false draws the terrain, true only the skirt ring (which
    // depth-tests against the terrain drawn first).
    void Draw(const Camera* camera, bool skirt_pass);
    // cam_bf: camera in BODY-FIXED axes (spin undone, not just position).
    // max_patch_px [screen px]: subdivide above, collapse below half
    // (hysteresis). Subdivision is async.
    void Update(const glm::dvec3 &cam_bf, double px_per_rad,
                int max_patch_px, JobRunner &jobs);

    // Main thread: the worker builds the children's GridGeoms, the continuation
    // does the GL upload + Bullet collision. px_width [screen px] doubles as the
    // queue priority.
    void requestSubdivide(JobRunner &jobs, double px_width);

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

/* Body kind, from the system JSON's "type". */
enum class BodyType : unsigned char {
    Planet,   // default
    Star,     // light source; frame-tree root (System::root)
    Moon,     // orbits a planet
};

struct TerrainBody {
    /* Every member is initialized deliberately: a half-built body is always
       destructed (load_system registers it before the first throw it can hit),
       so an uninitialized scalar is a live UB path (#227). The sentinels below
       mean "not set yet", not "valid physics"; load_system fills all four first
       (mu derived from mass, not read from the JSON). */
    GeoPatch *patches[6] = { nullptr };
    Shader *shader = nullptr;   // shared, not owned; set by the type switch
    /* Demand-built shell: the body OWNS the procedural mesh and, for clouds,
       the coverage texture; the shader is shared. All nulled because free_shell
       deletes mesh and texture unconditionally (#230). */
    struct Shell {
        Mesh *mesh = nullptr;       // owner
        Shader *shader = nullptr;   // shared
        Texture *texture = nullptr; // owner (clouds only)
    };
    Shell *atmosphere = nullptr; // Fresnel rim shell (built on demand)
    float atm_radius = 0.0f;     // shell radius [m]; 0 = no atmosphere
    Shell *clouds = nullptr;     // cloud deck shell (built on demand)
    float cloud_radius = 0.0f;   // deck radius [m]; 0 = no clouds
    Shell *ocean = nullptr;      // ocean surface shell (built on demand)
    float ocean_radius = 0.0f;   // shell radius [m]; 0 = no ocean
    /* Rings: one flat annulus per band in surface.rings, in the equatorial
       plane; the body owns only the meshes. */
    std::vector<Mesh *> ring_meshes;
    Shader *ring_shader = nullptr;
    float radius = 0.0f;   // [m]
    double mu = 0.0;       // gravitational parameter [m^3/s^2]
    double g = 0.0;        // [m/s^2]
    float mass = 0.0f;     // [kg]
    /* Science fields (system JSON, utils/sci_dist.py). transfer_dv is the
       approach leg [m/s]: home->planet, parent->moon. */
    double science_mult = 1.0;
    double transfer_dv = 0.0;
    std::string name;
    BodyType type = BodyType::Planet;
    /* A star has no classifiable surface; being the frame-tree root is parsed
       separately, not enforced. */
    bool isStar() const { return type == BodyType::Star; }
    /* Standable and biome-classifiable. Stars and banded gas giants have none
       (findings there leave the biome empty). */
    bool hasClassifiableSurface() const { return !isStar() && !surface.bands; }
    double seed = 0.0;   // noise-domain offset; 0 = legacy pattern
    // Subdivision stop (set by AttachRoot from the radius).
    int max_depth = 14;
    // Root terrain attached. False until the heavy phase lands: Draw/Update skip
    // the body.
    bool ready = false;
    Surface surface;
    // Owned; deleted unconditionally by ~TerrainBody.
    Frame *frame = nullptr;
    Frame *rot_frame = nullptr;
    Calendar cal; // this body's day/year (from its spin + orbit rates)
    glm::dmat4 transform = glm::dmat4(1.0);
    glm::vec3 sunlightVec = glm::vec3(0.0f);   // written by Draw, once ready

    // Main-thread-only liveness registry (GeoPatch ctor inserts, dtor erases):
    // an in-flight job's continuation checks its patch still exists.
    std::set<GeoPatch*> alive;
    bool patchAlive(GeoPatch *p) const { return alive.count(p) > 0; }

    // Terrain math snapshot for the worker thread. Returned by reference:
    // writers must refresh the cache.
    const TerrainParams &params() const { return paramsCache_; }
    void refreshParamsCache() {
        paramsCache_.surface = surface;
        paramsCache_.radius = radius;
        paramsCache_.colour_func = colour_func;
    }
    TerrainParams paramsCache_;

    // Defined in terrain.cpp (needs the complete Vehicle/StaticBuilding types).
    ~TerrainBody();

    // Ships in this body's SOI (a SOI crossing moves them, Vehicle::setSoi).
    // Aboard crew are NOT here; a free EVA character IS.
    std::vector<Vehicle *> ships;
    // Space pads on this body (StaticBuilding::Draw culls itself off-body).
    std::vector<StaticBuilding *> pads;

    COLOUR (*colour_func)(float v, float vmin, float vmax) = nullptr;

    float GetTerrainHeight(const glm::vec3& p) const {
        return terrainHeight(p, params());
    }
    // p: unit direction in the body's rotating frame; the exact per-vertex color
    // the grid bakes.
    glm::vec3 SurfaceColor(const glm::vec3& p) const {
        return terrainSurfaceColor(p, params());
    }

    // The rim / deck shell sphere; res = latitude = longitude rings.
    Mesh *create_atmosphere_mesh(float radius, int res);
    // A flat annulus in the local XZ plane (normal +Y) between inner and
    // outer radius [m]; res = angular segments.
    Mesh *create_ring_mesh(double inner, double outer, int res);

    // Heavy per-load work, split: CPU-bound on the worker, GL/Bullet on the main
    // thread.
    struct RootGeoms {
        // Initialized: crosses to the main thread as a value, so a field
        // BuildRootGeoms forgets reads as zero, not garbage (#230).
        int max_depth = 0;
        float max_height = 0.0f;
        std::array<GridGeom, 6> geoms;
    };

    // Worker-safe (const, pure math): no GL, no Bullet, no body-state write.
    RootGeoms BuildRootGeoms() const {
        // One extra subdivision level per radius doubling. Ceiling 12: the
        // skirt has a precision issue at depth >= 13.
        int md = 14 + (int)llround(std::log2((double)radius / 600.0e3));
        if (md < 8) { md = 8; }
        if (md > 12) { md = 12; }

        // Highest relief above sea level: sizes the palette ramp + the shells.
        float maxh;
        if (surface.bands) {
            maxh = 0.0f;   // gas giant: smooth sphere
        } else if (surface.heightmap) {
            // Authored map: take the true peak, not a point sample (sampling
            // saturates the palette).
            const float map_hi = surface.heightmap->maxMetres();
            const float detail = std::max(0.0f, surface.detail_amplitude);
            maxh = std::max(1.0f, (map_hi + detail - surface.sea_level) * 1.05f);
        } else {
            const TerrainParams tp0 = params();
            const int N = 2048;
            const float golden = 2.39996322972865332f;   // golden angle
            float hi = -FLT_MAX;
            for (int i = 0; i < N; i++) {
                const float y = 1.0f - 2.0f * (i + 0.5f) / (float)N;
                const float rr = std::sqrt(std::max(0.0f, 1.0f - y * y));
                const glm::vec3 d(rr * std::cos(i * golden), y,
                                  rr * std::sin(i * golden));
                hi = std::max(hi, terrainHeight(d, tp0) - radius);
            }
            maxh = std::max(1.0f, (hi - surface.sea_level) * 1.05f);
        }

        // Bake with the COMPUTED max_height (the palette ramp reads it);
        // surface.max_height keeps its default until AttachRoot applies it.
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

    // Atmosphere rim shell on demand, just above the highest terrain so no
    // peak pokes through.
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

    // Cloud deck shell on demand. Coverage is baked on the worker; the deck draws
    // a solid placeholder until the continuation uploads the real grid.
    void BuildClouds(Shader *cloudshader, int res, JobRunner &jobs) {
        if(clouds != nullptr || !surface.clouds.enabled) return;
        float shell_radius = radius + surface.max_height
                             + surface.clouds.height;
        if(shell_radius <= radius) shell_radius = radius * 1.02f;
        Mesh *m = create_atmosphere_mesh(shell_radius, res);
        // Solid placeholder (coverage 1).
        const unsigned char solid = 255;
        clouds = new Shell;
        // The Shell owns the texture; the bake re-uploads into it.
        clouds->mesh = m;
        clouds->shader = cloudshader;
        clouds->texture = make_coverage_texture(1, 1, &solid, true);
        cloud_radius = shell_radius;

        // The bake layout MUST match the deck shader's UV (cloudShader.vs)
        // and equirect.h: u = lon/2pi (lon 0 at the LEFT edge), v north to
        // south, pixel centres.
        const int W = 2048, H = 1024;
        // Snapshots for the worker (job.h: no game state, GL or imgui).
        const glm::mat3 rot = surface.seed_rot;
        const CloudParams cp = surface.clouds;
        Texture *tex = clouds->texture;
        const std::string label = std::string("Clouds (") + name + ")";
        jobs.post(label, [tex, rot, cp, W, H]() -> std::function<void()> {
            std::vector<unsigned char> px((size_t)W * H);
            for(int py = 0; py < H; py++) {
                const float lat = (0.5f - (py + 0.5f) / (float)H)
                                  * (float)std::numbers::pi;
                for(int pxi = 0; pxi < W; pxi++) {
                    const float lon = ((pxi + 0.5f) / (float)W)
                                      * 2.0f * (float)std::numbers::pi;
                    const glm::vec3 dir = equirectDir(lon, lat);
                    px[(size_t)py * W + pxi] =
                        (unsigned char)(cloudCover(dir, rot, cp) * 255.0f
                                        + 0.5f);
                }
            }
            // Main-thread continuation: upload into the texture the deck
            // already draws (shared_ptr: std::function needs copyable captures).
            std::shared_ptr<std::vector<unsigned char> > ppx =
                std::make_shared<std::vector<unsigned char> >(std::move(px));
            return [tex, W, H, ppx]() {
                upload_coverage_r8(tex, W, H, ppx->data());
            };
        });
    }

    // Ocean surface shell on demand: a UV sphere at sea level, land poking
    // through via the depth test. Only OceanMode::Mesh gets one.
    void BuildOcean(Shader *oceanshader) {
        if(ocean != nullptr || !surface.has_sea
           || surface.ocean_mode != OceanMode::Mesh) return;
        // Offset above sea level: coplanar surfaces z-fight at the coastline.
        float shell_radius = radius + surface.sea_level + 0.1f;
        Mesh *m = create_atmosphere_mesh(shell_radius, 128);
        ocean = new Shell;
        ocean->mesh = m;
        ocean->shader = oceanshader;
        ocean->texture = nullptr;
        ocean_radius = shell_radius;
    }

    // Ring annuli on demand, one mesh per band in surface.rings.
    void BuildRings(Shader *ringshader) {
        if(ring_shader != nullptr || surface.rings.empty()) return;
        ring_shader = ringshader;
        for(const RingParams &rp : surface.rings) {
            ring_meshes.push_back(create_ring_mesh(rp.inner, rp.outer, 128));
        }
    }

    // Main thread: the full heavy phase.
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

        // Depth test so land occludes; no depth write.
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

        // Inside the shell: cull FRONT so the inner surface reads as the sky dome.
        const glm::dvec3 center = glm::dvec3(transform[3]);
        const bool inside = glm::length(camera->GetPos() - center) < (double)atm_radius;

        const glm::dmat4 &View = camera->GetView();
        // double, then truncate; Normal/cameraPos stay in world coordinates.
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
        // Light direction (sun -> planet), the same value the terrain uses.
        atmosphere->shader->setUniform_vec3(6,
            glm::vec3(SunlightDir(this, sun, renderFrame)));
        atmosphere->shader->setUniform_vec1(7, inside ? 1.0f : 0.0f);
        atmosphere->shader->setUniform_vec3(8, glm::vec3(center));

        // Blend over what is behind; depth test on, depth write off.
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

        // Same inside/outside flip as the atmosphere: cull FRONT under the deck.
        const glm::dvec3 center = glm::dvec3(transform[3]);
        const bool inside = glm::length(camera->GetPos() - center) < (double)cloud_radius;

        const glm::dmat4 &View = camera->GetView();
        // double, then truncate.
        glm::dmat4 ModelView = View * glm::translate(-camera->GetRenderOrigin()) * transform;
        glm::mat4 ModelViewFloat = ModelView;
        const glm::mat4 &Projection = camera->GetProjection();

        clouds->shader->Bind();
        clouds->shader->setUniform_mat4(0, Projection * ModelViewFloat);
        clouds->shader->setUniform_mat4(1, glm::mat4(transform));
        clouds->shader->setUniform_vec3(2, glm::vec3(camera->GetPos()));
        clouds->shader->setUniform_vec3(3, surface.clouds.color);
        // Light direction (sun -> planet), same value the terrain uses.
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

        // Depth test on, depth write off (the cloud deck sits under the rim).
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

    // Ring annuli over the opaque terrain (the depth buffer hides the far arc).
    // Cull the far face: with culling off both coincident faces blend, rendering
    // the band too dark.
    void DrawRings(const Camera *camera, TerrainBody *sun, Frame *renderFrame) {
        if(ring_meshes.empty()) return;

        // Ring plane: through the body centre, normal = the body's pole
        // (local +Y in world).
        const glm::dvec3 center = glm::dvec3(transform[3]);
        const glm::dvec3 pole = glm::dvec3(transform[1]);
        const bool below =
            glm::dot(camera->GetPos() - center, pole) < 0.0;

        const glm::dmat4 &View = camera->GetView();
        // double, then truncate.
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

    // Light direction (sun -> object) in renderFrame's axes; object_root is in
    // universe (root) coords. Uses the object's actual position so it stays
    // defined when the SOI body IS the star (normalize(0) = NaN).
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

        // Body-constant uniforms upload once per pass; the MVP is per patch.
        shader->Bind();
        shader->setUniform_mat4(1, glm::mat4(transform));
        shader->setUniform_vec3(2, sunlightVec);
        shader->setUniform_vec4(3, glm::vec4(0.8, 0.8, 0.8, 1.0));

        // Detail texture: shared by all bodies (the get_texture registry).
        static Texture *detail_tex = get_texture("res/textures/terrain_detail.png");
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, detail_tex->id);

        // Skirt pass fills the LOD cracks between patches of different depth
        // (and the limb).
        for(auto&& patch : patches) {
            patch->Draw(camera, false);
        }
        for(auto&& patch : patches) {
            patch->Draw(camera, true);
        }
    }

    // The camera in this body's FIXED axes (spin undone).
    glm::dvec3 CameraInBodyFrame(const Camera *camera) const {
        return cameraInBodyFrame(transform, camera->GetPos());
    }

    void Update(const Camera* camera, int max_patch_px, JobRunner &jobs) {
        if(!ready) return;
        const glm::dvec3 cam_bf = CameraInBodyFrame(camera);
        // The screen scale depends only on the camera: once, not per patch.
        const double px_per_rad = lodPxPerRad(camera->viewport_h, camera->fov);
        for(auto&& patch : patches) {
            patch->Update(cam_bf, px_per_rad, max_patch_px, jobs);
        }
    }

    // LOD telemetry (--terrain-log): the live tree relative to the camera.
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

// A world building (space pad): static, shared by every ship standing on the same
// site, drawn like terrain. Owned by its body (TerrainBody::pads); render assets
// are registry-owned, so ~TerrainBody only frees the rigid Body.
class StaticBuilding {
public:
    TerrainBody *parent = nullptr;
    TerrainBody *sun = nullptr; // the star (light source)
    Body *body = nullptr;

    std::string id;           // instance id (unique on the body)
    std::string display_name; // for the VAB launch-site picker
    std::vector<std::string> flags;
    glm::dvec3 dir = glm::dvec3(0.0, 1.0, 0.0); // unit up (body rotating frame)
    double height_m = 5.0;    // origin above terrain
    double heading_deg = 0.0; // spin about local up

    bool hasFlag(const std::string &f) const {
        for(size_t i = 0; i < flags.size(); i++) {
            if(flags[i] == f) { return true; }
        }
        return false;
    }
    // Building origin in the body's rotating frame.
    glm::dvec3 origin() const;

    void Draw(const Camera* camera, const TerrainBody *current, Frame *renderFrame);
};
