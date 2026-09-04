// terrain.cpp -- GeoPatch + TerrainBody method implementations and the
// terrain free helpers (see terrain.h for the class/data declarations).
#include "terrain.h"

#include <array>
#include <cstdio>
#include <memory>
#include <vector>

// GeoPatch holds a btRigidBody* and `delete`s it in ~GeoPatch, so the
// complete bullet type is needed here. Defined double-precision to match
// body.h / physics.cpp (same ABI as main.cpp). bullet must come BEFORE
// physics.h, which names btDefaultCollisionConfiguration in a member
// (only forward-declared there) and so needs the complete type.
#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>
#include "physics.h"

#include "vehicle.h"   // Vehicle (complete type: ~TerrainBody deletes the
                       // ships this body owns)

TerrainBody::~TerrainBody() {
    // The ships in my SOI first (they reference my frame; the Vehicle dtor
    // also unregisters their bodies from the still-live Bullet world).
    for(auto *s : ships) { delete s; }
    ships.clear();
    // The pads next. Each owns its own model (like a ship part), so the
    // rigid Body is freed here: unregister it from the Bullet world first
    // (the world would otherwise hold a dangling btBody), then ~Body frees
    // its model + rigid body; the StaticBuilding struct is freed last.
    for(auto *p : pads) {
        RemoveBody(p->body);
        delete p->body;
        delete p;
    }
    pads.clear();
    for(int i = 0; i < 6; i++) { delete patches[i]; }
    delete atmosphere;
    delete frame;
    delete rot_frame;
}

/* A space pad (terrain.h): drawn like terrain, culled when the active
   ship is not on the pad's body; the light source is the star. */
void StaticBuilding::Draw(const Camera *camera, const TerrainBody *current,
                          Frame *renderFrame) {
    if(current == parent) {
        const Frame *posFrame = parent->frame->getRotFrame();
        const float shadow = ComputeTerrainShadow(parent, posFrame,
                                                  GetPosition(body), sun);
        // Light direction at the pad (sun -> pad); stays defined if the
        // pad's body were ever the star (SunlightDir would be a zero vector).
        const glm::dvec3 pad_root =
            posFrame->root_orient * GetPosition(body) + posFrame->root_pos;
        glm::vec3 sunlightVec =
            glm::vec3(TerrainBody::LightDirFrom(pad_root, sun, renderFrame));
        body->Draw(camera, sunlightVec, shadow);
    }
}

GeoPatch::~GeoPatch() {
    body->alive.erase(this);
    delete kids[0];
    delete kids[1];
    delete kids[2];
    delete kids[3];
    if(collision != NULL) {
        printf("removing terrain collision\n");
        removeTerrainCollision(collision);
        delete collision;
    }
    delete model;
}

// The four children's corner quads: the edge midpoints (v01, v12, v23,
// v30) + the shared center cn.
static void subdivideCorners(const glm::vec3 &v0, const glm::vec3 &v1,
                             const glm::vec3 &v2, const glm::vec3 &v3,
                             glm::vec3 quad[4][4]) {
    const glm::vec3 v01 = glm::normalize(v0+v1);
    const glm::vec3 v12 = glm::normalize(v1+v2);
    const glm::vec3 v23 = glm::normalize(v2+v3);
    const glm::vec3 v30 = glm::normalize(v3+v0);
    const glm::vec3 cn  = glm::normalize(v0+v1+v2+v3);

    quad[0][0] = v0;  quad[0][1] = v01; quad[0][2] = cn;  quad[0][3] = v30;
    quad[1][0] = v01; quad[1][1] = v1;  quad[1][2] = v12; quad[1][3] = cn;
    quad[2][0] = cn;  quad[2][1] = v12; quad[2][2] = v2;  quad[2][3] = v23;
    quad[3][0] = v30; quad[3][1] = cn;  quad[3][2] = v23; quad[3][3] = v3;
}

void GeoPatch::requestSubdivide(JobRunner &jobs) {
    subdivide_in_flight = true;
    TerrainBody *body = this->body;
    Shader *shader = body->shader;
    // Value snapshot of the terrain math (the worker never reads the
    // main-thread-owned body), like the porkchop grid snapshots its
    // state (see job.h).
    const TerrainParams tp = body->params();
    const glm::vec3 v0 = this->v0, v1 = this->v1, v2 = this->v2, v3 = this->v3;
    const int child_depth = depth + 1;
    GeoPatch *parent = this;

    jobs.post("Terrain", [body, shader, tp, v0, v1, v2, v3, child_depth, parent]()
              -> std::function<void()> {
        // Worker thread: pure math (terragen.h). No game state, GL or
        // imgui here. The result is handed to the main thread through
        // the returned continuation; the shared_ptr lets it outlive this
        // body (a C++11-safe handoff, like the surfmap's pixel buffer).
        glm::vec3 quad[4][4];
        subdivideCorners(v0, v1, v2, v3, quad);
        std::shared_ptr<std::array<GridGeom, 4> > geoms =
            std::make_shared<std::array<GridGeom, 4> >();
        for(int q = 0; q < 4; q++) {
            // has_skirt: children are always depth >= 2.
            geoms->at(q) = buildGridGeom(tp, true, child_depth, quad[q][0],
                                         quad[q][1], quad[q][2], quad[q][3]);
        }
        // Main-thread continuation (JobRunner::poll): attach the children
        // (GL upload + collision) -- or discard the grids.
        return [body, shader, child_depth, geoms, parent]() {
            // The parent may be gone (a grandparent's collapse freed the
            // subtree while the job was in flight) or no longer want
            // children (a zoom-out cleared the flag on the collapse path)
            // -- in either case drop the built grids.
            if(!body->patchAlive(parent) || !parent->subdivide_in_flight) {
                return;
            }
            glm::vec3 quad[4][4];
            subdivideCorners(parent->v0, parent->v1, parent->v2,
                             parent->v3, quad);
            for(int q = 0; q < 4; q++) {
                parent->kids[q] = new GeoPatch(body, shader, child_depth,
                                               quad[q][0], quad[q][1],
                                               quad[q][2], quad[q][3],
                                               geoms->at(q));
            }
            parent->subdivide_in_flight = false;
        };
    });
}

GeoPatch::GeoPatch(TerrainBody *body, Shader *shader, int depth, glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, const GridGeom &geom) {
    model = new Model;
    kids[0] = NULL;
    kids[1] = NULL;
    kids[2] = NULL;
    kids[3] = NULL;
    this->body = body;
    this->depth = depth;
    this->v0 = v0;
    this->v1 = v1;
    this->v2 = v2;
    this->v3 = v3;
    this->centroid = glm::normalize(v0 + v1 + v2 + v3);
    // Cache the per-patch constants Update() and Draw() use every frame
    // (the height sample is a full noise evaluation; doing it per patch
    // per frame in Update was pure waste).
    centroid_height = (double)body->GetTerrainHeight(centroid);
    width_m = (double)body->radius * (double)glm::length(v0 - v3);
    // Leaf patches (subdivision stops at depth == the body's max_depth)
    // get the collision mesh.
    bool has_collision = depth >= body->max_depth;
    // GridGeom (pure math, terragen.h) -> Mesh (GL upload): main thread
    // only, called from here (the startup path and the job continuation).
    std::vector<PosNorColVertex> pv(geom.verts.size());
    for(size_t i = 0; i < pv.size(); i++) {
        pv[i] = PosNorColVertex(geom.verts[i].pos, geom.verts[i].normal,
                                geom.verts[i].color);
    }
    Mesh *grid_mesh = new Mesh;
    grid_mesh->FromData(pv.data(), (unsigned int)pv.size(),
                        geom.indices.data(), (unsigned int)geom.indices.size(),
                        has_collision, geom.num_inner);
    model->FromData(grid_mesh, shader, NULL);
    if(has_collision == true) {
        collision = addTerrainCollision(grid_mesh);
        printf("added terrain collision with %p\n", (void*)this);
    } else {
        collision = NULL;
    }
    body->alive.insert(this);
}

void GeoPatch::Draw(const Camera* camera, bool skirt_pass) {
    if(kids[0] == NULL) {
        // patch isn't subdivided
        if(skirt_pass == false) {
            model->mesh->Draw();
        } else {
            // the stencil (set up in TerrainBody::Draw) only passes where
            // no terrain fragment was drawn, so the skirt shows in the
            // cracks/limb and can never z-fight the surface
            model->mesh->DrawSkirt();
        }
    }
    else {
        kids[0]->Draw(camera, skirt_pass);
        kids[1]->Draw(camera, skirt_pass);
        kids[2]->Draw(camera, skirt_pass);
        kids[3]->Draw(camera, skirt_pass);
    }
}

void GeoPatch::Update(const Camera* camera, const glm::dmat4& transform, int max_patch_px, JobRunner &jobs) {
    const glm::dvec3 camera_pos = camera->GetPos() - (glm::dvec3)(transform[3]);
    // Distance to this patch's OWN surface point (centroid_height is
    // cached in the ctor; an earlier version re-sampled the height here
    // every frame, and one before that sampled it at the camera
    // direction, which mis-measured the distance on slopes).
    const glm::dvec3 centroid_pos = centroid_height * (glm::dvec3)centroid;
    const double dist = glm::length(camera_pos - centroid_pos);

    // Projected patch width in screen pixels (small-angle; exact in the
    // small-patch regime that matters here). A patch subdivides while it
    // projects wider than max_patch_px and collapses below half of that --
    // the hysteresis band keeps the LOD from flapping near the boundary.
    // The budget is in px (not metres or degrees) so it follows FOV,
    // zoom, and window size/resolution automatically.
    const double fov_h = 2.0 * std::atan(std::tan(camera->fov * 0.5) * (double)camera->aspect);
    const double px_width = (width_m / dist) * ((double)camera->viewport_h / fov_h);

    // Subdivision is async: request it, keep drawing this (coarser) patch
    // until the continuation attaches the children. While a job is in
    // flight the flag suppresses re-posting; the continuation clears it.
    if(depth < body->max_depth and px_width > (double)max_patch_px and
       kids[0] == NULL and !subdivide_in_flight) {
        requestSubdivide(jobs);
    }
    else if(px_width < (double)max_patch_px * 0.5) {
        delete kids[0];
        delete kids[1];
        delete kids[2];
        delete kids[3];
        kids[0] = NULL;
        kids[1] = NULL;
        kids[2] = NULL;
        kids[3] = NULL;
        // A job may still be in flight (its grids are about to land for a
        // patch that no longer wants children): the flag check in the
        // continuation makes it discard them. (If the worker THREW, no
        // continuation ever runs and this is what also clears the flag.)
        subdivide_in_flight = false;
    }

    if(kids[0] != NULL) {
        kids[0]->Update(camera, transform, max_patch_px, jobs);
        kids[1]->Update(camera, transform, max_patch_px, jobs);
        kids[2]->Update(camera, transform, max_patch_px, jobs);
        kids[3]->Update(camera, transform, max_patch_px, jobs);
    }
}

float ComputeTerrainShadow(TerrainBody *planet, const Frame *posFrame,
                           const glm::dvec3 &posInFrame, TerrainBody *sun) {
    // Approximate terrain shadow for one point (a ship part / the space port):
    // cast a ray from the point toward the sun and test it against the
    // planet's terrain height function, which is analytic and therefore
    // available everywhere (not just where collision leaves exist).
    // Approximate by design: one test point per object, hard lit/shadow.

    if(sun == nullptr) { return 1.0f; }

    // The SOI body IS the star (ship in the sun's own SOI): the star is the
    // light source, so its own terrain can't shadow the ship. Without this the
    // "line to the sun" ray re-crosses the star's body and reads as shadow.
    if(planet == sun) { return 1.0f; }

    // Work in universe (root) axes: the ray to the sun and the planet
    // center are both absolute there, and root_orient carries the full
    // chain (orbital tilts of the ancestors, axial tilt + spin of the
    // body) for the conversion into the body-fixed frame below.
    const glm::dvec3 pos = posFrame->root_orient * posInFrame + posFrame->root_pos;
    const glm::dvec3 sunPos = sun->frame->root_pos;
    const glm::dvec3 dir = glm::normalize(sunPos - pos);

    const glm::dvec3 center = planet->frame->root_pos;
    const glm::dvec3 d = pos - center; // center -> point

    // Cheap reject: does the ray pass within (radius + max relief) of the
    // planet center?  If not, terrain cannot occlude. This is the common
    // case (high orbit, interplanetary space) and costs one quadratic.
    const double R = (double)planet->radius + (double)planet->surface.max_height;
    const double b = glm::dot(d, dir);
    const double c = glm::dot(d, d) - R * R;
    const double disc = b * b - c;
    if (disc <= 0.0) { return 1.0f; }

    // Chord of the ray inside the (radius + max relief) sphere; the forward
    // part of it is where terrain could occlude the sun.
    const double s = std::sqrt(disc);
    double t0 = -b - s;
    const double t1 = -b + s;
    if (t0 < 0.0) { t0 = 0.0; }
    if (t0 >= t1) { return 1.0f; }

    // March the chord against the actual height function. The terrain is a
    // star function in the planet's ROTATING frame (the frame its meshes
    // are built in), so convert each sample there.
    const int steps = (int)glm::clamp((t1 - t0) / 100.0, 8.0, 128.0);
    const double dt = (t1 - t0) / steps;
    const glm::dmat3 toLocal = glm::transpose(planet->frame->getRotFrame()->root_orient);
    for (int i = 0; i < steps; i++) {
        const glm::dvec3 q = pos + (t0 + (i + 0.5) * dt) * dir;
        const glm::dvec3 ql = toLocal * (q - center);
        const double r = glm::length(ql);
        if (r < 1.0) { continue; } // degenerate sample at the center
        if (r < planet->GetTerrainHeight(glm::vec3(ql / r))) {
            // Terrain occludes the line to the sun. 0.15 matches
            // partsShader's min_light so a shadowed part reads as "night".
            return 0.15f;
        }
    }
    return 1.0f;
}

// (The pure terrain math -- noise, height/color functions, the color
// palettes, and the grid builder that used to be create_grid_mesh -- now
// lives in terragen.h.)

// Smooth UV sphere for the atmosphere rim. No noise: it must be a clean
// shell just above the terrain. Winding is outward = front (CCW seen from
// outside) so back-face culling keeps the near hemisphere the camera sees.
Mesh *TerrainBody::create_atmosphere_mesh(float radius) {
    Mesh *mesh = new Mesh;
    const int lat = 128, lon = 128;
    std::vector<PosNorColVertex> verts;
    verts.reserve((lat + 1) * (lon + 1));
    for(int i = 0; i <= lat; i++) {
        float theta = (float)i / lat * M_PI;              // 0..pi (pole->pole)
        for(int j = 0; j <= lon; j++) {
            float phi = (float)j / lon * 2.0f * M_PI;     // 0..2pi
            glm::vec3 dir = glm::vec3(
                std::sin(theta) * std::cos(phi),
                std::cos(theta),
                std::sin(theta) * std::sin(phi));
            verts.push_back(PosNorColVertex(dir * radius, dir, glm::vec3(1,1,1)));
        }
    }
    std::vector<unsigned int> idx;
    idx.reserve(lat * lon * 6);
    for(int i = 0; i < lat; i++) {
        for(int j = 0; j < lon; j++) {
            unsigned int first  = i * (lon + 1) + j;
            unsigned int second = (i + 1) * (lon + 1) + j;
            idx.push_back(first);  idx.push_back(first + 1);  idx.push_back(second);
            idx.push_back(second); idx.push_back(first + 1);  idx.push_back(second + 1);
        }
    }
    mesh->FromData(verts.data(), (unsigned int)verts.size(),
                   idx.data(), (unsigned int)idx.size(), true);
    return mesh;
}
