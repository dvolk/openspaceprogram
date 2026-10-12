// terrain.cpp -- GeoPatch + TerrainBody method implementations (see terrain.h).
#include <numbers>
#include "terrain.h"

#include <array>
#include <cassert>
#include <cstdio>
#include <memory>
#include <vector>

// GeoPatch holds a btRigidBody* and deletes it in ~GeoPatch: the complete
// type is needed here (btcommon.h, matching physics.cpp's precision).
#include "btcommon.h"
#include "physics.h"

#include "vehicle.h"   // Vehicle (complete type: ~TerrainBody deletes ships)

// Free a demand-built shell (mesh + cloud texture are the body's; the shader
// is shared).
static void free_shell(TerrainBody::Shell *s) {
    if(s == nullptr) { return; }
    delete s->mesh;
    delete s->texture;
    delete s;
}

TerrainBody::~TerrainBody() {
    // Ships first (they reference my frame; ~Vehicle unregisters from the
    // still-live Bullet world).
    for(auto *s : ships) { delete s; }
    ships.clear();
    // Pads next: render assets are shared (registries own them), so only
    // the rigid Body is freed -- unregister it from the Bullet world first.
    for(auto *p : pads) {
        RemoveBody(p->body);
        delete p->body;
        delete p;
    }
    pads.clear();
    for(int i = 0; i < 6; i++) { delete patches[i]; }
    free_shell(atmosphere);
    free_shell(clouds);
    free_shell(ocean);
    // Ring meshes (the shared ring shader is registry-owned, not freed here).
    for(Mesh *m : ring_meshes) { delete m; }
    ring_meshes.clear();
    delete frame;
    delete rot_frame;
}

glm::dvec3 StaticBuilding::origin() const {
    // Body centre is the origin: dir * (terrain height + origin offset).
    const double h = (double)parent->GetTerrainHeight(glm::vec3(dir));
    return dir * (h + height_m);
}

/* A world building (terrain.h): drawn like terrain, culled off-body. */
void StaticBuilding::Draw(const Camera *camera, const TerrainBody *current,
                          Frame *renderFrame) {
    if(current == parent) {
        const Frame *posFrame = parent->frame->getRotFrame();
        const float shadow = ComputeTerrainShadow(parent, posFrame,
                                                  GetPosition(body), sun);
        // Stays defined if the pad's body were ever the star.
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
        removeTerrainCollision(collision);
        delete collision;
    }
    delete mesh;   // owned (procedural grid); the shader is shared
}

void GeoPatch::requestSubdivide(JobRunner &jobs, double px_width) {
    // px_width is also the queue priority (job.h::post). Update only gets
    // here when px_width > max_patch_px, and max_patch_px >= kTerrainPxMin
    // (the --terrain-px floor), so a subdivision always outranks the one-shot
    // jobs that post at the default 0.0 -- surface map bakes, porkchop sweeps.
    assert(px_width > 0.0 && "a subdivision request is past its LOD budget");
    subdivide_in_flight = true;
    TerrainBody *body = this->body;
    Shader *shader = body->shader;
    // Value snapshot of the terrain math (the worker never reads the
    // main-thread-owned body).
    const TerrainParams tp = body->params();
    const glm::vec3 v0 = this->v0, v1 = this->v1, v2 = this->v2, v3 = this->v3;
    const int child_depth = depth + 1;
    GeoPatch *parent = this;

    jobs.post("Terrain", [body, shader, tp, v0, v1, v2, v3, child_depth, parent]()
              -> std::function<void()> {
        // Worker thread: pure math (terragen.h). No game state, GL or
        // imgui here. The shared_ptr lets the result outlive this body
        // (std::function needs copyable captures).
        glm::vec3 quad[4][4];
        subdivideCorners(v0, v1, v2, v3, quad);
        std::shared_ptr<std::array<GridGeom, 4> > geoms =
            std::make_shared<std::array<GridGeom, 4> >();
        for(int q = 0; q < 4; q++) {
            // has_skirt: children are always depth >= 2.
            geoms->at(q) = buildGridGeom(tp, true, child_depth, quad[q][0],
                                         quad[q][1], quad[q][2], quad[q][3]);
        }
        // Main-thread continuation: attach the children (GL + collision)
        // or discard the grids.
        return [body, shader, child_depth, geoms, parent]() {
            // Drop the built grids if the parent is gone (a grandparent's
            // collapse freed the subtree) or no longer wants children.
            // patchAlive is a POINTER check: every continuation lands inside
            // one poll(), all before the frame's Update posts anything new, so
            // a reallocated address cannot come back with its flag already
            // set. A parent's continuation always precedes its child's -- the
            // child patch does not exist until that continuation attaches it,
            // so the queue's priority order cannot invert that either. Break
            // the poll()/Update ordering and this needs a per-patch id.
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
    }, px_width);
}

GeoPatch::GeoPatch(TerrainBody *body, Shader *shader, int depth, glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, const GridGeom &geom) {
    this->shader = shader;   // shared (the body's terrain shader)
    mesh = nullptr;          // set below, once the grid is built
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
    this->anchor = geom.anchor;
    this->centroid = glm::normalize(v0 + v1 + v2 + v3);
    // Cache the per-patch constants (the height sample is a full noise
    // evaluation -- not per-frame).
    centroid_height = (double)body->GetTerrainHeight(centroid);
    // Mean of the four edge chords, NOT one edge: midpoint subdivision
    // makes the quads unequal-edged, and a single edge biased the LOD
    // threshold between same-depth siblings.
    width_m = (double)body->radius * patchWidthUnit(v0, v1, v2, v3);
    // Leaf patches (depth == max_depth) get the collision mesh.
    bool has_collision = depth >= body->max_depth;
    // GridGeom (pure math) -> Mesh (GL upload): main thread only.
    std::vector<PosNorColVertex> pv(geom.verts.size());
    for(size_t i = 0; i < pv.size(); i++) {
        pv[i] = PosNorColVertex(geom.verts[i].pos, geom.verts[i].normal,
                                geom.verts[i].color);
    }
    Mesh *grid_mesh = new Mesh;
    grid_mesh->FromData(pv.data(), (unsigned int)pv.size(),
                        geom.indices.data(), (unsigned int)geom.indices.size(),
                        has_collision, geom.num_inner);
    mesh = grid_mesh;   // owned by this patch
    if(has_collision == true) {
        collision = addTerrainCollision(grid_mesh, anchor);
    } else {
        collision = NULL;
    }
    body->alive.insert(this);
}

/* Conservative bounding-sphere frustum test for one patch (#120). The sphere
   is centred on the patch's surface centroid (its anchor direction, at the
   full-detail height) with a radius of 2x the patch width w (the mean edge
   chord). What that has to cover:
     - the corner bulge: ~0.71w for a subdivided patch, but ~0.82w for the six
       ROOT faces, whose corners open wider than the mean edge (spherical
       excess) -- so the leaf margin is already tighter than it looks;
     - the skirt flare: one grid cell, ~0.021w;
     - the relief across the patch.
   That last term is the one nothing PROVES. Covering it needs relief <= 1.18w
   for a leaf, and a parent sphere must also contain its children's spheres,
   which tightens it to about 0.65w for the internal-node test that lets one
   rejection drop a whole subtree. The grid's band-limiting (terragen.h:
   wavelengths below 2 grid cells are filtered) gives that margin for the
   shipped bodies -- Kerbin's deepest leaf has ~45-90 m of relief against a
   436 m budget -- but a body with large surface.amplitude and small radius
   could break it, and the failure mode is silent popping at the frustum edge.
   Storing a per-patch min/max height at build time would make it exact.
   View-space test against the side planes and the near plane; there is no far
   plane (reverse-Z infinite-far projection). Returns true when the patch
   provably cannot contribute a pixel. */
static bool patchCulled(const GeoPatch *p, const TerrainBody *body,
                        const Camera *camera) {
    const glm::dvec3 wc = body->transform
        * glm::dvec4(p->centroid_height * (glm::dvec3)p->centroid, 1.0);
    const glm::dvec4 vs = camera->GetView()
                        * glm::dvec4(wc - camera->GetRenderOrigin(), 1.0);
    const double zv = -vs.z;                       // depth in front of camera
    const double rad = p->width_m * 2.0;
    const double mNear = zv - (double)camera->zNear + rad;
    const double mX = zv * camera->frustumSinX - std::fabs(vs.x) * camera->frustumCosX + rad;
    const double mY = zv * camera->frustumSinY - std::fabs(vs.y) * camera->frustumCosY + rad;
    if(mNear < 0) { return true; }
    if(mX < 0) { return true; }
    if(mY < 0) { return true; }
    return false;
}

void GeoPatch::Draw(const Camera* camera, bool skirt_pass) {
    if(patchCulled(this, body, camera)) {
        return;   // the whole subtree is off-frustum
    }
    if(kids[0] == NULL) {
        // Per-patch MVP: the mesh is baked relative to `anchor`, composed
        // into the modelview in DOUBLE, so the float32 uniform and the
        // vertex data only ever hold patch-scale numbers.
        const glm::dmat4 ModelView = camera->GetView()
            * glm::translate(-camera->GetRenderOrigin())
            * body->transform * glm::translate(anchor);
        shader->setUniform_mat4(0, camera->GetProjection()
                                   * glm::mat4(ModelView));
        shader->setUniform_vec3(4, glm::vec3(anchor));
        if(skirt_pass == false) {
            mesh->Draw();
        } else {
            // Skirt draws after the terrain and depth-tests against it, so
            // it shows only in the cracks/limb.
            mesh->DrawSkirt();
        }
    }
    else {
        kids[0]->Draw(camera, skirt_pass);
        kids[1]->Draw(camera, skirt_pass);
        kids[2]->Draw(camera, skirt_pass);
        kids[3]->Draw(camera, skirt_pass);
    }
}

void GeoPatch::Update(const glm::dvec3 &cam_bf, double px_per_rad,
                      int max_patch_px, JobRunner &jobs) {
    // Distance to this patch's OWN surface point (centroid_height is
    // cached in the ctor). cam_bf is already in body-fixed axes.
    const glm::dvec3 centroid_pos = centroid_height * (glm::dvec3)centroid;
    const double dist = glm::length(cam_bf - centroid_pos);

    // Projected screen extent [px]. Subdivides while wider than
    // max_patch_px, collapses below half (hysteresis). The budget is in
    // REAL px so it follows FOV, zoom, and window size (see lodPxPerRad
    // for why the aspect cancels out).
    const double px_width = lodPxWidth(width_m, dist, px_per_rad);

    // Subdivision is async: keep drawing this (coarser) patch until the
    // continuation attaches the children. subdivide_in_flight suppresses
    // re-posting.
    if(depth < body->max_depth and px_width > (double)max_patch_px and
       kids[0] == NULL and !subdivide_in_flight) {
        /* px_width is the queue priority, and that is the whole point of
           passing it: the worker has one thread, so a patch that is 4000 px
           wide against a 128 px budget must not queue behind the thousands of
           frontier patches that are barely over it. Update walks the tree in
           posting order, which spreads the worker over the widest part of the
           tree (mid-depth, far away) and leaves the camera's own column
           shallow -- and max_depth is where the collision meshes are built,
           so "shallow under the camera" means no ground under the ship.
           Ranking by px_width drills that column instead: a child cannot post
           until its parent's continuation attaches it, so the path to
           max_depth is ~max_depth jobs, and each of them outranks the
           frontier. */
        requestSubdivide(jobs, px_width);
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
        // A job may still be in flight: the continuation's flag check
        // discards its grids. (If the worker threw, this also clears the
        // flag.)
        subdivide_in_flight = false;
    }

    if(kids[0] != NULL) {
        kids[0]->Update(cam_bf, px_per_rad, max_patch_px, jobs);
        kids[1]->Update(cam_bf, px_per_rad, max_patch_px, jobs);
        kids[2]->Update(cam_bf, px_per_rad, max_patch_px, jobs);
        kids[3]->Update(cam_bf, px_per_rad, max_patch_px, jobs);
    }
}

float ComputeTerrainShadow(TerrainBody *planet, const Frame *posFrame,
                           const glm::dvec3 &posInFrame, TerrainBody *sun) {
    // Approximate terrain shadow for one point: one test point per object,
    // hard lit/shadow. The height function is analytic (available
    // everywhere, not just where collision leaves exist).
    if(sun == nullptr) { return 1.0f; }

    // Ship in the sun's own SOI: the star is the light source, so its own
    // terrain can't shadow the ship.
    if(planet == sun) { return 1.0f; }

    // Work in universe (root) axes; root_orient carries the full chain into
    // the body-fixed frame below.
    const glm::dvec3 pos = posFrame->root_orient * posInFrame + posFrame->root_pos;
    const glm::dvec3 sunPos = sun->frame->root_pos;
    const glm::dvec3 dir = glm::normalize(sunPos - pos);

    const glm::dvec3 center = planet->frame->root_pos;
    const glm::dvec3 d = pos - center; // center -> point

    // Cheap reject: the ray misses the (radius + max relief) sphere?
    // Common case (high orbit) and costs one quadratic.
    const double R = (double)planet->radius + (double)planet->surface.max_height;
    const double b = glm::dot(d, dir);
    const double c = glm::dot(d, d) - R * R;
    const double disc = b * b - c;
    if (disc <= 0.0) { return 1.0f; }

    // Chord of the ray inside the sphere; the forward part is where
    // terrain could occlude the sun.
    const double s = std::sqrt(disc);
    double t0 = -b - s;
    const double t1 = -b + s;
    if (t0 < 0.0) { t0 = 0.0; }
    if (t0 >= t1) { return 1.0f; }

    // March the chord against the height function (star function in the
    // planet's ROTATING frame -- convert each sample there).
    const int steps = (int)glm::clamp((t1 - t0) / 100.0, 8.0, 128.0);
    const double dt = (t1 - t0) / steps;
    const glm::dmat3 toLocal = glm::transpose(planet->frame->getRotFrame()->root_orient);
    for (int i = 0; i < steps; i++) {
        const glm::dvec3 q = pos + (t0 + (i + 0.5) * dt) * dir;
        const glm::dvec3 ql = toLocal * (q - center);
        const double r = glm::length(ql);
        if (r < 1.0) { continue; } // degenerate sample at the center
        if (r < planet->GetTerrainHeight(glm::vec3(ql / r))) {
            // 0.15 matches partsShader's min_light so a shadowed part
            // reads as "night".
            return 0.15f;
        }
    }
    return 1.0f;
}

// (The pure terrain math lives in terragen.h.)

// Smooth UV sphere for the atmosphere rim + the cloud deck. Winding is
// outward = front so back-face culling keeps the near hemisphere. res =
// latitude = longitude rings.
Mesh *TerrainBody::create_atmosphere_mesh(float radius, int res) {
    Mesh *mesh = new Mesh;
    const int lat = res, lon = res;
    std::vector<PosNorColVertex> verts;
    verts.reserve((size_t(lat) + 1) * (size_t(lon) + 1));
    for(int i = 0; i <= lat; i++) {
        float theta = (float)i / lat * std::numbers::pi;              // 0..pi (pole->pole)
        for(int j = 0; j <= lon; j++) {
            float phi = (float)j / lon * 2.0f * std::numbers::pi;     // 0..2pi
            glm::vec3 dir = glm::vec3(
                std::sin(theta) * std::cos(phi),
                std::cos(theta),
                std::sin(theta) * std::sin(phi));
            // The color slot carries the UNWRAPPED sphere params for the
            // cloud deck (a continuous longitude across the seam). phi is
            // atan2(z, x) from +X toward +Z; surface lon (equirect.h) is
            // -phi (mod 2pi). The atmosphere shader ignores color.
            verts.push_back(PosNorColVertex(dir * radius, dir,
                glm::vec3(phi / (2.0f * (float)std::numbers::pi),
                          theta / (float)std::numbers::pi, 0.0f)));
        }
    }
    std::vector<unsigned int> idx;
    idx.reserve(size_t(lat) * size_t(lon) * 6);
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

// A flat annulus in the local XZ plane (normal +Y). +Y is the body's spin
// axis (tilt is folded into initial_orient, see load_system). Winding is
// CCW seen from +Y; DrawRings culls the far face so the underside shows too.
Mesh *TerrainBody::create_ring_mesh(double inner, double outer, int res) {
    Mesh *mesh = new Mesh;
    const int seg = res;
    const glm::vec3 n(0.0f, 1.0f, 0.0f);
    std::vector<PosNorColVertex> verts;
    verts.reserve((size_t(seg) + 1) * 2);
    for(int j = 0; j <= seg; j++) {
        float phi = (float)j / seg * 2.0f * (float)std::numbers::pi;
        float c = std::cos(phi), s = std::sin(phi);
        verts.push_back(PosNorColVertex(
            glm::vec3((float)(inner * c), 0.0f, (float)(inner * s)),
            n, glm::vec3(0.0f)));
        verts.push_back(PosNorColVertex(
            glm::vec3((float)(outer * c), 0.0f, (float)(outer * s)),
            n, glm::vec3(0.0f)));
    }
    std::vector<unsigned int> idx;
    idx.reserve(size_t(seg) * 6);
    for(int j = 0; j < seg; j++) {
        unsigned int a = j * 2;             // inner  @ j
        unsigned int b = j * 2 + 1;         // outer  @ j
        unsigned int c = (j + 1) * 2;       // inner  @ j+1
        unsigned int d = (j + 1) * 2 + 1;   // outer  @ j+1
        // split the (a,b,d,c) quad on the (a,d) diagonal (both triangles
        // wind CCW from +Y)
        idx.push_back(a); idx.push_back(c); idx.push_back(d);
        idx.push_back(a); idx.push_back(d); idx.push_back(b);
    }
    // copyData=false: no collision is ever built from a ring.
    mesh->FromData(verts.data(), (unsigned int)verts.size(),
                   idx.data(), (unsigned int)idx.size(), false);
    return mesh;
}
