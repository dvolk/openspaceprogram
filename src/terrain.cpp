// terrain.cpp -- GeoPatch + TerrainBody method implementations (see terrain.h).
#include <numbers>
#include "terrain.h"

#include <array>
#include <cassert>
#include <cstdio>
#include <memory>
#include <vector>

// GeoPatch holds a btRigidBody* and deletes it in ~GeoPatch: the complete
// type is needed here.
#include "btcommon.h"
#include "physics.h"

#include "vehicle.h"   // Vehicle (Part::owner is a Vehicle*)
#include "part.h"      // Part (complete type: ~StaticBuilding deletes parked suits)

// mesh + cloud texture are the body's; the shader is shared.
static void free_shell(TerrainBody::Shell *s) {
    if(s == nullptr) { return; }
    delete s->mesh;
    delete s->texture;
    delete s;
}

TerrainBody::~TerrainBody() {
    /* No fleet here: a body never owns a Vehicle, so the fleet must already be
       gone before the first body dies (Game::unloadGame). A Vehicle references
       its body's frame, so getting that order wrong is a use-after-free in
       ~Vehicle rather than a leak. */
    // Pads: registries own the render assets, so only the rigid Body goes.
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
    // The shared ring shader is registry-owned, not freed here.
    for(Mesh *m : ring_meshes) { delete m; }
    ring_meshes.clear();
    delete frame;
    delete rot_frame;
}

StaticBuilding::~StaticBuilding() {
    // The cast standing here (crew.h, part.h): the suit owns its identity and
    // whatever its pockets hold, so ~Part frees the whole person.
    for(Part *p : crew) { delete p; }
    crew.clear();
}

glm::dvec3 StaticBuilding::origin() const {
    // Body centre is the origin: dir * (terrain height + origin offset).
    const double h = (double)parent->GetTerrainHeight(glm::vec3(dir));
    return dir * (h + height_m);
}

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
    // px_width doubles as the queue priority (job.h::post), so a subdivision
    // always outranks the one-shot jobs that post at 0.0.
    assert(px_width > 0.0 && "a subdivision request is past its LOD budget");
    subdivide_in_flight = true;
    TerrainBody *body = this->body;
    Shader *shader = body->shader;
    // Value snapshot: the worker never reads the main-thread-owned body.
    const TerrainParams tp = body->params();
    const glm::vec3 v0 = this->v0, v1 = this->v1, v2 = this->v2, v3 = this->v3;
    const int child_depth = depth + 1;
    GeoPatch *parent = this;

    jobs.post("Terrain", [body, shader, tp, v0, v1, v2, v3, child_depth, parent]()
              -> std::function<void()> {
        // Worker thread: pure math (terragen.h). No game state, GL or imgui.
        // The shared_ptr lets the result outlive this body.
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
            /* Drop the built grids if the parent is gone (a grandparent's
               collapse freed the subtree) or no longer wants children.
               patchAlive is a POINTER check: every continuation lands inside one
               poll(), all before the frame's Update posts anything new, and a
               parent's continuation always precedes its child's -- so a
               reallocated address cannot slip through. Break that poll()/Update
               ordering and this needs a per-patch id. */
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
    // Cached here: the height sample is a full noise evaluation, not per-frame.
    centroid_height = (double)body->GetTerrainHeight(centroid);
    // Mean of the four edge chords, NOT one edge: midpoint subdivision makes the
    // quads unequal-edged, and a single edge biased the LOD threshold between
    // same-depth siblings.
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

/* Conservative bounding-sphere frustum test for one patch (#120): sphere at the
   surface centroid, radius 2x the mean edge chord w -- enough for the corner
   bulge (wider on the six root faces), the skirt flare and the relief across the
   patch. The relief term is the one nothing PROVES: the internal-node test that
   lets one rejection drop a whole subtree needs relief <= ~0.65w, which grid
   band-limiting (terragen.h) supplies for the shipped bodies. Large
   surface.amplitude on a small radius could break it, failing as silent popping
   at the frustum edge; a per-patch min/max height at build time would make it
   exact. Side + near planes only (reverse-Z infinite far). True = the patch
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
        // Per-patch MVP: the mesh is baked relative to `anchor` and composed in
        // DOUBLE, so the float32 uniform and vertex data hold patch-scale numbers.
        const glm::dmat4 ModelView = camera->GetView()
            * glm::translate(-camera->GetRenderOrigin())
            * body->transform * glm::translate(anchor);
        shader->setUniform_mat4(0, camera->GetProjection()
                                   * glm::mat4(ModelView));
        shader->setUniform_vec3(4, glm::vec3(anchor));
        if(skirt_pass == false) {
            mesh->Draw();
        } else {
            // Depth-tests against the terrain, so only the cracks/limb show.
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
    // Distance to this patch's OWN surface point.
    const glm::dvec3 centroid_pos = centroid_height * (glm::dvec3)centroid;
    const double dist = glm::length(cam_bf - centroid_pos);

    // The budget is in REAL px so it follows FOV, zoom and window size (see
    // lodPxPerRad for why the aspect cancels out).
    const double px_width = lodPxWidth(width_m, dist, px_per_rad);

    if(depth < body->max_depth and px_width > (double)max_patch_px and
       kids[0] == NULL and !subdivide_in_flight) {
        /* Ranking the queue by px_width drills the camera's own column. Posting
           order spreads the single worker over the widest frontier patches
           (mid-depth, far away) and leaves the column under the camera shallow
           -- and max_depth is where collision meshes get built, so that means no
           ground under the ship. A child cannot post until its parent's
           continuation attaches it, so the path to max_depth is ~max_depth jobs,
           each outranking the frontier. */
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
        // A job may still be in flight: its continuation's flag check discards
        // the grids.
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
    // One test point per object, hard lit/shadow. The height function is
    // analytic, so this works where no collision leaves exist.
    assert(planet != nullptr && planet->frame != nullptr);
    if(sun == nullptr) { return 1.0f; }

    // The star is the light source, so its own terrain can't shadow the ship.
    if(planet == sun) { return 1.0f; }

    // Universe (root) axes; root_orient carries the chain into body-fixed below.
    const glm::dvec3 pos = posFrame->root_orient * posInFrame + posFrame->root_pos;
    const glm::dvec3 sunPos = sun->frame->root_pos;
    const glm::dvec3 dir = glm::normalize(sunPos - pos);

    const glm::dvec3 center = planet->frame->root_pos;
    const glm::dvec3 d = pos - center; // center -> point

    // Cheap reject: the ray misses the (radius + max relief) sphere.
    const double R = (double)planet->radius + (double)planet->surface.max_height;
    const double b = glm::dot(d, dir);
    const double c = glm::dot(d, d) - R * R;
    const double disc = b * b - c;
    if (disc <= 0.0) { return 1.0f; }

    // The forward part of the chord is where terrain could occlude the sun.
    const double s = std::sqrt(disc);
    double t0 = -b - s;
    const double t1 = -b + s;
    if (t0 < 0.0) { t0 = 0.0; }
    if (t0 >= t1) { return 1.0f; }

    // March the chord against the height function (in the planet's ROTATING
    // frame -- convert each sample there).
    const int steps = (int)glm::clamp((t1 - t0) / 100.0, 8.0, 128.0);
    const double dt = (t1 - t0) / steps;
    const glm::dmat3 toLocal = glm::transpose(planet->frame->getRotFrame()->root_orient);
    for (int i = 0; i < steps; i++) {
        const glm::dvec3 q = pos + (t0 + (i + 0.5) * dt) * dir;
        const glm::dvec3 ql = toLocal * (q - center);
        const double r = glm::length(ql);
        if (r < 1.0) { continue; } // degenerate sample at the center
        if (r < planet->GetTerrainHeight(glm::vec3(ql / r))) {
            // No sun reaches this point. partsShader.fs owns how dark a
            // shadowed hull reads; this only says whether the sun is there.
            return 0.0f;
        }
    }
    return 1.0f;
}

// Winding is outward = front, so back-face culling keeps the near hemisphere.
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
            // The color slot carries the UNWRAPPED sphere params for the cloud
            // deck (a continuous longitude across the seam). phi is atan2(z, x);
            // surface lon (equirect.h) is -phi (mod 2pi). The atmosphere shader
            // ignores color.
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

// +Y is the body's spin axis (tilt is folded into initial_orient, see load_system).
// Winding is CCW seen from +Y; DrawRings culls the far face so the underside shows.
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
        // split on the (a,d) diagonal: both triangles wind CCW from +Y
        idx.push_back(a); idx.push_back(c); idx.push_back(d);
        idx.push_back(a); idx.push_back(d); idx.push_back(b);
    }
    // copyData=false: no collision is ever built from a ring.
    mesh->FromData(verts.data(), (unsigned int)verts.size(),
                   idx.data(), (unsigned int)idx.size(), false);
    return mesh;
}
