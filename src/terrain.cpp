// terrain.cpp -- GeoPatch + TerrainBody method implementations and the
// terrain free helpers (see terrain.h for the class/data declarations).
#include "terrain.h"

#include <glm/gtc/noise.hpp>

#include <cstdio>

// GeoPatch holds a btRigidBody* and `delete`s it in ~GeoPatch, so the
// complete bullet type is needed here. Defined double-precision to match
// body.h / physics.cpp (same ABI as main.cpp). bullet must come BEFORE
// physics.h, which names btDefaultCollisionConfiguration in a member
// (only forward-declared there) and so needs the complete type.
#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>
#include "physics.h"


GeoPatch::~GeoPatch() {
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

void GeoPatch::Subdivide(void) {
    // TOOO need debug levels?
    // printf("%p subdiving (%d)!\n", this, depth);
    const glm::vec3 v01 = glm::normalize(v0+v1);
    const glm::vec3 v12 = glm::normalize(v1+v2);
    const glm::vec3 v23 = glm::normalize(v2+v3);
    const glm::vec3 v30 = glm::normalize(v3+v0);
    const glm::vec3 cn  = glm::normalize(centroid);

    const glm::vec3 vecs[4][4] = {
        {v0,  v01,   cn,  v30},
        {v01,  v1,  v12,   cn},
        {cn,  v12,   v2,  v23},
        {v30,  cn,  v23,   v3}
    };

    for (int quadrant = 0; quadrant < 4; quadrant++) {
        kids[quadrant]
            = new GeoPatch(body,
                           model->shader,
                           depth + 1,
                           vecs[quadrant][0],
                           vecs[quadrant][1],
                           vecs[quadrant][2],
                           vecs[quadrant][3]);
    }
}

GeoPatch::GeoPatch(TerrainBody *body, Shader *shader, int depth, glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 v3) {
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
    // Leaf patches (subdivision stops at depth == max_depth) get the
    // collision mesh; the old `>` was never true since depth never
    // exceeds max_depth, so terrain collision was silently never added.
    bool has_collision = depth >= max_depth;
    Mesh *grid_mesh = body->create_grid_mesh(has_collision, depth > 1, v0, v1, v2, v3);
    model->FromData(grid_mesh, shader, NULL);
    if(has_collision == true) {
        collision = addTerrainCollision(grid_mesh);
        printf("added terrain collision with %p\n", (void*)this);
    } else {
        collision = NULL;
    }
}

void GeoPatch::Draw(const Camera* camera, const glm::dmat4& transform, const glm::vec3& sunlightVec, bool skirt_pass) {
    if(kids[0] == NULL) {
        // patch isn't subdivided
        glm::vec4 color = glm::vec4(0.8, 0.8, 0.8, 1.0);
        model->shader->Bind();

        const glm::dmat4 & View = camera->GetView();
        // make sure View * Model happens with double precision
        // (transform shifted into the render frame: the view is built there)
        glm::dmat4 ModelView = View * glm::translate(-camera->GetRenderOrigin()) * transform;
        glm::mat4 ModelViewFloat = ModelView;
        const glm::mat4 & Projection = camera->GetProjection();
        glm::mat4 MVP = Projection * ModelViewFloat;
        glm::mat4 ModelFloat = transform;

        model->shader->setUniform_mat4(0, MVP);
        model->shader->setUniform_mat4(1, ModelFloat);
        model->shader->setUniform_vec3(2, sunlightVec);
        model->shader->setUniform_vec4(3, color);

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
        kids[0]->Draw(camera, transform, sunlightVec, skirt_pass);
        kids[1]->Draw(camera, transform, sunlightVec, skirt_pass);
        kids[2]->Draw(camera, transform, sunlightVec, skirt_pass);
        kids[3]->Draw(camera, transform, sunlightVec, skirt_pass);
    }
}

void GeoPatch::Update(const Camera* camera, const glm::dmat4& transform, int max_patch_px) {
    const glm::dvec3 camera_pos = camera->GetPos() - (glm::dvec3)(transform[3]);
    // Distance to this patch's OWN surface point: the height is sampled
    // at the patch direction (an earlier version sampled it at the camera
    // direction, which mis-measured the distance on slopes).
    const glm::dvec3 centroid_pos = (double)body->GetTerrainHeight(centroid) * (glm::dvec3)centroid;
    const double dist = glm::length(camera_pos - centroid_pos);

    // Projected patch width in screen pixels (small-angle; exact in the
    // small-patch regime that matters here). A patch subdivides while it
    // projects wider than max_patch_px and collapses below half of that --
    // the hysteresis band keeps the LOD from flapping near the boundary.
    // The budget is in px (not metres or degrees) so it follows FOV,
    // zoom, and window size/resolution automatically.
    const double width_m = (double)body->radius * glm::length(v0 - v3);
    const double fov_h = 2.0 * std::atan(std::tan(camera->fov * 0.5) * (double)camera->aspect);
    const double px_width = (width_m / dist) * ((double)camera->viewport_h / fov_h);

    if(depth < max_depth and px_width > (double)max_patch_px) {
        if(kids[0] == NULL) {
            Subdivide();
        }
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
    }

    if(kids[0] != NULL) {
        kids[0]->Update(camera, transform, max_patch_px);
        kids[1]->Update(camera, transform, max_patch_px);
        kids[2]->Update(camera, transform, max_patch_px);
        kids[3]->Update(camera, transform, max_patch_px);
    }
}

// TODO need doc
glm::vec3 getSpherePoint(const glm::vec3& v0, const glm::vec3& v1,
                         const glm::vec3& v2, const glm::vec3& v3,
                         const float x, const float y)
{
    return glm::normalize(v0 +
                          x * (1.0f - y) * (v1 - v0) +
                          x * y * (v2 - v0) +
                          (1.0f - x) * y * (v3 - v0));
}

// TODO need doc
float noise3d(const glm::vec3& p, int octaves, float persistence) {
    float sum = 0;
    float strength = 1.0;
    float scale = 2.0;

    for(int i = 0; i < octaves; i++) {
        sum += strength * glm::simplex(p * scale);
        scale *= 2.0;
        strength *= persistence;
    }

    return sum;
}

#define NOISE_FUNC (((noise3d(sphere_p, 12, 0.60) * 2500)))

float TerrainBody::GetTerrainHeight(const glm::vec3& sphere_p) const {
    const Surface &s = surface;
    if(s.bands) {
        return radius;   // gas giant: smooth sphere
    }
    float noise = noise3d(sphere_p * s.frequency + s.seed_offset,
                          s.octaves, s.persistence) * s.amplitude;

    if(s.has_sea && noise < s.sea_level) {
        noise = s.sea_level;
    }

    return radius + ScaleHeightNoise(noise);
}

float TerrainBody::GetTerrainHeightUnscaled(const glm::vec3& sphere_p) const {
    const Surface &s = surface;
    float noise = noise3d(sphere_p * s.frequency + s.seed_offset,
                          s.octaves, s.persistence) * s.amplitude;

    if(s.has_sea && noise < s.sea_level) {
        noise = s.sea_level;
    }

    return radius + noise;
}

float TerrainBody::ScaleHeightNoise(float noise) const {
    constexpr float ref_height = 3000.0; // guess

    // rescale noise by altitude (sign-safe for fractional powers)
    float sign = noise < 0 ? -1.0f : 1.0f;
    float n = sign * noise;
    n *= pow(n / ref_height, surface.power);
    return sign * n;
}

glm::vec3 TerrainBody::SurfaceColor(const glm::vec3& p) const {
    const Surface &s = surface;
    if(s.bands) {
        // gas giant: smooth sphere, color by latitude band
        COLOUR cb = s.BandColor(p);
        glm::vec3 color = glm::vec3(cb.r, cb.g, cb.b);
        float brightness = (cb.r + cb.g + cb.b) / 6;
        // lower contrast, increase brightness
        color = float(0.5) * color + glm::vec3(brightness,
                                               brightness,
                                               brightness);
        return color;
    }

    // set the color based on unscaled noise for better gradient
    float height = GetTerrainHeightUnscaled(p);

    // add some color noise
    float color_noise = noise3d(p * radius + s.seed_offset, 1, 0.60) * 100;
    float h = height + ((color_noise / 2) - color_noise);

    COLOUR c;
    if(s.palette.empty()) {
        // no palette in the JSON: fall back to the type-based default
        c = (*colour_func)(h, radius - 1, radius + 3000);
    } else {
        float t = (h - (radius + s.sea_level)) / s.max_height;
        c = s.PaletteColor(t);
    }

    glm::vec3 color = glm::vec3(c.r, c.g, c.b);
    float brightness = (c.r + c.g + c.b) / 6;
    // lower contrast, increase brightness
    color = float(0.5) * color + glm::vec3(brightness,
                                           brightness,
                                           brightness);

    if(s.has_sea && height <= radius + s.sea_level) {
        color = s.sea_color;
    }
    return color;
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

COLOUR GetColourMoon(float v, float vmin, float vmax) {
    return { 0.5, 0.5, 0.5 };
}

COLOUR GetColourSun(float v, float vmin, float vmax) {
    return { 1.0, 1.0, 0.0 };
}

COLOUR GetColourEarth(float v, float vmin, float vmax)
{
    COLOUR c = {1.0,1.0,1.0}; // white
    float dv;

    if (v < vmin)
        v = vmin;
    if (v > vmax)
        v = vmax;
    dv = vmax - vmin;

    const int factor = 3;

    if (v < (vmin + 0.25 * dv)) {
        c.r = 0;
        c.g = factor * (v - vmin) / dv;
    } else if (v < (vmin + 0.5 * dv)) {
        c.r = 0;
        c.b = 1 + factor * (vmin + 0.25 * dv - v) / dv;
    } else if (v < (vmin + 0.75 * dv)) {
        c.r = factor * (v - vmin - 0.5 * dv) / dv;
        c.b = 0;
    } else {
        c.g = 1 + 2 * (vmin + 0.75 * dv - v) / dv;
        c.b = 0;
    }

    return(c);
}

Mesh *TerrainBody::create_grid_mesh(bool has_collision, bool has_skirt, glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4) {
    Mesh *grid_mesh = new Mesh;
    const int size = 25;
    // The terrain grid is size x size; with a skirt it's (size+2)^2, one
    // extra ring of "skirt" vertices around it to hide the cracks that
    // open between neighbouring patches at different subdivision depths
    // (their edge polylines sample the heightfield at different points).
    // Each skirt vertex sits one grid cell OUTSIDE the patch boundary,
    // dropped to the patch's lowest terrain radius nudged in by 5e-6
    // (above float precision at any body size, below every point on all
    // four edges). Normals/colors are copied from the adjacent edge
    // vertex so the skirt shades identically to the terrain seam.
    // Technique from Pioneer's GeoPatch; with backface culling on, the
    // skirt only rasterises at the limb, exactly where the cracks show.
    // Root patches (depth 1) get no skirt: at the ranges they're visible
    // the float view-transform noise in fragment depth exceeds the tiny
    // skirt depth margin (zipper artefacts), gaps can't open between the
    // six equal-depth roots, and a root-vs-child T-junction is masked by
    // the child's skirt flaring across the seam.
    const int off = has_skirt ? 1 : 0;
    const int edge = size + 2 * off;
    const float frac = 1.0f / (size - 1);
    const float skirt_scale = 0.999995f;

    // sized for the skirted grid (edge == size+2); the skirtless root
    // patches just use the first edge*edge of them
    PosNorColVertex vertices[(size + 2) * (size + 2)];
    unsigned int indices[6 * (size + 1) * (size + 1)];

    float min_height = surface.bands ? radius : HUGE_VALF;

    // inner grid at grid coords [off..off+size-1]^2
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            glm::vec3 sphere_p = getSpherePoint(p1, p2, p3, p4, i*frac, j*frac);

            // The vertex color (noise, palette / band, sea, contrast):
            // shared with the 2-D surface map (SurfaceColor) so the map
            // matches the rendered surface.
            const glm::vec3 color = SurfaceColor(sphere_p);

            if (surface.bands) {
                // gas giant: smooth sphere
                vertices[(j + off) + edge * (i + off)] = PosNorColVertex(sphere_p * radius,
                                                         sphere_p,
                                                         color);
                continue;
            }

            // set the color based on unscaled noise for better gradient
            float height = GetTerrainHeightUnscaled(sphere_p);

            // add back scaling
            height = radius + ScaleHeightNoise(height - radius);

            min_height = std::min(min_height, height);

            glm::vec3 p = sphere_p * height;

            vertices[(j + off) + edge * (i + off)] = PosNorColVertex(p,
                                                 sphere_p,
                                                 color);
        }
    }

    // normals: finite differences over the whole inner grid, edge ring
    // included. Stencils that reach past the patch sample the true
    // heightfield one cell outside (it's analytic and shared, so neighbour
    // patches compute matching seam normals and no line shows at the
    // border). The skirt copies the edge normals; its dropped-down
    // vertices never enter a stencil.
    auto terrain_pos = [&](float u, float v) {
        glm::vec3 d = getSpherePoint(p1, p2, p3, p4, u, v);
        if (surface.bands) {
            return d * radius;
        }
        float hgt = GetTerrainHeightUnscaled(d);
        hgt = radius + ScaleHeightNoise(hgt - radius);
        return d * hgt;
    };
    for (int i = off; i < off + size; i++) {
        for (int j = off; j < off + size; j++) {
            // x along j, y along i: same axes as the old interior-only
            // pass, the cross product sign matters for lighting
            glm::vec3 x1 = (j - 1 >= off) ? vertices[(j-1) + i*edge].pos
                                          : terrain_pos((i - off) * frac, (j - 1 - off) * frac);
            glm::vec3 x2 = (j + 1 < off + size) ? vertices[(j+1) + i*edge].pos
                                                : terrain_pos((i - off) * frac, (j + 1 - off) * frac);
            glm::vec3 y1 = (i - 1 >= off) ? vertices[j + (i-1)*edge].pos
                                          : terrain_pos((i - 1 - off) * frac, (j - off) * frac);
            glm::vec3 y2 = (i + 1 < off + size) ? vertices[j + (i+1)*edge].pos
                                                : terrain_pos((i + 1 - off) * frac, (j - off) * frac);
            glm::vec3 n = glm::normalize(glm::cross(x2-x1, y2-y1));
            vertices[j + edge * i].normal = -n;
        }
    }

    // skirt ring: flare one cell past the boundary, down to the patch's
    // lowest radius; copies normal/color from the adjacent edge vertex
    // (after the normal pass above, so it gets the final normals)
    if (has_skirt) {
        const float skirt_r = min_height * skirt_scale;
        auto skirt_vertex = [&](int i, int j, float u, float v, int si, int sj) {
            glm::vec3 sphere_p = getSpherePoint(p1, p2, p3, p4, u, v);
            const PosNorColVertex &src = vertices[sj + edge * si];
            vertices[j + edge * i] = PosNorColVertex(sphere_p * skirt_r,
                                                     src.normal,
                                                     src.color);
        };
        for (int j = off; j < off + size; j++) {
            skirt_vertex(off - 1, j, -frac, (j - off)*frac, off, j);
            skirt_vertex(off + size, j, 1.0f + frac, (j - off)*frac, off + size - 1, j);
        }
        for (int i = off; i < off + size; i++) {
            skirt_vertex(i, off - 1, (i - off)*frac, -frac, i, off);
            skirt_vertex(i, off + size, (i - off)*frac, 1.0f + frac, i, off + size - 1);
        }
        // corners: duplicate the neighbouring skirt vertex
        vertices[(off - 1) + edge * (off - 1)] = vertices[off + edge * (off - 1)];
        vertices[(off + size) + edge * (off - 1)] = vertices[(off + size - 1) + edge * (off - 1)];
        vertices[(off - 1) + edge * (off + size)] = vertices[off + edge * (off + size)];
        vertices[(off + size) + edge * (off + size)] = vertices[(off + size - 1) + edge * (off + size)];
    }

    int i = 0;
    // inner terrain quads first, then the skirt-ring quads: DrawSkirt()
    // renders only the tail, after the terrain has written depth
    for (int y = off; y < off + size - 1; y++) {
        for (int x = off; x < off + size - 1; x++) {
            indices[i++] = (y + 1) * edge + x + 1;
            indices[i++] = y * edge + x + 1;
            indices[i++] = y * edge + x;

            indices[i++] = (y + 1) * edge + x;
            indices[i++] = (y + 1) * edge + x + 1;
            indices[i++] = y * edge + x;
        }
    }
    const int num_inner = has_skirt ? i : 0;
    if (has_skirt) {
        for (int y = 0; y < edge - 1; y++) {
            for (int x = 0; x < edge - 1; x++) {
                if (x >= off && x < off + size - 1 && y >= off && y < off + size - 1) {
                    continue;
                }
                indices[i++] = (y + 1) * edge + x + 1;
                indices[i++] = y * edge + x + 1;
                indices[i++] = y * edge + x;

                indices[i++] = (y + 1) * edge + x;
                indices[i++] = (y + 1) * edge + x + 1;
                indices[i++] = y * edge + x;
            }
        }
    }

    grid_mesh->FromData(vertices, edge * edge, indices, i, has_collision, num_inner);

    return grid_mesh;
}

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
