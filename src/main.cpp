// Open Space Program

#include <stdio.h>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <ctime>
#include <cstdlib>
#include <sys/stat.h>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <map>
#include <set>

#include "SDL2/SDL.h"
#include "SDL_keycode.h"

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/gtc/noise.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/projection.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/polar_coordinates.hpp>

#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>

#include "display.h"
#include "mesh.h"
#include "shader.h"
#include "camera.h"
#include "model.h"
#include "body.h"
#include "physics.h"
#include "gldebug.h"
#include "frame.h"
#include "calendar.h"
#include "orbit.h"
#include "shipdef.h"
#include "fleet.h"
#include <nlohmann/json.hpp>
#include "billboard.h"
#include "texture.h"
#include "skybox.h"
#include "postfx.h"
#include "ui.h"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include "../middleware/imgui/imgui.h"
#include "../middleware/imgui/backends/imgui_impl_sdl2.h"
#include "../middleware/imgui/backends/imgui_impl_opengl3.h"
#include "../middleware/implot/implot.h"

#include <CLI11/CLI11.hpp>

ImFont *bigger;

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
    void Update(const Camera* camera, const glm::dmat4& transform);

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
    bool moves = false;
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
    float GetTerrainHeight(const glm::vec3& p);
    float GetTerrainHeightUnscaled(const glm::vec3& p);
    float ScaleHeightNoise(float noise);

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

    static glm::dvec3 SunlightDir(TerrainBody *planet, TerrainBody *sun,
                                  Frame *renderFrame) {
        const glm::dvec3 d_root = sun->frame->root_pos - planet->frame->root_pos;
        return -glm::normalize(sun->frame->GetOrientRelTo(renderFrame) * d_root);
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

    void Update(const Camera* camera) {
        for(auto&& patch : patches) {
            patch->Update(camera, transform);
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

// Elevation palettes, defined later in the file; needed by load_system().
COLOUR GetColourMoon(float v, float vmin, float vmax);
COLOUR GetColourSun(float v, float vmin, float vmax);
COLOUR GetColourEarth(float v, float vmin, float vmax);

//  A loaded star system
struct System {
    std::vector<TerrainBody *> bodies;
    TerrainBody *root;      // the star (frame-tree root)
    TerrainBody *home;      // the planet the ship starts on
    TerrainBody *star;      // == root
    TerrainBody *moon;      // home's first moon, or NULL

    TerrainBody *find(const std::string &name) {
        for(auto&& b : bodies) {
            if(b->name == name) { return b; }
        }
        return nullptr;
    }
};

// TODO are these detailed docs needed?
/*
  load_system() reads a star-system description from a JSON file and builds,
  together, each TerrainBody (terrain mesh + physics constants) and the
  reference frames that belong to it:

    * every body gets an INERTIAL (non-rotating) frame, carrying the body's
      true sphere of influence and its orbital angular speed;
    * every body ALSO gets a ROTATING (near-body) frame, a child of its
      inertial frame in the frame tree, carrying the small "near body" SOI;
      bodies without a "rotating" JSON section (e.g. the star) get a dummy
      one with zero spin and soi = radius + 100 km;
    * the parent/child frame tree the per-tick SOI logic walks.

  JSON layout (see system.json and ksp_system.json):
    {
      "home": "<name of the planet the ship starts on>",
      "bodies": [
        {
          "name":  "...",
          "type":  "star" | "planet" | "moon",
          "orbits": "<parent body name>" | (omitted/null for the star),
          "radius":  metres,
          "mass":    kg,
          "g":       m/s^2 (surface gravity, used for TWR),
          "seed":         terrain-noise seed (noise-domain offset; 0 = legacy pattern),
          "has_sea":      bool,              // legacy: implies surface.sea_level = 0
          "power_scaler": int,               // legacy: default for surface.power
          "surface": {                       // optional; every field optional
            "amplitude":   m,                // peak noise height        (2500)
            "octaves":     int,              // simplex octaves          (12)
            "persistence": 0..1,             // octave falloff           (0.6)
            "frequency":   float,            // noise-domain scale       (1.0)
            "power":       int,              // height-distribution exp  (3)
            "sea_level":   m,                // key present => has ocean (0)
            "sea_color":   [r,g,b],          //                         (0.1,0.1,0.8)
            "palette":     [ [t, [r,g,b]], ... ]   // land-color stops, t in 0..1
            "bands":       bool,             // gas giant: smooth sphere,
                                             // latitude bands, no terrain
            "band_count":  int,              // stripes pole to pole (9);
                                             // odd => bright band at equator
          },
          "moves":   bool,
          "inertial": { "soi": m, "pos": [x,y,z], "orb_ang_speed": rad/s,
                        "orb_incl": rad },   // optional; tilt of the orbital
                                             // plane about the parent's +X
                                             // (line of nodes); 0 = coplanar
          "rotating": { "soi": m, "rot_ang_speed": rad/s,
                        "axial_tilt": rad }  // optional; lean of the spin axis
                                             // from the orbital normal toward
                                             // +X; 0 = pole on the orbit normal
        },   // "rotating" absent => dummy (zero spin, soi = radius + 100 km)
        ...
      ]
    }

  `mu` is derived from `mass` (mu = G * mass) to keep the files minimal. The
  bodies must list parents before children is NOT required — the frame tree is
  wired in a second pass, so the order in the file does not matter.

  Must be called after create_physics(), because Create() builds Bullet terrain
  collision.
*/
System load_system(const char *path, Shader *terrainshader, Shader *sunshader) {
    std::ifstream f(path);
    if(!f.is_open()) {
        throw std::runtime_error(std::string("system: cannot open ") + path);
    }
    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(f, nullptr, true);
    } catch(const std::exception &e) {
        throw std::runtime_error(std::string("system: bad JSON in ") + path
                                 + std::string(": ") + e.what());
    }

    if(!doc.is_object() || !doc.contains("bodies") || !doc["bodies"].is_array()
       || doc["bodies"].empty()) {
        throw std::runtime_error(std::string("system: no bodies in ") + path);
    }
    const nlohmann::json &bodies = doc["bodies"];
    const std::string home_name = doc.value("home", std::string(""));

    const double G = 6.674e-11;

    System sys;
    sys.root = nullptr;
    sys.home = nullptr;
    sys.star = nullptr;
    sys.moon = nullptr;

    // --- pass 1: create every body and its frames --------------------------
    for(size_t i = 0; i < bodies.size(); i++) {
        const nlohmann::json &bv = bodies[i];

        TerrainBody *body = new TerrainBody;
        body->frame = nullptr;
        body->rot_frame = nullptr;

        body->name       = bv.value("name", std::string("body"));
        const std::string type = bv.value("type", std::string("planet"));
        const double radius = bv.value("radius", 600000.0);
        const double mass   = bv.value("mass", 5e22);
        body->radius       = (float)radius;
        body->mass         = (float)mass;
        body->g            = bv.value("g", 9.81);
        body->mu           = G * mass;
        body->seed         = bv.value("seed", 0.0);
        body->moves        = bv.value("moves", false);

        // Legacy flat fields act as defaults for the surface parameters.
        Surface &s = body->surface;
        s.power   = bv.value("power_scaler", 3);
        s.has_sea = bv.value("has_sea", false);

        if(bv.contains("surface") && bv["surface"].is_object()) {
            const nlohmann::json &sv = bv["surface"];
            s.amplitude   = sv.value("amplitude", s.amplitude);
            s.octaves     = sv.value("octaves", s.octaves);
            s.persistence = sv.value("persistence", s.persistence);
            s.frequency   = sv.value("frequency", s.frequency);
            s.power       = sv.value("power", s.power);
            if(sv.contains("sea_level")) {
                s.has_sea = true;
                s.sea_level = sv.value("sea_level", 0.0);
            }
            if(sv.contains("sea_color") && sv["sea_color"].is_array()
               && sv["sea_color"].size() >= 3) {
                const nlohmann::json &c = sv["sea_color"];
                s.sea_color = glm::vec3(c[0].get<float>(),
                                        c[1].get<float>(),
                                        c[2].get<float>());
            }
            if(sv.contains("palette") && sv["palette"].is_array()) {
                for(const nlohmann::json &stop : sv["palette"]) {
                    if(!stop.is_array() || stop.size() < 2
                       || !stop[1].is_array() || stop[1].size() < 3) {
                        continue;
                    }
                    PaletteStop ps;
                    ps.t = stop[0].get<float>();
                    ps.color = glm::vec3(stop[1][0].get<float>(),
                                         stop[1][1].get<float>(),
                                         stop[1][2].get<float>());
                    s.palette.push_back(ps);
                }
                std::sort(s.palette.begin(), s.palette.end(),
                          [](const PaletteStop &a, const PaletteStop &b) {
                              return a.t < b.t;
                          });
            }
            s.bands = sv.value("bands", false);
            if(sv.contains("band_count") && sv["band_count"].is_number_integer()) {
                s.band_count = std::max(1, sv["band_count"].get<int>());
            }
            if(sv.contains("atmosphere") && sv["atmosphere"].is_object()) {
                const nlohmann::json &av = sv["atmosphere"];
                s.atmosphere.enabled = true;
                if(av.contains("color") && av["color"].is_array()
                   && av["color"].size() >= 3) {
                    const nlohmann::json &c = av["color"];
                    s.atmosphere.color = glm::vec3(c[0].get<float>(),
                                                   c[1].get<float>(),
                                                   c[2].get<float>());
                }
                // thickness omitted => a visible rim at ~2% of the radius
                s.atmosphere.thickness =
                    av.value("thickness", (float)(body->radius * 0.02));
                s.atmosphere.power = av.value("power", 3.0f);
                s.atmosphere.intensity = av.value("intensity", 1.0f);
            }
        }
        // Scaled relief a full-amplitude peak ends up at, after
        // ScaleHeightNoise() — used to normalize palette elevation t.
        if(s.amplitude > 0.0f) {
            s.max_height = s.amplitude
                           * std::pow(s.amplitude / 3000.0f, s.power);
        }
        if(s.max_height <= 0.0f) {
            s.max_height = 1.0f;
        }
        s.seed_offset = glm::vec3((float)body->seed * 100.0f);

        // Shader + elevation palette by body type.
        if(type == "star") {
            body->shader = sunshader;
            body->colour_func = GetColourSun;
        } else if(type == "moon") {
            body->shader = terrainshader;
            body->colour_func = GetColourMoon;
        } else { // "planet"
            body->shader = terrainshader;
            body->colour_func = GetColourEarth;
        }

        // --- inertial (non-rotating) frame ---------------------------------
        Frame *f = new Frame;
        f->name  = body->name + " (inertial)";
        f->body  = body;
        f->parent = nullptr;                 // wired in pass 2
        f->children.clear();
        f->rotating = false;
        f->has_rot_frame = false;
        f->pos = glm::dvec3(0, 0, 0);
        f->initial_pos = f->pos;
        // GLM 1.0.0+: default-constructed matrices are zero, not identity.
        f->initial_orient = glm::dmat3(1.0);
        f->orient = glm::dmat3(1.0);
        f->vel = glm::dvec3(0);
        f->orb_ang_speed = 0;
        f->rot_ang_speed = 0;
        f->spin_axis = glm::dvec3(0, 1, 0);   // inertial frame does not spin
        f->soi = 1e6;
        f->root_pos = glm::dvec3(0);
        f->root_vel = glm::dvec3(0);
        f->root_orient = glm::dmat3(1.0);

        if(bv.contains("inertial") && bv["inertial"].is_object()) {
            const nlohmann::json &in = bv["inertial"];
            f->soi = in.value("soi", 1e6);
            if(in.contains("pos") && in["pos"].is_array() && in["pos"].size() >= 3) {
                const nlohmann::json &pos = in["pos"];
                f->pos = glm::dvec3(pos[0].get<double>(),
                                    pos[1].get<double>(),
                                    pos[2].get<double>());
                f->initial_pos = f->pos;
            }
            f->orb_ang_speed = in.value("orb_ang_speed", 0.0);
            // Optional orbital inclination (radians): tilt the orbital plane
            // about the parent's X axis (line of nodes along parent +X). orient
            // then maps the local orbital plane (where pos lives) into the
            // parent frame; identity when absent / zero.
            const double orb_incl = in.value("orb_incl", 0.0);
            if(orb_incl != 0.0) {
                const double c = std::cos(orb_incl), s = std::sin(orb_incl);
                f->orient = glm::dmat3(glm::dvec3(1.0, 0.0, 0.0),
                                       glm::dvec3(0.0,  c,  s),
                                       glm::dvec3(0.0, -s,  c));
            }
        }
        body->frame = f;
        body->soi = f->soi;   // keep the body's soi in sync (display only)

        // --- rotating (near-body) frame -------------------------------------
        // Every body gets one. A body without a "rotating" JSON section (e.g.
        // the star) gets a DUMMY frame: zero spin (no stasis velocity, no
        // fictitious forces) and the standard near-body SOI (radius + 100 km,
        // the same convention the real data uses), so scenario radii, frame
        // resolution and frame switching work uniformly for every body.
        Frame *rf = new Frame;
        rf->name  = body->name + " (rotational)";
        rf->body  = body;
        rf->parent = f;                    // child of its own inertial frame
        rf->children.clear();
        rf->rotating = true;
        rf->has_rot_frame = false;
        rf->pos = glm::dvec3(0, 0, 0);
        rf->initial_pos = glm::dvec3(0, 0, 0);
        rf->initial_orient = glm::dmat3(1.0);
        rf->orient = glm::dmat3(1.0);
        rf->vel = glm::dvec3(0);
        rf->orb_ang_speed = 0;
        rf->spin_axis = glm::dvec3(0, 1, 0);   // no axial tilt by default
        rf->root_pos = glm::dvec3(0);
        rf->root_vel = glm::dvec3(0);
        rf->root_orient = glm::dmat3(1.0);
        if(bv.contains("rotating") && bv["rotating"].is_object()) {
            const nlohmann::json &rot = bv["rotating"];
            rf->soi = rot.value("soi", 1e5);
            rf->rot_ang_speed = rot.value("rot_ang_speed", 0.0);
            // Optional axial tilt (radians): lean the spin axis away from the
            // orbital normal (local +Y) toward +X. The spin axis in the body
            // frame is then (sin t, cos t, 0); (0,1,0) when absent / zero.
            const double axial_tilt = rot.value("axial_tilt", 0.0);
            if(axial_tilt != 0.0) {
                rf->spin_axis = glm::dvec3(std::sin(axial_tilt),
                                           std::cos(axial_tilt), 0.0);
            }
        } else {
            rf->soi = radius + 100e3;       // near-body SOI convention
            rf->rot_ang_speed = 0.0;        // dummy: does not spin
        }
        body->rot_frame = rf;
        f->has_rot_frame = true;
        f->children.push_back(rf);

        // Build the terrain mesh + collision (needs shader/colour_func set).
        body->Create((float)radius, (float)mass);

        sys.bodies.push_back(body);
    }

    // --- pass 2: wire the parent/child frame tree --------------------------
    for(size_t i = 0; i < sys.bodies.size(); i++) {
        TerrainBody *body = sys.bodies[i];
        const nlohmann::json &bv = bodies[i];
        const std::string parent_name = bv.value("orbits", std::string(""));
        if(parent_name.empty()) {
            // The star: root of the frame tree.
            if(sys.root != nullptr) {
                throw std::runtime_error("system: more than one root body");
            }
            sys.root = body;
        } else {
            TerrainBody *parent = sys.find(parent_name);
            if(parent == nullptr) {
                throw std::runtime_error("system: '" + body->name
                                         + "' orbits unknown body '"
                                         + parent_name + "'");
            }
            body->frame->parent = parent->frame;
            parent->frame->children.push_back(body->frame);
        }
    }

    if(sys.root == nullptr) {
        throw std::runtime_error("system: no root (star) body found");
    }
    sys.star = sys.root;

    // --- resolve the home planet and its first moon ------------------------
    if(!home_name.empty()) {
        sys.home = sys.find(home_name);
        if(sys.home == nullptr) {
            throw std::runtime_error("system: home body '" + home_name
                                     + "' not found");
        }
        // First moon = the first body whose parent is the home planet.
        for(size_t i = 0; i < sys.bodies.size(); i++) {
            const nlohmann::json &bv = bodies[i];
            if(bv.value("orbits", std::string("")) == home_name) {
                sys.moon = sys.bodies[i];
                break;
            }
        }
    } else if(sys.bodies.size() > 1) {
        // No explicit home: default to the first non-star body.
        for(size_t i = 0; i < sys.bodies.size(); i++) {
            if(sys.bodies[i] != sys.root) { sys.home = sys.bodies[i]; break; }
        }
    }

    // --- calendars ----------------------------------------------------------
    // Every body gets its own calendar from its spin (day) and orbit (year)
    // rates, so the HUD can show LOCAL time on whatever body the ship is in.
    // The year is snapped to a whole number of days (see calendar.h), so all
    // date boundaries fall on local midnight. Stars get an invalid calendar
    // (dummy zero-spin frame).
    const int epoch_year = 4724;  // the year the game starts in
    for(size_t i = 0; i < sys.bodies.size(); i++) {
        TerrainBody *b = sys.bodies[i];
        const double D = (b->rot_frame && b->rot_frame->rot_ang_speed > 0.0)
                       ? 2.0 * M_PI / b->rot_frame->rot_ang_speed : 0.0;
        const double Y = (b->frame && b->frame->orb_ang_speed > 0.0)
                       ? 2.0 * M_PI / b->frame->orb_ang_speed : 0.0;
        b->cal = Calendar::make(D, Y, epoch_year);
    }

    // Recompute the root-relative frame values so positions/velocities/orients
    // are consistent before the first render.
    sys.root->frame->UpdateOrbitRails(0.0, 1.0 / 60.0);

    printf("Loaded system '%s': %zu bodies (home=%s, moon=%s)\n",
           path, sys.bodies.size(),
           sys.home ? sys.home->name.c_str() : "(none)",
           sys.moon ? sys.moon->name.c_str() : "(none)");

    return sys;
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

void GeoPatch::Update(const Camera* camera, const glm::dmat4& transform) {
    const glm::dvec3& camera_pos = camera->GetPos() - (glm::dvec3)(transform[3]);
    const glm::dvec3& centroid_pos = body->GetTerrainHeight(glm::normalize(camera_pos)) * centroid;
    const float dist = glm::length(camera_pos - centroid_pos);
    const float subdiv = 2.0f * body->radius * glm::length(v0 - centroid);

    if(depth < max_depth and dist < subdiv) {
        if(kids[0] == NULL) {
            Subdivide();
        }
    }
    else if(dist > subdiv * 2) {
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
        kids[0]->Update(camera, transform);
        kids[1]->Update(camera, transform);
        kids[2]->Update(camera, transform);
        kids[3]->Update(camera, transform);
    }
}

/* ResourceType / ResourceContent / PartDef live in shipdef.h (the GL-free
   ship/part data model), shared with the JSON loaders and the headless
   tests. */

// Per-part terrain shadow factor: 1.0 = lit, <1.0 = the planet's terrain
// occludes the line to the sun (see ComputeTerrainShadow).
float ComputeTerrainShadow(TerrainBody *planet, const Frame *posFrame,
                           const glm::dvec3 &posInFrame, TerrainBody *sun);

// One ship-control command. The input layer (keyboard, UI, or a future
// autopilot) emits these; Vehicle::Command() is the only path from control
// to physics, so rules that apply to all controls (e.g. "no commands while
// paused") live in one place instead of at every call site.
enum ShipCmdType {
    ThrottleUp,
    ThrottleDown,
    Thrust,
    Pitch,    // amount: +1 = W, -1 = S
    Yaw,      // amount: +1 = A, -1 = D
    Roll,     // amount: +1 = Q, -1 = E
    KillRot,
    Prograde,    // align nose with velocity
    Retrograde,  // align nose against velocity
};

struct ShipCmd {
    ShipCmdType type;
    float amount;
    ShipCmd(ShipCmdType t, float a = 0.0f) : type(t), amount(a) { }
};

/* Autopilot slew targets -- mutually exclusive (last one wins). The manual
   stick is separate from these and composes with them. */
enum SlewMode {
    SlewNone = 0,
    SlewPrograde,    /* align nose with velocity */
    SlewRetrograde,  /* align nose against velocity */
    SlewKillRot      /* kill the spin */
};

class Vehicle {
public:
    std::string name;   // display name (def name, disambiguated in main)
    std::vector<Body *> parts;
    std::vector<ResourceContent> partResources;

    TerrainBody *m_parent;
    Frame *frame;
    TerrainBody *sun = nullptr; // the star (light source); set in main
    float m_thrust;

    glm::dvec3 m_com;

    Body *controller;
    int controllerIndex = -1;   // part index; -1 = last part (set by build_ship)
    std::vector<Body *> m_thrusters;
    std::vector<Body *> m_reaction_wheels;
    std::vector<void *> constraints;
    /* which parts each constraint joins: (parentIdx, childIdx) into
       `parts`, parallel to `constraints`. Staging splits on these links. */
    std::vector<std::pair<size_t, size_t>> constraintLinks;
    /* the weld's LOCAL anchor points (parentAnchor, childAnchor), parallel
       to `constraints`. The rails handoff detaches every weld and re-glues
       from these when the ship re-enters physics. */
    std::vector<std::pair<glm::dvec3, glm::dvec3>> constraintAnchors;

    /* Rails: an idle ship in free fall coasts analytically on its two-body
       conic instead of being integrated: its welds and rigid bodies are
       parked out of the Bullet world and the rigid cluster's pose is
       re-derived from the conic every tick (attitude frozen inertially,
       like a torque-free body). Exact at any time accel, zero solver
       cost. While coasting, ship->frame is the SOI body's INERTIAL frame
       node (where the trajectory is a conic). A grounded ship instead
       FREEZES: same parking, but the pose stays static in the rotating
       surface frame (railFrozen) -- that is what enables rails warp with
       pad ships aboard. */
    bool onRails = false;
    bool railFrozen = false;    // grounded park: no conic, pose fixed in the
                                // (rotating) frame -- the planet's spin is
                                // carried by the render-frame transform
    glm::dvec3 rail_pos;      // m, cluster COM in ship->frame coords
    glm::dvec3 rail_vel;      // m/s, inertial, ship->frame coords
    glm::dmat3 rail_orient = glm::dmat3(1.0); // cluster axes -> frame axes
    std::vector<glm::dvec3> rail_rel_pos;     // parked part pose, cluster axes
    std::vector<glm::dmat3> rail_rel_rot;

    /* Per-part catalog specs (parallel to parts; set by build_ship() before
       init()). init() copies the behavior values into the per-thruster /
       per-wheel vectors below, so the catalog may be freed afterwards. */
    std::vector<const PartDef *> partDefs;
    /* per-part stage number (parallel to parts; set by build_ship from the
       ship def). 1 = single stage. Drives engine/fuel gating + separation. */
    std::vector<int> partStages;
    std::vector<double> m_thrusterThrust;  // N at full throttle (T = 2*rate*ve)
    std::vector<double> m_thrusterRate;    // kg/s per tank at full throttle
    std::vector<glm::dvec2> m_thrusterDims;  // (radius, height) m, parallel to m_thrusters
    std::vector<int> m_thrusterStage;     // stage of each thruster (parallel to m_thrusters)
    std::vector<double> m_wheelTorque;     // N m, rated
    double m_exhaustVel = 0;               // m/s (first engine; delta-v estimate)
    /* the armed per-thruster force for THIS tick (N at current throttle);
       armed by ApplyThrust, applied per substep, disarmed by clearThrust */
    std::vector<float> m_armedThrust;

    float thruster_util = 1.0;
    float thrust_mag = 0.0;   /* N this tick (sum of m_armedThrust); armed by ApplyThrust, cleared by clearThrust */

    /* Rotation is armed once per tick (Command) and executed per SUBSTEP
       (applyRotationForce, before every stepSimulation) -- like thrust,
       because Bullet clears the accumulated torque on each stepSimulation.
       stick: +-1 per commanded body axis (diagonals allowed, e.g. W+A);
       slew:  the autopilot target (exclusive). */
    float stick[3] = {0.0f, 0.0f, 0.0f};
    int slew = SlewNone;

    void setRoot(Body *part) {
        parts = { part };
    }

    /* Weld `part` to the part at `parentIdx` at the given LOCAL anchor
       points (which must coincide in world space -- the 6DOF weld enforces
       zero relative linear offset). Records the link for staging. The
       caller has already setPosRot-ed the child to the matching pose. */
    void attach(Body *part, size_t parentIdx,
                const glm::dvec3 &parentAnchor, const glm::dvec3 &childAnchor) {
        void *constraint = GlueTogether(parts[parentIdx], part,
                                        parentAnchor, childAnchor);
        parts.push_back(part);
        constraints.push_back(constraint);
        constraintLinks.push_back(std::make_pair(parentIdx, parts.size() - 1));
        constraintAnchors.push_back(std::make_pair(parentAnchor, childAnchor));
    }

    void attachDown(Body *part, const PartDef *def) {
        /* weld at the part faces: parent bottom (-h/2) to child top (+h/2);
           generalizes the old hardcoded +-1 m (2 m parts). partDefs is
           parallel to parts, so the parent's def is the last one pushed. */
        const PartDef *parent = partDefs.back();
        attach(part, parts.size() - 1,
               glm::dvec3(0.0, 0.0, -parent->height / 2.0),
               glm::dvec3(0.0, 0.0,  def->height / 2.0));
    }

    void attachRadial(Body *part, const PartDef *def) {
        /* weld the part to the parent's SIDE: the part's local +Z axis is
           rotated to the parent's local +X (call site), so the part's
           bottom face (local -h/2) touches the parent's side at +radius.
           The anchors coincide in world space at the tangent point
           (parent local (r,0,0) == part local (0,0,-h/2)).
           Same partDefs convention as attachDown. */
        const PartDef *parent = partDefs.back();
        attach(part, parts.size() - 1,
               glm::dvec3(parent->radius, 0.0, 0.0),
               glm::dvec3(0.0, 0.0, -def->height / 2.0));
    }

    void attachSide(Body *part, const PartDef *def) {
        /* weld the part to the parent's SIDE with PARALLEL axes: the part
           keeps the parent's local +Z axis (no rotation at the call site),
           sits along the parent's local +X, and its cylindrical surface
           touches the parent's at +radius. The anchors coincide in world
           space at the tangent point (parent local (r,0,0) == part local
           (-r,0,0)). Unlike attachRadial the child is NOT rotated, so this
           is the "side by side, parallel axes" case.
           Same partDefs convention as attachDown. */
        const PartDef *parent = partDefs.back();
        attach(part, parts.size() - 1,
               glm::dvec3(parent->radius, 0.0, 0.0),
               glm::dvec3(-def->radius, 0.0, 0.0));
    }

    void init() {
        partResources.resize(parts.size());
        /* the cockpit part; build_ship() sets controllerIndex from the ship
           def (default -1 = last part, the old behavior) */
        int ci = controllerIndex < 0 ? (int)parts.size() - 1 : controllerIndex;
        controller = parts[(size_t)ci];
        NeverSleep(controller);
        if(partDefs.size() != parts.size()) {
            throw std::runtime_error("Vehicle::init: partDefs must be parallel to parts");
        }
        if(partStages.size() != parts.size()) {
            throw std::runtime_error("Vehicle::init: partStages must be parallel to parts");
        }
        /* propellant reservoirs: seed each tank part's resources so the
           thrusters can draw from them (they shed mass as they burn). Only
           done at construction -- separateStage() must NOT re-seed, so this
           stays here and out of rebuildBehavior(). */
        for(size_t i = 0; i < parts.size(); i++) {
            const PartDef *d = partDefs[i];
            bool has_capacity = false;
            for(size_t r = 0; r < d->capacity.size(); r++) {
                if(d->capacity[r] > 0.0f) { has_capacity = true; }
            }
            if(has_capacity) {
                for(int r = 0; r < (int)ResourceType::Num; r++) {
                    partResources[i].capacity[r] = d->capacity[r];
                    partResources[i].current[r] = d->capacity[r];
                }
            }
        }
        rebuildBehavior();
    }

    /* (Re)build the per-thruster / per-wheel behavior vectors from partDefs.
       Safe to call again after separateStage() has shrunk the part set: it
       touches ONLY the m_* vectors -- never partResources (fuel is never
       re-seeded) and never the parts/constraints themselves. */
    void rebuildBehavior() {
        m_thrusters.clear();
        m_reaction_wheels.clear();
        m_thrusterThrust.clear();
        m_thrusterRate.clear();
        m_thrusterDims.clear();
        m_thrusterStage.clear();
        m_wheelTorque.clear();
        m_exhaustVel = 0;
        for(size_t i = 0; i < parts.size(); i++) {
            const PartDef *d = partDefs[i];
            /* behavior is field-driven (see shipdef.h): a part may carry
               any combination of these, so the checks are independent */
            if(d->torque > 0.0) {
                m_reaction_wheels.push_back(parts[i]);
                m_wheelTorque.push_back(d->torque);
            }
            if(d->fuel_rate > 0.0 && d->exhaust_velocity > 0.0) {
                m_thrusters.push_back(parts[i]);
                m_thrusterThrust.push_back(d->fullThrust());
                m_thrusterRate.push_back(d->fuel_rate);
                m_thrusterDims.push_back(glm::dvec2(d->radius, d->height));
                m_thrusterStage.push_back(partStages[i]);
                if(m_exhaustVel == 0.0) { m_exhaustVel = d->exhaust_velocity; }
            }
        }
        /* no thrust may be armed across a rebuild (a split just happened) */
        m_armedThrust.assign(m_thrusters.size(), 0.0f);
    }

    /* Draw `amt` kg of `type` from a tank on the ACTIVE stage only (stages
       are self-contained: a stage burns its own propellant, so the booster's
       engines never drain the upper stage's tanks). Returns true if fuel was
       consumed. amt is the kg consumed THIS tick (the caller scales the kg/s
       flow by the tick's simulated time). */
    bool consumeResourceMass(enum ResourceType type, float amt /* kg */) {
        const int as = activeStage();
        for(size_t i = 0; i < parts.size(); i++) {
            if(partStages[i] != as) { continue; }
            if(partResources[i].current[(int)type] >= amt) {
                partResources[i].current[(int)type] -= amt;
                parts[i]->mass -= amt; /* why does Body have mass at all? */
                void SetMass(Body *body, double newMass);
                SetMass(parts[i], parts[i]->mass);
                return true;
            }
        }
        return false;
    }

    float getFuelMass(const std::vector /* eh */ <enum ResourceType>& types) {
        float fuel = 0;
        for(auto&& type : types) {
            for(auto&& partResource : partResources) {
                fuel += partResource.current[(int)type];
            }
        }
        return fuel;
    }

    float getDeltaV() {
        float remaining_fuel = getFuelMass({ ResourceType::Hydrogen, ResourceType::LOX }); /* kg */
        return (float)m_exhaustVel * log(getMass() / (getMass() - remaining_fuel));
    }

    /* TODO should be cached per frame */
    float getMass() {
        float r = 0;
        for(auto&& part : parts) {
            r += part->mass;
        }
        return r;
    }

    /* The active (currently live) stage: the lowest stage number still on
       the ship. Stages are dropped low-to-high, so this is what fires and
       what separateStage() detaches next. */
    int activeStage() {
        if(partStages.empty()) { return 1; }
        int s = partStages[0];
        for(size_t i = 1; i < partStages.size(); i++) {
            if(partStages[i] < s) { s = partStages[i]; }
        }
        return s;
    }

    /* Total number of stages on the ship (the highest stage number --
       stages are labelled 1..N from the booster up, so the max label IS the
       total). Deliberately the max, not the count of *remaining* stages, so
       the "stage X of N" readout stays stable after the active stage is
       dropped (a remaining-count would read e.g. "stage 2 of 1"). */
    int numStages() {
        int n = 0;
        for(size_t i = 0; i < partStages.size(); i++) {
            if(partStages[i] > n) { n = partStages[i]; }
        }
        return n;
    }

    float getThrust() {
        return GetActiveThrust() * thruster_util;
    }

    // current Thrust-to-weight ratio
    float getTWR() {
        return (thruster_util * GetActiveThrust()) / (getMass() * m_parent->g);
    }

    // full throttle TWR
    float getFullThrustTWR() {
        return GetActiveThrust() / (getMass() * m_parent->g);
    }

    // empty TWR
    float getMaxTWR() {
        float remaining_fuel = getFuelMass({ ResourceType::Hydrogen, ResourceType::LOX }); /* kg */
        return GetActiveThrust() / ((getMass() - remaining_fuel) * m_parent->g);
    }

    void setVelocity(glm::dvec3 vel) {
        for(auto&& part : parts) {
            SetVelocity(part, vel);
        }
    }

    void setPosition(glm::dvec3 pos) {
        void SetPosition(Body *b, glm::dvec3 com, glm::dvec3 pos);
        for(auto&& part : parts) {
            SetPosition(part, get_center_of_mass(), pos);
        }
    }

    const glm::dvec3& get_center_of_mass(void) {
        double total_mass = 0;
        for(auto&& part : parts) {
            total_mass += part->mass;
        }
        m_com = glm::dvec3(0, 0, 0);
        for(auto&& part : parts) {
            m_com += GetPosition(part) * (part->mass / total_mass);
        }
        return m_com;
    }

    glm::dvec3 applyGravity() {
        const double& parent_mass = m_parent->mass;
        const double G = 6.674e-11;
        glm::dvec3 gf;
        for(auto&& part : parts) {
            if(part->mass == 0) { continue; }
            const glm::dvec3& b1b2 = GetPosition(part);
            const double m1m2 = part->mass * parent_mass;
            const double invrsqr = 1.0 / glm::length2(b1b2);
            const double mag = G * m1m2 * invrsqr;
            gf = mag * sqrt(invrsqr) * -b1b2;
            ApplyCentralForce(part, gf, mag);
            if(frame->isRotFrame()) {
                // In rotating coordinates the ship additionally feels
                // Coriolis + centrifugal; without these its true inertial
                // orbit is perturbed for as long as it spends in the rotating
                // frame (see GetFictitiousAccel in frame.h).
                const glm::dvec3 a_fict = frame->GetFictitiousAccel(b1b2, GetVelocity(part));
                ApplyCentralForce(part, part->mass * a_fict);
            }
        }
        return gf;
    }

public:
    Vehicle() { }
    virtual ~Vehicle() {
        for(auto&& part : parts) { delete part; }
    }

    glm::dvec3 processGravity() {
        return applyGravity();
    }

    // Bullet clears all accumulated forces on every stepSimulation, so the
    // thrust -- like gravity -- must be re-applied before EVERY substep.
    // Applied once per tick it would only act during the first substep's
    // h seconds of the tick's n*h, cutting the delivered thrust to 1/n
    // (and n grows with time acceleration, so it got worse at warp).
    void applyThrustForce() {
        for(size_t i = 0; i < m_thrusters.size(); i++) {
            if(m_armedThrust[i] == 0.0f) { continue; }
            ApplyCentralForceForward(m_thrusters[i], (double)m_armedThrust[i]);
        }
    }

    // The armed rotation commands -- like thrust -- are re-applied before
    // EVERY substep (h = that substep's duration); applied once per tick
    // they would act only during the first substep, cutting the delivered
    // authority to 1/n and making it worse at warp.
    void applyRotationForce(double h) {
        if(m_reaction_wheels.empty()) { return; }
        /* Manual stick: the full rated torque along each commanded body
           axis (each wheel at its rating; diagonals compose). */
        if(stick[0] != 0.0f || stick[1] != 0.0f || stick[2] != 0.0f) {
            for(size_t wi = 0; wi < m_reaction_wheels.size(); wi++) {
                Body *wheel = m_reaction_wheels[wi];
                for(int a = 0; a < 3; a++) {
                    if(stick[a] == 0.0f) { continue; }
                    ApplyTorque(wheel,
                               (double)stick[a] * m_wheelTorque[wi] * getRelAxis_(wheel, a));
                }
            }
        }
        /* Autopilot: one authority-bounded step of the slew/kill-rot law
           (h = this substep's duration, so the law re-evaluates per
           substep -- the stable form of the same law). */
        if(slew == SlewPrograde) {
            slewToward(GetVel(), h);
        }
        else if(slew == SlewRetrograde) {
            slewToward(-GetVel(), h);
        }
        else if(slew == SlewKillRot) {
            killRotStep(h);
        }
    }

    /* the largest wheel's rated torque (N m) -- the per-wheel rating for
       the HUD; the ship's TOTAL wheel authority is maxTorque() (the sum) */
    float GetWheelTorque() {
        double t = 0.0;
        for(size_t i = 0; i < m_wheelTorque.size(); i++) {
            if(m_wheelTorque[i] > t) { t = m_wheelTorque[i]; }
        }
        return (float)t;
    }

    /* disarm the armed thrust (called once per tick, like clearRotCmd,
       so a tick without the keys doesn't keep firing) */
    void clearThrust() {
        thrust_mag = 0.0f;
        for(size_t i = 0; i < m_armedThrust.size(); i++) { m_armedThrust[i] = 0.0f; }
    }

    /* disarm the armed rotation commands (called once per tick, so a tick
       without the keys doesn't keep rotating) */
    void clearRotCmd() {
        stick[0] = stick[1] = stick[2] = 0.0f;
        slew = SlewNone;
    }

    /* called when control moves to ANOTHER ship: zero the throttle and
       clear the armed thrust + rotation commands, so this ship just
       coasts under its own physics from here on (no residual forces,
       no fuel flow). Control input reaches only the active ship. */
    void releaseControl() {
        thruster_util = 0.0f;
        clearThrust();
        clearRotCmd();
    }

    /* Separate `stage`: cut the welds joining it to the rest of the ship,
       remove its parts from the Bullet world and from this ship's part
       set, and delete them. The surviving parts keep their relative
       geometry (their internal welds are untouched). Refuses to drop the
       whole ship. Returns the number of parts dropped (0 = no-op). Call at
       a tick boundary, not mid-substep. */
    int separateStage(int stage) {
        const size_t n = parts.size();
        if(n < 2) { return 0; }   // single-part ship: nothing to separate

        /* drop set = every part on this stage */
        std::vector<bool> drop(n, false);
        size_t dropped = 0;
        for(size_t i = 0; i < n; i++) {
            if(partStages[i] == stage) { drop[i] = true; dropped++; }
        }
        if(dropped == 0) { return 0; }   // nothing on this stage
        if(dropped == n) { return 0; }   // can't drop the whole ship

        /* the controller part's OLD index (to remap, or replace if dropped) */
        const int oldCi = controllerIndex < 0 ? (int)n - 1 : controllerIndex;

        StageSplit split = computeStageSplit(n, constraintLinks, drop);

        /* 1) Remove every constraint touching a dropped part (the stage
           interface AND the dropped set's internal welds). */
        for(size_t c : split.cutConstraints) {
            Detach(constraints[c]);   // out of the world + deleted
        }
        /* 2) Unregister + delete the dropped parts' bodies. */
        for(size_t i : split.droppedParts) {
            RemoveBody(parts[i]);     // unregister from the world
            delete parts[i];          // frees model + rigid body
        }
        /* 3) Rebuild the part-set vectors from the kept parts. */
        std::vector<Body *>          keepParts;
        std::vector<const PartDef *> keepDefs;
        std::vector<ResourceContent> keepRes;
        std::vector<int>             keepStages;
        for(size_t i : split.keptParts) {
            keepParts.push_back(parts[i]);
            keepDefs.push_back(partDefs[i]);
            keepRes.push_back(partResources[i]);
            keepStages.push_back(partStages[i]);
        }
        parts.swap(keepParts);
        partDefs.swap(keepDefs);
        partResources.swap(keepRes);
        partStages.swap(keepStages);
        /* 4) Keep only the surviving constraints; their links are already
           remapped into the new (kept-only) index space. */
        std::vector<bool> isCut(constraints.size(), false);
        for(size_t c : split.cutConstraints) { isCut[c] = true; }
        std::vector<void *> keepCons;
        std::vector<std::pair<glm::dvec3, glm::dvec3>> keepAnchors;
        for(size_t c = 0; c < constraints.size(); c++) {
            if(!isCut[c]) {
                keepCons.push_back(constraints[c]);
                keepAnchors.push_back(constraintAnchors[c]);
            }
        }
        constraints.swap(keepCons);
        constraintAnchors.swap(keepAnchors);
        constraintLinks = split.keptLinks;
        /* 5) Remap the controller (or, if it was itself dropped, fall back
           to the first surviving part). */
        if(split.newIndexOf[(size_t)oldCi] >= 0) {
            controllerIndex = (int)split.newIndexOf[(size_t)oldCi];
        } else {
            controllerIndex = 0;
        }
        controller = parts[(size_t)controllerIndex];
        NeverSleep(controller);
        /* 6) Rebuild the thruster/wheel vectors (also disarms any thrust). */
        rebuildBehavior();
        return (int)dropped;
    }

    /* renderFrame: the frame the camera view is built in. Usually this
       ship's own frame (identity transform); an idle ship that switched
       SOI while another ship was being controlled lives in a different
       frame, so transform its parts into the render frame first. */
    void Draw(const Camera* camera, Frame *renderFrame) {
        // Light direction in this frame's axes
        glm::vec3 sunlightVec = glm::vec3(TerrainBody::SunlightDir(m_parent, sun, frame));

        glm::dmat4 xform = glm::dmat4(1.0);
        if(frame != renderFrame) {
            xform = glm::translate(frame->GetPositionRelTo(renderFrame))
                  * glm::dmat4(frame->GetOrientRelTo(renderFrame));
        }

        for(auto&& part : parts) {
            // Per-part terrain shadow
            const float shadow = ComputeTerrainShadow(m_parent, frame, GetPosition(part), sun);
            part->Draw(camera, sunlightVec, shadow, xform);
        }
    }

    // Single place to control the ship. While paused (simActive == false)
    // every command is dropped, so nothing accumulates in the rigid bodies
    // (a force/torque left in Bullet would dump out as a velocity kick on
    // resume) and settings like throttle stay frozen.
    /* step = the tick's simulated duration (dt * time_accel); only Thrust
       uses it (to scale this tick's fuel flow). */
    void Command(ShipCmd cmd, bool simActive, double step = 0.0) {
        if(not simActive)
            return;
        switch(cmd.type) {
            case ThrottleUp:
                adjustThrottle(+0.01);
                break;
            case ThrottleDown:
                adjustThrottle(-0.01);
                break;
            case Thrust:
                ApplyThrust(step);
                break;
            case Pitch:
                stick[0] = (cmd.amount >= 0) ? +1.0f : -1.0f;
                break;
            case Yaw:
                stick[1] = (cmd.amount >= 0) ? +1.0f : -1.0f;
                break;
            case Roll:
                stick[2] = (cmd.amount >= 0) ? +1.0f : -1.0f;
                break;
            case KillRot:
                slew = SlewKillRot;
                break;
            case Prograde:
                slew = SlewPrograde;
                break;
            case Retrograde:
                slew = SlewRetrograde;
                break;
        }
    }

    glm::dvec3 GetVel() {
        return GetVelocity(controller);
    }

private:
    // Control implementation: applies forces/torques to the Bullet bodies
    // directly, so it is reachable only through Command() above.
    void adjustThrottle(float delta) {
        thruster_util += delta;
        if(thruster_util > 1) { thruster_util = 1; }
        if(thruster_util < 0) { thruster_util = 0; }
    }

    /* the ship's full-throttle thrust RIGHT NOW (N) = the sum of the ACTIVE
       stage's engines' full thrust (each T = (H2 + LOX flow) x ve = 2 x
       fuel_rate x ve, both propellants end up in the plume). Only the active
       stage fires, so this is the usable thrust; for a single-stage ship it
       equals the grand total. */
    float GetActiveThrust() {
        const int as = activeStage();
        double t = 0;
        for(size_t i = 0; i < m_thrusterThrust.size(); i++) {
            if(m_thrusterStage[i] == as) { t += m_thrusterThrust[i]; }
        }
        return (float)t;
    }

    /* Called once per physics tick (step = the tick's simulated duration).
       Consumes the tick's fuel and arms the per-thruster thrust; the force
       itself is applied by applyThrustForce() before EVERY substep below.
       A thruster that can't consume its flow this tick doesn't thrust. Only
       the ACTIVE stage's thrusters fire (and they draw the active stage's
       tanks -- see consumeResourceMass), so a non-active engine is skipped
       before any fuel is spent. */
    void ApplyThrust(double step) {
        if(thruster_util == 0.0f) { return; } /* zero throttle: no burn, no plume */
        thrust_mag = 0.0f;
        const int as = activeStage();
        for(size_t i = 0; i < m_thrusters.size(); i++) {
            if(m_thrusterStage[i] != as) { continue; } /* not the live stage */
            const float flow =
                (float)(m_thrusterRate[i] * (double)thruster_util * step); /* kg this tick, per tank */
            if(consumeResourceMass(ResourceType::Hydrogen, flow) and
               consumeResourceMass(ResourceType::LOX,      flow))
                {
                    m_armedThrust[i] = (float)(m_thrusterThrust[i] * thruster_util);
                    thrust_mag += m_armedThrust[i];
                    m_thrust = 1.0;
                }
        }
    }

    // --- physical rotation model (private law implementation) -------------
    // The reaction wheel is rated at GetWheelTorque() N m -- the most torque
    // it can apply to the ship -- so the ship's angular authority is
    // alpha = maxTorque() / I (rad/s^2) with I the ship's total moment of
    // inertia (kg m^2, from Bullet). Stick, prograde/retrograde slew and
    // kill-rot all work within that authority, so no command can be more
    // forceful than a maxed manual stick. (The thrust analogue: T = mdot*ve.)

    double maxTorque() {
        double t = 0;
        for(size_t i = 0; i < m_wheelTorque.size(); i++) { t += m_wheelTorque[i]; }
        return t;
    }

    /* The ship's total moment of inertia about its COM (kg m^2): each
       part's local inertia (as Bullet has it) in world coordinates, plus
       the parallel-axis term for its offset from the ship's COM. This is
       the inertia a torque actually moves -- the denominator of the
       wheel's authority. */
    glm::dmat3 getInertia() {
        const glm::dvec3 com = get_center_of_mass();
        glm::dmat3 I = glm::dmat3(0.0);
        for(auto&& part : parts) {
            const glm::dvec3 d = GetPosition(part) - com;
            const glm::dmat3 R = GetOrient(part);
            const glm::dvec3 il = getInertiaDiag(part);
            /* part's local inertia is diagonal (Bullet stores it that way);
               build the diagonal matrix explicitly -- GLM has no
               vec -> diagonal-mat constructor */
            const glm::dmat3 il_diag(
                il.x, 0.0, 0.0,
                0.0, il.y, 0.0,
                0.0, 0.0, il.z);
            I += R * il_diag * glm::transpose(R);
            I += part->mass * (glm::dot(d, d) * glm::dmat3(1.0) - glm::outerProduct(d, d));
        }
        return I;
    }

    /* Slew the nose (local +Z) toward `dir` within the wheel's authority:
       the target rate is the braking curve sqrt(2*alpha*E) -- the fastest
       rate from which the ship can still stop exactly at the target
       (E = the error angle) -- capped at E/(2h) so no substep can cross
       the target, and the per-substep rate change is bounded by alpha*h,
       so the command never exceeds a maxed manual stick. */
    void slewToward(glm::dvec3 dir, double h) {
        dir = glm::normalize(dir);
        if(glm::length2(dir) < 0.5) { return; } /* no velocity to align to */
        Body *wheel = m_reaction_wheels.front();
        const glm::dvec3 facing = getRelAxis_(wheel, 2);
        const double E = glm::acos(glm::clamp(glm::dot(facing, dir), -1.0, 1.0));
        if(E < 1e-9) { return; } /* already aligned */
        glm::dvec3 axis = glm::cross(facing, dir); /* + turns the nose toward dir */
        if(glm::length2(axis) < 1e-12) {
            /* nose ~ opposite dir: any axis perpendicular to facing works */
            axis = (std::fabs(facing.y) > 0.9) ? glm::dvec3(1, 0, 0) : glm::dvec3(0, 1, 0);
            axis = glm::normalize(axis - facing * glm::dot(axis, facing));
        }
        axis = glm::normalize(axis);
        const glm::dmat3 I = getInertia();
        const double Ieff = glm::dot(axis, I * axis); /* kg m^2 about the slew axis */
        if(Ieff <= 0.0) { return; }
        const double alpha = maxTorque() / Ieff; /* rad/s^2, wheel-limited */
        const double A = alpha * h;              /* max |domega| this substep */
        const double w = glm::dot(GetAngVelocity(wheel), axis);
        const double w_des = std::min(std::sqrt(2.0 * alpha * E), E / (2.0 * h));
        double dw = w_des - w;
        if(dw > A) { dw = A; }
        if(dw < -A) { dw = -A; }
        const glm::dvec3 torque = axis * (Ieff * dw / h); /* |torque| <= maxTorque() */
        for(auto&& rw : m_reaction_wheels) {
            ApplyTorque(rw, torque / (double)m_reaction_wheels.size());
        }
    }

    /* Kill the spin within the wheel's authority: each axis' rate drops by
       min(|w|, alpha*h) per substep -- monotonic, no sign flip, never more
       forceful than a maxed manual stick. */
    void killRotStep(double h) {
        Body *wheel = m_reaction_wheels.front();
        const glm::dvec3 w = GetAngVelocity(wheel);
        if(glm::length(w) < 0.001) { return; } /* at rest: nothing to kill */
        const glm::dmat3 I = getInertia();
        glm::dvec3 torque(0.0);
        for(int i = 0; i < 3; i++) {
            const double Iii = I[i][i];
            if(Iii <= 0.0 || w[i] == 0.0) { continue; }
            const double A = (maxTorque() / Iii) * h; /* max |dw| on this axis */
            const double dw = -w[i] * std::min(1.0, A / std::fabs(w[i]));
            torque[i] = Iii * dw / h; /* |torque[i]| <= maxTorque() */
        }
        for(auto&& rw : m_reaction_wheels) {
            ApplyTorque(rw, torque / (double)m_reaction_wheels.size());
        }
    }

public:

    glm::dmat3 GetOrientRelTo(Body *part, Frame *relTo)
    {
        glm::dmat3 GetOrient(Body *b);
        glm::dmat3 forient = frame->GetOrientRelTo(relTo);
        return forient * GetOrient(part);
    }

    glm::dvec3 GetPositionRelTo(Body *part, Frame *relTo) {
        glm::dvec3 fpos = frame->GetPositionRelTo(relTo);
        glm::dmat3 forient = frame->GetOrientRelTo(relTo);
        return forient * GetPosition(part) + fpos;
    }

    glm::dvec3 GetVelocityRelTo(Body *part, Frame *relTo) {
        glm::dmat3 forient = frame->GetOrientRelTo(relTo);
        glm::dvec3 vel = GetVelocity(part);
        glm::dvec3 pos = GetPosition(part);
        if(frame != relTo) vel += frame->GetStasisVelocity(pos);
        return forient * vel + frame->GetVelocityRelTo(relTo);
    }

    void moveToFrame(Frame *newFrame) {
        void setPosRot(Body *b, glm::dvec3 pos, glm::dmat3 rot);
        glm::dmat3 GetOrient(Body *b);

        int i = 0;
        for(auto&& part : parts) {
            const char *name = partDefs[i]->name.c_str();
            i++;

            glm::dvec3 oldVel = GetVelocity(part);
            glm::dvec3 vel = GetVelocityRelTo(part, newFrame);
            glm::dvec3 fpos = frame->GetPositionRelTo(newFrame);
            glm::dmat3 forient = frame->GetOrientRelTo(newFrame);

            glm::dvec3 newPos = forient * GetPosition(part) + fpos;
            glm::dmat3 newOrient = forient * GetOrient(part);

            glm::dvec3 pos = GetPosition(part);
            printf("@@@ %s OLD position: %.0f %.0f %.0f\n", name, pos.x, pos.y, pos.z);
            printf("@@@ %s NEW position: %.0f %.0f %.0f\n", name, newPos.x, newPos.y, newPos.z);

            setPosRot(part, newPos, newOrient);

            pos = GetPosition(part);
            // The stored velocity is the frame-coordinate velocity, so a ship's
            // inertial velocity is R*(v + stasis(p)) + V.  GetVelocityRelTo
            // already added the OLD frame's stasis term; the NEW frame's term
            // must be SUBTRACTED, or the ship's inertial velocity is wrong by
            // 2*stasis and the orbit jumps shape at every inertial->rotational
            // switch.
            glm::dvec3 newVel = vel - newFrame->GetStasisVelocity(pos);
            printf("@@@ %s OLD velocity: %.0f %.0f %.0f\n", name, oldVel.x, oldVel.y, oldVel.z);
            printf("@@@ %s NEW velocity: %.0f %.0f %.0f\n", name, newVel.x, newVel.y, newVel.z);

            SetVelocity(part, newVel);
        }
        frame = newFrame;
        m_parent = newFrame->body;
    }

    /* Per-tick SOI bookkeeping for THIS ship: if the ship is outside the
       current frame's SOI, move to the parent frame; else if it has
       entered a child's SOI, move to the nearest such child. Called once
       per tick, per ship (the frame tree is shared; each ship tracks its
       own position in it). */
    void switchFrames() {
        const glm::dvec3 com = get_center_of_mass();
        double ship_r = glm::length(com);
        if(ship_r > frame->soi + 10000) {
            // switching to parent SOI if there is one
            if(frame->parent != NULL) {
                glm::dvec3 pos = GetPosition(controller);
                printf("@@@ %s switching frame from %s to parent %s\n",
                       name.c_str(), frame->name.c_str(),
                       frame->parent->name.c_str());
                glm::dvec3 offset = frame->GetPositionRelTo(frame->parent);
                printf("@@@ Frame offset: %.0f %.0f %.0f\n", offset.x, offset.y, offset.z);
                printf("@@@@@ OLD position: %.0f %.0f %.0f\n", pos.x, pos.y, pos.z);
                moveToFrame(frame->parent);
                pos = GetPosition(controller);
                printf("@@@@@ NEW position: %.0f %.0f %.0f\n", pos.x, pos.y, pos.z);
            }
        }
        else {
            // check if we've entered a child SOI
            for(auto&& child : frame->children) {
                double dist = glm::length(GetPositionRelTo(controller, child));
                if(dist < child->soi - 10000) {
                    printf("@@@ %s switching frame from %s to child %s, distance: %.0f\n",
                           name.c_str(), frame->name.c_str(),
                           child->name.c_str(), dist);
                    moveToFrame(child);
                    break;
                }
            }
        }
    }

    /* Write the rail state into the parked part transforms (once per
       tick). Draw, get_center_of_mass and everything else that reads
       Bullet transforms then sees the railed ship's current pose even
       though its bodies are not in the world. The velocities are kept in
       sync too -- the cluster is rigid and torque-free, so every part
       shares the rail velocity with zero spin, and readers like
       --orbit-log and the HUD fit their elements to consistent data. */
    void writeRailPose() {
        for(size_t i = 0; i < parts.size(); i++) {
            setPosRot(parts[i],
                      rail_pos + rail_orient * rail_rel_pos[i],
                      rail_orient * rail_rel_rot[i]);
            SetVelocity(parts[i], rail_vel);
            SetAngVelocity(parts[i], glm::dvec3(0.0));
        }
    }

    /* Rails classification: a FLYING ship (periapsis clear of the terrain
       band) coasts on its conic; a GROUNDED one (periapsis inside the
       band) can only freeze in its rotating surface frame. Anything else
       -- e.g. a suborbital descent -- is not rail-eligible. */
    bool canRail() {
        if(onRails) { return true; }
        Frame *inertial = frame->getNonRotFrame();
        glm::dvec3 p = get_center_of_mass();
        glm::dvec3 v(0.0);
        double mtot = 0.0;
        for(auto&& part : parts) {
            v += GetVelocity(part) * part->mass;
            mtot += part->mass;
        }
        v /= mtot;
        if(frame != inertial) {
            v += frame->GetStasisVelocity(p);
            v = frame->GetOrientRelTo(inertial) * v + frame->GetVelocityRelTo(inertial);
            p = frame->GetOrientRelTo(inertial) * p + frame->GetPositionRelTo(inertial);
        }
        const OrbitElements el = computeOrbitElements(p, v, inertial->body->mu);
        if(el.periapsis <= inertial->body->radius + 3000.0) {
            return frame->isRotFrame();   // grounded: freeze needs the surface frame
        }
        return true;
    }

    /* Park this ship out of the physics world and coast it analytically.
       Refuses (returns false) and changes nothing if the ship is not
       rail-eligible (see canRail). Flying ships follow their conic in the
       body's inertial node; grounded ships freeze in the rotating surface
       frame. */
    bool goOnRails() {
        if(onRails) { return true; }
        if(!canRail()) { return false; }

        /* COM state in the body's inertial frame node, where the
           trajectory is a Kepler conic (same transform the HUD uses).
           Cluster velocity = mass-weighted mean of the part velocities
           (the rigid cluster coasts as one body; residual spin is
           discarded with the attitude). */
        Frame *oldFrame = frame;
        Frame *inertial = frame->getNonRotFrame();
        glm::dvec3 p = get_center_of_mass();
        const glm::dvec3 com_frame = p;   // pre-transform, old frame coords
        glm::dvec3 v(0.0);
        double mtot = 0.0;
        for(auto&& part : parts) {
            v += GetVelocity(part) * part->mass;
            mtot += part->mass;
        }
        v /= mtot;
        const glm::dvec3 vel_frame = v;   // pre-transform, old frame coords
        if(frame != inertial) {
            v += frame->GetStasisVelocity(p);
            v = frame->GetOrientRelTo(inertial) * v + frame->GetVelocityRelTo(inertial);
            p = frame->GetOrientRelTo(inertial) * p + frame->GetPositionRelTo(inertial);
        }

        const OrbitElements el = computeOrbitElements(p, v, inertial->body->mu);
        const bool grounded = el.periapsis <= inertial->body->radius + 3000.0;

        /* the cluster pose at park time, relative to its COM (cluster
           axes == old frame axes; rail_orient carries them into the
           inertial node and then holds inertially) */
        rail_rel_pos.resize(parts.size());
        rail_rel_rot.resize(parts.size());
        for(size_t i = 0; i < parts.size(); i++) {
            rail_rel_pos[i] = GetPosition(parts[i]) - com_frame;
            rail_rel_rot[i] = GetOrient(parts[i]);
        }

        if(grounded) {
            /* freeze: the pose is static in the rotating surface frame
               (its transforms already are), so the rail state just holds
               it; the planet's spin carries it via the render transform. */
            rail_pos = com_frame;
            rail_vel = vel_frame;
            rail_orient = glm::dmat3(1.0);
            railFrozen = true;
        } else {
            rail_pos = p;
            rail_vel = v;
            rail_orient = oldFrame->GetOrientRelTo(inertial);
            frame = inertial;   // on rails, ship->frame == its inertial node
            railFrozen = false;
        }

        /* out of the world: welds first (they reference the bodies) */
        for(size_t c = 0; c < constraints.size(); c++) {
            Detach(constraints[c]);
        }
        constraints.clear();
        for(auto&& part : parts) { RemoveBody(part); }

        onRails = true;
        if(!railFrozen) { writeRailPose(); }
        if(grounded) {
            printf("@@@ %s frozen on rails (grounded around %s)\n",
                   name.c_str(), frame->body->name.c_str());
        } else {
            printf("@@@ %s parked on rails around %s: sma=%.6g m ecc=%.4f\n",
                   name.c_str(), inertial->body->name.c_str(),
                   el.semi_major, el.ecc);
        }
        return true;
    }

    /* Re-enter physics from rails: rebuild the Bullet state from the rail
       state and hand the cluster back to the integrator. Pose and velocity
       already track the rail state (writeRailPose), so this is just
       re-register and re-weld. */
    void leaveRails() {
        if(!onRails) { return; }
        writeRailPose();
        for(auto&& part : parts) {
            AddPhysicsBody(part);
        }
        /* GlueTogether locks the CURRENT relative pose, which is exactly
           the parked geometry, so the stored anchors reproduce the welds. */
        for(size_t c = 0; c < constraintLinks.size(); c++) {
            constraints.push_back(GlueTogether(parts[constraintLinks[c].first],
                                               parts[constraintLinks[c].second],
                                               constraintAnchors[c].first,
                                               constraintAnchors[c].second));
        }
        NeverSleep(controller);
        onRails = false;
        railFrozen = false;
        printf("@@@ %s left the rails around %s\n",
               name.c_str(), frame->body->name.c_str());
    }

    /* Per-tick rail advance: propagate the conic by the tick's simulated
       duration (exact for any step size), check SOI boundaries, refresh
       the parked transforms. A frozen (grounded) ship has nothing to
       propagate: its pose is static in the rotating frame. */
    void railsTick(const double step) {
        if(!onRails || railFrozen) { return; }
        propagateKepler(rail_pos, rail_vel, frame->body->mu, step,
                        rail_pos, rail_vel);
        railsSwitchFrames();
        writeRailPose();
    }

    /* SOI bookkeeping for a railed ship (the switchFrames() analog): the
       rail conic is only valid around frame->body while the ship stays in
       that SOI. The rotating child frame is the same body -- never a
       switch candidate; physics ships drop into it after the handoff. */
    void railsSwitchFrames() {
        const double r = glm::length(rail_pos);
        if(r > frame->soi + 10000) {
            if(frame->parent != NULL) {
                printf("@@@ %s rails switching frame from %s to parent %s\n",
                       name.c_str(), frame->name.c_str(),
                       frame->parent->name.c_str());
                moveToRailFrame(frame->parent);
            }
        } else {
            for(auto&& child : frame->children) {
                if(child->body == frame->body) { continue; }
                // ship position in the child's coordinates (the same
                // transform GetPositionRelTo(part, child) applies)
                const glm::dvec3 rel = frame->GetOrientRelTo(child) * rail_pos
                                     + frame->GetPositionRelTo(child);
                const double dist = glm::length(rel);
                if(dist < child->soi - 10000) {
                    printf("@@@ %s rails switching frame from %s to child %s, distance: %.0f\n",
                           name.c_str(), frame->name.c_str(),
                           child->name.c_str(), dist);
                    moveToRailFrame(child);
                    break;
                }
            }
        }
    }

    /* Re-anchor the rail state on another frame (moveToFrame's math for
       the analytic state; the new frame is inertial, so no stasis). */
    void moveToRailFrame(Frame *newFrame) {
        const glm::dmat3 O = frame->GetOrientRelTo(newFrame);
        rail_vel = O * rail_vel + frame->GetVelocityRelTo(newFrame);
        rail_pos = O * rail_pos + frame->GetPositionRelTo(newFrame);
        rail_orient = O * rail_orient;
        frame = newFrame;
        m_parent = newFrame->body;
    }
};

/* Instantiate a ship def: one rigid body per part (mesh + texture from the
   catalog entry), welded parent-first in the def's construction order.
   GL is needed here (shader binding); the JSON parse/validate and the
   attach geometry (attachPose) are GL-free (shipdef.cpp). The catalog must
   outlive the ship (the partDefs point into it). */
static void build_ship(Vehicle *ship, const ShipDef &def, Shader *partsshader,
                       const glm::dvec3 &base, const glm::dmat3 &orient)
{
    printf("Building ship '%s' (%d parts)\n", def.name.c_str(), (int)def.parts.size());
    const size_t n = def.parts.size();

    /* 1) relative poses in a canonical frame: the root at the origin, +Z =
       the stack axis, each child welded to its (earlier) parent by the
       shared attachPose geometry (shipdef.cpp). */
    std::vector<glm::dvec3> pos(n);
    std::vector<glm::dmat3> rot(n);
    std::vector<glm::dvec3> pAnchor(n), cAnchor(n);
    pos[0] = glm::dvec3(0.0);
    rot[0] = glm::dmat3(1.0);
    for(size_t i = 1; i < n; i++) {
        const ShipPart &sp = def.parts[i];
        AttachPose ap = attachPose(pos[(size_t)sp.parent], rot[(size_t)sp.parent],
                                   *def.parts[(size_t)sp.parent].def, *sp.def,
                                   sp.attach, sp.angle, sp.offset);
        pos[i] = ap.childPos;
        rot[i] = ap.childRot;
        pAnchor[i] = ap.parentAnchor;
        cAnchor[i] = ap.childAnchor;
    }

    /* 2) the ship's lowest point along the stack axis: an axis-aligned part
       (down/side) spans h/2 about its center, a radial part spans its
       radius (its cross-section lies across the stack axis). */
    double lowest = 1e30;
    for(size_t i = 0; i < n; i++) {
        const ShipPart &sp = def.parts[i];
        double extent = (i > 0 && sp.attach == AttachMode::Radial)
                       ? sp.def->radius : sp.def->height / 2.0;
        lowest = std::min(lowest, pos[i].z - extent);
    }

    /* 3) place it on the pad: the lowest point at the pad top, lifted by
       the collision margins (terrain 0.5 + hull 0.1) so the inflated
       shapes just touch instead of popping apart on the first solve. For
       orbit scenarios this is only staging -- spawn_vehicle repositions. */
    const glm::dvec3 shift = glm::dvec3(0.0, 0.0, -lowest + 0.6);

    for(size_t i = 0; i < n; i++) {
        const PartDef &pd = *def.parts[i].def;

        Mesh *mesh = new Mesh;
        mesh->FromFile((std::string("./res/") + pd.mesh).c_str(), true);
        Texture *tex = load_texture((std::string("./res/") + pd.texture).c_str());
        Model *model = new Model;
        model->FromData(mesh, partsshader, tex);
        model->hull_margin = resolveHullMargin(def.hull_margin, pd.hull_margin);

        Body *part = create_body(model, 0, 0, 0, (float)pd.mass, false);
        setPosRot(part, base + orient * (pos[i] + shift), orient * rot[i]);

        if(i == 0) {
            ship->setRoot(part);
        } else {
            const ShipPart &sp = def.parts[i];
            ship->attach(part, (size_t)sp.parent, pAnchor[i], cAnchor[i]);
        }
        ship->partDefs.push_back(&pd);
        ship->partStages.push_back(def.parts[i].stage);
    }
    ship->controllerIndex = def.controllerIndex();
    ship->init();
}

// Resolve the reference frame that owns a world position
static Frame *resolve_frame_by_soi(Frame *root, glm::dvec3 worldPos) {
    Frame *cur = root;
    while(true) {
        Frame *best = NULL;
        double best_d = 1e30;
        for(Frame *c : cur->children) {
            double d = glm::length(worldPos - c->root_pos);
            if(d < c->soi && d < best_d) {
                best = c;
                best_d = d;
            }
        }
        if(best == NULL) { return cur; }
        cur = best;
    }
}

/*
  Starting scenarios (chosen at the CLI on startup, see main). The pad
  scenarios are already set up in main (the ship is built on the pad); the
  orbit scenarios place the ship in a circular orbit around the home body at
  r = radius + alt_frac * (rotating-frame SOI - radius), in the equatorial
  plane (local +Z) or the polar plane (local +Y), nose prograde.

  The ellipse-* scenarios place the ship on a 10 km x 1000 km ASL orbit in
  the equatorial plane, prograde in the same sense as the circular ones
  (periapsis along world +Z), at periapsis (ell_phase 0), apoapsis (1), or
  90 deg of true anomaly - halfway by angle between the apsides (2).

  As before, the ship's frame is resolved from the innermost SOI containing
  the spawn point (resolve_frame_by_soi), with the stasis-velocity correction
  so a rotating frame still yields the correct inertial orbital velocity.
*/
struct ScenarioDef {
    const char *name;
    bool on_pad;
    double alt_frac; // circular: fraction of (rot-frame SOI - radius)
    bool polar;
    int ell_phase;   // -1: circular; 0: at periapsis; 1: at apoapsis; 2: at 90 deg
    double peri_alt; // ellipse: periapsis altitude above the body radius (m)
    double apo_alt;  // ellipse: apoapsis altitude above the body radius (m)
};
static const ScenarioDef kScenarios[] = {
    {"pad",          true,  0.0,  false, -1, 0.0,     0.0},
    {"pad-polar",    true,  0.0,  true,  -1, 0.0,     0.0},
    {"rot-orbit",    false, 0.85, false, -1, 0.0,     0.0},
    {"inertial-orbit", false, 1.25, false, -1, 0.0,   0.0},
    {"high-orbit",   false, 5.0,  false, -1, 0.0,     0.0},
    {"high-polar",   false, 5.0,  true,  -1, 0.0,     0.0},
    {"ellipse-peri", false, 0.0,  false,  0, 10e3, 1000e3},
    {"ellipse-apo",  false, 0.0,  false,  1, 10e3, 1000e3},
    {"ellipse-mid",  false, 0.0,  false,  2, 10e3, 1000e3},
};

static const ScenarioDef *scenario_by_name(const std::string &name) {
    for(size_t i = 0; i < sizeof(kScenarios) / sizeof(kScenarios[0]); i++) {
        if(kScenarios[i].name == name) { return &kScenarios[i]; }
    }
    std::string avail;
    for(size_t i = 0; i < sizeof(kScenarios) / sizeof(kScenarios[0]); i++) {
        if(i) { avail += ", "; }
        avail += kScenarios[i].name;
    }
    throw std::runtime_error("fleet: unknown scenario '" + name
                             + "' (available: " + avail + ")");
}

// Orientation with the nose (local +Z) along `dir`; the roll axis is the
// coordinate axis most orthogonal to dir (never singular for a unit dir).
static glm::dmat3 faceAlong(const glm::dvec3 &dir)
{
    const glm::dvec3 z = glm::normalize(dir);
    const glm::dvec3 refs[3] = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
    int best = 0;
    for(int i = 1; i < 3; i++) {
        if(fabs(glm::dot(refs[i], z)) < fabs(glm::dot(refs[best], z))) best = i;
    }
    const glm::dvec3 x = glm::normalize(refs[best] - glm::dot(refs[best], z) * z);
    const glm::dvec3 y = glm::cross(z, x);
    return glm::dmat3(x, y, z);
}

/* slot_offset (m): lateral separation for ships sharing a scenario --
   applied along the orbit binormal (perpendicular to both the radius
   vector and the velocity), so each ship's orbit stays essentially the
   same shape. 0 for a lone ship (and no-op for pad scenarios). */
static void spawn_vehicle(Vehicle *ship, const ScenarioDef &sc, TerrainBody *home,
                          System &sys, double slot_offset = 0.0)
{
    if(sc.on_pad) { return; } // already on the pad, set up in main

    const glm::dvec3 center = home->frame->root_pos;
    glm::dvec3 shipWorldPos, velWorld;

    if(sc.ell_phase >= 0) {
        // Elliptical orbit in the equatorial plane (world X-Z), prograde in
        // the same sense as the circular scenarios: periapsis along +Z, so
        // 90 deg along the travel direction is +X. The apsides are inertial
        // (root-frame) directions, as orbital elements should be.
        const double rp = home->radius + sc.peri_alt;
        const double ra = home->radius + sc.apo_alt;
        const double p = 2.0 * rp * ra / (rp + ra); // semi-latus rectum a(1-e^2)
        const double e = (ra - rp) / (ra + rp);
        const double h = sqrt(home->mu * p);        // specific angular momentum
        const glm::dvec3 xhat = glm::dvec3(1, 0, 0);
        const glm::dvec3 zhat = glm::dvec3(0, 0, 1); // periapsis direction
        if(sc.ell_phase == 0) { // at periapsis
            shipWorldPos = center + zhat * rp;
            velWorld = xhat * (h / rp);
        } else if(sc.ell_phase == 1) { // at apoapsis
            shipWorldPos = center - zhat * ra;
            velWorld = -xhat * (h / ra);
        } else { // 90 deg true anomaly (halfway by angle between the apsides)
            shipWorldPos = center + xhat * p;
            velWorld = (xhat * e - zhat) * (h / p);
        }
    } else {
        // Circular orbit around the home body: radius measured from its frame origin.
        const double r = home->radius + sc.alt_frac * (home->rot_frame->soi - home->radius);
        const glm::dvec3 rhat_local = sc.polar ? glm::dvec3(0, 1, 0) : glm::dvec3(0, 0, 1);
        shipWorldPos = center + home->frame->root_orient * (rhat_local * r);

        // Circular orbital speed (vis-viva with semi-major axis == r).
        const double speed = sqrt(home->mu / r);

        // Prograde: perpendicular to the radius vector, in the system's sense of
        // rotation (+y axis); polar orbits go around the spin axis instead.
        // Normalize: with an inclined body orbit rhat is not orthogonal to
        // the reference axis, and the raw cross product is short by
        // cos(incl) -- the spawn would arrive below circular speed, at the
        // apoapsis of an e = sin^2(incl) ellipse.
        const glm::dvec3 rhat = glm::normalize(shipWorldPos - center);
        const glm::dvec3 vhat = glm::normalize(
            sc.polar ? glm::cross(glm::dvec3(1, 0, 0), rhat)
                     : glm::cross(glm::dvec3(0, 1, 0), rhat));
        velWorld = speed * vhat;
    }

    if(slot_offset != 0.0) {
        const glm::dvec3 rhat = glm::normalize(shipWorldPos - center);
        const glm::dvec3 vhat = glm::normalize(velWorld);
        shipWorldPos += glm::normalize(glm::cross(rhat, vhat)) * slot_offset;
    }

    Frame *frame = resolve_frame_by_soi(sys.star->frame, shipWorldPos);

    // Express the spawn position and velocity in the resolved frame's local
    // coordinates: v = R^T * velWorld - stasis(p) (inertial vel =
    // R*(v + stasis(p)) + V, with V == body velocity since the frame origin
    // sits on the body).
    const glm::dvec3 target = glm::transpose(frame->root_orient) * (shipWorldPos - frame->root_pos);
    const glm::dvec3 vel = glm::transpose(frame->root_orient) * velWorld
                          - frame->GetStasisVelocity(target);

    if(frame != ship->frame) {
        ship->moveToFrame(frame);
    }

    // Nose (local +Z) along prograde: rigidly re-orient the whole ship.
    // Part 0 (the root) takes `orient`; every other part gets the same
    // world rotation (Rrel) about the ship's COM, so its RELATIVE geometry
    // survives -- a stacked part stays stacked, a radial part keeps its
    // perpendicular axis. (The old loop applied `orient` to every part,
    // which silently straightened a radial part into the stack axis.)
    const glm::dmat3 orient = faceAlong(velWorld);
    const glm::dvec3 com0 = ship->get_center_of_mass();
    const glm::dmat3 Rrel = orient * glm::transpose(GetOrient(ship->parts[0]));
    for(auto&& part : ship->parts) {
        const glm::dvec3 p = GetPosition(part);
        const glm::dmat3 R0 = GetOrient(part);
        setPosRot(part, target + Rrel * (p - com0), Rrel * R0);
        SetVelocity(part, vel);
    }

    printf("Spawn '%s' around %s: frame '%s' @ world (%.0f, %.0f, %.0f), r = %.0f m, |v| = %.1f m/s\n",
           sc.name, home->name.c_str(), frame->name.c_str(),
           shipWorldPos.x, shipWorldPos.y, shipWorldPos.z,
           glm::length(shipWorldPos - center), glm::length(velWorld));
}

/* --radial-test spin diagnostics (two-part ship): the per-part angular
   velocities (if they differ, the weld is not holding a rigid body), the
   INTERNAL contact torque between the two parts -- the only way a passive
   welded pair can spin itself -- and the tidal (differential gravity)
   torque, which is the one legitimate external torque and should be
   negligible at ship scale. */
static void spin_log(Vehicle *ship, double time) {
    if(ship->parts.size() < 2) { return; }

    const glm::dvec3 com = ship->get_center_of_mass();
    printf("[spin] t=%.2fs ship=%s com=[%.0f %.0f %.0f] parts=%zu\n",
           time, ship->name.c_str(), com.x, com.y, com.z, ship->parts.size());
    for(size_t i = 0; i < ship->parts.size(); i++) {
        const glm::dvec3 w = GetAngVelocity(ship->parts[i]);
        const glm::dvec3 p = GetPosition(ship->parts[i]);
        printf("[spin]   %-14s pos=[%.1f %.1f %.1f] w=[%.3e %.3e %.3e] |w|=%.3e\n",
               ship->partDefs[i]->name.c_str(),
               p.x, p.y, p.z, w.x, w.y, w.z, glm::length(w));
    }

    for(size_t i = 0; i < ship->parts.size(); i++) {
        for(size_t j = i + 1; j < ship->parts.size(); j++) {
            const ContactPairInfo cp = contact_report(ship->parts[i], ship->parts[j]);
            printf("[spin]   contact %-8s-%-8s: manifs=%d (other=%d) pts=%zu |F|=%.3e |T|=%.3e maxImp=%.3e\n",
                   ship->partDefs[i]->name.c_str(), ship->partDefs[j]->name.c_str(),
                   cp.manifolds, cp.otherManifolds, cp.points.size(),
                   glm::length(cp.netForce), glm::length(cp.netTorque), cp.maxImpulse);
            for(size_t k = 0; k < cp.points.size(); k++) {
                const ContactPointInfo &p = cp.points[k];
                printf("[spin]     pt%zu pos=[%.1f %.1f %.1f] pen=%.4f imp=[%.3e %.3e %.3e] |imp|=%.3e\n",
                       k, p.pos.x, p.pos.y, p.pos.z, p.pen,
                       p.impulse.x, p.impulse.y, p.impulse.z, glm::length(p.impulse));
            }
        }
    }

    const double G = 6.674e-11;
    const double M = ship->m_parent->mass;
    glm::dvec3 tau(0, 0, 0);
    for(size_t i = 0; i < ship->parts.size(); i++) {
        const glm::dvec3 p = GetPosition(ship->parts[i]);
        const double r = glm::length(p);
        const glm::dvec3 F = -G * M * ship->parts[i]->mass * p / (r * r * r);
        tau += glm::cross(p - com, F);
    }
    printf("[spin]   tidal gravity torque |tau|=%.3e\n", glm::length(tau));
    fflush(stdout);
}

class StaticBuilding {
public:
    TerrainBody *parent;
    TerrainBody *sun = nullptr; // the star (light source); set in main
    Body *body;

    void Draw(const Camera* camera, const TerrainBody *current, Frame *renderFrame) {
        if(current == parent) {
            const Frame *posFrame = parent->frame->getRotFrame();
            const float shadow = ComputeTerrainShadow(parent, posFrame,
                                                      GetPosition(body), sun);
            glm::vec3 sunlightVec = glm::vec3(TerrainBody::SunlightDir(parent, sun, renderFrame));
            body->Draw(camera, sunlightVec, shadow);
        }
    }
};

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

float TerrainBody::GetTerrainHeight(const glm::vec3& sphere_p) {
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

float TerrainBody::GetTerrainHeightUnscaled(const glm::vec3& sphere_p) {
    const Surface &s = surface;
    float noise = noise3d(sphere_p * s.frequency + s.seed_offset,
                          s.octaves, s.persistence) * s.amplitude;

    if(s.has_sea && noise < s.sea_level) {
        noise = s.sea_level;
    }

    return radius + noise;
}

float TerrainBody::ScaleHeightNoise(float noise) {
    constexpr float ref_height = 3000.0; // guess

    // rescale noise by altitude (sign-safe for fractional powers)
    float sign = noise < 0 ? -1.0f : 1.0f;
    float n = sign * noise;
    n *= pow(n / ref_height, surface.power);
    return sign * n;
}

float ComputeTerrainShadow(TerrainBody *planet, const Frame *posFrame,
                           const glm::dvec3 &posInFrame, TerrainBody *sun) {
    // Approximate terrain shadow for one point (a ship part / the space port):
    // cast a ray from the point toward the sun and test it against the
    // planet's terrain height function, which is analytic and therefore
    // available everywhere (not just where collision leaves exist).
    // Approximate by design: one test point per object, hard lit/shadow.

    if(sun == nullptr) { return 1.0f; }

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

inline COLOUR GetColourMoon(float v, float vmin, float vmax) {
    return { 0.5, 0.5, 0.5 };
}

inline COLOUR GetColourSun(float v, float vmin, float vmax) {
    return { 1.0, 1.0, 0.0 };
}

inline COLOUR GetColourEarth(float v, float vmin, float vmax)
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

            if (surface.bands) {
                // gas giant: smooth sphere, color by latitude band
                COLOUR cb = surface.BandColor(sphere_p);
                glm::vec3 color = glm::vec3(cb.r, cb.g, cb.b);
                float brightness = (cb.r + cb.g + cb.b) / 6;
                color = float(0.5) * color + glm::vec3(brightness,
                                                       brightness,
                                                       brightness);
                vertices[(j + off) + edge * (i + off)] = PosNorColVertex(sphere_p * radius,
                                                         sphere_p,
                                                         color);
                continue;
            }

            float height = GetTerrainHeightUnscaled(sphere_p);

            // add some color noise
            float color_noise = noise3d(sphere_p * radius + surface.seed_offset, 1, 0.60) * 100;
            float h = height + ((color_noise / 2) - color_noise);

            COLOUR c;
            if(surface.palette.empty()) {
                // no palette in the JSON: fall back to the type-based default
                c = (*colour_func)(h, radius - 1, radius + 3000);
            } else {
                float t = (h - (radius + surface.sea_level)) / surface.max_height;
                c = surface.PaletteColor(t);
            }

            // set the color based on unscaled noise for better gradient
            glm::vec3 color = glm::vec3(c.r, c.g, c.b);
            float brightness = (c.r + c.g + c.b) / 6;

            // lower contrast, increase brightness
            color = float(0.5) * color + glm::vec3(brightness,
                                                   brightness,
                                                   brightness);

            if(surface.has_sea && height <= radius + surface.sea_level) {
                color = surface.sea_color;
            }

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
    const int lat = 32, lon = 32;
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

glm::dvec3 projectVecOntoPlane(const glm::dvec3 & vec, const glm::dvec3 & normal) {
    return vec - glm::dot(vec, normal) * normal;
}

// Circular buffer of (sim time, value) samples for the telemetry plots.
// Fixed size and preallocated, so sampling in the render loop never
// allocates; once full, push() overwrites the oldest sample.
struct TimeSeries {
    enum { N = 8192 };
    double t[N];
    double v[N];
    double st_t[N];  // staging copies, filled oldest-first by stage()
    double st_v[N];
    int head = 0;    // next write slot
    int count = 0;   // samples stored (capped at N)
    void push(double tt, double vv) {
        const int last = (head - 1 + N) % N;
        if(count > 0 && t[last] == tt) {
            // sim time didn't advance (paused): refresh the last sample
            v[last] = vv;
            return;
        }
        t[head] = tt;
        v[head] = vv;
        head = (head + 1) % N;
        if(count < N) { count++; }
    }
    // Copies the samples oldest-first into the staging buffers (a wrapped
    // ring is not contiguous) and returns their count.
    int stage() {
        const int start = (head - count + N) % N;
        for(int i = 0; i < count; i++) {
            const int idx = (start + i) % N;
            st_t[i] = t[idx];
            st_v[i] = v[idx];
        }
        return count;
    }
    const double *t_arr() const { return st_t; }
    const double *v_arr() const { return st_v; }
};

/* --sim-press: synthetic key input for e2e testing.
   One entry = one key press: down `down_ms` after the main loop starts,
   up `up_ms` after. It feeds two input channels:
   - one-shot actions (SPACE, TAB, G, C, ...) fire from the synthetic
     SDL_KEYDOWN event SDL_PushEvent()ed into the queue by the loop;
   - held commands (WASDQE, I, X, B, N, R, F, ESC) read
     SDL_GetKeyboardState(), which SDL_PushEvent does NOT update (verified
     on this system's SDL 2.32), so the loop additionally ORs in each
     entry's down_sent..up_sent window (see isDown below).
   Keys are given by SDL key name (SPACE, A, F11, ...) or decimal SDL
   keycode (32, 105, 1073741911). */
struct SimKeyPress {
    Uint32 down_ms;
    Uint32 up_ms;
    SDL_Keycode key;
    SDL_Scancode sc;
    bool down_sent;
    bool up_sent;
};

/* --sim-mouse: synthetic mouse input for e2e testing.
   One entry = one mouse action: it starts `time_ms` after the main loop
   starts. Semantics by (button, duration):
   - button != 0 && duration > 0  -> a DRAG: press the button, then move
     the cursor to (x,y); release at time_ms + duration. Used to orbit the
     camera (RMB) -- the look code reads the motion delta, and the button
     must be down before the motion event.
   - button != 0 && duration == 0 -> a CLICK: move the cursor to (x,y),
     then press + release in place (same frame). Used to click UI buttons.
   - button == 0                  -> a MOVE: just reposition the cursor.
   (x,y) are absolute window pixels; the emitted MOUSEMOTION carries the
   delta from the previous simulated position, which is what the camera
   consumes (yaw = -dx/200 rad, pitch = +dy/200 rad; 200px ~= 1 rad). */
struct SimMouseAction {
    Uint32 time_ms;   // when the action starts (after the loop starts)
    Uint32 up_ms;     // time_ms + duration; the button release time
    int x;            // target position, window pixels
    int y;
    Uint8 button;     // SDL button code (1=LEFT,2=MIDDLE,3=RIGHT); 0 = move only
    bool started;     // start events (button-down + motion) already emitted
    bool released;    // button-up already emitted
};

static SDL_Keycode sim_parse_key(const std::string &s) {
    // decimal SDL keycode, e.g. 32 = SPACE
    if(!s.empty()) {
        char *end = nullptr;
        const unsigned long v = strtoul(s.c_str(), &end, 10);
        if(end != s.c_str() && *end == '\0') {
            return (SDL_Keycode)v;
        }
    }
    // This SDL header defines no uppercase letter aliases (SDLK_a..z only).
    static const std::map<std::string, SDL_Keycode> names = {
        {"A", SDLK_a}, {"B", SDLK_b}, {"C", SDLK_c}, {"D", SDLK_d},
        {"E", SDLK_e}, {"F", SDLK_f}, {"G", SDLK_g}, {"H", SDLK_h},
        {"I", SDLK_i}, {"J", SDLK_j}, {"K", SDLK_k}, {"L", SDLK_l},
        {"M", SDLK_m}, {"N", SDLK_n}, {"O", SDLK_o}, {"P", SDLK_p},
        {"Q", SDLK_q}, {"R", SDLK_r}, {"S", SDLK_s}, {"T", SDLK_t},
        {"U", SDLK_u}, {"V", SDLK_v}, {"W", SDLK_w}, {"X", SDLK_x},
        {"Y", SDLK_y}, {"Z", SDLK_z},
        {"SPACE", SDLK_SPACE}, {"TAB", SDLK_TAB},
        {"RETURN", SDLK_RETURN}, {"ENTER", SDLK_RETURN},
        {"ESCAPE", SDLK_ESCAPE},
        {"PERIOD", SDLK_PERIOD}, {"COMMA", SDLK_COMMA},
        {"LSHIFT", SDLK_LSHIFT}, {"RSHIFT", SDLK_RSHIFT},
        {"LCTRL", SDLK_LCTRL}, {"RCTRL", SDLK_RCTRL},
        {"LEFT", SDLK_LEFT}, {"RIGHT", SDLK_RIGHT},
        {"UP", SDLK_UP}, {"DOWN", SDLK_DOWN},
        {"F1", SDLK_F1}, {"F2", SDLK_F2}, {"F3", SDLK_F3}, {"F4", SDLK_F4},
        {"F5", SDLK_F5}, {"F6", SDLK_F6}, {"F7", SDLK_F7}, {"F8", SDLK_F8},
        {"F9", SDLK_F9}, {"F10", SDLK_F10}, {"F11", SDLK_F11},
        {"F12", SDLK_F12},
    };
    std::string up;
    up.reserve(s.size());
    for(size_t i = 0; i < s.size(); i++) {
        up.push_back((char)toupper((unsigned char)s[i]));
    }
    std::map<std::string, SDL_Keycode>::const_iterator it = names.find(up);
    if(it != names.end()) {
        return it->second;
    }
    return 0; // unknown
}

static int sim_parse_button(const std::string &s) {
    // decimal SDL button code (1 = LEFT, 2 = MIDDLE, 3 = RIGHT; 0 = none),
    // or a name for readability.
    if(!s.empty()) {
        char *end = nullptr;
        const unsigned long v = strtoul(s.c_str(), &end, 10);
        if(end != s.c_str() && *end == '\0') {
            return (int)v;
        }
    }
    std::string up;
    up.reserve(s.size());
    for(size_t i = 0; i < s.size(); i++) {
        up.push_back((char)toupper((unsigned char)s[i]));
    }
    static const std::map<std::string, int> names = {
        {"L", 1}, {"LEFT", 1}, {"LMB", 1},
        {"M", 2}, {"MIDDLE", 2}, {"MMB", 2},
        {"R", 3}, {"RIGHT", 3}, {"RMB", 3},
        {"NONE", 0},
    };
    std::map<std::string, int>::const_iterator it = names.find(up);
    if(it != names.end()) {
        return it->second;
    }
    return -1; // unknown
}

int main(int argc, char **argv)
{
    const auto prog_start = std::chrono::steady_clock::now();

    CLI::App app{"Open Space Program"};

    std::string body_name;
    app.add_option("--body", body_name,
        "Body the ship starts on / orbits (default: the system's home body)");

    std::string scenario = "pad";
    app.add_option("--scenario", scenario,
        "Starting scenario: pad, pad-polar, rot-orbit, inertial-orbit, "
        "high-orbit, high-polar, ellipse-peri, ellipse-apo, ellipse-mid "
        "(the ellipse-* scenarios are a 10x1000 km ASL orbit started at "
        "periapsis, apoapsis, or halfway by angle between them; default: pad)")
        ->check(CLI::IsMember({"pad", "pad-polar", "rot-orbit",
                               "inertial-orbit", "high-orbit", "high-polar",
                               "ellipse-peri", "ellipse-apo", "ellipse-mid"}));

    std::string system_file = "ksp_system.json";
    app.add_option("--system", system_file,
                   "Star-system JSON file to load (default: ksp_system.json; "
                   "try system.json for the Eerbon system)");

    std::string parts_file = "res/parts.json";
    app.add_option("--parts", parts_file,
                   "Parts catalog JSON (default: res/parts.json)");

    std::vector<std::string> ship_files;
    app.add_option("--ship", ship_files,
                   "Ship def JSON to build; repeat the flag to build more "
                   "ships (they share the body/scenario, each getting its "
                   "own pad slot / orbit slot). A uniform-fleet shorthand "
                   "-- --fleet overrides it. Default: res/ships/racer.json");

    std::string fleet_file;
    app.add_option("--fleet", fleet_file,
                   "Fleet JSON (default: none; then --ship applies). One "
                   "entry per ship, each with its own ship def, name, body "
                   "and scenario; omitted body/scenario fall back to "
                   "--body/--scenario. Ships sharing a body+scenario get "
                   "their own pad slot / orbit slot. Try res/fleet.json");

    /* Spin-instrumentation mode: build a test ship (no JSON ship def)
       and log its spin + the internal contact torque each tick.
       radial     = part B welded to part A's side, axes PERPENDICULAR
       parallel   = part B welded to part A's side, axes PARALLEL
                    (side by side, off-axis anchor)
       stacked    = part B welded on A's axis (known-good baseline)
       stacks     = two 2-part stacks side by side, 2nd stack PERPENDICULAR
       parstacks  = two 2-part stacks side by side, ALL axes PARALLEL
       All parts are passive tanks (no wheels/thrusters), so any spin
       is self-inflicted. */
    std::string radial_test;
    app.add_option("--radial-test", radial_test,
                   "Build the spin-test ship(s) instead of a fleet: "
                   "radial | parallel | stacked | stacks | parstacks")
        ->check(CLI::IsMember({"radial", "parallel", "stacked", "stacks",
                               "parstacks"}));

    int initial_time_accel = 0;
    app.add_option("-t,--time-accel", initial_time_accel,
                   "Initial time acceleration (0 = paused, default 0)")
        ->check(CLI::NonNegativeNumber);

    double timeout_seconds = 0.0;
    app.add_option("--timeout", timeout_seconds,
                   "Auto-exit the main loop after this many wall-clock "
                   "seconds (0 = run until closed; default: 0)")
        ->check(CLI::NonNegativeNumber);

    std::vector<std::string> sim_press;
    app.add_option("--sim-press", sim_press,
                   "Synthetic keypresses for e2e testing: a flat list of "
                   "START_MS,DURATION_MS,KEY triples (e.g. 500,200,SPACE, "
                   "1500,100,I; spaces also separate values). KEY is an SDL "
                   "key name (A..Z, SPACE, TAB, F1-F12, ...) or a decimal "
                   "SDL keycode. The key is pressed START_MS after the main "
                   "loop starts and held for DURATION_MS. Repeat the flag "
                   "to append more triples.")
        ->delimiter(',');

    std::vector<std::string> sim_mouse;
    app.add_option("--sim-mouse", sim_mouse,
                   "Synthetic mouse input for e2e testing: a flat list of "
                   "TIME_MS,DURATION_MS,X,Y,BTN quintuples (e.g. "
                   "500,0,400,300,1 = click LMB at (400,300) after 500ms; "
                   "500,600,900,500,RMB = RMB-drag to (900,500) over 600ms "
                   "to orbit the camera; spaces also separate values). X,Y "
                   "are absolute window pixels (the cursor moves there; the "
                   "delta from the previous position drives the camera look: "
                   "yaw = -dx/200 rad, pitch = +dy/200 rad, 200px ~= 1 rad). "
                   "BTN is an SDL button code (1=LEFT, 2=MIDDLE, 3=RIGHT) or "
                   "name (L/LEFT/LMB, M/MIDDLE/MMB, R/RIGHT/RMB); 0/NONE = "
                   "move only. DURATION_MS>0 with a button = a drag (held); "
                   "0 = a quick click. Repeat the flag to append more "
                   "quintuples.")
        ->delimiter(',');

    bool selftest_stage = false;
    app.add_flag("--selftest-stage", selftest_stage,
                 "Exercise the staging path on the first multi-stage ship "
                 "(drop the active stage, then refuse the last stage), "
                 "then exit after a few physics ticks");

    bool selftest_rails = false;
    app.add_flag("--selftest-rails", selftest_rails,
                 "Exercise the idle-ship rails path: coast the first "
                 "railed ship 30 ticks (conic must be conserved), hand "
                 "control to it (rails -> physics handoff), then exit "
                 "after 60 physics ticks");

    bool orbit_log = false;
    app.add_flag("--orbit-log", orbit_log,
                 "Periodically print the ship's orbital elements to stdout "
                 "(for measuring orbital stability)");

    double orbit_interval = 1.0;
    app.add_option("--orbit-interval", orbit_interval,
                   "Wall-clock seconds between --orbit-log lines (default: 1)")
        ->check(CLI::PositiveNumber);

    bool dbg_log = false;
    app.add_flag("--dbg-log", dbg_log,
                 "Periodically print ship position/altitude/velocity "
                 "(surface-level companion to --orbit-log)");

    bool spin_log_enabled = false;
    app.add_flag("--spin-log", spin_log_enabled,
                 "Periodically print the ship's spin diagnostics (per-part "
                 "angular velocities, inter-part contact impulses, tidal "
                 "torque) to stdout; also implied by --radial-test");

    std::vector<std::string> postfx_spec;
    app.add_option("--postfx", postfx_spec,
                   "Post-processing effect, in the order given; repeatable "
                   "and/or comma-separated (e.g. --postfx cas,grain). "
                   "Available: crt (retro tube look), grain (animated film "
                   "grain), cas (adaptive-contrast sharpening, 'sharpen' "
                   "also accepted). Omit for direct output (default)");

    bool gl_debug = false;
    app.add_flag("--gl-debug", gl_debug,
                 "Enable the OpenGL debug output callback (GL_DEBUG_* "
                 "messages print as they occur)");

    int screen_width = 1920;
    app.add_option("--width", screen_width,
                   "Window width in pixels (used with --borderless and "
                   "--exclusive; ignored with --fullscreen)")
        ->check(CLI::PositiveNumber);
    int screen_height = 1080;
    app.add_option("--height", screen_height,
                   "Window height in pixels (used with --borderless and "
                   "--exclusive; ignored with --fullscreen)")
        ->check(CLI::PositiveNumber);
    bool fullscreen = false;
    auto fs_opt = app.add_flag("--fullscreen", fullscreen,
                               "Start in borderless fullscreen at the "
                               "display's native resolution");
    bool borderless = false;
    auto bl_opt = app.add_flag("--borderless", borderless,
                               "Start as a borderless window (no title bar) "
                               "at --width/--height");
    bool exclusive = false;
    auto ex_opt = app.add_flag("--exclusive", exclusive,
                               "Exclusive fullscreen: change the display "
                               "mode to --width/--height (low latency, the "
                               "only way to go non-native on X11). Note: "
                               "SDL 2.32's X11 driver never restores the "
                               "previous mode on exit (X11_QuitModes is a "
                               "no-op), so restore it yourself with xrandr "
                               "if it matters");
    fs_opt->excludes(bl_opt);
    fs_opt->excludes(ex_opt);
    bl_opt->excludes(ex_opt);

    float font_size = 14.0f;
    app.add_option("--font-size", font_size,
                   "UI font size in pixels (the big HUD readout font is "
                   "twice this; default 14)")
        ->check(CLI::PositiveNumber);

    int frame_cap = 60;
    app.add_option("--frame-cap", frame_cap,
                   "Max render frames per second (0 = uncapped; default 60). "
                   "Without a cap the loop busy-spins between vsyncs, "
                   "idling a CPU core at 100% even while paused")
        ->check(CLI::NonNegativeNumber);

    // it's like a google maps link
    std::vector<double> free_cam_pos;
    app.add_option("--free-cam-pos", free_cam_pos,
                   "Start in the free camera at this world position: X Y Z "
                   "(ship-frame coordinates)")
        ->expected(3);

    std::vector<double> free_cam_fwd;
    app.add_option("--free-cam-fwd", free_cam_fwd,
                   "Initial free camera forward direction: X Y Z "
                   "(normalised)")
        ->expected(3);

    std::vector<double> free_cam_up;
    app.add_option("--free-cam-up", free_cam_up,
                   "Initial free camera up direction: X Y Z (default: 0 1 0)")
        ->expected(3);

    try {
        CLI11_PARSE(app, argc, argv);
    } catch(const CLI::ParseError &e) {
        return app.exit(e);
    }

    /* --sim-press: fold the flat START_MS,DURATION_MS,KEY list into press
       entries. */
    std::vector<SimKeyPress> sim_presses;
    if(!sim_press.empty()) {
        if(sim_press.size() % 3 != 0) {
            printf("error: --sim-press expects START_MS,DURATION_MS,KEY "
                   "triples; got %zu value(s)\n", sim_press.size());
            return 1;
        }
        for(size_t i = 0; i < sim_press.size(); i += 3) {
            char *end = nullptr;
            const unsigned long t = strtoul(sim_press[i].c_str(), &end, 10);
            if(end == sim_press[i].c_str() || *end != '\0') {
                printf("error: --sim-press start time '%s' is not an "
                       "integer ms\n", sim_press[i].c_str());
                return 1;
            }
            const unsigned long d =
                strtoul(sim_press[i + 1].c_str(), &end, 10);
            if(end == sim_press[i + 1].c_str() || *end != '\0') {
                printf("error: --sim-press duration '%s' is not an "
                       "integer ms\n", sim_press[i + 1].c_str());
                return 1;
            }
            const SDL_Keycode k = sim_parse_key(sim_press[i + 2]);
            if(k == 0) {
                printf("error: --sim-press key '%s' is not a known SDL "
                       "keycode or name\n", sim_press[i + 2].c_str());
                return 1;
            }
            SimKeyPress p;
            p.down_ms = (Uint32)t;
            p.up_ms = (Uint32)t + (Uint32)d;
            p.key = k;
            p.sc = SDL_SCANCODE_UNKNOWN; // resolved after SDL_Init (see below)
            p.down_sent = false;
            p.up_sent = false;
            sim_presses.push_back(p);
        }
    }

    /* --sim-mouse: fold the flat TIME_MS,DURATION_MS,X,Y,BTN list into
       actions. X,Y are signed (the cursor can move up/left from where it
       was), so they parse as strtol, unlike the unsigned times above. */
    std::vector<SimMouseAction> sim_mouse_actions;
    // Simulated cursor position (window pixels); each action's motion
    // carries the delta from here, which is what the camera look consumes.
    int sim_mouse_x = 0, sim_mouse_y = 0;
    if(!sim_mouse.empty()) {
        if(sim_mouse.size() % 5 != 0) {
            printf("error: --sim-mouse expects TIME_MS,DURATION_MS,X,Y,BTN "
                   "quintuples; got %zu value(s)\n", sim_mouse.size());
            return 1;
        }
        for(size_t i = 0; i < sim_mouse.size(); i += 5) {
            char *end = nullptr;
            unsigned long v;
            v = strtoul(sim_mouse[i].c_str(), &end, 10);
            if(end == sim_mouse[i].c_str() || *end != '\0') {
                printf("error: --sim-mouse time '%s' is not an "
                       "integer ms\n", sim_mouse[i].c_str());
                return 1;
            }
            const unsigned long t = v;
            v = strtoul(sim_mouse[i + 1].c_str(), &end, 10);
            if(end == sim_mouse[i + 1].c_str() || *end != '\0') {
                printf("error: --sim-mouse duration '%s' is not an "
                       "integer ms\n", sim_mouse[i + 1].c_str());
                return 1;
            }
            const unsigned long d = v;
            v = (unsigned long)strtol(sim_mouse[i + 2].c_str(), &end, 10);
            if(end == sim_mouse[i + 2].c_str() || *end != '\0') {
                printf("error: --sim-mouse X '%s' is not an "
                       "integer pixel\n", sim_mouse[i + 2].c_str());
                return 1;
            }
            const int x = (int)v;
            v = (unsigned long)strtol(sim_mouse[i + 3].c_str(), &end, 10);
            if(end == sim_mouse[i + 3].c_str() || *end != '\0') {
                printf("error: --sim-mouse Y '%s' is not an "
                       "integer pixel\n", sim_mouse[i + 3].c_str());
                return 1;
            }
            const int y = (int)v;
            const int b = sim_parse_button(sim_mouse[i + 4]);
            if(b < 0) {
                printf("error: --sim-mouse button '%s' is not a known SDL "
                       "button code or name (0=none, 1=LEFT, 2=MIDDLE, "
                       "3=RIGHT)\n", sim_mouse[i + 4].c_str());
                return 1;
            }
            SimMouseAction a;
            a.time_ms = (Uint32)t;
            a.up_ms = (Uint32)t + (Uint32)d;
            a.x = x;
            a.y = y;
            a.button = (Uint8)b;
            a.started = false;
            a.released = false;
            sim_mouse_actions.push_back(a);
        }
    }

    // Any of the --free-cam-* options opts in to starting in free-cam mode.
    const bool use_free_cam = !free_cam_pos.empty() || !free_cam_fwd.empty()
                            || !free_cam_up.empty();

    const WindowMode window_mode =
        exclusive  ? WindowMode::Exclusive
        : fullscreen ? WindowMode::Fullscreen
        : borderless ? WindowMode::Borderless
                     : WindowMode::Windowed;
    Renderer display(screen_width, screen_height, window_mode, gl_debug);
    check_gl_error();
    const Uint32 sim_win_id = SDL_GetWindowID(display.get_display());
    /* --sim-press: resolve keycodes to scancodes now that SDL is initialized
       (SDL_GetScancodeFromKey needs SDL_Init; the CLI parse ran before the
       Renderer above created the video subsystem). */
    for(auto &p : sim_presses) {
        p.sc = SDL_GetScancodeFromKey(p.key);
        if(p.sc == SDL_SCANCODE_UNKNOWN) {
            printf("warning: --sim-press key %d has no scancode in the "
                   "current keyboard layout: one-shot actions fire, held "
                   "commands for it do not\n", (int)p.key);
        }
    }
    ImGuiContext* ctx1 = ImGui::CreateContext();
    ImGui::SetCurrentContext(ctx1);
    // ImPlot keeps its own state per imgui context (v1.0 requires an
    // explicit context; it is bound to the current one at creation).
    ImPlot::CreateContext();
    ImGui_ImplSDL2_InitForOpenGL(display.get_display(), SDL_GL_GetCurrentContext());
    ImGui_ImplOpenGL3_Init("#version 430");
    check_gl_error();

    ImGuiIO& io = ImGui::GetIO();
    // No imgui.ini: window layout must not survive between runs or clobber
    // the layout the code sets up each frame.
    io.IniFilename = nullptr;
    io.Fonts->AddFontFromFileTTF("./res/DejaVuSansMono.ttf", font_size);
    bigger = io.Fonts->AddFontFromFileTTF("./res/DroidSans.ttf", 2.0f * font_size);
    check_gl_error();

    // start bullet; see physics.cpp
    void create_physics(void);
    create_physics();
    check_gl_error();

    /* data init */
    Shader *partsshader = new Shader;
    partsshader->registerAttribs({ "position", "uv", "normal" });
    partsshader->registerUniforms({ "MVP", "Normal", "lightDirection", "shadow" });
    partsshader->FromFile("./res/partsShader");

    Shader *terrainshader = new Shader;
    terrainshader->registerAttribs({ "position", "normal", "color" });
    terrainshader->registerUniforms({ "MVP", "Normal", "lightDirection", "color" });
    terrainshader->FromFile("./res/terrainShader");

    Shader *sunshader = new Shader;
    sunshader->registerAttribs({ "position", "normal", "color" });
    sunshader->registerUniforms({ "MVP", "Normal", "lightDirection", "color" });
    sunshader->FromFile("./res/sunShader");

    // Atmosphere rim shell (Fresnel limb glow). See reports/atmosphere2026_08_25.
    Shader *atmosphereshader = new Shader;
    atmosphereshader->registerAttribs({ "position", "normal" });
    atmosphereshader->registerUniforms({ "MVP", "Normal", "cameraPos",
                                         "color", "intensity", "power",
                                         "lightDirection" });
    atmosphereshader->FromFile("./res/atmosphereShader");

    Shader *skyboxshader = new Shader;
    skyboxshader->registerAttribs({ "position" });
    skyboxshader->registerUniforms({ "projectionview" });
    skyboxshader->FromFile("./res/skyboxShader");

    Shader *lineshader = new Shader;
    lineshader->registerAttribs({ "position" });
    lineshader->registerUniforms({ "MVP", "color" });
    lineshader->FromFile("./res/lineShader2");

    PostFX *postfx = new PostFX;
    // Each --postfx value may itself be comma-separated, so both
    // --postfx crt,grain and --postfx crt --postfx grain work.
    std::vector<std::string> fx_names;
    for(const std::string &spec : postfx_spec) {
        size_t start = 0;
        while(start <= spec.size()) {
            size_t comma = spec.find(',', start);
            std::string name = spec.substr(start, comma == std::string::npos
                                           ? std::string::npos
                                           : comma - start);
            size_t b = name.find_first_not_of(" \t");
            size_t e = name.find_last_not_of(" \t");
            name = (b == std::string::npos) ? "" : name.substr(b, e - b + 1);
            if(!name.empty()) {
                if(!postfx->AddEffect(name)) {
                    printf("error: unknown --postfx effect '%s' (available: ",
                           name.c_str());
                    const std::vector<std::string> &avail = PostFX::Available();
                    for(size_t i = 0; i < avail.size(); i++) {
                        printf("%s%s", i ? ", " : "", avail[i].c_str());
                    }
                    printf(")\n");
                    return 1;
                }
                fx_names.push_back(name);
            }
            if(comma == std::string::npos) break;
            start = comma + 1;
        }
    }
    if(!fx_names.empty()) {
        printf("postfx: %s", fx_names[0].c_str());
        for(size_t i = 1; i < fx_names.size(); i++) {
            printf(" -> %s", fx_names[i].c_str());
        }
        printf("\n");
    }
    postfx->Resize(display.get_width(), display.get_height());

    System sys = load_system(system_file.c_str(), terrainshader, sunshader);
    TerrainBody *sun = sys.star;
    TerrainBody *home;
    if(body_name.empty()) {
        home = sys.home;
    } else {
        home = sys.find(body_name);
        if(home == nullptr) {
            std::string avail;
            for(size_t i = 0; i < sys.bodies.size(); i++) {
                if(i) avail += ", ";
                avail += sys.bodies[i]->name;
            }
            printf("error: unknown body '%s' (available: %s)\n",
                   body_name.c_str(), avail.c_str());
            return 1;
        }
    }

    std::vector<TerrainBody *> planets = sys.bodies;

    // Build the atmosphere rim shells now that the bodies + shader exist.
    // Bodies without an atmosphere are no-ops (no mesh, no draw cost).
    for(auto&& b : planets) {
        b->BuildAtmosphere(atmosphereshader);
    }

    /* The ships are built from JSON: the parts catalog (res/parts.json)
       supplies each part's mass + behavior, the ship defs supply the stack
       order + offsets, and the fleet supplies one entry per ship: its def,
       name, body and scenario. The fleet comes from --fleet (res/fleet.json)
       or, when that is not given, from the --ship flags as a uniform fleet
       (all entries share the --body/--scenario). Omitted entry body/scenario
       fall back to the CLI values. Ships sharing a (body, scenario) pair are
       slotted: pad slots 20 m apart along the pad, orbit slots 100 m apart
       along the orbit binormal. */
    PartsCatalog part_catalog = load_parts_catalog(parts_file.c_str());
    std::vector<FleetEntry> fleet_entries;
    if(!fleet_file.empty()) {
        fleet_entries = load_fleet(fleet_file.c_str()).ships;
    } else {
        if(ship_files.empty()) { ship_files.push_back("res/ships/racer.json"); }
        for(size_t i = 0; i < ship_files.size(); i++) {
            FleetEntry e;
            e.ship = ship_files[i];
            fleet_entries.push_back(e);
        }
    }

    std::vector<Vehicle *> ships;
    std::vector<TerrainBody *> ship_homes;     // per ship: the body it starts on
    std::vector<const ScenarioDef *> ship_sc;  // per ship: its scenario
    std::vector<int> ship_slots;               // per ship: slot within its (body, scenario) group
    std::map<std::pair<TerrainBody *, bool>, StaticBuilding *> space_ports; // one per (body, pad site)
    {
        Mesh *space_port_mesh = new Mesh;
        space_port_mesh->FromFile("./res/space_port.obj", true);
        Texture *space_port_texture = load_texture("./res/space_port.png");
        Model *space_port_model = new Model;
        space_port_model->FromData(space_port_mesh, partsshader, space_port_texture);

        if(!radial_test.empty()) {
            /* --radial-test: minimal test ships built straight from the
               catalog (no JSON ship def). Passive tanks only -- no
               wheels, no thrusters -- so any spin is self-inflicted by
               the physics:
               - "radial":  tank_r5h5 + a tank_r3h2 welded to its side
               - "stacked": the same pair welded along the axis (baseline)
               - "stacks":  two 2-part stacks welded side by side:
                            [tank_r5h5 + tank_r3h2] beside
                            [tank_r5h5 + tank_r3h2], the second stack's
                            root welded radially to the first stack's
                            root (the in-game way to build it).
               Stacked welds use attachDown's convention: the child sits
               on the parent's -Z side, anchors (0,0,-hP/2) /
               (0,0,+hC/2) coinciding in world space. */
            const PartDef *defBig = part_catalog.find("tank_r5h5");
            const PartDef *defSml = part_catalog.find("tank_r3h2");
            if(defBig == nullptr || defSml == nullptr) {
                throw std::runtime_error("--radial-test: tank_r5h5 / "
                                         "tank_r3h2 missing from the parts catalog");
            }

            /* honor an explicit --scenario, otherwise orbit (no pad
               contact, no terrain noise in the spin measurement) */
            const bool scenario_given = app.get_option("--scenario") != nullptr
                && app.get_option("--scenario")->count() > 0;
            const ScenarioDef *sc = scenario_by_name(
                scenario_given ? scenario : "rot-orbit");

            Vehicle *v = new Vehicle;
            v->m_parent = home;
            v->sun = sun;
            v->frame = home->rot_frame;

            const glm::dvec3 pad_dir = glm::normalize(glm::dvec3(0.005, 0.005, 1.0));
            const glm::dmat3 pad_orient = faceAlong(pad_dir);
            /* Start 50 m above the surface so a --scenario pad ship drops
               onto the ground (the lowest part would otherwise start
               embedded in the terrain). For orbit scenarios this is only
               staging -- spawn_vehicle repositions the ship. */
            const glm::dvec3 base = pad_dir
                * ((double)home->GetTerrainHeight(pad_dir) + 50.0);
            /* radial weld: child's local +Z (its axis) -> parent's
               local +X. Columns = images of X, Y, Z. */
            const glm::dmat3 rotZtoX(glm::dvec3(0, 0, -1),
                                     glm::dvec3(0, 1, 0),
                                     glm::dvec3(1, 0, 0));

            auto makeBody = [&](const PartDef *def) -> Body * {
                Mesh *mesh = new Mesh;
                mesh->FromFile((std::string("./res/") + def->mesh).c_str(), true);
                Model *model = new Model;
                model->FromData(mesh, partsshader,
                                load_texture((std::string("./res/") + def->texture).c_str()));
                model->hull_margin = def->hull_margin;
                return create_body(model, 0, 0, 0, (float)def->mass, false);
            };

            if(radial_test == "stacks") {
                /* Two 2-part stacks, side by side. Pad normal = local +Z,
                   radial dir = local +X:
                     stack 1: A1 (tank_r5h5, root) + A2 (tank_r3h2)
                              attached below A1, axis Z
                     stack 2: B1 (tank_r5h5) welded to A1's +X side
                              (axis X) + B2 (tank_r3h2) attached beyond
                              B1 along B1's axis
                   Layout (local): A1 (0,0,0)  A2 (0,0,-3.5)
                                   B1 (7.5,0,0) B2 (11,0,0)
                   Welds (anchors coincide in world space):
                     A1-A2 stacked:  A1 (0,0,-2.5)   == A2 (0,0,+1)
                     A1-B1 radial:   A1 (5,0,0)      == B1 (0,0,-2.5)
                     B1-B2 stacked:  B1 (0,0,+2.5)   == B2 (0,0,-1) */
                v->name = "stacks4";
                Body *a1 = makeBody(defBig);
                Body *a2 = makeBody(defSml);
                Body *b1 = makeBody(defBig);
                Body *b2 = makeBody(defSml);
                setPosRot(a1, base, pad_orient);
                setPosRot(a2,
                          base - pad_orient * glm::dvec3(0.0, 0.0,
                                                         defBig->height / 2.0 + defSml->height / 2.0),
                          pad_orient);
                setPosRot(b1,
                          base + pad_orient * glm::dvec3(defBig->radius + defBig->height / 2.0, 0.0, 0.0),
                          pad_orient * rotZtoX);
                setPosRot(b2,
                          base + pad_orient * glm::dvec3(defBig->radius + defBig->height + defSml->height / 2.0, 0.0, 0.0),
                          pad_orient * rotZtoX);

                v->setRoot(a1);
                v->partDefs.push_back(defBig);
                v->partDefs.push_back(defSml);
                v->partDefs.push_back(defBig);
                v->partDefs.push_back(defSml);
                v->constraints.push_back(GlueTogether(a1, a2,
                                                      glm::dvec3(0.0, 0.0, -defBig->height / 2.0),
                                                      glm::dvec3(0.0, 0.0,  defSml->height / 2.0)));
                v->constraints.push_back(GlueTogether(a1, b1,
                                                      glm::dvec3(defBig->radius, 0.0, 0.0),
                                                      glm::dvec3(0.0, 0.0, -defBig->height / 2.0)));
                v->constraints.push_back(GlueTogether(b1, b2,
                                                      glm::dvec3(0.0, 0.0,  defBig->height / 2.0),
                                                      glm::dvec3(0.0, 0.0, -defSml->height / 2.0)));
                v->parts.push_back(a2);
                v->parts.push_back(b1);
                v->parts.push_back(b2);
            }
            else if(radial_test == "parstacks") {
                /* Two 2-part stacks side by side with ALL axes PARALLEL
                   (pad normal = local +Z) -- the variant of 'stacks' where
                   the second stack is NOT rotated, so both stacks' axes
                   point the same way:
                     stack 1: A1 (tank_r5h5, root) + A2 (tank_r3h2) below A1
                     stack 2: B1 (tank_r5h5) welded to A1's +X side
                              + B2 (tank_r3h2) below B1
                   Layout (local): A1 (0,0,0)    A2 (0,0,-3.5)
                                   B1 (10,0,0)   B2 (10,0,-3.5)
                   Welds (anchors coincide in world space):
                     A1-A2 stacked:  A1 (0,0,-2.5)  == A2 (0,0,+1)
                     B1-B2 stacked:  B1 (0,0,-2.5)  == B2 (0,0,+1)
                     A1-B1 lateral:  A1 (5,0,0)     == B1 (-5,0,0) */
                v->name = "parstacks4";
                Body *a1 = makeBody(defBig);
                Body *a2 = makeBody(defSml);
                Body *b1 = makeBody(defBig);
                Body *b2 = makeBody(defSml);
                const double dz = defBig->height / 2.0 + defSml->height / 2.0;
                setPosRot(a1, base, pad_orient);
                setPosRot(a2, base - pad_orient * glm::dvec3(0.0, 0.0, dz), pad_orient);
                setPosRot(b1, base + pad_orient * glm::dvec3(defBig->radius + defBig->radius, 0.0, 0.0), pad_orient);
                setPosRot(b2, base + pad_orient * glm::dvec3(defBig->radius + defBig->radius, 0.0, -dz), pad_orient);

                v->setRoot(a1);
                v->partDefs.push_back(defBig);
                v->partDefs.push_back(defSml);
                v->partDefs.push_back(defBig);
                v->partDefs.push_back(defSml);
                v->constraints.push_back(GlueTogether(a1, a2,
                                                      glm::dvec3(0.0, 0.0, -defBig->height / 2.0),
                                                      glm::dvec3(0.0, 0.0,  defSml->height / 2.0)));
                v->constraints.push_back(GlueTogether(a1, b1,
                                                      glm::dvec3(defBig->radius, 0.0, 0.0),
                                                      glm::dvec3(-defBig->radius, 0.0, 0.0)));
                v->constraints.push_back(GlueTogether(b1, b2,
                                                      glm::dvec3(0.0, 0.0, -defBig->height / 2.0),
                                                      glm::dvec3(0.0, 0.0,  defSml->height / 2.0)));
                v->parts.push_back(a2);
                v->parts.push_back(b1);
                v->parts.push_back(b2);
            }
            else {
                v->name = (radial_test == "radial") ? "radial2"
                           : (radial_test == "parallel") ? "parallel2" : "stack2";
                Body *a = makeBody(defBig);
                setPosRot(a, base, pad_orient);
                Body *b = makeBody(defSml);
                v->setRoot(a);
                v->partDefs.push_back(defBig);
                if(radial_test == "radial") {
                    /* B's bottom face (-hB/2) touches A's side at +rA */
                    setPosRot(b, base + pad_orient * glm::dvec3(defBig->radius + defSml->height / 2.0, 0.0, 0.0),
                              pad_orient * rotZtoX);
                    v->attachRadial(b, defSml);
                }
                else if(radial_test == "parallel") {
                    /* B's side touches A's side at +rA; both axes stay on
                       the pad normal (parallel). B at +X by rA + rB so the
                       cylindrical surfaces meet; anchor world point (rA,0,0)
                       on A == (-rB,0,0) on B. */
                    setPosRot(b, base + pad_orient * glm::dvec3(defBig->radius + defSml->radius, 0.0, 0.0),
                              pad_orient);
                    v->attachSide(b, defSml);
                }
                else {
                    /* attachDown welds the child on the parent's -Z side:
                       anchor coincidence needs B at base - (hA/2+hB/2)
                       along the pad normal */
                    setPosRot(b, base - pad_orient * glm::dvec3(0.0, 0.0, defBig->height / 2.0 + defSml->height / 2.0),
                              pad_orient);
                    v->attachDown(b, defSml);
                }
                v->partDefs.push_back(defSml);
            }
            /* These hand-built ships bypass build_ship(), which is the only
               place partStages (the per-part stage index, kept parallel to
               parts) gets set -- so seed it here. All parts are passive
               single-stage tanks, so the value is a placeholder; init()
               only requires the vector to be parallel to parts. */
            v->partStages.assign(v->parts.size(), 1);
            v->controllerIndex = 0;
            v->init();
            v->setVelocity(glm::dvec3(0, 0, 0));

            ships.push_back(v);
            ship_homes.push_back(home);
            ship_sc.push_back(sc);
            ship_slots.push_back(0);
        }
        else {
        std::map<std::pair<TerrainBody *, const ScenarioDef *>, int> slot_count;
        std::set<std::string> used_names;

        for(size_t i = 0; i < fleet_entries.size(); i++) {
            const FleetEntry &fe = fleet_entries[i];
            ShipDef def = load_ship_def(fe.ship.c_str(), part_catalog);

            TerrainBody *hb;
            if(fe.body.empty()) {
                hb = home; // the CLI --body resolution (or the system home)
            } else {
                hb = sys.find(fe.body);
                if(hb == nullptr) {
                    std::string avail;
                    for(size_t k = 0; k < sys.bodies.size(); k++) {
                        if(k) { avail += ", "; }
                        avail += sys.bodies[k]->name;
                    }
                    throw std::runtime_error("fleet: ship entry " + std::to_string(i)
                                             + ": unknown body '" + fe.body
                                             + "' (available: " + avail + ")");
                }
            }

            const ScenarioDef *sc =
                scenario_by_name(fe.scenario.empty() ? scenario : fe.scenario);

            // slot within the (body, scenario) group
            std::pair<TerrainBody *, const ScenarioDef *> key(hb, sc);
            int slot = slot_count[key];
            slot_count[key]++;

            // name: the entry's, else the def's; de-duplicated across the
            // fleet (first ship keeps the bare name, later ones get #2, #3..)
            std::string nm = fe.name.empty() ? def.name : fe.name;
            if(nm.empty()) { nm = "Ship"; }
            {
                std::string candidate = nm;
                int n = 2;
                while(used_names.count(candidate)) {
                    candidate = nm + " #" + std::to_string(n);
                    n++;
                }
                used_names.insert(candidate);
                nm = candidate;
            }

            // the pad top is this far above the terrain surface (space_port.obj
            // spans local z in [-10, 0], placed at dir * (terrain + pad_height))
            const double pad_height = 5.0;

            // pad site for this ship: the default tilted equatorial site, or
            // the body's pole for pad-polar. A space port is built per
            // (body, site): the default site always (so orbit views still show
            // the launch pad), the polar one when a pad-polar ship stands on it.
            const bool pad_polar = sc->on_pad && sc->polar;
            const glm::dvec3 pad_dir = pad_polar
                ? glm::dvec3(0.0, 1.0, 0.0)
                : glm::normalize(glm::dvec3(0.005, 0.005, 1.0));
            const glm::dmat3 pad_orient = faceAlong(pad_dir);
            auto place_pad = [&](bool polar, const glm::dvec3 &dir) {
                const std::pair<TerrainBody *, bool> key(hb, polar);
                if(space_ports.find(key) != space_ports.end()) { return; }
                const glm::dvec3 start = dir * (double)hb->GetTerrainHeight(dir);
                StaticBuilding *sp = new StaticBuilding;
                sp->body = create_body(space_port_model, 0, 0, 0, 0, false);
                setPosRot(sp->body, start + dir * pad_height, faceAlong(dir));
                sp->parent = hb;
                sp->sun = sun;
                space_ports[key] = sp;
            };
            place_pad(false, glm::normalize(glm::dvec3(0.005, 0.005, 1.0))); // default site
            if(pad_polar) { place_pad(true, pad_dir); }                      // polar site

            Vehicle *v = new Vehicle;
            v->name = nm;
            v->m_parent = hb;
            v->sun = sun;
            v->frame = hb->rot_frame;

            // stack base: this body's pad top + the ship's lateral pad slot
            // (pad local X, 20 m apart) so pad ships stand side by side.
            // For orbit scenarios this is only staging -- spawn_vehicle
            // repositions the ship along vhat; the part offsets relative to
            // the ship's own center of mass are what survive.
            const glm::dvec3 base = pad_dir * ((double)hb->GetTerrainHeight(pad_dir) + pad_height)
                + pad_orient * glm::dvec3(20.0 * (double)slot, 0.0, 0.0);
            build_ship(v, def, partsshader, base, pad_orient);
            v->setVelocity(glm::dvec3(0, 0, 0));

            ships.push_back(v);
            ship_homes.push_back(hb);
            ship_sc.push_back(sc);
            ship_slots.push_back(slot);
        }
        }
    }
    check_gl_error();

    /* Apply each ship's scenario (before the camera is constructed,
       so the camera focuses on the spawn point). Ships sharing a
       body+scenario group get their own orbit slot (100 m apart along the
       orbit binormal) so they don't spawn on top of each other. */
    for(size_t i = 0; i < ships.size(); i++) {
        spawn_vehicle(ships[i], *ship_sc[i], ship_homes[i], sys,
                      100.0 * (double)ship_slots[i]);
    }

    /* the active (player-controlled) ship: Tab / the SHIPS window switch
       it; the local `ship` below always points at it, so the HUD, camera,
       input and draw code follow the active ship without special cases. */
    int activeIdx = 0;
    Vehicle *ship = ships[0];

    /* Idle ships park on rails: flying ones coast on their conic, pad
       ships freeze in the surface frame (their pose rides the planet's
       spin via the render transform). Ships that are neither in free
       fall nor grounded refuse and stay in the physics world. */
    for(size_t i = 0; i < ships.size(); i++) {
        if((int)i != activeIdx) { ships[i]->goOnRails(); }
    }

    Mesh *engine_plume_mesh = new Mesh;
    engine_plume_mesh->FromFile("./res/engine_plume.obj", false);
    Texture *engine_plume_texture = load_texture("res/engine_plume.png");
    Model *engine_plume_model = new Model;
    engine_plume_model->FromData(engine_plume_mesh, partsshader, engine_plume_texture);

    Shader *billboardshader = new Shader;
    billboardshader->registerAttribs({ "position", "texcoord", "normal" });
    billboardshader->registerUniforms({ "MVP", "color_uniform" });
    billboardshader->FromFile("./res/billboardshader");

    // Billboard icons opt out of mip chains: their alpha cutouts bleed
    // into the neighbouring level when minified.
    Texture * front_indicator_texture = load_texture("res/front_crosshair.png", false);
    Texture * prograde_indicator_texture = load_texture("res/prograde_icon.png", false);
    Texture * retrograde_indicator_texture = load_texture("res/retrograde_icon.png", false);
    Texture * radial_in_indicator_texture = load_texture("res/radial_in_icon.png", false);
    Texture * radial_out_indicator_texture = load_texture("res/radial_out_icon.png", false);
    Texture * normal_plus_indicator_texture = load_texture("res/normal_plus_icon.png", false);
    Texture * normal_minus_indicator_texture = load_texture("res/normal_minus_icon.png", false);

    glm::vec4 billboardcolor = glm::vec4(1, 1, 1, 1.0); // TODO should these be different colors?

    Billboard *front_indicator =
        mk_billboard(billboardshader, front_indicator_texture, 1.0, 1.0, billboardcolor);
    Billboard *prograde_indicator =
        mk_billboard(billboardshader, prograde_indicator_texture, 1.0, 1.0, billboardcolor);
    Billboard *retrograde_indicator =
        mk_billboard(billboardshader, retrograde_indicator_texture, 1.0, 1.0, billboardcolor);
    Billboard *radial_in_indicator =
        mk_billboard(billboardshader, radial_in_indicator_texture, 1.0, 1.0, billboardcolor);
    Billboard *radial_out_indicator =
        mk_billboard(billboardshader, radial_out_indicator_texture, 1.0, 1.0, billboardcolor);
    Billboard *normal_plus_indicator =
        mk_billboard(billboardshader, normal_plus_indicator_texture, 1.0, 1.0, billboardcolor);
    Billboard *normal_minus_indicator =
        mk_billboard(billboardshader, normal_minus_indicator_texture, 1.0, 1.0, billboardcolor);

    /* camera init */
    const float camFov = M_PI/3.0; // TODO specify FOV in deg?
    // The drawable size the Renderer actually got (the WM may have clamped
    // it, or fullscreen may have used the display mode).
    const float camAspect = (float)display.get_width() / (float)display.get_height();
    const float camZNear = 1.0f;
    // zFar must exceed the farthest visible body. The log-depth shaders
    // (res/*Shader.vs) define the hard far limit as `far = 1e13` m, which
    // covers the real solar system (Pluto at ~5.9e12 m) and KSP-style
    // AU scales (~1.4e10 m). Keep zFar consistent with that.
    const float camZFar = 1e13;

    OrbitCamera *orbitCam = new OrbitCamera(GetPosition(ship->controller),
                                            camFov, camAspect, camZNear, camZFar);
    glm::dvec3 freeCamPos  = orbitCam->GetPos();
    glm::dvec3 freeCamFwd  = orbitCam->GetForward();
    glm::dvec3 freeCamUp   = orbitCam->up;
    if(free_cam_pos.size() == 3) { // TODO do we need these guards?
        freeCamPos = glm::dvec3(free_cam_pos[0], free_cam_pos[1], free_cam_pos[2]);
    }
    if(free_cam_fwd.size() == 3) {
        freeCamFwd = glm::dvec3(free_cam_fwd[0], free_cam_fwd[1], free_cam_fwd[2]);
    }
    if(free_cam_up.size() == 3) {
        freeCamUp = glm::dvec3(free_cam_up[0], free_cam_up[1], free_cam_up[2]);
    }
    FreeCamera *freeCam = new FreeCamera(freeCamPos, freeCamFwd, freeCamUp,
                                         camFov, camAspect, camZNear, camZFar);
    Camera *camera = orbitCam;   // active camera

    enum CameraMode { CAM_ORBIT, CAM_FREE };
    CameraMode camMode = CAM_ORBIT;
    if(use_free_cam) {
        camMode = CAM_FREE;
        camera = freeCam;
    }

    // Bodies the orbit camera can target (the ship is the default). Built from
    // the loaded system: the ship, then every body in the system (in file
    // order), so G cycles through all of them.
    struct FocusTarget { const char *name; TerrainBody *body; };
    std::vector<FocusTarget> focusTargets;
    focusTargets.push_back({ "ship", nullptr });
    for (TerrainBody *b : sys.bodies) {
        focusTargets.push_back({ b->name.c_str(), b });
    }
    const int numFocusTargets = (int)focusTargets.size();
    int focusBody = 0;   // index into focusTargets

    // World (ship-frame) position of a focus target, to point the orbit
    // camera at it each frame.
    auto focusWorldPos = [&](int i) -> glm::dvec3 {
        if (focusTargets[i].body == nullptr) {
            return ship->get_center_of_mass();
        }
        return focusTargets[i].body->frame->GetPositionRelTo(ship->frame);
    };

    bool running = true;
    bool redraw = false;
    bool screenshot_requested = false;
    int screenshot_count = 0;
    bool poly_mode = false;
    bool rmbCam = false;
    SDL_SetRelativeMouseMode(SDL_FALSE);

    const double dt = 1.0/50.0; // TODO explain why 50

    /* Rails warp: at this accel and above nobody is integrated -- every
       ship coasts on rails (or sits frozen on the ground) and the Bullet
       world is not stepped at all, so the cost per tick is O(ships).
       Below it the active ship is always in the physics world. */
    const int kRailsWarp = 10000;
    double currentTime = 0.001 * (double)(SDL_GetTicks());
    double accumulator = 0.0;
    int time_accel = initial_time_accel;

    /* Starting the game directly in rails warp: the active ship parks
       too (works on the pad -- that is the frozen mode), unless some
       ship is not rail-eligible, in which case clamp to physics warp. */
    if(time_accel >= kRailsWarp) {
        bool all_eligible = true;
        for(auto *s : ships) {
            if(!s->canRail()) {
                printf("Rails warp refused at start: '%s' is neither in free "
                       "fall nor grounded; clamping time accel to 1000\n",
                       s->name.c_str());
                all_eligible = false;
                break;
            }
        }
        if(all_eligible) {
            for(auto *s : ships) { s->goOnRails(); }
        } else {
            time_accel = 1000;
        }
    }
    int cam_speed = 1;
    bool physics_debug_drawing = false;
    bool world_drawing = true;
    bool draw_starfield = true;
    bool draw_skylines = false;

    /* UI windows (src/ui.h): one options block per window, plus a table the
       main-menu checkboxes, the F10 toggle and a UI reset all share. The
       main menu itself and the HUD trio (one "Top HUD" checkbox) are
       handled separately. Slots/offsets: left column stacks under the
       menu, right column under ORBITAL, the rest spread over the edges so
       the center stays clear for the 3D view. */
    const ImVec4 info_bg = ImVec4(0.15f, 0.15f, 0.15f, 1.0f);
    auto info_opts = [info_bg](ui::Slot slot) {
        ui::Options o;
        o.slot = slot;
        o.has_bg = true;
        o.bg_color = info_bg;
        return o;
    };
    // Layout: top left ORBITAL + SURFACE, top right RESOURCES,
    // middle right the menu, bottom right VESSEL, bottom left Orbital map.
    ui::Options o_orbit     = info_opts(ui::Slot::TopLeft);
    ui::Options o_surface   = info_opts(ui::Slot::TopLeft);
    o_surface.right_of = "ORBITAL";
    ui::Options o_resources = info_opts(ui::Slot::TopRight);
    o_resources.width_ratio = 1.5f; // bars have no width of their own
    ui::Options o_menu      = info_opts(ui::Slot::MiddleRight);
    o_menu.closable = false;
    ui::Options o_vessel    = info_opts(ui::Slot::BottomRight);
    ui::Options o_parts     = info_opts(ui::Slot::BottomRight);
    o_parts.below = "VESSEL";
    o_parts.default_open = false;
    ui::Options o_map       = info_opts(ui::Slot::BottomLeft);
    o_map.initial_size = ImVec2(480.0f, 480.0f); // orbit drawn at (200,200)
    // The rest stay out of the way of the above (all closed by default).
    ui::Options o_ships     = info_opts(ui::Slot::TopCenter);
    ui::Options o_autopilot = info_opts(ui::Slot::Center);
    o_autopilot.default_open = false;
    ui::Options o_controls  = info_opts(ui::Slot::BottomCenter);
    o_controls.default_open = false;
    ui::Options o_debug     = info_opts(ui::Slot::TopCenter);
    o_debug.default_open = false;
    ui::Options o_telemetry = info_opts(ui::Slot::MiddleLeft);
    o_telemetry.default_open = false;
    o_telemetry.initial_size = ImVec2(460.0f, 680.0f);
    ui::Options o_hud;
    o_hud.fixed = true;
    o_hud.closable = false;
    o_hud.default_open = false;
    o_hud.flags |= ImGuiWindowFlags_NoTitleBar;
    o_hud.has_bg = true;
    o_hud.bg_color = ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
    o_hud.slot = ui::Slot::TopCenter;
    ui::Options o_mainmenu = info_opts(ui::Slot::Center);
    o_mainmenu.fixed = true;
    o_mainmenu.closable = false;
    o_mainmenu.default_open = false;

    struct UiWin { const char *name, *label; ui::Options opts; };
    std::vector<UiWin> ui_windows;
    auto add_ui_window = [&](const char *name, const char *label,
                             const ui::Options &o) {
        ui_windows.push_back(UiWin{name, label, o});
    };
    add_ui_window("RESOURCES", "Resources", o_resources);
    add_ui_window("ORBITAL", "Orbit Info", o_orbit);
    add_ui_window("Orbital map", "Orbit Map", o_map);
    add_ui_window("SURFACE", "Surface Info", o_surface);
    add_ui_window("VESSEL", "Vessel Info", o_vessel);
    add_ui_window("SHIP PARTS", "Vessel Parts", o_parts);
    if(ships.size() > 1) {
        add_ui_window("SHIPS", "Ship List", o_ships);
    }
    add_ui_window("Autopilot", "DUMB-ASS", o_autopilot);
    add_ui_window("Controls help", "Controls Help", o_controls);
    add_ui_window("Game Debug Info", "Game Debug Info", o_debug);
    add_ui_window("TELEMETRY", "Telemetry", o_telemetry);
    const char *const hud_windows[3] = { "HUD Altitude", "HUD Orbit", "HUD Speed" };
    bool ui_visible = true; // F10 toggle: is the UI shown?

    double time = 0;

    // Telemetry plots (active ship, sampled once per rendered frame).
    TimeSeries energy_series;
    TimeSeries angmom_series;

    // --orbit-log: print at most once per wall-clock interval
    const Uint32 orbit_log_interval_ms = (Uint32)(orbit_interval * 1000.0);
    Uint32 orbit_log_last_ms = 0;

    Skybox skybox;
    skybox.init();

    // Two reference circles in the render frame's local axes. Each is its
    // own mesh so it can be drawn a distinct colour: the XZ plane (y=0, the
    // "flat" orbital/equatorial reference) and the XY plane (z=0, the
    // "vertical" meridian reference).
    Mesh *skyline_xz = new Mesh;
    Mesh *skyline_xy = new Mesh;
    {
        float r = 1000;
        int n = 128;
        PosInterface xzinterface;
        PosInterface xyinterface;
        for(int i = 1; i < 128; i++) {
            const double a = (2 * M_PI) * float(i-1)/float(n);
            xzinterface.positions.push_back(glm::vec3(r * cos(a), 0, r * sin(a)));  // y=0 -> XZ plane
            xyinterface.positions.push_back(glm::vec3(r * cos(a), r * sin(a), 0));  // z=0 -> XY plane
        }
        skyline_xz->InitMesh(xzinterface);
        skyline_xy->InitMesh(xyinterface);
    }

    // Switch the active (controlled) ship. The ship being left is released:
    // throttle zeroed, armed thrust + rotation commands cleared, and it
    // parks on rails (coasting or frozen) if it can. The ship being taken
    // re-enters physics. Taking control during rails warp drops the warp
    // to 1000 -- the active ship is now being integrated. The orbit
    // camera recenters on the ship being taken.
    auto select_ship = [&](int idx) {
        if(idx < 0 || idx >= (int)ships.size() || idx == activeIdx) { return; }
        ships[activeIdx]->releaseControl();
        ships[activeIdx]->goOnRails();
        activeIdx = idx;
        ships[activeIdx]->leaveRails();
        ship = ships[activeIdx];
        if(time_accel >= kRailsWarp) { time_accel = 1000; }
        focusBody = 0;   // back to the "ship" focus target
        if(camMode == CAM_ORBIT) {
            orbitCam->Follow(ship->get_center_of_mass());
            orbitCam->distance = 50.0;
        }
        printf("Active ship %d of %d: %s\n",
               activeIdx + 1, (int)ships.size(), ship->name.c_str());
    };

    /* Enter rails warp: park every ship (flying ones coast on their
       conic, grounded ones freeze on the ground). Refuses -- and keeps
       the current accel -- if any ship is not rail-eligible, e.g. a
       suborbital descent in progress. */
    auto enter_rails_warp = [&]() -> bool {
        for(auto *s : ships) {
            if(!s->canRail()) {
                printf("Rails warp refused: '%s' is neither in free fall nor "
                       "grounded (warp stays %d)\n", s->name.c_str(), time_accel);
                return false;
            }
        }
        for(auto *s : ships) { s->goOnRails(); }
        return true;
    };

    /* --selftest-stage: exercise the staging path on the first multi-stage
       ship (drop the active stage, then try again -- the last stage must be
       refused), printing before/after state. Runs before the loop; the
       loop then takes a few physics ticks to prove the world is stable with
       the weld cut and the dropped bodies gone, and exits. */
    int selftest_ticks = 0;
    if(selftest_stage) {
        Vehicle *stager = 0;
        for(size_t i = 0; i < ships.size(); i++) {
            if(ships[i]->numStages() >= 2) { stager = ships[i]; break; }
        }
        if(!stager) {
            printf("selftest-stage: no multi-stage ship in the fleet\n");
            running = false;
        } else {
            stager->leaveRails();   // idle pad ships park frozen; staging needs physics
            printf("== selftest-stage: %s ==\n", stager->name.c_str());
            printf("before: parts=%zu mass=%.1f kg stage=%d/%d TWR=%.2f\n",
                   stager->parts.size(), stager->getMass(),
                   stager->activeStage(), stager->numStages(), stager->getTWR());
            const int dropped = stager->separateStage(stager->activeStage());
            printf("drop 1: dropped=%d parts=%zu mass=%.1f kg stage=%d/%d TWR=%.2f\n",
                   dropped, stager->parts.size(), stager->getMass(),
                   stager->activeStage(), stager->numStages(), stager->getTWR());
            const int dropped2 = stager->separateStage(stager->activeStage());
            printf("drop 2: dropped=%d (expect 0: last stage stays)\n", dropped2);
            selftest_ticks = 30;
        }
    }

    /* --selftest-rails: coast the first railed ship 30 ticks and check its
       conic is conserved, then select it (rails -> physics handoff,
       checking COM continuity) and run 60 physics ticks to prove the
       world is stable with the re-added bodies. */
    int rails_test_ticks = 0;
    int rails_test_idx = -1;
    OrbitElements rails_test_e0;
    if(selftest_rails) {
        if(time_accel == 0) {
            time_accel = 100;
            printf("selftest-rails: forcing time accel to 100 (was paused)\n");
        }
        for(size_t i = 0; i < ships.size(); i++) {
            // a COASTING ship (frozen pad ships have no conic to conserve)
            if(ships[i]->onRails && !ships[i]->railFrozen) { rails_test_idx = (int)i; break; }
        }
        if(rails_test_idx < 0) {
            printf("selftest-rails: no coasting ship on rails in the fleet\n");
            running = false;
        } else {
            Vehicle *r = ships[rails_test_idx];
            rails_test_e0 = computeOrbitElements(r->rail_pos, r->rail_vel,
                                                 r->frame->body->mu);
            printf("== selftest-rails: %s ==\n", r->name.c_str());
            printf("rail t0: sma=%.9g m ecc=%.9g peri=%.9g m apo=%.9g m T=%.9g s\n",
                   rails_test_e0.semi_major, rails_test_e0.ecc,
                   rails_test_e0.periapsis, rails_test_e0.apoapsis,
                   rails_test_e0.period);
            rails_test_ticks = 90;
        }
    }

    // --timeout: wall-clock budget for the whole run (0 = run until closed).
    const Uint32 loop_start_ms = SDL_GetTicks();
    const double startup_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - prog_start).count();
    printf("Main loop starting: startup took %.3f s", startup_s);
    if(timeout_seconds > 0.0) {
        printf(" | auto-exit after %.1f s (wall clock)", timeout_seconds);
    }
    printf("\n");
    fflush(stdout);

    // --frame-cap: budget per loop iteration (0 = uncapped). Physics stays
    // at its fixed 50 Hz off the wall clock regardless of this.
    const int cap_ms = (frame_cap > 0) ? (int)(1000.0 / (double)frame_cap) : 0;
    if (cap_ms > 0) {
        printf("frame cap: %d fps\n", frame_cap);
    } else {
        printf("frame cap: off (uncapped)\n");
    }

    /* main loop timing from
       http://gafferongames.com/game-physics/fix-your-timestep/
    */
    while (running == true) {
        const Uint32 iter_start_ms = SDL_GetTicks();

        // --timeout: auto-exit once the wall-clock budget is spent.
        if(timeout_seconds > 0.0) {
            const double elapsed_s = (SDL_GetTicks() - loop_start_ms) * 0.001;
            if(elapsed_s >= timeout_seconds) {
                printf("Timeout reached (%.1f s); exiting main loop.\n", elapsed_s);
                fflush(stdout);
                running = false;
            }
        }

        // --selftest-stage: a few post-separation physics ticks, then exit.
        if(selftest_ticks > 0) {
            selftest_ticks--;
            if(selftest_ticks == 0) {
                printf("selftest-stage: 30 ticks after separation, no crash; OK\n");
                fflush(stdout);
                running = false;
            }
        }

        // --selftest-rails: 30 ticks on rails (verify), handoff, 60 physics
        // ticks (stability), exit.
        if(rails_test_ticks > 0) {
            rails_test_ticks--;
            if(rails_test_ticks == 60) {
                Vehicle *r = ships[rails_test_idx];
                const OrbitElements e1 = computeOrbitElements(r->rail_pos, r->rail_vel,
                                                              r->frame->body->mu);
                printf("rail t30: sma=%.9g m ecc=%.9g peri=%.9g m apo=%.9g m\n",
                       e1.semi_major, e1.ecc, e1.periapsis, e1.apoapsis);
                double drift = 0.0;
                drift = std::max(drift, fabs(e1.semi_major - rails_test_e0.semi_major)
                                        / fabs(rails_test_e0.semi_major));
                drift = std::max(drift, fabs(e1.ecc - rails_test_e0.ecc));
                drift = std::max(drift, fabs(e1.periapsis - rails_test_e0.periapsis)
                                        / fabs(rails_test_e0.periapsis));
                drift = std::max(drift, fabs(e1.apoapsis - rails_test_e0.apoapsis)
                                        / fabs(rails_test_e0.apoapsis));
                drift = std::max(drift, fabs(e1.ang_momentum - rails_test_e0.ang_momentum)
                                        / fabs(rails_test_e0.ang_momentum));
                printf("rail drift after 30 coast ticks: %.3e %s\n", drift,
                       drift < 1e-9 ? "(conserved OK)" : "(DRIFT!)");
                const glm::dvec3 rail_com = r->rail_pos;
                select_ship(rails_test_idx);   // rails -> physics handoff
                const double jump = glm::length(r->get_center_of_mass() - rail_com);
                printf("handoff continuity: |com - rail_com| = %.6f m\n", jump);
                fflush(stdout);
            }
            if(rails_test_ticks == 0) {
                printf("selftest-rails: 60 ticks after handoff, no crash; OK\n");
                fflush(stdout);
                running = false;
            }
        }

        /*
          EVENTS
        */
        /* --sim-press: emit the synthetic key events that fell due this
           frame, in down-then-up order per press. They are polled below in
           the same frame, so one-shot actions fire in the frame the press
           is due. */
        if(!sim_presses.empty()) {
            const Uint32 now = SDL_GetTicks() - loop_start_ms;
            auto push_key = [&](SDL_EventType type, const SimKeyPress &p) {
                SDL_Event kev = {0};
                kev.type = type;
                kev.key.windowID = sim_win_id;
                kev.key.state = (type == SDL_KEYDOWN) ? SDL_PRESSED : SDL_RELEASED;
                kev.key.repeat = 0;
                kev.key.keysym.sym = p.key;
                kev.key.keysym.scancode = p.sc;
                SDL_PushEvent(&kev);
            };
            for(auto &p : sim_presses) {
                if(!p.down_sent && now >= p.down_ms) {
                    push_key(SDL_KEYDOWN, p);
                    p.down_sent = true;
                }
                if(p.down_sent && !p.up_sent && now >= p.up_ms) {
                    push_key(SDL_KEYUP, p);
                    p.up_sent = true;
                }
            }
        }

        /* --sim-mouse: emit the synthetic mouse events that fell due this
           frame, in the order each gesture needs. A drag (button + held)
           presses the button BEFORE moving so the camera-look handler
           (gated on rmbCam) sees the button down first; a click moves the
           cursor into place then presses + releases in place; BTN==0 just
           repositions. The motion carries the delta from the previous
           simulated position (sim_mouse_x/y), which the camera consumes. */
        if(!sim_mouse_actions.empty()) {
            const Uint32 now = SDL_GetTicks() - loop_start_ms;
            auto push_motion = [&](int x, int y) {
                SDL_Event mev = {0};
                mev.type = SDL_MOUSEMOTION;
                mev.motion.windowID = sim_win_id;
                mev.motion.which = 0;
                mev.motion.x = x;
                mev.motion.y = y;
                mev.motion.xrel = x - sim_mouse_x;
                mev.motion.yrel = y - sim_mouse_y;
                mev.motion.state = 0;
                SDL_PushEvent(&mev);
                sim_mouse_x = x;
                sim_mouse_y = y;
            };
            auto push_btn = [&](SDL_EventType type, int button, int x, int y) {
                SDL_Event bev = {0};
                bev.type = type;
                bev.button.windowID = sim_win_id;
                bev.button.which = 0;
                bev.button.button = (Uint8)button;
                bev.button.state = (type == SDL_MOUSEBUTTONDOWN) ? SDL_PRESSED
                                                                 : SDL_RELEASED;
                bev.button.x = x;
                bev.button.y = y;
                SDL_PushEvent(&bev);
            };
            for(auto &a : sim_mouse_actions) {
                if(!a.started && now >= a.time_ms) {
                    if(a.button != 0 && a.up_ms > a.time_ms) {
                        // drag: press, then move (release comes at up_ms)
                        push_btn(SDL_MOUSEBUTTONDOWN, a.button, a.x, a.y);
                        push_motion(a.x, a.y);
                    } else if(a.button != 0) {
                        // click: move into place, press, release (same frame)
                        push_motion(a.x, a.y);
                        push_btn(SDL_MOUSEBUTTONDOWN, a.button, a.x, a.y);
                        push_btn(SDL_MOUSEBUTTONUP, a.button, a.x, a.y);
                        a.released = true;
                    } else {
                        // move only (no button)
                        push_motion(a.x, a.y);
                    }
                    a.started = true;
                }
                // release a held button at up_ms
                if(a.button != 0 && a.started && !a.released
                   && now >= a.up_ms) {
                    push_btn(SDL_MOUSEBUTTONUP, a.button, a.x, a.y);
                    a.released = true;
                }
            }
        }

        SDL_Event ev;

        while (SDL_PollEvent(&ev)) {
            ImGui_ImplSDL2_ProcessEvent(&ev);
            if (ev.type == SDL_QUIT) {
                running = false;
            }

            if (ev.type == SDL_WINDOWEVENT) {
                if(ev.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                    display.onResize(ev.window.data1, ev.window.data2);
                    check_gl_error();

                    postfx->Resize(ev.window.data1, ev.window.data2);
                    check_gl_error();

                    camera->setAspect((float)ev.window.data1 / (float)ev.window.data2);
                    check_gl_error();
                }
            }
            if(ev.type == SDL_KEYDOWN) {
                if(ev.key.keysym.sym == SDLK_PERIOD) {
                    if(time_accel < 1000) {
                        time_accel *= 10;
                        if(time_accel == 0) {
                            time_accel = 1;
                        }
                    } else if(time_accel < 100000) {
                        // 1000 -> 10000 -> 100000: rails warp. Every ship
                        // coasts (or freezes on the ground) and the physics
                        // world stops stepping; refuses if any ship is not
                        // rail-eligible.
                        if(enter_rails_warp()) {
                            time_accel *= 10;
                            printf("Rails warp: time accel %d (ships on rails)\n",
                                   time_accel);
                        }
                    }
                }
                if(ev.key.keysym.sym == SDLK_COMMA) {
                    if(time_accel > 1) {
                        const bool leaving_rails_warp =
                            (time_accel >= kRailsWarp) && (time_accel / 10 < kRailsWarp);
                        time_accel /= 10;
                        if(leaving_rails_warp) {
                            // dropped out of rails warp: the active ship
                            // re-enters physics (idle ships stay parked)
                            ship->leaveRails();
                            printf("Rails warp: exited, time accel %d\n", time_accel);
                        }
                    }
                    else if(time_accel == 1) {
                        time_accel = 0;
                    }
                }
                if(ev.key.keysym.sym == SDLK_l) {
                    if(cam_speed < 10000000) {
                        cam_speed *= 4;
                    }
                }
                if(ev.key.keysym.sym == SDLK_k) {
                    if(cam_speed > 1) {
                        cam_speed /= 4;
                    }
                }
                if(ev.key.keysym.sym == SDLK_c) {
                    // Toggle between the body-orbit camera and the free camera.
                    if(camMode == CAM_ORBIT) {
                        freeCam->pos = orbitCam->pos;
                        freeCam->forward = orbitCam->forward;
                        freeCam->up = orbitCam->up;
                        camMode = CAM_FREE;
                        camera = freeCam;
                    } else {
                        glm::dvec3 focus = focusWorldPos(focusBody);
                        orbitCam->Follow(focus);
                        double dist = glm::length(freeCam->pos - focus);
                        if(dist < 10.0) { dist = 10.0; }
                        orbitCam->distance = dist;
                        camMode = CAM_ORBIT;
                        camera = orbitCam;
                        printf("Camera: orbiting %s (G = switch body, C = free)\n",
                               focusTargets[focusBody].name);
                    }
                }
                if(ev.key.keysym.sym == SDLK_g) {
                    // Cycle the orbit camera's target body.
                    if(camMode == CAM_ORBIT) {
                        focusBody = (focusBody + 1) % numFocusTargets;
                        orbitCam->Follow(focusWorldPos(focusBody));
                        double d = (focusTargets[focusBody].body == nullptr)
                            ? 50.0
                            : (double)focusTargets[focusBody].body->radius * 3.0;
                        orbitCam->distance = d;
                        printf("Orbit camera targeting %s\n", focusTargets[focusBody].name);
                    } else {
                        printf("In free flight; press C to go to orbit, then G to switch body.\n");
                    }
                }
                if(ev.key.keysym.sym == SDLK_TAB) {
                    // cycle the active ship (ignored for a lone ship;
                    // auto-repeat would just keep cycling)
                    if(!ev.key.repeat && ships.size() > 1) {
                        select_ship((activeIdx + 1) % (int)ships.size());
                    }
                }
                if(ev.key.keysym.sym == SDLK_SPACE) {
                    // separate the active stage (one-shot; auto-repeat would
                    // keep dropping stages). Only while flying a ship with
                    // time running (a paused separation would leave the
                    // survivors frozen mid-air).
                    if(!ev.key.repeat && camMode == CAM_ORBIT && time_accel > 0) {
                        // staging needs the parts in the physics world:
                        // wake a ship parked on rails first
                        if(ship->onRails) {
                            ship->leaveRails();
                            if(time_accel >= kRailsWarp) { time_accel = 1; }
                        }
                        int dropped = ship->separateStage(ship->activeStage());
                        if(dropped > 0) {
                            printf("Stage: dropped %d part(s); now on stage %d of %d\n",
                                   dropped, ship->activeStage(), ship->numStages());
                        } else {
                            printf("Stage: nothing left to separate\n");
                        }
                    }
                }
                if(ev.key.keysym.sym == SDLK_F12) {
                    screenshot_requested = true;
                }
                if(ev.key.keysym.sym == SDLK_F11) {
                    if(poly_mode == false) {
                        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                        poly_mode = true;
                    } else {
                        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                        poly_mode = false;
                    }
                }
                if(ev.key.keysym.sym == SDLK_F10 ||
                   ev.key.keysym.sym == SDLK_ESCAPE) {
                    // Toggle the main menu; its "Toggle windows" button
                    // does the old F10 window show/hide.
                    ui::SetOpen("Main Menu", !ui::IsOpen("Main Menu"));
                }
            }
            if(ev.type == SDL_MOUSEBUTTONDOWN) {
                // holding RMB over 3D (not over a UI window) moves the camera.
                if(ev.button.button == SDL_BUTTON_RIGHT &&
                   !ImGui::GetIO().WantCaptureMouse) {
                    rmbCam = true;
                }
            }
            if(ev.type == SDL_MOUSEBUTTONUP) {
                if(ev.button.button == SDL_BUTTON_RIGHT) {
                    rmbCam = false;
                }
            }
            if(ev.type == SDL_MOUSEMOTION) {
                if(rmbCam && !ImGui::GetIO().WantCaptureMouse) {
                    camera->RotateY(-ev.motion.xrel / 200.0f);
                    camera->Pitch(ev.motion.yrel / 200.0f);
                }
            }
            if(ev.type == SDL_MOUSEWHEEL) {
                // Zoom when the wheel is not scrolling a UI window.
                if(!ImGui::GetIO().WantCaptureMouse) {
                    camera->wheel(ev.wheel.y);
                }
            }
        }

        /*
          LOGIC
        */
        double newTime = (double)(SDL_GetTicks()) * 0.001;
        double frameTime = newTime - currentTime;
        currentTime = newTime;
        accumulator += frameTime;

        glm::dvec3 com;

        if(accumulator > 10 * dt) {
            accumulator = 10 * dt;
        }

        // clear stats and stuff
        for(auto *s : ships) { s->m_thrust = 0.0; }

        while (accumulator >= dt) {
            // is this logic? ;_;
            // Thrust and rotation are armed once per tick (if the keys are
            // held, below) and then re-applied before every substep; clear
            // them first so a tick without the keys doesn't keep pushing or
            // slewing from the last one.
            ship->clearThrust();
            ship->clearRotCmd();

            const Uint8* key = SDL_GetKeyboardState(NULL);
            /* --sim-press: a synthetic key is "down" from its down time to
               its up time. SDL_PushEvent does not update the state array
               above (verified on this SDL), so held commands OR in each
               entry's window instead. */
            auto isDown = [&](SDL_Scancode sc) -> bool {
                if(key[sc]) { return true; }
                for(size_t i = 0; i < sim_presses.size(); i++) {
                    if(sim_presses[i].sc == sc && sim_presses[i].down_sent
                       && !sim_presses[i].up_sent) {
                        return true;
                    }
                }
                return false;
            };

            if (camMode == CAM_FREE) {
                if (isDown(SDL_SCANCODE_W)) { camera->MoveForward(cam_speed); }
                else if (isDown(SDL_SCANCODE_S)) { camera->MoveForward(-cam_speed); }

                if (isDown(SDL_SCANCODE_A)) { camera->MoveRight(-cam_speed); }
                else if (isDown(SDL_SCANCODE_D)) { camera->MoveRight(cam_speed); }

                if (isDown(SDL_SCANCODE_Q)) { camera->Roll(-0.05); }
                else if (isDown(SDL_SCANCODE_E)) { camera->Roll(0.05); }

                if (isDown(SDL_SCANCODE_LSHIFT) || isDown(SDL_SCANCODE_RSHIFT)) { camera->MoveUp(cam_speed); }
                else if (isDown(SDL_SCANCODE_LCTRL) || isDown(SDL_SCANCODE_RCTRL)) { camera->MoveUp(-cam_speed); }
            }

            if (camMode == CAM_ORBIT) {
                bool game_running = (time_accel > 0);
                /* touching the controls wakes a ship parked on rails: it
                   re-enters physics and rails warp drops to 1x (you cannot
                   maneuver on rails). */
                if(ship->onRails && time_accel >= kRailsWarp) {
                    if(isDown(SDL_SCANCODE_W) || isDown(SDL_SCANCODE_S) ||
                       isDown(SDL_SCANCODE_A) || isDown(SDL_SCANCODE_D) ||
                       isDown(SDL_SCANCODE_Q) || isDown(SDL_SCANCODE_E) ||
                       isDown(SDL_SCANCODE_I) || isDown(SDL_SCANCODE_X) ||
                       isDown(SDL_SCANCODE_B) || isDown(SDL_SCANCODE_N) ||
                       isDown(SDL_SCANCODE_R) || isDown(SDL_SCANCODE_F)) {
                        ship->leaveRails();
                        time_accel = 1;
                        printf("Control input: '%s' left the rails, warp -> 1\n",
                               ship->name.c_str());
                    }
                }
                // pitch
                if (isDown(SDL_SCANCODE_W)) { ship->Command(ShipCmd(Pitch, +1.0f), game_running); }
                if (isDown(SDL_SCANCODE_S)) { ship->Command(ShipCmd(Pitch, -1.0f), game_running); }
                // yaw
                if (isDown(SDL_SCANCODE_A)) { ship->Command(ShipCmd(Yaw, +1.0f), game_running); }
                if (isDown(SDL_SCANCODE_D)) { ship->Command(ShipCmd(Yaw, -1.0f), game_running); }
                // roll
                if (isDown(SDL_SCANCODE_Q)) { ship->Command(ShipCmd(Roll, +1.0f), game_running); }
                if (isDown(SDL_SCANCODE_E)) { ship->Command(ShipCmd(Roll, -1.0f), game_running); }

                if (isDown(SDL_SCANCODE_I)) { ship->Command(ShipCmd(Thrust), game_running, dt * time_accel); }
                if (isDown(SDL_SCANCODE_X)) { ship->Command(ShipCmd(KillRot), game_running); }

                if (isDown(SDL_SCANCODE_B)) { ship->Command(ShipCmd(Prograde), game_running); }
                if (isDown(SDL_SCANCODE_N)) { ship->Command(ShipCmd(Retrograde), game_running); }

                if (isDown(SDL_SCANCODE_R)) { ship->Command(ShipCmd(ThrottleUp), game_running); }
                if (isDown(SDL_SCANCODE_F)) { ship->Command(ShipCmd(ThrottleDown), game_running); }
            }

            void physics_tick(float timeStep);

            // Advance the analytic sim clock by exactly the physics timestep
            // (dt * time_accel), matching physics_tick(dt * time_accel) below.
            // The frame tree's analytic motion must run on the same clock as
            // the ship's integration. The old 1/60.0 constant disagreed with
            // dt (1/50), so the physics clock ran 20% faster than the analytic
            // body positions and the ship systematically outran the planets.
            time += dt * time_accel;

            if(time_accel != 0) {
                sun->frame->UpdateOrbitRails(time, dt * time_accel);

                if(time_accel >= kRailsWarp) {
                    /* Rails warp: every ship coasts analytically (or sits
                       frozen on the ground) and the Bullet world is not
                       stepped at all -- O(ships) per tick instead of a
                       substep count that explodes with the accel. */
                    for(auto *s : ships) { s->railsTick(dt * time_accel); }
                } else {

                // per-ship SOI bookkeeping: each ship tracks its own
                // position in the shared frame tree (an idle ship can
                // cross a boundary while we fly another one). Railed
                // ships advance their analytic conic here instead --
                // exact for any step size, at any time accel.
                for(auto *s : ships) {
                    if(s->onRails) { s->railsTick(dt * time_accel); }
                    else { s->switchFrames(); }
                }

                // Integrate the (time-accelerated) step in substeps,
                // re-applying gravity + the rotating-frame fictitious forces
                // + the engine thrust + the armed rotation commands before
                // EACH substep. Two reasons:
                //  1. Bullet clears accumulated forces at the end of every
                //     stepSimulation call, so applying gravity once and then
                //     stepping multiple substeps would leave the ship
                //     force-free for all but the first substep.
                //  2. Re-applying per substep keeps the central-force
                //     direction and the velocity-dependent Coriolis term
                //     accurate across the step instead of frozen at the
                //     step's start.
                // Keep >=3 substeps so low-accel behavior matches the old
                // 3-substep step, and grow the count so the substep stays
                // <= kMaxSubStep at high time-accel.
                const double step = dt * time_accel;
                const double kMaxSubStep = 0.1;
                int n = 3;
                int need = (int)(step / kMaxSubStep + 0.5);
                if (need > n) { n = need; }
                if (n > 2000) { n = 2000; }
                const double h = step / n;
                for (int i = 0; i < n; i++) {
                    // every NON-RAILED ship feels its own gravity/thrust/
                    // rotation each substep; physics_tick then steps the
                    // shared Bullet world all of them at once. Railed ships
                    // have no bodies in the world -- their conic already
                    // advanced this tick in railsTick.
                    for(auto *s : ships) {
                        if(s->onRails) { continue; }
                        s->processGravity();
                        s->applyThrustForce();
                        s->applyRotationForce(h);
                    }
                    physics_tick(h);
                }

                } // end physics-warp branch (time_accel < kRailsWarp)

                /* --spin-log (or --radial-test): spin diagnostics, once per
                   0.5 s of sim time (after the last substep's solve, so the
                   reported impulses are that solve's). */
                if(spin_log_enabled || !radial_test.empty()) {
                    static double last_spin_log = -1e30;
                    if(time - last_spin_log >= 0.5) {
                        last_spin_log = time;
                        spin_log(ship, time);
                    }
                }
            }

            // --orbit-log: orbital elements, fit in the body's inertial
            // frame, where the ship's trajectory is a Kepler conic.
            if(orbit_log) {
                const Uint32 now_ms = SDL_GetTicks();
                if(now_ms - orbit_log_last_ms >= orbit_log_interval_ms) {
                    orbit_log_last_ms = now_ms;

                    const double mu = ship->m_parent->mu;
                    glm::dvec3 o_pos = ship->get_center_of_mass();
                    glm::dvec3 o_vel = ship->GetVel();
                    if(ship->frame->isRotFrame()) {
                        Frame *inertial = ship->frame->getNonRotFrame();
                        o_vel += ship->frame->GetStasisVelocity(o_pos);
                        o_vel = ship->frame->GetOrientRelTo(inertial) * o_vel
                              + ship->frame->GetVelocityRelTo(inertial);
                        o_pos = ship->frame->GetOrientRelTo(inertial) * o_pos
                              + ship->frame->GetPositionRelTo(inertial);
                    }
                    OrbitElements o = computeOrbitElements(o_pos, o_vel, mu);
                    printf("[orbitlog] t=%.1fs frame=\"%s\" r=%.6g m v=%.6g m/s "
                           "sma=%.6g m ecc=%.6g peri=%.6g m apo=%.6g m "
                           "inc=%.4f deg T=%.6g s ttAp=%.6g s ttPe=%.6g s "
                           "|h|=%.6f m2/s E=%.6f J/kg\n",
                           time, ship->frame->name.c_str(), o.distance, o.speed,
                           o.semi_major, o.ecc, o.periapsis, o.apoapsis,
                           glm::degrees(o.inclination), o.period,
                           o.time_to_apo, o.time_to_peri,
                           o.ang_momentum, o.energy);
                    fflush(stdout);
                }
            }

            // --dbg-log: ship pos/alt/vel in its own frame
            if(dbg_log) {
                const Uint32 now_ms = SDL_GetTicks();
                if(now_ms - orbit_log_last_ms >= orbit_log_interval_ms) {
                    glm::dvec3 p = ship->get_center_of_mass();
                    glm::dvec3 v = ship->GetVel();
                    double r = glm::length(p);
                    double alt = r - ship->m_parent->GetTerrainHeight(glm::normalize(p));
                    printf("[dbg] t=%.1fs pos=[%.1f %.1f %.1f] alt=%.1f m "
                           "vel=[%.2f %.2f %.2f] |v|=%.2f m/s\n",
                           time, p.x, p.y, p.z, alt, v.x, v.y, v.z,
                           glm::length(v));
                    fflush(stdout);
                }
            }

            accumulator -= dt;
        }

        redraw = true;

        /*
          RENDERING
        */
        if(redraw == true) {
            check_gl_error();
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplSDL2_NewFrame();
            ImGui::NewFrame();
            check_gl_error();

            if(poly_mode == true) {
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                check_gl_error();
            }

            postfx->Begin();  // no-op unless --postfx effects are active
            display.Clear(0, 0, 0, 1);

            com = ship->get_center_of_mass();

            if(camMode == CAM_ORBIT) {
                camera->Follow(focusWorldPos(focusBody));
            }

            // Render frame origin = the active ship's COM (both are in
            // ship->frame, the render frame). The view is built there and
            // the Draw sites shift geometry by -renderOrigin, so the
            // float32 cast works on ship-relative numbers.
            camera->renderOrigin = com;
            camera->ComputeView();

            /*
              standard 3d stuff drawn here
            */

            if(world_drawing == true) {
                // one per home body; StaticBuilding::Draw culls itself when
                // the active ship is not on that body
                for(auto &kv : space_ports) {
                    kv.second->Draw(camera, ship->m_parent, ship->frame);
                }
                // render frame = the active ship's frame; idle ships in a
                // different frame are transformed into it in Vehicle::Draw
                for(auto *s : ships) { s->Draw(camera, ship->frame); }
            }

            for(auto&& planet : planets) {
                if(planet == ship->m_parent) {
                    //this is the planet we're on. This means its position is always 0, 0, 0

                    if(ship->frame->isRotFrame()) {
                        // we're in its rotational frame
                        planet->transform = glm::dmat4(1.0);
                    }
                    else {
                        // we're in its inertial frame
                        planet->transform = glm::dmat4(planet->frame->getRotFrame()->orient);
                    }
                }
                else {
                    // other planets
                    glm::dvec3 translate = planet->frame->GetPositionRelTo(ship->frame);
                    planet->transform = glm::translate(translate) * glm::dmat4(planet->frame->getRotFrame()->orient);
                }
            }

            for(auto&& planet : planets) {
                planet->Update(camera);
                if(world_drawing == true) {
                    planet->Draw(camera, sun, ship->frame);
                }
            }

            /*
              end 3d stuff drawn here
            */

            const double mu = ship->m_parent->mu;

            glm::dvec3 getRelAxis_(Body *body, int n);
            // surf pos??
            const glm::dvec3 pos = com;
            /* orbital velocity */
            glm::dvec3 vel = ship->GetVel();

            // The orbit is a Kepler conic in the body's INERTIAL (non-rotating)
            // frame — that is the frame the spawn/switching code targets and
            // the frame in which the ship's trajectory is a conic.
            glm::dvec3 orbit_pos = pos;
            glm::dvec3 orbit_vel = vel;
            if(ship->frame->isRotFrame() == true) {
                Frame *inertial = ship->frame->getNonRotFrame();
                orbit_vel += ship->frame->GetStasisVelocity(orbit_pos);
                orbit_vel = ship->frame->GetOrientRelTo(inertial) * orbit_vel + ship->frame->GetVelocityRelTo(inertial);
                orbit_pos = ship->frame->GetOrientRelTo(inertial) * orbit_pos + ship->frame->GetPositionRelTo(inertial);
            }

            // Surface-relative state: the ship's position/velocity in the
            // ROTATING frame (i.e. relative to the ground).
            glm::dvec3 surf_pos = pos;
            glm::dvec3 surf_vel = vel;

            if(ship->frame->isRotFrame() == false and
               ship->frame->hasRotFrame() == true) {
                Frame *rot = ship->frame->getRotFrame();
                surf_pos = ship->frame->GetOrientRelTo(rot) * pos;
                surf_vel = ship->frame->GetOrientRelTo(rot) * vel
                         - rot->GetStasisVelocity(surf_pos);
            }

            const OrbitElements o = computeOrbitElements(orbit_pos, orbit_vel, mu);
            const double distance = o.distance;
            const double speed = o.speed;

            // Telemetry: e and |h| are the two conserved 2-body constants,
            // so a drifting plot = integrator drift; steps = burns/staging.
            energy_series.push(time, o.energy);
            angmom_series.push(time, o.ang_momentum);

            const glm::dvec3 up = getRelAxis_(ship->controller, 1);
            const glm::dvec3 facing = getRelAxis_(ship->controller, 2);
            const glm::dvec3 other = getRelAxis_(ship->controller, 0);

            const glm::dvec3 facing_dir = glm::normalize(facing);
            const glm::dvec3 vel_dir = glm::normalize(vel);

            const glm::dvec3 _up = glm::normalize(pos);
            const glm::dvec3 _north = glm::normalize(projectVecOntoPlane(glm::dvec3(0, 1, 0), _up));
            const glm::dvec3 _east = glm::cross(_up, _north);

            const double ver_speed = glm::length(glm::proj(surf_vel, pos)); // m/s
            const double hor_speed2 = glm::length(projectVecOntoPlane(surf_vel, _up)); // m/s

            const glm::dvec3 groundHed = glm::normalize(projectVecOntoPlane(facing, _up));

            const double hedNorth = glm::dot(groundHed, _north);
            const double hedEast = glm::dot(groundHed, _east);
            const double heading = wrapAngleToPositive(atan2(hedEast, hedNorth));

            const double yaw = heading;
            const double pitch = asin(glm::dot(_up, facing));
            const double roll =
                glm::orientedAngle(glm::normalize(projectVecOntoPlane(-pos, glm::normalize(facing))),
                                   glm::normalize(-up),
                                   glm::normalize(facing));

            // ImGui::Text("pos: %.2f %.2f %.2f", pos.x, pos.y, pos.z);
            // ImGui::Text("facing: %.2f %.2f %.2f", facing.x, facing.y, facing.z);
            // ImGui::Text("up: %.2f %.2f %.2f", up.x, up.y, up.z);
            // ImGui::Text("other: %.2f %.2f %.2f", other.x, other.y, other.z);
            // ImGui::Text("Ground hed: %.2f %.2f %.2f", groundHed.x, groundHed.y, groundHed.z);
            // ImGui::Text("Pitch: %.2f", glm::degrees(pitch));
            // ImGui::Text("Heading: %.2f", glm::degrees(heading));
            // ImGui::Text("up: %.2f, %.2f, %.2f", up.x, up.y, up.z);
            // ImGui::Text("facing: %.2f, %.2f, %.2f", facing.x, facing.y, facing.z);

            const glm::dvec3 dir = glm::normalize(surf_pos);

            const double longitude = atan2(dir.x, dir.z);
            const double latitude = asin(dir.y);

            if(draw_starfield) {
                skybox.Draw(camera, skyboxshader, sun->frame->GetOrientRelTo(ship->frame));
            }

            // Atmosphere rims: transparent Fresnel shells, drawn after the
            // skybox (the starfield is the background) so the rim ring blends
            // over it and the horizon haze blends over the already-drawn
            // terrain. Depth-write off; no-ops for bodies without an
            // atmosphere. See reports/atmosphere2026_08_25.
            if(world_drawing == true) {
                for(auto&& planet : planets) {
                    planet->DrawAtmosphere(camera, sun, ship->frame);
                }
            }

            /* draw engine plume */
            glm::dmat4 View = camera->GetView();
            glm::mat4 Projection = camera->GetProjection();
            if(ship->m_thrust > 0) {
                for(size_t t = 0; t < ship->m_thrusters.size(); t++) {
                    /* the plume mesh is authored for the base part
                       (radius 1 m, height 2 m): scale it to this thruster's
                       size so the tail lands on the engine tail (-h/2) */
                    const glm::dvec2 &dim = ship->m_thrusterDims[t];
                    glm::dmat4 Model = ship->m_thrusters[t]->model_matrix
                        * glm::dmat4(glm::dmat3(dim.x, 0.0, 0.0,
                                                 0.0, dim.x, 0.0,
                                                 0.0, 0.0, dim.y / 2.0));
                    // shifted into the render frame, like the view
                    glm::mat4 ModelViewFloat = View * glm::translate(-camera->GetRenderOrigin()) * Model;
                    engine_plume_model->shader->Bind();
                    engine_plume_model->shader->setUniform_mat4(0, Projection * ModelViewFloat);
                    engine_plume_model->shader->setUniform_mat4(1, glm::mat4(1.0)); // identity (GLM 1.0.0+: default ctor is zero)
                    engine_plume_model->shader->setUniform_vec3(2, glm::vec3(1, 1, 1));

                    glActiveTexture(GL_TEXTURE0);
                    glBindTexture(GL_TEXTURE_2D, engine_plume_model->texture->id);
                    glEnable(GL_BLEND);
                    glBlendFunc(GL_ONE, GL_ONE);
                    glDisable(GL_CULL_FACE);
                    engine_plume_model->mesh->Draw();
                    glEnable(GL_CULL_FACE);
                    glDisable(GL_BLEND);
                    glBindTexture(GL_TEXTURE_2D, 0);
                }
            }
            /* end draw engine plume */

            glDisable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            front_indicator->pos = facing;
            front_indicator->Draw(camera, M_PI /* <- ?? */ + roll);
            prograde_indicator->pos = vel;
            prograde_indicator->Draw(camera, M_PI);
            retrograde_indicator->pos = - vel;
            retrograde_indicator->Draw(camera, M_PI);
            radial_in_indicator->pos = - pos;
            radial_in_indicator->Draw(camera, M_PI);
            radial_out_indicator->pos = pos;
            radial_out_indicator->Draw(camera, M_PI);
            normal_plus_indicator->pos = glm::cross(pos, vel);
            normal_plus_indicator->Draw(camera, M_PI);
            normal_minus_indicator->pos = -glm::cross(pos, vel);
            normal_minus_indicator->Draw(camera, M_PI);
            // horizon_indicator->pos = groundHed;
            // horizon_indicator->Draw(camera, M_PI);

            if(draw_skylines) {
                glLineWidth(4);
                lineshader->Bind();
                lineshader->setUniform_mat4(0, glm::dmat4(camera->GetProjection()) * glm::dmat4(glm::dmat3(camera->GetView())));
                // XZ plane (flat / orbital-equatorial reference): green
                lineshader->setUniform_vec4(1, glm::vec4(0, 1, 0, 0.5));
                skyline_xz->Draw(GL_LINE_LOOP);
                // XY plane (vertical / meridian reference): magenta
                lineshader->setUniform_vec4(1, glm::vec4(1, 0, 1, 0.5));
                skyline_xy->Draw(GL_LINE_LOOP);
            }

            glDisable(GL_BLEND);
            glEnable(GL_DEPTH_TEST);

            if(physics_debug_drawing == true) {
                glDisable(GL_DEPTH_TEST);
                debug_draw(camera);
                glEnable(GL_DEPTH_TEST);
            }

            postfx->End();  // no-op unless --postfx effects are active

            /*
              ImGui stuff below
            */

            glUseProgram(0);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            if(poly_mode == true) {
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            }

            /* HUD trio: fixed windows (no move, no resize, re-placed every
               frame so they track the viewport). "HUD Orbit" sits below
               "HUD Altitude". */
            ui::Window("HUD Altitude", o_hud, [&] {
                ImGui::PushFont(bigger);
                int terrain_height = (int)(distance - ship->m_parent->GetTerrainHeight(glm::normalize(pos)));
                ImGui::Text("%08dm", terrain_height);
                ImGui::PopFont();
            });
            {
                ui::Options ohud = o_hud;
                ohud.below = "HUD Altitude";
                ui::Window("HUD Orbit", ohud, [&] {
                    ImGui::PushFont(bigger);
                    ImGui::Text(o.energy < 0 ? "Elliptic Orbit" : "Hyperbolic Orbit");
                    ImGui::PopFont();
                });
            }
            ui::Window("HUD Speed", o_hud, [&] {
                ImGui::PushFont(bigger);
                ImGui::Text("%06dm/s", (int)speed);
                ImGui::PopFont();
            });

            /* Window list (single source of truth: the ui_windows table)
               plus the Top-HUD group switch. */
            ui::Window("Windows", o_menu, [&] {
                ImGui::Spacing();
                for(auto &w : ui_windows) {
                    bool open = ui::IsOpen(w.name);
                    if(ImGui::Checkbox(w.label, &open)) {
                        ui::SetOpen(w.name, open);
                    }
                }
                bool hud = ui::IsOpen(hud_windows[0]);
                if(ImGui::Checkbox("Top HUD", &hud)) {
                    for(auto *h : hud_windows) {
                        ui::SetOpen(h, hud);
                    }
                }
            });

            ui::Window("Game Debug Info", o_debug, [&] {
                ImGui::Checkbox("Physics debug draw", &physics_debug_drawing);
                ImGui::Checkbox("World draw", &world_drawing);
                ImGui::Checkbox("Starfield", &draw_starfield);
                ImGui::Checkbox("Reference circles", &draw_skylines);
                ImGui::Text("Time: %f", time);
                if(sys.home && sys.home->cal.valid()) {
                    CalTime ct = sys.home->cal.at(time);
                    if(ct.has_year) {
                        ImGui::Text("Clock:  Yr %d  Mo %d  Day %d  %02d:%02d:%02d  (%s time)",
                                    ct.year, ct.month, ct.day, ct.hh, ct.mm, ct.ss,
                                    sys.home->name.c_str());
                    } else {
                        ImGui::Text("Clock:  Day %d  %02d:%02d:%02d  (%s time)",
                                    ct.day, ct.hh, ct.mm, ct.ss,
                                    sys.home->name.c_str());
                    }
                }
                // Local date + time on the body the ship is currently in,
                // when it's not the home planet (e.g. the Moon's own day).
                TerrainBody *local_body = (ship && ship->frame)
                                        ? ship->frame->body : nullptr;
                if(local_body && local_body != sys.home &&
                   local_body->cal.valid()) {
                    CalTime lt = local_body->cal.at(time);
                    if(lt.has_year) {
                        ImGui::Text("Local:  %s  Yr %d  Mo %d  Day %d  %02d:%02d:%02d",
                                    local_body->name.c_str(),
                                    lt.year, lt.month, lt.day,
                                    lt.hh, lt.mm, lt.ss);
                    } else {
                        ImGui::Text("Local:  %s  Day %d  %02d:%02d:%02d",
                                    local_body->name.c_str(),
                                    lt.day, lt.hh, lt.mm, lt.ss);
                    }
                }
                ImGui::Text("Patches: %d", ship->m_parent->CountPatches());
                ImGui::Text("Cam speed: %d", cam_speed);
                ImGui::Text("Time Accel: %d%s", time_accel,
                            time_accel >= kRailsWarp ? " (rails)" : "");
                ImGui::Text("Camera altitude: %0.f",
                            glm::length(camera->GetPos()) - ship->m_parent->GetTerrainHeight(glm::normalize(camera->GetPos())));
                ImGui::Text("Camera ASL: %0.f", glm::length(camera->GetPos()) - ship->m_parent->radius);
                ImGui::Text("Camera Pos: %.0f %.0f %0.f", camera->GetPos().x, camera->GetPos().y, camera->GetPos().z);
                ImGui::Text("Cam forward: %.2f %.2f %.2f",
                            camera->forward.x, camera->forward.y, camera->forward.z);
                if(ImGui::Button("Print camera pose (CLI args)")) {
                    // Copy-paste the printed line after ./osp to relaunch at this view.
                    // pos/forward/up are world / ship-frame coordinates.
                    printf("Camera pose:\n");
                    printf("--free-cam-pos %.9g %.9g %.9g "
                           "--free-cam-fwd %.9g %.9g %.9g "
                           "--free-cam-up %.9g %.9g %.9g\n",
                           camera->pos.x, camera->pos.y, camera->pos.z,
                           camera->forward.x, camera->forward.y, camera->forward.z,
                           camera->up.x, camera->up.y, camera->up.z);
                }
                ImGui::Text("Home distance: %f",
                            glm::length(ship->GetPositionRelTo(ship->controller,
                                                                ship_homes[activeIdx]->frame)));
                ImGui::Text("Pos: %.3fkm", distance / 1000);
                ImGui::Text("xyz(%0.f, %0.f, %0.f)", pos.x, pos.y, pos.z);
                ImGui::Text("Vel: %.3fm/s", speed);
                ImGui::Text("xyz(%0.f, %0.f, %0.f)", vel.x, vel.y, vel.z);
            });

            // Labels are abbreviated to <= 3 chars and right-padded to the
            // same width so the values start at a tidy column.
            ui::Window("ORBITAL", o_orbit, [&] {
                ImGui::Text("Vel: %.1fm/s", speed);
                ImGui::Text("Alt: %.1fm", distance);
                if(o.ecc < 1.0) {
                    ImGui::Text("ApA: %.1fm", o.apoapsis);
                    ImGui::Text("ApT: %.1fs", o.time_to_apo);
                } else {
                    ImGui::Text("ApA: escape (no apoapsis)");
                }
                ImGui::Text("PeA: %.1fm", o.periapsis);
                if(o.time_to_peri >= 0.0) {
                    ImGui::Text("PeT: %.1fs", o.time_to_peri);
                }
                if(o.period > 0.0) {
                    ImGui::Text("  T: %.1fs", o.period);
                }
                ImGui::Text("Inc: %.2f", glm::degrees(o.inclination));
                ImGui::Text("Ecc: %f", o.ecc);
                ImGui::Text("SMa: %.1fm", o.semi_major);
                ImGui::Text("LAN: %.2f", glm::degrees(o.raan));
                ImGui::Text("LPe: %.2f", glm::degrees(o.arg_periapsis));
                double prograde_angle = glm::angle(facing_dir, vel_dir);
                double retrograde_angle = glm::angle(facing_dir, - vel_dir);
                ImGui::Text("Prg: %.2f", glm::degrees(prograde_angle));
                ImGui::Text("Rtg: %.2f", glm::degrees(retrograde_angle));
                ImGui::Text("Eng: %.2f J", o.energy);
            });

            // Initial size comes from o_telemetry.initial_size (the plots
            // need real estate; content-fit would clip them).
            ui::Window("TELEMETRY", o_telemetry, [&] {
                // Two separate plots: e (~1e5) and |h| (~1e11) differ by
                // ~6 orders of magnitude, so sharing one axis would flatten
                // e to a line and hide the drift we're looking for.
                if(energy_series.count > 1) {
                    const int n = energy_series.stage();
                    ImPlot::SetNextAxesToFit();  // live auto-fit, both axes
                    if(ImPlot::BeginPlot("specific orbital energy (J/kg)")) {
                        ImPlot::SetupAxis(ImAxis_X1, "t (s)");
                        ImPlot::PlotLine("e", energy_series.t_arr(), energy_series.v_arr(), n);
                        ImPlot::EndPlot();
                    }
                }
                if(angmom_series.count > 1) {
                    const int n = angmom_series.stage();
                    ImPlot::SetNextAxesToFit();  // live auto-fit, both axes
                    if(ImPlot::BeginPlot("angular momentum (m^2/s)")) {
                        ImPlot::SetupAxis(ImAxis_X1, "t (s)");
                        ImPlot::PlotLine("|h|", angmom_series.t_arr(), angmom_series.v_arr(), n);
                        ImPlot::EndPlot();
                    }
                }
            });

            // Labels right-padded to 3 chars, same as ORBITAL.
            ui::Window("SURFACE", o_surface, [&] {
                ImGui::Text("Alt: %.1fm", distance - ship->m_parent->GetTerrainHeight(glm::normalize(pos)));
                ImGui::Text("ASL: %.1fm", distance - ship->m_parent->radius);
                ImGui::Text(" Vs: %.2fm/s", ver_speed);
                ImGui::Text(" Hs: %.2fm/s", hor_speed2);
                ImGui::Text("Lat: %.4f", glm::degrees(latitude));
                ImGui::Text("Lon: %.4f", glm::degrees(longitude));
                ImGui::Text(" Pt: %.2f", glm::degrees(pitch));
                ImGui::Text("  R: %.2f", glm::degrees(roll));
                ImGui::Text("Hdg: %.2f", glm::degrees(yaw));
            });

            if(ships.size() > 1) {
                ui::Window("SHIPS", o_ships, [&] {
                for(size_t i = 0; i < ships.size(); i++) {
                    const bool active = ((int)i == activeIdx);
                    if(ImGui::Selectable(ships[i]->name.c_str(), active)) {
                        select_ship((int)i);
                    }
                }
                ImGui::Text("tab - cycle");
                });
            }

            ui::Window("VESSEL", o_vessel, [&] {
                ImGui::Text("Ship: %s", ship->name.c_str());
                ImGui::Text("Stage: %d / %d  (SPACE to drop)",
                            ship->activeStage(), ship->numStages());
                ImGui::Text("Reference frame: %s", ship->frame->name.c_str());
                ImGui::Text("Reference frame type: %s", ship->frame->isRotFrame() ? "Rotational" : "Inertial");
                ImGui::Text("Mass: %.3fkg", ship->getMass());
                ImGui::Text("Delta-v: %.1fm/s", ship->getDeltaV());
                ImGui::Text("Thrust Util: %.0f%%", ship->thruster_util * 100);
                ImGui::Text("Thrust: %.2fN", ship->getThrust());
                ImGui::Text("Current TWR: %.2f/%.2f", ship->getTWR(), ship->getFullThrustTWR());
                ImGui::Text("Max TWR: %.2f", ship->getMaxTWR());
                ImGui::Text("Wheel torque: %.0fN m", ship->GetWheelTorque());
                ImGui::Text("Angular rate: %.2fdeg/s",
                            glm::degrees(glm::length(GetAngVelocity(ship->controller))));
            });
            ui::Window("SHIP PARTS", o_parts, [&] {
                int i = 0;
                for(auto&& part : ship->parts) {
                    ImGui::Text("Part #%d  (stage %d)", i, ship->partStages[i]);
                    ImGui::Separator();
                    ImGui::Text("Name: %s", ship->partDefs[i]->name.c_str());
                    ImGui::Text("Mass: %.3fkg", part->mass);
                    ImGui::Text("Hydrogen: %.3fkg/%.3fkg",
                                ship->partResources[i].current[(int)ResourceType::Hydrogen],
                                ship->partResources[i].capacity[(int)ResourceType::Hydrogen]);
                    ImGui::Text("LOX: %.3fkg/%.3fkg",
                                ship->partResources[i].current[(int)ResourceType::LOX],
                                ship->partResources[i].capacity[(int)ResourceType::LOX]);
                    ImGui::Spacing();
                    i++;
                }
            });

            ui::Window("Controls help", o_controls, [&] {
                ImGui::Text("Game");
                ImGui::Separator();
                ImGui::Text("p - toggle wireframe mode");
                ImGui::Text(", - decrease time acceleration");
                ImGui::Text(". - increase time acceleration");
                ImGui::Text("k - decrease camera speed");
                ImGui::Text("l - increase camera speed");
                ImGui::Text("c - switch mode: orbit (flying) <-> free (exploring)");
                ImGui::Text("g - orbit mode: cycle target (ship/sun/planet/moon)");
                if(ships.size() > 1) { ImGui::Text("tab - switch active ship"); }
                ImGui::Text("mouse - UI (hold RMB over 3D to look, both modes)");
                ImGui::Text("wheel - zoom (orbit mode)");
                ImGui::Spacing();
                ImGui::Text("Orbit mode (flying the ship)");
                ImGui::Separator();
                ImGui::Text("w/s - pitch up/down");
                ImGui::Text("a/d - yaw left/right");
                ImGui::Text("q/e - roll left/right");
                ImGui::Text("i - fire ship engines");
                ImGui::Text("x - kill rotation");
                ImGui::Text("b - align prograde");
                ImGui::Text("n - align retrograde");
                ImGui::Text("r/f - throttle up/down");
                ImGui::Text("SPACE - separate the active stage");
                ImGui::Spacing();
                ImGui::Text("Free mode (exploring)");
                ImGui::Separator();
                ImGui::Text("w/s - forward/back");
                ImGui::Text("a/d - strafe");
                ImGui::Text("q/e - roll");
                ImGui::Text("shift/ctrl - up/down");
            });

            ui::Window("Autopilot", o_autopilot, [&] {
                ImGui::Button("Prograde");
                ImGui::Button("Retrograde");
                ImGui::Button("Radial-in");
                ImGui::Button("Radial-out");
                ImGui::Button("Normal");
                ImGui::Button("Anti-normal");
            });

            ui::Window("RESOURCES", o_resources, [&] {
                // aggregate across the active ship's parts (any ship layout)
                float h_cur = 0, h_cap = 0, l_cur = 0, l_cap = 0;
                for(size_t i = 0; i < ship->partResources.size(); i++) {
                    h_cur += ship->partResources[i].current[(int)ResourceType::Hydrogen];
                    h_cap += ship->partResources[i].capacity[(int)ResourceType::Hydrogen];
                    l_cur += ship->partResources[i].current[(int)ResourceType::LOX];
                    l_cap += ship->partResources[i].capacity[(int)ResourceType::LOX];
                }
                const float hydrogen_frac = (h_cap > 0) ? h_cur / h_cap : 0.0f;
                const float lox_frac = (l_cap > 0) ? l_cur / l_cap : 0.0f;

                ImGui::ProgressBar(hydrogen_frac, ImVec2(-1, 0), "Hydrogen");
                ImGui::ProgressBar(lox_frac, ImVec2(-1, 0), "LOX");
                ImGui::ProgressBar(0.13, ImVec2(-1, 0), "Hydrazine");
                ImGui::ProgressBar(0.45, ImVec2(-1, 0), "Electric charge");
                ImGui::ProgressBar(0.75, ImVec2(-1, 0), "Oxygen");
                ImGui::ProgressBar(0.83, ImVec2(-1, 0), "Water");
                ImGui::ProgressBar(0.94, ImVec2(-1, 0), "Food");
            });

            ui::Window("Orbital map", o_map, [&] {
                if(o.ecc >= 1.0) {
                    ImGui::Text("Escape trajectory (Ecc %f): no closed orbit to map.", o.ecc);
                } else {
                    ImVec2 pts[26];
                    // ImVec2 planet[26];
                    // ImGui::Text("%.1f %.1f", center.x, center.y);
                    int i = 0;
                    ImU32 color = ImGui::GetColorU32(ImVec4(255,255,255,255));
                    ImU32 color2 = ImGui::GetColorU32(ImVec4(255,0,0,255));
                    const ImVec2 p = ImGui::GetCursorScreenPos();
                    double E = 0;
                    static float div = 6000.0;
                    for(i = 0; i < 26; i++) {
                        double r = o.semi_major * (1 - o.ecc * cos(E));
                        // ImGui::Text("%.0f", r);
                        double argX = cos(E) - o.ecc;
                        double argY = sqrt(1 - (o.ecc * o.ecc)) * sin(E);
                        double phi = atan2(argY, argX);
                        pts[i].x = 200 + p.x + (r / div * cos(phi));
                        pts[i].y = 200 + p.y + (r / div * sin(phi));
                        // planet[i].x = 200 + p.x + (600000 / div * cos(phi));
                        // planet[i].y = 200 + p.y + (600000 / div * sin(phi));
                        E += 2 * M_PI / 25;
                    }

                    double argX = cos(o.ecc_anomaly) - o.ecc;
                    double argY = sqrt(1 - (o.ecc * o.ecc)) * sin(o.ecc_anomaly);
                    double phi = atan2(argY, argX);

                    ImVec2 ship_p = { float(200 + p.x + distance / div * cos(phi)),
                                      float(200 + p.y + distance / div * sin(phi)) };

                    ImVec2 raan_p = { float(200 + p.x + 100 * cos(o.raan)),
                                      float(200 + p.y + 100 * sin(o.raan)) };

                    /* incorrect */
                    ImVec2 peri_p = { float(200 + p.x + o.periapsis / div * cos(o.arg_periapsis - M_PI / 2)),
                                      float(200 + p.y + o.periapsis / div * sin(o.arg_periapsis - M_PI / 2)) };

                    ImVec2 apo_p = { float(200 + p.x + o.apoapsis / div * cos(o.arg_periapsis + M_PI / 2)),
                                     float(200 + p.y + o.apoapsis / div * sin(o.arg_periapsis + M_PI / 2)) };

                    ImGui::GetWindowDrawList()->AddPolyline(&pts[0], 26, color, false, 1.0);

                    ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(200 + p.x, 200 + p.y),
                                                                ship->m_parent->radius / div,
                                                                color2, 26);

                    ImGui::GetWindowDrawList()->AddCircleFilled(ship_p, 5, color);
                    ImGui::GetWindowDrawList()->AddLine(ImVec2(200 + p.x, 200 + p.y), ship_p, color);

                    // ImGui::GetWindowDrawList()->AddCircle(ImVec2(200 + p.x, 200 + p.y), 3 * 600000 / div, color2);
                    // ImGui::GetWindowDrawList()->AddCircleFilled(raan_p, 5, color);
                    // ImGui::GetWindowDrawList()->AddCircleFilled(peri_p, 5, color);
                    // ImGui::GetWindowDrawList()->AddCircleFilled(apo_p, 5, color);
                    // ImGui::GetWindowDrawList()->AddLine(p, ImVec2(p.x + 10,p.y + 10), color);

                    ImGui::SliderFloat("Scale", &div, 5000, 100000, "");
                    ImGui::Text("True anomaly: %.2f", o.true_anomaly);
                    ImGui::Text("Eccentric anomaly: %.2f", o.ecc_anomaly);
                }
            });

            // Main menu: F10 toggle. Fixed, so it stays centered and
            // tracks viewport resizes. Drawn last so it sits on top.
            // Buttons have an explicit width: -1 (full width) inside an
            // auto-resizing window collapses the window to a sliver.
            ui::Window("Main Menu", o_mainmenu, [&] {
                if(ImGui::Button("Toggle windows", ImVec2(160.0f, 0.0f))) {
                    ui_visible = !ui_visible;
                    for(auto &w : ui_windows) {
                        ui::SetOpen(w.name, ui_visible && w.opts.default_open);
                    }
                    for(auto *h : hud_windows) {
                        ui::SetOpen(h, false);
                    }
                }
                if(ImGui::Button("Reset windows", ImVec2(160.0f, 0.0f))) {
                    ui::ResetGui();
                }
                if(ImGui::Button("Quit game", ImVec2(160.0f, 0.0f))) {
                    running = false;
                }
            });

            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            if(screenshot_requested == true) {
                char fname[256];
                time_t now = ::time(nullptr);
                struct tm tm;
                localtime_r(&now, &tm);
                char stamp[32];
                strftime(stamp, sizeof(stamp), "%Y_%m_%d_%H_%M_%S", &tm);
                snprintf(fname, sizeof(fname), "./tmp/osp_%s.png", stamp);
                mkdir("./tmp", 0755);
                if(display.SaveScreenshot(fname)) {
                    screenshot_count++;
                }
                screenshot_requested = false;
            }

            display.SwapBuffers();
            check_gl_error();
        }

        // --frame-cap: burn the rest of the frame budget. Without this the
        // iteration spins at full speed whenever the swap isn't vsync-gated
        // (paused VAB, headless, vsync off) -- 100% of a core doing nothing.
        if (cap_ms > 0) {
            const Uint32 used_ms = SDL_GetTicks() - iter_start_ms;
            if (used_ms < (Uint32)cap_ms) {
                SDL_Delay(cap_ms - used_ms);
            }
        }
    }

    for(auto &kv : space_ports) { delete kv.second; }
    for(auto *s : ships) { delete s; }

    for(auto&& body : sys.bodies) { delete body; }

    delete partsshader;
    delete sunshader;
    delete terrainshader;
    delete atmosphereshader;
    delete billboardshader;
    delete skyboxshader;
    delete postfx;

    delete front_indicator;
    delete prograde_indicator;
    delete retrograde_indicator;

    delete front_indicator_texture;
    delete prograde_indicator_texture;
    delete retrograde_indicator_texture;

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    return 0;
}
