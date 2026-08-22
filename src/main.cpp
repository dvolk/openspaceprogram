// Open Space Program

#include <stdio.h>
#include <algorithm>
#include <chrono>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <map>

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
#include <nlohmann/json.hpp>
#include "billboard.h"
#include "texture.h"
#include "skybox.h"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include "../middleware/imgui/imgui.h"
#include "../middleware/imgui/backends/imgui_impl_sdl2.h"
#include "../middleware/imgui/backends/imgui_impl_opengl3.h"

#include <CLI11/CLI11.hpp>

ImFont *bigger;
bool planetsWindow = false;

static const int DISPLAY_WIDTH = 1920; // should be cli args
static const int DISPLAY_HEIGHT = 1080;
static const int FPS = 60; // TODO use FPS from display?

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
    void Draw(const Camera* camera, const glm::dmat4& transform, const glm::vec3 & sunlightVec);
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
    glm::dmat4 transform = glm::dmat4(1.0);
    glm::vec3 sunlightVec;
    int dbg_drew_patches;

    ~TerrainBody() {
        for(int i = 0; i < 6; i++) { delete patches[i]; }
        delete frame;
        delete rot_frame;
    }

    COLOUR (*colour_func)(float v, float vmin, float vmax);

    Mesh *create_grid_mesh(bool has_collision, glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4);
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

    static glm::dvec3 SunlightDir(TerrainBody *planet, TerrainBody *sun,
                                  Frame *renderFrame) {
        const glm::dvec3 d_root = sun->frame->root_pos - planet->frame->root_pos;
        return -glm::normalize(sun->frame->GetOrientRelTo(renderFrame) * d_root);
    }

    void Draw(const Camera* camera, TerrainBody *sun, Frame *renderFrame) {
        double cam_dist = glm::length(camera->GetPos() - glm::dvec3(transform[3]));

        sunlightVec = glm::vec3(SunlightDir(this, sun, renderFrame));

        dbg_drew_patches = 0;
        for(auto&& patch : patches) {
            // patch isn't subdivided
            patch->Draw(camera, transform, sunlightVec);
        }

        if(planetsWindow) {
            // TODO move this out maybe?
            ImGui::Begin("Planets");
            ImGui::Text("%s", name.c_str());
            ImGui::Separator();
            ImGui::Text("Distance: %.0f", cam_dist);
            ImGui::Text("Rotational angle: %f", frame->getRotFrame()->ang);
            ImGui::Text("Orbital angle: %f", frame->getNonRotFrame()->orb_ang);
            ImGui::Text("Patches drawn: %d", dbg_drew_patches);
            ImGui::Spacing();
            ImGui::End();
        }
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
    bool has_collision = depth > max_depth;
    Mesh *grid_mesh = body->create_grid_mesh(has_collision, v0, v1, v2, v3);
    model->FromData(grid_mesh, shader, NULL);
    if(has_collision == true) {
        collision = addTerrainCollision(grid_mesh);
        printf("added terrain collision with %p\n", (void*)this);
    } else {
        collision = NULL;
    }
}

void GeoPatch::Draw(const Camera* camera, const glm::dmat4& transform, const glm::vec3& sunlightVec) {
    body->dbg_drew_patches++;

    if(kids[0] == NULL) {
        // patch isn't subdivided
        glm::vec4 color = glm::vec4(0.8, 0.8, 0.8, 1.0);
        model->shader->Bind();

        const glm::dmat4 & View = camera->GetView();
        // make sure View * Model happens with double precision
        glm::dmat4 ModelView = View * transform;
        glm::mat4 ModelViewFloat = ModelView;
        const glm::mat4 & Projection = camera->GetProjection();
        glm::mat4 MVP = Projection * ModelViewFloat;
        glm::mat4 ModelFloat = transform;

        model->shader->setUniform_mat4(0, MVP);
        model->shader->setUniform_mat4(1, ModelFloat);
        model->shader->setUniform_vec3(2, sunlightVec);
        model->shader->setUniform_vec4(3, color);

        model->mesh->Draw();
    }
    else {
        kids[0]->Draw(camera, transform, sunlightVec);
        kids[1]->Draw(camera, transform, sunlightVec);
        kids[2]->Draw(camera, transform, sunlightVec);
        kids[3]->Draw(camera, transform, sunlightVec);
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

enum class ResourceType {
    Hydrogen,
    LOX,
    EC,
    Oxygen,
    Water,
    Food,
    Num
};

struct ResourceContent {
    float current[(int)ResourceType::Num];
    float capacity[(int)ResourceType::Num];

    ResourceContent() {
        for(int i = 0; i < (int)ResourceType::Num; i++) {
            current[i] = 0;
            capacity[i] = 0;
        }
    }
};

enum class VesselPartType {
    Capsule,
    ReactionWheel,
    Engine
};

const char *VesselPartTypeStr(VesselPartType& p) {
    switch(p)
        {
        case VesselPartType::Capsule:{ return "Capsule"; }
            break;
        case VesselPartType::ReactionWheel:{ return "Reaction wheel"; }
            break;
        case VesselPartType::Engine:{ return "Engine block"; }
            break;
        default:{ assert(false); }
        }
}

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
    std::vector<Body *> parts;
    std::vector<ResourceContent> partResources;
    std::vector<VesselPartType> partTypes;

    TerrainBody *m_parent;
    Frame *frame;
    TerrainBody *sun = nullptr; // the star (light source); set in main
    float m_thrust;

    glm::dvec3 m_com;

    Body *controller;
    std::vector<Body *> m_thrusters;
    std::vector<Body *> m_reaction_wheels;
    std::vector<void *> constraints;

    float thruster_util = 1.0;
    float thrust_mag = 0.0;   /* N; armed once per tick by ApplyThrust, re-applied per substep */

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

    void attachDown(Body *part) {
        void *constraint = GlueTogether(parts.back(), part);
        parts.push_back(part);
        constraints.push_back(constraint);
    }

    void Detach() {
        void Detach(void *constraint);
        Detach(constraints.back());
    }

    void init() {
        partResources.resize(parts.size());
        controller = parts.back();
        NeverSleep(controller);
        for(size_t i = 0; i < parts.size(); i++) {
            if(partTypes[i] == VesselPartType::ReactionWheel) {
                m_reaction_wheels.push_back(parts[i]);
            }
            else if(partTypes[i] == VesselPartType::Engine) {
                partResources[i].capacity[(int)ResourceType::Hydrogen] = 1000;
                partResources[i].capacity[(int)ResourceType::LOX] = 1000;
                partResources[i].current[(int)ResourceType::Hydrogen] = 1000;
                partResources[i].current[(int)ResourceType::LOX] = 1000;
                m_thrusters.push_back(parts[i]);
            }
        }
    }

    /* returns true if fuel was consumed. amt is the kg consumed THIS tick
       (the caller scales the kg/s flow by the tick's simulated time). */
    bool consumeResourceMass(enum ResourceType type, float amt /* kg */) {
        int i = 0;
        for(auto&& partResource : partResources) {
            if(partResource.current[(int)type] >= amt) {
                partResource.current[(int)type] -= amt;
                parts[i]->mass -= amt; /* why does Body have mass at all? */
                void SetMass(Body *body, double newMass);
                SetMass(parts[i], parts[i]->mass);
                return true;
            }
            i++;
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
        return GetExhaustVelocity() * log(getMass() / (getMass() - remaining_fuel));
    }

    /* TODO should be cached per frame */
    float getMass() {
        float r = 0;
        for(auto&& part : parts) {
            r += part->mass;
        }
        return r;
    }

    float getThrust() {
        return GetMaxThrust() * thruster_util;
    }

    // current Thrust-to-weight ratio
    float getTWR() {
        return (thruster_util * GetMaxThrust()) / (getMass() * m_parent->g);
    }

    // full throttle TWR
    float getFullThrustTWR() {
        return GetMaxThrust() / (getMass() * m_parent->g);
    }

    // empty TWR
    float getMaxTWR() {
        float remaining_fuel = getFuelMass({ ResourceType::Hydrogen, ResourceType::LOX }); /* kg */
        return GetMaxThrust() / ((getMass() - remaining_fuel) * m_parent->g);
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
        if(thrust_mag == 0.0f) { return; }
        for(auto&& thruster : m_thrusters) {
            ApplyCentralForceForward(thruster, thrust_mag);
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
            for(auto&& wheel : m_reaction_wheels) {
                for(int a = 0; a < 3; a++) {
                    if(stick[a] == 0.0f) { continue; }
                    ApplyTorque(wheel,
                               (double)stick[a] * (double)GetWheelTorque() * getRelAxis_(wheel, a));
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

    /* the reaction wheel's rated torque (N m) -- the ship's angular
       authority, for the HUD (like getThrust() for the engines) */
    float GetWheelTorque() {
        return 2000; /* N m -- rated torque of one reaction wheel */
    }

    /* disarm the armed rotation commands (called once per tick, like
       thrust_mag, so a tick without the keys doesn't keep rotating) */
    void clearRotCmd() {
        stick[0] = stick[1] = stick[2] = 0.0f;
        slew = SlewNone;
    }

    void Draw(const Camera* camera) {
        // Light direction in this frame's axes
        glm::vec3 sunlightVec = glm::vec3(TerrainBody::SunlightDir(m_parent, sun, frame));

        for(auto&& part : parts) {
            // Per-part terrain shadow
            const float shadow = ComputeTerrainShadow(m_parent, frame, GetPosition(part), sun);
            part->Draw(camera, sunlightVec, shadow);
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

    float GetMaxFuelRate() {
        return 1; /* kg/s PER TANK; the engine burns H2 AND LOX, so the
                          total exhaust flow is 2x this -- see GetMaxThrust */
    }

    float GetExhaustVelocity() {
        return 40492; /* m/s -- one engine model, shared by GetMaxThrust and getDeltaV */
    }

    float GetMaxThrust() {
        /* T = (total exhaust flow) x ve. Both propellants end up in the
           plume, so the flow is H2 + LOX = 2 tanks. */
        return 2 * GetMaxFuelRate() * GetExhaustVelocity();
    }

    /* Called once per physics tick (step = the tick's simulated duration).
       Consumes the tick's fuel and records the thrust; the force itself is
       applied by applyThrustForce() before EVERY substep below. */
    void ApplyThrust(double step) {
        if(thruster_util == 0.0f) { return; } /* zero throttle: no burn, no plume */
        float max_fuel_rate = GetMaxFuelRate();
        const float flow = max_fuel_rate * thruster_util * (float)step; /* kg this tick, per tank */

        thrust_mag = 0.0f;
        for(auto&& thruster : m_thrusters) {
            if(consumeResourceMass(ResourceType::Hydrogen, flow) and
               consumeResourceMass(ResourceType::LOX,      flow))
                {
                    thrust_mag = GetMaxThrust() * thruster_util;
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
        return (double)m_reaction_wheels.size() * GetWheelTorque();
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
            auto type = partTypes[i];
            const char *name = VesselPartTypeStr(type);
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
};

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

  As before, the ship's frame is resolved from the innermost SOI containing
  the spawn point (resolve_frame_by_soi), with the stasis-velocity correction
  so a rotating frame still yields the correct inertial orbital velocity.
*/
struct ScenarioDef {
    const char *name;
    bool on_pad;
    double alt_frac; // < 1: rot frame, > 1 non-rot frame
    bool polar;
};
static const ScenarioDef kScenarios[] = {
    {"pad",            true,  0.0,  false},
    {"pad-polar",      true,  0.0,  true},
    {"rot-orbit",      false, 0.85, false},
    {"inertial-orbit", false, 1.25, false},
    {"high-orbit",     false, 5.0,  false},
    {"high-polar",     false, 5.0,  true},
};

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

static void spawn_vehicle(Vehicle *ship, const ScenarioDef &sc, TerrainBody *home, System &sys)
{
    if(sc.on_pad) { return; } // already on the pad, set up in main

    // Circular orbit around the home body: radius measured from its frame origin.
    const double r = home->radius + sc.alt_frac * (home->rot_frame->soi - home->radius);
    const glm::dvec3 rhat_local = sc.polar ? glm::dvec3(0, 1, 0) : glm::dvec3(0, 0, 1);
    const glm::dvec3 shipWorldPos = home->frame->root_pos
                                  + home->frame->root_orient * (rhat_local * r);

    Frame *frame = resolve_frame_by_soi(sys.star->frame, shipWorldPos);

    // Circular orbital speed (vis-viva with semi-major axis == r).
    const double speed = sqrt(home->mu / r);

    // Prograde: perpendicular to the radius vector, in the system's sense of
    // rotation (+y axis); polar orbits go around the spin axis instead.
    const glm::dvec3 rhat = glm::normalize(shipWorldPos - home->frame->root_pos);
    const glm::dvec3 vhat = sc.polar ? glm::cross(glm::dvec3(1, 0, 0), rhat)
                                     : glm::cross(glm::dvec3(0, 1, 0), rhat);
    const glm::dvec3 velWorld = speed * vhat;

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

    // Nose (local +Z) along prograde. The parts are currently stacked along
    // their build axis (pad_dir); rotate the offsets by the same `orient`
    // that is applied to every part, so the whole ship rigidly aligns its
    // axis with vhat (capsule leading) instead of each part spinning in
    // place.
    const glm::dmat3 orient = faceAlong(vhat);
    const glm::dvec3 com = ship->get_center_of_mass();
    for(auto&& part : ship->parts) {
        const glm::dvec3 p = GetPosition(part);
        setPosRot(part, target + orient * (p - com), orient);
        SetVelocity(part, vel);
    }

    printf("Spawn '%s' around %s: frame '%s' @ world (%.0f, %.0f, %.0f), r = %.0f m, |v| = %.1f m/s\n",
           sc.name, home->name.c_str(), frame->name.c_str(),
           shipWorldPos.x, shipWorldPos.y, shipWorldPos.z, r, speed);
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

    // Work in the root (star) frame: root_pos/root_orient are both expressed
    // in root axes, so the composition below is unambiguous (and sidesteps
    // the GetPositionRelTo vec*mat path).
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

Mesh *TerrainBody::create_grid_mesh(bool has_collision, glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4) {
    Mesh *grid_mesh = new Mesh;
    const int size = 25;

    PosNorColVertex vertices[size * size];
    unsigned int indices[size * size * 6] = {0}; // TODO is this off by one?

    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            glm::vec3 sphere_p = getSpherePoint(p1, p2, p3, p4, i/(float)(size-1), j/(float)(size-1));

            if (surface.bands) {
                // gas giant: smooth sphere, color by latitude band
                COLOUR cb = surface.BandColor(sphere_p);
                glm::vec3 color = glm::vec3(cb.r, cb.g, cb.b);
                float brightness = (cb.r + cb.g + cb.b) / 6;
                color = float(0.5) * color + glm::vec3(brightness,
                                                       brightness,
                                                       brightness);
                vertices[j + size * i] = PosNorColVertex(sphere_p * radius,
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

            glm::vec3 p = sphere_p * height;

            vertices[j+size*i] = PosNorColVertex(p,
                                                 sphere_p,
                                                 color);
        }
    }

    for (int i = 1; i < size-1; i++) {
        for (int j = 1; j < size-1; j++) {
            glm::vec3 &x1 = vertices[j-1 + i*size].pos;
            glm::vec3 &x2 = vertices[j+1 + i*size].pos;
            glm::vec3 &y1 = vertices[j + (i-1)*size].pos;
            glm::vec3 &y2 = vertices[j + (i+1)*size].pos;
            glm::vec3 n = glm::normalize(glm::cross(x2-x1, y2-y1));
            vertices[j + size * i].normal = -n;
        }
    }

    int i = 0;
    for (int y = 0; y < (size - 1); y++) {
        for (int x = 0; x < (size - 1); x++) {
            indices[i++] = (y + 1) * size + x + 1;
            indices[i++] = y * size + x + 1;
            indices[i++] = y * size + x;

            indices[i++] = (y + 1) * size + x;
            indices[i++] = (y + 1) * size + x + 1;
            indices[i++] = y * size + x;
        }
    }

    grid_mesh->FromData(vertices, size * size, indices, size * size * 6, has_collision);

    return grid_mesh;
}

glm::dvec3 projectVecOntoPlane(const glm::dvec3 & vec, const glm::dvec3 & normal) {
    return vec - glm::dot(vec, normal) * normal;
}

double wrapAngleToPositive(const double theta) {
    return theta >= 0.0 ? theta : M_PI * 2 + theta;
}

// Two-body orbital elements of a (pos, vel) state relative to a central body
// with gravitational parameter mu. pos/vel must be in the body's INERTIAL
// (non-rotating) frame, where the trajectory is a Kepler conic. Used by the
// --orbit-log periodic printout.
struct OrbitElements {
    double distance;      // m, radius from focus
    double speed;         // m/s
    double semi_major;    // m (negative for hyperbolic trajectories)
    double ecc;           // eccentricity
    double periapsis;     // m, radius at periapsis
    double apoapsis;      // m, radius at apoapsis
    double inclination;   // rad
    double period;        // s (-1 for non-elliptic trajectories)
    double ang_momentum;  // |h|, m^2/s
    double energy;        // specific orbital energy, J/kg
};

OrbitElements computeOrbitElements(const glm::dvec3 &pos, const glm::dvec3 &vel, double mu) {
    const double distance = glm::length(pos);
    const double speed = glm::length(vel);
    const glm::dvec3 h = glm::cross(pos, vel);

    OrbitElements o;
    o.distance = distance;
    o.speed = speed;
    o.energy = 0.5 * speed * speed - mu / distance;
    o.semi_major = 1.0 / (2.0 / distance - speed * speed / mu);
    o.ang_momentum = glm::length(h);
    o.ecc = glm::length(glm::cross(vel, h) / mu - pos / distance);
    o.periapsis = (1.0 - o.ecc) * o.semi_major;
    o.apoapsis = (1.0 + o.ecc) * o.semi_major;
    o.inclination = o.ang_momentum > 0.0 ? acos(h.z / o.ang_momentum) : 0.0;
    o.period = (o.semi_major > 0.0)
        ? 2.0 * M_PI * sqrt(o.semi_major * o.semi_major * o.semi_major / mu)
        : -1.0;
    return o;
}

int main(int argc, char **argv)
{
    CLI::App app{"Open Space Program"};

    std::string body_name;
    app.add_option("--body", body_name,
        "Body the ship starts on / orbits (default: the system's home body)");

    std::string scenario = "pad";
    app.add_option("--scenario", scenario,
        "Starting scenario: pad, pad-polar, rot-orbit, inertial-orbit, "
        "high-orbit, high-polar (default: pad)")
        ->check(CLI::IsMember({"pad", "pad-polar", "rot-orbit",
                               "inertial-orbit", "high-orbit", "high-polar"}));

    std::string system_file = "ksp_system.json";
    app.add_option("--system", system_file,
                   "Star-system JSON file to load (default: ksp_system.json; "
                   "try system.json for the Eerbon system)");

    int initial_time_accel = 0;
    app.add_option("-t,--time-accel", initial_time_accel,
                   "Initial time acceleration (0 = paused, default 0)")
        ->check(CLI::NonNegativeNumber);

    bool orbit_log = false;
    app.add_flag("--orbit-log", orbit_log,
                 "Periodically print the ship's orbital elements to stdout "
                 "(for measuring orbital stability)");

    double orbit_interval = 1.0;
    app.add_option("--orbit-interval", orbit_interval,
                   "Wall-clock seconds between --orbit-log lines (default: 1)")
        ->check(CLI::PositiveNumber);

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

    // Any of the --free-cam-* options opts in to starting in free-cam mode.
    const bool use_free_cam = !free_cam_pos.empty() || !free_cam_fwd.empty()
                            || !free_cam_up.empty();

    Renderer display(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    check_gl_error();
    ImGuiContext* ctx1 = ImGui::CreateContext();
    ImGui::SetCurrentContext(ctx1);
    ImGui_ImplSDL2_InitForOpenGL(display.get_display(), SDL_GL_GetCurrentContext());
    ImGui_ImplOpenGL3_Init("#version 430");
    check_gl_error();

    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->AddFontFromFileTTF("./res/DejaVuSansMono.ttf", 14.0);
    bigger = io.Fonts->AddFontFromFileTTF("./res/DroidSans.ttf", 30.0);
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

    Shader *skyboxshader = new Shader;
    skyboxshader->registerAttribs({ "position" });
    skyboxshader->registerUniforms({ "projectionview" });
    skyboxshader->FromFile("./res/skyboxShader");

    Shader *lineshader = new Shader;
    lineshader->registerAttribs({ "position" });
    lineshader->registerUniforms({ "MVP", "color" });
    lineshader->FromFile("./res/lineShader2");

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

    Vehicle *ship = new Vehicle;
    ship->m_parent = home;
    ship->sun = sun;
    ship->frame = home->rot_frame;

    glm::dvec3 pad_dir = glm::dvec3(0.0, 1.0, 0.0);
    if (scenario != "pad-polar") {
        pad_dir = glm::normalize(glm::dvec3(0.005, 0.005, 1.0));
    }
    const glm::dmat3 pad_orient = faceAlong(pad_dir);

    StaticBuilding *space_port;
    {
        Mesh *space_port_mesh = new Mesh;
        Mesh *capsule_mesh = new Mesh;
        Mesh *wheel_mesh = new Mesh;
        Mesh *engine_mesh = new Mesh;

        space_port_mesh->FromFile("./res/space_port.obj", true);
        capsule_mesh->FromFile("./res/capsule.obj", false);
        wheel_mesh->FromFile("./res/reaction_wheel.obj", false);
        engine_mesh->FromFile("./res/engine.obj", false);

        Texture *space_port_texture = load_texture("./res/space_port.png");
        Texture *reaction_wheel_texture = load_texture("res/reaction_wheel.png");
        Texture *capsule_texture = load_texture("res/capsule.png");
        Texture *engine_texture = load_texture("res/engine.png");

        Model *space_port_model = new Model;
        Model *capsule_model = new Model;
        Model *wheel_model = new Model;
        Model *engine_model = new Model;

        space_port_model->FromData(space_port_mesh, partsshader, space_port_texture);
        capsule_model->FromData(capsule_mesh, partsshader, capsule_texture);
        wheel_model->FromData(wheel_mesh, partsshader, reaction_wheel_texture);
        engine_model->FromData(engine_mesh, partsshader, engine_texture);

        const double ground_alt = home->GetTerrainHeight(pad_dir);
        const glm::dvec3 start = pad_dir * ground_alt;

        space_port = new StaticBuilding;
        space_port->body = create_body(space_port_model, 0, 0, 0, 0, false);
        setPosRot(space_port->body, start + pad_dir * 5.0, pad_orient);
        space_port->parent = home;
        space_port->sun = sun;

        double ship_height = 3.5;

        // top
        Body *capsule = create_body(capsule_model, 0, 0, 0, 500, true);
        setPosRot(capsule, start + pad_dir * (ship_height + 7), pad_orient);
        // middle
        Body *reaction_wheel = create_body(wheel_model, 0, 0, 0, 1000, true);
        setPosRot(reaction_wheel, start + pad_dir * (ship_height + 5), pad_orient);
        // bottom
        Body *thruster = create_body(engine_model, 0, 0, 0, 3000, true);
        setPosRot(thruster, start + pad_dir * (ship_height + 3), pad_orient);

        ship->setRoot(capsule);
        ship->attachDown(reaction_wheel);
        ship->attachDown(thruster);

        ship->partTypes = {
            VesselPartType::Capsule,
            VesselPartType::ReactionWheel,
            VesselPartType::Engine
        };

        ship->init();
        ship->setVelocity(glm::dvec3(0, 0, 0));
    }
    check_gl_error();

    /* Apply the CLI scenario (before the camera is constructed,
       so the camera focuses on the spawn point). */
    const ScenarioDef *sc = &kScenarios[0];
    for(size_t i = 0; i < sizeof(kScenarios) / sizeof(kScenarios[0]); i++) {
        if(kScenarios[i].name == scenario) { sc = &kScenarios[i]; break; }
    }
    spawn_vehicle(ship, *sc, home, sys);

    Mesh *engine_plume_mesh = new Mesh;
    engine_plume_mesh->FromFile("./res/engine_plume.obj", false);
    Texture *engine_plume_texture = load_texture("res/engine_plume.png");
    Model *engine_plume_model = new Model;
    engine_plume_model->FromData(engine_plume_mesh, partsshader, engine_plume_texture);

    Shader *billboardshader = new Shader;
    billboardshader->registerAttribs({ "position", "texcoord", "normal" });
    billboardshader->registerUniforms({ "MVP", "color_uniform" });
    billboardshader->FromFile("./res/billboardshader");

    Texture * front_indicator_texture = load_texture("res/front_crosshair.png");
    Texture * prograde_indicator_texture = load_texture("res/prograde_icon.png");
    Texture * retrograde_indicator_texture = load_texture("res/retrograde_icon.png");
    Texture * radial_in_indicator_texture = load_texture("res/radial_in_icon.png");
    Texture * radial_out_indicator_texture = load_texture("res/radial_out_icon.png");
    Texture * normal_plus_indicator_texture = load_texture("res/normal_plus_icon.png");
    Texture * normal_minus_indicator_texture = load_texture("res/normal_minus_icon.png");

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
    const float camAspect = (float)DISPLAY_WIDTH / (float)DISPLAY_HEIGHT;
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
    double currentTime = 0.001 * (double)(SDL_GetTicks());
    double accumulator = 0.0;
    int time_accel = initial_time_accel;
    int cam_speed = 1;
    bool orbitInfoWindow = true;
    bool orbitMapWindow = true;
    bool shipInfoWindow = true;
    bool gameInfoWindow = false;
    bool controlsWindow = false;
    bool autoPilotWindow = false;
    bool surfaceInfoWindow = true;
    bool resourcesWindow = true;
    bool targetInfoWindow = false;
    bool topHUDWindows = false;
    bool shipDetailWindow = false;
    bool physics_debug_drawing = false;
    bool world_drawing = true;

    double time = 0;

    // --orbit-log: print at most once per wall-clock interval
    const Uint32 orbit_log_interval_ms = (Uint32)(orbit_interval * 1000.0);
    Uint32 orbit_log_last_ms = 0;

    Skybox skybox;
    skybox.init();

    Mesh *skylines = new Mesh;
    {
        PosInterface skyinterface;
        float r = 1000;
        int n = 128;
        for(int i = 1; i < 128; i++) {
            glm::vec3 p = glm::vec3(r * cos((2 * M_PI) * float(i-1)/float(n)),
                                    r * sin((2 * M_PI) * float(i-1)/float(n)),
                                    0);
            skyinterface.positions.push_back(p);
        }
        for(int i = 1; i < 128; i++) {
            glm::vec3 p = glm::vec3(r * cos((2 * M_PI) * float(i-1)/float(n)),
                                    0,
                                    r * sin((2 * M_PI) * float(i-1)/float(n)));
            skyinterface.positions.push_back(p);
        }
        skylines->InitMesh(skyinterface);
    }

    /* main loop timing from
       http://gafferongames.com/game-physics/fix-your-timestep/
    */
    while (running == true) {
        /*
          EVENTS
        */
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
                    }
                }
                if(ev.key.keysym.sym == SDLK_COMMA) {
                    if(time_accel > 1) {
                        time_accel /= 10;
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
                if(ev.key.keysym.sym == SDLK_F10) {
                    // TODO should really toggle the UI
                    orbitInfoWindow = false;
                    orbitMapWindow = false;
                    shipInfoWindow = false;
                    gameInfoWindow = false;
                    controlsWindow = false;
                    autoPilotWindow = false;
                    surfaceInfoWindow = false;
                    resourcesWindow = false;
                    targetInfoWindow = false;
                    topHUDWindows = false;
                    shipDetailWindow = false;
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

        glm::dvec3 grav;
        glm::dvec3 com;

        if(accumulator > 10 * dt) {
            accumulator = 10 * dt;
        }

        // clear stats and stuff
        ship->m_thrust = 0.0;

        while (accumulator >= dt) {
            // is this logic? ;_;
            // Thrust and rotation are armed once per tick (if the keys are
            // held, below) and then re-applied before every substep; clear
            // them first so a tick without the keys doesn't keep pushing or
            // slewing from the last one.
            ship->thrust_mag = 0.0;
            ship->clearRotCmd();

            const Uint8* key = SDL_GetKeyboardState(NULL);
            if(key[SDL_SCANCODE_ESCAPE]) { running = false; }

            if (camMode == CAM_FREE) {
                if (key[SDL_SCANCODE_W]) { camera->MoveForward(cam_speed); }
                else if (key[SDL_SCANCODE_S]) { camera->MoveForward(-cam_speed); }

                if (key[SDL_SCANCODE_A]) { camera->MoveRight(-cam_speed); }
                else if (key[SDL_SCANCODE_D]) { camera->MoveRight(cam_speed); }

                if (key[SDL_SCANCODE_Q]) { camera->Roll(-0.05); }
                else if (key[SDL_SCANCODE_E]) { camera->Roll(0.05); }

                if (key[SDL_SCANCODE_LSHIFT] || key[SDL_SCANCODE_RSHIFT]) { camera->MoveUp(cam_speed); }
                else if (key[SDL_SCANCODE_LCTRL] || key[SDL_SCANCODE_RCTRL]) { camera->MoveUp(-cam_speed); }
            }

            if (camMode == CAM_ORBIT) {
                bool game_running = (time_accel > 0);
                // pitch
                if (key[SDL_SCANCODE_W]) { ship->Command(ShipCmd(Pitch, +1.0f), game_running); }
                if (key[SDL_SCANCODE_S]) { ship->Command(ShipCmd(Pitch, -1.0f), game_running); }
                // yaw
                if (key[SDL_SCANCODE_A]) { ship->Command(ShipCmd(Yaw, +1.0f), game_running); }
                if (key[SDL_SCANCODE_D]) { ship->Command(ShipCmd(Yaw, -1.0f), game_running); }
                // roll
                if (key[SDL_SCANCODE_Q]) { ship->Command(ShipCmd(Roll, +1.0f), game_running); }
                if (key[SDL_SCANCODE_E]) { ship->Command(ShipCmd(Roll, -1.0f), game_running); }

                if (key[SDL_SCANCODE_I]) { ship->Command(ShipCmd(Thrust), game_running, dt * time_accel); }
                if (key[SDL_SCANCODE_X]) { ship->Command(ShipCmd(KillRot), game_running); }

                if (key[SDL_SCANCODE_B]) { ship->Command(ShipCmd(Prograde), game_running); }
                if (key[SDL_SCANCODE_N]) { ship->Command(ShipCmd(Retrograde), game_running); }

                if (key[SDL_SCANCODE_R]) { ship->Command(ShipCmd(ThrottleUp), game_running); }
                if (key[SDL_SCANCODE_F]) { ship->Command(ShipCmd(ThrottleDown), game_running); }
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

                com = ship->get_center_of_mass();
                double ship_r = glm::length(com);
                if(ship_r > ship->frame->soi + 10000) {
                    // switching to parent SOI if there is one
                    if(ship->frame->parent != NULL) {
                        glm::dvec3 pos = GetPosition(ship->controller);
                        printf("@@@ switching frame from %s to parent %s\n",
                               ship->frame->name.c_str(),
                               ship->frame->parent->name.c_str()
                              );
                        glm::dvec3 offset = ship->frame->GetPositionRelTo(ship->frame->parent);
                        printf("@@@ Frame offset: %.0f %.0f %.0f\n", offset.x, offset.y, offset.z);
                        printf("@@@@@ OLD position: %.0f %.0f %.0f\n", pos.x, pos.y, pos.z);
                        ship->moveToFrame(ship->frame->parent);
                        pos = GetPosition(ship->controller);
                        printf("@@@@@ NEW position: %.0f %.0f %.0f\n", pos.x, pos.y, pos.z);
                    }
                }
                else {
                    // check if we've entered a child SOI
                    for(auto&& child : ship->frame->children) {
                        double dist = glm::length(ship->GetPositionRelTo(ship->controller, child));
                        if(dist < child->soi - 10000) {
                            printf("@@@ switching frame from %s to child %s, distance: %.0f\n",
                                   ship->frame->name.c_str(),
                                   child->name.c_str(),
                                   dist
                                  );
                            ship->moveToFrame(child);
                            break;
                        }
                    }
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
                    grav = ship->processGravity();
                    ship->applyThrustForce();
                    ship->applyRotationForce(h);
                    physics_tick(h);
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
                           "inc=%.4f deg T=%.6g s |h|=%.6f m2/s E=%.6f J/kg\n",
                           time, ship->frame->name.c_str(), o.distance, o.speed,
                           o.semi_major, o.ecc, o.periapsis, o.apoapsis,
                           glm::degrees(o.inclination), o.period,
                           o.ang_momentum, o.energy);
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

            display.Clear(0, 0, 0, 1);

            com = ship->get_center_of_mass();

            if(camMode == CAM_ORBIT) {
                camera->Follow(focusWorldPos(focusBody));
            }

            camera->ComputeView();

            /*
              standard 3d stuff drawn here
            */

            if(world_drawing == true) {
                space_port->Draw(camera, ship->m_parent, ship->frame);
                ship->Draw(camera);
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

            const double distance = glm::length(orbit_pos);
            const double speed = glm::length(orbit_vel);
            // https://en.wikipedia.org/wiki/Standard_gravitational_parameter
            // const double G = 6.674e-11;
            // const double M = moon->mass;
            // https://en.wikipedia.org/wiki/Characteristic_energy
            const double e = (0.5 * pow(speed, 2)) - (mu / distance);
            const double SMa = 1.0 / (-((speed * speed) / (mu)) + (2.0 / distance));
            const double radial_vel = glm::dot(orbit_pos, orbit_vel) / distance;

            const glm::dvec3 ang_momentum = glm::cross(orbit_pos, orbit_vel);
            const glm::dvec3 eccentricity_vector = glm::cross(orbit_vel, ang_momentum) / (mu) - orbit_pos/distance;

            const double ecc = glm::length(eccentricity_vector);

            const double ApA = (1 + ecc) * SMa;
            const double PeA = (1 - ecc) * SMa;

            const double inclination = acos(ang_momentum.z / glm::length(ang_momentum));

            const glm::dvec3 node_vector = glm::cross(glm::dvec3(0, 0, 1), ang_momentum);

            double raan = acos(node_vector.x / glm::length(node_vector));
            if(node_vector.y < 0) {
                raan = 2 * M_PI - raan;
            }

            double arg_pe = acos( glm::dot(node_vector, eccentricity_vector) /
                                  (glm::length(node_vector) * glm::length(eccentricity_vector)) );
            if(eccentricity_vector.z < 0) {
                arg_pe = 2 * M_PI - arg_pe;
            }

            glm::dvec3 GetAngVelocity_(Body *b);
            const glm::dvec3 ang_vel_ = GetAngVelocity(ship->controller);

            const glm::dvec3 AoA = glm::dvec3();

            double TrueAnomaly = acos(glm::dot(eccentricity_vector, orbit_pos) / (glm::length(eccentricity_vector) * glm::length(orbit_pos)));
            if(glm::dot(orbit_pos, orbit_vel) < 0) {
                TrueAnomaly = 2 * M_PI - TrueAnomaly;
            }

            // http://www.bogan.ca/orbits/kepler/e_anomly.html
            double EccentricAnomaly = acos((((1 - ecc*ecc)*cos(TrueAnomaly)) / (1 + ecc * cos(TrueAnomaly))) + ecc);
            if(TrueAnomaly > M_PI) {
                EccentricAnomaly = 2 * M_PI - EccentricAnomaly;
            }

            const double MeanAnomaly = EccentricAnomaly - ecc * sin(EccentricAnomaly);
            const double PeT = sqrt((SMa * SMa * SMa) / (mu)) * MeanAnomaly; // s
            const double T = 2 * M_PI * sqrt((SMa * SMa * SMa) / (mu)); // s
            const double ApT = T - PeT; // s

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

            skybox.Draw(camera, skyboxshader, sun->frame->GetOrientRelTo(ship->frame));

            /* draw engine plume */
            glm::dmat4 View = camera->GetView();
            glm::mat4 Projection = camera->GetProjection();
            if(ship->m_thrust > 0) {
                for(auto&& thruster : ship->m_thrusters) {
                    glm::dmat4 Model = thruster->model_matrix;
                    glm::mat4 ModelViewFloat = View * Model;
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

            glLineWidth(4);
            lineshader->Bind();
            lineshader->setUniform_mat4(0, glm::dmat4(camera->GetProjection()) * glm::dmat4(glm::dmat3(camera->GetView())));
            lineshader->setUniform_vec4(1, glm::vec4(1, 1, 0, 0.5));
            skylines->Draw(GL_LINE_LOOP);

            glDisable(GL_BLEND);
            glEnable(GL_DEPTH_TEST);

            if(physics_debug_drawing == true) {
                glDisable(GL_DEPTH_TEST);
                debug_draw(camera);
                glEnable(GL_DEPTH_TEST);
            }

            /*
              ImGui stuff below
            */

            glUseProgram(0);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            if(poly_mode == true) {
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            }

            if(topHUDWindows == true) {
                ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize;
                ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.5, 0.5, 0.5, 1.0));
                ImGui::Begin("Top Middle Window", NULL, flags);
                ImGui::PushFont(bigger);
                int terrain_height = (int)(distance - ship->m_parent->GetTerrainHeight(glm::normalize(pos)));
                ImGui::Text("%08dm", terrain_height);
                ImGui::PopFont();
                ImGui::End();
                ImGui::Begin("Bottom Middle Window", NULL, flags);
                ImGui::PushFont(bigger);
                ImGui::Text("%06dm/s", (int)speed);
                ImGui::PopFont();
                ImGui::End();

                if(e < 0) {
                    ImGui::Begin("Elliptic Orbit Window", NULL, flags);
                    ImGui::PushFont(bigger);
                    ImGui::Text("Elliptic Orbit");
                    ImGui::PopFont();
                    ImGui::End();
                }
                else {
                    ImGui::Begin("Hyperbolic Orbit Window", NULL, flags);
                    ImGui::PushFont(bigger);
                    ImGui::Text("Hyperbolic Orbit");
                    ImGui::PopFont();
                    ImGui::End();
                }
                ImGui::PopStyleColor();
            }

            ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.15, 0.15, 0.15, 1.0));

            ImGui::Begin("Open Space Program");
            ImGui::Spacing();
            ImGui::Checkbox("Resources", &resourcesWindow);
            ImGui::Checkbox("Orbit Info", &orbitInfoWindow);
            ImGui::Checkbox("Orbit Map", &orbitMapWindow);
            ImGui::Checkbox("Surface Info", &surfaceInfoWindow);
            ImGui::Checkbox("Vessel Info", &shipInfoWindow);
            ImGui::Checkbox("Vessel Parts", &shipDetailWindow);
            ImGui::Checkbox("Target Info", &targetInfoWindow);
            ImGui::Checkbox("DUMB-ASS", &autoPilotWindow);
            ImGui::Checkbox("Controls Help", &controlsWindow);
            ImGui::Checkbox("Game Debug Info", &gameInfoWindow);
            ImGui::Checkbox("Top HUD", &topHUDWindows);
            ImGui::Checkbox("Planets", &planetsWindow);
            ImGui::End();

            if(gameInfoWindow == true) {
                ImGui::Begin("Game Debug Info");
                ImGui::Checkbox("Physics debug draw", &physics_debug_drawing);
                ImGui::Checkbox("World draw", &world_drawing);
                ImGui::Text("Time: %f", time);
                ImGui::Text("Patches: %d", ship->m_parent->CountPatches());
                ImGui::Text("Cam speed: %d", cam_speed);
                ImGui::Text("Time Accel: %d", time_accel);
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
                            glm::length(ship->GetPositionRelTo(ship->controller, home->frame)));
                ImGui::Text("Pos: %.3fkm", distance / 1000);
                ImGui::Text("xyz(%0.f, %0.f, %0.f)", pos.x, pos.y, pos.z);
                ImGui::Text("Vel: %.3fm/s", speed);
                ImGui::Text("xyz(%0.f, %0.f, %0.f)", vel.x, vel.y, vel.z);
                ImGui::End();
            }

            if(orbitInfoWindow == true) {
                ImGui::Begin("ORBITAL");
                ImGui::Text("Vel: %.1fm/s", speed);
                ImGui::Text("Alt: %.1fm", distance);
                ImGui::Text("ApA: %.1fm", ApA);
                ImGui::Text("ApT: %.1f", ApT);
                ImGui::Text("PeA: %.1fm", PeA);
                ImGui::Text("PeT: %.1f", PeT);
                ImGui::Text("  T: %.1f", T);
                ImGui::Text("Inc: %.2f", glm::degrees(inclination));
                ImGui::Text("Ecc: %f ", ecc);
                ImGui::Text("SMa: %.1fm", SMa);
                ImGui::Text("LAN: %.2f", glm::degrees(raan));
                ImGui::Text("LPe: %.2f", glm::degrees(arg_pe));
                double prograde_angle = glm::angle(facing_dir, vel_dir);
                double retrograde_angle = glm::angle(facing_dir, - vel_dir);
                ImGui::Text("Angle to Prograde: %.2f", glm::degrees(prograde_angle));
                ImGui::Text("Angle to Retrograde: %.2f", glm::degrees(retrograde_angle));
                ImGui::Text("Energy: %.2f J", e);
                ImGui::Separator();
                ImGui::Text("Gravity (%.2f): %0.f %0.f %0.f", glm::length(grav), grav.x, grav.y, grav.z);
                ImGui::Text("Radial velocity: %.2f", radial_vel);
                ImGui::Text("Ang Vel: %.2f %.2f %.2f", ang_vel_.x, ang_vel_.y, ang_vel_.z);
                ImGui::End();
            }

            if(surfaceInfoWindow == true) {
                ImGui::Begin("SURFACE");
                ImGui::Text("Altitude (True): %.1fm", distance - ship->m_parent->GetTerrainHeight(glm::normalize(pos)));
                ImGui::Text("Altitude (ASL): %.1fm", distance - ship->m_parent->radius);
                ImGui::Text("V speed: %.2fm/s", ver_speed);
                ImGui::Text("H speed: %.2fm/s", hor_speed2);
                ImGui::Text("Latitude: %.4f", glm::degrees(latitude));
                ImGui::Text("Longitude: %.4f", glm::degrees(longitude));
                ImGui::Text("Pitch: %.2f", glm::degrees(pitch));
                ImGui::Text("Roll: %.2f", glm::degrees(roll));
                ImGui::Text("Heading: %.2f", glm::degrees(yaw));
                ImGui::End();
            }

            if(shipInfoWindow == true) {
                ImGui::Begin("VESSEL");
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
                ImGui::End();
            }
            if(shipDetailWindow == true) {
                ImGui::Begin("SHIP PARTS");
                int i = 0;
                for(auto&& part : ship->parts) {
                    ImGui::Text("Part #%d", i);
                    ImGui::Separator();
                    ImGui::Text("Name: %s", VesselPartTypeStr(ship->partTypes[i]));
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
                ImGui::End();
            }

            if(controlsWindow == true) {
                ImGui::Begin("Controls help");
                ImGui::Text("Game");
                ImGui::Separator();
                ImGui::Text("p - toggle wireframe mode");
                ImGui::Text(", - decrease time acceleration");
                ImGui::Text(". - increase time acceleration");
                ImGui::Text("k - decrease camera speed");
                ImGui::Text("l - increase camera speed");
                ImGui::Text("c - switch mode: orbit (flying) <-> free (exploring)");
                ImGui::Text("g - orbit mode: cycle target (ship/sun/planet/moon)");
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
                ImGui::Spacing();
                ImGui::Text("Free mode (exploring)");
                ImGui::Separator();
                ImGui::Text("w/s - forward/back");
                ImGui::Text("a/d - strafe");
                ImGui::Text("q/e - roll");
                ImGui::Text("shift/ctrl - up/down");
                ImGui::End();
            }

            if(autoPilotWindow == true) {
                ImGui::Begin("Autopilot");
                ImGui::Button("Prograde");
                ImGui::Button("Retrograde");
                ImGui::Button("Radial-in");
                ImGui::Button("Radial-out");
                ImGui::Button("Normal");
                ImGui::Button("Anti-normal");
                ImGui::End();
            }

            if(resourcesWindow == true) {
                ImGui::Begin("RESOURCES");
                float hydrogen_frac =
                    ship->partResources[2].current[(int)ResourceType::Hydrogen] /
                    ship->partResources[2].capacity[(int)ResourceType::Hydrogen];
                float lox_frac =
                    ship->partResources[2].current[(int)ResourceType::LOX] /
                    ship->partResources[2].capacity[(int)ResourceType::LOX];

                ImGui::ProgressBar(hydrogen_frac, ImVec2(-1, 0), "Hydrogen");
                ImGui::ProgressBar(lox_frac, ImVec2(-1, 0), "LOX");
                ImGui::ProgressBar(0.13, ImVec2(-1, 0), "Hydrazine");
                ImGui::ProgressBar(0.45, ImVec2(-1, 0), "Electric charge");
                ImGui::ProgressBar(0.75, ImVec2(-1, 0), "Oxygen");
                ImGui::ProgressBar(0.83, ImVec2(-1, 0), "Water");
                ImGui::ProgressBar(0.94, ImVec2(-1, 0), "Food");
                ImGui::End();
            }

            if(orbitMapWindow == true) {
                ImVec2 pts[26];
                // ImVec2 planet[26];
                // ImGui::Text("%.1f %.1f", center.x, center.y);
                int i = 0;
                ImU32 color = ImGui::GetColorU32(ImVec4(255,255,255,255));
                ImU32 color2 = ImGui::GetColorU32(ImVec4(255,0,0,255));
                ImGui::Begin("Orbital map");
                const ImVec2 p = ImGui::GetCursorScreenPos();
                double E = 0;
                static float div = 6000.0;
                for(i = 0; i < 26; i++) {
                    double r = SMa * (1 - ecc * cos(E));
                    // ImGui::Text("%.0f", r);
                    double argX = cos(E) - ecc;
                    double argY = sqrt(1 - (ecc * ecc)) * sin(E);
                    double phi = atan2(argY, argX);
                    pts[i].x = 200 + p.x + (r / div * cos(phi));
                    pts[i].y = 200 + p.y + (r / div * sin(phi));
                    // planet[i].x = 200 + p.x + (600000 / div * cos(phi));
                    // planet[i].y = 200 + p.y + (600000 / div * sin(phi));
                    // double t = sqrt((SMa * SMa * SMa / (G * M))) * (E - ecc * sin(E));
                    // ImGui::Text("E=%.2f r=%.0f, phi=%.1f, t=%.1f", E, r, (phi + (E > M_PI ? 0 : 2 * M_PI)) * (180/M_PI), t);
                    E += 2 * M_PI / 25;
                }

                double argX = cos(EccentricAnomaly) - ecc;
                double argY = sqrt(1 - (ecc * ecc)) * sin(EccentricAnomaly);
                double phi = atan2(argY, argX);

                ImVec2 ship_p = { float(200 + p.x + distance / div * cos(phi)),
                                  float(200 + p.y + distance / div * sin(phi)) };

                ImVec2 raan_p = { float(200 + p.x + 100 * cos(raan)),
                                  float(200 + p.y + 100 * sin(raan)) };

                /* incorrect */
                ImVec2 peri_p = { float(200 + p.x + PeA / div * cos(arg_pe - M_PI / 2)),
                                  float(200 + p.y + PeA / div * sin(arg_pe - M_PI / 2)) };

                ImVec2 apo_p = { float(200 + p.x + ApA / div * cos(arg_pe + M_PI / 2)),
                                 float(200 + p.y + ApA / div * sin(arg_pe + M_PI / 2)) };

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
                ImGui::Text("True anomaly: %.2f", TrueAnomaly);
                ImGui::Text("Eccentric anomaly: %.2f", EccentricAnomaly);

                ImGui::End();
            }

            ImGui::PopStyleColor();
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            if(screenshot_requested == true) {
                char fname[256];
                // TODO change format to osp_YYYY_MM_DD_HH_MM_SS.png
                snprintf(fname, sizeof(fname), "./screenshot_%03d.png", screenshot_count);
                if(display.SaveScreenshot(fname)) {
                    screenshot_count++;
                }
                screenshot_requested = false;
            }

            display.SwapBuffers();
            check_gl_error();
        }
    }

    delete space_port;
    delete ship;

    for(auto&& body : sys.bodies) { delete body; }

    delete partsshader;
    delete sunshader;
    delete terrainshader;
    delete billboardshader;
    delete skyboxshader;

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
