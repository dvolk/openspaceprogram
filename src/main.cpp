/*
  Open Space Program

  Status: exploratory clusterfuck

  Features:
  * Planet generation with quad-tree surface
  * Gravity
  * Rigid body physics
  * Ground collision
  * Multipart ship
  * Basic shader
  * ImGui integration

  TODO:
  * MORE COMMENTS
  * Attitude autopilots
  * Better camera controls
  * Vessel orbiting camera
  * drawing force, velocities, orientations, etc
  * Fuel
  * Staging
  * // Fix kill-rot
  * surface information (lat, long, hor, vert speeds)
  * Calculate orbital elements
  * walking around on the ground
  * Why doesn't the collision between trigmeshes work?
  * fix seams between patches
  * Reference frames and other bodies
  * Patched conics
  * Check memory management
  * Glue quad tree patches together
  * Ground textures
  * Better ship placement
  * Better ship mesh
  * Better planet gen
  * Add a sun billboard
  * Atmosphere functionality
  * Orbit stability (different integrator?)
  * Better elevation palette and mixin with moisture noise
  * Ship construction
  * More debug information
  * Bullet physics debug drawing
  * Multithreaded patch generation
  * Shadowmapping
  * Atmosphere rendering
  * clamp reaction wheel torque
  * fix frame transitions for multipart ships
  * parts should be able to have several functions i.e. capsule + reaction wheel etc
  * RCS
  * ... lots more ...
  */

#include <stdio.h>
#include <algorithm>
#include <chrono>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <map>

#include "SDL2/SDL.h"

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/gtc/noise.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/projection.hpp>
#include <glm/gtx/vector_angle.hpp>
// #include <glm/gtx/matrix_decompose.hpp>
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

// static const int DISPLAY_WIDTH = 1440;
// static const int DISPLAY_HEIGHT = 900;
static const int DISPLAY_WIDTH = 1920;
static const int DISPLAY_HEIGHT = 1080;
static const int FPS = 60;

#define RAD2DEG(rad) (((180.0/M_PI) * rad))

struct TerrainBody;

// The reference-frame tree is built by load_system() below, together with the
// TerrainBodies it belongs to (each body owns its inertial frame and, when it
// spins, its rotating frame). There is no separate flat "frames" array.

class TerrainBody;

// Mesh *create_grid_mesh(TerrainBody *body,
// 		       bool has_collision,
//                        float radius,
//                        glm::vec3 p1, glm::vec3 p2,
//                        glm::vec3 p3, glm::vec3 p4);

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
    // The body OWNS its reference frames: `frame` is its inertial (non-rotating)
    // frame and `rot_frame` its rotating frame (NULL for bodies that don't
    // spin, e.g. the star). They are built alongside the body by load_system()
    // and deleted here, so there is no separate flat frames array to manage.
    Frame *frame;
    Frame *rot_frame;
    glm::dmat4 transform;
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

        // for(auto&& patch : patches) {
        //   patch->Subdivide();
        //   // patch->kids[0]->Subdivide();
        //   // patch->kids[1]->Subdivide();
        //   // patch->kids[2]->Subdivide();
        //   // patch->kids[3]->Subdivide();
        // }
    }

    void Draw(const Camera* camera, TerrainBody *sun) {
        double cam_dist = glm::length(camera->GetPos() - glm::dvec3(transform[3]));

        /*
          this is slightly wrong?
        */
        sunlightVec = -glm::normalize(
                                      frame->GetOrientRelTo(sun->frame) *
                                      sun->frame->GetPositionRelTo(frame)
                                     );

        dbg_drew_patches = 0;
        for(auto&& patch : patches) {
            // patch isn't subdivided
            patch->Draw(camera, transform, sunlightVec);
        }

        if(planetsWindow) {
            ImGui::Begin("Planets");
            // printf("%p\n", frame->getNonRotFrame());
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
        // transform = glm::translate(frame->root_pos) * glm::dmat4(frame->orient);

        for(auto&& patch : patches) {
            patch->Update(camera, transform);
        }
        // if(moves == true) {
        //   transform = glm::rotate(transform, 1/60.0/10.0, glm::dvec3(0,1,0));
        // }
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
COLOUR GetColourEerbon(float v, float vmin, float vmax);

/*
  A loaded star system: the bodies, plus the few roles the rest of the game
  needs (the star, the home planet the ship starts on, and the home planet's
  first moon). The bodies own their frames (see TerrainBody::frame / rot_frame).
*/
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
          },
          "moves":   bool,
          "inertial": { "soi": m, "pos": [x,y,z], "orb_ang_speed": rad/s },
          "rotating": { "soi": m, "rot_ang_speed": rad/s }   // optional; absent => dummy (zero spin, soi = radius + 100 km)
        },
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
            body->colour_func = GetColourEerbon;
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
        f->initial_orient = glm::dmat3();
        f->orient = glm::dmat3();
        f->vel = glm::dvec3(0);
        f->orb_ang_speed = 0;
        f->rot_ang_speed = 0;
        f->soi = 1e6;
        f->root_pos = glm::dvec3(0);
        f->root_vel = glm::dvec3(0);
        f->root_orient = glm::dmat3();

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
        rf->initial_orient = glm::dmat3();
        rf->orient = glm::dmat3();
        rf->vel = glm::dvec3(0);
        rf->orb_ang_speed = 0;
        rf->root_pos = glm::dvec3(0);
        rf->root_vel = glm::dvec3(0);
        rf->root_orient = glm::dmat3();
        if(bv.contains("rotating") && bv["rotating"].is_object()) {
            const nlohmann::json &rot = bv["rotating"];
            rf->soi = rot.value("soi", 1e5);
            rf->rot_ang_speed = rot.value("rot_ang_speed", 0.0);
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
    // if(glm::dot(glm::vec3(camera.GetForward()), centroid)

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
    if(depth > max_depth)
        return;

    const glm::dvec3& camera_pos = camera->GetPos() - (glm::dvec3)(transform[3]);
    const glm::dvec3& centroid_pos = body->GetTerrainHeight(glm::normalize(camera_pos)) * centroid;
    const float dist = glm::length(camera_pos - centroid_pos);
    const float subdiv = 2.0f * body->radius * glm::length(v0 - centroid);

    //body->radius / depth;

    // printf("%p camera: %f, centroid: %f, dist: %f < %f\n",
    //        this,
    //        glm::length(camera_pos),
    //        glm::length(centroid_pos),
    //        dist,
    //        subdiv);

    if(dist < subdiv) {
        if(kids[0] == NULL) {
            Subdivide();
        }
    }
    else if(// depth >= 3 and
            dist > subdiv * 2) {
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

class Vehicle {
public:
    std::vector<Body *> parts;
    std::vector<ResourceContent> partResources;
    std::vector<VesselPartType> partTypes;

    TerrainBody *m_parent;
    Frame *frame;
    float m_thrust;

    glm::dvec3 m_com;

    Body *controller;
    std::vector<Body *> m_thrusters;
    std::vector<Body *> m_reaction_wheels;
    std::vector<void *> constraints;

    float thruster_util = 1.0;

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
        // setVelocity(glm::dvec3(0, 0, 0));
        partResources.resize(parts.size());
        controller = parts.back();
        NeverSleep(controller);
        for(size_t i = 0; i < parts.size(); i++) {
            if(partTypes[i] == VesselPartType::ReactionWheel) {
                m_reaction_wheels.push_back(parts[i]);
            }
            else if(partTypes[i] == VesselPartType::Engine) {
                partResources[i].capacity[(int)ResourceType::Hydrogen] = 1.0;
                partResources[i].capacity[(int)ResourceType::LOX] = 1.0;
                partResources[i].current[(int)ResourceType::Hydrogen] = 1.0;
                partResources[i].current[(int)ResourceType::LOX] = 1.0;
                m_thrusters.push_back(parts[i]);
            }
        }
    }

    /* returns true if fuel was consumed*/
    bool consumeResourceMass(enum class ResourceType type, float amt /* kg */) {
        int i = 0;
        float consp_factor = 60; // since fps = 60 and fuel flow is kg/s
        for(auto&& partResource : partResources) {
            if(partResource.current[(int)type] >= amt / consp_factor) {
                partResource.current[(int)type] -= amt / consp_factor;
                parts[i]->mass -= amt / consp_factor; /* why does Body have mass at all? */
                void SetMass(Body *body, double newMass);
                SetMass(parts[i], parts[i]->mass);
                return true;
            }
            i++;
        }
        return false;
    }

    float getFuelMass(const std::vector /* eh */ <enum class ResourceType>& types) {
        float fuel = 0;
        for(auto&& type : types) {
            for(auto&& partResource : partResources) {
                fuel += partResource.current[(int)type];
            }
        }
        return fuel;
    }

    float getDeltaV() {
        float exaust_vel = 10123; /* m/s */
        float remaining_fuel = getFuelMass({ ResourceType::Hydrogen, ResourceType::LOX }) / 2.0; /* kg */
        return exaust_vel * log(getMass() / (getMass() - remaining_fuel));
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

    void Draw(const Camera* camera) {
        glm::vec3 sunlightVec = m_parent->sunlightVec; // more or less correct?

        for(auto&& part : parts) { part->Draw(camera, sunlightVec); }
    }

    void ThrottleUp() {
        thruster_util += 0.01;
        if(thruster_util > 1) thruster_util = 1;
    }
    void ThrottleDown() {
        thruster_util -= 0.01;
        if(thruster_util < 0) thruster_util = 0;
    }

    float GetMaxFuelRate() {
        return 0.01; /* kg/[T] fixme T == ?? */
    }

    float GetMaxThrust() {
        float exaust_velocity = 40492; /* m/s (Isp ~4049 s; raised 4x for easier orbit insertion) */
        return GetMaxFuelRate() * exaust_velocity;
    }

    void ApplyThrust() {
        float max_fuel_rate = GetMaxFuelRate();

        for(auto&& thruster : m_thrusters) {
            if(consumeResourceMass(ResourceType::Hydrogen, max_fuel_rate * thruster_util /* could == 0? */) and
               consumeResourceMass(ResourceType::LOX,      max_fuel_rate * thruster_util))
                {
                    ApplyCentralForceForward(thruster, GetMaxThrust() * thruster_util);
                    m_thrust = 1.0;
                }
        }
    }
    void ApplyRotXPlus() {
        for(auto&& reaction_wheel : m_reaction_wheels) {
            ApplyTorqueRelX(reaction_wheel, 2);
        }
    }
    void ApplyRotXMinus() {
        for(auto&& reaction_wheel : m_reaction_wheels) {
            ApplyTorqueRelX(reaction_wheel, -2);
        }
    }
    void ApplyRotYPlus() {
        for(auto&& reaction_wheel : m_reaction_wheels) {
            ApplyTorqueRelY(reaction_wheel, 2);
        }
    }
    void ApplyRotYMinus() {
        for(auto&& reaction_wheel : m_reaction_wheels) {
            ApplyTorqueRelY(reaction_wheel, -2);
        }
    }
    void ApplyRotZPlus() {
        for(auto&& reaction_wheel : m_reaction_wheels) {
            ApplyTorqueRelZ(reaction_wheel, 2);
        }
    }
    void ApplyRotZMinus() {
        for(auto&& reaction_wheel : m_reaction_wheels) {
            ApplyTorqueRelZ(reaction_wheel, -2);
        }
    }
    void KillRot() {
        const double angvel_treshhold = 0.001;
        glm::dvec3 ang_vel = GetAngVelocity(m_reaction_wheels.front());

        if(glm::length(ang_vel) < angvel_treshhold) {
            return;
        }
        else {
            void ApplyTorque(Body *body, glm::dvec3 torque);

            ApplyTorque(m_reaction_wheels.front(), - glm::normalize(ang_vel));
        }
    }

    glm::dvec3 GetVel() {
        return GetVelocity(controller);
    }

    void RotateToward(glm::dvec3 dir) {
        void ApplyTorque(Body *body, glm::dvec3 torque);
        glm::dvec3 getRelAxis_(Body *body, int n);

        glm::dvec3 facing = getRelAxis_(m_reaction_wheels.front(), 2);
        glm::dvec3 torque = -glm::normalize(glm::cross(dir, facing) / 10.0);

        ApplyTorque(m_reaction_wheels.front(), torque);
    }

    glm::dmat3 GetOrientRelTo(Body *part, Frame *relTo)
    {
        glm::dmat3 GetOrient(Body *b);
        glm::dmat3 forient = frame->GetOrientRelTo(relTo);
        return forient * GetOrient(part);
    }

    // vector3d Body::GetPositionRelTo(const Frame *relTo) const
    // {
    // 	vector3d fpos = m_frame->GetPositionRelTo(relTo);
    // 	matrix3x3d forient = m_frame->GetOrientRelTo(relTo);
    // 	return forient * GetPosition() + fpos;
    // }

    glm::dvec3 GetPositionRelTo(Body *part, Frame *relTo) {
        glm::dvec3 fpos = frame->GetPositionRelTo(relTo);
        glm::dmat3 forient = frame->GetOrientRelTo(relTo);
        return forient * GetPosition(part) + fpos;
    }
    // vector3d Body::GetVelocityRelTo(const Frame *relTo) const
    // {
    // 	matrix3x3d forient = m_frame->GetOrientRelTo(relTo);
    // 	vector3d vel = GetVelocity();
    // 	if (m_frame != relTo) vel -= m_frame->GetStasisVelocity(GetPosition());
    // 	return forient * vel + m_frame->GetVelocityRelTo(relTo);
    // }

    glm::dvec3 GetVelocityRelTo(Body *part, Frame *relTo) {
        glm::dmat3 forient = frame->GetOrientRelTo(relTo);
        glm::dvec3 vel = GetVelocity(part);
        glm::dvec3 pos = GetPosition(part);
        if(frame != relTo) vel += frame->GetStasisVelocity(pos);
        return forient * vel + frame->GetVelocityRelTo(relTo);
    }

    // void Body::SwitchToFrame(Frame *newFrame)
    // {
    // 	const vector3d vel = GetVelocityRelTo(newFrame);		// do this first because it uses position
    // 	const vector3d fpos = m_frame->GetPositionRelTo(newFrame);
    // 	const matrix3x3d forient = m_frame->GetOrientRelTo(newFrame);
    // 	SetPosition(forient * GetPosition() + fpos);
    // 	SetOrient(forient * GetOrient());
    // 	SetVelocity(vel + newFrame->GetStasisVelocity(GetPosition()));
    // 	SetFrame(newFrame);

    // 	LuaEvent::Queue("onFrameChanged", this);
    // }

    void moveToFrame(Frame *newFrame) {
        // void setModelMatrix(Body *b, glm::dmat4 model);
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
            // setModelMatrix(part, newModelMatrix);

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

/*
  Resolve the reference frame that owns a world position: walk down the frame
  tree from the root, descending into whichever child's sphere of influence
  contains the point. The deepest such frame is the ship's correct frame.
  This is the same rule the per-tick SOI switching in the main loop applies,
  so a ship spawned here is already in the frame the SOI logic expects and is
  not moved on the first physics tick.
*/
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
    double alt_frac; // orbit altitude as a fraction of (rot SOI - radius)
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
    Body *body;

    void Draw(const Camera* camera, const TerrainBody *current) {
        if(current == parent) {
            body->Draw(camera, parent->sunlightVec);
        }
    }
};

glm::vec3 getSpherePoint(const glm::vec3& v0, const glm::vec3& v1,
                         const glm::vec3& v2, const glm::vec3& v3,
                         const float x, const float y)
{
    return glm::normalize(v0 +
                          x * (1.0f - y) * (v1 - v0) +
                          x * y * (v2 - v0) +
                          (1.0f - x) * y * (v3 - v0));
}

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
//(((noise3d(sphere_p, 3, 0.5) * 15000)))

// float TerrainBody::GetTerrainHeight(const glm::dvec3& p) {
//   glm::vec3 sphere_p = glm::normalize(p);
//   glm::vec3 sphere_coord = radius * glm::normalize(sphere_p);
//   glm::vec3 noise = sphere_p * NOISE_FUNC;
//   return glm::length(sphere_coord + noise);
// }

float TerrainBody::GetTerrainHeight(const glm::vec3& sphere_p) {
    const Surface &s = surface;
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

inline COLOUR GetColourMoon(float v, float vmin, float vmax) {
    return { 0.5, 0.5, 0.5 };
}

inline COLOUR GetColourSun(float v, float vmin, float vmax) {
    return { 1.0, 1.0, 0.0 };
}

inline COLOUR GetColourEerbon(float v, float vmin, float vmax)
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
    unsigned int indices[size * size * 6] = {0};

    glm::vec2 dummyuv = glm::vec2(0, 0);

    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            glm::vec3 sphere_p = getSpherePoint(p1, p2, p3, p4, i/(float)(size-1), j/(float)(size-1));
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

            // else if(height < radius + 75) {
            // 	color = beach;
            // }
            // else if(height < radius + 1500) {
            // 	color = green;
            // }
            // else if(height < radius + 2800) {
            // 	color = brown;
            // }
            // else {
            // 	color = snow;
            // }

            vertices[j+size*i] = PosNorColVertex(p,
                                                 sphere_p,
                                                 color);
        }
    }

    glm::dvec3 centroid = glm::normalize(p1 + p2 + p3 + p4);

    // for (int i = 0; i < size; i++) {
    //   for (int j = 0; j < size; j++) {
    //     *vertices[j + size * i].GetNormal() = centroid;
    //   }
    // }

    for (int i = 1; i < size-1; i++) {
        for (int j = 1; j < size-1; j++) {
            glm::vec3 &x1 = vertices[j-1 + i*size].pos;
            glm::vec3 &x2 = vertices[j+1 + i*size].pos;
            glm::vec3 &y1 = vertices[j + (i-1)*size].pos;
            glm::vec3 &y2 = vertices[j + (i+1)*size].pos;
            // glm::vec3 n = centroid;//glm::normalize(centroid);
            glm::vec3 n = glm::normalize(glm::cross(x2-x1, y2-y1));
            vertices[j + size * i].normal = -n;
        }
    }

    // for(int i = 4; i < size-4; i++) {
    //   *vertices[i].GetNormal() = centroid;
    // }

    // for(int i = (size-2) * (size-1); i < (size-1) * (size-1); i++) {
    //   *vertices[i].GetNormal() = *vertices[i - size].GetNormal();
    // }

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

    // assert(find(indices.begin(), indices.end(), -1) == indices.end());

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
    /* CLI (parsed with CLI11):
       ./osp [--system FILE] [--body NAME] [--scenario NAME] [--time-accel N]
             [--orbit-log] [--orbit-interval S] [--free-cam-pos X Y Z]
             [--free-cam-fwd X Y Z] [--free-cam-up X Y Z] [--help]

       --system FILE: star-system JSON file to load (default: system.json).
                       Use ksp_system.json to fly in the Kerbal system.
       --body NAME: body the ship starts on / orbits (default: the system's
                       "home" body). Any body in the system, star included.
       --scenario NAME: starting scenario (default: pad)
         pad            landed on the spaceport pad (equatorial)
         pad-polar      landed on the spaceport pad over the north pole
         rot-orbit      circular orbit, altitude 0.85 * (rot-SOI - radius):
                        inside the rotating SOI, so the ship starts in the
                        ROTATING frame
         inertial-orbit circular orbit, altitude 1.25 * (rot-SOI - radius):
                        just outside the rotating SOI, so the ship starts in
                        the INERTIAL frame
         high-orbit     circular orbit, altitude 5 * (rot-SOI - radius)
         high-polar     same as high-orbit, but in a polar plane (the plane
                        containing the body's spin axis)

       --time-accel N: initial time acceleration (0 = paused; runtime is
                       still changed with the , / . keys)
       --orbit-log:    print the ship's orbital elements to stdout every
                       --orbit-interval wall-clock seconds, for measuring
                       orbital stability
       --free-cam-pos/fwd/up: start directly in the free (6DOF) camera at this
                       pose. Coordinates are world (ship-frame) space, i.e. the
                       ship's centre of mass is the origin. --free-cam-fwd and
                       --free-cam-up are directions (normalised). Giving any of
                       the three switches the starting mode to free flight. */
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

    std::string system_file = "system.json";
    app.add_option("--system", system_file,
                   "Star-system JSON file to load (default: system.json; "
                   "try ksp_system.json for the Kerbal system)");

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

    // Start directly in the free (6DOF) camera at an explicit pose. All
    // coordinates are in the world (ship-frame) coordinate system, i.e. the
    // same space the camera draws the scene in (ship centre of mass at origin).
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
    //io.Fonts->AddFontDefault();
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
    partsshader->registerUniforms({ "MVP", "Normal", "lightDirection" });
    partsshader->FromFile("./res/partsShader");

    Shader *terrainshader = new Shader;
    terrainshader->registerAttribs({ "position", "normal", "color" });
    terrainshader->registerUniforms({ "MVP", "Normal", "lightDirection", "color" });
    terrainshader->FromFile("./res/terrainShader");

    Shader *sunshader = new Shader;
    sunshader->registerAttribs({ "position", "normal", "color" });
    sunshader->registerUniforms({ "MVP", "Normal", "lightDirection", "color" });
    // The sun is the light source, so it must be self-illuminated: use the
    // emissive sunShader (gl_FragColor = color0, no lightDirection) rather than
    // the lit terrainShader, whose light vector is normalize(0) = NaN for the sun.
    sunshader->FromFile("./res/sunShader");

    Shader *skyboxshader = new Shader;
    skyboxshader->registerAttribs({ "position" });
    skyboxshader->registerUniforms({ "projectionview" });
    skyboxshader->FromFile("./res/skyboxShader");

    Shader *lineshader = new Shader;
    lineshader->registerAttribs({ "position" });
    lineshader->registerUniforms({ "MVP", "color" });
    lineshader->FromFile("./res/lineShader2");

    // Load the star system (bodies + their reference frames) from a JSON file.
    // Default is the Eerbon system (system.json); --system picks another, e.g.
    // ksp_system.json to fly in the Kerbal system.
    System sys = load_system(system_file.c_str(), terrainshader, sunshader);
    TerrainBody *sun   = sys.star;   // the star
    // The body the ship starts on / orbits: --body, or the system's "home".
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
    // The ship starts in the body's rotating (near-body) frame; every body
    // has one (a zero-spin dummy for bodies that don't rotate, see load_system).
    ship->frame = home->rot_frame;

    // The pad the ship starts on: over the spin axis (+Y) for the polar pad,
    // the legacy near-equator spot otherwise. Orbit scenarios build the ship
    // here too, then teleport it into orbit (spawn_vehicle below).
    const bool pad_polar = (scenario == "pad-polar");
    const glm::dvec3 pad_dir = pad_polar
        ? glm::dvec3(0.0, 1.0, 0.0)
        : glm::normalize(glm::dvec3(0.005, 0.005, 1.0));
    // Nose (local +Z, the thrust axis) along the pad radial = the launch
    // direction; ~identity at the legacy pad, a 90° tilt at the pole.
    const glm::dmat3 pad_orient = faceAlong(pad_dir);

    StaticBuilding *space_port;
    {
        glm::vec3 grey = glm::vec3(0.55, 0.5, 0.6);
        glm::vec3 pink = glm::vec3(1.0, 192.0/255.0, 203.0/255.0);
        glm::vec3 red = glm::vec3(0.9, 0.2, 0.1);
        glm::vec3 blue = glm::vec3(0.1, 0.2, 0.9);

        Mesh *space_port_mesh = new Mesh;
        Mesh *capsule_mesh = new Mesh;
        Mesh *wheel_mesh = new Mesh;
        Mesh *engine_mesh = new Mesh;

        space_port_mesh->FromFile("./res/space_port.obj", true);
        capsule_mesh->FromFile("./res/capsule.obj", false);
        wheel_mesh->FromFile("./res/reaction_wheel.obj", false);
        engine_mesh->FromFile("./res/engine.obj", false);

        Texture *no_texture = load_texture("./res/no_texture.png");
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

        // double ship_height = 190000.5;
        double ship_height = 3.5;

        // top
        Body *capsule = create_body(capsule_model, 0, 0, 0, 0.5, true);
        setPosRot(capsule, start + pad_dir * (ship_height + 7), pad_orient);
        // middle
        Body *reaction_wheel = create_body(wheel_model, 0, 0, 0, 1.0, true);
        setPosRot(reaction_wheel, start + pad_dir * (ship_height + 5), pad_orient);
        // bottom
        Body *thruster = create_body(engine_model, 0, 0, 0, 3.0, true);
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

    // glm::vec4 billboardcolor = glm::vec4(83/255.0, 238/255.0, 83/255.0, 1.0);
    glm::vec4 billboardcolor = glm::vec4(1, 1, 1, 1.0);

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
    // zFar must exceed the farthest visible body. The log-depth shaders
    // (res/*Shader.vs) define the hard far limit as `far = 1e13` m, which
    // covers the real solar system (Pluto at ~5.9e12 m) and KSP-style
    // AU scales (~1.4e10 m). Keep zFar consistent with that.
    const float camFov = M_PI/3.0;
    const float camAspect = (float)DISPLAY_WIDTH / (float)DISPLAY_HEIGHT;
    const float camZNear = 1.0f;
    const float camZFar = 1e13;

    OrbitCamera *orbitCam = new OrbitCamera(GetPosition(ship->controller),
                                            camFov, camAspect, camZNear, camZFar);
    // Free camera defaults to the orbit camera's initial pose so switching
    // between modes does not jump; an explicit --free-cam-* pose overrides it.
    glm::dvec3 freeCamPos  = orbitCam->GetPos();
    glm::dvec3 freeCamFwd  = orbitCam->GetForward();
    glm::dvec3 freeCamUp   = orbitCam->up;
    if(free_cam_pos.size() == 3) {
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
        printf("Camera: free flight (C = orbit, mouse = look)\n");
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
    bool teleport_beyond_soi_requested = false;
    bool poly_mode = false;
    bool capture_pointer = true;
    SDL_SetRelativeMouseMode(SDL_TRUE);

    const double dt = 1.0/50.0;
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

    /* main loop timing from
       http://gafferongames.com/game-physics/fix-your-timestep/
    */

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
            // printf("%.2f %.2f %.2f\n", p.x, p.y, p.z);
        }
        for(int i = 1; i < 128; i++) {
            glm::vec3 p = glm::vec3(r * cos((2 * M_PI) * float(i-1)/float(n)),
                                    0,
                                    r * sin((2 * M_PI) * float(i-1)/float(n)));
            skyinterface.positions.push_back(p);
        }
        skylines->InitMesh(skyinterface);
    }

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
                        cam_speed *= 10;
                    }
                }
                if(ev.key.keysym.sym == SDLK_k) {
                    if(cam_speed > 1) {
                        cam_speed /= 10;
                    }
                }
                // if(ev.key.keysym.sym == SDLK_g) {
                //   ship->Detach();
                // }
                if(ev.key.keysym.sym == SDLK_c) {
                    // Toggle between the body-orbit camera and the free camera.
                    if(camMode == CAM_ORBIT) {
                        // orbit -> free: start the free camera from the orbit
                        // camera's current pose so the view does not jump.
                        freeCam->pos = orbitCam->pos;
                        freeCam->forward = orbitCam->forward;
                        freeCam->up = orbitCam->up;
                        camMode = CAM_FREE;
                        camera = freeCam;
                        printf("Camera: free flight (C = orbit, mouse = look)\n");
                    } else {
                        // free -> orbit: refocus the orbit camera on the current
                        // target, keeping a similar distance.
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
                if(ev.key.keysym.sym == SDLK_t) {
                    teleport_beyond_soi_requested = true;
                }
                if(ev.key.keysym.sym == SDLK_m) {
                    physics_debug_drawing = not physics_debug_drawing;
                }
                if(ev.key.keysym.sym == SDLK_p) {
                    if(poly_mode == false) {
                        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                        poly_mode = true;
                    } else {
                        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                        poly_mode = false;
                    }
                }
                if(ev.key.keysym.sym == SDLK_v) {
                    capture_pointer = not capture_pointer;
                    if(capture_pointer == true) {
                        SDL_SetRelativeMouseMode(SDL_TRUE);
                    } else {
                        SDL_SetRelativeMouseMode(SDL_FALSE);
                    }
                }
                if(ev.key.keysym.sym == SDLK_t) {
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
            if(ev.type == SDL_MOUSEMOTION) {
                if(capture_pointer == true) {
                    camera->RotateY(-ev.motion.xrel / 200.0f);
                    camera->Pitch(ev.motion.yrel / 200.0f);
                }
            }
            if(ev.type == SDL_MOUSEWHEEL) {
                if(capture_pointer == true) {
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
            const Uint8* key = SDL_GetKeyboardState(NULL);
            if(key[SDL_SCANCODE_ESCAPE]) { running = false; }

            if(camMode == CAM_FREE) {
                // Free flight: E/Q roll, W/S forward/back, A/D strafe,
                // Shift/Ctrl up/down. Speed scales with the current zoom level
                // (orbit distance) so it stays usable at system scale.
                double base = orbitCam->distance;
                if(base < 1.0) { base = 1.0; }
                double freeScale = (double)cam_speed * base;

                if(key[SDL_SCANCODE_E]) { camera->Roll(0.05); }
                else if(key[SDL_SCANCODE_Q]) { camera->Roll(-0.05); }

                if(key[SDL_SCANCODE_W]) { camera->MoveForward(freeScale); }
                else if(key[SDL_SCANCODE_S]) { camera->MoveForward(-freeScale); }

                if(key[SDL_SCANCODE_A]) { camera->MoveRight(-freeScale); }
                else if(key[SDL_SCANCODE_D]) { camera->MoveRight(freeScale); }

                if(key[SDL_SCANCODE_LSHIFT] || key[SDL_SCANCODE_RSHIFT]) { camera->MoveUp(freeScale); }
                else if(key[SDL_SCANCODE_LCTRL] || key[SDL_SCANCODE_RCTRL]) { camera->MoveUp(-freeScale); }
            } else {
                // Orbit: E/Q yaw (W/S/A/D are no-ops for the orbit camera).
                if(key[SDL_SCANCODE_E]) { camera->RotateY(0.05); }
                else if(key[SDL_SCANCODE_Q]) { camera->RotateY(-0.05); }
            }

            if(key[SDL_SCANCODE_I]) { ship->ApplyThrust(); }
            if(key[SDL_SCANCODE_X]) { ship->KillRot(); }

            if(key[SDL_SCANCODE_L]) { ship->ApplyRotXPlus(); }
            else if(key[SDL_SCANCODE_J]) { ship->ApplyRotXMinus(); }

            if(key[SDL_SCANCODE_U]) { ship->ApplyRotYPlus(); }
            else if(key[SDL_SCANCODE_O]) { ship->ApplyRotYMinus(); }

            if(key[SDL_SCANCODE_Y]) { ship->ApplyRotZPlus(); }
            else if(key[SDL_SCANCODE_H]) { ship->ApplyRotZMinus(); }

            if(key[SDL_SCANCODE_B]) { ship->RotateToward(GetVelocity(ship->controller)); }
            if(key[SDL_SCANCODE_N]) { ship->RotateToward(-GetVelocity(ship->controller)); }

            if(key[SDL_SCANCODE_R]) { ship->ThrottleUp(); }
            else if(key[SDL_SCANCODE_F]) { ship->ThrottleDown(); }

            /* DEBUG: teleport the ship beyond the current frame's SOI (radially
               outward to 2x SOI) and zero its velocity, to force a reference-frame
               switch on the next physics tick. Triggered by the 't' key event. */
            if(teleport_beyond_soi_requested == true) {
                void setPosRot(Body *b, glm::dvec3 pos, glm::dmat3 rot);
                void SetVelocity(Body *b, glm::dvec3 vel);
                glm::dmat3 GetOrient(Body *b);
                double soi = ship->frame->soi;
                glm::dvec3 com = ship->get_center_of_mass();
                glm::dvec3 dir = glm::normalize(com);
                glm::dvec3 target = dir * (soi * 2.0);
                glm::dvec3 base = GetPosition(ship->parts[0]);
                for(auto&& part : ship->parts) {
                    glm::dvec3 p = GetPosition(part);
                    glm::dvec3 np = target + (p - base);
                    setPosRot(part, np, GetOrient(part));
                    SetVelocity(part, glm::dvec3(0, 0, 0));
                }
                /* teleporting is an explicit frame change; the SOI logic will
                   switch the ship to the correct frame on the next tick. */
                printf("DEBUG[t]: teleported to r=%.0f in %s (soi=%.0f)\n",
                       glm::length(target), ship->frame->name.c_str(), soi);
                teleport_beyond_soi_requested = false;
            }

            void physics_tick(float timeStep);
            void collisions();

            // // star system body updates
            // earth->transform = glm::rotate(earth->transform, 0.001,
            // 				     glm::dvec3(0, 1, 0));


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
                // before EACH substep. Two reasons:
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
                    physics_tick(h);
                }
            }

            // Periodic orbital-element printout (--orbit-log), once per
            // wall-clock interval, for measuring orbital stability. The state
            // is expressed in the current frame's body INERTIAL (non-rotating)
            // frame, where the trajectory is a Kepler conic — the same
            // conversion the orbit-info window does in the render section.
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
                           RAD2DEG(o.inclination), o.period,
                           o.ang_momentum, o.energy);
                    fflush(stdout);
                }
            }

            // collisions();

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

            // Point the orbit camera at the current focus target (the ship, or
            // whichever body is selected). The free camera keeps its own pos.
            if(camMode == CAM_ORBIT) {
                camera->Follow(focusWorldPos(focusBody));
            }

            for(auto&& planet : planets) {
                if(planet == ship->m_parent) {
                    /* this is the planet we're on.

                       This means its position is always 0, 0, 0
                    */

                    if(ship->frame->isRotFrame()) {
                        /* we're in its rotational frame */
                        planet->transform = glm::dmat4();
                    }
                    else {
                        /* we're in its inertial frame */
                        planet->transform = glm::dmat4(planet->frame->getRotFrame()->orient);
                    }
                }
                else {
                    /* other planets */
                    glm::dvec3 translate = planet->frame->GetPositionRelTo(ship->frame);
                    planet->transform =
                        glm::translate(translate) * glm::dmat4(planet->frame->getRotFrame()->orient);
                }
            }

            camera->ComputeView();

            /*
              standard 3d stuff drawn here
            */

            if(world_drawing == true) {
                space_port->Draw(camera, ship->m_parent);
                ship->Draw(camera);

            }

            for(auto&& planet : planets) {
                planet->Update(camera);
                if(world_drawing == true) {
                    planet->Draw(camera, sun);
                }
            }
            /*
              end 3d stuff drawn here
            */

            const double mu = ship->m_parent->mu;

            glm::dvec3 getRelAxis_(Body *body, int n);
            // surf pos??
            const glm::dvec3 pos = com;
            glm::dvec3 polar_pos = glm::polar(pos);
            /* orbital velocity */
            glm::dvec3 vel = ship->GetVel();

            // The orbit is a Kepler conic in the body's INERTIAL (non-rotating)
            // frame — that is the frame the spawn/switching code targets and
            // the frame in which the ship's trajectory is a conic.  Fit the
            // conic from the state expressed in that frame, otherwise the
            // same orbit reads as a different ellipse in each frame (a true
            // 610/1600 km orbit shows up as ~610/2700 km in the rotating one).
            glm::dvec3 orbit_pos = pos;
            glm::dvec3 orbit_vel = vel;
            if(ship->frame->isRotFrame() == true) {
                Frame *inertial = ship->frame->getNonRotFrame();
                orbit_vel += ship->frame->GetStasisVelocity(orbit_pos);
                orbit_vel = ship->frame->GetOrientRelTo(inertial) * orbit_vel
                          + ship->frame->GetVelocityRelTo(inertial);
                orbit_pos = ship->frame->GetOrientRelTo(inertial) * orbit_pos
                          + ship->frame->GetPositionRelTo(inertial);
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

            // raan = acos(n.x / norm(n))

            double raan = acos(node_vector.x / glm::length(node_vector));
            if(node_vector.y < 0) {
                raan = 2 * M_PI - raan;
            }

            // arg_pe = acos(dot(n, ev) / (norm(n) * norm(ev)))

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

            // damn wikipedia
            // double EccentricAnomaly = atan((sqrt(1 - ecc*ecc) * sin(TrueAnomaly)) / (ecc + cos(TrueAnomaly)));
            // http://www.bogan.ca/orbits/kepler/e_anomly.html
            double EccentricAnomaly = acos((((1 - ecc*ecc)*cos(TrueAnomaly)) / (1 + ecc * cos(TrueAnomaly))) + ecc);
            if(TrueAnomaly > M_PI) {
                EccentricAnomaly = 2 * M_PI - EccentricAnomaly;
            }

            const double MeanAnomaly = EccentricAnomaly - ecc * sin(EccentricAnomaly);
            const double PeT = sqrt((SMa * SMa * SMa) / (mu)) * MeanAnomaly; // s
            const double T = 2 * M_PI * sqrt((SMa * SMa * SMa) / (mu)); // s
            const double ApT = T - PeT; // s

            /*  y
                |
                |
                |
                /-----x
                /
                /
                z
            */

            const glm::dvec3 up = getRelAxis_(ship->controller, 1);
            const glm::dvec3 facing = getRelAxis_(ship->controller, 2);
            const glm::dvec3 other = getRelAxis_(ship->controller, 0);

            const glm::dvec3 facing_dir = glm::normalize(facing);
            const glm::dvec3 vel_dir = glm::normalize(vel);

            const glm::dvec3 _up = glm::normalize(pos);
            const glm::dvec3 _north = glm::normalize(projectVecOntoPlane(glm::dvec3(0, 1, 0), _up));
            const glm::dvec3 _east = glm::cross(_up, _north);

            const double ver_speed = glm::length(glm::proj(surf_vel, pos)); // m/s
            const glm::dvec3 surface_tangent = glm::cross(pos, glm::cross(pos, surf_vel)); // hmm
            // const double hor_speed = glm::length(glm::proj(surf_vel, surface_tangent)); // m/s
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
            // ImGui::Text("Pitch: %.2f", RAD2DEG(pitch));
            // ImGui::Text("Heading: %.2f", RAD2DEG(heading));
            // ImGui::Text("up: %.2f, %.2f, %.2f", up.x, up.y, up.z);
            // ImGui::Text("facing: %.2f, %.2f, %.2f", facing.x, facing.y, facing.z);

            const glm::dvec3 dir = glm::normalize(surf_pos);

            const double longitude = atan2(dir.x, dir.z);
            const double latitude = asin(dir.y);

            skybox.Draw(camera, skyboxshader);

            /* draw engine plume */
            glm::dmat4 View = camera->GetView();
            glm::mat4 Projection = camera->GetProjection();
            if(ship->m_thrust > 0) {
                for(auto&& thruster : ship->m_thrusters) {
                    glm::dmat4 Model = thruster->model_matrix;
                    glm::mat4 ModelViewFloat = View * Model;
                    engine_plume_model->shader->Bind();
                    engine_plume_model->shader->setUniform_mat4(0, Projection * ModelViewFloat);
                    engine_plume_model->shader->setUniform_mat4(1, glm::mat4());
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

            // for(auto&& frame : frames) {
            // 	ImGui::Text("root pos: %.0f, %.0f, %.0f",
            // 		    frame->root_pos.x,
            // 		    frame->root_pos.y,
            // 		    frame->root_pos.z);
            // }

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
                ImGui::Text("Inc: %.2f", RAD2DEG(inclination));
                ImGui::Text("Ecc: %f ", ecc);
                ImGui::Text("SMa: %.1fm", SMa);
                ImGui::Text("LAN: %.2f", RAD2DEG(raan));
                ImGui::Text("LPe: %.2f", RAD2DEG(arg_pe));
                double prograde_angle = glm::angle(facing_dir, vel_dir);
                double retrograde_angle = glm::angle(facing_dir, - vel_dir);
                ImGui::Text("Angle to Prograde: %.2f", RAD2DEG(prograde_angle));
                ImGui::Text("Angle to Retrograde: %.2f", RAD2DEG(retrograde_angle));
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
                ImGui::Text("Latitude: %.4f", RAD2DEG(latitude));
                ImGui::Text("Longitude: %.4f", RAD2DEG(longitude));
                ImGui::Text("Pitch: %.2f", RAD2DEG(pitch));
                ImGui::Text("Roll: %.2f", RAD2DEG(roll));
                ImGui::Text("Heading: %.2f", RAD2DEG(yaw));
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
                ImGui::Text("c - switch camera: orbit <-> free flight");
                ImGui::Text("g - orbit camera: cycle target (ship/sun/planet/moon)");
                ImGui::Text("mouse - look (both modes)");
                ImGui::Text("wheel - zoom (orbit mode)");
                ImGui::Text("free: w/s fwd-back, a/d strafe, shift/ctrl up-down, e/q roll");
                ImGui::Text("orbit: e/q yaw");
                ImGui::Text("v - capture/release mouse pointer");
                ImGui::Spacing();
                ImGui::Text("Ship");
                ImGui::Separator();
                ImGui::Text("i - fire ship engines");
                ImGui::Text("x - kill rotation");
                ImGui::Text("l & j - ship X rotation");
                ImGui::Text("u & o - ship Y rotation");
                ImGui::Text("y & h - ship Z rotation");
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

                ImVec2 ship_p = { 200 + p.x + distance / div * cos(phi),
                                  200 + p.y + distance / div * sin(phi) };

                ImVec2 raan_p = { 200 + p.x + 100 * cos(raan),
                                  200 + p.y + 100 * sin(raan) };

                /* incorrect */
                ImVec2 peri_p = { 200 + p.x + PeA / div * cos(arg_pe - M_PI / 2),
                                  200 + p.y + PeA / div * sin(arg_pe - M_PI / 2) };

                ImVec2 apo_p = { 200 + p.x + ApA / div * cos(arg_pe + M_PI / 2),
                                 200 + p.y + ApA / div * sin(arg_pe + M_PI / 2) };

                // auto whut = ;
                // ImGui::GetWindowDrawList()->AddPolyline(&planet[0], 26, color2, false, 1, true);

                ImGui::GetWindowDrawList()->AddPolyline(&pts[0], 26, color, false, 1.0);
                ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2( 200 + p.x,
                                                                    200 + p.y ),
                                                            ship->m_parent->radius / div,
                                      // 100 / (div / 6000),
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

    // Bodies own their frames (Frame::name is a std::string now), so deleting
    // every body releases exactly its inertial + rotating frames. The ship and
    // space port reference a body via ->m_parent, so delete them first.
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
