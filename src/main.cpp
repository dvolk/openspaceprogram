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
#include <chrono>

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
#include "billboard.h"
#include "texture.h"
#include "skybox.h"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include "../middleware/imgui/imgui.h"
#include "../middleware/imgui/examples/sdl_opengl3_example/imgui_impl_sdl_gl3.h"

ImFont *bigger;
bool planetsWindow = false;

// static const int DISPLAY_WIDTH = 1440;
// static const int DISPLAY_HEIGHT = 900;
static const int DISPLAY_WIDTH = 1920;
static const int DISPLAY_HEIGHT = 1080;
static const int FPS = 60;

#define RAD2DEG(rad) (((180.0/M_PI) * rad))

struct TerrainBody;

std::vector<Frame *> setup_frames() {
    Frame *sun = new Frame;
    Frame *eerbon = new Frame;
    Frame *eerbon_rot = new Frame;
    Frame *moon = new Frame;
    Frame *moon_rot = new Frame;

    bool correct_rot_speeds = true;

    /*
      0
    */
    sun->name = "sun (inertial)";
    sun->parent = NULL;
    sun->children = std::vector<Frame *>{ eerbon };
    sun->rotating = false;
    sun->has_rot_frame = false;
    sun->pos = glm::dvec3(0);
    sun->initial_pos = sun->pos;
    sun->initial_orient = glm::dmat3();
    sun->orient = glm::dmat3();
    sun->vel = glm::dvec3(0);
    sun->rot_ang_speed = 0;
    sun->orb_ang_speed = 0;
    sun->soi = 9999999999999999;
    sun->root_pos = glm::dvec3(0);
    sun->root_vel = glm::dvec3(0);
    sun->root_orient = glm::dmat3();

    /*
      1
    */
    eerbon->name = "eerbon (inertial)";
    eerbon->parent = sun;
    eerbon->rotating = false;
    eerbon->has_rot_frame = true;
    eerbon->children = std::vector<Frame *>{ eerbon_rot, moon };
    // eerbon->pos = glm::dvec3(0, 0, -13599840256);
    eerbon->pos = glm::dvec3(0, 0, -100000000);
    eerbon->initial_pos = eerbon->pos;
    eerbon->initial_orient = glm::dmat3();
    eerbon->orient = glm::dmat3();
    eerbon->vel = glm::dvec3(0);
    eerbon->orb_ang_speed = 0.01;
    if(correct_rot_speeds)
        eerbon->orb_ang_speed = 0.00000068269186570822291594437651; // rad/s;
    eerbon->rot_ang_speed = 0;
    eerbon->soi = 84159286;
    eerbon->root_pos = glm::dvec3(0);
    eerbon->root_vel = glm::dvec3(0);
    eerbon->root_orient = glm::dmat3();

    /*
      2
    */
    eerbon_rot->name = "eerbon (rotational)";
    eerbon_rot->parent = eerbon;
    eerbon_rot->rotating = true;
    eerbon_rot->has_rot_frame = false;
    eerbon_rot->children = std::vector<Frame *>{ };
    eerbon_rot->pos = glm::dvec3(0, 0, 0);
    eerbon_rot->initial_pos = glm::dvec3(0, 0, 0);
    eerbon_rot->initial_orient = glm::dmat3();
    eerbon_rot->orient = glm::dmat3();
    eerbon_rot->vel = glm::dvec3(0);
    eerbon_rot->orb_ang_speed = 0;
    eerbon_rot->rot_ang_speed = 0.01;
    if(correct_rot_speeds)
        eerbon_rot->rot_ang_speed = 0.00029157090303706880702966723086; // rad/s
    eerbon_rot->soi = 700000; // 700km
    eerbon_rot->root_pos = glm::dvec3(0);
    eerbon_rot->root_vel = glm::dvec3(0);
    eerbon_rot->root_orient = glm::dmat3();

    /*
      3
    */
    moon->name = "moon (inertial)";
    moon->parent = eerbon;
    moon->rotating = false;
    moon->has_rot_frame = true;
    moon->children = std::vector<Frame *> { moon_rot };
    moon->pos = glm::dvec3(-12e6, 0, 0);
    moon->initial_pos = glm::dvec3(-12e6, 0, 0);
    moon->initial_orient = glm::dmat3();
    moon->orient = glm::dmat3();
    moon->vel = glm::dvec3(0);
    moon->orb_ang_speed = 0.001;
    if(correct_rot_speeds)
        moon->orb_ang_speed = 0.00004520797578987211820731369629; // rad/s
    moon->rot_ang_speed = 0; // rad/s
    moon->soi = 2429559.1;
    moon->root_pos = glm::dvec3(0);
    moon->root_vel = glm::dvec3(0);
    moon->root_orient = glm::dmat3();

    /*
      4
    */
    moon_rot->name = "moon (rotational)";
    moon_rot->parent = moon;
    moon_rot->rotating = true;
    moon_rot->has_rot_frame = false;
    moon_rot->children = std::vector<Frame *> { };
    moon_rot->pos = glm::dvec3(0, 0, 0);
    moon_rot->initial_pos = glm::dvec3(0, 0, 0);
    moon_rot->initial_orient = glm::dmat3();
    moon_rot->orient = glm::dmat3();
    moon_rot->vel = glm::dvec3(0);
    moon_rot->orb_ang_speed = 0;
    moon_rot->rot_ang_speed = 0.001; // rad/s
    if(correct_rot_speeds)
        moon_rot->rot_ang_speed = 0.00004520785218583258404235991675; // rad/s
    moon_rot->soi = 300000;
    moon_rot->root_pos = glm::dvec3(0);
    moon_rot->root_vel = glm::dvec3(0);
    moon_rot->root_orient = glm::dmat3();

    return std::vector<Frame *>{
        sun, // 0
        eerbon,// 1
        eerbon_rot,// 2
        moon,// 3
        moon_rot// 4
    };
}

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

struct TerrainBody {
    GeoPatch *patches[6];
    Shader *shader;
    float radius;
    double mu;
    double g; // [m/s^2]
    double soi; // [m]
    float mass;
    const char *name;
    double seed = 1;
    bool has_sea;
    int power_scaler;
    bool moves = false;
    Frame *frame;
    glm::dmat4 transform;
    glm::vec3 sunlightVec;
    int dbg_drew_patches;

    ~TerrainBody() {
        for(int i = 0; i < 6; i++) { delete patches[i]; }
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
            ImGui::Text("%s", name);
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
        printf("added terrain collision with %p\n", this);
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

    float thruster_util = 0.5;

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
        for(int i = 0; i < parts.size(); i++) {
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
        float exaust_velocity = 10123; /* m/s */
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
            glm::dvec3 newVel = vel + newFrame->GetStasisVelocity(pos);
            printf("@@@ %s OLD velocity: %.0f %.0f %.0f\n", name, oldVel.x, oldVel.y, oldVel.z);
            printf("@@@ %s NEW velocity: %.0f %.0f %.0f\n", name, newVel.x, newVel.y, newVel.z);

            /*
              if we're moving from inertial to rotational frame, subtract stasis velocity?
            */

            SetVelocity(part, newVel);


        }
        frame = newFrame;
        m_parent = newFrame->body;
    }
};

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
    // glm::vec3 sphere_coord = radius * glm::normalize(sphere_p);
    float noise = noise3d(sphere_p, 12, 0.60) * 2500;

    if(noise < 0) {
        if(has_sea == true) {
            noise = 0;
        }
    }

    return radius + ScaleHeightNoise(noise);
}

float TerrainBody::GetTerrainHeightUnscaled(const glm::vec3& sphere_p) {
    // glm::vec3 sphere_coord = radius * glm::normalize(sphere_p);
    float noise = noise3d(sphere_p, 12, 0.60) * 2500;

    if(noise < 0) {
        if(has_sea == true) {
            noise = 0;
        }
    }

    return radius + noise;
}

float TerrainBody::ScaleHeightNoise(float noise) {
    constexpr float max_height = 3000.0; // guess

    // rescale noise by altitude
    noise *= pow(noise / max_height, power_scaler);

    return noise;
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
    int size = 25;

    glm::vec3 blue = glm::vec3(0.1, 0.1, 0.8);

    PosNorColVertex vertices[size * size];
    unsigned int indices[size * size * 6] = {0};

    glm::vec2 dummyuv = glm::vec2(0, 0);

    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            glm::vec3 sphere_p = getSpherePoint(p1, p2, p3, p4, i/(float)(size-1), j/(float)(size-1));
            float height = GetTerrainHeightUnscaled(sphere_p);

            // add some color noise
            float color_noise = noise3d(sphere_p * radius, 1, 0.60) * 100;

            COLOUR c = (*colour_func)(height + ((color_noise / 2) - color_noise), radius - 1, radius + 3000);


            // set the color based on unscaled noise for better gradient
            glm::vec3 color = glm::vec3(c.r, c.g, c.b);
            float brightness = (c.r + c.g + c.b) / 6;

            // lower contrast, increase brightness
            color = float(0.5) * color + glm::vec3(brightness,
                                                   brightness,
                                                   brightness);

            if(height <= radius) {
                if(has_sea == true) {
                    color = blue;
                }
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

int main(int argc, char **argv)
{
    Renderer display(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    check_gl_error();
    ImGuiContext* ctx1 = ImGui::CreateContext();
    ImGui::SetCurrentContext(ctx1);
    ImGui_ImplSdlGL3_Init(display.get_display());
    check_gl_error();

    ImGuiIO& io = ImGui::GetIO();
    //io.Fonts->AddFontDefault();
    io.Fonts->AddFontFromFileTTF("./res/DejaVuSansMono.ttf", 20.0);
    bigger = io.Fonts->AddFontFromFileTTF("./res/DroidSans.ttf", 40.0);
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
    sunshader->FromFile("./res/terrainShader");

    Shader *skyboxshader = new Shader;
    skyboxshader->registerAttribs({ "position" });
    skyboxshader->registerUniforms({ "projectionview" });
    skyboxshader->FromFile("./res/skyboxShader");

    Shader *lineshader = new Shader;
    lineshader->registerAttribs({ "position" });
    lineshader->registerUniforms({ "MVP", "color" });
    lineshader->FromFile("./res/lineShader2");

    // earth->Create(6300000, 5.97237e24);

    TerrainBody *sun = new TerrainBody;
    sun->shader = sunshader;
    sun->name = "Sun";
    sun->colour_func = GetColourSun;
    sun->seed = 0.1;
    sun->moves = false;
    sun->has_sea = false;
    sun->power_scaler = 1;
    sun->g = 17.1;
    sun->mu = 1.1723328e18;
    sun->Create(6000000, 9.7600236e20);

    TerrainBody *earth = new TerrainBody;
    earth->shader = terrainshader;
    earth->name = "Eerbon";
    earth->colour_func = GetColourEerbon;
    earth->seed = 1;
    earth->has_sea = true;
    earth->power_scaler = 3;
    earth->g = 9.81;
    earth->mu = 3.5316000e12;
    earth->Create(600000, 5.2915793e22);

    TerrainBody *moon = new TerrainBody;
    moon->shader = terrainshader;
    moon->name = "Moon";
    moon->colour_func = GetColourMoon;
    moon->seed = 0.1;
    moon->moves =true;
    moon->has_sea = false;
    moon->power_scaler = 1;
    moon->g = 1.63;
    moon->mu = 6.5138398e10;
    moon->Create(200000, 9.7600236e20);

    std::vector<Frame *> frames = setup_frames();

    sun->frame = frames[0];
    earth->frame = frames[1];
    moon->frame = frames[3];
    sun->frame->body = sun;
    earth->frame->body = earth;
    moon->frame->body = moon;
    frames[2]->body = earth;
    frames[4]->body = moon;

    sun->frame->UpdateOrbitRails(0, 1/60.0);

    std::vector<TerrainBody *> planets = { sun, earth, moon };

    Vehicle *ship = new Vehicle;
    ship->m_parent = earth;
    ship->frame = frames[2]; // moon rotational

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

        glm::dvec3 start(0);
        glm::dvec3 p = glm::normalize(glm::dvec3(0.005, 0.005, 1.0));
        double ground_alt = ship->m_parent->GetTerrainHeight(p);
        start = ((ground_alt + 0.0f) * p);

        space_port = new StaticBuilding;
        space_port->body = create_body(space_port_model, start.x, start.y, start.z + 5, 0, false);
        space_port->parent = ship->m_parent;

        // double ship_height = 190000.5;
        double ship_height = 3.5;

        // top
        Body *capsule =
            create_body(capsule_model, start.x, start.y, start.z + ship_height + 7, 0.5, true);
        // middle
        Body *reaction_wheel =
            create_body(wheel_model, start.x, start.y, start.z + ship_height + 5, 1.0, true);
        // bottom
        Body *thruster =
            create_body(engine_model, start.x, start.y, start.z + ship_height + 3, 3.0, true);

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
    // WeirdCamera *camera = new WeirdCamera(glm::vec3(1000000.0f, 0.0f, 0.0f), 45.0f,
    // 					(float)DISPLAY_WIDTH / (float)DISPLAY_HEIGHT,
    // 					0.00001f, 10e6);
    // focused_planet->Update(camera);
    OrbitCamera *camera = new OrbitCamera(GetPosition(ship->controller),
                                          M_PI/3.0, (float)DISPLAY_WIDTH / (float)DISPLAY_HEIGHT,
                                          1.0f, 10e6);

    // Camera *camera = camera1;

    bool running = true;
    bool redraw = false;
    bool follow_ship = false;
    bool poly_mode = false;
    bool capture_pointer = true;
    SDL_SetRelativeMouseMode(SDL_TRUE);

    const double dt = 1.0/50.0;
    double currentTime = 0.001 * (double)(SDL_GetTicks());
    double accumulator = 0.0;
    int time_accel = 0;
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
            ImGui_ImplSdlGL3_ProcessEvent(&ev);
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
                    follow_ship = not follow_ship;
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

            if(key[SDL_SCANCODE_E]) { camera->RotateY(0.05); }
            else if(key[SDL_SCANCODE_Q]) { camera->RotateY(-0.05); }

            if(key[SDL_SCANCODE_W]) { camera->MoveForward(cam_speed); }
            else if(key[SDL_SCANCODE_S]) { camera->MoveForward(-cam_speed); }

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

            void physics_tick(float timeStep);
            void collisions();

            // // star system body updates
            // earth->transform = glm::rotate(earth->transform, 0.001,
            // 				     glm::dvec3(0, 1, 0));


            time += 1/60.0 * time_accel;

            if(time_accel != 0) {
                sun->frame->UpdateOrbitRails(time, 1/60.0 * time_accel);

                com = ship->get_center_of_mass();
                double ship_r = glm::length(com);
                if(ship_r > ship->frame->soi + 10000) {
                    // switching to parent SOI if there is one
                    if(ship->frame->parent != NULL) {
                        glm::dvec3 pos = GetPosition(ship->controller);
                        printf("@@@ switching frame from %s to parent %s\n",
                               ship->frame->name,
                               ship->frame->parent->name
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
                                   ship->frame->name,
                                   child->name,
                                   dist
                                  );
                            ship->moveToFrame(child);
                            break;
                        }
                    }
                }

                grav = ship->processGravity();
                physics_tick(dt * time_accel);
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
            ImGui_ImplSdlGL3_NewFrame(display.get_display());
            check_gl_error();

            if(poly_mode == true) {
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                check_gl_error();
            }

            display.Clear(0, 0, 0, 1);

            com = ship->get_center_of_mass();

            if(follow_ship == true) {
                camera->Follow(com);
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
                if(planet == sun) continue;
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

            glm::dvec3 surf_pos = pos;
            glm::dvec3 surf_vel = vel;

            if(ship->frame->isRotFrame() == false and
               ship->frame->hasRotFrame() == true) {
                glm::dvec3 stasis_vel = ship->frame->getRotFrame()->GetStasisVelocity(pos);
                surf_vel -= stasis_vel;
                surf_pos = ship->frame->getRotFrame()->orient * pos;
            }

            const double distance = glm::length(pos);
            const double speed = glm::length(vel);
            // https://en.wikipedia.org/wiki/Standard_gravitational_parameter
            // const double G = 6.674e-11;
            // const double M = moon->mass;
            // https://en.wikipedia.org/wiki/Characteristic_energy
            const double e = (0.5 * pow(speed, 2)) - (mu / distance);
            const double SMa = 1.0 / (-((speed * speed) / (mu)) + (2.0 / distance));
            const double radial_vel = glm::dot(pos, vel) / distance;

            const glm::dvec3 ang_momentum = glm::cross(pos, vel);
            const glm::dvec3 eccentricity_vector = glm::cross(vel, ang_momentum) / (mu) - pos/distance;

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

            double TrueAnomaly = acos(glm::dot(eccentricity_vector, pos) / (glm::length(eccentricity_vector) * glm::length(pos)));
            if(glm::dot(pos, vel) < 0) {
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
                ImGui::Text("Earth distance: %f",
                            glm::length(ship->GetPositionRelTo(ship->controller, earth->frame)));
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
                ImGui::Text("Reference frame: %s", ship->frame->name);
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
                ImGui::Text("c - toggle camera ship follow mode");
                ImGui::Text("v - capture/release mouse pointer");
                ImGui::Text("e & q - roll camera");
                ImGui::Text("w & s - forward/backward camera");
                ImGui::Text("r & f - camera pitch");
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
            ImGui_ImplSdlGL3_RenderDrawData(ImGui::GetDrawData());

            display.SwapBuffers();
            check_gl_error();
        }
    }

    delete sun;
    delete earth;
    delete moon;

    delete space_port;
    delete ship;

    for(auto&& frame : frames) { delete frame; }

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

    return 0;
}
