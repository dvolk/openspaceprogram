#include <stdio.h>
#include <chrono>

#include "SDL2/SDL.h"

#include <glm/gtc/noise.hpp>
#include <glm/gtx/norm.hpp>

#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>

#include "display.h"
#include "mesh.h"
#include "shader.h"
#include "texture.h"
#include "camera.h"
#include "model.h"
#include "body.h"
#include "physics.h"
#include "gldebug.h"

// static const int DISPLAY_WIDTH = 1440;
// static const int DISPLAY_HEIGHT = 900;
static const int DISPLAY_WIDTH = 720;
static const int DISPLAY_HEIGHT = 480;
static const int FPS = 60;

struct Frame {
    Frame *m_parent;
    std::vector<Frame *> m_children;

    btTransform m_transform;
    double m_angSpeed;
};

Mesh *create_grid_mesh(int depth,
                       float radius,
                       glm::vec3 p1, glm::vec3 p2,
                       glm::vec3 p3, glm::vec3 p4);


struct TerrainBody;

struct GeoPatch {
    TerrainBody *body;
    Model *model;
    GeoPatch *parent_geopatch;
    btRigidBody *collision;
    static const int max_depth = 5;
    int quadrant;

    GeoPatch *kids[4];

    glm::vec3 centroid;
    glm::vec3 v0, v1, v2, v3;

    int depth;

    GeoPatch(TerrainBody *body, Shader *shader, int depth, glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 v3);
    ~GeoPatch();

    void Subdivide(void);
    void Draw(const Camera& camera);
    void Update(const Camera& camera);
};

GeoPatch::~GeoPatch() {
    delete model->mesh;
    delete kids[0];
    delete kids[1];
    delete kids[2];
    delete kids[3];
    if(collision != NULL) {
        removeTerrainCollision(collision);
        delete collision;
    }
}

void GeoPatch::Subdivide(void) {
    printf("%p subdiving (%d)!\n", this, depth);
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

void GeoPatch::Draw(const Camera& camera) {
    if(kids[0] == NULL) {
        // patch isn't subdivided
        glm::vec4 color = glm::vec4(0.8, 0.8, 0.8, 1.0);
        glm::dmat4 id = glm::dmat4();
        model->shader->Bind();
        model->shader->Update(id, color, camera);
        model->mesh->Draw();
    }
    else {
        kids[0]->Draw(camera);
        kids[1]->Draw(camera);
        kids[2]->Draw(camera);
        kids[3]->Draw(camera);
    }
}

struct TerrainBody {
    GeoPatch *patches[6];
    Shader *shader;
    float radius;
    float mass;

    float GetTerrainHeight(const glm::dvec3& p);

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

        for(auto&& patch : patches) {
            patch->Subdivide();
            // patch->kids[0]->Subdivide();
            // patch->kids[1]->Subdivide();
            // patch->kids[2]->Subdivide();
            // patch->kids[3]->Subdivide();
        }
    }

    void Draw(const Camera& camera) {
        for(auto&& patch : patches) {
            // patch isn't subdivided
            patch->Draw(camera);
        }
    }

    void Update(const Camera& camera) {
        for(auto&& patch : patches) {
            patch->Update(camera);
        }
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
    Mesh *grid_mesh = create_grid_mesh(depth, body->radius, v0, v1, v2, v3);
    model->FromData(grid_mesh, shader);
    // if(depth > max_depth) {
    //     collision = addTerrainCollision(grid_mesh);
    //     printf("added terrain collision with %p\n", this);
    // } else {
    //     collision = NULL;
    // }
}

void GeoPatch::Update(const Camera& camera) {
    if(depth > max_depth)
        return;

    const glm::dvec3& camera_pos = camera.GetPos();
    const glm::dvec3& centroid_pos = body->radius * centroid;
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
    else if(depth >= 3 and dist > subdiv * 2) {
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
        kids[0]->Update(camera);
        kids[1]->Update(camera);
        kids[2]->Update(camera);
        kids[3]->Update(camera);
    }
}

class Vehicle {
public:
    std::vector<Body *> parts;

    TerrainBody *m_parent;

    glm::dvec3 m_com;

    Body *controller;
    std::vector<Body *> m_thrusters;
    std::vector<Body *> m_reaction_wheels;

    void setVelocity(glm::dvec3 vel) {
        for(auto&& part : parts) {
            SetVelocity(part, vel);
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
        // TODO simplify this computation?
        for(auto&& part : parts) {
            if(part->mass == 0) { continue; }
            const glm::dvec3& b1b2 = GetPosition(part);
            const double m1m2 = part->mass * parent_mass;
            const double invrsqr = 1.0 / glm::length2(b1b2);
            const double mag = G * m1m2 * invrsqr;
            gf = mag * sqrt(invrsqr) * -b1b2;
            ApplyCentralForce(part, gf, mag);
        }
    };

public:
    Vehicle() { }
    virtual ~Vehicle() { }

    glm::dvec3 processGravity() {
        return applyGravity();
    }

    void Draw(const Camera& camera) {
        for(auto&& part : parts) { part->Draw(camera); }
    }

    void ApplyThrust() {
        for(auto&& thruster : m_thrusters) {
            ApplyCentralForceForward(thruster, 50);
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

        if(ang_vel.x > angvel_treshhold) { ApplyRotXMinus(); }
        else if(ang_vel.x < -angvel_treshhold) { ApplyRotXPlus(); }
        if(ang_vel.y > angvel_treshhold) { ApplyRotYMinus(); }
        else if(ang_vel.y < -angvel_treshhold) { ApplyRotYPlus(); }
        if(ang_vel.z > angvel_treshhold) { ApplyRotZMinus(); }
        else if(ang_vel.z < -angvel_treshhold) { ApplyRotZPlus(); }
    }
    glm::dvec3 GetVel() {
        return GetVelocity(controller);
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

float noise3d(glm::vec3 p, int octaves, float persistence) {
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

float TerrainBody::GetTerrainHeight(const glm::dvec3& p) {
    glm::vec3 sphere_p = glm::normalize(p);
    glm::vec3 sphere_coord = radius * glm::normalize(sphere_p);
    glm::vec3 noise = sphere_p * (((noise3d(sphere_p, 3, 1) * 2000)
                                   + noise3d(sphere_p, 8, 8) / 10000));
    return glm::length(sphere_coord + noise);
}

Mesh *create_grid_mesh(int depth, float radius, glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4) {
    Mesh *grid_mesh = new Mesh;
    int size = 10;

    Vertex vertices[size * size];
    unsigned int indices[size * size * 6] = {0};
    //const float height = 500.0 / (depth * depth * depth);

    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            glm::vec3 sphere_p = getSpherePoint(p1, p2, p3, p4, i/(float)(size-1), j/(float)(size-1));
            glm::vec3 sphere_coord = radius * sphere_p;
            // glm::vec3 noise = sphere_p * (((noise3d(sphere_p, 3, 8) * 100) + noise3d(sphere_p, 8, 8) / 10000));
            glm::vec3 noise = sphere_p * (((noise3d(sphere_p, 3, 1) * 2000)
                                           + noise3d(sphere_p, 8, 8) / 10000));
            //glm::vec3 noise = glm::vec3(0);

            vertices[j+size*i] = Vertex(sphere_coord + noise,
                                        glm::vec2(0, 0),
                                        glm::normalize(glm::vec3(1, 1, 1)));
        }
    }

    glm::dvec3 centroid = glm::normalize(p1 + p2 + p3 + p4);

    for (int i = 1; i < size-1; i++) {
        for (int j = 1; j < size-1; j++) {
            glm::vec3 &x1 = vertices[j-1 + i*size].pos;
            glm::vec3 &x2 = vertices[j+1 + i*size].pos;
            glm::vec3 &y1 = vertices[j + (i-1)*size].pos;
            glm::vec3 &y2 = vertices[j + (i+1)*size].pos;
            // glm::vec3 n = centroid;//glm::normalize(centroid);
            glm::vec3 n = glm::normalize(glm::cross(x2-x1, y2-y1));
            *vertices[j + size * i].GetNormal() = n;
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

    // assert(find(indices.begin(), indices.end(), -1) == indices.end());

    grid_mesh->FromData(vertices, size * size, indices, size * size * 6);

    return grid_mesh;
}

Mesh *create_plane_mesh(float size_x, float size_y, glm::vec3 normal) {
    Mesh *plane_mesh = new Mesh;

    Vertex vertices[] = {
        Vertex(glm::vec3(-size_x, 0, -size_y), glm::vec2(1, 0), normal),
        Vertex(glm::vec3(-size_x, 0,  size_y), glm::vec2(0, 0), normal),
        Vertex(glm::vec3( size_x, 0,  size_y), glm::vec2(0, 1), normal),
        Vertex(glm::vec3( size_x, 0, -size_y), glm::vec2(1, 1), normal),
    };

    unsigned int indices[] = {
        0, 1, 2,
        0, 2, 3,
    };

    plane_mesh->FromData(vertices, sizeof(vertices)/sizeof(vertices[0]), indices, sizeof(indices)/sizeof(indices[0]));
    return plane_mesh;
}

Mesh *create_box_mesh(float size_x, float size_y, float size_z) {
    Mesh *box_mesh = new Mesh;

    Vertex vertices[] = {
        Vertex(glm::vec3(-1, -1, -1), glm::vec2(1, 0), glm::vec3(0, 0, -1)),
        Vertex(glm::vec3(-1, 1, -1), glm::vec2(0, 0), glm::vec3(0, 0, -1)),
        Vertex(glm::vec3(1, 1, -1), glm::vec2(0, 1), glm::vec3(0, 0, -1)),
        Vertex(glm::vec3(1, -1, -1), glm::vec2(1, 1), glm::vec3(0, 0, -1)),

        Vertex(glm::vec3(-1, -1, 1), glm::vec2(1, 0), glm::vec3(0, 0, 1)),
        Vertex(glm::vec3(-1, 1, 1), glm::vec2(0, 0), glm::vec3(0, 0, 1)),
        Vertex(glm::vec3(1, 1, 1), glm::vec2(0, 1), glm::vec3(0, 0, 1)),
        Vertex(glm::vec3(1, -1, 1), glm::vec2(1, 1), glm::vec3(0, 0, 1)),

        Vertex(glm::vec3(-1, -1, -1), glm::vec2(0, 1), glm::vec3(0, -1, 0)),
        Vertex(glm::vec3(-1, -1, 1), glm::vec2(1, 1), glm::vec3(0, -1, 0)),
        Vertex(glm::vec3(1, -1, 1), glm::vec2(1, 0), glm::vec3(0, -1, 0)),
        Vertex(glm::vec3(1, -1, -1), glm::vec2(0, 0), glm::vec3(0, -1, 0)),

        Vertex(glm::vec3(-1, 1, -1), glm::vec2(0, 1), glm::vec3(0, 1, 0)),
        Vertex(glm::vec3(-1, 1, 1), glm::vec2(1, 1), glm::vec3(0, 1, 0)),
        Vertex(glm::vec3(1, 1, 1), glm::vec2(1, 0), glm::vec3(0, 1, 0)),
        Vertex(glm::vec3(1, 1, -1), glm::vec2(0, 0), glm::vec3(0, 1, 0)),

        Vertex(glm::vec3(-1, -1, -1), glm::vec2(1, 1), glm::vec3(-1, 0, 0)),
        Vertex(glm::vec3(-1, -1, 1), glm::vec2(1, 0), glm::vec3(-1, 0, 0)),
        Vertex(glm::vec3(-1, 1, 1), glm::vec2(0, 0), glm::vec3(-1, 0, 0)),
        Vertex(glm::vec3(-1, 1, -1), glm::vec2(0, 1), glm::vec3(-1, 0, 0)),

        Vertex(glm::vec3(1, -1, -1), glm::vec2(1, 1), glm::vec3(1, 0, 0)),
        Vertex(glm::vec3(1, -1, 1), glm::vec2(1, 0), glm::vec3(1, 0, 0)),
        Vertex(glm::vec3(1, 1, 1), glm::vec2(0, 0), glm::vec3(1, 0, 0)),
        Vertex(glm::vec3(1, 1, -1), glm::vec2(0, 1), glm::vec3(1, 0, 0)),
    };

    int vertex_count = sizeof(vertices)/sizeof(vertices[0]);
    for(int i = 0; i < vertex_count; i++) {
        *vertices[i].GetPos() = glm::vec3(vertices[i].GetPos()->x * size_x,
                                          vertices[i].GetPos()->y * size_y,
                                          vertices[i].GetPos()->z * size_z);
        *vertices[i].GetNormal() = - *vertices[i].GetNormal();
    }

    unsigned int indices[] = {
        0, 1, 2,
        0, 2, 3,

        6, 5, 4,
        7, 6, 4,

        10, 9, 8,
        11, 10, 8,

        12, 13, 14,
        12, 14, 15,

        16, 17, 18,
        16, 18, 19,

        22, 21, 20,
        23, 22, 20
    };

    int index_count = sizeof(indices)/sizeof(indices[0]);

    box_mesh->FromData(vertices, vertex_count, indices, index_count);
    return box_mesh;
}

int main(int argc, char **argv)
{
    Renderer display(DISPLAY_WIDTH, DISPLAY_HEIGHT);

    //extern int text_init_resources();
    //assert(text_init_resources() == true);

    // start bullet; see physics.cpp
    void create_physics(void);
    create_physics();

    /* data init */
    Shader *shader = new Shader;
    shader->FromFile("./res/basicShader");

    // earth->Create(6300000, 5.97237e24);

    TerrainBody *moon = new TerrainBody;
    moon->shader = shader;
    moon->Create(600000, 5.2915793e22);

    Vehicle *ship = new Vehicle;
    Body *space_port;
    {
        Mesh *space_port_mesh = create_box_mesh(10, 10, 10);
        Model *space_port_model = new Model;
        space_port_model->FromData(space_port_mesh, shader);

        Mesh *box_mesh = create_box_mesh(0.5, 1.0, 0.5);

        Model *box_model = new Model;
        box_model->FromData(box_mesh, shader);

        glm::dvec3 start(0);
        glm::dvec3 p = glm::normalize(glm::dvec3(0.1, 0.1, 1));
        double ground_alt = moon->GetTerrainHeight(p);
        start = ((ground_alt + 600000) * p);

        space_port =
            create_body(space_port_model, start.x-10, start.y-10, start.z - 15, 0,
                        glm::vec4(1.0, 0.5, 1.0, 1.0), true);

        // top
        Body *capsule =
            create_body(box_model, start.x, start.y + 7, start.z + 50, 0.5,
                        glm::vec4(0.5, 0.5, 0.5, 1.0), false);
        // middle
        Body *reaction_wheel =
            create_body(box_model, start.x, start.y + 5, start.z + 50, 1,
                        glm::vec4(0.9, 0.9, 0.9, 1.0), false);
        // bottom
        Body *thruster =
            create_body(box_model, start.x, start.y + 3, start.z + 50, 1,
                        glm::vec4(1.0, 0.25, 0.25, 1.0), false);

        ship->parts = { capsule,
                        reaction_wheel,
                        thruster };
        ship->controller = capsule;
        ship->m_parent = moon;
        ship->m_thrusters = { thruster };
        ship->m_reaction_wheels = { reaction_wheel };
        ship->setVelocity(glm::dvec3(0, 0, 0));
        printf("%f\n", ship->controller->mass);
        NeverSleep(ship->controller);
        GlueTogether(reaction_wheel, thruster);
        GlueTogether(capsule, reaction_wheel);
    }

    /* camera init */
    Camera camera(glm::vec3(1000000.0f, 0.0f, 0.0f), 45.0f,
                  (float)DISPLAY_WIDTH / (float)DISPLAY_HEIGHT,
                  0.00001f, 10000000.0f);
    moon->Update(camera);

    bool running = true;
    bool redraw = false;
    bool follow_ship = true;
    bool poly_mode = false;
    bool capture_pointer = true;
    SDL_SetRelativeMouseMode(SDL_TRUE);

    const double dt = 1.0/50.0;
    double currentTime = 0.001 * (double)(SDL_GetTicks());
    double accumulator = 0.0;
    int time_accel = 1;

    /* main loop timing from
       http://gafferongames.com/game-physics/fix-your-timestep/
    */

    while (running == true) {
        /*
          EVENTS
        */
        SDL_Event ev;

        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) {
                running = false;
            }

            if (ev.type == SDL_WINDOWEVENT) {
                if(ev.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                    display.onResize(ev.window.data1, ev.window.data2);
                }
            }
            if(ev.type == SDL_KEYDOWN) {
                if(ev.key.keysym.sym == SDLK_PERIOD) {
                    if(time_accel < 1000) {
                        time_accel *= 10;
                    }
                }
                if(ev.key.keysym.sym == SDLK_COMMA) {
                    if(time_accel > 1) {
                        time_accel /= 10;
                    }
                }
                if(ev.key.keysym.sym == SDLK_c) {
                    follow_ship = not follow_ship;
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
            }
            if(ev.type == SDL_MOUSEMOTION) {
                if(capture_pointer == true) {
                    camera.RotateY(-ev.motion.xrel / 200.0f);
                    camera.Pitch(ev.motion.yrel / 200.0f);
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

        if(accumulator > 10 * dt) {
            accumulator = 10 * dt;
        }

        while (accumulator >= dt) {
            // is this logic? ;_;
            const Uint8* key = SDL_GetKeyboardState(NULL);
            if(key[SDL_SCANCODE_ESCAPE]) { running = false; }

            if(key[SDL_SCANCODE_E]) { camera.RotateY(0.05); }
            else if(key[SDL_SCANCODE_Q]) { camera.RotateY(-0.05); }

            if(key[SDL_SCANCODE_W]) { camera.MoveForward(1000); }
            else if(key[SDL_SCANCODE_S]) { camera.MoveForward(-1000); }

            if(key[SDL_SCANCODE_I]) { ship->ApplyThrust(); }
            if(key[SDL_SCANCODE_X]) { ship->KillRot(); }

            if(key[SDL_SCANCODE_L]) { ship->ApplyRotXPlus(); }
            else if(key[SDL_SCANCODE_J]) { ship->ApplyRotXMinus(); }

            if(key[SDL_SCANCODE_U]) { ship->ApplyRotYPlus(); }
            else if(key[SDL_SCANCODE_O]) { ship->ApplyRotYMinus(); }

            if(key[SDL_SCANCODE_Y]) { ship->ApplyRotXPlus(); }
            else if(key[SDL_SCANCODE_H]) { ship->ApplyRotZMinus(); }

            if(key[SDL_SCANCODE_R]) { camera.Pitch(0.1); }
            else if(key[SDL_SCANCODE_F]) { camera.Pitch(-0.1); }

            void physics_tick(float timeStep);
            void collisions();
            grav = ship->processGravity();
            physics_tick(dt * time_accel);
            //collisions();

            accumulator -= dt;
            redraw = true;
        }

        /*
          RENDERING
        */
        if(redraw == true) {
            display.Clear(0, 0, 0, 1);

            glm::dvec3 com = ship->get_center_of_mass();

            if(follow_ship == true) {
                camera.Follow(com);
            }

            moon->Update(camera);
            moon->Draw(camera);
            space_port->Draw(camera);
            ship->Draw(camera);

            glm::dvec3 pos = com;
            glm::dvec3 vel = ship->GetVel();
            double distance = glm::length(pos);
            double speed = glm::length(vel);
            // https://en.wikipedia.org/wiki/Standard_gravitational_parameter
            const double G = 6.674e-11;
            const double M = moon->mass;
            // https://en.wikipedia.org/wiki/Characteristic_energy
            const double e = (0.5 * pow(speed, 2)) - (G*M / distance);
            const double a = 1.0 / (-((speed * speed) / (G*M)) + (2.0 / distance));
            const double radial_vel = glm::dot(pos, vel) / distance;

            printf("pos(%0.fkm): %0.f %0.f %0.f, vel(%0.fm/s): %0.f %0.f %0.f, grav(%0.f): %0.f %0.f %0.f, energy: %f, SMa: %f, r: %f\n",
                   distance / 1000, pos.x, pos.y, pos.z,
                   speed, vel.x, vel.y, vel.z,
                   glm::length(grav), grav.x, grav.y, grav.z,
                   e,
                   a,
                   radial_vel
                   );

            void drawtext(const char *fmt, ...);
            //drawtext("Speed: %0.fm/s, time accel: %dx", speed, time_accel);

            // doesn't work any more?
            glm::mat4 view = camera.GetView();
            glm::mat4 proj = camera.GetProjection();
            glUseProgram(0);
            glActiveTexture(GL_TEXTURE0);
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glMultMatrixf(&proj[0][0]);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            glMultMatrixf(&view[0][0]);
            debug_draw();

            display.SwapBuffers();
            check_gl_error();
        }
    }
    return 0;
}
