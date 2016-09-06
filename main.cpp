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
  * // Fix kill-rot
  * surface information (lat, long, hor, vert speeds)
  * Calculate orbital elements
  * walking around on the ground
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
  * ... lots more ...

 */

#include <stdio.h>
#include <chrono>

#include "SDL2/SDL.h"

#include <glm/gtc/noise.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/projection.hpp>
#include <glm/gtx/vector_angle.hpp>

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

#include "../../lib/imgui/imgui.h"
#include "../../lib/imgui/examples/sdl_opengl_example/imgui_impl_sdl.h"

ImFont *bigger;

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

class TerrainBody;

Mesh *create_grid_mesh(TerrainBody *body,
		       int depth,
                       float radius,
                       glm::vec3 p1, glm::vec3 p2,
                       glm::vec3 p3, glm::vec3 p4);


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
  void Draw(const Camera& camera);
  void Update(const Camera& camera);

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
  delete model->mesh;
  delete kids[0];
  delete kids[1];
  delete kids[2];
  delete kids[3];
  if(collision != NULL) {
    printf("removing terrain collision\n");
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
  Mesh *grid_mesh = create_grid_mesh(body, depth, body->radius, v0, v1, v2, v3);
  model->FromData(grid_mesh, shader);
  if(depth > max_depth) {
      collision = addTerrainCollision(grid_mesh);
      printf("added terrain collision with %p\n", this);
  } else {
      collision = NULL;
  }
}

void GeoPatch::Update(const Camera& camera) {
  if(depth > max_depth)
    return;

  const glm::dvec3& camera_pos = camera.GetPos();
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

  float thruster_util = 0.5;

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
  ;

public:
  Vehicle() { }
  virtual ~Vehicle() { }

  glm::dvec3 processGravity() {
    return applyGravity();
  }

  void Draw(const Camera& camera) {
    for(auto&& part : parts) { part->Draw(camera); }
  }

  void ThrottleUp() {
    thruster_util += 0.01;
    if(thruster_util > 1) thruster_util = 1;
  }
  void ThrottleDown() {
    thruster_util -= 0.01;
    if(thruster_util < 0) thruster_util = 0;
  }

  void ApplyThrust() {
    for(auto&& thruster : m_thrusters) {
      ApplyCentralForceForward(thruster, 100 * thruster_util);
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
    double angleFacing(Body *body, glm::dvec3 dir);

    glm::dvec3 facing = getRelAxis_(m_reaction_wheels.front(), 2);
    glm::dvec3 torque = -glm::normalize(glm::cross(dir, facing) / 10.0);

    ApplyTorque(m_reaction_wheels.front(), torque);
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
    noise = 0;
  }

  return radius + ScaleHeightNoise(noise);
}

float TerrainBody::GetTerrainHeightUnscaled(const glm::vec3& sphere_p) {
  // glm::vec3 sphere_coord = radius * glm::normalize(sphere_p);
  float noise = noise3d(sphere_p, 12, 0.60) * 2500;

  if(noise < 0) {
    noise = 0;
  }

  return radius + noise;
}

float TerrainBody::ScaleHeightNoise(float noise) {
  constexpr float max_height = 3000.0; // guess

  // rescale noise by altitude
  noise *= pow(noise / max_height, 2);

  return noise;
}

typedef struct {
    float r, g, b;
} COLOUR;

inline COLOUR GetColour(float v, float vmin, float vmax)
{
   COLOUR c = {1.0,1.0,1.0}; // white
   float dv;

   if (v < vmin)
      v = vmin;
   if (v > vmax)
      v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + 0.25 * dv)) {
      c.r = 0;
      c.g = 4 * (v - vmin) / dv;
   } else if (v < (vmin + 0.5 * dv)) {
      c.r = 0;
      c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
   } else if (v < (vmin + 0.75 * dv)) {
      c.r = 4 * (v - vmin - 0.5 * dv) / dv;
      c.b = 0;
   } else {
      c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
      c.b = 0;
   }

   return(c);
}

Mesh *create_grid_mesh(TerrainBody *body, int depth, float radius, glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4) {
  Mesh *grid_mesh = new Mesh;
  int size = 25;

  glm::vec3 blue = glm::vec3(0.1, 0.1, 0.8);

  Vertex vertices[size * size];
  unsigned int indices[size * size * 6] = {0};

  glm::vec2 dummyuv = glm::vec2(0, 0);
  glm::vec3 dummynormal = glm::vec3(1, 1, 1);

  for (int i = 0; i < size; i++) {
    for (int j = 0; j < size; j++) {
      glm::vec3 sphere_p = getSpherePoint(p1, p2, p3, p4, i/(float)(size-1), j/(float)(size-1));
      float height = body->GetTerrainHeightUnscaled(sphere_p);

      // set the color based on unscaled noise for better gradient
      COLOUR c = GetColour(height, radius - 1, radius + 3000);
      glm::vec3 color = glm::vec3(c.r, c.g, c.b);

      if(height <= radius) {
      	color = blue;
      }

      // add back scaling
      height = radius + body->ScaleHeightNoise(height - radius);

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

      vertices[j+size*i] = Vertex(p,
				  dummyuv,
				  dummynormal,
				  color);
    }
  }

  glm::dvec3 centroid = glm::normalize(p1 + p2 + p3 + p4);

  for (int i = 0; i < size; i++) {
    for (int j = 0; j < size; j++) {
      *vertices[j + size * i].GetNormal() = -centroid;
    }
  }

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

  grid_mesh->FromData(vertices, size * size, indices, size * size * 6);

  return grid_mesh;
}

// Mesh *create_triangle_mesh(float size_x, float size_y) {
//   Mesh *trig_mesh = new Mesh;

//   Vertex vertices[] = {
//     Vertex(glm::vec3(0,       0, -size_y), glm::vec2(1, 0), glm::vec3(0, 1, 0)),
//     Vertex(glm::vec3(-size_x, 0,  size_y), glm::vec2(0, 0), glm::vec3(0, 1, 0)),
//     Vertex(glm::vec3( size_x, 0,  size_y), glm::vec2(0, 1), glm::vec3(0, 1, 0)),
//   };

//   unsigned int indices[] = {
//     0, 1, 2,
//   };

//   trig_mesh->FromData(vertices, sizeof(vertices)/sizeof(vertices[0]), indices, sizeof(indices)/sizeof(indices[0]));
//   return trig_mesh;
// }

// Mesh *create_plane_mesh(float size_x, float size_y, glm::vec3 normal) {
//   Mesh *plane_mesh = new Mesh;

//   Vertex vertices[] = {
//     Vertex(glm::vec3(-size_x, 0, -size_y), glm::vec2(1, 0), normal),
//     Vertex(glm::vec3(-size_x, 0,  size_y), glm::vec2(0, 0), normal),
//     Vertex(glm::vec3( size_x, 0,  size_y), glm::vec2(0, 1), normal),
//     Vertex(glm::vec3( size_x, 0, -size_y), glm::vec2(1, 1), normal),
//   };

//   unsigned int indices[] = {
//     0, 1, 2,
//     0, 2, 3,
//   };

//   plane_mesh->FromData(vertices, sizeof(vertices)/sizeof(vertices[0]), indices, sizeof(indices)/sizeof(indices[0]));
//   return plane_mesh;
// }

Mesh *create_box_mesh(float size_x, float size_y, float size_z, glm::vec3 color) {
  Mesh *box_mesh = new Mesh;

  Vertex vertices[] = {
    // bottom
    Vertex(glm::vec3(-1, -1, -1), glm::vec2(1, 0), glm::vec3(0, 0, -1), color),
    Vertex(glm::vec3(-1, 1, -1), glm::vec2(0, 0), glm::vec3(0, 0, -1), color),
    Vertex(glm::vec3(1, 1, -1), glm::vec2(0, 1), glm::vec3(0, 0, -1), color),
    Vertex(glm::vec3(1, -1, -1), glm::vec2(1, 1), glm::vec3(0, 0, -1), color),

    // top
    Vertex(glm::vec3(-1, -1, 1), glm::vec2(1, 0), glm::vec3(0, 0, 1), color),
    Vertex(glm::vec3(-1, 1, 1), glm::vec2(0, 0), glm::vec3(0, 0, 1), color),
    Vertex(glm::vec3(1, 1, 1), glm::vec2(0, 1), glm::vec3(0, 0, 1), color),
    Vertex(glm::vec3(1, -1, 1), glm::vec2(1, 1), glm::vec3(0, 0, 1), color),

    // sides
    Vertex(glm::vec3(-1, -1, -1), glm::vec2(0, 1), glm::vec3(0, -1, 0), color),
    Vertex(glm::vec3(-1, -1, 1), glm::vec2(1, 1), glm::vec3(0, -1, 0), color),
    Vertex(glm::vec3(1, -1, 1), glm::vec2(1, 0), glm::vec3(0, -1, 0), color),
    Vertex(glm::vec3(1, -1, -1), glm::vec2(0, 0), glm::vec3(0, -1, 0), color),

    Vertex(glm::vec3(-1, 1, -1), glm::vec2(0, 1), glm::vec3(0, 1, 0), color),
    Vertex(glm::vec3(-1, 1, 1), glm::vec2(1, 1), glm::vec3(0, 1, 0), color),
    Vertex(glm::vec3(1, 1, 1), glm::vec2(1, 0), glm::vec3(0, 1, 0), color),
    Vertex(glm::vec3(1, 1, -1), glm::vec2(0, 0), glm::vec3(0, 1, 0), color),

    Vertex(glm::vec3(-1, -1, -1), glm::vec2(1, 1), glm::vec3(-1, 0, 0), color),
    Vertex(glm::vec3(-1, -1, 1), glm::vec2(1, 0), glm::vec3(-1, 0, 0), color),
    Vertex(glm::vec3(-1, 1, 1), glm::vec2(0, 0), glm::vec3(-1, 0, 0), color),
    Vertex(glm::vec3(-1, 1, -1), glm::vec2(0, 1), glm::vec3(-1, 0, 0), color),

    Vertex(glm::vec3(1, -1, -1), glm::vec2(1, 1), glm::vec3(1, 0, 0), color),
    Vertex(glm::vec3(1, -1, 1), glm::vec2(1, 0), glm::vec3(1, 0, 0), color),
    Vertex(glm::vec3(1, 1, 1), glm::vec2(0, 0), glm::vec3(1, 0, 0), color),
    Vertex(glm::vec3(1, 1, -1), glm::vec2(0, 1), glm::vec3(1, 0, 0), color),
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
  ImGui_ImplSdl_Init(display.get_display());

  ImGuiIO& io = ImGui::GetIO();
  io.Fonts->AddFontDefault();
  // io.Fonts->AddFontFromFileTTF("DroidSansMono.ttf", 14.0);
  bigger = io.Fonts->AddFontFromFileTTF("DroidSans.ttf", 40.0);

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

  // Mesh *trig_mesh = create_triangle_mesh(1, 1);
  // Model *trig_model = new Model;
  // trig_model->FromData(trig_mesh, shader);


  Vehicle *ship = new Vehicle;
  Body *space_port;
  {
    glm::vec3 grey = glm::vec3(0.5, 0.5, 0.5);
    glm::vec3 pink = glm::vec3(1.0, 192.0/255.0, 203.0/255.0);
    glm::vec3 red = glm::vec3(1,0,0);
    glm::vec3 blue = glm::vec3(0,0,1);

    Mesh *space_port_mesh = create_box_mesh(10, 10, 10, pink);
    Model *space_port_model = new Model;
    space_port_model->FromData(space_port_mesh, shader);

    Mesh *capsule_mesh = create_box_mesh(0.1, 0.1, 1.0, blue);
    Mesh *wheel_mesh = create_box_mesh(0.5, 0.5, 1.0, grey);
    Mesh *engine_mesh = create_box_mesh(1.0, 1.0, 1.0, red);

    Model *capsule_model = new Model;
    Model *wheel_model = new Model;
    Model *engine_model = new Model;
    capsule_model->FromData(capsule_mesh, shader);
    wheel_model->FromData(wheel_mesh, shader);
    engine_model->FromData(engine_mesh, shader);

    glm::dvec3 start(0);
    glm::dvec3 p = glm::normalize(glm::dvec3(0.005, 0.005, 1.0));
    double ground_alt = moon->GetTerrainHeight(p);
    start = ((ground_alt + 0.0f) * p);

    space_port =
      create_body(space_port_model, start.x-3, start.y-3, start.z + 5, 0,
    		  glm::vec4(1.0, 0.5, 1.0, 1.0), true);

    double ship_height = 14;

    // top
    Body *capsule =
      create_body(capsule_model, start.x, start.y, start.z + ship_height + 7, 0.5,
		  glm::vec4(0.5, 0.5, 0.5, 1.0) /*redundant*/, false);
    // middle
    Body *reaction_wheel =
      create_body(wheel_model, start.x, start.y, start.z + ship_height + 5, 1,
		  glm::vec4(0.9, 0.9, 0.9, 1.0), false);
    // bottom
    Body *thruster =
      create_body(engine_model, start.x, start.y, start.z + ship_height + 3, 3,
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
  int time_accel = 0;
  int cam_speed = 1;
  bool orbitInfoWindow = true;
  bool shipPartsWindow = true;
  bool gameInfoWindow = true;
  bool controlsWindow = true;
  bool autoPilotWindow = true;
  bool surfaceInfoWindow = true;

  /* main loop timing from
     http://gafferongames.com/game-physics/fix-your-timestep/
  */

  while (running == true) {
    /*
      EVENTS
    */
    SDL_Event ev;

    while (SDL_PollEvent(&ev)) {
      ImGui_ImplSdl_ProcessEvent(&ev);
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
	  if(cam_speed < 100000) {
	    cam_speed *= 10;
	  }
	}
	if(ev.key.keysym.sym == SDLK_k) {
	  if(cam_speed > 1) {
	    cam_speed /= 10;
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
	if(ev.key.keysym.sym == SDLK_t) {
	  // toggle stabilization
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

      if(key[SDL_SCANCODE_W]) { camera.MoveForward(cam_speed); }
      else if(key[SDL_SCANCODE_S]) { camera.MoveForward(-cam_speed); }

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

      if(time_accel != 0) {
	grav = ship->processGravity();
	physics_tick(dt * time_accel);
      }
      //collisions();

      accumulator -= dt;
    }

    redraw = true;

    /*
      RENDERING
    */
    if(redraw == true) {
      ImGui_ImplSdl_NewFrame(display.get_display());

      if(poly_mode == true) {
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      }

      display.Clear(0, 0, 0, 1);

      glm::dvec3 com = ship->get_center_of_mass();

      if(follow_ship == true) {
	camera.Follow(com);
      }

      moon->Update(camera);
      moon->Draw(camera);
      space_port->Draw(camera);
      ship->Draw(camera);

      const glm::dvec3 pos = com;
      const glm::dvec3 vel = ship->GetVel();
      const double distance = glm::length(pos);
      const double speed = glm::length(vel);
      // https://en.wikipedia.org/wiki/Standard_gravitational_parameter
      const double G = 6.674e-11;
      const double M = moon->mass;
      // https://en.wikipedia.org/wiki/Characteristic_energy
      const double e = (0.5 * pow(speed, 2)) - (G*M / distance);
      const double SMa = 1.0 / (-((speed * speed) / (G*M)) + (2.0 / distance));
      const double radial_vel = glm::dot(pos, vel) / distance;

      const glm::dvec3 ang_momentum = glm::cross(pos, vel);
      const glm::dvec3 eccentricity_vector = glm::cross(vel, ang_momentum) / (G * M) - pos/distance;

      const double ecc = glm::length(eccentricity_vector);

      const double ApA = (1 + ecc) * SMa;
      const double PeA = (1 - ecc) * SMa;

      const double inclination = acos(ang_momentum.z / glm::length(ang_momentum));

      const glm::dvec3 node_vector = glm::cross(glm::dvec3(0, 0, 1), ang_momentum);

      // raan = acos(n.x / norm(n))

      const double raan = acos(node_vector.x / glm::length(node_vector));

      // arg_pe = acos(dot(n, ev) / (norm(n) * norm(ev)))

      const double arg_pe = acos(
				 glm::dot(node_vector, eccentricity_vector) /
				 (glm::length(node_vector) * glm::length(eccentricity_vector))
				 );

      glm::dvec3 GetAngVelocity_(Body *b);
      const glm::dvec3 ang_vel_ = GetAngVelocity(ship->m_reaction_wheels.front());

      const glm::dvec3 AoA = glm::dvec3();

      const double TrueAnomaly = acos(glm::dot(eccentricity_vector, pos) / (glm::length(eccentricity_vector) * glm::length(pos)));
      double EccentricAnomaly = (sqrt(1 - ecc*ecc) * sin(TrueAnomaly)) / (ecc + cos(TrueAnomaly));
      if(EccentricAnomaly < 0) { EccentricAnomaly += 2 * M_PI; }
      const double MeanAnomaly = EccentricAnomaly - ecc * sin(EccentricAnomaly);
      const double PeT = sqrt((SMa * SMa * SMa) / (G*M)) * MeanAnomaly; // TODO units?
      const double T = 2 * M_PI * sqrt((SMa * SMa * SMa) / (G * M));
      const double ApT = T - PeT;

      const double ver_speed = glm::length(glm::proj(vel, pos));
      glm::dvec3 surface_tangent = glm::cross(pos, glm::cross(pos, vel)); // hmm
      const double hor_speed = glm::length(glm::proj(vel, surface_tangent));
      /*  y
	  |
          |
	  |
	  /-----x
	 /
	/
       z
       */
      const double longitude = glm::orientedAngle(glm::dvec3(0, 0, 1),
						 glm::normalize(glm::dvec3(pos.x, 0, pos.z)),
						 glm::dvec3(0, 1, 0)
						 );
      const double latitude = glm::orientedAngle(glm::dvec3(0, 1, 0),
						  glm::normalize(glm::dvec3(pos.x, pos.y, 0)),
						  glm::dvec3(0, 0, -1)
						  );

      // doesn't work any more?
      // glm::mat4 view = camera.GetView();
      // glm::mat4 proj = camera.GetProjection();
      glUseProgram(0);
      glBindBuffer(GL_ARRAY_BUFFER, 0);
      if(poly_mode == true) {
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      }

      ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize;
      ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.5, 0.5, 0.5, 1.0));
      ImGui::Begin("Top Middle Window", NULL, flags);
      ImGui::PushFont(bigger);
      int terrain_height = (int)(distance - moon->GetTerrainHeight(glm::normalize(pos)));
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
      ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.15, 0.15, 0.15, 1.0));

      ImGui::Begin("Open Space Program");
      ImGui::Spacing();
      ImGui::Checkbox("Orbit Info", &orbitInfoWindow);
      ImGui::Checkbox("Surface Info", &surfaceInfoWindow);
      ImGui::Checkbox("Vessel Info", &shipPartsWindow);
      ImGui::Checkbox("DUMB-ASS", &autoPilotWindow);
      ImGui::Checkbox("Controls Help", &controlsWindow);
      ImGui::Checkbox("Game Debug Info", &gameInfoWindow);
      ImGui::End();

      if(gameInfoWindow == true) {
	ImGui::Begin("Game Debug Info");
	ImGui::Text("Patches: %d", moon->CountPatches());
	ImGui::Text("Cam speed: %d", cam_speed);
	ImGui::Text("Time Accel: %d", time_accel);
	ImGui::Text("Camera altitude: %0.f",
		    glm::length(camera.GetPos()) - moon->GetTerrainHeight(glm::normalize(camera.GetPos())));
	ImGui::Text("Camera ASL: %0.f", glm::length(camera.GetPos()) - moon->radius);
	ImGui::Text("Camera Pos: %.0f %.0f %0.f", camera.GetPos().x, camera.GetPos().y, camera.GetPos().z);
	ImGui::End();
      }

      if(orbitInfoWindow == true) {
	ImGui::Begin("ORBITAL");
 	ImGui::Text("ApA: %.1fm", ApA);
	ImGui::Text("ApT: %.1f", ApT);
 	ImGui::Text("PeA: %.1fm", PeA);
	ImGui::Text("PeT: %.1f", PeT);
	ImGui::Text("T: %f", T);
	ImGui::Text("Inc: %frad", inclination);
	ImGui::Text("Ecc: %f ", ecc);
	ImGui::Text("SMa: %fm", SMa);
	ImGui::Text("Angle to Prograde:");
	ImGui::Text("Angle to Retrograde:");
	ImGui::Separator();
	ImGui::Text("Gravity (%.2f): %0.f %0.f %0.f", glm::length(grav), grav.x, grav.y, grav.z);
	ImGui::Text("Energy: %.2f kJ", e / 1000.0);
	ImGui::Text("Radial velocity: %.2f", radial_vel);
	ImGui::Text("Right Ascension of AN: %f", raan);
	ImGui::Text("Argument of Periapsis: %f", arg_pe);
	ImGui::Text("Ang Vel: %.2f %.2f %.2f", ang_vel_.x, ang_vel_.y, ang_vel_.z);
	ImGui::End();
      }

      if(surfaceInfoWindow == true) {
	ImGui::Begin("SURFACE");
	ImGui::Text("V speed: %.2fm/s", ver_speed);
	ImGui::Text("H speed: %.2fm/s", hor_speed);
	ImGui::Text("Latitude: %.4f", latitude * 180/M_PI);
	ImGui::Text("Longitude: %.4f", longitude * 180/M_PI);
	ImGui::Text("Pos (%.3fkm): %0.f %0.f %0.f", distance / 1000, pos.x, pos.y, pos.z);
	ImGui::Text("Vel (%.3fm/s): %0.f %0.f %0.f", speed, vel.x, vel.y, vel.z);
	ImGui::End();
      }

      if(shipPartsWindow == true) {
	ImGui::Begin("VESSEL");
	ImGui::Text("Thrust: %d%%", (int)(ship->thruster_util * 100));
	float TWR = (100 * ship->thruster_util) / (4.5 * 10);
	ImGui::Text("TWR: %.2f", TWR);
	for(auto&& part : ship->parts) {
	  // TODO
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

      ImGui::PopStyleColor();

      ImGui::Render();

      // glActiveTexture(GL_TEXTURE0);
      // glMatrixMode(GL_PROJECTION);
      // glLoadIdentity();
      // glMultMatrixf(&proj[0][0]);
      // glMatrixMode(GL_MODELVIEW);
      // glLoadIdentity();
      // glMultMatrixf(&view[0][0]);
      // debug_draw();

      display.SwapBuffers();
      check_gl_error();
    }
  }
  return 0;
}
