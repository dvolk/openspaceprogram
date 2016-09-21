#ifndef MESH_INCLUDED_H
#define MESH_INCLUDED_H

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>
//#include "obj_loader.h"

struct PosTexNorIndColInterface {
  std::vector<glm::vec3> positions;
  std::vector<glm::vec2> texCoords;
  std::vector<glm::vec3> normals;
  std::vector<unsigned int> indices;
  std::vector<glm::vec3> colors;
};

struct PosNorIndColInterface {
  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  std::vector<unsigned int> indices;
  std::vector<glm::vec3> colors;
};

struct PosVertex {
  glm::vec3 pos;

  PosVertex(float x, float y, float z) {
    pos.x = x;
    pos.y = y;
    pos.z = z;
  }
};

struct PosNorColVertex {
  glm::vec3 pos;
  glm::vec3 normal;
  glm::vec3 color;

  PosNorColVertex() {
  }

  PosNorColVertex(const glm::vec3& pos, const glm::vec3& normal, const glm::vec3& color) {
    this->pos = pos;
    this->normal = normal;
    this->color = color;
  }

  PosNorColVertex(const glm::vec3& pos, const glm::vec3& normal) {
    static const glm::vec3 pink = glm::vec3(1.0, 192.0/255.0, 203.0/255.0);
    this->pos = pos;
    this->normal = normal;
    this->color = pink;
  }
};

class Mesh
{
 public:
  virtual ~Mesh();

  void AssImpFromFile(const std::string& fileName, bool copyData);
  void FromFile(const std::string& fileName, bool copyData);
  void FromData(PosNorColVertex* vertices, unsigned int numVertices, unsigned int* indices, unsigned int numIndices, bool copyData);
  void InitMesh(const PosNorIndColInterface& model, bool copyData);

  void Draw();

  glm::vec3 color;

  // for bullet physics
  double *vs;
  unsigned int num_vertices;
  int *is;
  unsigned int num_indices;

 private:

  int num_VABs;
  GLuint *m_vertexArrayBuffers;
  GLuint m_vertexArrayObject;
  unsigned int m_numIndices;
};

#endif
