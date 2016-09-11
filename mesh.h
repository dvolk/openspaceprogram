#ifndef MESH_INCLUDED_H
#define MESH_INCLUDED_H

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include "obj_loader.h"

struct Vertex
{
public:
    Vertex() {};
  Vertex(const glm::vec3& pos, const glm::vec2& texCoord, const glm::vec3& normal, const glm::vec3& color)
    {
        this->pos = pos;
        this->texCoord = texCoord;
        this->normal = normal;
	this->color = color;
    }
  Vertex(const glm::vec3& pos, const glm::vec2& texCoord, const glm::vec3& normal)
    {
      static const glm::vec3 pink = glm::vec3(1.0, 192.0/255.0, 203.0/255.0);
        this->pos = pos;
        this->texCoord = texCoord;
        this->normal = normal;
	this->color = pink;
    }

    glm::vec3* GetPos() { return &pos; }
    glm::vec2* GetTexCoord() { return &texCoord; }
    glm::vec3* GetNormal() { return &normal; }

/* private: */
    glm::vec3 pos;
    glm::vec2 texCoord;
    glm::vec3 normal;
  glm::vec3 color;
};

enum MeshBufferPositions
    {
	POSITION_VB,
	TEXCOORD_VB,
	NORMAL_VB,
	INDEX_VB,
	COLOR_VB
    };

class Mesh
{
 public:
  void AssImpFromFile(const std::string& fileName);
    void FromFile(const std::string& fileName);
    void FromData(Vertex* vertices, unsigned int numVertices, unsigned int* indices, unsigned int numIndices);

    void Draw();

    glm::vec3 color;

    virtual ~Mesh();

    // for bullet physics
    double *vs;
    int num_vertices;
    int *is;
    int num_indices;

 protected:
 private:
    static const unsigned int NUM_BUFFERS = 5;

    void InitMesh(const IndexedModel& model);

    GLuint m_vertexArrayObject;
    GLuint m_vertexArrayBuffers[NUM_BUFFERS];
    unsigned int m_numIndices;
};

#endif
