#ifndef MESH_INCLUDED_H
#define MESH_INCLUDED_H

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>

struct PosTexNorIndColInterface {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec2> texcoords;
    std::vector<glm::vec3> normals;
    std::vector<unsigned int> indices;
    std::vector<glm::vec3> colors;
};

struct PosTexNorIndInterface {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec2> texcoords;
    std::vector<glm::vec3> normals;
    std::vector<unsigned int> indices;
};

struct PosNorIndColInterface {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<unsigned int> indices;
    std::vector<glm::vec3> colors;
};

struct PosInterface {
    std::vector<glm::vec3> positions;
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
    // numInnerIndices: when nonzero, the first numInnerIndices indices are
    // the terrain and the tail is a skirt; DrawSkirt() renders the tail.
    void FromData(const PosNorColVertex* vertices, unsigned int numVertices, const unsigned int* indices, unsigned int numIndices, bool copyData, unsigned int numInnerIndices = 0);

    void InitMesh(const PosInterface& model);
    void InitMesh(const PosNorIndColInterface& model, bool copyData);
    void InitMesh(const PosTexNorIndInterface& model, bool copyData);
    void InitMesh(const PosTexNorIndColInterface& model, bool copyData);

    void Draw();
    // draw lines initialized by PosInterface
    void Draw(GLenum mode);
    // draw the skirt index tail (see FromData); drawn after Draw() so the
    // skirt depth-tests against the terrain in front of it
    void DrawSkirt();

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
    unsigned int m_numInnerIndices = 0;
};

#endif
