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

    /* true on success; on failure the Mesh is left empty (no VAO, no
       buffers) -- a caller that might share the result must check. */
    bool AssImpFromFile(const std::string& fileName, bool copyData);
    bool FromFile(const std::string& fileName, bool copyData);
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
    double *vs = NULL;
    unsigned int num_vertices = 0;
    int *is = NULL;
    unsigned int num_indices = 0;

private:

    /* defaults keep an empty (import-failed) Mesh destructible: no VAO,
       no buffers, nothing to delete. */
    int num_VABs = 0;
    GLuint *m_vertexArrayBuffers = NULL;
    GLuint m_vertexArrayObject = 0;
    unsigned int m_numIndices = 0;
    unsigned int m_numInnerIndices = 0;
};

/* Shared file-asset registry (mesh.cpp): lookup-or-load, ONE Mesh per
   file, shared by every part/pad that uses it. The registry owns the
   mesh (lives until process exit), so callers must never delete it.
   The CPU-side vs/is copies are ALWAYS kept (BuildPartHull's convex hull
   needs them and asserts their presence), and a file that fails to import
   yields a unit-cube placeholder instead of a half-built Mesh. */
Mesh *get_mesh(const std::string &path);

#endif
