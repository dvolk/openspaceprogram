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

    // true on success; on failure the Mesh is left empty (no VAO, no buffers).
    bool AssImpFromFile(const std::string& fileName, bool copyData);
    bool FromFile(const std::string& fileName, bool copyData);
    // numInnerIndices: when nonzero, the first N indices are terrain and the
    // tail is a skirt; DrawSkirt() renders the tail.
    void FromData(const PosNorColVertex* vertices, unsigned int numVertices, const unsigned int* indices, unsigned int numIndices, bool copyData, unsigned int numInnerIndices = 0);

    void InitMesh(const PosInterface& model);
    void InitMesh(const PosNorIndColInterface& model, bool copyData);
    void InitMesh(const PosTexNorIndInterface& model, bool copyData);
    void InitMesh(const PosTexNorIndColInterface& model, bool copyData);

    void Draw();
    void Draw(GLenum mode);   // lines initialized by PosInterface
    // Draw the skirt index tail (see FromData); drawn after Draw() so it
    // depth-tests against the terrain in front of it.
    void DrawSkirt();

    unsigned int numInnerIndices() const { return m_numInnerIndices; }

    // for bullet physics
    double *vs = NULL;
    unsigned int num_vertices = 0;
    int *is = NULL;
    unsigned int num_indices = 0;

private:

    // Defaults keep an empty (import-failed) Mesh destructible.
    int num_VABs = 0;
    GLuint *m_vertexArrayBuffers = NULL;
    GLuint m_vertexArrayObject = 0;
    unsigned int m_numIndices = 0;
    unsigned int m_numInnerIndices = 0;
};

/* Shared file-asset registry: lookup-or-load, ONE Mesh per file. The registry
   owns the mesh (lives until process exit). vs/is copies are ALWAYS kept
   (BuildPartHull needs them); a failed import yields a unit-cube placeholder. */
Mesh *get_mesh(const std::string &path);

#endif
