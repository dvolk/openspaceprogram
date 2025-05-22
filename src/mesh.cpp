#include "mesh.h"

// #include <map>
// #include <algorithm>
#include <fstream>
#include <iostream>
#include <stdlib.h>

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include "gldebug.h"

void Mesh::AssImpFromFile(const std::string& pFile, bool copyData)
{
    Assimp::Importer importer;

    const aiScene* scene = importer.ReadFile( pFile,
                                              aiProcess_CalcTangentSpace       |
                                              aiProcess_Triangulate            |
                                              aiProcess_JoinIdenticalVertices  |
                                              aiProcess_SortByPType);

    if(!scene) {
        printf("Error: %s\n", importer.GetErrorString());
        return;
    }

    printf("scene meshes: %d\n", scene->HasMeshes());

    aiMesh *aim = scene->mMeshes[0];

    PosTexNorIndInterface model;
    static const glm::vec3 pink = glm::vec3(1.0, 192.0/255.0, 203.0/255.0);

    for(int i = 0; i < aim->mNumVertices; i++) {
        model.positions.push_back(glm::vec3(aim->mVertices[i].x, aim->mVertices[i].y, aim->mVertices[i].z));
        model.normals.push_back(glm::vec3(aim->mNormals[i].x, aim->mNormals[i].y, aim->mNormals[i].z));

        if(aim->GetNumUVChannels() > 0) {
            glm::vec2 uv = glm::vec2(aim->mTextureCoords[0][i].x,
                                     aim->mTextureCoords[0][i].y);
            model.texcoords.push_back(uv);
        }
        else {
            model.texcoords.push_back(glm::vec2(0,0));
        }
    }

    for(int i = 0; i < aim->mNumFaces; i++) {
        for(int j = 0; j < aim->mFaces[i].mNumIndices; j++) {
            model.indices.push_back(aim->mFaces[i].mIndices[j]);
        }
    }

    InitMesh(model, copyData);
}

void Mesh::FromFile(const std::string& fileName, bool copyData)
{
    AssImpFromFile(fileName, copyData);
    printf("Loaded file: %s\n", fileName.c_str());
}

void Mesh::InitMesh(const PosInterface & model) {
    check_gl_error();
    glGenVertexArrays(1, &m_vertexArrayObject);
    check_gl_error();
    glBindVertexArray(m_vertexArrayObject);
    check_gl_error();
    num_VABs = 1;
    check_gl_error();
    num_vertices = model.positions.size();
    check_gl_error();
    m_vertexArrayBuffers = (GLuint*)malloc(sizeof(GLuint) * num_VABs);
    check_gl_error();
    glGenBuffers(num_VABs, m_vertexArrayBuffers);
    check_gl_error();
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[0]);
    check_gl_error();
    glBufferData(GL_ARRAY_BUFFER, sizeof(model.positions[0]) * model.positions.size(), &model.positions[0], GL_STATIC_DRAW);
    check_gl_error();
    glEnableVertexAttribArray(0);
    check_gl_error();
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    check_gl_error();
    glBindVertexArray(0);
    check_gl_error();
}

void Mesh::InitMesh(const PosNorIndColInterface& model, bool copyData)
{
    // printf("InitMesh(): copyData: %d\n", copyData);
    // printf("InitMesh(): model.positions.size(): %d\n", model.positions.size());
    // printf("InitMesh(): model.indices.size(): %d\n", model.indices.size());

    if(copyData == true) {
        num_vertices = model.positions.size();
        vs = new double[num_vertices * 3];

        int j = 0;
        for(unsigned int i = 0; i < num_vertices; i++) {
            vs[j + 0] = model.positions[i].x;
            vs[j + 1] = model.positions[i].y;
            vs[j + 2] = model.positions[i].z;
            j += 3;
        }

        num_indices = model.indices.size();
        is = new int[num_indices];
        memcpy(is, &model.indices[0], sizeof(int) * num_indices);
    }
    else {
        vs = NULL;
        is = NULL;
    }

    m_numIndices = model.indices.size();

    glGenVertexArrays(1, &m_vertexArrayObject);
    glBindVertexArray(m_vertexArrayObject);

    num_VABs = 4;
    m_vertexArrayBuffers = (GLuint*)malloc(sizeof(GLuint) * num_VABs);

    glGenBuffers(num_VABs, m_vertexArrayBuffers);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vertexArrayBuffers[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(model.indices[0]) * model.indices.size(), &model.indices[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model.positions[0]) * model.positions.size(), &model.positions[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[2]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model.normals[0]) * model.normals.size(), &model.normals[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[3]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model.colors[0]) * model.colors.size(), &model.colors[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindVertexArray(0);
}

void Mesh::InitMesh(const PosTexNorIndColInterface& model, bool copyData)
{
    // printf("InitMesh(): copyData: %d\n", copyData);
    // printf("InitMesh(): model.positions.size(): %d\n", model.positions.size());
    // printf("InitMesh(): model.indices.size(): %d\n", model.indices.size());

    if(copyData == true) {
        num_vertices = model.positions.size();
        vs = new double[num_vertices * 3];

        int j = 0;
        for(unsigned int i = 0; i < num_vertices; i++) {
            vs[j + 0] = model.positions[i].x;
            vs[j + 1] = model.positions[i].y;
            vs[j + 2] = model.positions[i].z;
            j += 3;
        }

        num_indices = model.indices.size();
        is = new int[num_indices];
        memcpy(is, &model.indices[0], sizeof(int) * num_indices);
    }
    else {
        vs = NULL;
        is = NULL;
    }

    m_numIndices = model.indices.size();

    glGenVertexArrays(1, &m_vertexArrayObject);
    glBindVertexArray(m_vertexArrayObject);

    num_VABs = 5;
    m_vertexArrayBuffers = (GLuint*)malloc(sizeof(GLuint) * num_VABs);

    glGenBuffers(num_VABs, m_vertexArrayBuffers);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vertexArrayBuffers[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(model.indices[0]) * model.indices.size(), &model.indices[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model.positions[0]) * model.positions.size(), &model.positions[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[2]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model.texcoords[0]) * model.texcoords.size(), &model.texcoords[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[3]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model.normals[0]) * model.normals.size(), &model.normals[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[4]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model.colors[0]) * model.colors.size(), &model.colors[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindVertexArray(0);
}

void Mesh::InitMesh(const PosTexNorIndInterface& model, bool copyData)
{
    // printf("InitMesh(): copyData: %d\n", copyData);
    // printf("InitMesh(): model.positions.size(): %d\n", model.positions.size());
    // printf("InitMesh(): model.indices.size(): %d\n", model.indices.size());

    if(copyData == true) {
        num_vertices = model.positions.size();
        vs = new double[num_vertices * 3];

        int j = 0;
        for(unsigned int i = 0; i < num_vertices; i++) {
            vs[j + 0] = model.positions[i].x;
            vs[j + 1] = model.positions[i].y;
            vs[j + 2] = model.positions[i].z;
            j += 3;
        }

        num_indices = model.indices.size();
        is = new int[num_indices];
        memcpy(is, &model.indices[0], sizeof(int) * num_indices);
    }
    else {
        vs = NULL;
        is = NULL;
    }

    m_numIndices = model.indices.size();

    glGenVertexArrays(1, &m_vertexArrayObject);
    glBindVertexArray(m_vertexArrayObject);

    num_VABs = 4;
    m_vertexArrayBuffers = (GLuint*)malloc(sizeof(GLuint) * num_VABs);

    glGenBuffers(num_VABs, m_vertexArrayBuffers);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vertexArrayBuffers[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(model.indices[0]) * model.indices.size(), &model.indices[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model.positions[0]) * model.positions.size(), &model.positions[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[2]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model.texcoords[0]) * model.texcoords.size(), &model.texcoords[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[3]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model.normals[0]) * model.normals.size(), &model.normals[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindVertexArray(0);
}

void Mesh::FromData(PosNorColVertex* vertices, unsigned int numVertices, unsigned int* indices, unsigned int numIndices, bool copyData)
{
    PosNorIndColInterface model;

    for(unsigned int i = 0; i < numVertices; i++) {
        model.positions.push_back(vertices[i].pos);
        model.normals.push_back(vertices[i].normal);
        model.colors.push_back(vertices[i].color);
    }

    assert(model.positions.size() == model.colors.size());

    for(unsigned int i = 0; i < numIndices; i++) {
        model.indices.push_back(indices[i]);
    }

    InitMesh(model, copyData);
}

Mesh::~Mesh()
{
    glDeleteBuffers(num_VABs, m_vertexArrayBuffers);
    glDeleteVertexArrays(1, &m_vertexArrayObject);

    delete[] vs;
    delete[] is;

    free(m_vertexArrayBuffers);
}

void Mesh::Draw()
{
    glBindVertexArray(m_vertexArrayObject);

    //glDrawElements(GL_TRIANGLES, m_numIndices, GL_UNSIGNED_INT, 0);

    glDrawElementsBaseVertex(GL_TRIANGLES, m_numIndices, GL_UNSIGNED_INT, 0, 0);

    glBindVertexArray(0);
}

void Mesh::Draw(GLenum mode) {
    glBindVertexArray(m_vertexArrayObject);

    glDrawArrays(mode, 0, num_vertices);

    glBindVertexArray(0);
}
