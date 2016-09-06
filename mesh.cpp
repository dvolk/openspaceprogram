#include "mesh.h"
#include "util.h"
#include "debugTimer.h"
#include <map>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdlib.h>

void Mesh::FromFile(const std::string& fileName)
{
  OBJModel m = OBJModel(fileName);
  printf("*** OBJModel vertices: %d\n", m.vertices.size());
  printf("*** OBJModel normals: %d\n", m.normals.size());
  IndexedModel im = m.ToIndexedModel();
  printf("*** IndexedModel vertices: %d\n", im.positions.size());
  printf("*** IndexedModel normals: %d\n", im.normals.size());
  printf("*** IndexedModel colors: %d\n", im.colors.size());
  printf("*** IndexedModel indices: %d\n", im.indices.size());
  InitMesh(im);
  printf("*** Mesh vertices: %d\n", num_vertices);
  printf("*** Mesh normals: %d\n", num_indices);
}

void Mesh::InitMesh(const IndexedModel& model)
{
    m_numIndices = model.indices.size();

    glGenVertexArrays(1, &m_vertexArrayObject);
    glBindVertexArray(m_vertexArrayObject);

    glGenBuffers(NUM_BUFFERS, m_vertexArrayBuffers);
	
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[POSITION_VB]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model.positions[0]) * model.positions.size(), &model.positions[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    // glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[TEXCOORD_VB]);
    // glBufferData(GL_ARRAY_BUFFER, sizeof(model.texCoords[0]) * model.texCoords.size(), &model.texCoords[0], GL_STATIC_DRAW);
    // glEnableVertexAttribArray(1);
    // glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[NORMAL_VB]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model.normals[0]) * model.normals.size(), &model.normals[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vertexArrayBuffers[INDEX_VB]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(model.indices[0]) * model.indices.size(), &model.indices[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[COLOR_VB]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model.colors[0]) * model.colors.size(), &model.colors[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindVertexArray(0);
}

void Mesh::FromData(Vertex* vertices, unsigned int numVertices, unsigned int* indices, unsigned int numIndices)
{
    IndexedModel model;

    for(unsigned int i = 0; i < numVertices; i++)
	{
            model.positions.push_back(*vertices[i].GetPos());
            model.texCoords.push_back(*vertices[i].GetTexCoord());
            model.normals.push_back(*vertices[i].GetNormal());
	    model.colors.push_back(vertices[i].color);
	}

    assert(model.positions.size() == model.colors.size());

    for(unsigned int i = 0; i < numIndices; i++)
        model.indices.push_back(indices[i]);

    vs = new double[numVertices * 3];
    num_vertices = numVertices;
    int j = 0;
    for(unsigned int i = 0; i < numVertices; i++) {
        vs[j + 0] = vertices[i].pos.x;
        vs[j + 1] = vertices[i].pos.y;
        vs[j + 2] = vertices[i].pos.z;
        j+=3;
    }

    is = new int[numIndices];
    num_indices = numIndices;
    memcpy(is, indices, sizeof(unsigned int) * numIndices);

    InitMesh(model);
}

Mesh::~Mesh()
{
    glDeleteBuffers(NUM_BUFFERS, m_vertexArrayBuffers);
    glDeleteVertexArrays(1, &m_vertexArrayObject);
}

void Mesh::Draw()
{
    glBindVertexArray(m_vertexArrayObject);

    //glDrawElements(GL_TRIANGLES, m_numIndices, GL_UNSIGNED_INT, 0);
    glDrawElementsBaseVertex(GL_TRIANGLES, m_numIndices, GL_UNSIGNED_INT, 0, 0);

    glBindVertexArray(0);
}
