#include "body.h"

void setRigidBody(Body *b, btRigidBody *rb) { b->btBody = rb; }
btRigidBody* getRigidBody(Body *b) { return b->btBody; }

Body *create_body(Model *model, float x, float y, float z, float mass, bool debug_mesh)
{
    Body *body = new Body;
    body->model = model;
    body->mass = mass;
    glm::vec3 pos = glm::vec3(x, y, z);
    RegisterPhysicsBody(body, pos, glm::vec3(0, 0, 0), debug_mesh);
    return body;
}

void print_mat(glm::dmat4 m) {
    printf("- %f %f %f %f\n  %f %f %f %f\n  %f %f %f %f\n  %f %f %f %f\n",
           m[0][0], m[0][1], m[0][2], m[0][3],
           m[1][0], m[1][1], m[1][2], m[1][3],
           m[2][0], m[2][1], m[2][2], m[2][3],
           m[3][0], m[3][1], m[3][2], m[3][3]);
}
