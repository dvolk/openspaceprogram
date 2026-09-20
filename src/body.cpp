#include "body.h"

void DrawModelAt(const Camera *camera, Mesh *mesh, Shader *shader, Texture *texture,
                 const glm::dmat4 &modelMat, glm::vec3 &sunlightVec, float shadow,
                 const glm::dmat4 &xform, const DrawOpts &opts)
{
    glm::dmat4 View = camera->GetView();
    // The view is built in the render frame (origin = renderOrigin),
    // so shift the geometry into that frame before the float32 cast.
    const glm::dmat4 xf = glm::translate(-camera->GetRenderOrigin()) * xform;
    // make sure View * Model happens with double precision
    glm::dmat4 ModelView = View * xf * modelMat;
    glm::mat4 ModelViewFloat = ModelView;
    glm::mat4 Projection = camera->GetProjection();
    glm::mat4 MVP = Projection * ModelViewFloat;
    glm::mat4 ModelFloat = xf * modelMat;

    shader->Bind();
    shader->setUniform_mat4(0, MVP);
    shader->setUniform_mat4(1, ModelFloat);
    shader->setUniform_vec3(2, sunlightVec);
    shader->setUniform_vec1(3, shadow);
    shader->setUniform_vec1(4, opts.alpha);
    shader->setUniform_vec3(5, opts.tint);
    shader->setUniform_vec1(6, opts.flat);

    /* Translucent (ghost) pass: blend over what is behind and don't write
       depth, so a ghost previews without occluding the ship under it.
       Restore the opaque state afterwards. */
    const bool blend = opts.alpha < 1.0f;
    if(blend) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
    }

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture->id);

    mesh->Draw();

    glBindTexture(GL_TEXTURE_2D, 0);

    if(blend) {
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
}

void setRigidBody(Body *b, btRigidBody *rb) { b->btBody = rb; }
btRigidBody* getRigidBody(Body *b) { return b->btBody; }

Body *create_body(Mesh *mesh, Shader *shader, Texture *texture,
                  float x, float y, float z, float mass)
{
    Body *body = new Body;
    body->mesh = mesh;
    body->shader = shader;
    body->texture = texture;
    body->mass = mass;
    glm::vec3 pos = glm::vec3(x, y, z);
    RegisterPhysicsBody(body, pos, glm::vec3(0, 0, 0));
    return body;
}

Body *create_part_body(Mesh *mesh, Shader *shader, Texture *texture,
                       float mass, double hull_margin)
{
    Body *body = new Body;
    body->mesh = mesh;
    body->shader = shader;
    body->texture = texture;
    body->mass = mass;
    body->hull_margin = hull_margin;
    BuildPartHull(body);
    /* The aerodynamic faces, from the same mesh the hull is built from (the
       part-local frame). Computed here -- the one place every part-creation
       path goes through -- so flight, the VAB, saves and the dock/radial
       tests all get them without each having to remember. A failed import
       leaves the mesh empty and this yields an empty list (no drag). */
    body->aeroFaces = extractAeroFaces(mesh->vs, mesh->num_vertices,
                                       mesh->is, mesh->num_indices);
    return body;
}

void print_mat(glm::dmat4 m) {
    printf("- %f %f %f %f\n  %f %f %f %f\n  %f %f %f %f\n  %f %f %f %f\n",
           m[0][0], m[0][1], m[0][2], m[0][3],
           m[1][0], m[1][1], m[1][2], m[1][3],
           m[2][0], m[2][1], m[2][2], m[2][3],
           m[3][0], m[3][1], m[3][2], m[3][3]);
}
