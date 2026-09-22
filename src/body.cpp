#include "body.h"

#include <cstdio>    // the PRECDBG precision instrument below
#include <cstdlib>

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
    // Precision instrument (PRECDBG=1): per-draw ModelView translation,
    // double vs the float32 the GPU gets. Wobble in dbl == sim/frame
    // quantization; wobble only in flt == the cast. Capped at 4000 lines.
    {
        static const bool on = getenv("PRECDBG") != nullptr;
        static int s_n = 0;
        if(on && s_n < 4000) {
            s_n++;
            const glm::dvec3 t = ModelView[3];
            const glm::vec3 tf = ModelViewFloat[3];
            printf("PRECDBG-MV n=%d mesh=%p dbl=(%.17g,%.17g,%.17g) flt=(%.9g,%.9g,%.9g) err=(%.3e,%.3e,%.3e)\n",
                   s_n, (const void*)mesh, t.x, t.y, t.z, tf.x, tf.y, tf.z,
                   t.x - tf.x, t.y - tf.y, t.z - tf.z);
        }
    }
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
    /* The collision hull's vertices (part-local frame) for the projected-area
       drag (drag.h projectedArea). Read from body->shape -- the SAME
       btConvexHullShape the collision uses -- so the drag silhouette matches
       the collision shape by construction, even for a non-convex mesh (the
       engine's hollow nozzle). Computed here -- the one place every
       part-creation path goes through -- so flight, the VAB, saves and the
       dock/radial tests all get it. A failed import leaves the hull with < 3
       vertices and projectedArea reads 0 (no drag). */
    if(const btConvexHullShape *hull =
           static_cast<const btConvexHullShape *>(body->shape)) {
        const int n = hull->getNumVertices();
        body->hullVerts.reserve(n);
        for(int i = 0; i < n; i++) {
            btVector3 v;
            hull->getVertex(i, v);
            body->hullVerts.push_back(
                glm::dvec3(v.getX(), v.getY(), v.getZ()));
        }
    }
    return body;
}

void print_mat(glm::dmat4 m) {
    printf("- %f %f %f %f\n  %f %f %f %f\n  %f %f %f %f\n  %f %f %f %f\n",
           m[0][0], m[0][1], m[0][2], m[0][3],
           m[1][0], m[1][1], m[1][2], m[1][3],
           m[2][0], m[2][1], m[2][2], m[2][3],
           m[3][0], m[3][1], m[3][2], m[3][3]);
}
