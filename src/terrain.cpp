// terrain.cpp -- GeoPatch method implementations (see terrain.h for the
// class/data declarations).
#include "terrain.h"

#include <cstdio>

// GeoPatch holds a btRigidBody* and `delete`s it in ~GeoPatch, so the
// complete bullet type is needed here. Defined double-precision to match
// body.h / physics.cpp (same ABI as main.cpp). bullet must come BEFORE
// physics.h, which names btDefaultCollisionConfiguration in a member
// (only forward-declared there) and so needs the complete type.
#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>
#include "physics.h"


GeoPatch::~GeoPatch() {
    delete kids[0];
    delete kids[1];
    delete kids[2];
    delete kids[3];
    if(collision != NULL) {
        printf("removing terrain collision\n");
        removeTerrainCollision(collision);
        delete collision;
    }
    delete model;
}

void GeoPatch::Subdivide(void) {
    // TOOO need debug levels?
    // printf("%p subdiving (%d)!\n", this, depth);
    const glm::vec3 v01 = glm::normalize(v0+v1);
    const glm::vec3 v12 = glm::normalize(v1+v2);
    const glm::vec3 v23 = glm::normalize(v2+v3);
    const glm::vec3 v30 = glm::normalize(v3+v0);
    const glm::vec3 cn  = glm::normalize(centroid);

    const glm::vec3 vecs[4][4] = {
        {v0,  v01,   cn,  v30},
        {v01,  v1,  v12,   cn},
        {cn,  v12,   v2,  v23},
        {v30,  cn,  v23,   v3}
    };

    for (int quadrant = 0; quadrant < 4; quadrant++) {
        kids[quadrant]
            = new GeoPatch(body,
                           model->shader,
                           depth + 1,
                           vecs[quadrant][0],
                           vecs[quadrant][1],
                           vecs[quadrant][2],
                           vecs[quadrant][3]);
    }
}

GeoPatch::GeoPatch(TerrainBody *body, Shader *shader, int depth, glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 v3) {
    model = new Model;
    kids[0] = NULL;
    kids[1] = NULL;
    kids[2] = NULL;
    kids[3] = NULL;
    this->body = body;
    this->depth = depth;
    this->v0 = v0;
    this->v1 = v1;
    this->v2 = v2;
    this->v3 = v3;
    this->centroid = glm::normalize(v0 + v1 + v2 + v3);
    // Leaf patches (subdivision stops at depth == max_depth) get the
    // collision mesh; the old `>` was never true since depth never
    // exceeds max_depth, so terrain collision was silently never added.
    bool has_collision = depth >= max_depth;
    Mesh *grid_mesh = body->create_grid_mesh(has_collision, depth > 1, v0, v1, v2, v3);
    model->FromData(grid_mesh, shader, NULL);
    if(has_collision == true) {
        collision = addTerrainCollision(grid_mesh);
        printf("added terrain collision with %p\n", (void*)this);
    } else {
        collision = NULL;
    }
}

void GeoPatch::Draw(const Camera* camera, const glm::dmat4& transform, const glm::vec3& sunlightVec, bool skirt_pass) {
    if(kids[0] == NULL) {
        // patch isn't subdivided
        glm::vec4 color = glm::vec4(0.8, 0.8, 0.8, 1.0);
        model->shader->Bind();

        const glm::dmat4 & View = camera->GetView();
        // make sure View * Model happens with double precision
        // (transform shifted into the render frame: the view is built there)
        glm::dmat4 ModelView = View * glm::translate(-camera->GetRenderOrigin()) * transform;
        glm::mat4 ModelViewFloat = ModelView;
        const glm::mat4 & Projection = camera->GetProjection();
        glm::mat4 MVP = Projection * ModelViewFloat;
        glm::mat4 ModelFloat = transform;

        model->shader->setUniform_mat4(0, MVP);
        model->shader->setUniform_mat4(1, ModelFloat);
        model->shader->setUniform_vec3(2, sunlightVec);
        model->shader->setUniform_vec4(3, color);

        if(skirt_pass == false) {
            model->mesh->Draw();
        } else {
            // the stencil (set up in TerrainBody::Draw) only passes where
            // no terrain fragment was drawn, so the skirt shows in the
            // cracks/limb and can never z-fight the surface
            model->mesh->DrawSkirt();
        }
    }
    else {
        kids[0]->Draw(camera, transform, sunlightVec, skirt_pass);
        kids[1]->Draw(camera, transform, sunlightVec, skirt_pass);
        kids[2]->Draw(camera, transform, sunlightVec, skirt_pass);
        kids[3]->Draw(camera, transform, sunlightVec, skirt_pass);
    }
}

void GeoPatch::Update(const Camera* camera, const glm::dmat4& transform) {
    const glm::dvec3& camera_pos = camera->GetPos() - (glm::dvec3)(transform[3]);
    const glm::dvec3& centroid_pos = body->GetTerrainHeight(glm::normalize(camera_pos)) * centroid;
    const float dist = glm::length(camera_pos - centroid_pos);
    const float subdiv = 2.0f * body->radius * glm::length(v0 - centroid);

    if(depth < max_depth and dist < subdiv) {
        if(kids[0] == NULL) {
            Subdivide();
        }
    }
    else if(dist > subdiv * 2) {
        delete kids[0];
        delete kids[1];
        delete kids[2];
        delete kids[3];
        kids[0] = NULL;
        kids[1] = NULL;
        kids[2] = NULL;
        kids[3] = NULL;
    }

    if(kids[0] != NULL) {
        kids[0]->Update(camera, transform);
        kids[1]->Update(camera, transform);
        kids[2]->Update(camera, transform);
        kids[3]->Update(camera, transform);
    }
}
