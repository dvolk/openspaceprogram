// vab.cpp -- the VAB editor's interaction layer (see vab.h): physics-free
// picking of the build tree, the ghost preview pose, and placing a part.
//
// Picking reuses Bullet's own convex cast (pick.cpp castRay) against a
// per-part-type convex hull built from the mesh -- the build tree has hull
// shapes but NO rigid bodies, which is what keeps the authoring preview
// cheap. pickRay's unprojection already cancels the draw-side renderOrigin
// shift (see pick.cpp), so the ray and every hit live directly in the build
// frame S: hulls go at the parts' localPos as-is, no -vab_center shift.
#include "vab.h"

#include <map>
#include <string>

#include "mesh.h"     // get_mesh (the part mesh's vertex array)
#include "ships.h"    // Ships::catalog (resolve the armed palette name)

namespace {

struct VabAsset {
    btConvexHullShape *hull = nullptr;
    btCollisionObject *obj = nullptr;
};
std::map<const PartDef *, VabAsset> g_vabAssets;

VabAsset &vabAsset(const PartDef *def) {
    auto it = g_vabAssets.find(def);
    if(it != g_vabAssets.end()) { return it->second; }
    VabAsset a;
    Mesh *m = get_mesh(std::string("./res/") + def->mesh);
    if(m != nullptr && m->vs != nullptr && m->num_vertices >= 3) {
        a.hull = new btConvexHullShape(m->vs, (int)m->num_vertices,
                                       3 * sizeof(double));
        a.hull->setMargin(0.1);
        a.obj = new btCollisionObject();
        a.obj->setCollisionShape(a.hull);
    }
    return g_vabAssets.emplace(def, a).first->second;
}

btTransform toBt(const glm::dmat4 &m) {
    btTransform t;
    t.setFromOpenGLMatrix(&m[0][0]);   // column-major, btScalar == double
    return t;
}

// Build-frame (S) point -> window pixel, the inverse of pickRay's
// unprojection. false if behind the camera.
bool project(const Game &g, const glm::dvec3 &pS, double &px, double &py) {
    const Camera &cam = *g.camera;
    // The view's camera sits at pos - renderOrigin and DrawModelAt shifts
    // geometry by -renderOrigin; the two cancel, so S-frame points map
    // straight as v = R * (pS - pos) (the same contract pickRay inverts).
    const glm::dmat3 R(cam.view);
    const glm::dvec3 v = R * (pS - cam.pos);
    if(v.z >= -1e-6) { return false; }
    const double fx = cam.projection[0][0];
    const double fy = cam.projection[1][1];
    const double nx = fx * v.x / (-v.z);
    const double ny = fy * v.y / (-v.z);
    px = (nx + 1.0) * 0.5 * (double)cam.viewport_w;
    py = (1.0 - ny) * 0.5 * (double)cam.viewport_h;
    return true;
}

// The child stack node that best mates a parent node: the non-surface child
// node whose direction is most anti-parallel to the parent node's (so a
// parent "top" grabs the child "bottom", etc.).
const Node *bestMatingChildNode(const PartDef &child, const glm::dvec3 &parentDir) {
    const Node *best = nullptr;
    double bestDot = -2.0;
    for(size_t i = 0; i < child.nodes.size(); i++) {
        const Node &cn = child.nodes[i];
        if(cn.surface) { continue; }
        const double d = glm::dot(glm::normalize(cn.dir), -glm::normalize(parentDir));
        if(d > bestDot) { bestDot = d; best = &cn; }
    }
    return best;
}

} // namespace

btCollisionShape *vabPartHull(const PartDef *def) { return vabAsset(def).hull; }
btCollisionObject *vabPartObject(const PartDef *def) { return vabAsset(def).obj; }

bool pickVabPart(Game &g, int px, int py, int &partIdx, PickBodyHit &hit) {
    if(g.camera == nullptr) { return false; }
    const PickRay ray = pickRay(*g.camera, g.camera->viewport_w,
                                g.camera->viewport_h, px, py);
    double best = 1e300;
    int bestI = -1;
    PickBodyHit bestH;
    for(size_t i = 0; i < g.vab.parts.size(); i++) {
        const BuildPart &bp = g.vab.parts[i];
        if(bp.def == nullptr) { continue; }
        VabAsset &a = vabAsset(bp.def);
        if(a.hull == nullptr) { continue; }
        const glm::dmat4 model = glm::translate(bp.localPos)
                               * glm::dmat4(bp.localRot);
        PickBodyHit h;
        if(!castRay(ray, a.obj, a.hull, toBt(model), h)) { continue; }
        if(h.dist < best) { best = h.dist; bestI = (int)i; bestH = h; }
    }
    if(bestI < 0) { return false; }
    partIdx = bestI;
    hit = bestH;
    return true;
}

glm::dvec3 vabNodePos(const Game &g, int partIdx, int nodeIdx) {
    const BuildPart &bp = g.vab.parts[(size_t)partIdx];
    const Node &n = bp.def->nodes[(size_t)nodeIdx];
    return bp.localPos + bp.localRot * n.pos;
}

int pickVabNode(Game &g, int px, int py, int partIdx, double thresholdPx) {
    if(partIdx < 0 || (size_t)partIdx >= g.vab.parts.size()) { return -1; }
    const BuildPart &bp = g.vab.parts[(size_t)partIdx];
    if(bp.def == nullptr) { return -1; }
    double best = thresholdPx;
    int bestI = -1;
    for(size_t i = 0; i < bp.def->nodes.size(); i++) {
        if(bp.def->nodes[i].surface) { continue; }   // stack ports only
        if(g.vab.nodeOccupied(partIdx, bp.def->nodes[i].id)) { continue; }
        double sx = 0, sy = 0;
        if(!project(g, vabNodePos(g, partIdx, (int)i), sx, sy)) { continue; }
        const double d = glm::length(glm::dvec2(sx - (double)px, sy - (double)py));
        if(d < best) { best = d; bestI = (int)i; }
    }
    return bestI;
}

void vabUpdateHover(Game &g, int px, int py) {
    g.vab_hover = -1;
    g.vab_hoverNode = -1;
    g.vab_hoverParent = -1;
    g.vab_ghostValid = false;

    int pi = -1;
    PickBodyHit hit;
    if(!pickVabPart(g, px, py, pi, hit)) { return; }
    g.vab_hover = pi;
    if(g.vab_armed.empty()) { return; }   // inspecting only; no ghost

    const PartDef *childDef = g.ships.catalog().find(g.vab_armed);
    if(childDef == nullptr) { return; }
    const BuildPart &pp = g.vab.parts[(size_t)pi];
    if(pp.def == nullptr) { return; }

    const int node = pickVabNode(g, px, py, pi, 24.0);
    if(node >= 0) {
        // stack attach onto the hovered port
        const Node &pn = pp.def->nodes[(size_t)node];
        const Node *cn = bestMatingChildNode(*childDef, pn.dir);
        if(cn == nullptr) { return; }
        const AttachPose ap = attachNodes(pp.localPos, pp.localRot, pn, *cn, 0.0, 0.0);
        g.vab_hoverParent = pi;
        g.vab_hoverNode = node;
        g.vab_ghostSurface = false;
        g.vab_ghostParentNode = pn.id;
        g.vab_ghostChildNode = cn->id;
        g.vab_ghostPos = ap.childPos;
        g.vab_ghostRot = ap.childRot;
        g.vab_ghostValid = true;
        return;
    }

    // no port nearby: surface-attach at the ray hit on the hovered parent
    const Node *cs = childDef->findSurfaceNode();
    if(cs == nullptr) { return; }
    const glm::dvec3 pS = hit.point;   // already S frame (pickVabPart)
    const glm::dmat3 invR = glm::transpose(pp.localRot);
    const glm::dvec3 localPoint = invR * (pS - pp.localPos);
    const glm::dvec3 localNormal = glm::normalize(invR * hit.normal);
    const AttachPose ap = attachSurface(pp.localPos, pp.localRot, localPoint,
                                        localNormal, *cs, 0.0, 0.0);
    g.vab_hoverParent = pi;
    g.vab_hoverNode = -1;
    g.vab_ghostSurface = true;
    g.vab_ghostPoint = localPoint;
    g.vab_ghostNormal = localNormal;
    g.vab_ghostChildNode = cs->id;
    g.vab_ghostPos = ap.childPos;
    g.vab_ghostRot = ap.childRot;
    g.vab_ghostValid = true;
}

int vabPlace(Game &g) {
    if(!g.vab_ghostValid || g.vab_hoverParent < 0) { return -1; }
    const PartDef *childDef = g.ships.catalog().find(g.vab_armed);
    if(childDef == nullptr) { return -1; }

    BuildPart np;
    np.def = childDef;
    np.id = g.vab_armed + "_" + std::to_string(g.vab.parts.size() + 1);
    np.parent = g.vab_hoverParent;
    if(g.vab_ghostSurface) {
        np.attach = AttachMode::Surface;
        np.contactPoint = g.vab_ghostPoint;
        np.contactNormal = g.vab_ghostNormal;
        np.childNode = g.vab_ghostChildNode;
    } else {
        np.attach = AttachMode::Down;   // a stack edge; the node ids carry the mating
        np.parentNode = g.vab_ghostParentNode;
        np.childNode = g.vab_ghostChildNode;
    }
    g.vab.parts.push_back(np);
    g.vab.recomputePoses();
    return (int)g.vab.parts.size() - 1;
}
