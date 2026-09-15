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

#include <cmath>
#include <cstdio>
#include <map>
#include <string>

#include <glm/gtc/quaternion.hpp>   // angleAxis / mat3_cast (the snap + symmetry)

#include "mesh.h"     // get_mesh (the part mesh's vertex array)
#include "ships.h"    // Ships::catalog (resolve the armed palette name)

namespace {

/* The placement snap grids: 10 cm along the parent's axis / 10 deg around
   it (and for the part roll), per the editor's alignment convention. */
const double kSnapLen = 0.1;    // m
const double kSnapAng = 10.0;   // deg

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

bool altHeld() {
    const bool *ks = SDL_GetKeyboardState(nullptr);   // SDL3: bool per scancode
    return ks[SDL_SCANCODE_LALT] || ks[SDL_SCANCODE_RALT];
}

double snapAngleDeg(double deg) {
    return std::round(deg / kSnapAng) * kSnapAng;
}

/* The next 10-deg grid point in the direction of `delta` -- grid-aligned
   even when the current value is off-grid (fine-tuned with snap off). */
double gridStepDeg(double cur, double delta) {
    if(delta > 0.0) { return std::floor(cur / kSnapAng + 1e-9) * kSnapAng + kSnapAng; }
    return std::ceil(cur / kSnapAng - 1e-9) * kSnapAng - kSnapAng;
}

/* Snap a surface contact in the parent's local frame: the clock angle about
   the parent's long Z to the 10 deg grid and the height to the 10 cm grid
   (the radius is untouched, so the contact stays on a cylindrical side);
   the normal rides the clock rotation to stay perpendicular. Near the axis
   (r ~ 0, a cap hit) only the height snaps. */
void snapSurfaceContact(glm::dvec3 &point, glm::dvec3 &normal) {
    const double r = std::hypot(point.x, point.y);
    point.z = std::round(point.z / kSnapLen) * kSnapLen;
    if(r < 1e-4) { return; }
    const double a = std::atan2(point.y, point.x);
    const double a2 = glm::radians(snapAngleDeg(glm::degrees(a)));
    const glm::dmat3 Rz = glm::mat3_cast(
        glm::angleAxis(a2 - a, glm::dvec3(0.0, 0.0, 1.0)));
    point = Rz * point;
    normal = Rz * normal;
}

/* A placement id "<base>_<n>" not yet used in the tree, scanning n upward
   (deletes leave holes the size-based guess would collide with). */
std::string nextBuildId(const BuildShip &bs, const std::string &base, int &n) {
    for(;;) {
        const std::string id = base + "_" + std::to_string(n);
        bool dup = false;
        for(size_t i = 0; i < bs.parts.size(); i++) {
            if(bs.parts[i].id == id) { dup = true; break; }
        }
        if(!dup) { return id; }
        n++;
    }
}

} // namespace

// Build-frame (S) point -> window pixel, the inverse of pickRay's
// unprojection. false if behind the camera.
bool vabProject(const Game &g, const glm::dvec3 &pS, double &px, double &py) {
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
        if(!vabProject(g, vabNodePos(g, partIdx, (int)i), sx, sy)) { continue; }
        const double d = glm::length(glm::dvec2(sx - (double)px, sy - (double)py));
        if(d < best) { best = d; bestI = (int)i; }
    }
    return bestI;
}

void vabClearHover(Game &g) {
    g.vab_hover = -1;
    g.vab_hoverNode = -1;
    g.vab_hoverParent = -1;
    g.vab_ghostValid = false;
    g.vab_ghostClones.clear();
}

void vabUpdateHover(Game &g, int px, int py) {
    vabClearHover(g);

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
    const bool snap = g.vab_snap != altHeld();   // Alt bypasses while held
    if(node >= 0) {
        // stack attach onto the hovered port (symmetry does not apply: the
        // synthesized axial ports are singletons, clones would coincide)
        const Node &pn = pp.def->nodes[(size_t)node];
        const Node *cn = bestMatingChildNode(*childDef, pn.dir);
        if(cn == nullptr) { return; }
        const double roll = snap ? snapAngleDeg(g.vab_ghostRoll) : g.vab_ghostRoll;
        g.vab_ghostRollUsed = roll;
        const AttachPose ap = attachNodes(pp.localPos, pp.localRot, pn, *cn,
                                          roll, 0.0);
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
    glm::dvec3 localPoint = invR * (pS - pp.localPos);
    glm::dvec3 localNormal = glm::normalize(invR * hit.normal);
    double roll = g.vab_ghostRoll;
    if(snap) {
        snapSurfaceContact(localPoint, localNormal);
        roll = snapAngleDeg(roll);
    }
    g.vab_ghostRollUsed = roll;
    const AttachPose ap = attachSurface(pp.localPos, pp.localRot, localPoint,
                                        localNormal, *cs, roll, 0.0);
    g.vab_hoverParent = pi;
    g.vab_hoverNode = -1;
    g.vab_ghostSurface = true;
    g.vab_ghostPoint = localPoint;
    g.vab_ghostNormal = localNormal;
    g.vab_ghostChildNode = cs->id;
    g.vab_ghostPos = ap.childPos;
    g.vab_ghostRot = ap.childRot;
    g.vab_ghostValid = true;
    // radial symmetry: the clones ring the hovered parent's own axis
    g.vab_ghostClones = radialSymmetryClones(pp.localPos, pp.localRot, *cs,
                                             localPoint, localNormal, roll,
                                             0.0, g.vab_symmetry);
}

int vabPlace(Game &g) {
    if(!g.vab_ghostValid || g.vab_hoverParent < 0) { return -1; }
    const PartDef *childDef = g.ships.catalog().find(g.vab_armed);
    if(childDef == nullptr) { return -1; }

    int idn = (int)g.vab.parts.size() + 1;
    BuildPart np;
    np.def = childDef;
    np.id = nextBuildId(g.vab, g.vab_armed, idn);
    np.parent = g.vab_hoverParent;
    if(g.vab_ghostSurface) {
        np.attach = AttachMode::Surface;
        np.contactPoint = g.vab_ghostPoint;
        np.contactNormal = g.vab_ghostNormal;
        np.childNode = g.vab_ghostChildNode;
        np.roll = g.vab_ghostRollUsed;
    } else {
        np.attach = AttachMode::Down;   // a stack edge; the node ids carry the mating
        np.parentNode = g.vab_ghostParentNode;
        np.childNode = g.vab_ghostChildNode;
        np.angle = g.vab_ghostRollUsed;
    }
    g.vab.parts.push_back(np);
    // the radial-symmetry siblings: ordinary independent surface parts on
    // the same parent (each selectable/deletable on its own afterwards)
    for(size_t k = 0; k < g.vab_ghostClones.size(); k++) {
        const SymClone &c = g.vab_ghostClones[k];
        BuildPart sp;
        sp.def = childDef;
        sp.id = nextBuildId(g.vab, g.vab_armed, idn);
        sp.parent = np.parent;
        sp.attach = AttachMode::Surface;
        sp.contactPoint = c.edge.point;
        sp.contactNormal = c.edge.normal;
        sp.childNode = np.childNode;
        sp.roll = c.edge.rollDeg;
        g.vab.parts.push_back(sp);
    }
    g.vab.recomputePoses();
    g.vab_ghostRoll = 0.0;   // the next placement starts unrolled
    g.vab_ghostClones.clear();
    return (int)g.vab.parts.size() - 1;
}

/* Q/E rotate: the ghost's pending roll, or the selected part's attach roll
   (stack angle / surface roll) with its subtree re-solved. With snap on the
   steps land exactly on the 10 deg grid; off, they are free 5 deg steps. */
void vabRotate(Game &g, double deltaDeg) {
    const bool snap = g.vab_snap != altHeld();
    if(g.vab_ghostValid) {
        g.vab_ghostRoll = snap ? gridStepDeg(g.vab_ghostRoll, deltaDeg)
                               : g.vab_ghostRoll + deltaDeg;
        return;
    }
    if(g.vab_selected < 0 || (size_t)g.vab_selected >= g.vab.parts.size()) {
        return;
    }
    BuildPart &bp = g.vab.parts[(size_t)g.vab_selected];
    if(bp.parent < 0) { return; }   // the root has no edge to spin
    if(!snap) { g.vab.rotatePart(g.vab_selected, deltaDeg); return; }
    double &a = (bp.attach == AttachMode::Surface) ? bp.roll : bp.angle;
    a = gridStepDeg(a, deltaDeg);
    g.vab.recomputePoses();
}

void vabDeleteSelected(Game &g) {
    const int sel = g.vab_selected;
    if(sel < 0 || (size_t)sel >= g.vab.parts.size()) { return; }
    if(sel == 0) { g.toast("Cannot delete the root part"); return; }
    const std::string id = g.vab.parts[(size_t)sel].id;
    if(g.vab.removePart(sel)) {
        g.vab_selected = -1;
        vabClearHover(g);
        g.toast("Deleted %s", id.c_str());
    }
}

void vabSave(Game &g, const char *path) {
    if(save_ship_def(g.vab, path)) {
        printf("[vab] saved %s (%d parts)\n", path, (int)g.vab.parts.size());
        fflush(stdout);
        g.toast("Saved %s", path);
    } else {
        g.toast("Save FAILED: %s", path);
    }
}

void vabLaunch(Game &g) {
    if(g.vab.parts.empty()) { g.toast("Nothing to launch"); return; }
    ShipDef def = g.vab.toShipDef();
    const ScenarioDef *sc = scenario_by_name("pad");
    /* defPath "": the ship was built in memory -- there is no file to
       respawn it from until it is saved (the Respawn button hides). */
    Vehicle *v = g.ships.place_ship_def(def, "", def.name, g.home, sc, g.sys);
    g.ships.spawn_crew(v, g.sys);   // crew aboard the capsules, like startup
    g.select_ship(v);
    vabClearHover(g);
    g.vab_armed.clear();
    g.vab_ghostRoll = 0.0;
    g.vab_selected = -1;
    g.scene = Scene::Flight;
    printf("[vab] launched '%s' (%d parts)\n", v->name.c_str(),
           (int)g.vab.parts.size());
    fflush(stdout);
    g.toast("Launched %s", v->name.c_str());
}
