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
    g.vab_ghostRoot = false;
    g.vab_ghostAssembly = -1;
    g.vab_ghostClones.clear();
}

/* The definition the ghost places: the armed subassembly's ROOT part, or
   the armed catalog part. *asmOut (when non-null) receives the assembly's
   tree for an assembly ghost, else nullptr. */
static const PartDef *armedChildDef(const Game &g, const BuildShip **asmOut) {
    if(asmOut != nullptr) { *asmOut = nullptr; }
    if(g.vab_armedAsm >= 0
       && (size_t)g.vab_armedAsm < g.vab_subassemblies.size()) {
        const BuildShip &sub = g.vab_subassemblies[(size_t)g.vab_armedAsm].ship;
        if(sub.parts.empty() || sub.parts[0].def == nullptr) { return nullptr; }
        if(asmOut != nullptr) { *asmOut = &sub; }
        return sub.parts[0].def;
    }
    if(g.vab_armed.empty()) { return nullptr; }
    return g.ships.catalog().find(g.vab_armed);
}

void vabUpdateHover(Game &g, int px, int py) {
    vabClearHover(g);

    int pi = -1;
    PickBodyHit hit;
    const bool hitPart = pickVabPart(g, px, py, pi, hit);
    if(hitPart) { g.vab_hover = pi; }
    if(g.vab_linkMode) { return; }   // link picking: no placement ghost
    const BuildShip *asmShip = nullptr;
    const PartDef *childDef = armedChildDef(g, &asmShip);
    if(childDef == nullptr) { return; }  // inspecting only; no ghost
    if(g.vab.parts.empty()) {
        /* An empty build: the armed part/assembly becomes the ROOT,
           anchored at the S origin (the frame's anchor -- KSP's "first
           part" moment). The ghost sits at the origin wherever the
           cursor is; a plain click commits it. */
        g.vab_ghostRoot = true;
        g.vab_ghostAssembly = asmShip ? g.vab_armedAsm : -1;
        g.vab_ghostSurface = false;
        g.vab_ghostPos = glm::dvec3(0.0);
        g.vab_ghostRot = glm::dmat3(1.0);
        g.vab_ghostRollUsed = 0.0;
        g.vab_ghostValid = true;
        return;
    }
    if(!hitPart) { return; }
    const BuildPart &pp = g.vab.parts[(size_t)pi];
    if(pp.def == nullptr) { return; }

    const int node = pickVabNode(g, px, py, pi, 24.0);
    const bool alt = altHeld();                      // Alt bypasses while held
    const bool snapL = g.vab_snapLen != alt;
    const bool snapA = g.vab_snapAng != alt;
    if(node >= 0) {
        // stack attach onto the hovered port (symmetry does not apply: the
        // synthesized axial ports are singletons, clones would coincide)
        const Node &pn = pp.def->nodes[(size_t)node];
        const Node *cn = bestMatingChildNode(*childDef, pn.dir);
        if(cn == nullptr) { return; }
        const double roll = snapA ? snapAngleDeg(g.vab_ghostRoll) : g.vab_ghostRoll;
        g.vab_ghostRollUsed = roll;
        const AttachPose ap = attachNodes(pp.localPos, pp.localRot, pn, *cn,
                                          roll, 0.0);
        g.vab_hoverParent = pi;
        g.vab_hoverNode = node;
        g.vab_ghostSurface = false;
        g.vab_ghostAssembly = asmShip ? g.vab_armedAsm : -1;
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
    if(snapA) { roll = snapAngleDeg(roll); }
    snapSurfaceContact(localPoint, localNormal, snapL, snapA);
    g.vab_ghostRollUsed = roll;
    const AttachPose ap = attachSurface(pp.localPos, pp.localRot, localPoint,
                                        localNormal, *cs, roll, 0.0);
    g.vab_hoverParent = pi;
    g.vab_hoverNode = -1;
    g.vab_ghostSurface = true;
    g.vab_ghostAssembly = asmShip ? g.vab_armedAsm : -1;
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
    if(!g.vab_ghostValid) { return -1; }
    const int parent = g.vab_ghostRoot ? -1 : g.vab_hoverParent;
    // parent -1 is the valid ROOT placement (empty build); only a
    // non-root ghost with no resolved parent is a rejection.
    if(!g.vab_ghostRoot && parent < 0) { return -1; }

    /* An armed SUBASSEMBLY: graft a copy under the resolved root edge (one
       graft per symmetry clone); the list entry is NOT consumed -- placing
       is copy & paste (ids uniquify per graft). */
    if(g.vab_ghostAssembly >= 0
       && (size_t)g.vab_ghostAssembly < g.vab_subassemblies.size()) {
        const BuildShip &sub = g.vab_subassemblies[(size_t)g.vab_ghostAssembly].ship;
        if(sub.parts.empty() || sub.parts[0].def == nullptr) { return -1; }
        BuildPart root;
        root.def = sub.parts[0].def;
        root.id = sub.parts[0].id;   // graftTree uniquifies on collision
        root.parent = parent;
        if(g.vab_ghostSurface) {
            root.attach = AttachMode::Surface;
            root.contactPoint = g.vab_ghostPoint;
            root.contactNormal = g.vab_ghostNormal;
            root.childNode = g.vab_ghostChildNode;
            root.roll = g.vab_ghostRollUsed;
        } else {
            root.attach = AttachMode::Down;   // node ids carry the mating
            root.parentNode = g.vab_ghostParentNode;
            root.childNode = g.vab_ghostChildNode;
            root.angle = g.vab_ghostRollUsed;
        }
        const size_t ri = g.vab.graftTree(sub, root);
        for(size_t k = 0; k < g.vab_ghostClones.size(); k++) {
            const SymClone &c = g.vab_ghostClones[k];
            BuildPart rc = root;
            rc.attach = AttachMode::Surface;   // clones are always surface edges
            rc.parentNode.clear();
            rc.contactPoint = c.edge.point;
            rc.contactNormal = c.edge.normal;
            rc.roll = c.edge.rollDeg;
            g.vab.graftTree(sub, rc);
        }
        g.vab_ghostRoll = 0.0;
        g.vab_ghostClones.clear();
        return (int)ri;
    }

    const PartDef *childDef = g.ships.catalog().find(g.vab_armed);
    if(childDef == nullptr) { return -1; }

    int idn = (int)g.vab.parts.size() + 1;
    BuildPart np;
    np.def = childDef;
    np.id = nextBuildId(g.vab, g.vab_armed, idn);
    np.parent = parent;
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
   (stack angle / surface roll) with its subtree re-solved. With the angle
   snap on the steps land exactly on the 10 deg grid; off, they are free
   5 deg steps. */
void vabRotate(Game &g, double deltaDeg) {
    const bool snap = g.vab_snapAng != altHeld();
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

void vabLinkClick(Game &g) {
    if(g.vab_hover < 0 || (size_t)g.vab_hover >= g.vab.parts.size()) {
        return;   // empty space: stay in link mode, keep waiting
    }
    const BuildPart &bp = g.vab.parts[(size_t)g.vab_hover];
    if(g.vab_linkFromId.empty()) {
        g.vab_linkFromId = bp.id;
        g.toast("Link source: %s -- now click the destination", bp.id.c_str());
        return;
    }
    if(bp.id == g.vab_linkFromId) {
        g.toast("Source and destination must differ");
        return;
    }
    for(size_t i = 0; i < g.vab.fuelLinks.size(); i++) {
        const BuildShip::FuelLink &fl = g.vab.fuelLinks[i];
        if(fl.from == g.vab_linkFromId && fl.to == bp.id) {
            g.toast("%s already feeds %s", fl.from.c_str(), fl.to.c_str());
            g.vab_linkMode = false;
            g.vab_linkFromId.clear();
            return;
        }
    }
    const PartDef *ld = g.ships.catalog().find("fuel_link");
    if(ld == nullptr) {
        g.toast("The catalog has no \"fuel_link\" part");
        g.vab_linkMode = false;
        g.vab_linkFromId.clear();
        return;
    }
    BuildShip::FuelLink fl;
    fl.def = ld;
    int n = (int)g.vab.fuelLinks.size() + 1;
    for(;;) {   // a unique "<part>_<n>" id among the existing links
        fl.id = ld->name + "_" + std::to_string(n);
        bool dup = false;
        for(size_t i = 0; i < g.vab.fuelLinks.size(); i++) {
            if(g.vab.fuelLinks[i].id == fl.id) { dup = true; break; }
        }
        if(!dup) { break; }
        n++;
    }
    fl.from = g.vab_linkFromId;
    fl.to = bp.id;
    g.vab.fuelLinks.push_back(fl);
    printf("[vab] fuel link %s: %s -> %s\n", fl.id.c_str(), fl.from.c_str(),
           fl.to.c_str());
    fflush(stdout);
    g.toast("Fuel link: %s feeds %s", fl.from.c_str(), fl.to.c_str());
    g.vab_linkMode = false;
    g.vab_linkFromId.clear();
}

void vabDeleteSelected(Game &g) {
    // a selected fuel link goes first (the two selections are exclusive)
    if(g.vab_linkSel >= 0 && (size_t)g.vab_linkSel < g.vab.fuelLinks.size()) {
        const std::string id = g.vab.fuelLinks[(size_t)g.vab_linkSel].id;
        g.vab.fuelLinks.erase(g.vab.fuelLinks.begin() + g.vab_linkSel);
        g.vab_linkSel = -1;
        g.toast("Deleted fuel link %s", id.c_str());
        return;
    }
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

void vabDetachSelected(Game &g) {
    const int sel = g.vab_selected;
    if(sel < 0 || (size_t)sel >= g.vab.parts.size()) { return; }
    if(sel == 0) { g.toast("Cannot detach the root part"); return; }
    const std::string rootId = g.vab.parts[(size_t)sel].id;
    BuildShip sub = g.vab.detachSubtree(sel);
    if(sub.parts.empty()) { return; }
    Game::VabSubassembly sa;
    sa.name = (g.vab.name.empty() ? std::string("ship") : g.vab.name)
              + " > " + rootId;
    sa.ship = sub;
    const int n = (int)sa.ship.parts.size();
    g.vab_subassemblies.push_back(sa);
    g.vab_selected = -1;
    vabClearHover(g);
    printf("[vab] detached %s (%d parts)\n", rootId.c_str(), n);
    fflush(stdout);
    g.toast("Detached %s (+%d) to Subassemblies", rootId.c_str(), n - 1);
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
    // Launch config from the top bar dropdowns, with the startup defaults
    // (the home body + the pad) as fallback.
    TerrainBody *hb = g.home;
    if(!g.vab_bodyName.empty()) {
        TerrainBody *b = g.sys.find(g.vab_bodyName);
        if(b != nullptr) { hb = b; }
    }
    const std::string scName = g.vab_scenarioName.empty() ? "pad" : g.vab_scenarioName;
    const ScenarioDef *sc = scenario_by_name(scName);
    /* defPath "": the ship was built in memory -- there is no file to
       respawn it from until it is saved (the Respawn button hides). */
    Vehicle *v = g.ships.place_ship_def(def, "", def.name, hb, sc, g.sys);
    // Non-pad scenarios place the ship in orbit (position + orbit velocity),
    // like the fleet spawn. Left live (NOT on rails) so it is
    // player-controlled -- the VAB's model.
    if(!sc->on_pad) { spawn_vehicle(v, *sc, hb, g.sys, 0.0); }
    // Crew ABOARD after the reposition: each kerbal then parks at the
    // capsule's final (orbit) pose, not the pad's.
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
    if(!g.vab_subassemblies.empty()) {
        g.toast("%d subassemblies stayed in the VAB",
                (int)g.vab_subassemblies.size());
    }
}

void vabOpen(Game &g) {
    // seed the launch config once (the dropdowns' initial selection): the
    // home body + the pad. Left alone afterwards, so a body/scenario chosen
    // in a prior VAB session is kept when the editor is re-entered.
    if(g.vab_bodyName.empty() && g.home != nullptr) { g.vab_bodyName = g.home->name; }
    if(g.vab_scenarioName.empty()) { g.vab_scenarioName = "pad"; }
    // park the flight camera (in EITHER mode -- free does not re-aim
    // itself) for the duration of the VAB session
    if(g.camera != nullptr && !g.vab_camSaved) {
        g.vab_camSaved = true;
        g.vab_camMode = g.camera->mode;
        g.vab_camPos = g.camera->pos;
        g.vab_camFwd = g.camera->forward;
        g.vab_camUp = g.camera->up;
        g.vab_camDistance = g.camera->distance;
        g.vab_camYaw = g.camera->orbitYaw;
        g.vab_camPitch = g.camera->orbitPitch;
        g.vab_camFocusBody = g.focusBody;
    }
    // aim the orbit camera at the build tree (empty build -> the origin)
    g.vab_center = glm::dvec3(0.0);
    double dist = 30.0;
    if(!g.vab.parts.empty()) {
        glm::dvec3 lo(1e30), hi(-1e30);
        for(size_t i = 0; i < g.vab.parts.size(); i++) {
            lo = glm::min(lo, g.vab.parts[i].localPos);
            hi = glm::max(hi, g.vab.parts[i].localPos);
        }
        g.vab_center = (lo + hi) * 0.5;
        dist = glm::length(hi - lo) * 1.2 + 10.0;
    }
    vabClearHover(g);
    g.vab_selected = -1;
    g.vab_linkMode = false;
    g.vab_linkFromId.clear();
    g.scene = Scene::Vab;
    if(g.camera != nullptr) {
        g.camera->toOrbit(g.vab_center);   // also from Free mode
        g.camera->distance = dist;
    }
    printf("[vab] entered the editor (sim paused)\n");
    fflush(stdout);
    g.toast("VAB -- the simulation is paused");
}

void vabClose(Game &g) {
    g.scene = Scene::Flight;
    vabClearHover(g);
    g.vab_linkMode = false;
    g.vab_linkFromId.clear();
    if(g.camera != nullptr) {
        if(g.vab_camSaved) {
            // hand the parked flight camera back exactly as it was
            g.vab_camSaved = false;
            g.focusBody = g.vab_camFocusBody;
            if(g.vab_camMode == CAM_FREE) {
                g.camera->setFreePose(g.vab_camPos, g.vab_camFwd, g.vab_camUp);
            } else {
                g.camera->mode = CAM_ORBIT;
                g.camera->orbitYaw = g.vab_camYaw;
                g.camera->orbitPitch = g.vab_camPitch;
                g.camera->distance = g.vab_camDistance;
                g.camera->Follow(g.focusWorldPos(g.focusBody));
                g.camera->ComputeView();   // sane pos/forward/up immediately
            }
        } else if(g.ship != nullptr && g.camera->mode == CAM_ORBIT) {
            // no parked pose (a --vab boot): re-aim like select_ship does
            g.focusBody = 0;
            g.camera->Follow(g.ship->get_center_of_mass());
            g.camera->distance = g.ship->isEva() ? 10.0 : 50.0;
        }
    }
    printf("[vab] back to flight (sim resumed)\n");
    fflush(stdout);
    g.toast("Back to flight -- the simulation resumes");
}
