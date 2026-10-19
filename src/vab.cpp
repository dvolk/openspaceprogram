// vab.cpp -- the VAB editor's interaction layer (see vab.h): physics-free
// picking of the build tree, the ghost preview pose, and placing a part.
// Picking reuses Bullet's convex cast (pick.cpp castRay) against per-part
// hulls; the ray lives directly in the build frame S.

#include "vab.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <string>

#include <glm/gtc/quaternion.hpp>   // angleAxis / mat3_cast (the snap + symmetry)

#include "../middleware/imgui/imgui.h"   // ImGui::GetIO().WantCaptureMouse

#include "buildings.h" // find_launch_site
#include "crew.h"     // KerbalProfile (the part window's seat rows name the cast)
#include "datadir.h" // datadir::ships (VAB Save target -- never the install tree)
#include "mesh.h"     // get_mesh (the part mesh's vertex array)
#include "part.h"     // Part (a standing suit's kerbal id, on the pad's crew line)
#include "resdir.h"   // resdir::path
#include "shipdef.h"  // PartsCatalog (resolve the armed palette name)
#include "btcommon.h" // complete Bullet types (the ghost hulls + collision objects)

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
    Mesh *m = get_mesh(std::string("res/") + def->mesh);
    if(m != nullptr && m->vs != nullptr && m->num_vertices >= 3) {
        a.hull = new btConvexHullShape(m->vs, (int)m->num_vertices,
                                       3 * sizeof(double));
        /* Zero, and explicitly so: Bullet's default convex margin is 0.04
           and its convex raycast reports the hit on the shape INFLATED by
           that margin, which would float a surface-attached part off the
           parent's skin by exactly the margin. These hulls are only ever
           raycast, so the skin buys nothing. */
        a.hull->setMargin(0.0);
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

/* A placement id "<base>_<n>" not yet used in the tree (deletes leave holes). */
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

/* Where part <id> can be CLICKED, for --ui-focus. The spiral in aimAtPoint is
   load-bearing, not decoration: tanker's part 1 projects to 960,540 from the
   default VAB view and the pick there returns part 3, so the projection alone
   would aim a click at the wrong part. */
std::string vabAimAtPart(Game &g, const std::string &id, double &sx, double &sy) {
    if(!sceneIs(g, SceneId::Vab)) { return "not in the VAB editor"; }
    int want = -1;
    for(size_t i = 0; i < g.vab.build.parts.size(); i++) {
        if(g.vab.build.parts[i].id == id) { want = (int)i; break; }
    }
    if(want < 0) { return "no such part in the build"; }
    return aimAtPoint(g, g.vab.build.parts[(size_t)want].localPos,
                      [&](int px, int py) {
        int got = -1;
        PickBodyHit hit;
        return pickVabPart(g, px, py, got, hit) && got == want;
    }, sx, sy);
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
    for(size_t i = 0; i < g.vab.build.parts.size(); i++) {
        const BuildPart &bp = g.vab.build.parts[i];
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
    const BuildPart &bp = g.vab.build.parts[(size_t)partIdx];
    const Node &n = bp.def->nodes[(size_t)nodeIdx];
    return bp.localPos + bp.localRot * n.pos;
}

int pickVabNode(Game &g, int px, int py, int partIdx, double thresholdPx) {
    if(partIdx < 0 || (size_t)partIdx >= g.vab.build.parts.size()) { return -1; }
    const BuildPart &bp = g.vab.build.parts[(size_t)partIdx];
    if(bp.def == nullptr) { return -1; }
    double best = thresholdPx;
    int bestI = -1;
    for(size_t i = 0; i < bp.def->nodes.size(); i++) {
        if(bp.def->nodes[i].surface) { continue; }   // stack ports only
        if(g.vab.build.nodeOccupied(partIdx, bp.def->nodes[i].id)) { continue; }
        double sx = 0, sy = 0;
        if(!projectPixel(g, vabNodePos(g, partIdx, (int)i), sx, sy)) { continue; }
        const double d = glm::length(glm::dvec2(sx - (double)px, sy - (double)py));
        if(d < best) { best = d; bestI = (int)i; }
    }
    return bestI;
}

void vabClearHover(Game &g) {
    g.vab.hover = -1;
    g.vab.hoverNode = -1;
    g.vab.hoverParent = -1;
    g.vab.ghostValid = false;
    g.vab.ghostRoot = false;
    g.vab.ghostAssembly = -1;
    g.vab.ghostClones.clear();
}

/* The armed subassembly's ROOT part, or the armed catalog part. */
static const PartDef *armedChildDef(const Game &g, const BuildShip **asmOut) {
    if(asmOut != nullptr) { *asmOut = nullptr; }
    if(g.vab.armedAsm >= 0
       && (size_t)g.vab.armedAsm < g.vab.subassemblies.size()) {
        const BuildShip &sub = g.vab.subassemblies[(size_t)g.vab.armedAsm].ship;
        if(sub.parts.empty() || sub.parts[0].def == nullptr) { return nullptr; }
        if(asmOut != nullptr) { *asmOut = &sub; }
        return sub.parts[0].def;
    }
    if(g.vab.armed.empty()) { return nullptr; }
    return g.parts.find(g.vab.armed);
}

void vabUpdateHover(Game &g, int px, int py) {
    vabClearHover(g);

    int pi = -1;
    PickBodyHit hit;
    const bool hitPart = pickVabPart(g, px, py, pi, hit);
    if(hitPart) { g.vab.hover = pi; }
    if(g.vab.linkMode) { return; }   // link picking: no placement ghost
    const BuildShip *asmShip = nullptr;
    const PartDef *childDef = armedChildDef(g, &asmShip);
    if(childDef == nullptr) { return; }  // inspecting only; no ghost
    if(g.vab.build.parts.empty()) {
        /* An empty build: the armed part/assembly becomes the ROOT,
           anchored at the S origin. */
        g.vab.ghostRoot = true;
        g.vab.ghostAssembly = asmShip ? g.vab.armedAsm : -1;
        g.vab.ghostSurface = false;
        g.vab.ghostPos = glm::dvec3(0.0);
        g.vab.ghostRot = glm::dmat3(1.0);
        g.vab.ghostRollUsed = 0.0;
        g.vab.ghostValid = true;
        return;
    }
    if(!hitPart) { return; }
    const BuildPart &pp = g.vab.build.parts[(size_t)pi];
    if(pp.def == nullptr) { return; }

    const int node = pickVabNode(g, px, py, pi, 24.0);
    const bool alt = altHeld();                      // Alt bypasses while held
    const bool snapL = g.vab.snapLen != alt;
    const bool snapA = g.vab.snapAng != alt;
    if(node >= 0) {
        // stack attach onto the hovered port (symmetry does not apply:
        // the synthesized axial ports are singletons)
        const Node &pn = pp.def->nodes[(size_t)node];
        const Node *cn = bestMatingChildNode(*childDef, pn.dir);
        if(cn == nullptr) { return; }
        const double roll = snapA ? snapAngleDeg(g.vab.ghostRoll) : g.vab.ghostRoll;
        g.vab.ghostRollUsed = roll;
        const AttachPose ap = attachNodes(pp.localPos, pp.localRot, pn, *cn,
                                          roll, 0.0);
        g.vab.hoverParent = pi;
        g.vab.hoverNode = node;
        g.vab.ghostSurface = false;
        g.vab.ghostAssembly = asmShip ? g.vab.armedAsm : -1;
        g.vab.ghostParentNode = pn.id;
        g.vab.ghostChildNode = cn->id;
        g.vab.ghostPos = ap.childPos;
        g.vab.ghostRot = ap.childRot;
        g.vab.ghostValid = true;
        return;
    }

    const Node *cs = childDef->findSurfaceNode();
    if(cs == nullptr) { return; }
    const glm::dvec3 pS = hit.point;   // already S frame (pickVabPart)
    const glm::dmat3 invR = glm::transpose(pp.localRot);
    glm::dvec3 localPoint = invR * (pS - pp.localPos);
    glm::dvec3 localNormal = glm::normalize(invR * hit.normal);
    double roll = g.vab.ghostRoll;
    if(snapA) { roll = snapAngleDeg(roll); }
    snapSurfaceContact(localPoint, localNormal, snapL, snapA);
    g.vab.ghostRollUsed = roll;
    const AttachPose ap = attachSurface(pp.localPos, pp.localRot, localPoint,
                                        localNormal, *cs, roll, 0.0);
    g.vab.hoverParent = pi;
    g.vab.hoverNode = -1;
    g.vab.ghostSurface = true;
    g.vab.ghostAssembly = asmShip ? g.vab.armedAsm : -1;
    g.vab.ghostPoint = localPoint;
    g.vab.ghostNormal = localNormal;
    g.vab.ghostChildNode = cs->id;
    g.vab.ghostPos = ap.childPos;
    g.vab.ghostRot = ap.childRot;
    g.vab.ghostValid = true;
    // radial symmetry: the clones ring the hovered parent's own axis
    g.vab.ghostClones = radialSymmetryClones(pp.localPos, pp.localRot, *cs,
                                             localPoint, localNormal, roll,
                                             0.0, g.vab.symmetry);
}

int vabPlace(Game &g) {
    if(!g.vab.ghostValid) { return -1; }
    const int parent = g.vab.ghostRoot ? -1 : g.vab.hoverParent;
    // parent -1 is the valid ROOT placement (empty build).
    if(!g.vab.ghostRoot && parent < 0) { return -1; }

    /* An armed SUBASSEMBLY: graft a copy under the resolved root edge.
       The list entry is NOT consumed -- placing is copy & paste. */
    if(g.vab.ghostAssembly >= 0
       && (size_t)g.vab.ghostAssembly < g.vab.subassemblies.size()) {
        const BuildShip &sub = g.vab.subassemblies[(size_t)g.vab.ghostAssembly].ship;
        if(sub.parts.empty() || sub.parts[0].def == nullptr) { return -1; }
        BuildPart root;
        root.def = sub.parts[0].def;
        root.id = sub.parts[0].id;   // graftTree uniquifies on collision
        root.parent = parent;
        if(g.vab.ghostSurface) {
            root.attach = AttachMode::Surface;
            root.contactPoint = g.vab.ghostPoint;
            root.contactNormal = g.vab.ghostNormal;
            root.childNode = g.vab.ghostChildNode;
            root.roll = g.vab.ghostRollUsed;
        } else {
            root.attach = AttachMode::Down;   // node ids carry the mating
            root.parentNode = g.vab.ghostParentNode;
            root.childNode = g.vab.ghostChildNode;
            root.angle = g.vab.ghostRollUsed;
        }
        const size_t ri = g.vab.build.graftTree(sub, root);
        for(size_t k = 0; k < g.vab.ghostClones.size(); k++) {
            const SymClone &c = g.vab.ghostClones[k];
            BuildPart rc = root;
            rc.attach = AttachMode::Surface;   // clones are always surface edges
            rc.parentNode.clear();
            rc.contactPoint = c.edge.point;
            rc.contactNormal = c.edge.normal;
            rc.roll = c.edge.rollDeg;
            g.vab.build.graftTree(sub, rc);
        }
        g.vab.ghostRoll = 0.0;
        g.vab.ghostClones.clear();
        return (int)ri;
    }

    const PartDef *childDef = g.parts.find(g.vab.armed);
    if(childDef == nullptr) { return -1; }

    int idn = (int)g.vab.build.parts.size() + 1;
    BuildPart np;
    np.def = childDef;
    np.id = nextBuildId(g.vab.build, g.vab.armed, idn);
    np.parent = parent;
    if(g.vab.ghostSurface) {
        np.attach = AttachMode::Surface;
        np.contactPoint = g.vab.ghostPoint;
        np.contactNormal = g.vab.ghostNormal;
        np.childNode = g.vab.ghostChildNode;
        np.roll = g.vab.ghostRollUsed;
    } else {
        np.attach = AttachMode::Down;   // a stack edge; the node ids carry the mating
        np.parentNode = g.vab.ghostParentNode;
        np.childNode = g.vab.ghostChildNode;
        np.angle = g.vab.ghostRollUsed;
    }
    g.vab.build.parts.push_back(np);
    // the radial-symmetry siblings: independent surface parts on the same parent
    for(size_t k = 0; k < g.vab.ghostClones.size(); k++) {
        const SymClone &c = g.vab.ghostClones[k];
        BuildPart sp;
        sp.def = childDef;
        sp.id = nextBuildId(g.vab.build, g.vab.armed, idn);
        sp.parent = np.parent;
        sp.attach = AttachMode::Surface;
        sp.contactPoint = c.edge.point;
        sp.contactNormal = c.edge.normal;
        sp.childNode = np.childNode;
        sp.roll = c.edge.rollDeg;
        g.vab.build.parts.push_back(sp);
    }
    g.vab.build.recomputePoses();
    g.vab.ghostRoll = 0.0;   // the next placement starts unrolled
    g.vab.ghostClones.clear();
    return (int)g.vab.build.parts.size() - 1;
}

/* Q/E rotate: the ghost's pending roll, or the selected part's attach roll
   with its subtree re-solved. */
void vabRotate(Game &g, double deltaDeg) {
    const bool snap = g.vab.snapAng != altHeld();
    if(g.vab.ghostValid) {
        g.vab.ghostRoll = snap ? gridStepDeg(g.vab.ghostRoll, deltaDeg)
                               : g.vab.ghostRoll + deltaDeg;
        return;
    }
    if(g.vab.selected < 0 || (size_t)g.vab.selected >= g.vab.build.parts.size()) {
        return;
    }
    BuildPart &bp = g.vab.build.parts[(size_t)g.vab.selected];
    if(bp.parent < 0) { return; }   // the root has no edge to spin
    if(!snap) { g.vab.build.rotatePart(g.vab.selected, deltaDeg); return; }
    double &a = (bp.attach == AttachMode::Surface) ? bp.roll : bp.angle;
    a = gridStepDeg(a, deltaDeg);
    g.vab.build.recomputePoses();
}

namespace {

// The build part with this id, or nullptr.
const BuildPart *buildPartById(const Game &g, const std::string &id) {
    for(const BuildPart &bp : g.vab.build.parts) {
        if(bp.id == id) { return &bp; }
    }
    return nullptr;
}

}  // namespace

void vabLinkClick(Game &g) {
    if(g.vab.hover < 0 || (size_t)g.vab.hover >= g.vab.build.parts.size()) {
        return;   // empty space: stay in link mode, keep waiting
    }
    const BuildPart &bp = g.vab.build.parts[(size_t)g.vab.hover];
    /* Name the part each click took. The source already says so in its toast,
       but the destination's only output is the RESULT -- and a refusal prints
       the loop that already existed, not the part just clicked -- so a test
       could aim the destination at any part at all and still pass (246).
       hover only comes from pickVabPart, which skips a part with no def. */
    printf("[vab] link %s %s\n",
           g.vab.linkFromId.empty() ? "src" : "dest", bp.id.c_str());
    fflush(stdout);
    if(g.vab.linkFromId.empty()) {
        g.vab.linkFromId = bp.id;
        g.toast("Link source: %s -- now click the destination", bp.id.c_str());
        return;
    }
    /* The source was picked on an EARLIER click, and Del/X can have removed it
       since (link mode outlives a delete). A link from a part that is no longer
       in the build feeds nothing -- toShipDef drops it quietly -- and no loop
       check can ever see it, so drop the pick rather than author a ghost. The
       destination click still stands, so stay in link mode. */
    if(buildPartById(g, g.vab.linkFromId) == nullptr) {
        g.toast("The link source is gone -- click it again");
        g.vab.linkFromId.clear();
        return;
    }
    if(bp.id == g.vab.linkFromId) {
        g.toast("Source and destination must differ");
        return;
    }
    for(size_t i = 0; i < g.vab.build.fuelLinks.size(); i++) {
        const BuildShip::FuelLink &fl = g.vab.build.fuelLinks[i];
        if(fl.from == g.vab.linkFromId && fl.to == bp.id) {
            g.toast("%s already feeds %s", fl.from.c_str(), fl.to.c_str());
            g.vab.linkMode = false;
            g.vab.linkFromId.clear();
            return;
        }
    }
    const PartDef *ld = g.parts.find("fuel_link");
    if(ld == nullptr) {
        g.toast("The catalog has no \"fuel_link\" part");
        g.vab.linkMode = false;
        g.vab.linkFromId.clear();
        return;
    }
    /* A loop cannot drain, so build_ship throws on one -- at launch, inside a
       UI handler, where the player has no undo. Refuse it HERE instead, where
       the build is still on the table and the pair they clicked is the thing to
       change. Checked over fuel GROUPS, exactly as build_ship groups them, so a
       link this accepts cannot turn into a launch-time throw (#291).
       Two refusals, two sentences: a def loaded off disk can arrive already
       looping, and naming THAT loop blames the player for a link they did not
       just make -- and would refuse every later click with the wrong reason. */
    const std::string existing = g.vab.build.fuelLinkLoop();
    if(!existing.empty()) {
        printf("[vab] fuel link refused: the design already loops: %s\n",
               existing.c_str());
        fflush(stdout);
        g.toast("This design already has a fuel-link loop: %s -- delete a link",
                existing.c_str());
        g.vab.linkMode = false;
        g.vab.linkFromId.clear();
        return;
    }
    const BuildShip::FuelLink probe{ld, "", g.vab.linkFromId, bp.id};
    const std::string loop = g.vab.build.fuelLinkLoop(&probe);
    if(!loop.empty()) {
        printf("[vab] fuel link refused (a loop cannot drain): %s\n",
               loop.c_str());
        fflush(stdout);
        g.toast("Fuel links cannot form a loop: %s", loop.c_str());
        g.vab.linkMode = false;
        g.vab.linkFromId.clear();
        return;
    }
    BuildShip::FuelLink fl;
    fl.def = ld;
    int n = (int)g.vab.build.fuelLinks.size() + 1;
    for(;;) {   // a unique "<part>_<n>" id among the existing links
        fl.id = ld->name + "_" + std::to_string(n);
        bool dup = false;
        for(size_t i = 0; i < g.vab.build.fuelLinks.size(); i++) {
            if(g.vab.build.fuelLinks[i].id == fl.id) { dup = true; break; }
        }
        if(!dup) { break; }
        n++;
    }
    fl.from = g.vab.linkFromId;
    fl.to = bp.id;
    g.vab.build.fuelLinks.push_back(fl);
    printf("[vab] fuel link %s: %s -> %s\n", fl.id.c_str(), fl.from.c_str(),
           fl.to.c_str());
    fflush(stdout);
    g.toast("Fuel link: %s feeds %s", fl.from.c_str(), fl.to.c_str());
    g.vab.linkMode = false;
    g.vab.linkFromId.clear();
}

void vabDeleteSelected(Game &g) {
    // a selected fuel link goes first (the two selections are exclusive)
    if(g.vab.linkSel >= 0 && (size_t)g.vab.linkSel < g.vab.build.fuelLinks.size()) {
        const std::string id = g.vab.build.fuelLinks[(size_t)g.vab.linkSel].id;
        g.vab.build.fuelLinks.erase(g.vab.build.fuelLinks.begin() + g.vab.linkSel);
        g.vab.linkSel = -1;
        g.toast("Deleted fuel link %s", id.c_str());
        return;
    }
    const int sel = g.vab.selected;
    if(sel < 0 || (size_t)sel >= g.vab.build.parts.size()) { return; }
    if(sel == 0) { g.toast("Cannot delete the root part"); return; }
    const std::string id = g.vab.build.parts[(size_t)sel].id;
    if(g.vab.build.removePart(sel)) {
        g.vab.selected = -1;
        vabClearHover(g);
        g.toast("Deleted %s", id.c_str());
    }
}

void vabDetachSelected(Game &g) {
    const int sel = g.vab.selected;
    if(sel < 0 || (size_t)sel >= g.vab.build.parts.size()) { return; }
    if(sel == 0) { g.toast("Cannot detach the root part"); return; }
    const std::string rootId = g.vab.build.parts[(size_t)sel].id;
    BuildShip sub = g.vab.build.detachSubtree(sel);
    if(sub.parts.empty()) { return; }
    g.vab.selected = -1;
    vabClearHover(g);
    // A lone part is not a subassembly -- the palette already has it.
    if(sub.parts.size() == 1) {
        printf("[vab] deleted %s (single part, not stashed)\n", rootId.c_str());
        fflush(stdout);
        g.toast("Deleted %s", rootId.c_str());
        return;
    }
    VabState::Subassembly sa;
    sa.name = (g.vab.build.name.empty() ? std::string("ship") : g.vab.build.name)
              + " > " + rootId;
    sa.ship = sub;
    const int n = (int)sa.ship.parts.size();
    g.vab.subassemblies.push_back(sa);
    printf("[vab] detached %s (%d parts)\n", rootId.c_str(), n);
    fflush(stdout);
    g.toast("Detached %s (+%d) to Subassemblies", rootId.c_str(), n - 1);
}

namespace {

// Ship-def identity for the VAB is a NAME ("racer"). Paths and ".json" are
// reduced to the stem.
std::string shipDefStem(const std::string &spec) {
    std::string s = spec;
    while(!s.empty() && (s.back() == '/' || s.back() == '\\')) { s.pop_back(); }
    const size_t slash = s.find_last_of("/\\");
    if(slash != std::string::npos) { s = s.substr(slash + 1); }
    if(s.size() > 5 && s.compare(s.size() - 5, 5, ".json") == 0) {
        s.resize(s.size() - 5);
    }
    return s;
}

// Resolve a ship-def name (or path) to a file. Data-dir ships/ wins over
// stock res/ships/.
std::string shipDefFile(const std::string &spec) {
    if(spec.find('/') != std::string::npos
       || spec.find('\\') != std::string::npos) {
        const std::string p = resdir::path(spec);
        std::error_code ec;
        if(std::filesystem::is_regular_file(p, ec)) { return p; }
    }
    const std::string stem = shipDefStem(spec);
    if(stem.empty()) { return std::string(); }
    std::error_code ec;
    const std::string user = datadir::ships() + "/" + stem + ".json";
    if(std::filesystem::is_regular_file(user, ec)) { return user; }
    return resdir::path("res/ships/" + stem + ".json");
}

} // namespace

void vabSave(Game &g, const char *name) {
    // Always saved to the data dir's ships/ (user content).
    const std::string stem = shipDefStem(name != nullptr ? name : "");
    if(stem.empty()) {
        g.toast("Save FAILED: empty name");
        return;
    }
    g.vab.build.name = stem;   // the save name IS the ship's identity
    datadir::make_dir(datadir::ships());
    const std::string out = datadir::ships() + "/" + stem + ".json";
    if(save_ship_def(g.vab.build, out.c_str())) {
        printf("[vab] saved %s (%d parts)\n", stem.c_str(), (int)g.vab.build.parts.size());
        fflush(stdout);
        g.toast("Saved %s", stem.c_str());
    } else {
        g.toast("Save FAILED: %s", stem.c_str());
    }
}

bool vabLoad(Game &g, const char *name) {
    const std::string spec = name != nullptr ? name : "";
    const std::string stem = shipDefStem(spec);
    const std::string file = shipDefFile(spec);
    if(file.empty()) {
        printf("[vab] load failed %s: empty name\n", spec.c_str());
        fflush(stdout);
        g.toast("Load failed: empty name");
        return false;
    }
    ShipDef def;
    try {
        def = load_ship_def(file.c_str(), g.parts);
    } catch(const std::exception &e) {
        printf("[vab] load failed %s: %s\n", stem.c_str(), e.what());
        fflush(stdout);
        g.toast("Load failed: %s", e.what());
        return false;
    }
    if(def.parts.empty()) {
        printf("[vab] load failed %s: no parts\n", stem.c_str());
        fflush(stdout);
        g.toast("Load failed: %s (no parts)", stem.c_str());
        return false;
    }
    /* Replace the current build, then re-aim the editor (not a scene entry). */
    g.vab.build = BuildShip::fromShipDef(def);
    vabAimCamera(g);
    vabClearHover(g);
    g.vab.selected = -1;
    g.vab.linkMode = false;
    g.vab.linkFromId.clear();
    // linkSel indexes the tree that was just replaced; reset it (a stale one
    // would flip Del from "detach" to destructive "delete").
    g.vab.linkSel = -1;
    /* A design on disk holds no people (crew stands on the pad), and loading it
       replaces every part in the build, so the seats the player named belong to
       parts that are no longer here. Say so: the next launch falls back to the
       pad's line, and a player who named a crew cannot otherwise see the choice
       go (issue #288). Do NOT claim the loaded design has no seats -- it may have
       plenty, just not these ones. */
    if(!g.vab.seatAssigns.empty()) {
        const int n = (int)g.vab.seatAssigns.size();
        printf("[vab] dropped %d seat assignment(s): the loaded design replaced "
               "the parts they name\n", n);
        fflush(stdout);
        g.toast("%d seat assignment%s dropped: the loaded design replaced the "
                "seats they name", n, n == 1 ? "" : "s");
    }
    g.vab.seatAssigns.clear();
    g.vab.partWinId.clear();
    printf("[vab] loaded %s (%d parts)\n", stem.c_str(), (int)g.vab.build.parts.size());
    fflush(stdout);
    g.toast("Loaded %s (%d parts)", stem.c_str(), (int)g.vab.build.parts.size());
    return true;
}

void vabLaunch(Game &g) {
    if(g.vab.build.parts.empty()) { g.toast("Nothing to launch"); return; }
    /* The editor prunes the seat plan once per frame (vabUpdate), so a plan can
       still name a part that left the build when the launch arrives from the test
       hooks, which fire detach/assign/launch inside one frame. The fill loop
       below treats the plan as the player's last word on the crew, so prune
       first: an entry naming a part that is not in this build has no seat to
       fill. */
    vabPruneSeats(g);
    ShipDef def = g.vab.build.toShipDef();
    // Launch config from the top bar dropdowns, with the startup defaults
    // (the home body + the pad) as fallback.
    TerrainBody *hb = g.home;
    if(!g.vab.bodyName.empty()) {
        TerrainBody *b = g.sys.find(g.vab.bodyName);
        if(b != nullptr) { hb = b; }
    }
    const std::string scName = g.vab.scenarioName.empty() ? "pad" : g.vab.scenarioName;
    const ScenarioDef *sc = scenario_by_name(scName);
    const StaticBuilding *site = nullptr;
    if(sc->on_pad) {
        site = find_launch_site(hb, g.vab.siteName);
        if(site == nullptr) {
            g.toast("No launch site on %s", hb->name.c_str());
            return;
        }
        /* One deck, one ship (#231): two ships on a pad sit inside each
           other's grounded proximity radius, so the railed one wakes on the
           first tick and the pair torque each other. Ask rather than tell --
           the out is right there, and nothing silently relocates. */
        if(Vehicle *blocker = findPadOccupant(site, g.vehicles)) {
            printf("[vab] launch refused: '%s' occupies '%s'\n",
                   blocker->name.c_str(), site->id.c_str());
            fflush(stdout);
            /* Deferred by construction: the card answers on a later frame, so
               the callback re-finds the occupant and re-resolves the site by
               ID instead of holding this pointer -- and the BODY by NAME, which
               the same rule requires (game.h: a Question callback captures
               identity, never a pointer). Holding hb segfaults when a system
               switch while the card is up frees it. ui_remove_vehicle's own
               guards (crew aboard, last ship) do the refusing. A crew policy
               for pad clearing is the open half of #231. */
            const std::string padName = hb->name;
            std::string siteId = site->id;
            g.ask(blocker->name + " is standing on " + site->display_name
                    + ". Clear the pad?",
                  {"Clear the pad", "Cancel"},
                  [&g, padName, siteId](int choice) {
                if(choice != 0) { return; }
                /* Both lookups are total here, and that is an invariant rather
                   than a hope: a card cannot outlive the world it was asked in,
                   because every teardown drops it (clearEphemeralState, #304)
                   and a system switch always runs a teardown. So the body and
                   the site this callback named are still the ones that asked --
                   case 258 pins the drop, and the toasts that used to sit on
                   these two nulls were dead code. */
                TerrainBody *padBody = g.sys.find(padName);
                assert(padBody != nullptr
                       && "a Question outlived the system it was asked in");
                StaticBuilding *padSite = find_launch_site(padBody, siteId);
                assert(padSite != nullptr
                       && "a launch site vanished under a live Question");
                Vehicle *occ = findPadOccupant(padSite, g.vehicles);
                if(occ == nullptr) {
                    g.toast("The pad is clear now");
                    return;
                }
                g.ui_remove_vehicle(occ);
            });
            return;
        }
    }
    /* A kerbal-rooted build is a PERSON, and a person is one part (part.h). The
       palette hides kerbal parts, so this only bites a def loaded from a file
       (--vab, the VAB's load). Guard rather than throw: place_vehicle_def's
       throw would land inside a UI handler, and a launch is something the player
       can undo by editing the build. */
    if(!kerbalDefConsistent(def)) {
        printf("[vab] launch refused: '%s' is kerbal-rooted but has %d parts\n",
               def.name.c_str(), (int)def.parts.size());
        fflush(stdout);
        g.toast("Launch refused: a kerbal must be a single part");
        return;
    }
    /* Crew chosen in the part window: validate it BEFORE the ship exists. A
       person who is not standing on the pad (taken by an earlier launch, left
       behind by a system switch) cannot be put in a seat, and a ship that
       launched with the seat half-filled reads like a success. */
    const bool pickedCrew = !g.vab.seatAssigns.empty();
    if(pickedCrew) {
        const std::string why = vabSeatProblem(g);
        if(!why.empty()) {
            printf("[vab] launch refused: %s\n", why.c_str());
            fflush(stdout);
            g.toast("Launch refused: %s", why.c_str());
            return;
        }
    }
    /* defPath "": the ship was built in memory -- no file to respawn from.
       build_ship throws on a data bug the build can still be holding (a
       fuel-link loop: vabLinkClick refuses to author one, a def loaded off disk
       may already contain one). place_vehicle_def frees the half-built vehicle
       and rethrows, so the build and the crew plan are untouched -- report it
       the way vabLoad does rather than letting a throw land in an imgui handler,
       which is a SIGABRT and a lost game (#291). */
    Vehicle *v = nullptr;
    try {
        v = place_vehicle_def(g, def, "", def.name, hb, sc, site);
    } catch(const std::exception &e) {
        printf("[vab] launch refused: %s\n", e.what());
        fflush(stdout);
        g.toast("Launch refused: %s", e.what());
        return;
    }
    // Left live (NOT on rails) so it is player-controlled.
    if(!sc->on_pad) { spawn_vehicle(v, *sc, hb, g.sys, 0.0, g.time); }
    // Crew, after the reposition so the ship is where the player launched it
    // before the roster is consulted.
    if(pickedCrew) {
        /* Every named person, in their named seat -- and NOTHING else fills.
           An unassigned seat flies empty: the player chose the crew, so the
           FIFO line stays on the pad. A build part keeps its id through
           toShipDef -> build_ship (vehicle.cpp), which is what makes the
           partId key survive here. A slot named EMPTY (profileId 0) is skipped
           rather than dropped from the list, so it stays a choice (issue #287). */
        for(const VabState::SeatAssign &a : g.vab.seatAssigns) {
            if(a.profileId == 0) { continue; }
            size_t pi = (size_t)-1;
            for(size_t i = 0; i < v->parts.size(); i++) {
                if(v->parts[i]->id == a.partId) { pi = i; break; }
            }
            Part *suit = takeCrewById(g.sys, a.profileId);
            assert(pi < v->parts.size() && suit != nullptr);   // checked above
            fill_seat(v, pi, g.sys, suit, a.slot);
        }
    } else {
        spawn_crew(v, g.sys);
    }
    g.select_vehicle(v);
    vabClearHover(g);
    g.vab.armed.clear();
    g.vab.ghostRoll = 0.0;
    g.vab.selected = -1;
    /* The plan was consumed: those people are aboard, no longer standing on the
       pad. Keep the names and the NEXT launch of this build refuses over people
       who are already in orbit. */
    g.vab.seatAssigns.clear();
    g.vab.partWinId.clear();
    /* Collapse the stack to [Flight]: the launched ship is the active one now;
       leaving the editor's camera pose would make the next vabOpen skip
       capturing one. */
    enterFlight(g);
    printf("[vab] launched '%s' (%d parts)\n", v->name.c_str(),
           (int)g.vab.build.parts.size());
    fflush(stdout);
    g.toast("Launched %s", v->name.c_str());
    if(!g.vab.subassemblies.empty()) {
        const int n = (int)g.vab.subassemblies.size();
        g.toast("%d subassemb%s stayed in the VAB", n, n == 1 ? "ly" : "ies");
    }
}

/* --- seat assignments (see vab.h) ------------------------------------------
   The window is a convenience over VabState::seatAssigns; everything that
   matters to the sim happens at launch (vabLaunch). */

namespace {

// A seat of the current build, addressable by (partId, slot).
bool seatExists(const Game &g, const std::string &partId, int slot) {
    const BuildPart *bp = buildPartById(g, partId);
    return bp != nullptr && bp->def != nullptr
        && slot >= 0 && slot < bp->def->crew_capacity;
}

}  // namespace

/* Name a seat, or empty it. profileId 0 is a CHOICE and is stored, not erased:
   the list being non-empty is what tells vabLaunch the player chose the crew
   (game.h), so erasing the last entry would turn "nobody in this seat" back
   into "never asked" and put the pad's FIFO line aboard. */
bool vabAssignSeat(Game &g, const std::string &partId, int slot,
                   uint64_t profileId) {
    if(!seatExists(g, partId, slot)) { return false; }
    for(size_t i = 0; i < g.vab.seatAssigns.size(); i++) {
        const VabState::SeatAssign &a = g.vab.seatAssigns[i];
        if(a.partId != partId || a.slot != slot) { continue; }
        g.vab.seatAssigns[i].profileId = profileId;
        return true;
    }
    g.vab.seatAssigns.push_back({partId, slot, profileId});
    return true;
}

/* Drop assignments whose seat is no longer in the build: Delete and Detach
   renumber the tree under them. A plan for a part that left is gone -- say so,
   because otherwise the next launch silently draws the pad's line again
   (issue #288). */
void vabPruneSeats(Game &g) {
    int dropped = 0;
    for(size_t i = g.vab.seatAssigns.size(); i-- > 0; ) {
        const VabState::SeatAssign &a = g.vab.seatAssigns[i];
        if(seatExists(g, a.partId, a.slot)) { continue; }
        const KerbalProfile *p = g.roster.find(a.profileId);
        const std::string who = p != nullptr ? " '" + p->name + "'" : "";
        // Named HERE, per entry: a summary cannot say which of several parts
        // went, and naming one of them reads as blaming the wrong one.
        printf("[vab] dropped the seat plan for '%s'%s (it left the build)\n",
               a.partId.c_str(), who.c_str());
        fflush(stdout);
        g.vab.seatAssigns.erase(g.vab.seatAssigns.begin() + (std::ptrdiff_t)i);
        dropped++;
    }
    if(dropped > 0) {
        g.toast("%d seat assignment%s dropped: no such seat in this build",
                dropped, dropped == 1 ? "" : "s");
        if(g.vab.seatAssigns.empty()) {
            g.toast("No seats named: the next launch draws the pad's line");
        }
    }
    // The window follows its part: no part means no window.
    if(!g.vab.partWinId.empty()
       && !buildPartById(g, g.vab.partWinId)) {
        g.vab.partWinId.clear();
    }
}

std::string vabSeatProblem(const Game &g) {
    StaticBuilding *pad = homeCrewPad(g.sys);
    for(const VabState::SeatAssign &a : g.vab.seatAssigns) {
        if(a.profileId == 0) { continue; }   // a chosen empty seat names nobody
        const KerbalProfile *p = g.roster.find(a.profileId);
        const std::string who = p != nullptr
            ? p->name : ("roster id " + std::to_string(a.profileId));
        bool standing = false;
        if(pad != nullptr) {
            for(const Part *suit : pad->crew) {
                if(suit->kerbal != nullptr
                   && suit->kerbal->id == a.profileId) {
                    standing = true;
                    break;
                }
            }
        }
        if(!standing) { return "'" + who + "' is not at the launch pad"; }
        for(const VabState::SeatAssign &b : g.vab.seatAssigns) {
            if(&b == &a) { continue; }
            if(b.profileId == a.profileId) {
                return "'" + who + "' cannot sit in two seats";
            }
        }
    }
    return "";
}

/* Open the VAB part window for build part `partIdx`, to be placed near the click
   at (px, py). Any part with a def qualifies: seats are one section of the
   dossier, not the condition for opening it. */
void vabOpenPartWindow(Game &g, int partIdx, int px, int py) {
    if(partIdx < 0 || (size_t)partIdx >= g.vab.build.parts.size()) { return; }
    const BuildPart &bp = g.vab.build.parts[(size_t)partIdx];
    if(bp.def == nullptr) { return; }
    g.vab.partWinId = bp.id;
    g.vab.partWinMx = px;
    g.vab.partWinMy = py;
    g.vab.partWinPlaced = false;   // place it next to the click on the first draw
    /* The stat rows are plain imgui Text, so --ui-list cannot see them and a
       case has nothing to pin. This line is the testable face of the dossier:
       what the window decided the part carries. Log-style short names, as
       Vehicle::resourceName uses everywhere else stdout is the reader. */
    char res[200];
    res[0] = '\0';
    size_t used = 0;
    for(int r = 0; r < (int)ResourceType::Num; r++) {
        if(bp.def->capacity[r] <= 0.0f) { continue; }
        const int n = snprintf(res + used, sizeof(res) - used, "%s%.1f%s %s",
                               used ? ", " : "", bp.def->capacity[r],
                               Vehicle::resourceUnit(r), Vehicle::resourceName(r));
        if(n < 0 || (size_t)n >= sizeof(res) - used) { break; }
        used += (size_t)n;
    }
    printf("[vab] part window: '%s' seats %d, carries %s\n", bp.id.c_str(),
           bp.def->crew_capacity, res[0] ? res : "nothing");
    fflush(stdout);
}

void vabPickPart(Game &g, int px, int py) {
    int idx = -1;
    PickBodyHit hit;
    if(!pickVabPart(g, px, py, idx, hit)) {
        printf("[vab] part window: nothing under the cursor\n");
        fflush(stdout);
        return;
    }
    vabOpenPartWindow(g, idx, px, py);
}

/* Aim the editor's orbit camera at the build tree. */
void vabAimCamera(Game &g) {
    g.vab.center = glm::dvec3(0.0);
    double dist = 30.0;
    if(!g.vab.build.parts.empty()) {
        glm::dvec3 lo(1e30), hi(-1e30);
        for(size_t i = 0; i < g.vab.build.parts.size(); i++) {
            lo = glm::min(lo, g.vab.build.parts[i].localPos);
            hi = glm::max(hi, g.vab.build.parts[i].localPos);
        }
        g.vab.center = (lo + hi) * 0.5;
        dist = glm::length(hi - lo) * 1.2 + 10.0;
    }
    if(g.camera != nullptr) {
        g.camera->toOrbit(g.vab.center);   // also from Free mode
        g.camera->distance = dist;
    }
    // e2e anchor: "the editor camera followed the tree".
    printf("[vab] camera aimed at the build: %d part(s), %.1f m out\n",
           (int)g.vab.build.parts.size(), dist);
    fflush(stdout);
}

/* Enter the editor: push the Vab scene (captures the flight camera pose). */
void vabOpen(Game &g) { pushScene(g, SceneId::Vab); }

/* Scene table's enter hook. */
void vabEnter(Game &g) {
    // Seed the launch config once (home body + pad); left alone afterwards.
    // An empty siteName means "the body's default site" (its first
    // VAB_LAUNCH_PAD) until the top-bar combo picks one.
    if(g.vab.bodyName.empty() && g.home != nullptr) { g.vab.bodyName = g.home->name; }
    if(g.vab.scenarioName.empty()) { g.vab.scenarioName = "pad"; }
    vabClearHover(g);
    g.vab.selected = -1;
    g.vab.linkMode = false;
    g.vab.linkFromId.clear();
    g.vab.linkSel = -1;      // no link selected on a fresh entry
    vabAimCamera(g);
    printf("[vab] entered the editor (sim keeps coasting)\n");
    fflush(stdout);
    g.toastLog("VAB -- the sim keeps running");
}

/* Scene table's exit hook: drop only the state that is meaningless once the
   tree is off screen. Deliberately NOT the armed part / selection / roll --
   "Back to game" must leave the editor as you left it. */
void vabExit(Game &g) {
    vabClearHover(g);
    g.vab.linkMode = false;
    g.vab.linkFromId.clear();
}

/* Leave the editor: pop the stack (hands the parked flight camera back). */
void vabClose(Game &g) {
    popScene(g);
    printf("[vab] back to %s (sim keeps coasting)\n", sceneName(curSceneId(g)));
    fflush(stdout);
    g.toastLog("Back from the VAB -- the sim keeps running");
}

/* Register the headless transition hooks (--vab-launch, --vab-detach,
   --vab-assign, ...), each fired at most once. They join the pre-scene phase
   AFTER the main-loop hooks -- the order the old per-frame poll used -- and
   a launch collapses the stack to Flight, so the live scene is read after
   them. The inVab gate reproduces the old early return. --vab-open,
   --vab-load, --vab-save and --vab-close are gone: the scene transition is a
   title-screen button and the load, save and back-to-game controls are
   widgets, all reachable with --ui-click/--ui-type. */
void vabInstallHooks(Game &g) {
    GameArgs &args = g.args;
    auto inVab = [](Game &g) { return sceneIs(g, SceneId::Vab); };
    /* The top bar's Load and Save are clickable now, and --ui-type fills the
       name field, so both headless stand-ins are gone. Note what --vab-load
       used to allow: any ship file at all, including the testships that
       list_vab_ship_defs hides from the picker (shipdef.h:578) -- a case could
       load a ship no player can pick. */
    /* --vab-detach: select by index and detach, in one step. Players pick the
       part and click "Detach subtree"; cases 77, 78, 239 and 240 do that. This
       is left for 244, which wants a detach and a launch close enough to land
       in one frame. */
    if(args.vab_detach_ms >= 0 && args.vab_detach_idx >= 0) {
        const int idx = args.vab_detach_idx;
        g.addHook(kHookPreScene, args.vab_detach_ms, inVab, [idx](Game &g) {
            g.vab.selected = idx;
            g.vab.linkSel = -1;   // a part detach, not a fuel-link delete
            vabDetachSelected(g);
            printf("[vab] detach hook: build %d parts, %d subassemblies\n",
                   (int)g.vab.build.parts.size(), (int)g.vab.subassemblies.size());
            fflush(stdout);
        });
    }
    /* --vab-assign: a seat plan the window refuses to build. It still opens the
       window it stands in for, so --ui-list dumps the rows a player would have
       had. Case 237 is the only user: the dropdown greys an already-seated
       person, so naming one person twice has no click path. */
    for(const GameArgs::VabAssignArg &a : args.vab_assigns) {
        const int partIdx = a.partIdx, slot = a.slot;
        const std::string kerbal = a.kerbal;
        g.addHook(kHookPreScene, a.ms, inVab, [partIdx, slot, kerbal](Game &g) {
            if(partIdx < 0 || (size_t)partIdx >= g.vab.build.parts.size()) {
                printf("[vab] assign hook: no build part %d\n", partIdx);
                fflush(stdout);
                return;
            }
            const std::string partId = g.vab.build.parts[(size_t)partIdx].id;
            const KerbalProfile *p = g.roster.byName(kerbal);
            if(p == nullptr) {
                printf("[vab] assign hook: no kerbal '%s'\n", kerbal.c_str());
                fflush(stdout);
                return;
            }
            const ImGuiIO &io = ImGui::GetIO();
            vabOpenPartWindow(g, partIdx, (int)io.DisplaySize.x / 2,
                              (int)io.DisplaySize.y / 2);
            const bool ok = vabAssignSeat(g, partId, slot, p->id);
            printf("[vab] assign hook: '%s' seat %d = '%s'%s\n", partId.c_str(),
                   slot, kerbal.c_str(), ok ? "" : " (no such seat)");
            fflush(stdout);
        });
    }
    /* --vab-launch: the launch itself is driven by the top bar's LAUNCH button
       (--ui-click "VAB TopBar/LAUNCH"), which is all the other cases needed this
       hook for. It survives for ONE thing a click cannot do: land in the SAME
       frame as another hook. A --ui-click presses in one frame and releases in
       the next (ButtonBehavior needs the release), and vabUpdate prunes the seat
       plan between frames, so case 244's stale-plan launch -- detach and launch
       with no prune in between -- only arrives through here. */
    if(args.vab_launch_ms >= 0) {
        g.addHook(kHookPreScene, args.vab_launch_ms, inVab,
                  [](Game &g) { vabLaunch(g); });
    }
}

/* The editor's mouse half, part 1: hover-pick and ghost preview, which must
   resolve before the 3D pass draws them. WantCaptureMouse here is LAST
   frame's value -- the click half therefore lives in vabClickAfterUI. */
void vabUpdate(Game &g) {
    /* Delete/Detach renumber the tree under an assignment's feet; drop the ones
       that no longer name a seat before the window reads them. */
    vabPruneSeats(g);
    if(ImGui::GetIO().WantCaptureMouse) {
        vabClearHover(g);
    } else {
        vabUpdateHover(g, g.mposX, g.mposY);
    }
}

/* The editor's mouse half, part 2: what a left click DOES. Called after
   ImGui::Render, not from vabUpdate, because the gate has to be THIS frame's
   WantCaptureMouse. --sim-mouse pushes MOTION and BUTTON_DOWN in one
   poll_events batch, so on the frame a click lands, the value vabUpdate
   sampled is still the previous frame's: the cursor read as being over empty
   space, and a click aimed at the palette placed a part THROUGH the palette
   window (measured: a click at 1750,100 inside the Palette rect still
   launched 1 part). A real flick-and-click delivers the same pair in one
   batch, so this is a game bug, not just a harness one.
   Deferring within the frame is safe: lmbPressed is an edge that lives
   exactly one frame (poll_events clears it on entry). The hover it acts on is
   one frame older than the gate -- unavoidable, the ghost is drawn before
   imgui runs -- so a click on the very frame the cursor leaves a window sees a
   cleared hover and does nothing rather than attaching at the wrong parent. */
void vabClickAfterUI(Game &g) {
    if(!g.lmbPressed) { return; }
    if(ImGui::GetIO().WantCaptureMouse) { return; }   // imgui took this click
    if(g.vab.linkMode) {
        vabLinkClick(g);
    } else if(!g.vab.armed.empty() || g.vab.armedAsm >= 0) {
        vabPlace(g);
    } else if(g.vab.hover >= 0) {
        // nothing armed: a plain click selects the hovered part
        g.vab.selected = g.vab.hover;
        g.vab.linkSel = -1;
        /* The panel says "selected: <id> (<label>)" as plain imgui Text, which
           has no item id, so --ui-list cannot show it (the registry skips
           id==0). A test that aims a 3D pick at a part therefore cannot tell
           whether it hit the part it meant. This line makes the pick
           assertable, so a case can pin it (#311). hover only comes from
           pickVabPart, which skips a part with no def. */
        printf("[vab] select %d %s\n", g.vab.selected,
               g.vab.build.parts[(size_t)g.vab.selected].id.c_str());
        fflush(stdout);
    }
}
