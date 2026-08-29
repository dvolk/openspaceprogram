// radialtest.cpp -- the --radial-test spin-test ship builder.
//
// Builds a passive-tank test ship straight from the parts catalog (no JSON
// ship def). Every part is a passive single-stage tank -- no wheels, no
// thrusters -- so any spin is self-inflicted by the physics. The per-mode
// comments below document the exact weld layout and anchor coincidence.
#include "radialtest.h"

#include <stdexcept>

#include "body.h"     // create_body
#include "mesh.h"     // Mesh
#include "model.h"    // Model
#include "physics.h"  // setPosRot, GlueTogether
#include "texture.h"  // load_texture

RadialTestShip build_radial_test_ship(const std::string &mode,
                                      bool scenario_given,
                                      const std::string &scenario,
                                      const PartsCatalog &part_catalog,
                                      TerrainBody *home,
                                      TerrainBody *sun,
                                      Shader *partsshader)
{
    /* --radial-test: minimal test ships built straight from the
       catalog (no JSON ship def). Passive tanks only -- no
       wheels, no thrusters -- so any spin is self-inflicted by
       the physics:
       - "radial":  tank_r5h5 + a tank_r3h2 welded to its side
       - "stacked": the same pair welded along the axis (baseline)
       - "stacks":  two 2-part stacks welded side by side:
                    [tank_r5h5 + tank_r3h2] beside
                    [tank_r5h5 + tank_r3h2], the second stack's
                    root welded radially to the first stack's
                    root (the in-game way to build it).
       Stacked welds use attachDown's convention: the child sits
       on the parent's -Z side, anchors (0,0,-hP/2) /
       (0,0,+hC/2) coinciding in world space. */
    const PartDef *defBig = part_catalog.find("tank_r5h5");
    const PartDef *defSml = part_catalog.find("tank_r3h2");
    if(defBig == nullptr || defSml == nullptr) {
        throw std::runtime_error("--radial-test: tank_r5h5 / "
                                 "tank_r3h2 missing from the parts catalog");
    }

    /* honor an explicit --scenario, otherwise orbit (no pad
       contact, no terrain noise in the spin measurement) */
    const ScenarioDef *sc = scenario_by_name(
        scenario_given ? scenario : "rot-orbit");

    Vehicle *v = new Vehicle;
    v->m_parent = home;
    v->sun = sun;
    v->frame = home->rot_frame;

    const glm::dvec3 pad_dir = glm::normalize(glm::dvec3(0.005, 0.005, 1.0));
    const glm::dmat3 pad_orient = faceAlong(pad_dir);
    /* Start 50 m above the surface so a --scenario pad ship drops
       onto the ground (the lowest part would otherwise start
       embedded in the terrain). For orbit scenarios this is only
       staging -- spawn_vehicle repositions the ship. */
    const glm::dvec3 base = pad_dir
        * ((double)home->GetTerrainHeight(pad_dir) + 50.0);
    /* radial weld: child's local +Z (its axis) -> parent's
       local +X. Columns = images of X, Y, Z. */
    const glm::dmat3 rotZtoX(glm::dvec3(0, 0, -1),
                             glm::dvec3(0, 1, 0),
                             glm::dvec3(1, 0, 0));

    auto makeBody = [&](const PartDef *def) -> Body * {
        Mesh *mesh = new Mesh;
        mesh->FromFile((std::string("./res/") + def->mesh).c_str(), true);
        Model *model = new Model;
        model->FromData(mesh, partsshader,
                        load_texture((std::string("./res/") + def->texture).c_str()));
        model->hull_margin = def->hull_margin;
        return create_body(model, 0, 0, 0, (float)def->mass, false);
    };

    if(mode == "stacks") {
        /* Two 2-part stacks, side by side. Pad normal = local +Z,
           radial dir = local +X:
             stack 1: A1 (tank_r5h5, root) + A2 (tank_r3h2)
                      attached below A1, axis Z
             stack 2: B1 (tank_r5h5) welded to A1's +X side
                      (axis X) + B2 (tank_r3h2) attached beyond
                      B1 along B1's axis
           Layout (local): A1 (0,0,0)  A2 (0,0,-3.5)
                           B1 (7.5,0,0) B2 (11,0,0)
           Welds (anchors coincide in world space):
             A1-A2 stacked:  A1 (0,0,-2.5)   == A2 (0,0,+1)
             A1-B1 radial:   A1 (5,0,0)      == B1 (0,0,-2.5)
             B1-B2 stacked:  B1 (0,0,+2.5)   == B2 (0,0,-1) */
        v->name = "stacks4";
        Body *a1 = makeBody(defBig);
        Body *a2 = makeBody(defSml);
        Body *b1 = makeBody(defBig);
        Body *b2 = makeBody(defSml);
        setPosRot(a1, base, pad_orient);
        setPosRot(a2,
                  base - pad_orient * glm::dvec3(0.0, 0.0,
                                                 defBig->height / 2.0 + defSml->height / 2.0),
                  pad_orient);
        setPosRot(b1,
                  base + pad_orient * glm::dvec3(defBig->radius + defBig->height / 2.0, 0.0, 0.0),
                  pad_orient * rotZtoX);
        setPosRot(b2,
                  base + pad_orient * glm::dvec3(defBig->radius + defBig->height + defSml->height / 2.0, 0.0, 0.0),
                  pad_orient * rotZtoX);

        v->setRoot(a1);
        v->partDefs.push_back(defBig);
        v->partDefs.push_back(defSml);
        v->partDefs.push_back(defBig);
        v->partDefs.push_back(defSml);
        v->constraints.push_back(GlueTogether(a1, a2,
                                              glm::dvec3(0.0, 0.0, -defBig->height / 2.0),
                                              glm::dvec3(0.0, 0.0,  defSml->height / 2.0)));
        v->constraints.push_back(GlueTogether(a1, b1,
                                              glm::dvec3(defBig->radius, 0.0, 0.0),
                                              glm::dvec3(0.0, 0.0, -defBig->height / 2.0)));
        v->constraints.push_back(GlueTogether(b1, b2,
                                              glm::dvec3(0.0, 0.0,  defBig->height / 2.0),
                                              glm::dvec3(0.0, 0.0, -defSml->height / 2.0)));
        v->parts.push_back(a2);
        v->parts.push_back(b1);
        v->parts.push_back(b2);
    }
    else if(mode == "parstacks") {
        /* Two 2-part stacks side by side with ALL axes PARALLEL
           (pad normal = local +Z) -- the variant of 'stacks' where
           the second stack is NOT rotated, so both stacks' axes
           point the same way:
             stack 1: A1 (tank_r5h5, root) + A2 (tank_r3h2) below A1
             stack 2: B1 (tank_r5h5) welded to A1's +X side
                      + B2 (tank_r3h2) below B1
           Layout (local): A1 (0,0,0)    A2 (0,0,-3.5)
                           B1 (10,0,0)   B2 (10,0,-3.5)
           Welds (anchors coincide in world space):
             A1-A2 stacked:  A1 (0,0,-2.5)  == A2 (0,0,+1)
             B1-B2 stacked:  B1 (0,0,-2.5)  == B2 (0,0,+1)
             A1-B1 lateral:  A1 (5,0,0)     == B1 (-5,0,0) */
        v->name = "parstacks4";
        Body *a1 = makeBody(defBig);
        Body *a2 = makeBody(defSml);
        Body *b1 = makeBody(defBig);
        Body *b2 = makeBody(defSml);
        const double dz = defBig->height / 2.0 + defSml->height / 2.0;
        setPosRot(a1, base, pad_orient);
        setPosRot(a2, base - pad_orient * glm::dvec3(0.0, 0.0, dz), pad_orient);
        setPosRot(b1, base + pad_orient * glm::dvec3(defBig->radius + defBig->radius, 0.0, 0.0), pad_orient);
        setPosRot(b2, base + pad_orient * glm::dvec3(defBig->radius + defBig->radius, 0.0, -dz), pad_orient);

        v->setRoot(a1);
        v->partDefs.push_back(defBig);
        v->partDefs.push_back(defSml);
        v->partDefs.push_back(defBig);
        v->partDefs.push_back(defSml);
        v->constraints.push_back(GlueTogether(a1, a2,
                                              glm::dvec3(0.0, 0.0, -defBig->height / 2.0),
                                              glm::dvec3(0.0, 0.0,  defSml->height / 2.0)));
        v->constraints.push_back(GlueTogether(a1, b1,
                                              glm::dvec3(defBig->radius, 0.0, 0.0),
                                              glm::dvec3(-defBig->radius, 0.0, 0.0)));
        v->constraints.push_back(GlueTogether(b1, b2,
                                              glm::dvec3(0.0, 0.0, -defBig->height / 2.0),
                                              glm::dvec3(0.0, 0.0,  defSml->height / 2.0)));
        v->parts.push_back(a2);
        v->parts.push_back(b1);
        v->parts.push_back(b2);
    }
    else {
        v->name = (mode == "radial") ? "radial2"
                   : (mode == "parallel") ? "parallel2" : "stack2";
        Body *a = makeBody(defBig);
        setPosRot(a, base, pad_orient);
        Body *b = makeBody(defSml);
        v->setRoot(a);
        v->partDefs.push_back(defBig);
        if(mode == "radial") {
            /* B's bottom face (-hB/2) touches A's side at +rA */
            setPosRot(b, base + pad_orient * glm::dvec3(defBig->radius + defSml->height / 2.0, 0.0, 0.0),
                      pad_orient * rotZtoX);
            v->attachRadial(b, defSml);
        }
        else if(mode == "parallel") {
            /* B's side touches A's side at +rA; both axes stay on
               the pad normal (parallel). B at +X by rA + rB so the
               cylindrical surfaces meet; anchor world point (rA,0,0)
               on A == (-rB,0,0) on B. */
            setPosRot(b, base + pad_orient * glm::dvec3(defBig->radius + defSml->radius, 0.0, 0.0),
                      pad_orient);
            v->attachSide(b, defSml);
        }
        else {
            /* attachDown welds the child on the parent's -Z side:
               anchor coincidence needs B at base - (hA/2+hB/2)
               along the pad normal */
            setPosRot(b, base - pad_orient * glm::dvec3(0.0, 0.0, defBig->height / 2.0 + defSml->height / 2.0),
                      pad_orient);
            v->attachDown(b, defSml);
        }
        v->partDefs.push_back(defSml);
    }
    /* These hand-built ships bypass build_ship(), which is the only
       place partStages (the per-part stage index, kept parallel to
       parts) gets set -- so seed it here. All parts are passive
       single-stage tanks, so the value is a placeholder; init()
       only requires the vector to be parallel to parts. */
    v->partStages.assign(v->parts.size(), 1);
    v->controllerIndex = 0;
    v->init();
    v->setVelocity(glm::dvec3(0, 0, 0));

    RadialTestShip r;
    r.v = v;
    r.sc = sc;
    r.slot = 0;
    return r;
}
