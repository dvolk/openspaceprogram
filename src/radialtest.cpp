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
       - "radial":  tank_r2.25h5 + a tank_r1.5h2 welded to its side
       - "stacked": the same pair welded along the axis (baseline)
       - "stacks":  two 2-part stacks welded side by side:
                    [tank_r2.25h5 + tank_r1.5h2] beside
                    [tank_r2.25h5 + tank_r1.5h2], the second stack's
                    root welded radially to the first stack's
                    root (the in-game way to build it).
       Stacked welds use attachDown's convention: the child sits
       on the parent's -Z side, anchors (0,0,-hP/2) /
       (0,0,+hC/2) coinciding in world space. */
    const PartDef *defBig = part_catalog.find("tank_r2.25h5");
    const PartDef *defSml = part_catalog.find("tank_r1.5h2");
    if(defBig == nullptr || defSml == nullptr) {
        throw std::runtime_error("--radial-test: tank_r2.25h5 / "
                                 "tank_r1.5h2 missing from the parts catalog");
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

    /* One Part (a rigid body + render model wrapped with its catalog spec).
       All parts are passive single-stage tanks (stage 1); the Part OWNS the
       Body and the def points into the catalog (which outlives the ship). */
    auto makePart = [&](const PartDef *def) -> Part * {
        Mesh *mesh = new Mesh;
        mesh->FromFile((std::string("./res/") + def->mesh).c_str(), true);
        Model *model = new Model;
        model->FromData(mesh, partsshader,
                        load_texture((std::string("./res/") + def->texture).c_str()));
        model->hull_margin = def->hull_margin;
        Body *b = create_body(model, 0, 0, 0, (float)def->mass, false);
        Part *p = new Part;
        p->body  = b;
        p->def   = def;
        p->stage = 1;
        return p;
    };
    /* Weld two Parts at coinciding local anchors, recording the link (by
       pointer) + anchors the same way Vehicle::attach() does. */
    auto weld = [&](Part *pa, Part *pb,
                    const glm::dvec3 &paA, const glm::dvec3 &pbA) {
        v->constraints.push_back(GlueTogether(pa->body, pb->body, paA, pbA));
        v->constraintLinks.push_back(std::make_pair(pa, pb));
        v->constraintAnchors.push_back(std::make_pair(paA, pbA));
    };

    if(mode == "stacks") {
        /* Two 2-part stacks, side by side. Pad normal = local +Z,
           radial dir = local +X:
             stack 1: A1 (tank_r2.25h5, root) + A2 (tank_r1.5h2)
                      attached below A1, axis Z
             stack 2: B1 (tank_r2.25h5) welded to A1's +X side
                      (axis X) + B2 (tank_r1.5h2) attached beyond
                      B1 along B1's axis
           Layout (local): A1 (0,0,0)  A2 (0,0,-3.5)
                           B1 (4.75,0,0) B2 (8.25,0,0)
           Welds (anchors coincide in world space):
             A1-A2 stacked:  A1 (0,0,-2.5)   == A2 (0,0,+1)
             A1-B1 radial:   A1 (2.25,0,0)   == B1 (0,0,-2.5)
             B1-B2 stacked:  B1 (0,0,+2.5)   == B2 (0,0,-1) */
        v->name = "stacks4";
        Part *a1 = makePart(defBig);
        Part *a2 = makePart(defSml);
        Part *b1 = makePart(defBig);
        Part *b2 = makePart(defSml);
        setPosRot(a1->body, base, pad_orient);
        setPosRot(a2->body,
                  base - pad_orient * glm::dvec3(0.0, 0.0,
                                                 defBig->height / 2.0 + defSml->height / 2.0),
                  pad_orient);
        setPosRot(b1->body,
                  base + pad_orient * glm::dvec3(defBig->radius + defBig->height / 2.0, 0.0, 0.0),
                  pad_orient * rotZtoX);
        setPosRot(b2->body,
                  base + pad_orient * glm::dvec3(defBig->radius + defBig->height + defSml->height / 2.0, 0.0, 0.0),
                  pad_orient * rotZtoX);

        v->setRoot(a1);
        v->parts.push_back(a2);
        v->parts.push_back(b1);
        v->parts.push_back(b2);
        weld(a1, a2,
             glm::dvec3(0.0, 0.0, -defBig->height / 2.0),
             glm::dvec3(0.0, 0.0,  defSml->height / 2.0));
        weld(a1, b1,
             glm::dvec3(defBig->radius, 0.0, 0.0),
             glm::dvec3(0.0, 0.0, -defBig->height / 2.0));
        weld(b1, b2,
             glm::dvec3(0.0, 0.0,  defBig->height / 2.0),
             glm::dvec3(0.0, 0.0, -defSml->height / 2.0));
    }
    else if(mode == "parstacks") {
        /* Two 2-part stacks side by side with ALL axes PARALLEL
           (pad normal = local +Z) -- the variant of 'stacks' where
           the second stack is NOT rotated, so both stacks' axes
           point the same way:
             stack 1: A1 (tank_r2.25h5, root) + A2 (tank_r1.5h2) below A1
             stack 2: B1 (tank_r2.25h5) welded to A1's +X side
                      + B2 (tank_r1.5h2) below B1
           Layout (local): A1 (0,0,0)    A2 (0,0,-3.5)
                           B1 (4.5,0,0)  B2 (4.5,0,-3.5)
           Welds (anchors coincide in world space):
             A1-A2 stacked:  A1 (0,0,-2.5)  == A2 (0,0,+1)
             B1-B2 stacked:  B1 (0,0,-2.5)  == B2 (0,0,+1)
             A1-B1 lateral:  A1 (2.25,0,0) == B1 (-2.25,0,0) */
        v->name = "parstacks4";
        Part *a1 = makePart(defBig);
        Part *a2 = makePart(defSml);
        Part *b1 = makePart(defBig);
        Part *b2 = makePart(defSml);
        const double dz = defBig->height / 2.0 + defSml->height / 2.0;
        setPosRot(a1->body, base, pad_orient);
        setPosRot(a2->body, base - pad_orient * glm::dvec3(0.0, 0.0, dz), pad_orient);
        setPosRot(b1->body, base + pad_orient * glm::dvec3(defBig->radius + defBig->radius, 0.0, 0.0), pad_orient);
        setPosRot(b2->body, base + pad_orient * glm::dvec3(defBig->radius + defBig->radius, 0.0, -dz), pad_orient);

        v->setRoot(a1);
        v->parts.push_back(a2);
        v->parts.push_back(b1);
        v->parts.push_back(b2);
        weld(a1, a2,
             glm::dvec3(0.0, 0.0, -defBig->height / 2.0),
             glm::dvec3(0.0, 0.0,  defSml->height / 2.0));
        weld(a1, b1,
             glm::dvec3(defBig->radius, 0.0, 0.0),
             glm::dvec3(-defBig->radius, 0.0, 0.0));
        weld(b1, b2,
             glm::dvec3(0.0, 0.0, -defBig->height / 2.0),
             glm::dvec3(0.0, 0.0,  defSml->height / 2.0));
    }
    else {
        v->name = (mode == "radial") ? "radial2"
                   : (mode == "parallel") ? "parallel2" : "stack2";
        Part *a = makePart(defBig);
        setPosRot(a->body, base, pad_orient);
        Part *b = makePart(defSml);
        v->setRoot(a);
        if(mode == "radial") {
            /* B's bottom face (-hB/2) touches A's side at +rA */
            setPosRot(b->body, base + pad_orient * glm::dvec3(defBig->radius + defSml->height / 2.0, 0.0, 0.0),
                      pad_orient * rotZtoX);
            v->attachRadial(b);
        }
        else if(mode == "parallel") {
            /* B's side touches A's side at +rA; both axes stay on
               the pad normal (parallel). B at +X by rA + rB so the
               cylindrical surfaces meet; anchor world point (rA,0,0)
               on A == (-rB,0,0) on B. */
            setPosRot(b->body, base + pad_orient * glm::dvec3(defBig->radius + defSml->radius, 0.0, 0.0),
                      pad_orient);
            v->attachSide(b);
        }
        else {
            /* attachDown welds the child on the parent's -Z side:
               anchor coincidence needs B at base - (hA/2+hB/2)
               along the pad normal */
            setPosRot(b->body, base - pad_orient * glm::dvec3(0.0, 0.0, defBig->height / 2.0 + defSml->height / 2.0),
                      pad_orient);
            v->attachDown(b);
        }
    }
    /* These hand-built ships bypass build_ship(), which normally resolves
       the controller from the ship def. All parts are passive single-stage
       tanks (each Part already carries stage 1), so the controller is just
       the root. */
    v->controller = v->parts[0];
    v->init();
    v->setVelocity(glm::dvec3(0, 0, 0));

    RadialTestShip r;
    r.v = v;
    r.sc = sc;
    r.slot = 0;
    return r;
}
