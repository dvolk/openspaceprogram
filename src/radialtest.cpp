// radialtest.cpp -- the --radial-test spin-test ship builder.
//
// Builds a passive-tank test ship straight from the parts catalog (no JSON
// ship def). Every part is a passive single-stage tank -- no wheels, no
// thrusters -- so any spin is self-inflicted by the physics. Every edge goes
// through Vehicle::attachMode -> attachPose, the same solver build_ship uses,
// so these layouts exercise the real attach geometry rather than literals.
#include "radialtest.h"

#include <stdexcept>

#include "body.h"     // create_part_body
#include "mesh.h"     // get_mesh
#include "texture.h"  // get_texture

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
       - "radial":  tank_r2.25h5 + a tank_r1.5h2 attached to its side
       - "stacked": the same pair attached along the axis (baseline)
       - "stacks":  two 2-part stacks attached side by side:
                    [tank_r2.25h5 + tank_r1.5h2] beside
                    [tank_r2.25h5 + tank_r1.5h2], the second stack's
                    root attached radially to the first stack's
                    root (the in-game way to build it). */
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

    /* One Part (shared render assets + hull + mass, wrapped with the
       catalog spec). All parts are passive single-stage tanks (stage 1);
       the Part OWNS the Body and the def points into the catalog (which
       outlives the ship). */
    auto makePart = [&](const PartDef *def) -> Part * {
        Mesh *mesh = get_mesh(std::string("./res/") + def->mesh);
        Texture *tex = get_texture(std::string("./res/") + def->texture);
        Body *b = create_part_body(mesh, partsshader, tex, (float)def->mass,
                                   def->hull_margin);
        Part *p = new Part;
        p->body  = b;
        p->def   = def;
        p->stage = 1;
        return p;
    };

    if(mode == "stacks") {
        /* Two 2-part stacks, side by side. Pad normal = local +Z, radial
           dir = local +X:
             stack 1: A1 (tank_r2.25h5, root) + A2 (tank_r1.5h2) below A1
             stack 2: B1 (tank_r2.25h5) radial off A1's +X side (its axis
                      turned onto X) + B2 (tank_r1.5h2) stacked outward
                      beyond B1 along that axis.
           Every edge goes through attachMode/attachPose, so the layout is
           the solver's, not hand-written literals. */
        v->name = "stacks4";
        Part *a1 = makePart(defBig);
        Part *a2 = makePart(defSml);
        Part *b1 = makePart(defBig);
        Part *b2 = makePart(defSml);

        v->setRoot(a1);
        v->attachMode(a2, 0, AttachMode::Down);    // A2 below A1
        v->attachMode(b1, 0, AttachMode::Radial);  // B1 off A1's +X side
        v->attachMode(b2, 2, AttachMode::Up);      // B2 outward beyond B1
    }
    else if(mode == "parstacks") {
        /* Two 2-part stacks side by side with ALL axes PARALLEL (pad normal
           = local +Z) -- the variant of 'stacks' where the second stack is
           NOT rotated, so both stacks' axes point the same way:
             stack 1: A1 (root) + A2 below A1
             stack 2: B1 side-by-side with A1 (parallel axes) + B2 below B1 */
        v->name = "parstacks4";
        Part *a1 = makePart(defBig);
        Part *a2 = makePart(defSml);
        Part *b1 = makePart(defBig);
        Part *b2 = makePart(defSml);

        v->setRoot(a1);
        v->attachMode(a2, 0, AttachMode::Down);   // A2 below A1
        // B1 beside A1: surface-attach at clock 0 on A1's side (parallel axes)
        v->attachSurface(b1, 0, glm::dvec3(defBig->radius, 0.0, 0.0),
                         glm::dvec3(1.0, 0.0, 0.0));
        v->attachMode(b2, 2, AttachMode::Down);   // B2 below B1
    }
    else {
        v->name = (mode == "radial") ? "radial2"
                   : (mode == "parallel") ? "parallel2" : "stack2";
        Part *a = makePart(defBig);
        Part *b = makePart(defSml);
        v->setRoot(a);
        if(mode == "radial") {
            /* B's bottom face (-hB/2) touches A's side at +rA */
            v->attachRadial(b);
        }
        else if(mode == "parallel") {
            /* B's side touches A's side at +rA; both axes stay on the pad
               normal (parallel). Surface-attach B at clock 0 on A's side: its
               surface node (-rB,0,0) lands on the contact (rA,0,0), so B sits
               at +X by rA + rB and the cylindrical surfaces meet. */
            v->attachSurface(b, 0, glm::dvec3(defBig->radius, 0.0, 0.0),
                             glm::dvec3(1.0, 0.0, 0.0));
        }
        else {
            /* attachDown stacks B on A's -Z side, face to face. */
            v->attachDown(b);
        }
    }
    /* These hand-built ships bypass build_ship(), which normally resolves
       the controller from the ship def. All parts are passive single-stage
       tanks (each Part already carries stage 1), so the controller is just
       the root. */
    v->controller = v->parts[0];
    v->init();
    v->placeShip(base, pad_orient);
    v->enterWorld();
    v->setVelocity(glm::dvec3(0, 0, 0));

    RadialTestShip r;
    r.v = v;
    r.sc = sc;
    r.slot = 0;
    return r;
}
