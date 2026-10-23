// radialtest.cpp -- the --radial-test spin-test ship builder: passive-tank
// ships from the parts catalog. Every edge goes through Vehicle::attachMode ->
// attachPose, so the real attach geometry is tested.
#include "radialtest.h"

#include <stdexcept>

#include "body.h"     // create_part_body
#include "mesh.h"     // get_mesh
#include "texture.h"  // get_texture

RadialTestShip build_radial_test_ship(const std::string &mode,
                                      const PartsCatalog &part_catalog,
                                      TerrainBody *home,
                                      TerrainBody *sun,
                                      Shader *partsshader)
{
    const PartDef *defBig = part_catalog.find("tank_r2.25h5");
    const PartDef *defSml = part_catalog.find("tank_r1.5h2");
    if(defBig == nullptr || defSml == nullptr) {
        throw std::runtime_error("--radial-test: tank_r2.25h5 / "
                                 "tank_r1.5h2 missing from the parts catalog");
    }

    const ScenarioDef *sc = scenario_by_name("rot-orbit");

    Vehicle *v = new Vehicle;
    v->m_parent = home;
    v->sun = sun;
    v->frame = home->rot_frame;

    const glm::dvec3 pad_dir = glm::normalize(glm::dvec3(0.005, 0.005, 1.0));
    const glm::dmat3 pad_orient = faceAlong(pad_dir);
    /* 50 m up so a pad-scenario ship drops cleanly. */
    const glm::dvec3 base = pad_dir
        * ((double)home->GetTerrainHeight(pad_dir) + 50.0);

    auto makePart = [&](const PartDef *def) -> Part * {
        Part *p = make_part(*def, def->hull_margin, nullptr, nullptr,
                            def->name, partsshader);
        p->stage = 1;
        return p;
    };

    if(mode == "stacks") {
        /* Pad normal = local +Z. */
        v->name = "stacks4";
        Part *a1 = makePart(defBig);
        Part *a2 = makePart(defSml);
        Part *b1 = makePart(defBig);
        Part *b2 = makePart(defSml);

        v->setRoot(a1);
        v->attachMode(a2, 0, AttachMode::Down);
        v->attachSurface(b1, 0, glm::dvec3(defBig->radius, 0.0, 0.0),
                         glm::dvec3(1.0, 0.0, 0.0));
        v->attachMode(b2, 2, AttachMode::Up);      // B2 outward beyond B1
    }
    else if(mode == "parstacks") {
        /* Two 2-part stacks side by side with ALL axes PARALLEL. */
        v->name = "parstacks4";
        Part *a1 = makePart(defBig);
        Part *a2 = makePart(defSml);
        Part *b1 = makePart(defBig);
        Part *b2 = makePart(defSml);

        v->setRoot(a1);
        v->attachMode(a2, 0, AttachMode::Down);
        // B1 beside A1 at clock 0, axes parallel
        v->attachSurface(b1, 0, glm::dvec3(defBig->radius, 0.0, 0.0),
                         glm::dvec3(1.0, 0.0, 0.0));
        v->attachMode(b2, 2, AttachMode::Down);   // B2 below B1
    }
    else {
        v->name = (mode == "parallel") ? "parallel2" : "stack2";
        Part *a = makePart(defBig);
        Part *b = makePart(defSml);
        v->setRoot(a);
        if(mode == "parallel") {
            /* Side to side at +rA; both axes stay on the pad. */
            v->attachSurface(b, 0, glm::dvec3(defBig->radius, 0.0, 0.0),
                             glm::dvec3(1.0, 0.0, 0.0));
        }
        else {
            /* B on A's -Z side, face to face. */
            v->attachDown(b);
        }
    }
    /* Hand-built: bypasses build_ship(), which resolves the attach poses from
       the catalog's node data. */
    v->controller = v->parts[0];
    v->init();
    v->placeShip(base, pad_orient);
    v->enterWorld();
    v->setVelocity(glm::dvec3(0, 0, 0));

    RadialTestShip r;
    r.v = v;
    r.sc = sc;
    return r;
}
