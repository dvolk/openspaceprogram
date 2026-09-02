// test_shipload: the GL-free ship/part JSON data model (src/shipdef.cpp).
// Runs from the repo root (needs res/):
//   make test   (or: ./test_shipload)
//
// Covers: catalog + ship-def parsing (the parent-relative tree schema:
// ids, parent/attach/angle/offset/stage, construction-order validation),
// the aggregates the Vehicle would compute, the default-controller rule,
// the attachPose geometry (anchor coincidence across mode/angle/offset),
// and the error paths.

#include "shipdef.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>

static int failures = 0;
#define CHECK(cond) do { \
        if(!(cond)) { \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++; \
        } \
    } while(0)

static bool expect_throw(const std::function<void()> &fn) {
    try { fn(); }
    catch(const std::runtime_error &) { return true; }
    return false;
}

static bool near(double a, double b) { return std::fabs(a - b) < 1e-6; }
static bool vnear(const glm::dvec3 &a, const glm::dvec3 &b) {
    return glm::length(a - b) < 1e-6;
}
static bool mnear(const glm::dmat3 &a, const glm::dmat3 &b) {
    const glm::dmat3 d = a - b;
    double s = 0;
    for(int i = 0; i < 3; i++) for(int j = 0; j < 3; j++) s += d[i][j] * d[i][j];
    return std::sqrt(s) < 1e-6;
}
// the weld invariant: the anchors coincide in world space
static bool coincide(const AttachPose &p, const glm::dvec3 &ppos,
                     const glm::dmat3 &prot) {
    const glm::dvec3 wp = ppos + prot * p.parentAnchor;
    const glm::dvec3 wc = p.childPos + p.childRot * p.childAnchor;
    return vnear(wp, wc);
}

// a fixed non-trivial parent orientation (+Z pointing at world (1,1,1))
static glm::dmat3 testOrient() {
    const glm::dvec3 z = glm::normalize(glm::dvec3(1.0, 1.0, 1.0));
    const glm::dvec3 x = glm::normalize(glm::cross(glm::dvec3(0.0, 1.0, 0.0), z));
    const glm::dvec3 y = glm::cross(z, x);
    return glm::dmat3(x, y, z);
}

int main() {
    // --- parts catalog ----------------------------------------------------
    PartsCatalog cat = load_parts_catalog("res/parts.json");
    CHECK(cat.parts.size() == 30);
    CHECK(cat.find("nope") == nullptr);

    const PartDef *cap = cat.find("capsule");
    const PartDef *rw  = cat.find("reaction_wheel");
    const PartDef *eng = cat.find("engine");
    const PartDef *ft  = cat.find("fuel_tank");
    CHECK(cap != nullptr && rw != nullptr && eng != nullptr && ft != nullptr);

    // the EVA kerbal (gen_kerbal.py): a crew-mass part with no ship
    // behaviors (no wheel or thruster) but a small hydrazine tank for its
    // RCS suit -- the mass INCLUDES that propellant (a spent suit keeps
    // its dry structure, like the tanks above).
    const PartDef *kb = cat.find("kerbal");
    CHECK(kb != nullptr);
    CHECK(kb->type == "kerbal");
    CHECK(kb->mass > 50.0 && kb->mass < 150.0);
    CHECK(kb->torque == 0.0 && kb->fuel_rate == 0.0);
    CHECK(kb->capacity[(int)ResourceType::Hydrazine] > 0.0f); // the suit's RCS propellant
    CHECK(kb->capacity[(int)ResourceType::Hydrazine] < 100.0f); // a suit load, not a tank
    CHECK(kb->mass > kb->capacity[(int)ResourceType::Hydrazine]); // mass includes the fuel

    // pre-size parts default to the legacy 2 m cube (radius 1, height 2)
    CHECK(cap->radius == 1.0 && cap->height == 2.0);
    CHECK(eng->radius == 1.0 && eng->height == 2.0);

    // type is a free-form label; behavior comes from the fields. The exact
    // numbers are owned by gen_parts.py, so pin the invariants, not the values.
    CHECK(cap->type == "capsule");
    CHECK(cap->mass > 0.0);
    CHECK(cap->mesh == "capsule.obj");
    CHECK(cap->texture == "capsule.png");
    CHECK(cap->torque > 0.0);   // the capsule carries an attitude wheel
    CHECK(cap->fuel_rate == 0.0 && cap->exhaust_velocity == 0.0);

    CHECK(rw->type == "reaction_wheel");
    CHECK(rw->mass > 0.0);
    CHECK(rw->torque > 0.0);
    // the wheel is a thin disc: height = 25% of the radius
    CHECK(rw->radius == 1.0 && rw->height == 0.25);
    CHECK(rw->mesh == "reaction_wheel_r1h0.25.obj");

    // engine is the pump: thrust params, but no propellant of its own
    CHECK(eng->type == "engine");
    CHECK(eng->mass > 0.0);
    CHECK(eng->fuel_rate > 0.0);
    CHECK(eng->exhaust_velocity > 0.0);
    CHECK(eng->torque == 0.0);
    CHECK(eng->capacity[(int)ResourceType::Hydrogen] == 0.0f); // fuel moved to the tank
    CHECK(eng->capacity[(int)ResourceType::LOX] == 0.0f);
    // the thrust model: T = (H2 + LOX flow) * ve = 2 * fuel_rate * ve
    CHECK(near(eng->fullThrust(), 2.0 * eng->fuel_rate * eng->exhaust_velocity));

    // fuel tank is the reservoir: holds the propellant, no thrust params
    CHECK(ft->type == "fuel_tank");
    CHECK(ft->mass > 0.0);
    CHECK(ft->mesh == "fuel_tank.obj");
    CHECK(ft->texture == "fuel_tank.png");
    CHECK(ft->torque == 0.0);
    CHECK(ft->fuel_rate == 0.0 && ft->exhaust_velocity == 0.0);
    CHECK(ft->capacity[(int)ResourceType::Hydrogen] > 0.0f);
    CHECK(ft->capacity[(int)ResourceType::LOX] > 0.0f);
    // the mass INCLUDES the propellant (a spent tank keeps its dry structure)
    CHECK(ft->mass > ft->capacity[(int)ResourceType::Hydrogen]
                       + ft->capacity[(int)ResourceType::LOX]);

    // hull margin: no catalog part sets one -> -1 (physics falls back to
    // its default); the field itself still parses
    CHECK(cap->hull_margin == -1.0);
    CHECK(eng->hull_margin == -1.0);
    CHECK(rw->hull_margin == -1.0);
    {
        const char *hm = "/tmp/test_shipload_hullmargin.json";
        std::ofstream f(hm);
        f << "{ \"parts\": [ { \"name\": \"x\", \"type\": \"engine\", "
             "\"mesh\": \"a.obj\", \"texture\": \"a.png\", \"mass\": 1.0, "
             "\"hull_margin\": 0.0 } ] }";
        f.close();
        PartsCatalog hmcat = load_parts_catalog(hm);
        CHECK(hmcat.find("x")->hull_margin == 0.0);
        std::remove(hm);
    }

    // sized parts: radius/height are explicit, thrust scales with the size.
    const PartDef *e5 = cat.find("engine_r5h10");
    const PartDef *e3 = cat.find("engine_r3h6");
    const PartDef *t32 = cat.find("tank_r3h2");
    const PartDef *t11 = cat.find("tank_r1h1");
    const PartDef *t55 = cat.find("tank_r5h5");
    const PartDef *c32 = cat.find("capsule_r3h6");
    CHECK(e5 != nullptr && e3 != nullptr && t32 != nullptr
          && t11 != nullptr && t55 != nullptr && c32 != nullptr);
    CHECK(t55->radius == 5.0 && t55->height == 5.0);
    CHECK(e5->radius == 5.0 && e5->height == 10.0);
    CHECK(e3->radius == 3.0 && e3->height == 6.0);
    // bigger = better: thrust, capacity and torque all scale with the size
    CHECK(e5->fullThrust() > e3->fullThrust() && e3->fullThrust() > eng->fullThrust());
    CHECK(t32->radius == 3.0 && t32->height == 2.0);
    CHECK(t11->radius == 1.0 && t11->height == 1.0);
    CHECK(t32->capacity[(int)ResourceType::Hydrogen]
          > t11->capacity[(int)ResourceType::Hydrogen]);
    CHECK(c32->radius == 3.0 && c32->height == 6.0 && c32->torque > cap->torque);

    // nose caps: a simple pointy cone on a tank's top face, one per tank
    // radius; height = radius / 2; passive (no behavior fields at all)
    const PartDef *nc1 = cat.find("nose_cap");
    const PartDef *nc3 = cat.find("nose_cap_r3h1.5");
    const PartDef *nc5 = cat.find("nose_cap_r5h2.5");
    CHECK(nc1 != nullptr && nc3 != nullptr && nc5 != nullptr);
    CHECK(nc1->radius == 1.0 && nc1->height == 0.5);
    CHECK(nc3->radius == 3.0 && nc3->height == 1.5);
    CHECK(nc5->radius == 5.0 && nc5->height == 2.5);
    for(const PartDef *nc : { nc1, nc3, nc5 }) {
        CHECK(near(nc->height, nc->radius / 2.0));
        CHECK(nc->mass > 0.0);
        CHECK(nc->type == "nose_cap");
        CHECK(nc->torque == 0.0);
        CHECK(nc->fuel_rate == 0.0 && nc->exhaust_velocity == 0.0);
        for(size_t r = 0; r < nc->capacity.size(); r++) {
            CHECK(nc->capacity[r] == 0.0f);
        }
    }
    CHECK(nc5->mesh == "nose_cap_r5h2.5.obj");
    CHECK(nc5->texture == "nose_cap.png");

    // --- ship def: the parent-relative tree schema ------------------------
    // racer: a bare linear stack (no attach given -> default chain, all
    // "down"). Geometry is now derived, so there are no offsets to check.
    ShipDef def = load_ship_def("res/ships/racer.json", cat);
    CHECK(def.name == "racer");
    CHECK(def.parts.size() == 4);
    CHECK(def.parts[0].part == "capsule");
    CHECK(def.parts[0].def == cap);
    CHECK(def.parts[1].def == rw);
    CHECK(def.parts[2].def == ft);   // the fuel tank
    CHECK(def.parts[3].def == eng);
    // auto ids, per catalog name
    CHECK(def.parts[0].id == "capsule_1");
    CHECK(def.parts[2].id == "fuel_tank_1");
    // the default parent chain: each part on the previous one, all "down"
    CHECK(def.parts[0].parent == -1);            // root
    CHECK(def.parts[1].parent == 0);
    CHECK(def.parts[2].parent == 1);
    CHECK(def.parts[3].parent == 2);
    for(size_t i = 0; i < def.parts.size(); i++) {
        CHECK(def.parts[i].attach == AttachMode::Down);
        CHECK(near(def.parts[i].angle, 0.0));
        CHECK(near(def.parts[i].offset, 0.0));
        CHECK(def.parts[i].stage == 1);
    }
    // no "controller" in the file -> defaults to the first reaction wheel
    // (the capsule carries a small one)
    CHECK(def.controllerIndex() == 0);

    // the aggregates the Vehicle derives from the same data (field-driven,
    // mirroring Vehicle::init)
    double mass = 0, thrust = 0, torque = 0, fuel = 0;
    for(size_t i = 0; i < def.parts.size(); i++) {
        const PartDef *d = def.parts[i].def;
        mass += d->mass;
        if(d->fuel_rate > 0.0 && d->exhaust_velocity > 0.0) { thrust += d->fullThrust(); }
        if(d->torque > 0.0) { torque += d->torque; }
        for(size_t r = 0; r < d->capacity.size(); r++) { fuel += d->capacity[r]; }
    }
    CHECK(mass > 0.0);
    CHECK(near(thrust, eng->fullThrust())); // only the engine thrusts
    CHECK(near(torque, cap->torque + rw->torque)); // capsule wheel + reaction wheel
    CHECK(near(fuel, ft->capacity[(int)ResourceType::Hydrogen]
                + ft->capacity[(int)ResourceType::LOX])); // only the tank holds fuel

    // the transporter: same as racer but with two fuel tanks (double the fuel)
    ShipDef ex = load_ship_def("res/ships/transporter.json", cat);
    CHECK(ex.name == "transporter");
    CHECK(ex.parts.size() == 5);
    {
        int tanks = 0;
        double ex_mass = 0, ex_fuel = 0, ex_torque = 0;
        for(size_t i = 0; i < ex.parts.size(); i++) {
            const PartDef *d = ex.parts[i].def;
            ex_mass += d->mass;
            if(d->torque > 0.0) { ex_torque += d->torque; }
            bool has_cap = false;
            for(size_t r = 0; r < d->capacity.size(); r++) {
                ex_fuel += d->capacity[r];
                if(d->capacity[r] > 0.0f) { has_cap = true; }
            }
            if(has_cap) { tanks++; }
        }
        CHECK(tanks == 2);
        // capsule + wheel + engine + two tanks
        CHECK(near(ex_mass, cap->mass + rw->mass + eng->mass + 2.0 * ft->mass));
        CHECK(near(ex_fuel, 2.0 * (ft->capacity[(int)ResourceType::Hydrogen]
                                   + ft->capacity[(int)ResourceType::LOX])));
        CHECK(near(ex_torque, cap->torque + rw->torque)); // same capsule + wheel as basic
        // the two tanks get distinct auto ids
        CHECK(ex.parts[2].id == "fuel_tank_1");
        CHECK(ex.parts[3].id == "fuel_tank_2");
    }

    // mixed-size ships (self-contained defs; faces touch, geometry derived
    // from the sizes)
    const char *mix = "/tmp/test_shipload_mix.json";
    {
        std::ofstream f(mix);
        f << "{ \"name\": \"big\", "
             "\"parts\": [ { \"part\": \"capsule\" }, "
             " { \"part\": \"tank_r3h2\" }, { \"part\": \"engine_r5h10\" } ] }";
        f.close();
    }
    ShipDef big = load_ship_def(mix, cat);
    CHECK(big.parts.size() == 3);
    CHECK(big.parts[0].def == cap && big.parts[1].def == t32 && big.parts[2].def == e5);
    CHECK(big.controllerIndex() == 0);  // no "controller" -> first reaction wheel (capsule)
    std::remove(mix);

    // no controller + side tanks: the default must be the first reaction
    // wheel, NOT the last part. The old default (last part) made the
    // camera basis ride on a side tank whose local frame is rotated about
    // the nose by its attach angle, so the stick controls read swapped
    // (4 tanks, 270 deg) or mixed (3 tanks, 240 deg) on screen.
    {
        std::ofstream f(mix);
        f << "{ \"name\": \"side\", "
             "\"parts\": [ { \"part\": \"capsule\" }, "
             " { \"part\": \"fuel_tank\" }, { \"part\": \"engine\" }, "
             " { \"part\": \"tank_r1h3\", \"attach\": \"side\", "
             "   \"parent\": \"engine_1\", \"angle\": 240 }, "
             " { \"part\": \"tank_r1h3\", \"attach\": \"side\", "
             "   \"parent\": \"engine_1\", \"angle\": 120 }, "
             " { \"part\": \"tank_r1h3\", \"attach\": \"side\", "
             "   \"parent\": \"engine_1\", \"angle\": 0 } ] }";
        f.close();
    }
    ShipDef side = load_ship_def(mix, cat);
    CHECK(side.parts.size() == 6);
    CHECK(side.controllerIndex() == 0);   // first wheel, not the last tank
    std::remove(mix);

    {
        std::ofstream f(mix);
        f << "{ \"name\": \"tall\", "
             "\"parts\": [ { \"part\": \"capsule_r3h6\" }, "
             " { \"part\": \"tank_r3h2\" }, { \"part\": \"engine_r5h10\" } ] }";
        f.close();
    }
    ShipDef tall = load_ship_def(mix, cat);
    CHECK(tall.parts.size() == 3);
    CHECK(tall.parts[0].def == c32 && tall.parts[2].def == e5);
    std::remove(mix);

    // the booster: two side pods on opposite sides of a tall core. The pods
    // are small (tank_r1h1) and the core is tall (tank_r5h5) so no two
    // non-welded parts touch -- a valid ship the hull-margin can't destabilize.
    {
        std::ofstream f(mix);
        f << "{ \"name\": \"booster\", \"controller\": \"capsule_1\", "
             "\"parts\": [ { \"part\": \"capsule\", \"id\": \"capsule_1\" }, "
             " { \"part\": \"tank_r5h5\", \"id\": \"core\" }, "
             " { \"part\": \"engine\", \"id\": \"eng\" }, "
             " { \"part\": \"tank_r1h1\", \"id\": \"p1\", \"attach\": \"side\", "
             "   \"parent\": \"core\", \"angle\": 0 }, "
             " { \"part\": \"tank_r1h1\", \"id\": \"p2\", \"attach\": \"side\", "
             "   \"parent\": \"core\", \"angle\": 180 } ] }";
        f.close();
    }
    ShipDef bo = load_ship_def(mix, cat);
    CHECK(bo.parts.size() == 5);
    CHECK(bo.controllerIndex() == 0);   // "controller": "capsule_1"
    {
        const ShipPart &side = bo.parts[3];   // tank_r1h1, side of the tall tank
        CHECK(side.def == t11);
        CHECK(side.attach == AttachMode::Side);
        CHECK(side.parent == 1);             // the tank_r5h5 core
        CHECK(near(side.angle, 0.0));
        const ShipPart &opp = bo.parts[4];   // tank_r1h1, other side at 180 deg
        CHECK(opp.def == t11);
        CHECK(opp.attach == AttachMode::Side);
        CHECK(opp.parent == 1);              // the tank_r5h5 core
        CHECK(near(opp.angle, 180.0));
    }
    std::remove(mix);

    // tanker (shipped def): core tank + engine below, four side pods
    // (tank_r3h3) around the core at 0/90/180/270
    const PartDef *t33 = cat.find("tank_r3h3");
    CHECK(t33 != nullptr);
    ShipDef tk = load_ship_def("res/ships/tanker.json", cat);
    CHECK(tk.name == "tanker");
    CHECK(tk.parts.size() == 7);
    CHECK(tk.controllerIndex() == 0);   // "controller": "capsule_1"
    CHECK(tk.parts[0].def == cap && tk.parts[1].def == t33 && tk.parts[2].def == eng);
    {
        const double podAngles[4] = { 0.0, 90.0, 180.0, 270.0 };
        for(int i = 0; i < 4; i++) {
            const ShipPart &pod = tk.parts[3 + (size_t)i];
            CHECK(pod.def == t33);
            CHECK(pod.attach == AttachMode::Side);
            CHECK(pod.parent == 1);
            CHECK(near(pod.angle, podAngles[i]));
        }
    }

    // laythe_explorer (shipped def): the same shape as the tanker, but the
    // pods are small (tank_r1h1)
    ShipDef lx = load_ship_def("res/ships/laythe_explorer.json", cat);
    CHECK(lx.name == "laythe_explorer");
    CHECK(lx.parts.size() == 7);
    CHECK(lx.controllerIndex() == 0);
    CHECK(lx.parts[1].def == t11 && lx.parts[2].def == eng);
    {
        const double podAngles[4] = { 0.0, 90.0, 180.0, 270.0 };
        for(int i = 0; i < 4; i++) {
            const ShipPart &pod = lx.parts[3 + (size_t)i];
            CHECK(pod.def == t11);
            CHECK(pod.attach == AttachMode::Side);
            CHECK(pod.parent == 1);
            CHECK(near(pod.angle, podAngles[i]));
        }
    }

    // nose cap: "up" on the tank's top face -- the cap's base face sits on
    // the tank's top face, the apex points up along the shared axis
    {
        std::ofstream f(mix);
        f << "{ \"name\": \"capped\", "
             "\"parts\": [ { \"part\": \"tank_r3h3\", \"id\": \"tank\" }, "
             " { \"part\": \"nose_cap_r3h1.5\", \"id\": \"cap\", "
             "   \"attach\": \"up\", \"parent\": \"tank\" } ] }";
        f.close();
    }
    ShipDef cd = load_ship_def(mix, cat);
    CHECK(cd.parts.size() == 2);
    CHECK(cd.parts[1].def == nc3);
    CHECK(cd.parts[1].attach == AttachMode::Up);
    CHECK(cd.parts[1].parent == 0);
    {
        AttachPose p = attachPose(glm::dvec3(0.0), glm::dmat3(1.0), *t33, *nc3,
                                  AttachMode::Up, 0.0, 0.0);
        CHECK(near(p.childPos.z, (3.0 + 1.5) / 2.0));
        CHECK(near((p.childRot * glm::dvec3(0.0, 0.0, 1.0)).z, 1.0));
        // the cap's base face (local -h/2) rests on the tank's top face (+1.5)
        CHECK(near(p.childPos.z + (p.childRot * glm::dvec3(0.0, 0.0, -0.75)).z, 1.5));
        // the apex lands at tank top + cap height
        CHECK(near(p.childPos.z + (p.childRot * glm::dvec3(0.0, 0.0, 0.75)).z, 3.0));
        const glm::dvec3 wp = p.parentAnchor;
        const glm::dvec3 wc = p.childPos + p.childRot * p.childAnchor;
        CHECK(glm::length(wp - wc) < 1e-9);
    }
    std::remove(mix);

    // --- attachPose geometry ---------------------------------------------
    // All cases: parent at the origin, identity orientation, unless noted.
    const glm::dvec3 O(0.0, 0.0, 0.0);
    const glm::dmat3 I(1.0);

    // DOWN: child below the parent (shared axis), faces touching
    {
        AttachPose p = attachPose(O, I, *cap, *eng, AttachMode::Down, 0.0, 0.0);
        CHECK(vnear(p.childPos, glm::dvec3(0, 0, -2.0)));   // (2+2)/2 below
        CHECK(mnear(p.childRot, I));
        CHECK(vnear(p.parentAnchor, glm::dvec3(0, 0, -1.0)));
        CHECK(vnear(p.childAnchor,  glm::dvec3(0, 0,  1.0)));
        CHECK(coincide(p, O, I));
    }
    // DOWN with a 0.5 m spacer gap (anchors sit at the gap edge)
    {
        AttachPose p = attachPose(O, I, *cap, *eng, AttachMode::Down, 0.0, 0.5);
        CHECK(vnear(p.childPos, glm::dvec3(0, 0, -2.5)));
        CHECK(vnear(p.parentAnchor, glm::dvec3(0, 0, -1.5)));
        CHECK(vnear(p.childAnchor,  glm::dvec3(0, 0,  1.0)));
        CHECK(coincide(p, O, I));
    }
    // UP: child above the parent (shared axis) -- stacking outward
    {
        AttachPose p = attachPose(O, I, *cap, *eng, AttachMode::Up, 0.0, 0.0);
        CHECK(vnear(p.childPos, glm::dvec3(0, 0, 2.0)));
        CHECK(mnear(p.childRot, I));
        CHECK(vnear(p.parentAnchor, glm::dvec3(0, 0,  1.0)));
        CHECK(vnear(p.childAnchor,  glm::dvec3(0, 0, -1.0)));
        CHECK(coincide(p, O, I));
    }
    // RADIAL at 0 deg: child axis -> parent +X, base face on the parent side
    {
        AttachPose p = attachPose(O, I, *cap, *t32, AttachMode::Radial, 0.0, 0.0);
        // child center is r_parent + h_child/2 = 1 + 1 = 2 along +X
        CHECK(vnear(p.childPos, glm::dvec3(2.0, 0.0, 0.0)));
        // child +Z points along parent +X
        CHECK(vnear(p.childRot * glm::dvec3(0, 0, 1.0), glm::dvec3(1.0, 0.0, 0.0)));
        CHECK(vnear(p.parentAnchor, glm::dvec3(1.0, 0.0, 0.0)));
        CHECK(vnear(p.childAnchor,  glm::dvec3(0.0, 0.0, -1.0)));
        CHECK(coincide(p, O, I));
    }
    // RADIAL at 90 deg: child at the parent's +Y side
    {
        AttachPose p = attachPose(O, I, *cap, *t32, AttachMode::Radial, 90.0, 0.0);
        CHECK(vnear(p.childPos, glm::dvec3(0.0, 2.0, 0.0)));
        CHECK(vnear(p.parentAnchor, glm::dvec3(0.0, 1.0, 0.0)));
        CHECK(coincide(p, O, I));
    }
    // RADIAL at 180 deg + 0.5 gap: child at the parent's -X side, 0.5 out
    {
        AttachPose p = attachPose(O, I, *cap, *t32, AttachMode::Radial, 180.0, 0.5);
        CHECK(vnear(p.childPos, glm::dvec3(-2.5, 0.0, 0.0)));
        CHECK(vnear(p.parentAnchor, glm::dvec3(-1.5, 0.0, 0.0)));
        CHECK(vnear(p.childAnchor,  glm::dvec3(0.0, 0.0, -1.0)));
        CHECK(coincide(p, O, I));
    }
    // SIDE at 0 deg: parallel axes, side by side along +X
    {
        AttachPose p = attachPose(O, I, *cap, *t32, AttachMode::Side, 0.0, 0.0);
        CHECK(vnear(p.childPos, glm::dvec3(4.0, 0.0, 0.0)));   // r_p + r_c = 1+3
        CHECK(mnear(p.childRot, I));                            // axis stays parallel
        CHECK(vnear(p.parentAnchor, glm::dvec3(1.0, 0.0, 0.0)));
        CHECK(vnear(p.childAnchor,  glm::dvec3(-3.0, 0.0, 0.0)));
        CHECK(coincide(p, O, I));
    }
    // SIDE at 180 deg: the other side, still parallel
    {
        AttachPose p = attachPose(O, I, *cap, *t32, AttachMode::Side, 180.0, 0.0);
        CHECK(vnear(p.childPos, glm::dvec3(-4.0, 0.0, 0.0)));
        CHECK(vnear(p.parentAnchor, glm::dvec3(-1.0, 0.0, 0.0)));
        CHECK(coincide(p, O, I));
    }
    // a non-trivial parent frame: the same relations hold in the parent's frame
    {
        const glm::dmat3 rot = testOrient();
        const glm::dvec3 pp(3.0, -1.0, 7.0);
        AttachPose p = attachPose(pp, rot, *cap, *t32, AttachMode::Radial, 45.0, 0.25);
        CHECK(coincide(p, pp, rot));
        // child offset from the parent is along the rotated attach direction
        const double d = glm::radians(45.0);
        const glm::dvec3 dir = rot * glm::dvec3(cos(d), sin(d), 0.0);
        CHECK(vnear(p.childPos - pp, dir * (1.0 + 1.0 + 0.25)));
    }

    // --- error paths ---------------------------------------------------------
    CHECK(expect_throw([](){ load_parts_catalog("res/no_such_file.json"); }));
    CHECK(expect_throw([&](){ load_ship_def("res/no_such_file.json", cat); }));

    // unknown part name in a ship def
    {
        const char *bad = "/tmp/test_shipload_bad.json";
        std::ofstream f(bad);
        f << "{ \"parts\": [ { \"part\": \"warp_drive\" } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_ship_def(bad, cat); }));
        std::remove(bad);
    }

    // duplicate part id
    {
        const char *bad = "/tmp/test_shipload_bad.json";
        std::ofstream f(bad);
        f << "{ \"parts\": [ { \"part\": \"capsule\", \"id\": \"x\" }, "
             "{ \"part\": \"engine\", \"id\": \"x\" } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_ship_def(bad, cat); }));
        std::remove(bad);
    }

    // parent must be defined before the child (also rules out cycles)
    {
        const char *bad = "/tmp/test_shipload_bad.json";
        std::ofstream f(bad);
        f << "{ \"parts\": [ { \"part\": \"capsule\" }, "
             "{ \"part\": \"engine\", \"parent\": \"nope\" } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_ship_def(bad, cat); }));
        std::remove(bad);
    }
    // referencing a part defined LATER is rejected (construction order)
    {
        const char *bad = "/tmp/test_shipload_bad.json";
        std::ofstream f(bad);
        f << "{ \"parts\": [ { \"part\": \"engine\", \"parent\": \"capsule_1\" }, "
             "{ \"part\": \"capsule\" } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_ship_def(bad, cat); }));
        std::remove(bad);
    }

    // a bad attach value
    {
        const char *bad = "/tmp/test_shipload_bad.json";
        std::ofstream f(bad);
        f << "{ \"parts\": [ { \"part\": \"capsule\" }, "
             "{ \"part\": \"engine\", \"attach\": \"diagonal\" } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_ship_def(bad, cat); }));
        std::remove(bad);
    }

    // negative spacer
    {
        const char *bad = "/tmp/test_shipload_bad.json";
        std::ofstream f(bad);
        f << "{ \"parts\": [ { \"part\": \"capsule\" }, "
             "{ \"part\": \"engine\", \"offset\": -1.0 } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_ship_def(bad, cat); }));
        std::remove(bad);
    }

    // stage must be >= 1
    {
        const char *bad = "/tmp/test_shipload_bad.json";
        std::ofstream f(bad);
        f << "{ \"parts\": [ { \"part\": \"capsule\", \"stage\": 0 } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_ship_def(bad, cat); }));
        std::remove(bad);
    }

    // controller must be a known part id
    {
        const char *bad = "/tmp/test_shipload_bad.json";
        std::ofstream f(bad);
        f << "{ \"controller\": \"ghost\", \"parts\": [ { \"part\": \"capsule\" } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_ship_def(bad, cat); }));
        std::remove(bad);
    }

    // a negative ship hull margin must be rejected
    {
        const char *bad = "/tmp/test_shipload_badship.json";
        std::ofstream f(bad);
        f << "{ \"hull_margin\": -1, \"parts\": [ { \"part\": \"capsule\" } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_ship_def(bad, cat); }));
        std::remove(bad);
    }

    // a valid ship: explicit ids, an angle, a spacer, a stage
    {
        const char *ok = "/tmp/test_shipload_ok.json";
        std::ofstream f(ok);
        f << "{ \"name\": \"T\", \"controller\": \"nose\", "
             "\"parts\": [ "
             " { \"part\": \"capsule\", \"id\": \"nose\" }, "
             " { \"part\": \"engine\", \"attach\": \"down\", \"stage\": 1 }, "
             " { \"part\": \"tank_r3h2\", \"attach\": \"radial\", "
             "   \"parent\": \"engine_1\", \"angle\": 30, \"offset\": 0.25 } ] }";
        f.close();
        bool threw = false;
        ShipDef okd;
        try {
            okd = load_ship_def(ok, cat);
        } catch(const std::runtime_error &) {
            threw = true;
        }
        CHECK(!threw);
        if(!threw) {
            CHECK(okd.parts.size() == 3);
            CHECK(okd.controllerIndex() == 0);
            CHECK(okd.parts[2].attach == AttachMode::Radial);
            CHECK(okd.parts[2].parent == 1);
            CHECK(near(okd.parts[2].angle, 30.0));
            CHECK(near(okd.parts[2].offset, 0.25));
        }
        std::remove(ok);
    }

    // ship-level hull margin: applies to the ship's whole layout (the
    // welded-hull overlap problem is layout-dependent, so it lives here,
    // not on the parts); unset -> -1
    {
        const char *hm = "/tmp/test_shipload_shipmargin.json";
        std::ofstream f(hm);
        f << "{ \"name\": \"T\", \"hull_margin\": 0, "
             "\"parts\": [ { \"part\": \"capsule\" } ] }";
        f.close();
        ShipDef hmdef = load_ship_def(hm, cat);
        CHECK(hmdef.hull_margin == 0.0);
        std::remove(hm);
    }
    {
        const char *hm = "/tmp/test_shipload_shipmargin.json";
        std::ofstream f(hm);
        f << "{ \"name\": \"T\", \"parts\": [ { \"part\": \"capsule\" } ] }";
        f.close();
        ShipDef hmdef = load_ship_def(hm, cat);
        CHECK(hmdef.hull_margin == -1.0);
        std::remove(hm);
    }
    // precedence: ship def > catalog; either unset -> the other; both
    // unset -> -1 (the physics engine applies its own default)
    CHECK(resolveHullMargin(0.0, -1.0) == 0.0);
    CHECK(resolveHullMargin(-1.0, 0.0) == 0.0);
    CHECK(resolveHullMargin(0.25, 0.0) == 0.25);
    CHECK(resolveHullMargin(-1.0, -1.0) == -1.0);

    // thruster fields must be given together (fuel_rate without exhaust_velocity)
    {
        const char *bad = "/tmp/test_shipload_badcat.json";
        std::ofstream f(bad);
        f << "{ \"parts\": [ { \"name\": \"x\", \"type\": \"engine\", "
             "\"mesh\": \"a.obj\", \"texture\": \"a.png\", \"mass\": 1.0, "
             "\"fuel_rate\": 1.0 } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_parts_catalog(bad); }));
        std::remove(bad);
    }

    // a capacity must total > 0
    {
        const char *bad = "/tmp/test_shipload_badcat.json";
        std::ofstream f(bad);
        f << "{ \"parts\": [ { \"name\": \"x\", \"type\": \"fuel_tank\", "
             "\"mesh\": \"a.obj\", \"texture\": \"a.png\", \"mass\": 1.0, "
             "\"capacity\": { \"hydrogen\": 0, \"lox\": 0 } } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_parts_catalog(bad); }));
        std::remove(bad);
    }

    // radius/height must be > 0
    {
        const char *bad = "/tmp/test_shipload_badcat.json";
        {
            std::ofstream f(bad);
            f << "{ \"parts\": [ { \"name\": \"x\", \"type\": \"engine\", "
                 "\"mesh\": \"a.obj\", \"texture\": \"a.png\", \"mass\": 1.0, "
                 "\"radius\": 0 } ] }";
        }
        CHECK(expect_throw([&](){ load_parts_catalog(bad); }));
        {
            std::ofstream f(bad);
            f << "{ \"parts\": [ { \"name\": \"x\", \"type\": \"engine\", "
                 "\"mesh\": \"a.obj\", \"texture\": \"a.png\", \"mass\": 1.0, "
                 "\"height\": -1 } ] }";
        }
        CHECK(expect_throw([&](){ load_parts_catalog(bad); }));
        std::remove(bad);
    }

    // hull margin must be >= 0
    {
        const char *bad = "/tmp/test_shipload_badcat.json";
        std::ofstream f(bad);
        f << "{ \"parts\": [ { \"name\": \"x\", \"type\": \"engine\", "
             "\"mesh\": \"a.obj\", \"texture\": \"a.png\", \"mass\": 1.0, "
             "\"hull_margin\": -0.5 } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_parts_catalog(bad); }));
        std::remove(bad);
    }

    if(failures) {
        printf("test_shipload: %d FAILURES\n", failures);
        return 1;
    }
    printf("test_shipload: all tests passed\n");
    return 0;
}
