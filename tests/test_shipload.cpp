// test_shipload: the GL-free ship/part JSON data model (src/shipdef.cpp).
// Runs from the repo root (needs res/):
//   make test   (or: ./test_shipload)
//
// Covers: catalog + ship-def parsing (the parent-relative tree schema:
// ids, parent/attach/angle/offset/stage, construction-order validation),
// the aggregates the Vehicle would compute, the default-controller rule,
// the attachPose geometry (child pose across mode/angle/offset), and the
// error paths.

#include "shipdef.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <dirent.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <sys/stat.h>
#include <unistd.h>

#include <glm/gtc/quaternion.hpp>   // angleAxis / mat3_cast (the symmetry test)

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

// a fixed non-trivial parent orientation (+Z pointing at world (1,1,1))
static glm::dmat3 testOrient() {
    const glm::dvec3 z = glm::normalize(glm::dvec3(1.0, 1.0, 1.0));
    const glm::dvec3 x = glm::normalize(glm::cross(glm::dvec3(0.0, 1.0, 0.0), z));
    const glm::dvec3 y = glm::cross(z, x);
    return glm::dmat3(x, y, z);
}

int main() {
    // --- parts catalog ----------------------------------------------------
    PartsCatalog cat = load_parts_catalog("res/data/parts.json");
    // floor, not an exact count: the catalog grows as parts are added
    CHECK(cat.parts.size() >= 35);
    CHECK(cat.find("nope") == nullptr);

    // controlAxisParams: the pure axis -> (moment axis, force plane, stick,
    // target sign) selection (I1). The moment axis MUST equal the reaction
    // wheel's axis for that control, and the target sign the wheel's torque
    // sign on a positive stick (pitch -> -right, yaw -> -up, roll -> +nose).
    // Pinning it here means a swap (about, targetSign) can't slip through.
    {
        const ControlAxisParams pitch = controlAxisParams(ControlAxis::Pitch);
        CHECK(pitch.aboutAxis == 0);        // moment about RIGHT (wheel: -right)
        CHECK(pitch.forceDirKind == 0);     // force in the up plane (liftDir)
        CHECK(pitch.stickIndex == 1);       // driven by W/S (stick[1])
        CHECK(pitch.targetSign == -1.0);    // wheel W/S -> -right
        const ControlAxisParams yaw = controlAxisParams(ControlAxis::Yaw);
        CHECK(yaw.aboutAxis == 1);          // moment about UP (wheel: -up)
        CHECK(yaw.forceDirKind == 1);       // force in the right plane (yawDir)
        CHECK(yaw.stickIndex == 2);         // driven by A/D (stick[2])
        CHECK(yaw.targetSign == -1.0);      // wheel A/D -> -up
        const ControlAxisParams roll = controlAxisParams(ControlAxis::Roll);
        CHECK(roll.aboutAxis == 2);         // moment about NOSE (wheel: +nose)
        CHECK(roll.forceDirKind == 0);      // force in the up plane (liftDir)
        CHECK(roll.stickIndex == 0);        // driven by Q/E (stick[0])
        CHECK(roll.targetSign == +1.0);     // wheel Q/E -> +nose (the +1!)
        // the three axes are distinct: different moment axis AND stick
        CHECK(pitch.aboutAxis != yaw.aboutAxis
              && yaw.aboutAxis != roll.aboutAxis
              && roll.aboutAxis != pitch.aboutAxis);
        CHECK(pitch.stickIndex != yaw.stickIndex
              && yaw.stickIndex != roll.stickIndex
              && roll.stickIndex != pitch.stickIndex);
    }

    const PartDef *cap = cat.find("capsule");
    const PartDef *rw  = cat.find("reaction_wheel");
    const PartDef *eng = cat.find("engine");
    const PartDef *ft  = cat.find("fuel_tank");
    CHECK(cap != nullptr && rw != nullptr && eng != nullptr && ft != nullptr);

    // the EVA kerbal: a crew-mass part with no ship behaviors (no wheel or
    // thruster) but a small hydrazine tank for its RCS suit -- the mass is
    // the DRY suit (the propellant rides capacity / effectiveMass, like the
    // tanks above, so a spent suit weighs the same).
    const PartDef *kb = cat.find("kerbal");
    CHECK(kb != nullptr);
    CHECK(kb->type == "kerbal");
    CHECK(kb->mass > 50.0 && kb->mass < 150.0);
    CHECK(kb->torque == 0.0 && kb->fuel_rate == 0.0);
    CHECK(kb->capacity[(int)ResourceType::Hydrazine] > 0.0f); // the suit's RCS propellant
    CHECK(kb->capacity[(int)ResourceType::Hydrazine] < 100.0f); // a suit load, not a tank
    CHECK(kb->mass > kb->capacity[(int)ResourceType::Hydrazine]); // dry suit outweighs its propellant load
    CHECK(kb->inventory_capacity > 0); // the suit pocket (phase 4.2)

    // a cargo crate: a container (inventory_capacity > 0), no other behavior
    const PartDef *cg = cat.find("cargo");
    CHECK(cg != nullptr);
    CHECK(cg->inventory_capacity > 0);
    CHECK(cg->crew_capacity == 0);
    CHECK(cg->torque == 0.0 && cg->fuel_rate == 0.0);

    // decouplers are fuel barriers: propellant does not flow across one, so
    // it splits fuel groups (an engine can't draw from the other side).
    const PartDef *dc = cat.find("decoupler_r1");
    CHECK(dc != nullptr && dc->decoupler && dc->fuel_barrier);

    // docking ports: a connection point (end face that locks to another
    // port). They are fuel barriers (a boundary between two ships' fuel
    // systems) and carry NO other behavior -- no thrust, no wheel, no tank,
    // not a decoupler. One per standard radius, reusing the decoupler geometry.
    const PartDef *dp1   = cat.find("docking_port_r1");
    const PartDef *dp15  = cat.find("docking_port_r1.5");
    const PartDef *dp225 = cat.find("docking_port_r2.25");
    CHECK(dp1 != nullptr && dp15 != nullptr && dp225 != nullptr);
    for(const PartDef *dp : { dp1, dp15, dp225 }) {
        CHECK(dp->docking_port);
        CHECK(dp->fuel_barrier);        // forced by the parser, like a decoupler
        CHECK(!dp->decoupler);
        CHECK(dp->mass > 0.0);
        CHECK(dp->torque == 0.0);
        CHECK(dp->fuel_rate == 0.0 && dp->exhaust_velocity == 0.0);
        for(size_t r = 0; r < dp->capacity.size(); r++) {
            CHECK(dp->capacity[r] == 0.0f);
        }
    }
    // sizes track the decoupler sizes (same geometry)
    CHECK(dp1->radius == dc->radius && dp1->height == dc->height);
    CHECK(dp15->radius == 1.5 && dp15->height == 0.375);
    CHECK(dp225->radius == 2.25 && dp225->height == 0.5625);

    // fuel link: a virtual (no-mesh) one-way fuel connection. It is a marker
    // entry -- no geometry, no mass (the catalog parser skips the validation
    // for it). The endpoints come from the ship def (from/to), not the catalog.
    const PartDef *fl = cat.find("fuel_link");
    CHECK(fl != nullptr);
    CHECK(fl->fuel_link);
    CHECK(fl->mesh.empty() && fl->texture.empty());
    CHECK(fl->mass == 0.0);
    CHECK(fl->torque == 0.0 && fl->fuel_rate == 0.0);
    for(size_t r = 0; r < fl->capacity.size(); r++) {
        CHECK(fl->capacity[r] == 0.0f);
    }

    // display_name: a human-readable label (the `name` is a machine id).
    // Display-only, so pin that it is present and distinct from the id, not
    // the exact wording (that is owned by gen_parts.py).
    CHECK(!cap->display_name.empty());
    CHECK(cap->display_name != cap->name);
    CHECK(!eng->display_name.empty());
    CHECK(!fl->display_name.empty());
    CHECK(fl->display_name != fl->name);

    // pre-size parts default to the legacy 2 m cube (radius 1, height 2)
    CHECK(cap->radius == 1.0 && cap->height == 2.0);
    CHECK(eng->radius == 1.0 && eng->height == 2.0);

    // type is a free-form label; behavior comes from the fields. The exact
    // numbers are owned by gen_parts.py, so pin the invariants, not the values.
    CHECK(cap->type == "capsule");
    CHECK(cap->mass > 0.0);
    CHECK(cap->mesh == "meshes/capsule.obj");
    CHECK(cap->texture == "textures/capsule.png");
    CHECK(cap->torque > 0.0);   // the capsule carries an attitude wheel
    CHECK(cap->fuel_rate == 0.0 && cap->exhaust_velocity == 0.0);
    // the capsule is a crew module: a CONSTANT life-support draw (on all the
    // time, unlike a wheel's active power_draw) plus a small built-in battery
    // (EC capacity) as the reserve. It is not an active draw and not a source.
    CHECK(cap->power_draw_constant > 0.0);            // life support, always on
    CHECK(cap->capacity[(int)ResourceType::EC] > 0.0f); // built-in battery
    CHECK(cap->power_draw == 0.0);                    // not an active draw
    CHECK(cap->power_gen == 0.0);                     // not a source

    CHECK(rw->type == "reaction_wheel");
    CHECK(rw->mass > 0.0);
    CHECK(rw->torque > 0.0);
    // the wheel is a thin disc: height = 25% of the radius
    CHECK(rw->radius == 1.0 && rw->height == 0.25);
    CHECK(rw->mesh == "meshes/reaction_wheel_r1h0.25.obj");
    // the wheel draws EC while active (the power budget) but generates none
    CHECK(rw->power_draw > 0.0);
    CHECK(rw->power_gen == 0.0);

    // battery: EC STORAGE (capacity[EC] > 0), no power draw or gen. The mass
    // includes the cells (like a tank includes propellant).
    const PartDef *bat = cat.find("battery");
    CHECK(bat != nullptr);
    CHECK(bat->type == "battery");
    CHECK(bat->mass > 0.0);
    CHECK(bat->capacity[(int)ResourceType::EC] > 0.0f);
    CHECK(bat->power_draw == 0.0 && bat->power_gen == 0.0);
    CHECK(bat->torque == 0.0);
    // the other resource slots stay empty (it's an EC store, not a fuel tank)
    CHECK(bat->capacity[(int)ResourceType::Hydrogen] == 0.0f);
    CHECK(bat->capacity[(int)ResourceType::LOX] == 0.0f);

    // rtg: a constant EC source (power_gen > 0), no draw, no EC storage.
    const PartDef *rtg = cat.find("rtg");
    CHECK(rtg != nullptr);
    CHECK(rtg->type == "rtg");
    CHECK(rtg->mass > 0.0);
    CHECK(rtg->power_gen > 0.0);
    CHECK(rtg->power_draw == 0.0);
    CHECK(rtg->capacity[(int)ResourceType::EC] == 0.0f);
    CHECK(rtg->torque == 0.0);

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
    CHECK(ft->mesh == "meshes/fuel_tank.obj");
    CHECK(ft->texture == "textures/fuel_tank.png");
    CHECK(ft->torque == 0.0);
    CHECK(ft->fuel_rate == 0.0 && ft->exhaust_velocity == 0.0);
    CHECK(ft->capacity[(int)ResourceType::Hydrogen] > 0.0f);
    CHECK(ft->capacity[(int)ResourceType::LOX] > 0.0f);
    // the mass is the DRY structure only; the propellant rides
    // resources.current / effectiveMass, not mass. Structure is lighter
    // than the fuel it holds (density ratio ~1:10), so mass < capacity.
    CHECK(ft->mass < ft->capacity[(int)ResourceType::Hydrogen]
                      + ft->capacity[(int)ResourceType::LOX]);

    // jet (air-breathing): an AIR-BREATHING thruster that burns a SEPARATE
    // fuel type from rocket H2 -- so it and a rocket on the same ship never
    // share a propellant pool, and its fuel gives no delta-v (no onboard
    // oxidizer). Phase 1 added the engine; Phase 2 separated its fuel.
    const PartDef *jet   = cat.find("jet");
    const PartDef *jtank = cat.find("jet_tank_r1h3");
    CHECK(jet != nullptr && jtank != nullptr);
    CHECK(jet->jet && jet->fuel_rate > 0.0 && jet->exhaust_velocity > 0.0);
    CHECK(jet->jet_fan_thrust > 0.0 && jet->jet_intake_area > 0.0);
    // the jet engine is the pump, not a reservoir: no propellant of its own
    for(size_t r = 0; r < jet->capacity.size(); r++) {
        CHECK(jet->capacity[r] == 0.0f);
    }
    // the jet-fuel tank is a reservoir of the NEW JetFuel resource -- and
    // ONLY that (the Phase 2 separation: not rocket H2/LOX, so it is
    // excluded from delta-v and never feeds a rocket engine).
    CHECK(jtank->type == "jet_tank");
    CHECK(jtank->fuel_rate == 0.0 && jtank->exhaust_velocity == 0.0);
    CHECK(jtank->capacity[(int)ResourceType::JetFuel] > 0.0f);
    CHECK(jtank->capacity[(int)ResourceType::Hydrogen] == 0.0f);
    CHECK(jtank->capacity[(int)ResourceType::LOX] == 0.0f);
    CHECK(jtank->mass < jtank->capacity[(int)ResourceType::JetFuel]);

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
    const PartDef *e225 = cat.find("engine_r2.25h4.5");
    const PartDef *e15  = cat.find("engine_r1.5h3");
    const PartDef *t152 = cat.find("tank_r1.5h2");
    const PartDef *t11  = cat.find("tank_r1h1");
    const PartDef *t2255 = cat.find("tank_r2.25h5");
    const PartDef *c153 = cat.find("capsule_r1.5h3");
    CHECK(e225 != nullptr && e15 != nullptr && t152 != nullptr
          && t11 != nullptr && t2255 != nullptr && c153 != nullptr);
    CHECK(t2255->radius == 2.25 && t2255->height == 5.0);
    CHECK(e225->radius == 2.25 && e225->height == 4.5);
    CHECK(e15->radius == 1.5 && e15->height == 3.0);
    // bigger = better: thrust, capacity and torque all scale with the size
    CHECK(e225->fullThrust() > e15->fullThrust() && e15->fullThrust() > eng->fullThrust());
    CHECK(t152->radius == 1.5 && t152->height == 2.0);
    CHECK(t11->radius == 1.0 && t11->height == 1.0);
    CHECK(t152->capacity[(int)ResourceType::Hydrogen]
          > t11->capacity[(int)ResourceType::Hydrogen]);
    CHECK(c153->radius == 1.5 && c153->height == 3.0 && c153->torque > cap->torque);
    // the larger capsule shelters more crew: more constant draw + a bigger battery
    CHECK(c153->power_draw_constant > cap->power_draw_constant);
    CHECK(c153->capacity[(int)ResourceType::EC] > cap->capacity[(int)ResourceType::EC]);

    // nose caps: a simple pointy cone on a tank's top face, one per tank
    // radius; height = radius / 2; passive (no behavior fields at all)
    const PartDef *nc1   = cat.find("nose_cap");
    const PartDef *nc15  = cat.find("nose_cap_r1.5h0.75");
    const PartDef *nc225 = cat.find("nose_cap_r2.25h1.125");
    CHECK(nc1 != nullptr && nc15 != nullptr && nc225 != nullptr);
    CHECK(nc1->radius == 1.0 && nc1->height == 0.5);
    CHECK(nc15->radius == 1.5 && nc15->height == 0.75);
    CHECK(nc225->radius == 2.25 && nc225->height == 1.125);
    for(const PartDef *nc : { nc1, nc15, nc225 }) {
        CHECK(near(nc->height, nc->radius / 2.0));
        CHECK(nc->mass > 0.0);
        CHECK(nc->type == "nose_cap");
        CHECK(nc->torque == 0.0);
        CHECK(nc->fuel_rate == 0.0 && nc->exhaust_velocity == 0.0);
        for(size_t r = 0; r < nc->capacity.size(); r++) {
            CHECK(nc->capacity[r] == 0.0f);
        }
    }
    CHECK(nc225->mesh == "meshes/nose_cap_r2.25h1.125.obj");
    CHECK(nc225->texture == "textures/nose_cap.png");

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
        // propellant only (H2/LOX) -- the capsule's built-in EC battery is
        // charge, not propellant
        fuel += d->capacity[(int)ResourceType::Hydrogen]
              + d->capacity[(int)ResourceType::LOX];
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
            // propellant only (H2/LOX): the capsule's built-in EC battery is
            // charge, not propellant, and is not a propellant tank.
            double cap_prop = d->capacity[(int)ResourceType::Hydrogen]
                            + d->capacity[(int)ResourceType::LOX];
            ex_fuel += cap_prop;
            if(cap_prop > 0.0) { tanks++; }
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
             " { \"part\": \"tank_r1.5h2\" }, { \"part\": \"engine_r2.25h4.5\" } ] }";
        f.close();
    }
    ShipDef big = load_ship_def(mix, cat);
    CHECK(big.parts.size() == 3);
    CHECK(big.parts[0].def == cap && big.parts[1].def == t152 && big.parts[2].def == e225);
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
             " { \"part\": \"tank_r1h3\", \"attach\": \"surface\", "
             "   \"parent\": \"engine_1\", \"angle\": 240 }, "
             " { \"part\": \"tank_r1h3\", \"attach\": \"surface\", "
             "   \"parent\": \"engine_1\", \"angle\": 120 }, "
             " { \"part\": \"tank_r1h3\", \"attach\": \"surface\", "
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
             "\"parts\": [ { \"part\": \"capsule_r1.5h3\" }, "
             " { \"part\": \"tank_r1.5h2\" }, { \"part\": \"engine_r2.25h4.5\" } ] }";
        f.close();
    }
    ShipDef tall = load_ship_def(mix, cat);
    CHECK(tall.parts.size() == 3);
    CHECK(tall.parts[0].def == c153 && tall.parts[2].def == e225);
    std::remove(mix);

    // the booster: two surface pods on opposite sides of a tall core. The pods
    // are small (tank_r1h1) and the core is tall (tank_r2.25h5) so no two
    // non-welded parts touch -- a valid ship the hull-margin can't destabilize.
    {
        std::ofstream f(mix);
        f << "{ \"name\": \"booster\", \"controller\": \"capsule_1\", "
             "\"parts\": [ { \"part\": \"capsule\", \"id\": \"capsule_1\" }, "
             " { \"part\": \"tank_r2.25h5\", \"id\": \"core\" }, "
             " { \"part\": \"engine\", \"id\": \"eng\" }, "
             " { \"part\": \"tank_r1h1\", \"id\": \"p1\", \"attach\": \"surface\", "
             "   \"parent\": \"core\", \"angle\": 0 }, "
             " { \"part\": \"tank_r1h1\", \"id\": \"p2\", \"attach\": \"surface\", "
             "   \"parent\": \"core\", \"angle\": 180 } ] }";
        f.close();
    }
    ShipDef bo = load_ship_def(mix, cat);
    CHECK(bo.parts.size() == 5);
    CHECK(bo.controllerIndex() == 0);   // "controller": "capsule_1"
    {
        const ShipPart &side = bo.parts[3];   // tank_r1h1, side of the tall tank
        CHECK(side.def == t11);
        CHECK(side.attach == AttachMode::Surface);
        CHECK(side.parent == 1);             // the tank_r2.25h5 core
        CHECK(near(side.angle, 0.0));
        // the cylinder shorthand resolved to a contact on the core's +X side
        CHECK(side.isSurfaceEdge());
        CHECK(vnear(side.contactNormal, glm::dvec3(1, 0, 0)));
        CHECK(vnear(side.contactPoint, glm::dvec3(t2255->radius, 0, 0)));
        CHECK(side.childNode == "srf");      // the synthesized surface node
        const ShipPart &opp = bo.parts[4];   // tank_r1h1, other side at 180 deg
        CHECK(opp.def == t11);
        CHECK(opp.attach == AttachMode::Surface);
        CHECK(opp.parent == 1);              // the tank_r2.25h5 core
        CHECK(near(opp.angle, 180.0));
        CHECK(vnear(opp.contactNormal, glm::dvec3(-1, 0, 0)));
    }
    std::remove(mix);

    // tanker (shipped def): core tank + engine below, four side pods
    // (tank_r1.5h3) around the core at 0/90/180/270
    const PartDef *t153 = cat.find("tank_r1.5h3");
    CHECK(t153 != nullptr);
    ShipDef tk = load_ship_def("res/ships/tanker.json", cat);
    CHECK(tk.name == "tanker");
    CHECK(tk.parts.size() == 7);
    CHECK(tk.controllerIndex() == 0);   // "controller": "capsule_1"
    CHECK(tk.parts[0].def == cap && tk.parts[1].def == t153 && tk.parts[2].def == eng);
    {
        const double podAngles[4] = { 0.0, 90.0, 180.0, 270.0 };
        for(int i = 0; i < 4; i++) {
            const ShipPart &pod = tk.parts[3 + (size_t)i];
            CHECK(pod.def == t153);
            CHECK(pod.attach == AttachMode::Surface);
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
            CHECK(pod.attach == AttachMode::Surface);
            CHECK(pod.parent == 1);
            CHECK(near(pod.angle, podAngles[i]));
        }
    }

    // nose cap: "up" on the tank's top face -- the cap's base face sits on
    // the tank's top face, the apex points up along the shared axis
    {
        std::ofstream f(mix);
        f << "{ \"name\": \"capped\", "
             "\"parts\": [ { \"part\": \"tank_r1.5h3\", \"id\": \"tank\" }, "
             " { \"part\": \"nose_cap_r1.5h0.75\", \"id\": \"cap\", "
             "   \"attach\": \"up\", \"parent\": \"tank\" } ] }";
        f.close();
    }
    ShipDef cd = load_ship_def(mix, cat);
    CHECK(cd.parts.size() == 2);
    CHECK(cd.parts[1].def == nc15);
    CHECK(cd.parts[1].attach == AttachMode::Up);
    CHECK(cd.parts[1].parent == 0);
    {
        AttachPose p = attachPose(glm::dvec3(0.0), glm::dmat3(1.0), *t153, *nc15,
                                  AttachMode::Up, 0.0, 0.0);
        CHECK(near(p.childPos.z, (3.0 + 0.75) / 2.0));
        CHECK(near((p.childRot * glm::dvec3(0.0, 0.0, 1.0)).z, 1.0));
        // the cap's base face (local -h/2) rests on the tank's top face (+1.5)
        CHECK(near(p.childPos.z + (p.childRot * glm::dvec3(0.0, 0.0, -0.375)).z, 1.5));
        // the apex lands at tank top + cap height
        CHECK(near(p.childPos.z + (p.childRot * glm::dvec3(0.0, 0.0, 0.375)).z, 2.25));
    }
    std::remove(mix);

    // --- attachPose geometry ---------------------------------------------
    // All cases: parent at the origin, identity orientation, unless noted.
    // The numeric childPos/childRot expectations ARE the contract: they pin
    // the exact solved pose per mode/angle/offset (face contact is implied by
    // childPos plus the part dimensions).
    const glm::dvec3 O(0.0, 0.0, 0.0);
    const glm::dmat3 I(1.0);

    // DOWN: child below the parent (shared axis), faces touching
    {
        AttachPose p = attachPose(O, I, *cap, *eng, AttachMode::Down, 0.0, 0.0);
        CHECK(vnear(p.childPos, glm::dvec3(0, 0, -2.0)));   // (2+2)/2 below
        CHECK(mnear(p.childRot, I));
    }
    // DOWN with a 0.5 m spacer gap
    {
        AttachPose p = attachPose(O, I, *cap, *eng, AttachMode::Down, 0.0, 0.5);
        CHECK(vnear(p.childPos, glm::dvec3(0, 0, -2.5)));
    }
    // UP: child above the parent (shared axis) -- stacking outward
    {
        AttachPose p = attachPose(O, I, *cap, *eng, AttachMode::Up, 0.0, 0.0);
        CHECK(vnear(p.childPos, glm::dvec3(0, 0, 2.0)));
        CHECK(mnear(p.childRot, I));
    }
    // SURFACE at clock 0: parallel axes, side by side along +X. This is the
    // old procedural Side, now expressed as surface attach -- the child's
    // synthesized surface node (-rC,0,0) lands on the parent's +X contact.
    {
        const Node *cn = t152->findSurfaceNode();
        CHECK(cn != nullptr);
        AttachPose p = attachSurface(O, I, glm::dvec3(cap->radius, 0, 0),
                                     glm::dvec3(1, 0, 0), *cn, 0.0, 0.0);
        CHECK(vnear(p.childPos, glm::dvec3(2.5, 0.0, 0.0)));   // r_p + r_c = 1+1.5
        CHECK(mnear(p.childRot, I));                            // axis stays parallel
    }
    // SURFACE at clock 180: the other side, axis still parallel (rolled 180)
    {
        const Node *cn = t152->findSurfaceNode();
        AttachPose p = attachSurface(O, I, glm::dvec3(-cap->radius, 0, 0),
                                     glm::dvec3(-1, 0, 0), *cn, 0.0, 0.0);
        CHECK(vnear(p.childPos, glm::dvec3(-2.5, 0.0, 0.0)));
        CHECK(near((p.childRot * glm::dvec3(0, 0, 1.0)).z, 1.0));
    }
    // a non-trivial parent frame: surface attach still lands the child's
    // surface node on the contact and opposes the parent's outward normal
    {
        const glm::dmat3 rot = testOrient();
        const glm::dvec3 pp(3.0, -1.0, 7.0);
        const Node *cn = t152->findSurfaceNode();
        const glm::dvec3 pt(cap->radius, 0.0, 0.0), nrm(1.0, 0.0, 0.0); // parent-local
        AttachPose p = attachSurface(pp, rot, pt, nrm, *cn, 0.0, 0.0);
        CHECK(vnear(p.childPos + p.childRot * cn->pos, pp + rot * pt));   // coincide
        CHECK(vnear(p.childRot * cn->dir, -(rot * nrm)));                 // opposed
    }

    // --- node schema: synthesis, attachNodes, node-ref parsing -------------
    // a catalog part with no explicit nodes gets synthesized axial top/bottom
    {
        const Node *top = cap->findNode("top");
        const Node *bot = cap->findNode("bottom");
        CHECK(top != nullptr && bot != nullptr);
        CHECK(vnear(top->pos, glm::dvec3(0, 0,  cap->height / 2.0)));
        CHECK(vnear(top->dir, glm::dvec3(0, 0, 1)));
        CHECK(vnear(bot->pos, glm::dvec3(0, 0, -cap->height / 2.0)));
        CHECK(vnear(bot->dir, glm::dvec3(0, 0, -1)));
    }
    // attachNodes on the synthesized axial nodes reproduces attachPose(Down)
    {
        const Node *pb = cap->findNode("bottom");
        const Node *ct = eng->findNode("top");
        CHECK(pb != nullptr && ct != nullptr);
        AttachPose vn = attachNodes(O, I, *pb, *ct, 0.0, 0.0);
        AttachPose vm = attachPose(O, I, *cap, *eng, AttachMode::Down, 0.0, 0.0);
        CHECK(vnear(vn.childPos, vm.childPos));
        CHECK(mnear(vn.childRot, vm.childRot));
    }
    // roll spins the child about the mating axis without moving the node point
    {
        Node pn; pn.id = "bottom"; pn.pos = glm::dvec3(0, 0, -1); pn.dir = glm::dvec3(0, 0, -1);
        Node cn; cn.id = "top";    cn.pos = glm::dvec3(0, 0,  1); cn.dir = glm::dvec3(0, 0,  1);
        AttachPose r0  = attachNodes(O, I, pn, cn, 0.0, 0.0);
        AttachPose r90 = attachNodes(O, I, pn, cn, 90.0, 0.0);
        CHECK(mnear(r0.childRot, I));
        CHECK(vnear(r90.childRot * glm::dvec3(1, 0, 0), glm::dvec3(0, 1, 0)));
        CHECK(vnear(r0.childPos, r90.childPos));
    }
    // non-axial mating: a +X port taking a -Z stack node turns the child's
    // axis onto +X (the hub case synthesis can't express)
    {
        Node pn; pn.id = "port";   pn.pos = glm::dvec3(1, 0, 0);  pn.dir = glm::dvec3(1, 0, 0);
        Node cn; cn.id = "bottom"; cn.pos = glm::dvec3(0, 0, -1); cn.dir = glm::dvec3(0, 0, -1);
        AttachPose p = attachNodes(O, I, pn, cn, 0.0, 0.0);
        CHECK(vnear(p.childRot * glm::dvec3(0, 0, 1), glm::dvec3(1, 0, 0)));
        CHECK(vnear(p.childPos + p.childRot * cn.pos, pn.pos));  // nodes coincide
    }
    // a ship-def stack edge with explicit node refs resolves and mates them
    {
        std::ofstream f(mix);
        f << "{ \"name\": \"noderef\", \"parts\": ["
             "{ \"part\": \"tank_r1.5h3\", \"id\": \"t\" },"
             "{ \"part\": \"nose_cap_r1.5h0.75\", \"id\": \"c\", \"parent\": \"t\","
             "  \"parentNode\": \"top\", \"childNode\": \"bottom\" } ] }";
        f.close();
        ShipDef nr = load_ship_def(mix, cat);
        CHECK(nr.parts.size() == 2);
        CHECK(nr.parts[1].isStackEdge());
        CHECK(nr.parts[1].parentNode == "top");
        CHECK(nr.parts[1].childNode == "bottom");
        // explicit parent-top/child-bottom == an Up mating
        AttachPose p = attachNodes(glm::dvec3(0.0), glm::dmat3(1.0),
                                   *t153->findNode("top"), *nc15->findNode("bottom"),
                                   0.0, 0.0);
        AttachPose u = attachPose(glm::dvec3(0.0), glm::dmat3(1.0), *t153, *nc15,
                                  AttachMode::Up, 0.0, 0.0);
        CHECK(vnear(p.childPos, u.childPos));
        CHECK(mnear(p.childRot, u.childRot));
        std::remove(mix);
    }
    // a bad node id is a load error, not a build-time null deref
    {
        std::ofstream f(mix);
        f << "{ \"name\": \"badnode\", \"parts\": ["
             "{ \"part\": \"tank_r1.5h3\", \"id\": \"t\" },"
             "{ \"part\": \"nose_cap_r1.5h0.75\", \"id\": \"c\", \"parent\": \"t\","
             "  \"parentNode\": \"nonexistent\", \"childNode\": \"bottom\" } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_ship_def(mix, cat); }));
        std::remove(mix);
    }
    // a surface edge with an explicit point + normal (parent-local, what the
    // editor's raycast writes) parses straight through
    {
        std::ofstream f(mix);
        f << "{ \"name\": \"srf\", \"parts\": ["
             "{ \"part\": \"tank_r1.5h3\", \"id\": \"t\" },"
             "{ \"part\": \"tank_r1h1\", \"id\": \"p\", \"parent\": \"t\","
             "  \"attach\": \"surface\", \"point\": [1.5, 0, 0.5],"
             "  \"normal\": [1, 0, 0], \"roll\": 90 } ] }";
        f.close();
        ShipDef sr = load_ship_def(mix, cat);
        const ShipPart &sp = sr.parts[1];
        CHECK(sp.isSurfaceEdge());
        CHECK(vnear(sp.contactPoint, glm::dvec3(1.5, 0, 0.5)));
        CHECK(vnear(sp.contactNormal, glm::dvec3(1, 0, 0)));
        CHECK(near(sp.roll, 90.0));
        CHECK(sp.childNode == "srf");
        std::remove(mix);
    }
    // roll spins the child about the contact normal without moving the contact
    {
        const Node *cn = t152->findSurfaceNode();
        AttachPose a = attachSurface(O, I, glm::dvec3(1, 0, 0), glm::dvec3(1, 0, 0), *cn, 0.0, 0.0);
        AttachPose b = attachSurface(O, I, glm::dvec3(1, 0, 0), glm::dvec3(1, 0, 0), *cn, 90.0, 0.0);
        CHECK(vnear(a.childPos, b.childPos));                 // same contact point
        CHECK(!mnear(a.childRot, b.childRot));                // but spun
        // roll is about the normal (+X): the child's axis stays perpendicular to it
        CHECK(near(glm::dot(b.childRot * glm::dvec3(0, 0, 1), glm::dvec3(1, 0, 0)), 0.0));
    }

    // --- VAB build tree: fromShipDef + recomputePoses ----------------------
    // The physics-free build tree reproduces the flight solver's poses exactly
    // (both go through solveEdge), keeps construction order, and puts the root
    // at identity.
    {
        ShipDef tk2 = load_ship_def("res/ships/tanker.json", cat);
        BuildShip bs = BuildShip::fromShipDef(tk2);
        CHECK(bs.parts.size() == tk2.parts.size());   // tanker has no fuel links
        CHECK(bs.parts[0].parent == -1);
        CHECK(vnear(bs.parts[0].localPos, glm::dvec3(0)));
        CHECK(mnear(bs.parts[0].localRot, glm::dmat3(1.0)));
        // a stack child (capsule -> tank, down) matches attachNodes
        {
            const BuildPart &c = bs.parts[1];
            CHECK(c.attach == AttachMode::Down);
            AttachPose want = attachNodes(bs.parts[0].localPos, bs.parts[0].localRot,
                                          *bs.parts[0].def->findNode("bottom"),
                                          *c.def->findNode("top"), c.angle, c.offset);
            CHECK(vnear(c.localPos, want.childPos));
            CHECK(mnear(c.localRot, want.childRot));
        }
        // a surface child (side pod) matches attachSurface
        {
            const BuildPart &p = bs.parts[3];
            CHECK(p.attach == AttachMode::Surface);
            const Node *cn = p.def->findSurfaceNode();
            CHECK(cn != nullptr);
            AttachPose want = attachSurface(bs.parts[(size_t)p.parent].localPos,
                                            bs.parts[(size_t)p.parent].localRot,
                                            p.contactPoint, p.contactNormal,
                                            *cn, p.roll, p.offset);
            CHECK(vnear(p.localPos, want.childPos));
            CHECK(mnear(p.localRot, want.childRot));
        }
    }
    // an occupied stack port refuses new attachments (the editor rule);
    // surface edges never occupy a stack port
    {
        ShipDef tk2 = load_ship_def("res/ships/tanker.json", cat);
        BuildShip bs = BuildShip::fromShipDef(tk2);
        // the real tanker: the capsule->tank stack edge consumes BOTH mated
        // nodes (the parent's port and the child's)
        const BuildPart &c1 = bs.parts[1];
        CHECK(c1.attach == AttachMode::Down);
        CHECK(bs.nodeOccupied(c1.parent, c1.parentNode));
        CHECK(bs.nodeOccupied(1, c1.childNode));
        // hand-built mini tree: deterministic occupancy as children are added
        BuildShip mini;
        BuildPart root; root.def = bs.parts[0].def; root.id = "root";
        mini.parts.push_back(root);
        CHECK(root.def->findNode("top") != nullptr);
        CHECK(!mini.nodeOccupied(0, "top"));
        BuildPart kid; kid.def = bs.parts[1].def; kid.id = "kid";
        kid.parent = 0; kid.attach = AttachMode::Down;
        kid.parentNode = "top"; kid.childNode = "bottom";
        mini.parts.push_back(kid);
        CHECK(mini.nodeOccupied(0, "top"));
        CHECK(mini.nodeOccupied(1, "bottom"));   // child side consumed too
        CHECK(!mini.nodeOccupied(0, "bottom"));
        CHECK(!mini.nodeOccupied(1, "top"));     // kid's far port stays free
        BuildPart pod; pod.def = bs.parts[3].def; pod.id = "pod";
        pod.parent = 0; pod.attach = AttachMode::Surface;
        pod.childNode = pod.def->findSurfaceNode()->id;
        mini.parts.push_back(pod);
        CHECK(mini.nodeOccupied(0, "top"));       // unchanged by the surface child
        CHECK(!mini.nodeOccupied(0, "bottom"));
        CHECK(!mini.nodeOccupied(2, "top"));      // surface-attached pod's
        CHECK(!mini.nodeOccupied(2, "bottom"));   // stack ports stay free
    }
    // --- VAB tree ops: removePart / rotatePart ------------------------------
    {
        ShipDef tk2 = load_ship_def("res/ships/tanker.json", cat);
        BuildShip bs = BuildShip::fromShipDef(tk2);
        // the tanker's tank (parts[1]) carries the engine + all four side
        // pods: deleting it takes the whole subtree, leaving the capsule
        CHECK(bs.parts.size() == 7);
        CHECK(bs.removePart(1));
        CHECK(bs.parts.size() == 1);
        CHECK(bs.parts[0].parent == -1);
        CHECK(!bs.removePart(0));       // the root refuses
        CHECK(!bs.removePart(5));       // out of range refuses
        CHECK(bs.parts.size() == 1);
    }
    {
        // remap: root + kid + grandkid + kid2; deleting kid takes grandkid,
        // kid2 survives with its parent remapped and its pose re-solved
        ShipDef tk2 = load_ship_def("res/ships/tanker.json", cat);
        BuildShip full = BuildShip::fromShipDef(tk2);
        BuildShip mini;
        for(int i = 0; i < 4; i++) {
            BuildPart bp;
            bp.def = full.parts[(size_t)(i == 2 ? 1 : (i == 3 ? 0 : i))].def;
            bp.id = "p" + std::to_string(i);
            bp.parent = (i == 0) ? -1 : (i == 2 ? 1 : 0);
            bp.attach = AttachMode::Down;
            bp.parentNode = (i == 0) ? "" : "bottom";
            bp.childNode = (i == 0) ? "" : "top";
            mini.parts.push_back(bp);
        }
        mini.recomputePoses();
        const glm::dvec3 kid2Pos = mini.parts[3].localPos;
        CHECK(mini.removePart(1));
        CHECK(mini.parts.size() == 2);
        CHECK(mini.parts[1].id == "p3");
        CHECK(mini.parts[1].parent == 0);
        CHECK(vnear(mini.parts[1].localPos, kid2Pos));   // pose unchanged
    }
    {
        // rotatePart: a stack child's angle and a surface child's roll change
        // and the poses re-solve; the root is a no-op
        ShipDef tk2 = load_ship_def("res/ships/tanker.json", cat);
        BuildShip bs = BuildShip::fromShipDef(tk2);
        const glm::dmat3 rootBefore = bs.parts[0].localRot;
        bs.rotatePart(0, 90.0);
        CHECK(mnear(bs.parts[0].localRot, rootBefore));
        const glm::dmat3 kidBefore = bs.parts[1].localRot;
        bs.rotatePart(1, 90.0);
        CHECK(near(bs.parts[1].angle, 90.0));
        CHECK(!mnear(bs.parts[1].localRot, kidBefore));
        const double rollBefore = bs.parts[3].roll;
        bs.rotatePart(3, -45.0);
        CHECK(near(bs.parts[3].roll, rollBefore - 45.0));
        CHECK(bs.parts[3].attach == AttachMode::Surface);
    }
    // --- detachSubtree / graftTree (the subassembly model ops) ---------------
    {
        // tanker: detaching the tank subtree and grafting it back with the
        // SAME edge reproduces the original ship exactly (ids, order, poses)
        ShipDef tk2 = load_ship_def("res/ships/tanker.json", cat);
        BuildShip bs = BuildShip::fromShipDef(tk2);
        CHECK(bs.detachSubtree(0).parts.empty());       // the root refuses
        std::vector<glm::dvec3> pos0;
        std::vector<glm::dmat3> rot0;
        std::vector<std::string> ids0;
        for(size_t i = 0; i < bs.parts.size(); i++) {
            pos0.push_back(bs.parts[i].localPos);
            rot0.push_back(bs.parts[i].localRot);
            ids0.push_back(bs.parts[i].id);
        }
        const BuildPart edge0 = bs.parts[1];   // the tank's original edge
        const glm::dvec3 tankPos = bs.parts[1].localPos;
        const glm::dmat3 tankRot = bs.parts[1].localRot;
        BuildShip sub = bs.detachSubtree(1);
        CHECK(bs.parts.size() == 1);
        CHECK(sub.parts.size() == 6);
        CHECK(sub.parts[0].id == "tank_r1.5h3_1");
        CHECK(sub.parts[0].parent == -1);
        CHECK(sub.parts[0].parentNode.empty());   // the edge was cleared
        CHECK(vnear(sub.parts[0].localPos, glm::dvec3(0)));
        CHECK(mnear(sub.parts[0].localRot, glm::dmat3(1.0)));
        // the relative shape survives: the engine's sub-frame pose is its
        // old S pose seen from the tank's frame
        {
            const glm::dmat3 invR = glm::transpose(tankRot);
            bool found = false;
            for(size_t i = 0; i < sub.parts.size(); i++) {
                if(sub.parts[i].id != "engine_1") { continue; }
                found = true;
                CHECK(vnear(sub.parts[i].localPos, invR * (pos0[2] - tankPos)));
                CHECK(mnear(sub.parts[i].localRot, invR * rot0[2]));
            }
            CHECK(found);
        }
        BuildPart rootT = edge0;
        rootT.id = sub.parts[0].id;
        CHECK(bs.graftTree(sub, rootT) == 1);
        CHECK(bs.parts.size() == 7);
        for(size_t i = 0; i < bs.parts.size(); i++) {
            CHECK(bs.parts[i].id == ids0[i]);
            CHECK(vnear(bs.parts[i].localPos, pos0[i]));
            CHECK(mnear(bs.parts[i].localRot, rot0[i]));
        }
        // copy-paste: grafting again uniquifies the ids (the assembly is
        // NOT consumed by placing)
        CHECK(bs.graftTree(sub, rootT) == 7);
        CHECK(bs.parts.size() == 13);
        CHECK(bs.parts[7].id == "tank_r1.5h3_1_2");
        CHECK(bs.parts[8].id == "engine_1_2");
        bool unique = true;
        for(size_t i = 0; i < bs.parts.size(); i++) {
            for(size_t j = i + 1; j < bs.parts.size(); j++) {
                if(bs.parts[i].id == bs.parts[j].id) { unique = false; }
            }
        }
        CHECK(unique);
    }
    {
        // a single surface part: detach + graft with its original contact
        // lands it back at the same pose (appended at the tail this time)
        ShipDef tk2 = load_ship_def("res/ships/tanker.json", cat);
        BuildShip bs = BuildShip::fromShipDef(tk2);
        const BuildPart edgeP = bs.parts[3];
        const glm::dvec3 podPos = bs.parts[3].localPos;
        const glm::dmat3 podRot = bs.parts[3].localRot;
        BuildShip sub = bs.detachSubtree(3);
        CHECK(sub.parts.size() == 1);
        CHECK(bs.parts.size() == 6);
        BuildPart rootT = edgeP;
        rootT.id = sub.parts[0].id;
        bs.graftTree(sub, rootT);
        CHECK(bs.parts.size() == 7);
        CHECK(bs.parts[6].id == edgeP.id);
        CHECK(vnear(bs.parts[6].localPos, podPos));
        CHECK(mnear(bs.parts[6].localRot, podRot));
    }
    {
        // heavy_two: interior links MOVE with the assembly, crossing links
        // DROP, the controller follows its part, and the graft restores the
        // exact ship (links included)
        ShipDef h2 = load_ship_def("res/ships/heavy_two.json", cat);
        BuildShip bs = BuildShip::fromShipDef(h2);
        CHECK(bs.fuelLinks.size() == 2);
        std::vector<glm::dvec3> pos0;
        std::vector<glm::dmat3> rot0;
        std::vector<std::string> ids0;
        for(size_t i = 0; i < bs.parts.size(); i++) {
            pos0.push_back(bs.parts[i].localPos);
            rot0.push_back(bs.parts[i].localRot);
            ids0.push_back(bs.parts[i].id);
        }
        int t2 = -1, rt = -1;
        for(size_t i = 0; i < bs.parts.size(); i++) {
            if(bs.parts[i].id == "central_tank2") { t2 = (int)i; }
            if(bs.parts[i].id == "rtank12") { rt = (int)i; }
        }
        CHECK(t2 > 0 && rt > 0);
        const BuildPart edgeT2 = bs.parts[(size_t)t2];
        BuildShip sub = bs.detachSubtree(t2);
        CHECK(sub.parts.size() == 11);
        CHECK(bs.parts.size() == 12);
        CHECK(bs.fuelLinks.empty());            // both asparagus links moved
        CHECK(sub.fuelLinks.size() == 2);
        CHECK(bs.controllerId == "capsule_1");  // the controller stayed
        CHECK(sub.controllerId.empty());
        // a crossing link drops from BOTH trees
        BuildShip b2 = BuildShip::fromShipDef(h2);
        BuildShip s2 = b2.detachSubtree(rt);
        CHECK(s2.parts.size() == 2);            // rtank12 + rengine1
        CHECK(s2.fuelLinks.empty());
        CHECK(b2.fuelLinks.size() == 1);
        CHECK(b2.fuelLinks[0].id == "asparagus2");
        // the controller follows its part into the assembly
        BuildShip b3 = BuildShip::fromShipDef(h2);
        b3.controllerId = "central_tank2";
        BuildShip s3 = b3.detachSubtree(t2);
        CHECK(b3.controllerId.empty());
        CHECK(s3.controllerId == "central_tank2");
        // graft back with the original edge: the exact original ship
        BuildPart rootT = edgeT2;
        rootT.id = sub.parts[0].id;
        CHECK(bs.graftTree(sub, rootT) == (size_t)t2);
        CHECK(bs.parts.size() == 23);
        CHECK(bs.fuelLinks.size() == 2);
        for(size_t i = 0; i < bs.parts.size(); i++) {
            CHECK(bs.parts[i].id == ids0[i]);
            CHECK(vnear(bs.parts[i].localPos, pos0[i]));
            CHECK(mnear(bs.parts[i].localRot, rot0[i]));
        }
        for(size_t i = 0; i < bs.fuelLinks.size(); i++) {
            bool haveFrom = false, haveTo = false;
            for(size_t k = 0; k < bs.parts.size(); k++) {
                if(bs.parts[k].id == bs.fuelLinks[i].from) { haveFrom = true; }
                if(bs.parts[k].id == bs.fuelLinks[i].to)   { haveTo = true; }
            }
            CHECK(haveFrom);
            CHECK(haveTo);
        }
    }
    // --- grafting a subassembly as the ROOT of an empty build: the VAB's
    //     "build a ship from nothing" with an armed subassembly. The grafted
    //     root (parent -1) anchors at the S origin and the assembly's relative
    //     shape is preserved (e2e 56 pins the single-part root; this pins the
    //     subassembly root at the model level)
    {
        ShipDef tk3 = load_ship_def("res/ships/tanker.json", cat);
        BuildShip bs3 = BuildShip::fromShipDef(tk3);
        BuildShip sub3 = bs3.detachSubtree(1);   // the tank subtree
        CHECK(sub3.parts.size() == 6);
        CHECK(sub3.parts[0].parent == -1);
        // the root edge the VAB stores for a subassembly-as-root
        BuildPart root3;
        root3.def = sub3.parts[0].def;
        root3.id = sub3.parts[0].id;
        root3.parent = -1;                        // the ROOT of the new build
        root3.attach = AttachMode::Down;
        BuildShip empty;                          // the --vab-empty state
        const size_t ri = empty.graftTree(sub3, root3);
        CHECK(ri == 0);                           // the root lands at index 0
        CHECK(empty.parts.size() == 6);
        CHECK(empty.parts[0].parent == -1);
        CHECK(vnear(empty.parts[0].localPos, glm::dvec3(0)));    // S origin
        CHECK(mnear(empty.parts[0].localRot, glm::dmat3(1.0)));
        // the relative shape survives: a descendant's S-frame pose equals its
        // pose in the sub tree (both re-solved off the identity root)
        CHECK(vnear(empty.parts[2].localPos, sub3.parts[2].localPos));
        CHECK(mnear(empty.parts[2].localRot, sub3.parts[2].localRot));
    }
    // --- radialSymmetryClones: symmetric rings about the parent's axis ------
    {
        const PartDef *child = cat.find("tank_r1.5h3");   // the tanker's side pod
        CHECK(child != nullptr);
        const Node *cn = child->findSurfaceNode();
        CHECK(cn != nullptr);
        const double PI = std::acos(-1.0);
        // A) axis-aligned parent, radial contact + roll: each clone is the
        //    primary rotated by k*360/N about the parent's Z, and its stored
        //    edge data re-solves EXACTLY to the stored pose (what the tree's
        //    recomputePoses will do)
        {
            const glm::dvec3 P(0, 0, 3);
            const glm::dmat3 R(1.0);
            const glm::dvec3 pt(1.5, 0, 0.5);
            const glm::dvec3 nl(1, 0, 0);
            const AttachPose primary = attachSurface(P, R, pt, nl, *cn, 25.0, 0.0);
            for(int n = 2; n <= 8; n++) {
                std::vector<SymClone> cl =
                    radialSymmetryClones(P, R, *cn, pt, nl, 25.0, 0.0, n);
                CHECK(cl.size() == (size_t)(n - 1));
                for(int k = 1; k < n; k++) {
                    const SymClone &c = cl[(size_t)k - 1];
                    const double th = 2.0 * PI * (double)k / (double)n;
                    const glm::dmat3 Rz = glm::mat3_cast(
                        glm::angleAxis(th, glm::dvec3(0, 0, 1)));
                    CHECK(vnear(c.pose.childPos, Rz * primary.childPos));
                    CHECK(mnear(c.pose.childRot, Rz * primary.childRot));
                    AttachPose re = attachSurface(P, R, c.edge.point,
                                                  c.edge.normal, *cn,
                                                  c.edge.rollDeg, 0.0);
                    CHECK(vnear(re.childPos, c.pose.childPos));
                    CHECK(mnear(re.childRot, c.pose.childRot));
                }
            }
            CHECK(radialSymmetryClones(P, R, *cn, pt, nl, 25.0, 0.0, 1).empty());
        }
        // B) tilted parent pose + tilted (non-radial) normal: exercises the
        //    minimal-arc holonomy correction -- the clones must still be
        //    congruent about the parent's OWN axis
        {
            const glm::dvec3 P(0, 0, -2);
            const glm::dmat3 R = glm::mat3_cast(
                glm::angleAxis(0.7, glm::normalize(glm::dvec3(1, 2, 3))));
            const glm::dvec3 pt(1.2, 0.3, -0.4);
            const glm::dvec3 nl = glm::normalize(glm::dvec3(0.9, 0.15, 0.4));
            const AttachPose primary = attachSurface(P, R, pt, nl, *cn, 0.0, 0.0);
            const glm::dvec3 axisS = R * glm::dvec3(0, 0, 1);
            const int ns[2] = { 3, 6 };
            for(int ni = 0; ni < 2; ni++) {
                const int n = ns[ni];
                std::vector<SymClone> cl =
                    radialSymmetryClones(P, R, *cn, pt, nl, 0.0, 0.0, n);
                CHECK(cl.size() == (size_t)(n - 1));
                for(int k = 1; k < n; k++) {
                    const SymClone &c = cl[(size_t)k - 1];
                    const double th = 2.0 * PI * (double)k / (double)n;
                    const glm::dmat3 RzS = glm::mat3_cast(
                        glm::angleAxis(th, axisS));
                    const glm::dvec3 wantPos = P + RzS * (primary.childPos - P);
                    const glm::dmat3 wantRot = RzS * primary.childRot;
                    CHECK(vnear(c.pose.childPos, wantPos));
                    CHECK(mnear(c.pose.childRot, wantRot));
                    AttachPose re = attachSurface(P, R, c.edge.point,
                                                  c.edge.normal, *cn,
                                                  c.edge.rollDeg, 0.0);
                    CHECK(vnear(re.childPos, c.pose.childPos));
                    CHECK(mnear(re.childRot, c.pose.childRot));
                }
            }
        }
    }
    // --- placement snapping (snapSurfaceContact / gridStepDeg) ---------------
    {
        const double D2R = std::acos(-1.0) / 180.0;
        // a revolution-surface side contact with a FACET-quantized normal
        // (the normal azimuth leads the hit position by a few degrees, as on
        // the pick hull's flat facets): both land on the POINT's snapped
        // azimuth, the radius and the normal's tilt survive, the height snaps
        glm::dvec3 p(1.5 * std::cos(7.0 * D2R), 1.5 * std::sin(7.0 * D2R), 0.37);
        glm::dvec3 n = glm::normalize(glm::dvec3(std::cos(11.0 * D2R),
                                                 std::sin(11.0 * D2R), 0.35));
        const double nz = n.z;
        snapSurfaceContact(p, n, true, true);
        CHECK(near(glm::degrees(std::atan2(p.y, p.x)), 10.0));
        CHECK(near(std::hypot(p.x, p.y), 1.5));
        CHECK(near(p.z, 0.4));
        CHECK(near(glm::degrees(std::atan2(n.y, n.x)), 10.0));
        CHECK(near(n.z, nz));
        // a non-revolution contact (normal azimuth diverges from the
        // point's): each snaps its OWN azimuth
        p = glm::dvec3(1.2 * std::cos(7.0 * D2R), 1.2 * std::sin(7.0 * D2R), 0.0);
        n = glm::dvec3(std::cos(103.0 * D2R), std::sin(103.0 * D2R), 0.0);
        snapSurfaceContact(p, n, true, true);
        CHECK(near(glm::degrees(std::atan2(p.y, p.x)), 10.0));
        CHECK(near(glm::degrees(std::atan2(n.y, n.x)), 100.0));
        // a cap hit (on the axis): only the height snaps
        p = glm::dvec3(1e-6, 2e-6, 0.62);
        n = glm::dvec3(0, 0, 1);
        snapSurfaceContact(p, n, true, true);
        CHECK(near(p.z, 0.6));
        CHECK(near(glm::degrees(std::atan2(p.y, p.x)),
                   glm::degrees(std::atan2(2e-6, 1e-6))));
        CHECK(near(n.z, 1.0));
        // the toggles are independent
        p = glm::dvec3(1.5 * std::cos(7.0 * D2R), 1.5 * std::sin(7.0 * D2R), 0.37);
        n = glm::dvec3(std::cos(7.0 * D2R), std::sin(7.0 * D2R), 0.0);
        snapSurfaceContact(p, n, true, false);   // distance only
        CHECK(near(p.z, 0.4));
        CHECK(near(glm::degrees(std::atan2(p.y, p.x)), 7.0));
        snapSurfaceContact(p, n, false, true);   // angle only
        CHECK(near(p.z, 0.4));                   // height untouched this time
        CHECK(near(glm::degrees(std::atan2(p.y, p.x)), 10.0));
        // the grids
        CHECK(near(snapAngleDeg(23.0), 20.0));
        CHECK(near(snapAngleDeg(-23.0), -20.0));
        CHECK(near(gridStepDeg(17.0, +1.0), 20.0));
        CHECK(near(gridStepDeg(17.0, -1.0), 10.0));
        CHECK(near(gridStepDeg(20.0, +1.0), 30.0));   // on-grid steps onward
        CHECK(near(gridStepDeg(20.0, -1.0), 10.0));
        CHECK(near(gridStepDeg(-3.0, +1.0), 0.0));
        CHECK(near(gridStepDeg(-3.0, -1.0), -10.0));
    }
    // --- save_ship_def round trip -------------------------------------------
    {
        const char *rt = "/tmp/test_shipload_rt.json";
        // tanker: poses + ids + controller survive the round trip, including
        // the surface pods (their load-resolved contacts are written as
        // explicit point/normal)
        ShipDef tk2 = load_ship_def("res/ships/tanker.json", cat);
        BuildShip bs = BuildShip::fromShipDef(tk2);
        CHECK(bs.controllerId == "capsule_1");
        CHECK(save_ship_def(bs, rt));
        ShipDef rl = load_ship_def(rt, cat);
        BuildShip bs2 = BuildShip::fromShipDef(rl);
        CHECK(bs2.parts.size() == bs.parts.size());
        CHECK(bs2.controllerId == bs.controllerId);
        for(size_t i = 0; i < bs.parts.size(); i++) {
            CHECK(bs2.parts[i].id == bs.parts[i].id);
            CHECK(bs2.parts[i].parent == bs.parts[i].parent);
            CHECK(bs2.parts[i].attach == bs.parts[i].attach);
            CHECK(vnear(bs2.parts[i].localPos, bs.parts[i].localPos));
            CHECK(mnear(bs2.parts[i].localRot, bs.parts[i].localRot));
        }
        // heavy_two: fuel links + stages survive; deleting a link endpoint
        // drops just that link
        ShipDef h2 = load_ship_def("res/ships/heavy_two.json", cat);
        BuildShip hb = BuildShip::fromShipDef(h2);
        CHECK(hb.fuelLinks.size() == 2);
        CHECK(save_ship_def(hb, rt));
        BuildShip hb2 = BuildShip::fromShipDef(load_ship_def(rt, cat));
        CHECK(hb2.parts.size() == hb.parts.size());
        CHECK(hb2.fuelLinks.size() == 2);
        CHECK(hb2.controllerId == "capsule_1");
        CHECK(hb2.parts[0].stage == 1);
        for(size_t i = 0; i < hb.parts.size(); i++) {
            CHECK(hb2.parts[i].id == hb.parts[i].id);
            CHECK(vnear(hb2.parts[i].localPos, hb.parts[i].localPos));
            CHECK(mnear(hb2.parts[i].localRot, hb.parts[i].localRot));
        }
        // toShipDef: the controller resolves to its index, links re-append
        ShipDef back = hb.toShipDef();
        CHECK(back.controller == 0);
        size_t nlink = 0;
        for(size_t i = 0; i < back.parts.size(); i++) {
            if(back.parts[i].isFuelLink()) { nlink++; }
        }
        CHECK(nlink == 2);
        // delete rtank12 (asparagus1's source): that link drops, the other stays
        int victim = -1;
        for(size_t i = 0; i < hb.parts.size(); i++) {
            if(hb.parts[i].id == "rtank12") { victim = (int)i; }
        }
        CHECK(victim > 0);
        CHECK(hb.removePart(victim));
        // the dead link is purged from the TREE immediately (not only at
        // convert/save time), and the survivor stays
        CHECK(hb.fuelLinks.size() == 1);
        CHECK(hb.fuelLinks[0].id == "asparagus2");
        ShipDef trimmed = hb.toShipDef();
        size_t kept = 0;
        for(size_t i = 0; i < trimmed.parts.size(); i++) {
            if(trimmed.parts[i].isFuelLink()) {
                kept++;
                CHECK(trimmed.parts[i].id == "asparagus2");
            }
        }
        CHECK(kept == 1);
        CHECK(save_ship_def(hb, rt));
        BuildShip hb3 = BuildShip::fromShipDef(load_ship_def(rt, cat));
        CHECK(hb3.fuelLinks.size() == 1);
        // empty tree refuses to save
        BuildShip empty;
        CHECK(!save_ship_def(empty, rt));
        std::remove(rt);
    }
    // fuel links are dropped and parent indices remapped (heavy_two has links);
    // construction order means every non-root parent is an earlier part
    {
        ShipDef h2 = load_ship_def("res/ships/heavy_two.json", cat);
        BuildShip bs = BuildShip::fromShipDef(h2);
        size_t nphys = 0;
        for(size_t i = 0; i < h2.parts.size(); i++) {
            if(!h2.parts[i].isFuelLink()) { nphys++; }
        }
        CHECK(bs.parts.size() == nphys);
        CHECK(bs.parts.size() < h2.parts.size());
        for(size_t i = 1; i < bs.parts.size(); i++) {
            CHECK(bs.parts[i].parent >= 0 && (size_t)bs.parts[i].parent < i);
        }
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

    // a valid ship: explicit ids, a surface angle, a spacer, a stage
    {
        const char *ok = "/tmp/test_shipload_ok.json";
        std::ofstream f(ok);
        f << "{ \"name\": \"T\", \"controller\": \"nose\", "
             "\"parts\": [ "
             " { \"part\": \"capsule\", \"id\": \"nose\" }, "
             " { \"part\": \"engine\", \"attach\": \"down\", \"stage\": 1 }, "
             " { \"part\": \"tank_r1.5h2\", \"attach\": \"surface\", "
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
            CHECK(okd.parts[2].attach == AttachMode::Surface);
            CHECK(okd.parts[2].parent == 1);
            CHECK(near(okd.parts[2].angle, 30.0));
            CHECK(near(okd.parts[2].offset, 0.25));
            // the cylinder shorthand resolved the contact normal at clock 30
            CHECK(vnear(okd.parts[2].contactNormal,
                        glm::dvec3(cos(glm::radians(30.0)), sin(glm::radians(30.0)), 0.0)));
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

    // power fields must be >= 0 (W)
    {
        const char *bad = "/tmp/test_shipload_badcat.json";
        std::ofstream f(bad);
        f << "{ \"parts\": [ { \"name\": \"x\", \"type\": \"reaction_wheel\", "
             "\"mesh\": \"a.obj\", \"texture\": \"a.png\", \"mass\": 1.0, "
             "\"power_draw\": -5 } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_parts_catalog(bad); }));
        std::remove(bad);
    }
    {
        const char *bad = "/tmp/test_shipload_badcat.json";
        std::ofstream f(bad);
        f << "{ \"parts\": [ { \"name\": \"x\", \"type\": \"capsule\", "
             "\"mesh\": \"a.obj\", \"texture\": \"a.png\", \"mass\": 1.0, "
             "\"power_draw_constant\": -5 } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_parts_catalog(bad); }));
        std::remove(bad);
    }
    {
        const char *bad = "/tmp/test_shipload_badcat.json";
        std::ofstream f(bad);
        f << "{ \"parts\": [ { \"name\": \"x\", \"type\": \"rtg\", "
             "\"mesh\": \"a.obj\", \"texture\": \"a.png\", \"mass\": 1.0, "
             "\"power_gen\": -5 } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_parts_catalog(bad); }));
        std::remove(bad);
    }

    // fuel link in a ship def: from/to are parsed, the part is flagged
    {
        const char *ok = "/tmp/test_shipload_fuellink.json";
        std::ofstream f(ok);
        f << "{ \"name\": \"linkship\", "
             "\"parts\": [ { \"part\": \"tank_r1h3\", \"id\": \"tA\" }, "
             " { \"part\": \"engine\", \"id\": \"eA\", \"parent\": \"tA\" }, "
             " { \"part\": \"tank_r1h3\", \"id\": \"tB\" }, "
             " { \"part\": \"fuel_link\", \"id\": \"l1\", "
             "   \"from\": \"tB\", \"to\": \"eA\" } ] }";
        f.close();
        ShipDef fldef = load_ship_def(ok, cat);
        CHECK(fldef.parts.size() == 4);
        CHECK(fldef.parts[3].isFuelLink());
        CHECK(fldef.parts[3].from == "tB");
        CHECK(fldef.parts[3].to == "eA");
        CHECK(!fldef.parts[0].isFuelLink());
        CHECK(fldef.parts[0].from.empty() && fldef.parts[0].to.empty());
        std::remove(ok);
    }

    // fuel link without from/to: rejected
    {
        const char *bad = "/tmp/test_shipload_fuellink.json";
        std::ofstream f(bad);
        f << "{ \"name\": \"linkship\", "
             "\"parts\": [ { \"part\": \"tank_r1h3\", \"id\": \"tA\" }, "
             " { \"part\": \"fuel_link\", \"id\": \"l1\" } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_ship_def(bad, cat); }));
        std::remove(bad);
    }

    // fuel link with from == to: rejected
    {
        const char *bad = "/tmp/test_shipload_fuellink.json";
        std::ofstream f(bad);
        f << "{ \"name\": \"linkship\", "
             "\"parts\": [ { \"part\": \"tank_r1h3\", \"id\": \"tA\" }, "
             " { \"part\": \"fuel_link\", \"id\": \"l1\", "
             "   \"from\": \"tA\", \"to\": \"tA\" } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_ship_def(bad, cat); }));
        std::remove(bad);
    }

    // fuel link referencing itself: rejected
    {
        const char *bad = "/tmp/test_shipload_fuellink.json";
        std::ofstream f(bad);
        f << "{ \"name\": \"linkship\", "
             "\"parts\": [ { \"part\": \"tank_r1h3\", \"id\": \"tA\" }, "
             " { \"part\": \"fuel_link\", \"id\": \"l1\", "
             "   \"from\": \"l1\", \"to\": \"tA\" } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_ship_def(bad, cat); }));
        std::remove(bad);
    }
    /* Every ship def in res/ships/ must resolve against the real catalog. An
       unknown part name throws at load time -- and a ship that nothing loads
       (not fleet.json, not an e2e case, not src/) can sit broken for a long
       time: heavy_one.json referenced "reaction_wheel2.25h0.5625", missing
       the _r, and no test or case ever reached it. This sweep reads the
       directory rather than carrying a list, so a ship added tomorrow is
       covered the moment it lands. */
    {
        DIR *ships = opendir("res/ships");
        CHECK(ships != nullptr);
        if(ships != nullptr) {
            int swept = 0;
            struct dirent *e;
            while((e = readdir(ships)) != nullptr) {
                const std::string nm(e->d_name);
                if(nm.size() < 6 || nm.compare(nm.size() - 5, 5, ".json") != 0) {
                    continue;
                }
                const std::string path = "res/ships/" + nm;
                bool ok = false;
                try {
                    ok = !load_ship_def(path.c_str(), cat).parts.empty();
                } catch(const std::exception &ex) {
                    printf("  %s: %s\n", path.c_str(), ex.what());
                }
                if(!ok) { printf("  FAILED to load: %s\n", path.c_str()); }
                CHECK(ok);
                swept++;
            }
            closedir(ships);
            printf("  swept %d ship defs in res/ships/\n", swept);
            CHECK(swept > 0);   // a silent empty sweep would pass vacuously
        }
    }

    // list_ship_defs: keep only the .json entries, return their sorted slugs
    // (extension stripped). A temp dir gives a deterministic set so the
    // filter + ordering are checked, not just whatever res/ships holds.
    {
        const std::string dir = "/tmp/test_list_ship_defs";
        CHECK(std::system(("rm -rf '" + dir + "'").c_str()) == 0);
        CHECK(mkdir(dir.c_str(), 0755) == 0 || access(dir.c_str(), F_OK) == 0);
        // two .json files (created in reverse-alphabetical order) + a
        // non-.json file + a subdirectory, which must all be excluded.
        { std::ofstream f((dir + "/zzz.json").c_str()); f << "{}"; }
        { std::ofstream f((dir + "/aaa.json").c_str()); f << "{}"; }
        { std::ofstream f((dir + "/notes.txt").c_str()); f << "x"; }
        CHECK(mkdir((dir + "/adir").c_str(), 0755) == 0 || true);
        const std::vector<std::string> got = list_ship_defs(dir);
        CHECK(got.size() == 2);
        CHECK(got.size() >= 1 && got[0] == "aaa");
        CHECK(got.size() >= 2 && got[1] == "zzz");
        // a missing directory yields an empty list (no throw)
        CHECK(list_ship_defs("/tmp/test_list_ship_defs_no_such_dir").empty());
        CHECK(std::system(("rm -rf '" + dir + "'").c_str()) == 0);
    }

    // list_ship_defs on the real res/ships: sorted, and every slug resolves
    // to a file that load_ship_def can parse (the Load picker offers only
    // loadable ships).
    {
        const std::vector<std::string> slugs = list_ship_defs("res/ships");
        CHECK(!slugs.empty());
        for(size_t i = 1; i < slugs.size(); i++) {
            CHECK(slugs[i - 1] < slugs[i]);   // strictly sorted (unique names)
        }
        for(size_t i = 0; i < slugs.size(); i++) {
            const std::string path = "res/ships/" + slugs[i] + ".json";
            bool ok = false;
            try { ok = !load_ship_def(path.c_str(), cat).parts.empty(); }
            catch(const std::exception &) { ok = false; }
            CHECK(ok);
        }
    }


    if(failures) {
        printf("test_shipload: %d FAILURES\n", failures);
        return 1;
    }
    printf("test_shipload: all tests passed\n");
    return 0;
}
