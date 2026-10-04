// test_retrograde.cpp -- pins the sanctioned retrograde encodings through the
// REAL loader (load_system on res/systems/solar_system.json).
//
// The project's policy (utils/make_solar_system.py:174 and issue #139):
// rates are magnitudes; the SENSE lives in the orientation fields.
//   - retrograde ORBIT: orb_ang_speed > 0 + orb_incl > 90 deg (the frame's
//     orient flips the orbital normal; the rail stays locally prograde).
//     Live case: Triton around Neptune, incl 157.3 deg.
//   - retrograde SPIN: rot_ang_speed > 0 + axial_tilt > 90 deg.
//     Live case: Venus, tilt 177.3 deg (the fact sheet's -243 d rotation
//     period is abs()'d by the generator).
// Both paths were untested before #139's QA pass; the calendar gates
// (system.cpp, D/Y > 0) silently invalidate negative rates, so these
// encodings are the ONLY supported ones -- and load_system now throws on
// orb_ang_speed < 0 / rot_ang_speed < 0 instead of silently loading
// prograde (the a = cbrt(mu/w^2) sign drop), and on rates so small the
// derived orbit escapes the universe bound (the magnitude twin, #140).
// #141 adds the epoch spin-phase fields (spin_phase0, tilt_azimuth): the
// defaults must reproduce the pre-#141 initial_orient exactly, and the
// authored values must land the epoch longitude/pole azimuth where named.
// The throw and authoring cases run on targeted mutations of the real
// system JSON (written to tmp/).
//
// Build & run (from repo root): see Makefile ($(TESTDIR)/test_retrograde).

#include "system.h"
#include "frame.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <string>

#include <nlohmann/json.hpp>

static constexpr double PI = std::numbers::pi;

static int g_failures = 0;

static void check(bool cond, const char *what) {
    if(!cond) {
        std::printf("FAIL %s\n", what);
        ++g_failures;
    }
}

static nlohmann::json read_json(const char *path) {
    std::ifstream f(path);
    if(!f.is_open()) {
        std::printf("FAIL cannot open %s (run from the repo root)\n", path);
        ++g_failures;
        return nlohmann::json::object();
    }
    return nlohmann::json::parse(f);
}

// Authored orb_incl for a body, straight from the same JSON the loader read
// (so the test tracks data edits instead of hard-coding a value).
static double authored_orb_incl(const char *path, const std::string &name) {
    for(auto &&b : read_json(path).value("bodies", nlohmann::json::array())) {
        if(b["name"] == name) {
            return b.value("inertial", nlohmann::json::object())
                       .value("orb_incl", 0.0);
        }
    }
    return 0.0;
}

static double azimuth(const glm::dvec3 &r) {
    return std::atan2(r.z, r.x);
}

// Write a mutated system JSON (tmp/), require load_system to reject it with
// the expected message, and clean up. The mutations differ from the
// legitimate file only in the one field, so a pass can only come from the
// specific guard named by msg_needle.
static void expect_reject(const nlohmann::json &j, const char *tag,
                          const char *msg_needle) {
    std::filesystem::create_directories("tmp");
    const std::string tmp = std::string("tmp/test_retrograde_") + tag + ".json";
    std::ofstream o(tmp);
    o << j.dump();
    o.close();
    const char *what = nullptr;
    try {
        load_system(tmp.c_str(), nullptr, nullptr);
    } catch(const std::exception &e) {
        what = e.what();
    }
    // Catch-all above so cleanup runs even on an unexpected exception type.
    const bool rejected = what && std::strstr(what, msg_needle);
    check(rejected, tag);
    std::remove(tmp.c_str());
}

// The authored value at bodies[name][section][field] (0 if absent).
static double authored(const nlohmann::json &j, const std::string &name,
                       const char *section, const char *field) {
    for(auto &&b : j["bodies"]) {
        if(b["name"] == name) { return b.at(section).at(field).get<double>(); }
    }
    return 0.0;
}

// j with bodies[name][section][field] set to v (creates the field; asserts
// the section exists).
static nlohmann::json mutated(nlohmann::json j, const std::string &name,
                              const char *section, const char *field,
                              double v) {
    for(auto &&b : j["bodies"]) {
        if(b["name"] == name) {
            check(b.contains(section), "mutation target section present");
            b[section][field] = v;
        }
    }
    return j;
}

// Write a mutated JSON (tmp/), load it successfully, run fn on the loaded
// System, then clean up. Used for the #141 authoring pins.
template<class F>
static void with_loaded(const nlohmann::json &j, const char *tag, F &&fn) {
    std::filesystem::create_directories("tmp");
    const std::string tmp = std::string("tmp/test_retrograde_") + tag + ".json";
    std::ofstream o(tmp);
    o << j.dump();
    o.close();
    try {
        System sys = load_system(tmp.c_str(), nullptr, nullptr);
        fn(sys);
        for(TerrainBody *b : sys.bodies) { delete b; }
    } catch(const std::exception &e) {
        std::printf("FAIL %s: unexpected load throw: %s\n", tag, e.what());
        ++g_failures;
    }
    std::remove(tmp.c_str());
}

int main() {
    const char *sys_path = "res/systems/solar_system.json";
    System sys = load_system(sys_path, nullptr, nullptr);
    nlohmann::json j = read_json(sys_path);

    // --- Triton: retrograde ORBIT via orb_incl > 90 deg -------------------
    TerrainBody *triton = sys.find("Triton");
    TerrainBody *neptune = sys.find("Neptune");
    check(triton && neptune, "Triton and Neptune present in solar_system.json");
    if(!triton || !neptune) { return 1; }

    Frame *tf = triton->frame;
    Frame *nf = neptune->frame;
    check(tf->parent == nf, "Triton inertial frame is a child of Neptune's");
    // The transfer planner and calendar gate on orb_ang_speed > 0: the
    // sanctioned encoding keeps w positive, so Triton passes by construction.
    check(tf->orb_ang_speed > 0.0, "Triton orb_ang_speed is positive");

    // Rail epoch state is local (prograde about local +Y); orient maps it
    // into the parent frame. The orbital angular momentum must sit at the
    // authored inclination from the parent's +Y -- past 90 deg.
    const double incl = authored_orb_incl(sys_path, "Triton");
    check(incl > PI / 2.0, "Triton's authored orb_incl is retrograde (> 90 deg)");
    const glm::dvec3 h = tf->orient * glm::cross(tf->orbit_pos0, tf->orbit_vel0);
    const double h_ang = std::acos(h.y / glm::length(h));
    check(std::fabs(h_ang - incl) < 1e-6,
          "loaded orbit normal sits at the authored inclination from parent +Y");
    check(h.y < 0.0, "Triton's orbit normal is below Neptune's orbital plane "
                     "(retrograde sense reached the rail)");

    // Motion, not just geometry: prograde (h.Y > 0) sweeps the azimuth
    // atan2(z, x) DOWN (rail velocity is +Y x r_hat); retrograde sweeps it UP.
    const double T = 2.0 * PI / tf->orb_ang_speed;
    sys.root->frame->UpdateOrbitRails(0.0);
    const double az0 = azimuth(tf->GetPositionRelTo(nf));
    sys.root->frame->UpdateOrbitRails(T / 8.0);
    double daz = azimuth(tf->GetPositionRelTo(nf)) - az0;
    if(daz > PI) { daz -= 2.0 * PI; }
    if(daz < -PI) { daz += 2.0 * PI; }
    check(daz > 0.0, "Triton sweeps counter-clockwise about Neptune's +Y "
                     "(retrograde motion over T/8)");

    // Tripwire that the rail still propagates at all (propagateKepler folds
    // whole periods, so this is near-exact by construction).
    sys.root->frame->UpdateOrbitRails(T);
    const glm::dvec3 rT = tf->GetPositionRelTo(nf);
    sys.root->frame->UpdateOrbitRails(0.0);
    const glm::dvec3 r0 = tf->GetPositionRelTo(nf);
    check(glm::length(rT - r0) < 1e-6 * glm::length(r0),
          "Triton's rail closes after 2*pi/w");
    // Tripwire for the Y > 0 calendar gate: year_seconds is set from w
    // directly, so this only fails if the gate or the rate goes away.
    check(triton->cal.year_seconds > 0.9 * T && triton->cal.year_seconds < 1.1 * T,
          "Triton's calendar year matches the orbital period (Y gate survives)");

    // Prograde control: the same sweep test must read the OTHER sign for a
    // normal body, so the Triton check above is discriminating, not vacuous.
    TerrainBody *earth = sys.find("Earth");
    check(earth != nullptr, "Earth present in solar_system.json");
    if(earth) {
        sys.root->frame->UpdateOrbitRails(0.0);
        const double e0 = azimuth(earth->frame->GetPositionRelTo(sys.root->frame));
        sys.root->frame->UpdateOrbitRails(PI / (2.0 * earth->frame->orb_ang_speed));
        double edaz = azimuth(earth->frame->GetPositionRelTo(sys.root->frame)) - e0;
        if(edaz > PI) { edaz -= 2.0 * PI; }
        if(edaz < -PI) { edaz += 2.0 * PI; }
        check(edaz < 0.0, "Earth sweeps prograde (azimuth DOWN) -- control");
    }

    // --- Venus: retrograde SPIN via axial_tilt > 90 deg --------------------
    TerrainBody *venus = sys.find("Venus");
    check(venus != nullptr, "Venus present in solar_system.json");
    if(venus) {
        Frame *vf = venus->rot_frame;
        check(vf && vf->rot_ang_speed > 0.0,
              "Venus rot_ang_speed is positive (sense lives in the tilt)");
        // The pole in the inertial frame's axes: initial_orient * spin_axis.
        // Tilt past 90 deg flips it below the orbital plane.
        const glm::dvec3 pole = vf->initial_orient * vf->spin_axis;
        check(pole.y < 0.0, "Venus' pole is flipped (axial_tilt > 90 deg "
                            "reached initial_orient)");
        check(venus->cal.day_seconds > 0.0 && venus->cal.year_seconds > 0.0,
              "Venus' calendar stays valid (the make_solar_system.py:174 trap)");
    }

    // --- #141: the new epoch-phase fields -------------------------------
    // Defaults pin: zeroed spin_phase0/tilt_azimuth must reproduce the
    // pre-#141 Rz(axial_tilt) exactly (the loader treats 0 as absent, so
    // any system that omits them loads byte-identically).
    with_loaded(mutated(mutated(mutated(j, "Venus", "rotating", "spin_phase0", 0.0),
                                "Venus", "rotating", "tilt_azimuth", 0.0),
                        "Venus", "rotating", "axial_tilt", 0.4),
                "phase_defaults", [](System &sys) {
        Frame *vf = sys.find("Venus")->rot_frame;
        const double ct = std::cos(0.4), st = std::sin(0.4);
        const glm::dmat3 old(glm::dvec3(ct, -st, 0.0),
                             glm::dvec3(st,  ct, 0.0),
                             glm::dvec3(0.0, 0.0, 1.0));
        const glm::dmat3 &io = vf->initial_orient;
        double delta = 0.0;
        for(int c = 0; c < 3; c++) {
            delta += glm::length(io[c] - old[c]);
        }
        check(delta < 1e-12, "zeroed phase fields reproduce the "
                             "pre-#141 initial_orient exactly");
    });

    // spin_phase0 on an untilted body: the epoch longitude-0 point sits at
    // the authored rail azimuth, and the pole stays untilted (the phase is a
    // pre-rotation about the figure axis, so spin stays about +Y, #101).
    with_loaded(mutated(mutated(mutated(j, "Venus", "rotating", "axial_tilt", 0.0),
                                "Venus", "rotating", "tilt_azimuth", 0.0),
                        "Venus", "rotating", "spin_phase0", 1.234),
                "spin_phase0", [](System &sys) {
        Frame *vf = sys.find("Venus")->rot_frame;
        const glm::dvec3 lon0 = vf->initial_orient * glm::dvec3(1.0, 0.0, 0.0);
        check(std::fabs(azimuth(lon0) - 1.234) < 1e-9,
              "spin_phase0 places epoch longitude 0 at the authored azimuth");
        const glm::dvec3 pole = vf->initial_orient * vf->spin_axis;
        check(glm::length(pole - glm::dvec3(0.0, 1.0, 0.0)) < 1e-9,
              "spin_phase0 leaves the pole untilted (figure-axis pre-rotation)");
    });

    // tilt_azimuth: the lean direction swings to the authored azimuth while
    // the tilt magnitude is preserved (frees the obliquity node from the
    // ascending node).
    with_loaded(mutated(mutated(j, "Venus", "rotating", "axial_tilt", 0.3),
                        "Venus", "rotating", "tilt_azimuth", 2.0),
                "tilt_azimuth", [](System &sys) {
        Frame *vf = sys.find("Venus")->rot_frame;
        const glm::dvec3 pole = vf->initial_orient * vf->spin_axis;
        check(std::fabs(azimuth(pole) - 2.0) < 1e-9,
              "tilt_azimuth swings the lean to the authored azimuth");
        check(std::fabs(pole.y - std::cos(0.3)) < 1e-9,
              "tilt_azimuth preserves the tilt magnitude");
    });

    // --- the rejected channel: negative rates are data bugs, not retrograde
    expect_reject(mutated(j, "Triton", "inertial", "orb_ang_speed",
                          -std::fabs(authored(j, "Triton", "inertial",
                                             "orb_ang_speed"))),
                  "neg_orb", "negative orb_ang_speed");
    expect_reject(mutated(j, "Venus", "rotating", "rot_ang_speed",
                          -std::fabs(authored(j, "Venus", "rotating",
                                             "rot_ang_speed"))),
                  "neg_spin", "negative rot_ang_speed");

    // #140: the magnitude twin. 1e-300 underflows w*w so a becomes inf;
    // 1e-21 keeps a finite (~1e19 m) but far beyond the authored universe
    // bound (solar_system.json: 1e14 m).
    expect_reject(mutated(j, "Triton", "inertial", "orb_ang_speed", 1e-300),
                  "tiny_orb", "overflows to infinity");
    expect_reject(mutated(j, "Triton", "inertial", "orb_ang_speed", 1e-21),
                  "wide_orb", "universe bound");

    // Light-phase bodies own no Bullet/GL state, so teardown is safe and
    // keeps sanitizer runs quiet.
    for(TerrainBody *b : sys.bodies) { delete b; }

    if(g_failures == 0) {
        std::printf("test_retrograde: all checks passed\n");
        return 0;
    }
    std::printf("test_retrograde: %d FAILURES\n", g_failures);
    return 1;
}
