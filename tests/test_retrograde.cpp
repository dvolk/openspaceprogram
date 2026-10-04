// test_retrograde.cpp -- pins the sanctioned retrograde encodings through the
// REAL loader (load_system on res/systems/solar_system.json).
//
// The project's policy (utils/make_solar_system.py:172 and issue #139):
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
// prograde (the a = cbrt(mu/w^2) sign drop). The throw cases are tested
// below on mutated copies of the real system JSON (written to tmp/).
//
// Build & run (from repo root): see Makefile ($(TESTDIR)/test_retrograde).

#include "system.h"
#include "frame.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

static int g_failures = 0;

static void check(bool cond, const char *what) {
    if(!cond) {
        std::printf("FAIL %s\n", what);
        ++g_failures;
    }
}

// Authored orb_incl for a body, straight from the same JSON the loader read
// (so the test tracks data edits instead of hard-coding a value).
static double authored_orb_incl(const char *path, const std::string &name) {
    std::ifstream f(path);
    nlohmann::json j = nlohmann::json::parse(f);
    for(auto &&b : j["bodies"]) {
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

// Negate one rate field in a copy of the system JSON (tmp/) and require
// load_system to reject it.
static void expect_reject(const char *path, const std::string &name,
                          const char *section, const char *field,
                          const char *tag) {
    std::ifstream f(path);
    nlohmann::json j = nlohmann::json::parse(f);
    for(auto &&b : j["bodies"]) {
        if(b["name"] == name) { b[section][field] = -1e-5; }
    }
    std::filesystem::create_directories("tmp");
    const std::string tmp = std::string("tmp/test_retrograde_") + tag + ".json";
    std::ofstream o(tmp);
    o << j.dump();
    o.close();
    bool threw = false;
    try {
        load_system(tmp.c_str(), nullptr, nullptr);
    } catch(const std::runtime_error &) {
        threw = true;
    }
    std::printf("  (reject check: %s -> %s)\n", tag, threw ? "threw" : "LOADED");
    check(threw, tag);
    std::remove(tmp.c_str());
}

int main() {
    const char *sys_path = "res/systems/solar_system.json";
    System sys = load_system(sys_path, nullptr, nullptr);

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
    check(incl > M_PI_2, "Triton's authored orb_incl is retrograde (> 90 deg)");
    const glm::dvec3 h = tf->orient * glm::cross(tf->orbit_pos0, tf->orbit_vel0);
    const double h_ang = std::acos(h.y / glm::length(h));
    check(std::fabs(h_ang - incl) < 1e-6,
          "loaded orbit normal sits at the authored inclination from parent +Y");
    check(h.y < 0.0, "Triton's orbit normal is below Neptune's orbital plane "
                     "(retrograde sense reached the rail)");

    // Motion, not just geometry: prograde (h.Y > 0) sweeps the azimuth
    // atan2(z, x) DOWN (rail velocity is +Y x r_hat); retrograde sweeps it UP.
    const double T = 2.0 * M_PI / tf->orb_ang_speed;
    sys.root->frame->UpdateOrbitRails(0.0);
    const double az0 = azimuth(tf->GetPositionRelTo(nf));
    sys.root->frame->UpdateOrbitRails(T / 8.0);
    double daz = azimuth(tf->GetPositionRelTo(nf)) - az0;
    if(daz > M_PI) { daz -= 2.0 * M_PI; }
    if(daz < -M_PI) { daz += 2.0 * M_PI; }
    check(daz > 0.0, "Triton sweeps counter-clockwise about Neptune's +Y "
                     "(retrograde motion over T/8)");

    // The rail must close after exactly one authored period: pins w as the
    // mean motion of the loaded conic (inclination must not distort it).
    sys.root->frame->UpdateOrbitRails(T);
    const glm::dvec3 rT = tf->GetPositionRelTo(nf);
    sys.root->frame->UpdateOrbitRails(0.0);
    const glm::dvec3 r0 = tf->GetPositionRelTo(nf);
    check(glm::length(rT - r0) < 1e-6 * glm::length(r0),
          "Triton's rail closes after 2*pi/w");
    check(triton->cal.year_seconds > 0.9 * T && triton->cal.year_seconds < 1.1 * T,
          "Triton's calendar year matches the orbital period (Y gate survives)");

    // Prograde control: the same sweep test must read the OTHER sign for a
    // normal body, so the Triton check above is discriminating, not vacuous.
    TerrainBody *earth = sys.find("Earth");
    if(earth) {
        sys.root->frame->UpdateOrbitRails(0.0);
        const double e0 = azimuth(earth->frame->GetPositionRelTo(sys.root->frame));
        sys.root->frame->UpdateOrbitRails(M_PI / (2.0 * earth->frame->orb_ang_speed));
        double edaz = azimuth(earth->frame->GetPositionRelTo(sys.root->frame)) - e0;
        if(edaz > M_PI) { edaz -= 2.0 * M_PI; }
        if(edaz < -M_PI) { edaz += 2.0 * M_PI; }
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
              "Venus' calendar stays valid (the make_solar_system.py:172 trap)");
    }

    // --- the rejected channel: negative rates are data bugs, not retrograde
    expect_reject(sys_path, "Triton", "inertial", "orb_ang_speed", "neg_orb");
    expect_reject(sys_path, "Venus", "rotating", "rot_ang_speed", "neg_spin");

    if(g_failures == 0) {
        std::printf("test_retrograde: all checks passed\n");
        return 0;
    }
    std::printf("test_retrograde: %d FAILURES\n", g_failures);
    return 1;
}
