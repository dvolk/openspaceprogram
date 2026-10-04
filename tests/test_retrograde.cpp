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
// #143 authors REAL epoch orientation for the 8 planets + Pluto from the
// vendored IAU WGCCRE 2015 report, and #144 tidally locks the regular
// moons (spin = mean motion, near side facing the parent at t=0). #147
// refers regular moons' inclination to the parent's EQUATOR
// (inertial.incl_ref), so their planes tip with the parent's axis. All
// rest on ONE sky convention: rail azimuth = -ecliptic longitude (parse_
// planet negates the fact sheets' physical angles; the WGCCRE mapping is
// a proper rotation).
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
    return std::atan2(r.z, r.x);   // (-pi, pi]: compare against constants
                                   // inside this range, or wrap the delta
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
        if(b["name"] == name && b.contains(section)
           && b[section].contains(field)) {
            return b[section][field].get<double>();
        }
    }
    return 0.0;
}

// j with bodies[name][section][field] removed (exercises the loader's
// omission path, distinct from an authored 0.0).
static nlohmann::json erased(nlohmann::json j, const std::string &name,
                             const char *section, const char *field) {
    for(auto &&b : j["bodies"]) {
        if(b["name"] == name && b.contains(section)) { b[section].erase(field); }
    }
    return j;
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
        try {
            fn(sys);
        } catch(const std::exception &e) {
            std::printf("FAIL %s: check threw: %s\n", tag, e.what());
            ++g_failures;
        }
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
        // The authored calendar year must match the J2000 sky the orbital
        // phases encode (kills the old hardcoded 4724 for real systems).
        check(earth->cal.epoch_year == 2000,
              "solar_system.json authors epoch_year 2000 (clock matches sky)");
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
    // Generator<->loader loop: the shipped JSON must actually author the
    // fields (key drift would leave every system node-locked with a green
    // suite), and the loaded pole must sit at the authored tilt_azimuth.
    const double ship_phase = authored(j, "Venus", "rotating", "spin_phase0");
    const double ship_az = authored(j, "Venus", "rotating", "tilt_azimuth");
    check(ship_phase != 0.0 && ship_az != 0.0,
          "shipped solar_system.json authors the #141 fields");
    if(venus) {
        double d = azimuth(venus->rot_frame->initial_orient
                           * venus->rot_frame->spin_axis) - ship_az;
        if(d > PI) { d -= 2.0 * PI; }
        if(d < -PI) { d += 2.0 * PI; }
        check(std::fabs(d) < 1e-9,
              "loaded Venus pole azimuth == authored tilt_azimuth");
    }

    // Defaults pin: OMITTED spin_phase0/tilt_azimuth must reproduce the
    // pre-#141 Rz(axial_tilt) exactly (the omission path, not an authored
    // 0.0), so any system that leaves them out loads byte-identically.
    with_loaded(mutated(erased(erased(j, "Venus", "rotating", "spin_phase0"),
                               "Venus", "rotating", "tilt_azimuth"),
                        "Venus", "rotating", "axial_tilt", 0.4),
                "phase_defaults", [](System &loaded) {
        Frame *vf = loaded.find("Venus")->rot_frame;
        const double ct = std::cos(0.4), st = std::sin(0.4);
        const glm::dmat3 old(glm::dvec3(ct, -st, 0.0),
                             glm::dvec3(st,  ct, 0.0),
                             glm::dvec3(0.0, 0.0, 1.0));
        const glm::dmat3 &io = vf->initial_orient;
        double delta = 0.0;
        for(int c = 0; c < 3; c++) {
            delta += glm::length(io[c] - old[c]);
        }
        check(delta < 1e-12, "omitted phase fields reproduce the "
                             "pre-#141 initial_orient exactly");
    });

    // spin_phase0 on an untilted body: the epoch longitude-0 point sits at
    // the authored rail azimuth, and the pole stays untilted (the phase is a
    // pre-rotation about the figure axis, so spin stays about +Y, #101).
    with_loaded(mutated(mutated(mutated(j, "Venus", "rotating", "axial_tilt", 0.0),
                                "Venus", "rotating", "tilt_azimuth", 0.0),
                        "Venus", "rotating", "spin_phase0", 1.234),
                "spin_phase0", [](System &loaded) {
        Frame *vf = loaded.find("Venus")->rot_frame;
        const glm::dvec3 lon0 = vf->initial_orient * glm::dvec3(1.0, 0.0, 0.0);
        check(std::fabs(azimuth(lon0) - 1.234) < 1e-9,
              "spin_phase0 places epoch longitude 0 at the authored azimuth");
        const glm::dvec3 pole = vf->initial_orient * vf->spin_axis;
        check(glm::length(pole - glm::dvec3(0.0, 1.0, 0.0)) < 1e-9,
              "spin_phase0 leaves the pole untilted (figure-axis pre-rotation)");
    });

    // tilt_azimuth: the lean direction swings to the authored azimuth while
    // the tilt magnitude is preserved (frees the obliquity node from the
    // ascending node). Venus' authored spin_phase0 stays in place: the
    // figure-axis pre-rotation maps +Y to +Y, so it cannot reach the pole.
    with_loaded(mutated(mutated(j, "Venus", "rotating", "axial_tilt", 0.3),
                        "Venus", "rotating", "tilt_azimuth", 2.0),
                "tilt_azimuth", [](System &loaded) {
        Frame *vf = loaded.find("Venus")->rot_frame;
        const glm::dvec3 pole = vf->initial_orient * vf->spin_axis;
        check(std::fabs(azimuth(pole) - 2.0) < 1e-9,
              "tilt_azimuth swings the lean to the authored azimuth");
        check(std::fabs(pole.y - std::cos(0.3)) < 1e-9,
              "tilt_azimuth preserves the tilt magnitude");
    });

    // --- #143: real epoch sky (WGCCRE 2015 + fact-sheet phases) ---------
    // One convention end to end: rail azimuth = -ecliptic longitude. The
    // sharpest cross-check is seasonal: t=0 is J2000.0 (mid-January), so
    // the Sun must sit SOUTH of Earth's equator -- dot(pole, earth->sun)
    // reads about -sin(obliquity). With the pre-fix mirrored orbital
    // phases this flips sign (June in January).
    if(earth) {
        const glm::dvec3 pole = earth->rot_frame->initial_orient
                                * earth->rot_frame->spin_axis;
        sys.root->frame->UpdateOrbitRails(0.0);
        const glm::dvec3 to_sun = sys.root->frame->GetPositionRelTo(earth->frame);
        check(glm::dot(pole, glm::normalize(to_sun)) < -0.30,
              "January sun south of Earth's equator: WGCCRE orientation and "
              "orbital phases share one sky convention (#143)");
        // The rot frame hangs off the INERTIAL frame, so the authored
        // tilt_azimuth/spin_phase0 are orbital-frame angles; the generator
        // pre-rotates the sky pole by inv(orient). Skip that and the
        // universe-frame pole is off by Earth's raan (11.3 deg azimuth).
        const double eps = 23.4392911 * PI / 180.0;   // J2000 obliquity
        const glm::dvec3 want(0.0, std::cos(eps), -std::sin(eps));
        const glm::dvec3 upole = glm::normalize(earth->rot_frame->root_orient
                                                * earth->rot_frame->spin_axis);
        check(glm::dot(upole, want) > std::cos(1.0 * PI / 180.0),
              "Earth's universe-frame pole == the J2000 sky pole (#143)");
        // t=0 is 2000-01-01 00:00 and the calendar anchors midnight there:
        // the lon-0 meridian must face AWAY from the sun. The Sun's
        // January declination (~-23 deg, plus the RA/ecliptic-longitude
        // conversion) caps the dot near -cos(26 deg); evaluating W at the
        // report's 12h epoch instead flips it to +0.9 (noon at midnight).
        const glm::dvec3 lon0 = earth->rot_frame->orient
                                * glm::dvec3(1.0, 0.0, 0.0);
        check(glm::dot(lon0, glm::normalize(to_sun)) < -0.88,
              "calendar midnight at epoch: lon 0 faces away from the sun (#143)");
    }

    // --- #144: the Moon is tidally locked --------------------------------
    TerrainBody *moon = sys.find("Moon");
    check(moon != nullptr, "Moon present in solar_system.json");
    if(moon) {
        check(std::fabs(moon->rot_frame->rot_ang_speed
                        - moon->frame->orb_ang_speed)
              < 1e-12 * moon->frame->orb_ang_speed,
              "Moon spin rate == orbital rate (synchronous, #144)");
        // Longitude 0 faces Earth at t=0 and stays facing it a half period
        // later: uniform spin vs Keplerian sweep differs only by the
        // physical libration, bounded by ~2e. A non-spinning Moon would
        // read cos(quarter turn) ~ 0 at the middle sample, so this bites.
        const double e_moon = authored(j, "Moon", "inertial", "ecc");
        const double lim = std::cos(2.0 * e_moon + 0.02);
        const double Tm = 2.0 * PI / moon->frame->orb_ang_speed;
        for(double t : {0.0, Tm / 4.0, Tm / 2.0}) {
            sys.root->frame->UpdateOrbitRails(t);
            const glm::dvec3 lon0 = moon->rot_frame->orient
                                    * glm::dvec3(1.0, 0.0, 0.0);
            const glm::dvec3 to_earth = -glm::normalize(moon->frame->pos);
            check(glm::dot(lon0, to_earth) > lim,
                  "Moon near side stays Earth-facing (synchronous rail, #144)");
        }
        sys.root->frame->UpdateOrbitRails(0.0);
    }

    // --- #147: regular moons ride the parent's EQUATOR -------------------
    // The fact sheets refer regular moons' inclination to the parent's
    // equator (irregulars to the ecliptic); inertial.incl_ref == "equator"
    // makes the loader compose the moon's orient UNDER the parent's
    // rot-frame tilt. Sharpest case: Pluto's 119.5 deg obliquity puts its
    // equator ~54 deg off its orbital plane, and Charon (incl ~0 to
    // Pluto) must ride that equator. Poles are compared in the universe
    // frame (root_orient), so parent/child frames mix safely.
    auto orbit_pole = [](Frame *f) {
        return glm::normalize(f->root_orient * glm::dvec3(0.0, 1.0, 0.0));
    };
    auto spin_pole = [](TerrainBody *b) {
        return glm::normalize(b->rot_frame->root_orient
                              * b->rot_frame->spin_axis);
    };
    sys.root->frame->UpdateOrbitRails(0.0);
    TerrainBody *charon = sys.find("Charon");
    TerrainBody *pluto = sys.find("Pluto");
    check(charon && pluto, "Charon and Pluto present in solar_system.json");
    if(charon && pluto) {
        // Exact pins against the AUTHORED orb_incl (a loose threshold
        // would pass a loader that silently dropped the inclination).
        const double ic = authored(j, "Charon", "inertial", "orb_incl");
        check(std::fabs(std::acos(glm::dot(orbit_pole(charon->frame),
                                          spin_pole(pluto))) - ic) < 1e-6,
              "Charon's plane sits at its authored incl from Pluto's equator (#147)");
        // Pluto's pole is now REAL (WGCCRE Table 3; seeded pre-#147):
        // the universe-frame pole must match (alpha0, delta0) pushed
        // through the ecliptic -> rail embedding (rail = (x_ecl, z_ecl,
        // -y_ecl), the make_solar_system.py _railvec_eq convention).
        const double a0 = 132.993 * PI / 180.0, d0 = -6.163 * PI / 180.0;
        const double eps = 23.4392911 * PI / 180.0;
        const glm::dvec3 eq(std::cos(d0) * std::cos(a0),
                            std::cos(d0) * std::sin(a0),
                            std::sin(d0));
        const glm::dvec3 ecl(eq.x,
                             eq.y * std::cos(eps) + eq.z * std::sin(eps),
                             -eq.y * std::sin(eps) + eq.z * std::cos(eps));
        const glm::dvec3 want(ecl.x, ecl.z, -ecl.y);
        check(glm::dot(spin_pole(pluto), want) > std::cos(1.0 * PI / 180.0),
              "Pluto's universe-frame pole == the Table 3 sky pole (#147)");
        // The mutual lock is now EXACT: with Charon on the equator,
        // Pluto's tilt is common-mode and the lon-0 POINT sits sub-Charon
        // (pre-#147 only the meridian plane could be aligned, the point
        // staying ~54 deg off). Charon's lon 0 faces Pluto (note n).
        const glm::dvec3 to_charon = glm::normalize(
            charon->frame->root_pos - pluto->frame->root_pos);
        const glm::dvec3 plon0 = pluto->rot_frame->root_orient
                                 * glm::dvec3(1.0, 0.0, 0.0);
        check(glm::dot(plon0, to_charon) > std::cos(1.0 * PI / 180.0),
              "Pluto's lon 0 faces Charon exactly (mutual lock, #144/#147)");
        const glm::dvec3 clon0 = charon->rot_frame->root_orient
                                 * glm::dvec3(1.0, 0.0, 0.0);
        check(glm::dot(clon0, -to_charon) > std::cos(1.0 * PI / 180.0),
              "Charon's lon 0 faces Pluto (sub-Pluto meridian, #144)");
    }
    // Regular moons of tilted parents ride the equator, each at its
    // authored incl (exact pins, as above).
    struct EqPair { const char *moon, *parent; };
    for(EqPair pr : {EqPair{"Io", "Jupiter"}, EqPair{"Phobos", "Mars"},
                     EqPair{"Titan", "Saturn"}, EqPair{"Titania", "Uranus"}}) {
        TerrainBody *m = sys.find(pr.moon);
        TerrainBody *p = sys.find(pr.parent);
        check(m && p, (std::string(pr.moon) + " and " + pr.parent
                       + " present in solar_system.json").c_str());
        if(m && p) {
            const double incl = authored(j, pr.moon, "inertial", "orb_incl");
            check(std::fabs(std::acos(glm::dot(orbit_pole(m->frame),
                                              spin_pole(p))) - incl) < 1e-6,
                  (std::string(pr.moon) + "'s plane sits at its authored incl from "
                   + pr.parent + "'s equator (#147)").c_str());
        }
    }
    // ...and the ecliptic-referred rows must NOT have moved: each stays
    // at its authored incl from the PARENT'S ORBITAL pole (the old
    // reference), and far from the parent's tilted spin pole.
    if(moon && earth) {
        const double im = authored(j, "Moon", "inertial", "orb_incl");
        check(std::fabs(std::acos(glm::dot(orbit_pole(moon->frame),
                                          orbit_pole(earth->frame))) - im) < 1e-6,
              "Moon still rides the ecliptic band at its authored incl (incl_ref orbit, #147)");
        check(glm::dot(orbit_pole(moon->frame), spin_pole(earth))
              < std::cos(15.0 * PI / 180.0),
              "Moon's pole is NOT Earth's pole (not equator-referred, #147)");
    }
    if(triton && neptune) {
        const double it = authored(j, "Triton", "inertial", "orb_incl");
        check(std::fabs(std::acos(glm::dot(orbit_pole(triton->frame),
                                          orbit_pole(neptune->frame))) - it) < 1e-6,
              "Triton stays ecliptic-referred at its authored incl (irregular, #147)");
    }
    // lon_asc_node under incl_ref=equator is an azimuth IN THE EQUATOR
    // PLANE (no shipped moon has a nonzero node, so pin it synthetically):
    // pulling the node line back through the parent's equator frame must
    // read the authored angle, and the incl must sit off the EQUATOR pole.
    {
        const double node = 0.75;
        with_loaded(mutated(j, "Io", "inertial", "lon_asc_node", node),
                    "equator_node", [&](System &loaded) {
            Frame *iof = loaded.find("Io")->frame;
            Frame *peq = loaded.find("Jupiter")->rot_frame;
            const glm::dvec3 n = glm::transpose(peq->equator_orient)
                                 * (iof->orient * glm::dvec3(1.0, 0.0, 0.0));
            check(std::fabs(azimuth(n) - node) < 1e-9,
                  "equator-referred lon_asc_node reads as an azimuth in the "
                  "parent's equator plane (#147)");
            const double ang = std::acos(glm::dot(
                glm::normalize(iof->orient * glm::dvec3(0.0, 1.0, 0.0)),
                glm::normalize(peq->equator_orient * glm::dvec3(0.0, 1.0, 0.0))));
            check(std::fabs(ang - authored(j, "Io", "inertial", "orb_incl")) < 1e-9,
                  "equator-referred orb_incl measures off the equator pole (#147)");
        });
    }
    // Control: OMITTED incl_ref (the default "orbit" path) puts Charon
    // back in Pluto's ORBITAL plane -- 119.5 deg of obliquity away from
    // the equator -- so the equator pin above is discriminating, and
    // hand-authored systems load exactly as before.
    with_loaded(erased(j, "Charon", "inertial", "incl_ref"), "charon_orbit",
                [&](System &loaded) {
        TerrainBody *c = loaded.find("Charon");
        TerrainBody *p = loaded.find("Pluto");
        check(glm::dot(orbit_pole(c->frame), spin_pole(p))
              < std::cos(100.0 * PI / 180.0),
              "omitted incl_ref reproduces the old orbital-plane rail");
    });
    // A typo'd reference plane is a data bug, not a silent default.
    {
        nlohmann::json bad = j;
        for(auto &&b : bad["bodies"]) {
            if(b["name"] == "Io") { b["inertial"]["incl_ref"] = "ecliptic"; }
        }
        expect_reject(bad, "bad_incl_ref", "incl_ref");
    }

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
