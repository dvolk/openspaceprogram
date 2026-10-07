// test_skybox.cpp -- the system JSON "skybox" field (src/system.cpp).
//
// The star field moved from code (src/skybox.cpp used to hard-code which
// faces to use) into the system JSON, so the sky is data: every shipped
// system names its own, and a system switch can change it. The field is a
// DIRECTORY holding skybox_px/nx/py/ny/pz/nz.png; the loader builds the six
// names itself in GL cubemap order, so the data cannot reorder them. (It
// used to be an object keyed by axis; the guard against a mirrored sky moved
// to the alignment pins in utils/skybox/skybox_common.py, which test-py runs
// against the committed faces.) Pinned here:
//   - every committed res/systems/*.json names a directory whose six faces
//     resolve from here,
//   - the loader derives them in px,nx,py,ny,pz,nz order -- skybox.cpp
//     uploads faces[i] to GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
//   - a directory outside "res/" works, which is how a staged bake is tried,
//   - malformed or missing faces throw: a typo in a hand-edited JSON should
//     fail the load, not render a mystery sky.
//
// Build & run (from repo root): see Makefile ($(TESTDIR)/test_skybox).

#include "system.h"
#include "resdir.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

static int g_failures = 0;

static void check(bool cond, const char *what) {
    if(!cond) {
        std::printf("FAIL %s\n", what);
        ++g_failures;
    }
}

// The base for the mutations: a real committed system, minus/with the one
// field under test.
static nlohmann::json read_json(const char *path) {
    std::ifstream f(path);
    if(!f.is_open()) {
        std::printf("FAIL cannot open %s (run from the repo root)\n", path);
        ++g_failures;
        return nlohmann::json::object();
    }
    return nlohmann::json::parse(f);
}

// Load `path`, expecting success. Empty vector on a throw (the caller's
// checks then fail, naming the case).
static std::vector<std::string> load_faces(const std::string &path) {
    try {
        System sys = load_system(path.c_str(), nullptr, nullptr);
        return sys.skybox_faces;
    } catch(const std::exception &e) {
        std::printf("FAIL %s: unexpected throw: %s\n", path.c_str(), e.what());
        ++g_failures;
        return {};
    }
}

// Write a mutated system (tmp/), load it, expect a throw naming msg_needle.
static void expect_reject(const nlohmann::json &j, const char *tag,
                          const char *msg_needle) {
    std::filesystem::create_directories("tmp");
    const std::string tmp = std::string("tmp/test_skybox_") + tag + ".json";
    std::ofstream o(tmp);
    o << j.dump();
    o.close();
    // Copy the message out of the catch: e.what() points into the exception,
    // so a const char* kept past its destruction reads freed memory and the
    // needle match becomes luck of the allocator.
    bool threw = false;
    std::string what;
    try {
        load_system(tmp.c_str(), nullptr, nullptr);
    } catch(const std::exception &e) {
        threw = true;
        what = e.what();
    }
    check(threw && what.find(msg_needle) != std::string::npos, tag);
    std::remove(tmp.c_str());
}

// Write a mutated system (tmp/), load it, expect success (the caller checks
// the faces).
static std::vector<std::string> load_mutated(const nlohmann::json &j,
                                             const char *tag) {
    std::filesystem::create_directories("tmp");
    const std::string tmp = std::string("tmp/test_skybox_") + tag + ".json";
    std::ofstream(tmp) << j.dump();
    const std::vector<std::string> faces = load_faces(tmp);
    std::remove(tmp.c_str());
    return faces;
}

// Six dummy images under tmp/, one per axis name, returned as the directory
// name a system JSON would hold. load_system only resolves names -- the GL
// upload is Skybox::load's job, and that needs a context -- so empty files
// suffice. Deliberately NOT under res/: a cwd-relative skybox directory is
// how a staged bake gets tried in game, so it must load.
static std::string make_face_dir() {
    const std::string dir = "tmp/test_skybox_faces";
    std::filesystem::create_directories(dir);
    for(const char *suff : { "px", "nx", "py", "ny", "pz", "nz" }) {
        std::ofstream(dir + "/skybox_" + suff + ".png") << 'x';
    }
    return dir;
}

int main() {
    // --- every committed system names a sky it can load --------------------
    const std::vector<std::string> slugs = list_systems("res/systems");
    check(!slugs.empty(), "res/systems lists systems");
    for(const std::string &slug : slugs) {
        const std::string path = "res/systems/" + slug + ".json";
        const nlohmann::json doc = read_json(path.c_str());
        check(doc.contains("skybox") && doc["skybox"].is_string(),
              ("committed system names a skybox directory: " + slug).c_str());
        const std::vector<std::string> faces = load_faces(path);
        // WHICH sky is the author's call (pointing a system at a staged
        // bake must not turn `make test` red); that it resolves to six
        // images from here is the rule.
        check(faces.size() == 6, ("six skybox faces: " + slug).c_str());
        for(const std::string &face : faces) {
            check(std::filesystem::exists(resdir::path(face)),
                  ("skybox face exists: " + slug + " " + face).c_str());
        }
    }

    // --- the loader fixes the face order, not the data ---------------------
    const char *base = "res/systems/old_system.json";
    {
        const std::string dir = make_face_dir();
        nlohmann::json j = read_json(base);
        j["skybox"] = dir;
        const std::vector<std::string> want = { dir + "/skybox_px.png",
            dir + "/skybox_nx.png", dir + "/skybox_py.png",
            dir + "/skybox_ny.png", dir + "/skybox_pz.png",
            dir + "/skybox_nz.png" };
        check(load_mutated(j, "order") == want,
              "six names derived in GL cubemap order");
        // A trailing slash is the same directory, not a missing face.
        j["skybox"] = dir + "/";
        check(load_mutated(j, "slash") == want, "trailing slash tolerated");
        std::filesystem::remove_all(dir);
    }

    // --- malformed data throws ---------------------------------------------
    {
        nlohmann::json j = read_json(base);
        j.erase("skybox");
        expect_reject(j, "missing skybox rejected", "no \"skybox\"");
    }
    {   // the old axis-keyed object form is gone, not quietly accepted
        nlohmann::json j = read_json(base);
        j["skybox"] = { { "+X", "res/skybox/v1/skybox_px.png" },
                        { "-X", "res/skybox/v1/skybox_nx.png" },
                        { "+Y", "res/skybox/v1/skybox_py.png" },
                        { "-Y", "res/skybox/v1/skybox_ny.png" },
                        { "+Z", "res/skybox/v1/skybox_pz.png" },
                        { "-Z", "res/skybox/v1/skybox_nz.png" } };
        expect_reject(j, "object form rejected", "must be a directory name");
    }
    {   // six names in GL order: never accepted (a swapped pair in that form
        // would load as a mirrored sky)
        nlohmann::json j = read_json(base);
        j["skybox"] = nlohmann::json::array();
        for(const char *suff : { "px", "nx", "py", "ny", "pz", "nz" }) {
            j["skybox"].push_back(std::string("res/skybox/v1/skybox_") + suff + ".png");
        }
        expect_reject(j, "array form rejected", "must be a directory name");
    }
    {
        nlohmann::json j = read_json(base);
        j["skybox"] = "";
        expect_reject(j, "empty directory rejected", "must be a directory name");
    }
    {
        nlohmann::json j = read_json(base);
        j["skybox"] = 0;
        expect_reject(j, "non-string skybox rejected", "must be a directory name");
    }
    {
        nlohmann::json j = read_json(base);
        j["skybox"] = "res/skybox/no_such_set";
        expect_reject(j, "missing directory rejected", "does not exist");
    }
    {   // one face short: the directory exists, the set does not
        const std::string dir = make_face_dir();
        std::filesystem::remove(dir + "/skybox_nz.png");
        nlohmann::json j = read_json(base);
        j["skybox"] = dir;
        expect_reject(j, "missing face rejected", "does not exist");
        std::filesystem::remove_all(dir);
    }

    if(g_failures == 0) { std::printf("test_skybox: OK\n"); return 0; }
    std::printf("test_skybox: %d FAILURES\n", g_failures);
    return 1;
}
