// test_skybox.cpp -- the system JSON "skybox" object (src/system.cpp).
//
// The star field moved from code (src/skybox.cpp used to hard-code which
// faces to use) into the system JSON, so the sky is data: every shipped
// system names its own, and a system switch can change it. The field is an
// object naming all six cubemap faces, keyed by the axis each one is
// ("+X","-X","+Y","-Y","+Z","-Z") -- keys rather than six names in GL order,
// because a swapped pair in an array loads happily and gives a mirrored sky.
// Pinned here:
//   - every committed res/systems/*.json declares all six faces,
//   - each key's image lands on ITS face (the loader reads by key, never by
//     iteration order),
//   - malformed or missing faces throw: a typo in a hand-edited JSON should
//     fail the load, not render a mystery sky.
//
// Build & run (from repo root): see Makefile ($(TESTDIR)/test_skybox).

#include "system.h"
#include "resdir.h"

#include <cstdio>
#include <cstring>
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
    const char *what = nullptr;
    try {
        load_system(tmp.c_str(), nullptr, nullptr);
    } catch(const std::exception &e) {
        what = e.what();
    }
    check(what && std::strstr(what, msg_needle), tag);
    std::remove(tmp.c_str());
}

// Six dummy images (tmp/), one per face, returned in GL order
// (+X,-X,+Y,-Y,+Z,-Z). load_system only resolves names -- the GL upload is
// Skybox::load's job, and that needs a context -- so empty files suffice.
static std::vector<std::string> make_faces() {
    std::filesystem::create_directories("tmp");
    std::vector<std::string> faces;
    for(const char *axis : { "+X", "-X", "+Y", "-Y", "+Z", "-Z" }) {
        const std::string p = std::string("tmp/test_skybox_face") + axis + ".png";
        std::ofstream(p) << 'x';
        faces.push_back(p);
    }
    return faces;
}

// A skybox object naming `faces` (GL order) under the six axis keys.
static nlohmann::json face_object(const std::vector<std::string> &faces) {
    return { { "+X", faces[0] }, { "-X", faces[1] }, { "+Y", faces[2] },
             { "-Y", faces[3] }, { "+Z", faces[4] }, { "-Z", faces[5] } };
}

int main() {
    // --- every committed system names its six faces ------------------------
    const std::vector<std::string> slugs = list_systems("res/systems");
    check(!slugs.empty(), "res/systems lists systems");
    for(const std::string &slug : slugs) {
        const std::string path = "res/systems/" + slug + ".json";
        const nlohmann::json doc = read_json(path.c_str());
        check(doc.contains("skybox") && doc["skybox"].is_object(),
              ("committed system declares a skybox object: " + slug).c_str());
        const std::vector<std::string> faces = load_faces(path);
        // WHICH sky is the author's call (pointing a system at a staged
        // bake must not turn `make test` red); that it names six images
        // that resolve from here is the rule.
        check(faces.size() == 6, ("six skybox faces: " + slug).c_str());
        for(const std::string &face : faces) {
            check(std::filesystem::exists(resdir::path(face)),
                  ("skybox face exists: " + slug + " " + face).c_str());
        }
    }

    // --- each key lands on its own face ------------------------------------
    const char *base = "res/systems/old_system.json";
    {
        const std::vector<std::string> faces = make_faces();
        nlohmann::json j = read_json(base);
        j["skybox"] = face_object(faces);
        const std::string tmp = "tmp/test_skybox_keys.json";
        std::ofstream(tmp) << j.dump();
        check(load_faces(tmp) == faces, "each axis key maps to its GL slot");
        std::remove(tmp.c_str());
        for(const std::string &p : faces) { std::remove(p.c_str()); }
    }

    // --- malformed data throws ---------------------------------------------
    {
        nlohmann::json j = read_json(base);
        j.erase("skybox");
        expect_reject(j, "missing skybox rejected", "no \"skybox\"");
    }
    {
        nlohmann::json j = read_json(base);
        j["skybox"] = { { "+X", "res/textures/skybox.png" } };
        expect_reject(j, "partial face list rejected", "needs a non-empty image name");
    }
    {   // the px/nx spelling (the bake's file names) is not the key spelling
        nlohmann::json j = read_json(base);
        j["skybox"] = { { "px", "res/textures/skybox.png" },
                        { "+X", "res/textures/skybox.png" },
                        { "-X", "res/textures/skybox.png" },
                        { "+Y", "res/textures/skybox.png" },
                        { "-Y", "res/textures/skybox.png" },
                        { "+Z", "res/textures/skybox.png" },
                        { "-Z", "res/textures/skybox.png" } };
        expect_reject(j, "unknown face key rejected", "unknown face");
    }
    {   // six names in GL order: rejected, not silently accepted (a swapped
        // pair in that form would load as a mirrored sky)
        nlohmann::json j = read_json(base);
        j["skybox"] = nlohmann::json::array();
        for(int i = 0; i < 6; i++) { j["skybox"].push_back("res/textures/skybox.png"); }
        expect_reject(j, "array form rejected", "must be an object");
    }
    {
        nlohmann::json j = read_json(base);
        j["skybox"] = "res/textures/skybox.png";
        expect_reject(j, "single-name form rejected", "must be an object");
    }
    {
        nlohmann::json j = read_json(base);
        j["skybox"] = { { "+X", 0 },
                        { "-X", "res/textures/skybox.png" },
                        { "+Y", "res/textures/skybox.png" },
                        { "-Y", "res/textures/skybox.png" },
                        { "+Z", "res/textures/skybox.png" },
                        { "-Z", "res/textures/skybox.png" } };
        expect_reject(j, "non-string face rejected", "needs a non-empty image name");
    }
    {
        nlohmann::json j = read_json(base);
        j["skybox"] = { { "+X", "res/textures/no_such_sky.png" },
                        { "-X", "res/textures/skybox.png" },
                        { "+Y", "res/textures/skybox.png" },
                        { "-Y", "res/textures/skybox.png" },
                        { "+Z", "res/textures/skybox.png" },
                        { "-Z", "res/textures/skybox.png" } };
        expect_reject(j, "missing face rejected", "does not exist");
    }

    if(g_failures == 0) { std::printf("test_skybox: OK\n"); return 0; }
    std::printf("test_skybox: %d FAILURES\n", g_failures);
    return 1;
}
