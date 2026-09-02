#include "shipdef.h"

#include <cmath>
#include <fstream>
#include <map>
#include <stdexcept>

#include <nlohmann/json.hpp>

PartDef::PartDef()
    : mass(0.0), radius(1.0), height(2.0), torque(0.0), fuel_rate(0.0),
      exhaust_velocity(0.0), hull_margin(-1.0) {
    capacity.resize((int)ResourceType::Num, 0.0f);
}

const PartDef *PartsCatalog::find(const std::string &name) const {
    for(size_t i = 0; i < parts.size(); i++) {
        if(parts[i].name == name) { return &parts[i]; }
    }
    return nullptr;
}

static int resource_index_from_string(const std::string &s, const char *ctx) {
    if(s == "hydrogen") { return (int)ResourceType::Hydrogen; }
    if(s == "lox") { return (int)ResourceType::LOX; }
    if(s == "ec") { return (int)ResourceType::EC; }
    if(s == "oxygen") { return (int)ResourceType::Oxygen; }
    if(s == "water") { return (int)ResourceType::Water; }
    if(s == "food") { return (int)ResourceType::Food; }
    if(s == "hydrazine") { return (int)ResourceType::Hydrazine; }
    throw std::runtime_error(std::string(ctx) + ": unknown resource '" + s
                             + "' (expected: hydrogen, lox, ec, oxygen, water, food, hydrazine)");
}

PartsCatalog load_parts_catalog(const char *path) {
    std::ifstream f(path);
    if(!f.is_open()) {
        throw std::runtime_error(std::string("parts: cannot open ") + path);
    }
    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(f, nullptr, true);
    } catch(const std::exception &e) {
        throw std::runtime_error(std::string("parts: bad JSON in ") + path
                                 + std::string(": ") + e.what());
    }
    if(!doc.is_object() || !doc.contains("parts") || !doc["parts"].is_array()
       || doc["parts"].empty()) {
        throw std::runtime_error(std::string("parts: no parts in ") + path);
    }

    PartsCatalog cat;
    const nlohmann::json &arr = doc["parts"];
    for(size_t i = 0; i < arr.size(); i++) {
        const nlohmann::json &pv = arr[i];

        PartDef d;
        d.name = pv.value("name", std::string(""));
        if(d.name.empty()) {
            throw std::runtime_error(std::string("parts: entry ") + std::to_string(i)
                                     + " of " + path + ": missing \"name\"");
        }
        const char *ctx = (std::string("parts: ") + d.name + ": ").c_str();
        if(cat.find(d.name) != nullptr) {
            throw std::runtime_error(std::string(ctx) + "duplicate part name");
        }

        d.type = pv.value("type", std::string(""));   // free-form label (display only)
        d.mesh = pv.value("mesh", std::string(""));
        d.texture = pv.value("texture", std::string(""));
        if(d.mesh.empty() || d.texture.empty()) {
            throw std::runtime_error(std::string(ctx) + "missing \"mesh\"/\"texture\"");
        }
        d.mass = pv.value("mass", -1.0);
        if(d.mass <= 0.0) {
            throw std::runtime_error(std::string(ctx) + "\"mass\" must be > 0 (kg)");
        }

        /* size (metres): the .obj is authored to match; defaults are the
           legacy 2 m cube so pre-size parts are unchanged */
        d.radius = pv.value("radius", 1.0);
        d.height = pv.value("height", 2.0);
        if(d.radius <= 0.0) {
            throw std::runtime_error(std::string(ctx) + "\"radius\" must be > 0 (m)");
        }
        if(d.height <= 0.0) {
            throw std::runtime_error(std::string(ctx) + "\"height\" must be > 0 (m)");
        }

        /* Behavior is field-driven (see shipdef.h): each optional field is
           validated on its own, and they combine freely. A part with none
           of them is a passive mass (e.g. a bare capsule). */
        d.torque = pv.value("torque", 0.0);
        if(d.torque < 0.0) {
            throw std::runtime_error(std::string(ctx) + "\"torque\" must be >= 0 (N m)");
        }

        bool has_rate = pv.contains("fuel_rate");
        bool has_ve = pv.contains("exhaust_velocity");
        d.fuel_rate = pv.value("fuel_rate", 0.0);
        d.exhaust_velocity = pv.value("exhaust_velocity", 0.0);
        if(has_rate != has_ve) {
            throw std::runtime_error(std::string(ctx)
                                     + "\"fuel_rate\" and \"exhaust_velocity\" must be given together");
        }
        if(has_rate && (d.fuel_rate <= 0.0 || d.exhaust_velocity <= 0.0)) {
            throw std::runtime_error(std::string(ctx)
                                     + "\"fuel_rate\" and \"exhaust_velocity\" must be > 0");
        }

        if(pv.contains("capacity")) {
            if(!pv["capacity"].is_object() || pv["capacity"].empty()) {
                throw std::runtime_error(std::string(ctx)
                                         + "\"capacity\" must be a non-empty object");
            }
            for(auto it = pv["capacity"].begin(); it != pv["capacity"].end(); ++it) {
                d.capacity[resource_index_from_string(it.key(), ctx)] =
                    it.value().get<float>();
            }
            double cap_total = 0.0;
            for(size_t r = 0; r < d.capacity.size(); r++) { cap_total += d.capacity[r]; }
            if(cap_total <= 0.0) {
                throw std::runtime_error(std::string(ctx)
                                         + "\"capacity\" must total > 0 (kg)");
            }
        }

        /* hull margin (m); omitted -> -1, the physics engine then falls
           back to OSP_HULL_MARGIN / 0.1 */
        if(pv.contains("hull_margin")) {
            d.hull_margin = pv["hull_margin"].get<double>();
            if(d.hull_margin < 0.0) {
                throw std::runtime_error(std::string(ctx)
                                         + "\"hull_margin\" must be >= 0 (m)");
            }
        }

        cat.parts.push_back(d);
    }
    return cat;
}

ShipDef load_ship_def(const char *path, const PartsCatalog &catalog) {
    std::ifstream f(path);
    if(!f.is_open()) {
        throw std::runtime_error(std::string("ship: cannot open ") + path);
    }
    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(f, nullptr, true);
    } catch(const std::exception &e) {
        throw std::runtime_error(std::string("ship: bad JSON in ") + path
                                 + std::string(": ") + e.what());
    }
    if(!doc.is_object() || !doc.contains("parts") || !doc["parts"].is_array()
       || doc["parts"].empty()) {
        throw std::runtime_error(std::string("ship: no parts in ") + path);
    }

    ShipDef def;
    def.name = doc.value("name", std::string(""));
    def.controller = -1;
    def.hull_margin = -1.0;
    if(doc.contains("hull_margin")) {
        def.hull_margin = doc["hull_margin"].get<double>();
        if(def.hull_margin < 0.0) {
            throw std::runtime_error(std::string("ship: \"hull_margin\" in ") + path
                                     + " must be >= 0 (m)");
        }
    }

    const nlohmann::json &arr = doc["parts"];
    std::map<std::string, size_t> idToIndex;   // instance id -> part index (defined so far)
    std::map<std::string, int> autoCount;      // catalog name -> auto-id counter
    for(size_t i = 0; i < arr.size(); i++) {
        const nlohmann::json &pv = arr[i];

        ShipPart sp;
        sp.def = nullptr;
        sp.part = pv.value("part", std::string(""));
        if(sp.part.empty()) {
            throw std::runtime_error(std::string("ship: part entry ") + std::to_string(i)
                                     + " of " + path + ": missing \"part\"");
        }
        sp.def = catalog.find(sp.part);
        if(sp.def == nullptr) {
            std::string avail;
            for(size_t k = 0; k < catalog.parts.size(); k++) {
                if(k) { avail += ", "; }
                avail += catalog.parts[k].name;
            }
            throw std::runtime_error(std::string("ship: unknown part '") + sp.part
                                     + "' in " + path + " (catalog has: " + avail + ")");
        }

        /* instance id: explicit, or auto "<catalog name>_<n>" (n per catalog
           name, starting at 1). Must be unique within the ship. */
        sp.id = pv.value("id", std::string(""));
        if(sp.id.empty()) {
            int &n = autoCount[sp.part];
            n++;
            sp.id = sp.part + "_" + std::to_string(n);
        }
        if(idToIndex.count(sp.id)) {
            throw std::runtime_error(std::string("ship: duplicate part id '") + sp.id
                                     + "' in " + path);
        }

        /* weld parent, by id: it must already be defined (construction
           order -- the one rule that catches dangling refs AND cycles).
           Default: the previous part, so a linear stack is a bare list. */
        sp.parent = (i > 0) ? (int)(i - 1) : -1;
        if(pv.contains("parent")) {
            if(!pv["parent"].is_string()) {
                throw std::runtime_error(std::string("ship: part '") + sp.id + "' in " + path
                                         + ": \"parent\" must be a part id (string)");
            }
            std::string pid = pv["parent"].get<std::string>();
            std::map<std::string, size_t>::iterator it = idToIndex.find(pid);
            if(it == idToIndex.end()) {
                throw std::runtime_error(std::string("ship: part '") + sp.id + "' in " + path
                                         + ": parent '" + pid + "' is not defined before it");
            }
            sp.parent = (int)it->second;
        }

        sp.attach = AttachMode::Down;
        if(pv.contains("attach")) {
            if(!pv["attach"].is_string()) {
                throw std::runtime_error(std::string("ship: part '") + sp.id + "' in " + path
                                         + ": \"attach\" must be a string");
            }
            std::string m = pv["attach"].get<std::string>();
            if(m == "down") { sp.attach = AttachMode::Down; }
            else if(m == "up") { sp.attach = AttachMode::Up; }
            else if(m == "radial") { sp.attach = AttachMode::Radial; }
            else if(m == "side") { sp.attach = AttachMode::Side; }
            else {
                throw std::runtime_error(std::string("ship: part '") + sp.id + "' in " + path
                                         + ": \"attach\" must be 'down', 'up', 'radial', or 'side' (got '"
                                         + m + "')");
            }
        }

        sp.angle = pv.value("angle", 0.0);
        if(!std::isfinite(sp.angle)) {
            throw std::runtime_error(std::string("ship: part '") + sp.id + "' in " + path
                                     + ": \"angle\" must be a finite number of degrees");
        }

        sp.offset = pv.value("offset", 0.0);
        if(sp.offset < 0.0) {
            throw std::runtime_error(std::string("ship: part '") + sp.id + "' in " + path
                                     + ": \"offset\" must be >= 0 (m)");
        }

        /* stage: reserved for staging (separable stages); no runtime effect
           yet -- parsed and validated so the schema is settled. */
        sp.stage = pv.value("stage", 1);
        if(sp.stage < 1) {
            throw std::runtime_error(std::string("ship: part '") + sp.id + "' in " + path
                                     + ": \"stage\" must be >= 1");
        }

        idToIndex[sp.id] = i;
        def.parts.push_back(sp);
    }

    /* controller: a part id (resolved now that all ids are known) */
    if(doc.contains("controller")) {
        if(!doc["controller"].is_string()) {
            throw std::runtime_error(std::string("ship: \"controller\" in ") + path
                                     + " must be a part id (string)");
        }
        std::string cid = doc["controller"].get<std::string>();
        std::map<std::string, size_t>::iterator it = idToIndex.find(cid);
        if(it == idToIndex.end()) {
            throw std::runtime_error(std::string("ship: controller id '") + cid + "' in " + path
                                     + " is not a part of the ship");
        }
        def.controller = (int)it->second;
    }
    return def;
}

AttachPose attachPose(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                      const PartDef &parentDef, const PartDef &childDef,
                      AttachMode mode, double angleDeg, double offset)
{
    const double rP = parentDef.radius, hP = parentDef.height;
    const double rC = childDef.radius,  hC = childDef.height;

    /* Rz(angle): maps parent-local +X to `dir`. Angle 0 = parent +X, so a
       radial/side part at angle a sits on the parent's side at clock
       position a (90 deg = parent +Y). */
    const double a = glm::radians(angleDeg);
    const double c = cos(a), s = sin(a);
    const glm::dmat3 rz(glm::dvec3(c, s, 0.0),
                        glm::dvec3(-s, c, 0.0),
                        glm::dvec3(0.0, 0.0, 1.0));
    const glm::dvec3 dir = glm::dvec3(c, s, 0.0);

    /* child +Z -> parent +X (the --radial-test rotZtoX): columns are the
       images of X, Y, Z. */
    const glm::dmat3 rotZtoX(glm::dvec3(0.0, 0.0, -1.0),
                             glm::dvec3(0.0, 1.0, 0.0),
                             glm::dvec3(1.0, 0.0, 0.0));

    AttachPose p;
    if(mode == AttachMode::Down) {
        /* face-to-face on the parent's -Z face, shared axis */
        p.childPos  = parentPos - parentRot * glm::dvec3(0.0, 0.0, (hP + hC) / 2.0 + offset);
        p.childRot  = parentRot * rz;
        p.parentAnchor = glm::dvec3(0.0, 0.0, -(hP / 2.0 + offset));
        p.childAnchor  = glm::dvec3(0.0, 0.0,  hC / 2.0);
    }
    else if(mode == AttachMode::Up) {
        /* face-to-face on the parent's +Z face, shared axis: stacking
           OUTWARD from a radially attached part (its +Z points away from
           the ship), or a nose part above the root. The child's base face
           (-hC/2) meets the parent's top face (+hP/2). */
        p.childPos  = parentPos + parentRot * glm::dvec3(0.0, 0.0, (hP + hC) / 2.0 + offset);
        p.childRot  = parentRot * rz;
        p.parentAnchor = glm::dvec3(0.0, 0.0,  hP / 2.0 + offset);
        p.childAnchor  = glm::dvec3(0.0, 0.0, -hC / 2.0);
    }
    else if(mode == AttachMode::Radial) {
        /* child axis perpendicular: its base face (-hC/2) on the parent's
           side at radius rP, in the `dir` clock position */
        p.childPos  = parentPos + parentRot * (dir * (rP + hC / 2.0 + offset));
        p.childRot  = parentRot * rz * rotZtoX;
        p.parentAnchor = dir * (rP + offset);
        p.childAnchor  = glm::dvec3(0.0, 0.0, -hC / 2.0);
    }
    else { // Side
        /* parallel axes, side by side: surfaces meet at rP + rC in `dir` */
        p.childPos  = parentPos + parentRot * (dir * (rP + rC + offset));
        p.childRot  = parentRot * rz;
        p.parentAnchor = dir * (rP + offset);
        p.childAnchor  = glm::dvec3(-rC, 0.0, 0.0);
    }

    /* invariant (enforced by the 6DOF weld): the anchors coincide in world
       space. The parent anchor sits at the gap edge (surface + offset), so
       the gap is what the solver holds. */
    const glm::dvec3 wp = parentPos + parentRot * p.parentAnchor;
    const glm::dvec3 wc = p.childPos + p.childRot * p.childAnchor;
    if(glm::length(wp - wc) > 1e-9) {
        throw std::runtime_error("attachPose: anchors do not coincide "
                                 "(internal geometry error)");
    }
    return p;
}

double resolveHullMargin(double shipMargin, double partMargin) {
    if(shipMargin >= 0.0) { return shipMargin; }
    if(partMargin >= 0.0) { return partMargin; }
    return -1.0;
}

StageSplit computeStageSplit(size_t nParts,
                             const std::vector<std::pair<size_t,size_t>> &links,
                             const std::vector<bool> &drop)
{
    if(drop.size() != nParts) {
        throw std::runtime_error("computeStageSplit: drop.size() != nParts");
    }
    StageSplit r;
    r.newIndexOf.assign(nParts, -1);

    /* partition the parts and build the old->new index map for the keepers */
    size_t next = 0;
    for(size_t i = 0; i < nParts; i++) {
        if(drop[i]) {
            r.droppedParts.push_back(i);
        } else {
            r.keptParts.push_back(i);
            r.newIndexOf[i] = (long)next++;
        }
    }

    /* classify each constraint: any dropped end -> remove; both kept -> keep
       (remapped). The two cases are exhaustive and disjoint. */
    for(size_t c = 0; c < links.size(); c++) {
        const size_t a = links[c].first;
        const size_t b = links[c].second;
        if(a >= nParts || b >= nParts) {
            throw std::runtime_error("computeStageSplit: link endpoint out of range");
        }
        if(drop[a] || drop[b]) {
            r.cutConstraints.push_back(c);
        } else {
            r.keptLinks.push_back(std::make_pair((size_t)r.newIndexOf[a],
                                                 (size_t)r.newIndexOf[b]));
        }
    }
    return r;
}
