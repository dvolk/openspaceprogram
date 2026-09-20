#include "shipdef.h"

#include <cmath>
#include <fstream>
#include <map>
#include <stdexcept>

#include <glm/gtc/quaternion.hpp>   // angleAxis / mat3_cast (attachNodes)
#include <nlohmann/json.hpp>

PartDef::PartDef()
    : mass(0.0), radius(1.0), height(2.0), torque(0.0), fuel_rate(0.0),
      exhaust_velocity(0.0), jet(false), jet_fan_thrust(0.0),
      jet_intake_area(0.0), rcs_thrust(0.0), power_draw(0.0),
      power_draw_constant(0.0), power_gen(0.0),
      crew_capacity(0), decoupler(false), docking_port(false),
      fuel_barrier(false), fuel_link(false), hull_margin(-1.0),
      drag(0.0), drag_forward(0.0), drag_side(0.0), drag_backward(0.0),
      lift_area(0.0), cl(0.0), stall_angle(0.0),
      control_area(0.0), control_axis(ControlAxis::Pitch), cl_control(0.0),
      max_deflection(0.0) {
    capacity.resize((int)ResourceType::Num, 0.0f);
}

const PartDef *PartsCatalog::find(const std::string &name) const {
    for(size_t i = 0; i < parts.size(); i++) {
        if(parts[i].name == name) { return &parts[i]; }
    }
    return nullptr;
}

static int resource_index_from_string(const std::string &s, const std::string &ctx) {
    if(s == "hydrogen") { return (int)ResourceType::Hydrogen; }
    if(s == "lox") { return (int)ResourceType::LOX; }
    if(s == "ec") { return (int)ResourceType::EC; }
    if(s == "oxygen") { return (int)ResourceType::Oxygen; }
    if(s == "water") { return (int)ResourceType::Water; }
    if(s == "food") { return (int)ResourceType::Food; }
    if(s == "hydrazine") { return (int)ResourceType::Hydrazine; }
    if(s == "jetfuel") { return (int)ResourceType::JetFuel; }
    throw std::runtime_error(ctx + ": unknown resource '" + s
                             + "' (expected: hydrogen, lox, ec, oxygen, water, food, hydrazine, jetfuel)");
}

/* Read a 3-element numeric array (a node's pos/dir) or throw with context. */
static glm::dvec3 parse_vec3(const nlohmann::json &obj, const char *key,
                             const std::string &ctx) {
    if(!obj.contains(key) || !obj[key].is_array() || obj[key].size() != 3) {
        throw std::runtime_error(ctx + std::string("\"") + key
                                 + "\" must be a 3-element array [x, y, z]");
    }
    const nlohmann::json &a = obj[key];
    for(int k = 0; k < 3; k++) {
        if(!a[k].is_number()) {
            throw std::runtime_error(ctx + std::string("\"") + key
                                     + "\" must hold three numbers");
        }
    }
    return glm::dvec3(a[0].get<double>(), a[1].get<double>(), a[2].get<double>());
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
        const std::string ctx = "parts: " + d.name + ": ";
        if(cat.find(d.name) != nullptr) {
            throw std::runtime_error(ctx + "duplicate part name");
        }

        d.type = pv.value("type", std::string(""));   // free-form label (display only)
        // human-readable name (display only); optional, empty -> the UI
        // falls back to the machine `name`
        d.display_name = pv.value("display_name", std::string(""));

        /* fuel link: a virtual (no-mesh) one-way fuel connection. It is a
           marker entry -- no geometry, no mass, so the mesh/texture/mass/
           size validation below is skipped. (It still needs a catalog
           entry so ship defs can reference it by name.) */
        d.fuel_link = pv.value("fuel_link", false);

        if(!d.fuel_link) {
            d.mesh = pv.value("mesh", std::string(""));
            d.texture = pv.value("texture", std::string(""));
            if(d.mesh.empty() || d.texture.empty()) {
                throw std::runtime_error(ctx + "missing \"mesh\"/\"texture\"");
            }
            /* Engine shroud (optional, see PartDef.shroud): the pair is
               all-or-nothing -- a half-set shroud is a catalog bug, not a
               configuration. */
            d.shroud = pv.value("shroud", std::string(""));
            d.shroud_texture = pv.value("shroud_texture", std::string(""));
            if((d.shroud.empty()) != (d.shroud_texture.empty())) {
                throw std::runtime_error(ctx
                                         + "\"shroud\" and \"shroud_texture\" must be set together");
            }
            d.mass = pv.value("mass", -1.0);
            if(d.mass <= 0.0) {
                throw std::runtime_error(ctx + "\"mass\" must be > 0 (kg)");
            }

            /* size (metres): the .obj is authored to match; defaults are
               the legacy 2 m cube so pre-size parts are unchanged */
            d.radius = pv.value("radius", 1.0);
            d.height = pv.value("height", 2.0);
            if(d.radius <= 0.0) {
                throw std::runtime_error(ctx + "\"radius\" must be > 0 (m)");
            }
            if(d.height <= 0.0) {
                throw std::runtime_error(ctx + "\"height\" must be > 0 (m)");
            }
        }

        /* Behavior is field-driven (see shipdef.h): each optional field is
           validated on its own, and they combine freely. A part with none
           of them is a passive mass (e.g. a bare capsule). */
        d.torque = pv.value("torque", 0.0);
        if(d.torque < 0.0) {
            throw std::runtime_error(ctx + "\"torque\" must be >= 0 (N m)");
        }

        /* RCS translation authority (N); > 0 -> the part contributes to the
           ship's RCS (burns hydrazine mono, applied at the COM). Independent
           of the other behavior fields -- a part may be a wheel AND an RCS. */
        d.rcs_thrust = pv.value("rcs_thrust", 0.0);
        if(d.rcs_thrust < 0.0) {
            throw std::runtime_error(ctx + "\"rcs_thrust\" must be >= 0 (N)");
        }

        /* electrical (KSP-style EC): power_draw (W) is a part's draw while
           active (a reaction wheel); power_draw_constant (W) is a draw that
           runs all the time (capsule life support); power_gen (W) is a
           constant source (an RTG). Independent, each optional, each >= 0.
           A battery is capacity[EC] > 0 (parsed with the capacity object
           below); a capsule carries both a constant draw and its own small
           built-in battery. */
        d.power_draw = pv.value("power_draw", 0.0);
        if(d.power_draw < 0.0) {
            throw std::runtime_error(ctx + "\"power_draw\" must be >= 0 (W)");
        }
        d.power_draw_constant = pv.value("power_draw_constant", 0.0);
        if(d.power_draw_constant < 0.0) {
            throw std::runtime_error(ctx + "\"power_draw_constant\" must be >= 0 (W)");
        }
        d.power_gen = pv.value("power_gen", 0.0);
        if(d.power_gen < 0.0) {
            throw std::runtime_error(ctx + "\"power_gen\" must be >= 0 (W)");
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

        /* jet engine (air-breathing) modifier: a flag + the air-breathing
           parameters (see PartDef.jet / drag.h jetThrust). Omitted -> not a
           jet, and the parameters keep their defaults (harmless). A jet
           without a thrust source (fuel_rate + exhaust_velocity) is a load
           error: the flag alone does nothing. */
        if(pv.contains("jet")) {
            d.jet = pv["jet"].get<bool>();
        }
        d.jet_fan_thrust = pv.value("jet_fan_thrust", 0.0);
        if(d.jet_fan_thrust < 0.0) {
            throw std::runtime_error(ctx + "\"jet_fan_thrust\" must be >= 0 (N)");
        }
        d.jet_intake_area = pv.value("jet_intake_area", 0.0);
        if(d.jet_intake_area < 0.0) {
            throw std::runtime_error(ctx + "\"jet_intake_area\" must be >= 0 (m^2)");
        }
        if(d.jet && !(d.fuel_rate > 0.0 && d.exhaust_velocity > 0.0)) {
            throw std::runtime_error(ctx + "\"jet\" requires \"fuel_rate\" "
                                          "and \"exhaust_velocity\" (the real exhaust velocity)");
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

        /* crew capacity (int); > 0 marks a capsule (holds that many EVA
           characters, see PartDef.crew_capacity); omitted -> 0 */
        if(pv.contains("crew_capacity")) {
            d.crew_capacity = pv["crew_capacity"].get<int>();
            if(d.crew_capacity < 0) {
                throw std::runtime_error(std::string(ctx)
                                         + "\"crew_capacity\" must be >= 0");
            }
        }

        /* decoupler (bool); a staging boundary (see PartDef.decoupler).
           Omitted -> false. */
        if(pv.contains("decoupler")) {
            d.decoupler = pv["decoupler"].get<bool>();
        }

        /* fuel barrier (bool); fuel does not flow across a barrier, so it
           splits fuel groups (see PartDef.fuel_barrier). Omitted -> false.
           A decoupler is a fuel barrier by definition -- force it so the
           flag can't be silently lost by a stale catalog regen. */
        if(pv.contains("fuel_barrier")) {
            d.fuel_barrier = pv["fuel_barrier"].get<bool>();
        }
        if(d.decoupler) { d.fuel_barrier = true; }

        /* docking port (bool); an end face that can lock to another port
           (see PartDef.docking_port). Omitted -> false. A docking port is a
           fuel barrier by definition -- a boundary between two ships' fuel
           systems -- force it, like the decoupler, so the flag can't be
           silently lost by a stale catalog. */
        if(pv.contains("docking_port")) {
            d.docking_port = pv["docking_port"].get<bool>();
        }
        if(d.docking_port) { d.fuel_barrier = true; }

        /* hull margin (m); omitted -> -1, the physics engine then falls
           back to OSP_HULL_MARGIN / 0.1 */
        if(pv.contains("hull_margin")) {
            d.hull_margin = pv["hull_margin"].get<double>();
            if(d.hull_margin < 0.0) {
                throw std::runtime_error(std::string(ctx)
                                         + "\"hull_margin\" must be >= 0 (m)");
            }
        }

        /* aerodynamics (src/drag.h). Each optional and >= 0; omitted -> 0.
           Drag (R2: silhouette area x per-part shape): `drag` is the part's
           drag COEFFICIENT (dimensionless, its shape's bluntness) -- the
           symmetric default. `drag_forward` / `drag_side` / `drag_backward`
           override it for the three ways the part can face the flow (nose /
           broadside / base into the flow; each defaults to `drag`). The
           ship's drag blends the parts' cds by the area each shows to the
           flow, over the ship's hull silhouette (Vehicle::applyAeroForce).
           Lift terms: 0 = no lift (a rocket stays a rocket); a lifting
           surface sets lift_area and cl. */
        d.drag = pv.value("drag", 0.0);
        if(d.drag < 0.0) {
            throw std::runtime_error(ctx +
                "\"drag\" must be >= 0 (dimensionless drag coefficient)");
        }
        d.drag_forward = pv.value("drag_forward", d.drag);
        if(d.drag_forward < 0.0) {
            throw std::runtime_error(ctx +
                "\"drag_forward\" must be >= 0 (dimensionless)");
        }
        d.drag_side = pv.value("drag_side", d.drag);
        if(d.drag_side < 0.0) {
            throw std::runtime_error(ctx +
                "\"drag_side\" must be >= 0 (dimensionless)");
        }
        d.drag_backward = pv.value("drag_backward", d.drag);
        if(d.drag_backward < 0.0) {
            throw std::runtime_error(ctx +
                "\"drag_backward\" must be >= 0 (dimensionless)");
        }
        d.lift_area = pv.value("lift_area", 0.0);
        if(d.lift_area < 0.0) {
            throw std::runtime_error(ctx + "\"lift_area\" must be >= 0 (m^2)");
        }
        d.cl = pv.value("cl", 0.0);
        if(d.cl < 0.0) {
            throw std::runtime_error(ctx + "\"cl\" must be >= 0");
        }
        d.stall_angle = pv.value("stall_angle", 0.0);
        if(d.stall_angle < 0.0) {
            throw std::runtime_error(ctx + "\"stall_angle\" must be >= 0 (rad)");
        }
        d.control_area = pv.value("control_area", 0.0);
        if(d.control_area < 0.0) {
            throw std::runtime_error(ctx + "\"control_area\" must be >= 0 (m^2)");
        }
        // control_axis: the one steering axis the surface acts on. A string
        // (pitch|yaw|roll) so a part reads like the other part fields; an
        // unknown value is a load error, not a silent default.
        const std::string axis = pv.value("control_axis", std::string("pitch"));
        if(axis == "pitch")      { d.control_axis = ControlAxis::Pitch; }
        else if(axis == "yaw")   { d.control_axis = ControlAxis::Yaw; }
        else if(axis == "roll")  { d.control_axis = ControlAxis::Roll; }
        else {
            throw std::runtime_error(ctx + "\"control_axis\" must be "
                                          "\"pitch\", \"yaw\" or \"roll\"");
        }
        d.cl_control = pv.value("cl_control", 0.0);
        if(d.cl_control < 0.0) {
            throw std::runtime_error(ctx + "\"cl_control\" must be >= 0");
        }
        d.max_deflection = pv.value("max_deflection", 0.0);
        if(d.max_deflection < 0.0) {
            throw std::runtime_error(ctx + "\"max_deflection\" must be >= 0 (rad)");
        }

        /* Attachment nodes. An explicit "nodes" array wins; otherwise (the
           common case) synthesize the two axial stack faces from the size, so
           an axis-aligned cylinder part needs no node authoring and existing
           catalogs are unchanged. Fuel links are virtual (no geometry) and
           get none. */
        if(!d.fuel_link) {
            if(pv.contains("nodes")) {
                if(!pv["nodes"].is_array()) {
                    throw std::runtime_error(ctx + "\"nodes\" must be an array");
                }
                const nlohmann::json &na = pv["nodes"];
                for(size_t k = 0; k < na.size(); k++) {
                    const nlohmann::json &nv = na[k];
                    const std::string nctx = ctx + "node " + std::to_string(k) + ": ";
                    Node nd;
                    nd.id = nv.value("id", std::string(""));
                    if(nd.id.empty()) {
                        throw std::runtime_error(nctx + "missing \"id\"");
                    }
                    if(d.findNode(nd.id) != nullptr) {
                        throw std::runtime_error(ctx + "duplicate node id \"" + nd.id + "\"");
                    }
                    nd.pos = parse_vec3(nv, "pos", ctx + "node \"" + nd.id + "\": ");
                    nd.dir = parse_vec3(nv, "dir", ctx + "node \"" + nd.id + "\": ");
                    const double len = glm::length(nd.dir);
                    if(len < 1e-9) {
                        throw std::runtime_error(ctx + "node \"" + nd.id
                                                 + "\": \"dir\" must be non-zero");
                    }
                    nd.dir /= len;   // nodes mate by direction; keep them unit
                    d.nodes.push_back(nd);
                }
            }
            d.synthesizeNodes();   // no-op if explicit nodes were given
        }

        cat.parts.push_back(d);
    }
    return cat;
}

ShipDef shipDefFromJson(const nlohmann::json &doc, const PartsCatalog &catalog,
                        const std::string &path) {
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
            else if(m == "surface") { sp.attach = AttachMode::Surface; }
            else {
                throw std::runtime_error(std::string("ship: part '") + sp.id + "' in " + path
                                         + ": \"attach\" must be 'down', 'up', or 'surface' (got '"
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

        /* Resolve the edge. A STACK edge (down/up) mates two named nodes; a
           SURFACE edge places the child's surface node at a contact point on
           the parent. Fuel links are virtual (no edge); the root has none.
           Node ids / contacts are validated here so a typo is a load error,
           not a null deref in build_ship. The parent is an earlier part
           (construction order), so it is already in def.parts. */
        sp.parentNode = pv.value("parentNode", std::string(""));
        sp.childNode  = pv.value("childNode", std::string(""));
        sp.roll          = pv.value("roll", 0.0);
        sp.contactPoint  = glm::dvec3(0.0);
        sp.contactNormal = glm::dvec3(0.0);
        if(!sp.isFuelLink() && sp.parent >= 0) {
            const std::string ectx = std::string("ship: part '") + sp.id + "' in "
                                     + path + ": ";
            const ShipPart &pp = def.parts[(size_t)sp.parent];
            if(sp.isStackEdge()) {
                /* An explicit "parentNode"/"childNode" wins; otherwise default
                   to the synthesized axial faces (down: parent bottom / child
                   top; up: parent top / child bottom), so attach:down works. */
                const bool down = (sp.attach == AttachMode::Down);
                if(sp.parentNode.empty()) { sp.parentNode = down ? "bottom" : "top"; }
                if(sp.childNode.empty())  { sp.childNode  = down ? "top" : "bottom"; }
                if(pp.def && !pp.def->findNode(sp.parentNode)) {
                    throw std::runtime_error(ectx + "parentNode '" + sp.parentNode
                                             + "' is not a node of parent part '" + pp.part + "'");
                }
                if(!sp.def->findNode(sp.childNode)) {
                    throw std::runtime_error(ectx + "childNode '" + sp.childNode
                                             + "' is not a node of part '" + sp.part + "'");
                }
            }
            else if(sp.isSurfaceEdge()) {
                /* Contact in the PARENT's local frame: an explicit point+normal
                   (what the editor's raycast writes), or the cylinder shorthand
                   -- clock `angle` [+ height `z`] on the parent's radius. */
                if(pv.contains("point") || pv.contains("normal")) {
                    sp.contactPoint  = parse_vec3(pv, "point",  ectx);
                    sp.contactNormal = parse_vec3(pv, "normal", ectx);
                } else {
                    const double rP = (pp.def != nullptr) ? pp.def->radius : 1.0;
                    const double z  = pv.value("z", 0.0);
                    const double a  = glm::radians(sp.angle);
                    sp.contactNormal = glm::dvec3(cos(a), sin(a), 0.0);
                    sp.contactPoint  = glm::dvec3(rP * cos(a), rP * sin(a), z);
                }
                const double nl = glm::length(sp.contactNormal);
                if(nl < 1e-9) {
                    throw std::runtime_error(ectx + "surface \"normal\" must be non-zero");
                }
                sp.contactNormal /= nl;
                /* The child attaches by its surface node (default) or a named one. */
                if(sp.childNode.empty()) {
                    const Node *srf = sp.def->findSurfaceNode();
                    if(srf == nullptr) {
                        throw std::runtime_error(ectx + "part '" + sp.part
                                                 + "' has no surface node to attach with");
                    }
                    sp.childNode = srf->id;
                } else if(!sp.def->findNode(sp.childNode)) {
                    throw std::runtime_error(ectx + "childNode '" + sp.childNode
                                             + "' is not a node of part '" + sp.part + "'");
                }
            }
        }

        /* stage: reserved for staging (separable stages); no runtime effect
           yet -- parsed and validated so the schema is settled. */
        sp.stage = pv.value("stage", 1);
        if(sp.stage < 1) {
            throw std::runtime_error(std::string("ship: part '") + sp.id + "' in " + path
                                     + ": \"stage\" must be >= 1");
        }

        /* fuel link: from/to (the two parts it connects, by instance id).
           Required, distinct, not the link's own id. The ids are resolved
           to Part* at build time (after all parts exist), so here I only
           check the strings. parent/attach/angle/offset/stage are ignored
           for a fuel link (it is virtual -- not welded). */
        if(sp.isFuelLink()) {
            sp.from = pv.value("from", std::string(""));
            sp.to = pv.value("to", std::string(""));
            if(sp.from.empty() || sp.to.empty()) {
                throw std::runtime_error(std::string("ship: part '") + sp.id + "' in " + path
                                         + ": a fuel link needs \"from\" and \"to\"");
            }
            if(sp.from == sp.to) {
                throw std::runtime_error(std::string("ship: part '") + sp.id + "' in " + path
                                         + ": a fuel link's \"from\" and \"to\" must differ");
            }
            if(sp.from == sp.id || sp.to == sp.id) {
                throw std::runtime_error(std::string("ship: part '") + sp.id + "' in " + path
                                         + ": a fuel link cannot reference itself");
            }
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
    return shipDefFromJson(doc, catalog, path);
}

/* The rotation taking unit direction `a` onto unit direction `b` by the
   shortest arc. The roll about that arc is the caller's to resolve (see
   attachNodes). The anti-parallel case picks a deterministic perpendicular so
   the result never depends on floating-point whim. */
static glm::dmat3 rotationFromTo(const glm::dvec3 &a, const glm::dvec3 &b) {
    const double d = glm::clamp(glm::dot(a, b), -1.0, 1.0);
    if(d > 1.0 - 1e-12) { return glm::dmat3(1.0); }   // already aligned
    if(d < -1.0 + 1e-12) {                            // opposed: 180 about a perpendicular
        const glm::dvec3 axis = glm::normalize(
            (std::fabs(a.x) < 0.9) ? glm::cross(a, glm::dvec3(1.0, 0.0, 0.0))
                                   : glm::cross(a, glm::dvec3(0.0, 1.0, 0.0)));
        return glm::mat3_cast(glm::angleAxis(std::acos(-1.0), axis));
    }
    const glm::dvec3 axis = glm::normalize(glm::cross(a, b));
    return glm::mat3_cast(glm::angleAxis(std::acos(d), axis));
}

AttachPose attachNodes(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                       const Node &parentNode, const Node &childNode,
                       double rollDeg, double offset)
{
    const glm::dvec3 dP = glm::normalize(parentNode.dir);   // parent-local outward
    const glm::dvec3 dC = glm::normalize(childNode.dir);    // child-local outward
    const glm::dvec3 target = -dP;                          // child dir opposes parent's

    /* Relative rotation (child frame w.r.t. the parent frame): the minimal
       arc taking the child's node dir onto the opposed parent dir, then the
       authored roll about that mating axis. For the synthesized axial nodes
       the arc is the identity, so the child inherits the parent's full
       orientation -- which is what makes this match the old procedural
       Down/Up exactly. */
    glm::dmat3 rrel = rotationFromTo(dC, target);
    if(rollDeg != 0.0) {
        rrel = glm::mat3_cast(glm::angleAxis(glm::radians(rollDeg), target)) * rrel;
    }

    AttachPose p;
    p.childRot = parentRot * rrel;
    /* Coincide the node positions, pushed apart by `offset` along the parent
       node dir:  childPos + childRot*childNode.pos
                  == parentPos + parentRot*(parentNode.pos + dP*offset). */
    const glm::dvec3 contact = parentPos + parentRot * (parentNode.pos + dP * offset);
    p.childPos = contact - p.childRot * childNode.pos;
    return p;
}

AttachPose attachSurface(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                         const glm::dvec3 &point, const glm::dvec3 &normal,
                         const Node &childNode, double rollDeg, double offset)
{
    /* Surface attach IS node mating: a synthetic parent node at the contact
       (position = the contact point, direction = the outward normal). The
       child's surface node dir points inward, so attachNodes anti-aligns it
       onto the normal exactly as it would a stack node -- one solver, no
       separate surface geometry to drift. */
    Node contact;
    contact.id  = "srf-contact";
    contact.pos = point;
    contact.dir = normal;   // attachNodes normalizes
    return attachNodes(parentPos, parentRot, contact, childNode, rollDeg, offset);
}

double snapAngleDeg(double deg) {
    return std::round(deg / kSnapAngDeg) * kSnapAngDeg;
}

double gridStepDeg(double cur, double delta) {
    if(delta > 0.0) {
        return std::floor(cur / kSnapAngDeg + 1e-9) * kSnapAngDeg + kSnapAngDeg;
    }
    return std::ceil(cur / kSnapAngDeg - 1e-9) * kSnapAngDeg - kSnapAngDeg;
}

/* degrees wrapped to [-180, 180) */
static double wrap180Deg(double deg) {
    double d = std::fmod(deg + 180.0, 360.0);
    if(d < 0.0) { d += 360.0; }
    return d - 180.0;
}

void snapSurfaceContact(glm::dvec3 &point, glm::dvec3 &normal,
                        bool snapLen, bool snapAng)
{
    const glm::dvec3 Z(0.0, 0.0, 1.0);
    if(snapLen) {
        point.z = std::round(point.z / kSnapLenM) * kSnapLenM;
    }
    if(!snapAng) { return; }
    const double r = std::hypot(point.x, point.y);
    const bool havePt = (r > 1e-4);
    double pa = 0.0, pa2 = 0.0;
    if(havePt) {
        pa = std::atan2(point.y, point.x);
        pa2 = glm::radians(snapAngleDeg(glm::degrees(pa)));
        point = glm::mat3_cast(glm::angleAxis(pa2 - pa, Z)) * point;
    }
    /* The normal's azimuth: the pick hull is faceted, so a flat facet's
       normal stays constant while the hit POSITION sweeps several degrees
       across it -- rotating the normal by the point's snap delta (or
       snapping its raw facet angle) would cant the part against the
       snapped position and make it counter-rotate between grid points.
       Parts are surfaces of revolution about their own axis, where the
       true normal azimuth equals the contact azimuth: when the two raw
       azimuths agree (within a facet's span), the normal takes the
       point's SNAPPED azimuth, keeping its polar tilt. Only a genuinely
       non-revolution contact (azimuths divergent, e.g. a wing plate)
       snaps the normal's own azimuth. */
    const double rn = std::hypot(normal.x, normal.y);
    if(rn > 1e-6) {
        const double an = std::atan2(normal.y, normal.x);
        double target;
        if(havePt
           && std::fabs(wrap180Deg(glm::degrees(an) - glm::degrees(pa))) < 20.0) {
            target = pa2;
        } else {
            target = glm::radians(snapAngleDeg(glm::degrees(an)));
        }
        normal = glm::mat3_cast(glm::angleAxis(target - an, Z)) * normal;
    }
}

std::vector<SymClone> radialSymmetryClones(const glm::dvec3 &parentPos,
                                           const glm::dmat3 &parentRot,
                                           const Node &childNode,
                                           const glm::dvec3 &point,
                                           const glm::dvec3 &normal,
                                           double rollDeg, double offset,
                                           int n)
{
    std::vector<SymClone> out;
    if(n <= 1) { return out; }
    const AttachPose primary = attachSurface(parentPos, parentRot, point,
                                             normal, childNode, rollDeg, offset);
    const glm::dvec3 axisL(0.0, 0.0, 1.0);          // the parent's long axis
    const glm::dvec3 axisS = parentRot * axisL;     // ... in the shared frame
    const glm::dvec3 nS = parentRot * glm::normalize(normal);
    const double step = 2.0 * std::acos(-1.0) / (double)n;
    for(int k = 1; k < n; k++) {
        const double th = step * (double)k;
        const glm::dmat3 RzL = glm::mat3_cast(glm::angleAxis(th, axisL));
        SymClone c;
        c.edge.point  = RzL * point;
        c.edge.normal = RzL * normal;
        c.edge.rollDeg = rollDeg;
        /* The congruent target is the primary's pose rotated about the
           parent's axis. Solving the rotated contact with the SAME roll
           lands there only when the minimal arc to the rotated normal
           equals the rotated minimal arc (true for radial contacts);
           in general the two differ by a roll about the mating axis, so
           measure that residual and fold it into the clone's roll. */
        const glm::dmat3 RzS = glm::mat3_cast(glm::angleAxis(th, axisS));
        const glm::dmat3 wantRot = RzS * primary.childRot;
        const AttachPose guess = attachSurface(parentPos, parentRot,
                                               c.edge.point, c.edge.normal,
                                               childNode, rollDeg, offset);
        // childRot(roll + d) == Rot(-nS_k, d) * childRot(roll), so the
        // residual D = wantRot * guess^T is exactly Rot(axis, psi).
        const glm::dvec3 axis = -(RzS * nS);
        const glm::dmat3 D = wantRot * glm::transpose(guess.childRot);
        glm::dvec3 u = glm::cross(axis, glm::dvec3(1.0, 0.0, 0.0));
        if(glm::dot(u, u) < 1e-12) { u = glm::cross(axis, glm::dvec3(0.0, 1.0, 0.0)); }
        u = glm::normalize(u);
        const glm::dvec3 Du = D * u;
        const double psi = std::atan2(glm::dot(glm::cross(axis, u), Du),
                                      glm::dot(u, Du));
        c.edge.rollDeg = rollDeg + glm::degrees(psi);
        c.pose = attachSurface(parentPos, parentRot, c.edge.point,
                               c.edge.normal, childNode, c.edge.rollDeg, offset);
        out.push_back(c);
    }
    return out;
}

AttachPose attachPose(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                      const PartDef &parentDef, const PartDef &childDef,
                      AttachMode mode, double angleDeg, double offset)
{
    /* Stack modes mate the axial nodes -- attachNodes is the single source of
       stack-attach geometry. `angleDeg` is the roll about the stack axis.
       Surface edges go through attachSurface (point + normal), not here. */
    if(mode != AttachMode::Down && mode != AttachMode::Up) {
        throw std::runtime_error("attachPose: only stack modes (down/up); a "
                                 "surface edge uses attachSurface(point + normal)");
    }
    const bool down = (mode == AttachMode::Down);
    const Node *pn = parentDef.findNode(down ? "bottom" : "top");
    const Node *cn = childDef.findNode(down ? "top" : "bottom");
    if(pn == nullptr || cn == nullptr) {
        throw std::runtime_error(std::string("attachPose: ")
                                 + (pn == nullptr ? parentDef.name : childDef.name)
                                 + " has no axial stack node (a part that declares "
                                   "explicit nodes must include top/bottom to be "
                                   "stacked with attach down/up)");
    }
    return attachNodes(parentPos, parentRot, *pn, *cn, angleDeg, offset);
}

AttachPose solveEdge(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                     const PartDef &parentDef, const PartDef &childDef,
                     AttachMode mode,
                     const std::string &parentNode, const std::string &childNode,
                     const glm::dvec3 &contactPoint, const glm::dvec3 &contactNormal,
                     double angleDeg, double rollDeg, double offset)
{
    if(mode == AttachMode::Down || mode == AttachMode::Up) {
        const Node *pn = parentDef.findNode(parentNode);
        const Node *cn = childDef.findNode(childNode);
        if(pn == nullptr || cn == nullptr) {
            throw std::runtime_error(std::string("solveEdge: stack edge references a "
                                                 "missing node (parent '") + parentNode
                                     + "' / child '" + childNode + "')");
        }
        return attachNodes(parentPos, parentRot, *pn, *cn, angleDeg, offset);
    }
    // surface edge
    const Node *cn = childDef.findNode(childNode);
    if(cn == nullptr) {
        throw std::runtime_error(std::string("solveEdge: surface edge references a "
                                             "missing child node '") + childNode + "'");
    }
    return attachSurface(parentPos, parentRot, contactPoint, contactNormal,
                         *cn, rollDeg, offset);
}

void BuildShip::recomputePoses() {
    for(size_t i = 0; i < parts.size(); i++) {
        BuildPart &bp = parts[i];
        if(bp.parent < 0) {
            bp.localPos = glm::dvec3(0.0);
            bp.localRot = glm::dmat3(1.0);
            continue;
        }
        const BuildPart &pp = parts[(size_t)bp.parent];
        const AttachPose ap = solveEdge(pp.localPos, pp.localRot,
                                        *pp.def, *bp.def, bp.attach,
                                        bp.parentNode, bp.childNode,
                                        bp.contactPoint, bp.contactNormal,
                                        bp.angle, bp.roll, bp.offset);
        bp.localPos = ap.childPos;
        bp.localRot = ap.childRot;
    }
}

bool BuildShip::nodeOccupied(int partIdx, const std::string &nodeId) const {
    for(size_t i = 0; i < parts.size(); i++) {
        const BuildPart &p = parts[i];
        if(p.attach == AttachMode::Surface) { continue; }
        if(p.parent == partIdx && p.parentNode == nodeId) { return true; }
        if((int)i == partIdx && p.childNode == nodeId) { return true; }
    }
    return false;
}

BuildShip BuildShip::fromShipDef(const ShipDef &def) {
    BuildShip bs;
    bs.name = def.name;
    bs.hull_margin = def.hull_margin;
    if(def.controller >= 0 && (size_t)def.controller < def.parts.size()) {
        bs.controllerId = def.parts[(size_t)def.controller].id;
    }
    /* physical parts only; remap parent indices as fuel links are dropped
       from the TREE (they are kept as FuelLink records for the round trip,
       same partition build_ship does). */
    std::map<size_t, size_t> physIndex;   // def.parts index -> build index
    for(size_t i = 0; i < def.parts.size(); i++) {
        const ShipPart &sp = def.parts[i];
        if(sp.isFuelLink()) {
            bs.fuelLinks.push_back(FuelLink{sp.def, sp.id, sp.from, sp.to});
            continue;
        }
        physIndex[i] = bs.parts.size();
        BuildPart bp;
        bp.def = sp.def;
        bp.id = sp.id;
        bp.attach = sp.attach;
        bp.parentNode = sp.parentNode;
        bp.childNode = sp.childNode;
        bp.contactPoint = sp.contactPoint;
        bp.contactNormal = sp.contactNormal;
        bp.angle = sp.angle;
        bp.roll = sp.roll;
        bp.offset = sp.offset;
        bp.stage = sp.stage;
        if(sp.parent >= 0) {
            auto it = physIndex.find((size_t)sp.parent);
            bp.parent = (it != physIndex.end()) ? (int)it->second : -1;
        }
        bs.parts.push_back(bp);
    }
    bs.recomputePoses();
    return bs;
}

/* Mark part `idx` and all its descendants. Construction order puts every
   parent before its children, so one forward pass suffices. */
static std::vector<bool> subtreeMask(const std::vector<BuildPart> &parts,
                                     int idx) {
    std::vector<bool> sub(parts.size(), false);
    sub[(size_t)idx] = true;
    for(size_t i = 1; i < parts.size(); i++) {
        if(parts[i].parent >= 0 && sub[(size_t)parts[i].parent]) { sub[i] = true; }
    }
    return sub;
}

bool BuildShip::removePart(int idx) {
    if(idx <= 0 || idx >= (int)parts.size()) { return false; }   // root stays
    const std::vector<bool> dead = subtreeMask(parts, idx);
    std::vector<int> remap(parts.size(), -1);
    std::vector<BuildPart> keep;
    keep.reserve(parts.size());
    for(size_t i = 0; i < parts.size(); i++) {
        if(dead[i]) { continue; }
        remap[i] = (int)keep.size();
        keep.push_back(parts[i]);
    }
    for(size_t k = 0; k < keep.size(); k++) {
        if(keep[k].parent >= 0) { keep[k].parent = remap[(size_t)keep[k].parent]; }
    }
    parts.swap(keep);
    /* purge fuel links whose endpoint died with the subtree */
    std::vector<FuelLink> liveLinks;
    for(size_t i = 0; i < fuelLinks.size(); i++) {
        const FuelLink &fl = fuelLinks[i];
        bool haveFrom = false, haveTo = false;
        for(size_t k = 0; k < parts.size(); k++) {
            if(parts[k].id == fl.from) { haveFrom = true; }
            if(parts[k].id == fl.to)   { haveTo = true; }
        }
        if(haveFrom && haveTo) { liveLinks.push_back(fl); }
    }
    fuelLinks.swap(liveLinks);
    recomputePoses();
    return true;
}

BuildShip BuildShip::detachSubtree(int idx) {
    BuildShip out;
    if(idx <= 0 || idx >= (int)parts.size()) { return out; }   // root stays
    const std::vector<bool> sub = subtreeMask(parts, idx);

    /* the detached tree, in construction order (idx is the subtree's
       ancestor, so it comes first and becomes the new root) */
    out.name = name;
    out.hull_margin = hull_margin;
    std::map<size_t, size_t> toNew;   // old index -> out index
    for(size_t i = 0; i < parts.size(); i++) {
        if(!sub[i]) { continue; }
        toNew[i] = out.parts.size();
        BuildPart bp = parts[i];
        if((int)i == idx) {
            bp.parent = -1;   // the new root: its edge is re-made at graft
            bp.attach = AttachMode::Down;
            bp.parentNode.clear();
            bp.childNode.clear();
            bp.contactPoint = glm::dvec3(0.0);
            bp.contactNormal = glm::dvec3(0.0);
            bp.angle = 0.0;
            bp.roll = 0.0;
            bp.offset = 0.0;
        } else {
            bp.parent = (int)toNew[(size_t)parts[i].parent];
        }
        out.parts.push_back(bp);
    }

    /* fuel links: both endpoints inside -> move with the assembly;
       boundary-crossing -> dropped (a detached pipe feeds nothing) */
    std::vector<FuelLink> stay;
    for(size_t i = 0; i < fuelLinks.size(); i++) {
        const FuelLink &fl = fuelLinks[i];
        bool fromIn = false, toIn = false, fromKnown = false, toKnown = false;
        for(size_t k = 0; k < parts.size(); k++) {
            if(parts[k].id == fl.from) { fromKnown = true; fromIn = sub[k]; }
            if(parts[k].id == fl.to)   { toKnown = true;   toIn = sub[k]; }
        }
        if(fromIn && toIn) { out.fuelLinks.push_back(fl); }
        else if(fromKnown && toKnown && !fromIn && !toIn) { stay.push_back(fl); }
    }
    fuelLinks.swap(stay);

    // the explicit controller follows its part
    if(!controllerId.empty()) {
        for(size_t i = 0; i < parts.size(); i++) {
            if(parts[i].id == controllerId && sub[i]) {
                out.controllerId = controllerId;
                controllerId.clear();
                break;
            }
        }
    }

    /* the survivors: remove the subtree + remap (same pass removePart does) */
    std::vector<int> remap(parts.size(), -1);
    std::vector<BuildPart> keep;
    keep.reserve(parts.size());
    for(size_t i = 0; i < parts.size(); i++) {
        if(sub[i]) { continue; }
        remap[i] = (int)keep.size();
        keep.push_back(parts[i]);
    }
    for(size_t k = 0; k < keep.size(); k++) {
        if(keep[k].parent >= 0) { keep[k].parent = remap[(size_t)keep[k].parent]; }
    }
    parts.swap(keep);
    recomputePoses();
    out.recomputePoses();
    return out;
}

namespace {
/* An id free among the parts: the base when unused, else "<base>_<n>". */
std::string uniquePartId(const std::vector<BuildPart> &parts,
                         const std::string &base) {
    std::string id = base;
    for(int n = 2;; n++) {
        bool dup = false;
        for(size_t i = 0; i < parts.size(); i++) {
            if(parts[i].id == id) { dup = true; break; }
        }
        if(!dup) { return id; }
        id = base + "_" + std::to_string(n);
    }
}
std::string uniqueLinkId(const std::vector<BuildShip::FuelLink> &links,
                         const std::string &base) {
    std::string id = base;
    for(int n = 2;; n++) {
        bool dup = false;
        for(size_t i = 0; i < links.size(); i++) {
            if(links[i].id == id) { dup = true; break; }
        }
        if(!dup) { return id; }
        id = base + "_" + std::to_string(n);
    }
}
} // namespace

size_t BuildShip::graftTree(const BuildShip &sub, const BuildPart &root) {
    if(sub.parts.empty()) { return (size_t)-1; }
    std::map<std::string, std::string> idMap;   // sub id -> grafted id
    const size_t rootIdx = parts.size();

    BuildPart rp = root;
    rp.id = uniquePartId(parts, root.id.empty()
                         ? std::string(sub.parts[0].def != nullptr
                                       ? sub.parts[0].def->name : "part")
                         : root.id);
    idMap[sub.parts[0].id] = rp.id;
    parts.push_back(rp);
    for(size_t i = 1; i < sub.parts.size(); i++) {
        BuildPart bp = sub.parts[i];
        const std::string newId = uniquePartId(parts, bp.id);
        idMap[bp.id] = newId;
        bp.id = newId;
        // sub is in construction order, so every parent index maps by the
        // same offset (the assembly root lands on rootIdx)
        bp.parent = (int)(rootIdx + (size_t)bp.parent);
        parts.push_back(bp);
    }
    for(size_t i = 0; i < sub.fuelLinks.size(); i++) {
        const FuelLink &fl = sub.fuelLinks[i];
        FuelLink nl = fl;
        nl.id = uniqueLinkId(fuelLinks, fl.id);
        std::map<std::string, std::string>::const_iterator f = idMap.find(fl.from);
        std::map<std::string, std::string>::const_iterator t = idMap.find(fl.to);
        nl.from = (f != idMap.end()) ? f->second : fl.from;
        nl.to   = (t != idMap.end()) ? t->second : fl.to;
        fuelLinks.push_back(nl);
    }
    recomputePoses();
    return rootIdx;
}

void BuildShip::rotatePart(int idx, double deltaDeg) {
    if(idx <= 0 || idx >= (int)parts.size()) { return; }   // root has no edge
    BuildPart &bp = parts[(size_t)idx];
    if(bp.parent < 0) { return; }
    if(bp.attach == AttachMode::Surface) { bp.roll += deltaDeg; }
    else { bp.angle += deltaDeg; }
    recomputePoses();
}

ShipDef BuildShip::toShipDef() const {
    ShipDef def;
    def.name = name;
    def.hull_margin = hull_margin;
    def.controller = -1;
    for(size_t i = 0; i < parts.size(); i++) {
        const BuildPart &bp = parts[i];
        ShipPart sp{};
        sp.part = (bp.def != nullptr) ? bp.def->name : std::string("");
        sp.id = bp.id;
        sp.def = bp.def;
        sp.parent = bp.parent;
        sp.attach = bp.attach;
        sp.angle = bp.angle;
        sp.offset = bp.offset;
        sp.parentNode = bp.parentNode;
        sp.childNode = bp.childNode;
        sp.contactPoint = bp.contactPoint;
        sp.contactNormal = bp.contactNormal;
        sp.roll = bp.roll;
        sp.stage = bp.stage;
        if(!controllerId.empty() && bp.id == controllerId) { def.controller = (int)i; }
        def.parts.push_back(sp);
    }
    /* fuel links re-appended after the physical parts (they are virtual, so
       their position is free; the tail keeps parent defaults sane). A link
       whose endpoint part was deleted is dropped. */
    for(size_t k = 0; k < fuelLinks.size(); k++) {
        const FuelLink &fl = fuelLinks[k];
        bool haveFrom = false, haveTo = false;
        for(size_t i = 0; i < parts.size(); i++) {
            if(parts[i].id == fl.from) { haveFrom = true; }
            if(parts[i].id == fl.to)   { haveTo = true; }
        }
        if(!haveFrom || !haveTo) { continue; }
        ShipPart sp{};
        sp.part = (fl.def != nullptr) ? fl.def->name : std::string("fuel_link");
        sp.id = fl.id;
        sp.def = fl.def;
        sp.parent = -1;
        sp.attach = AttachMode::Down;   // ignored for a link
        sp.stage = 1;
        sp.from = fl.from;
        sp.to = fl.to;
        def.parts.push_back(sp);
    }
    return def;
}

/* degrees -> [0, 360), for tidy saved files (the solvers are wrap-agnostic) */
static double wrap360(double deg) {
    double a = std::fmod(deg, 360.0);
    if(a < 0.0) { a += 360.0; }
    return a;
}

bool save_ship_def(const BuildShip &bs, const char *path) {
    if(bs.parts.empty()) { return false; }
    const ShipDef def = bs.toShipDef();

    nlohmann::ordered_json doc;
    doc["name"] = def.name;
    if(def.controller >= 0 && (size_t)def.controller < def.parts.size()) {
        doc["controller"] = def.parts[(size_t)def.controller].id;
    }
    if(def.hull_margin >= 0.0) { doc["hull_margin"] = def.hull_margin; }
    nlohmann::ordered_json arr = nlohmann::ordered_json::array();
    for(size_t i = 0; i < def.parts.size(); i++) {
        const ShipPart &sp = def.parts[i];
        nlohmann::ordered_json pv;
        pv["part"] = sp.part;
        pv["id"] = sp.id;
        if(sp.isFuelLink()) {
            pv["from"] = sp.from;
            pv["to"] = sp.to;
            arr.push_back(pv);
            continue;
        }
        // the root is part 0 (no "parent" key); every other part names its
        // parent explicitly (the load default -- previous part -- is a trap
        // for edited trees)
        if(sp.parent >= 0 && (size_t)sp.parent < def.parts.size()) {
            pv["parent"] = def.parts[(size_t)sp.parent].id;
        }
        pv["attach"] = (sp.attach == AttachMode::Surface) ? "surface"
                     : (sp.attach == AttachMode::Up) ? "up" : "down";
        if(sp.isStackEdge()) {
            // the root's ids are empty (no edge); omit empty keys so the
            // load defaults apply exactly as for a hand-written file
            if(!sp.parentNode.empty()) { pv["parentNode"] = sp.parentNode; }
            if(!sp.childNode.empty())  { pv["childNode"] = sp.childNode; }
            pv["angle"] = wrap360(sp.angle);
        } else {
            pv["childNode"] = sp.childNode;
            pv["point"] = { sp.contactPoint.x, sp.contactPoint.y, sp.contactPoint.z };
            pv["normal"] = { sp.contactNormal.x, sp.contactNormal.y, sp.contactNormal.z };
            pv["roll"] = wrap360(sp.roll);
        }
        if(sp.offset != 0.0) { pv["offset"] = sp.offset; }
        if(sp.stage != 1) { pv["stage"] = sp.stage; }
        arr.push_back(pv);
    }
    doc["parts"] = arr;

    std::ofstream f(path);
    if(!f.is_open()) { return false; }
    f << doc.dump(2) << "\n";
    f.flush();
    return !f.fail();
}

double resolveHullMargin(double shipMargin, double partMargin) {
    if(shipMargin >= 0.0) { return shipMargin; }
    if(partMargin >= 0.0) { return partMargin; }
    return -1.0;
}
