// part.cpp -- minting a Part from a catalog def (see part.h).
#include "part.h"

#include <stdexcept>
#include <string>
#include <vector>

#include "body.h"     // create_part_body
#include "mesh.h"     // get_mesh
#include "resdir.h"   // resdir::path
#include "shader.h"   // Shader
#include "shipdef.h"  // load_ship_def, PartDef, PartsCatalog, resolveHullMargin
#include "texture.h"  // get_texture

Part *make_part(const PartDef &pd, double mass, double hull_margin,
                const std::vector<double> *fuel,
                const std::vector<Experiment> *experiments,
                const std::string &id, Shader *partsshader)
{
    Mesh *mesh = get_mesh(std::string("res/") + pd.mesh);
    Texture *tex = get_texture(std::string("res/") + pd.texture);
    Part *p = new Part;
    p->body = create_part_body(mesh, partsshader, tex, (float)mass, hull_margin);
    p->def = &pd;
    p->id = id;
    /* The chute canopy and the engine shroud are resolved at build time, so every
       path resolves them. Only a chute def has the first and only an engine the
       second today; a def that grows one cannot be forgotten here. The shroud
       especially: it was resolved ONLY on the def-spawn path, so every loaded
       engine lost the cylinder that hides what it shrouds (#278). */
    if(!pd.chute_mesh.empty()) {
        p->chute_mesh = get_mesh(std::string("res/") + pd.chute_mesh);
        p->chute_texture = get_texture(std::string("res/") + pd.chute_texture);
    }
    if(!pd.shroud.empty()) {
        p->shroud = get_mesh(std::string("res/") + pd.shroud);
        p->shroud_texture = get_texture(std::string("res/") + pd.shroud_texture);
    }
    for(int r = 0; r < (int)ResourceType::Num; r++) {
        p->resources.capacity[r] = pd.capacity[r];
        p->resources.current[r] = (fuel != nullptr && r < (int)fuel->size())
            ? (float)(*fuel)[r] : 0.0f;
    }
    if(experiments != nullptr) { p->experiments = *experiments; }
    return p;
}

/* Mint a suit Part from a kerbal ship def (the occupant of a seat, part.h). The
   Body exists to carry mass + render assets; it never enters the physics world,
   because a seat's mass rides the containment edge (Part::effectiveMass), not
   the capsule's compound. */
Part *make_suit(const std::string &defPath, const PartsCatalog &cat,
                Shader *partsshader)
{
    ShipDef def = load_ship_def(resdir::path(defPath).c_str(), cat);
    /* A seat is exactly ONE Part, and it must be a kerbal-type part: that is
       what makes it a seat rather than an inventory item (Part::isSeat). A
       multi-part def would silently drop the rest, and parking a non-kerbal
       part in a capsule breaks checkPartInvariants -- a hand-edited save's
       defPath must be refused here, not asserted on two calls later. */
    if(def.parts.size() != 1) {
        throw std::runtime_error(std::string("suit: '") + defPath
                                 + "' is not exactly one part (got "
                                 + std::to_string(def.parts.size()) + ")");
    }
    const PartDef *pd = def.parts[0].def;
    if(pd->type != "kerbal") {
        throw std::runtime_error(std::string("suit: '") + defPath + "' is a '"
                                 + pd->type + "' part, not a kerbal suit");
    }
    // A fresh part takes the def's own mass and the def's ship-level margin.
    Part *suit = make_part(*pd, pd->mass,
                           resolveHullMargin(def.hull_margin, pd->hull_margin),
                           nullptr, nullptr, def.parts[0].id, partsshader);
    /* Seeded like Vehicle::init does for a tank: a partly-burned suit re-seeds
       full here, which is right for a fresh crew member. */
    if(suit->isTank()) {
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            suit->resources.current[r] = suit->resources.capacity[r];
        }
    }
    return suit;
}
