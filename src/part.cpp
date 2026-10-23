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

Part *make_part(const PartDef &pd, double hull_margin,
                const std::vector<double> *fuel,
                const std::vector<Experiment> *experiments,
                const std::string &id, Shader *partsshader)
{
    /* A physical part always weighs what its def says. The catalog refuses
       mass <= 0 (shipdef.cpp); only a virtual fuel_link def has none, and it
       never reaches here -- it is a fuel-group edge, never a Body. */
    assert(pd.mass > 0.0);
    Mesh *mesh = get_mesh(std::string("res/") + pd.mesh);
    Texture *tex = get_texture(std::string("res/") + pd.texture);
    Part *p = new Part;
    p->body = create_part_body(mesh, partsshader, tex, (float)pd.mass, hull_margin);
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
        // No cast: SavePart.fuel is double and the store is double (#379), so
        // this was the last narrowing on the load path -- half a ULP at 12 kWh
        // is one substep of charge, thrown away on every load.
        p->resources.current[r] = (fuel != nullptr && r < (int)fuel->size())
            ? (*fuel)[r] : 0.0;
    }
    if(experiments != nullptr) { p->experiments = *experiments; }
    return p;
}

/* Load and validate the def a NEW suit is minted from. The two checks are the
   whole contract make_suit relies on, so they run once per def, not once per
   suit (#291). */
ShipDef load_suit_def(const std::string &defPath, const PartsCatalog &cat)
{
    ShipDef def = load_ship_def(resdir::path(defPath).c_str(), cat);
    /* A seat is exactly ONE Part, and it must be a kerbal-type part: that is
       what makes it a seat rather than an inventory item
       (Part::isContainedPerson). A multi-part def would silently drop the rest,
       and parking a non-kerbal part in a capsule breaks checkPartInvariants --
       a hand-edited save's defPath must be refused here, not asserted on two
       calls later. */
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
    return def;
}

Part *make_suit(const ShipDef &def, Shader *partsshader)
{
    /* The contract load_suit_def establishes, asserted before the deref. A
       ShipDef that skipped load_suit_def can still carry a null def: toShipDef
       copies BuildPart::def, and a BuildPart whose part the catalog lost has
       one. */
    assert(def.parts.size() == 1 && def.parts[0].def != nullptr
           && def.parts[0].def->type == "kerbal");
    const PartDef *pd = def.parts[0].def;
    // A fresh part takes the def's ship-level margin; its mass is the def's.
    Part *suit = make_part(*pd,
                           resolveHullMargin(def.hull_margin, pd->hull_margin),
                           nullptr, nullptr, def.parts[0].id, partsshader);
    /* Seeded like Vehicle::init does for a tank: a partly-burned suit re-seeds
       full here, which is right for a fresh crew member. */
    if(suit->isTank()) { suit->fillTanks(); }
    return suit;
}
