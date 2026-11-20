// staging.h -- VAB staging analysis: per-stage delta-v and TWR, fuel links
// (asparagus) included. Pure math over BuildShip; the burn model matches
// flight (see staging.cpp).

#pragma once

#include <cassert>
#include <cstdint>
#include <string>
#include <vector>

#include "shipdef.h"   // BuildShip, PartDef, ResourceType

// One row of the staging table (a stage-number period, flight order).
struct StageRow {
    int stage = 1;
    double deltaV = 0.0;     // m/s, vacuum
    double minTWR = 0.0;     // TWR at ignition (heaviest)
    double maxTWR = 0.0;     // TWR at the drop (lightest)
    double massStart = 0.0;  // kg, at ignition
    double massEnd = 0.0;    // kg, at the drop
    int engines = 0;         // rocket engines firing
    bool drops = false;      // separates parts (vs a pure burn)
};

/* Per-part seeding, for staging a ship that EXISTS rather than one being
   authored (#948). A flight Vehicle holds what it holds: tanks partly drained,
   crew and inventory riding the hull. Index-aligned with ship.parts; an empty
   member keeps the as-designed figure (full tanks, PartDef dry mass).
   The caller owns the split: pass fuelKg = what the part holds of the burnable
   resources (see stagingPropellantMask) and dryKg = everything else about that
   part, so dryKg + fuelKg equals the mass the ship really carries for it. */
struct StagingSeed {
    std::vector<double> fuelKg;
    std::vector<double> dryKg;
};

// Simulate the staged burn of `ship` against surface
// gravity `g` (0 skips TWR). `exhaust_scale` scales thrust only, never the
// fuel burn. Stage periods: the labels that actually separate something, PLUS the
// lowest label whether or not it separates (the walk spans minStage..maxStage and
// skips a label with nothing to separate -- so the HIGHEST label gets no row at all
// unless it carries a decoupler, which is why a ship can fly N labels while the table
// lists fewer rows; pinned in tests/test_staging_dv.cpp by test_a_gap_costs_nothing).
// Without `seed`, tanks start FULL -- the VAB's question.
std::vector<StageRow> computeStaging(const BuildShip &ship, double g,
                                     double exhaust_scale = 1.0,
                                     const StagingSeed &seed = {});

/* The build grouped by authored label, ASCENDING: which parts ride each label and what
   they answer to at a press (the StageRole bits, shipdef.h). One pass, two readers --
   validateStaging's rules below and the Staging ladder's rungs (#1022, which draws a rung
   per group and lists only the parts its `responds` bit says answer) -- so the
   grouping is not walked twice per edit. Parts inside a label keep build order, which is
   parent before child (shipdef.h:610-612), so reading a group reads like the rocket;
   reading the groups back-to-front gives the press order, because the counter starts at
   the highest label. A label the build carries but no press can reach (the lowest) is
   still a group: the parts are there, and the view has to be able to say so. */
struct StagingLabel {
    int stage = 1;
    unsigned responds = 0u;   // stageRoles() OR'd over the group; 0 = nothing answers
    std::vector<int> parts;   // indices into ship.parts, build order
};
std::vector<StagingLabel> stagingLabels(const BuildShip &ship);

/* Three words saying where the press counter stands relative to `stage`, for a build whose
   labels run from `lo` to `hi` (a StagingLabel vector's ends -- the same edges finalize()
   seeds, vehicle.cpp:2457-2458): "press 3 of 4", "the floor", or for a one-label build
   "only label: press 1".

   NOBODY PRINTS THIS TODAY. It was the Staging window's cell (#1011); #1022 turned that
   window into a ladder and dropped the prose with it, so what is left is the arithmetic and
   the test that pins it. The candidate reader is #1015's part-window number box, which asks
   the same question about one part and currently accepts a value up to 10000 while the ladder
   it sits beside is four rungs long. If #1015 does not want it either, delete this rather than
   keep a tested sentence nobody says.

   The arithmetic is the part worth keeping, and it can be WRONG: an advancing press acts at
   the value the counter already holds and only then steps (game.cpp:2612 reads activeStage,
   :2702 steps), so the presses that stand anywhere are hi, hi-1, ... lo+1 and the label at lo
   waits for a press past them. Deliberately short, for the same reason as before: whoever
   prints it is printing into a narrow column, and a sentence that clips mid-thought is worse
   than none. */
std::string stagingPressNote(int lo, int hi, int stage);

/* Which resources the staged burn may spend: the union over the build's rocket
   engines, jets excluded. A caller filling StagingSeed::fuelKg MUST weigh the
   part with this same rule, or a live ship's rows disagree with the VAB's.
   Same #41 limitation as computeStaging: ship-wide, not per fuel group. */
void stagingPropellantMask(const BuildShip &ship, bool *burn);

/* What a build's authored labels DO, as opposed to what computeStaging says they
   COST (#1010). Appends one short human-readable line per problem to `warns`
   (empty after the call means the order works); it does not sort or clear the
   caller's list. It computes the grouping itself -- (#1022) took the VAB's second
   reader away, so there is no longer a caller that has the groups on hand and must
   not pay for the walk twice; the grouping-taking form is static in staging.cpp and
   this is the only entry point anyone uses: --fleet-report, --stage-dump, and the
   tests.

   A label is a position on the press counter and nothing else, so every rule here
   is about presses: the counter starts at the highest label and steps down one
   integer per press, and the presses a ship can make are the integers hi down to
   lo+1. Reports a press that does nothing (a gap, or a label whose parts answer to
   nothing), a DECOUPLER riding the lowest label (the counter never stands there, so
   it needs a press past the last one counted -- a chute there is the normal way to
   author one, so it is not reported), and a decoupler whose side is every part in
   the ship (the split is refused and the label is spent, #541). Warn-only and
   deliberately so -- see the comment at the definition for what a refuse would
   punish today and which rule is the candidate. */
void validateStaging(const BuildShip &ship, std::vector<std::string> &warns);

/* Number the labels from the tree instead of by hand (#1012). A placed part gets label 1
   and stays there until a player types over it, so a two-stage rocket is currently built
   by hand-numbering every part -- and getting it wrong is quiet, which is what #1010's
   checker exists to notice. The tree already knows the answer, because a label is only
   ever a position on the press counter:

   Order the cuts by the pair (the author's label, the part's DEPTH from the tree root),
   biggest first = earliest press (#1020). Where EVERY cut carries a number, that order IS the
   answer: an asparagus hull empties its radial boosters on a fuel schedule the tree cannot
   see, and heavy_asp is the shape -- six boosters hang off one core tank at the SAME tree
   depth, so a depth-only sort drops all six on one press and throws away the authored 6/5/4
   pairing that keeps opposite boosters dropping together. Half-numbered is the fallback, not
   the mixed case: a placed part sits at label 1 until the player types over it, so a fresh cut
   would read as "the last press" and land on the floor rather than where the tree puts it.
   Where the build was placed and never numbered, every authored label is equal and the pair
   falls back to depth: the deepest cut is the one the first press makes, because the counter
   starts high. Depth is also what #540 uses to order decouplers WITHIN one press; ordering
   cuts ACROSS presses is where the author's number wins. Give each distinct key one label running
   from hi = levels + 1 down to 2, and give every other part the label of the decoupler that
   separates it -- walking up its parent chain. A decoupler separates ITSELF: the press drops
   the decoupler with its stage (Vehicle::droppedPartsAtStage), so it rides its own label.
   What nothing separates gets the floor, 1 -- which is why no decoupler can land there, and
   why the labels come out contiguous: an auto-staged build leaves #1010's checker nothing to
   say, and --fleet-report checks that over every shipped hull. Re-spacing is dense, which is
   also how a stray 9 in the middle of a ladder stops being seven dead presses (#1023).

   Decouplers at the same key SHARE a label on purpose. A ring of four is one press, and
   it is shipped, not hypothetical -- #540 counts them on 4 of 32 hulls.

   No cut may ride a press INSIDE the press of a cut that contains it. The outer press takes the
   inner cut with it (droppedPartsAtStage spans the whole subtree), so an authored pair running the
   other way across a containment relation is not an order the stack can fly: the outer number is
   read down to the inner one, and the depth tie-break then puts the inner cut higher -- the only
   order that works. Depth gets this for free; authored numbers are clamped to it, and the plan
   says how many it bent rather than bending them silently. A bent number moves that cut relative to
   the unrelated cuts it used to sit beside, and can split a press the author shared or add a rung,
   so the too-many-rungs refusal is measured AFTER the repair.

   An engine the build lights FROM THE PAD stays lit at the pad, pinned to the plan's hi. The
   tree says when a part FALLS and never when an engine STARTS, and those are two different
   rockets: heavy_asp burns engine_central through all six booster separations, while the cut
   that releases its tanks is three presses later. Nothing is pinned when the build carries one
   label everywhere -- that is the case autoStage exists for, with no intent to contradict. The
   pin reads ONE part's number while the ordering insists on all of them, deliberately: a sequence
   needs every member numbered to mean anything, and "this engine lights at the pad" is one part's
   own statement. Two limits on it: a part that is ALSO a cut or a chute is left alone (that label
   belongs to the separation, and a pinned chute would open on the first press), and a pin standing
   on the floor is declined once only one part would be left there -- an empty label 1 leaves the
   lowest label the build carries being a cut, which is #1010 rule 2. A part reaches the floor only
   when NO cut is above it, so an engine given up this way lights only on a press past the ladder:
   both choices cost the player something, and the plan protects the ladder. Pins standing anywhere
   else are unaffected.

   Refuses rather than guesses: leaves the build exactly as it was and says why in `notes`.
   The refusals are the places where the tree cannot see the player's intent, or cannot read
   the stack at all:

   * no decoupler at all -- one label is the answer, and renumbering would invent presses;
   * a parent chain that loops (a self-parent: nothing could ever separate those parts);
   * a decoupler whose side is every part -- the press is refused in flight, so the label
     would be spent on nothing (#541);
   * a build whose every root is a decoupler, which would leave the LOWEST label carrying a
     cut -- exactly what #1010 rule 2 reports;
   * more cut levels than the label range (1..kMaxStage), past which a save would not load;
   * an engine pair the build ordered that the tree would re-sequence. (An engine the build
     lights from the pad is NOT a refusal any more -- the plan keeps it lit at the pad, above.)

   A chute is left with its part even when that lands it on the floor: a floor chute is the
   normal way to author one (#1010 rule 2), so `notes` says what the player has to do about
   it rather than the plan quietly moving it.

   One call plans AND applies. A dry run copies the BuildShip (cheap: indices, strings and
   const PartDef*, no self-references) the way --fleet-report does; the undo below replays a
   plan backwards, which is why the change list carries the part id rather than making a
   caller re-derive it. `part` indexes the vector as it was when the plan ran and is invalid
   once parts are added or removed -- use `id`. */
struct StagingChange {
    int part = 0;              // index into ship.parts, valid only for the build as planned
    std::string id;            // the part's id, which survives the vector moving
    int from = 1;              // label it had
    int to = 1;                // label it got
};
struct StagingPlan {
    bool applied = false;                  // false = refused: `ship` is untouched
    std::vector<StagingChange> changes;    // empty when applied with nothing to fix
    std::vector<std::string> notes;        // "refuse: ..." on a refusal, else plan caveats

    /* Which refusal, for a caller that has to say something in ONE short line. `notes` is the
       full sentence -- 160+ characters, about twice the width of a screen once drawToasts puts
       it in the big font (#1027) -- so it belongs in the log and --fleet-report, not in a
       three-second banner. Named kinds are only where the generic verdict would be WRONG:
       a build with no decoupler is a one-stage rocket, not a broken one, and that is 25 of the
       32 shipped hulls (--fleet-report). Every other refusal says the stack has a shape the
       short line cannot carry, which is why the detail stays in the log. */
    enum class Refusal : unsigned { Other, NoDecoupler };
    Refusal refusal = Refusal::Other;
};
/* `fuelStaging` (#1031) orders the cuts by the fuel chain instead of the part tree -- where the
   ladder has no order of its own. A cut ranks by the FURTHEST fuel leg the stage it releases
   still has to supply: link hops from a group inside the stage out to an engine outside it, the
   same walk flight drains through. The chain has an opinion only where fuel crosses the cut
   boundary, so a stage that feeds nothing outside itself ranks 0 and orders by depth exactly as
   it does with the switch off; and a rank that would stand a cut above one nested inside it is
   clamped down, the same repair an authored number gets.

   A typed number still wins (#1020): when every cut already carries one, the switch changes
   nothing and the plan says so. Ranking above an author's number would strand fuel they meant to
   burn, split a ring they left on one label on purpose, and can tip the engine-order guard into
   a refusal nobody can trace back to a checkbox.

   Measured over res/ships, the two hulls carrying links (heavy_asp, heavy_two) already have
   their authors' cut numbers equal to these ranks, so on the shipped fleet the switch changes
   nothing either way. It is for the asparagus stack a player builds in the VAB and never
   numbers: a chain three hops long comes out three presses apart rather than one press for the
   whole ring. Furthest-leg is a proxy for the drain order, not the drain order -- see the note
   on fuelCutRanks about a stage that carries other cut stages. */
/* `fromScratch` (#1035) ignores the numbers the build carries and derives everything: the ladder
   the tree gives (plus the chain, if `fuelStaging` is on), with no authored cut order, no pad-lit
   pin and no engine-order refusal. It is the answer to "I moved a number and now Auto-stage won't
   touch this ship" -- e.g. an engine on a part that never separates can only hold label 1, which
   means lit at every press, so any ignition the player typed for it is something the tree cannot
   produce and the default path refuses rather than overwrites. Replaced numbers stay on the Undo
   stack; the mode is off by default so a hand-numbered hull is still protected from one stray
   click. */
StagingPlan autoStage(BuildShip &ship, bool fuelStaging = false, bool fromScratch = false);

/* Taking a staging edit back (#1013), scoped to LABELS. The VAB has no undo for place,
   delete, detach or clear -- those destroy parts, and undoing them is a feature of its own
   (#1013's own note about scope). A label is an inert int, so a staging edit needs no
   snapshot of the build: it needs the list of (part, from, to) it performed, which is
   exactly what a StagingPlan already carries. An auto-stage plan is therefore its own undo
   record, and the part window's number box records a one-element list.

   Newest last, and the stack only ever describes the build it was recorded against: a load
   or a clear REPLACES the build, and whoever does that clears the stack with it.

   Recorded by ID because the parts vector moves under the player: placing grafts, deleting
   and detaching renumber everything below them, and a change that remembered index 7 would
   put a label back on whatever part now sits at 7. */

/* One edit: the labels it moved, in the order it moved them. Undoing runs the list
   backwards, so an auto-stage plan IS its own undo record.
   `typed` is the part window's number box, which fires once per arrow-repeat frame: a run of
   them on one part is ONE decision, so a typed edit chains onto the typed entry below it and
   the question the player asks afterwards -- "what did this part say before I started" -- has
   one answer. A bulk edit (a plan) never merges with its neighbour and nothing merges into
   it, however few parts it moved: the kind is RECORDED rather than inferred from the length
   of the list, because a plan that happens to move one part is still a plan, and one Ctrl+Z
   must not swallow the player's own edit from above it as well. */
struct StagingEdit {
    std::vector<StagingChange> changes;
    bool typed = false;
};
/* The undo stack, oldest first. Newest last, and it only ever describes the build it was
   recorded against -- a load or a clear replaces the build and clears the stack with it. */
using StagingUndo = std::vector<StagingEdit>;

/* Record one edit. Nothing is recorded when the list is empty (an autoStage with nothing to
   fix) or when a change's two labels are equal (a step that cannot move anything is an Undo
   button that lies). A merge that lands back on the original label deletes the entry rather
   than leaving an Undo that does nothing. */
void recordStagingEdit(StagingUndo &undo, std::vector<StagingChange> changes, bool typed);

/* Reverse the newest edit and pop it. Returns how many parts got their label back, and
   appends a line to `notes` for each that did not: a part the player has since deleted, or
   one whose label something else has moved. Both are left alone rather than forced -- the
   stack is LIFO, so a label that disagrees with the entry means the entry describes a build
   that no longer exists, and guessing a number for it is how an undo starts destroying work.
   The entry is popped either way: an edit that cannot be applied a second time is stale, and
   a button that keeps answering "undone" while changing nothing is worse than one that says
   so. */
size_t undoStagingEdit(BuildShip &ship, StagingUndo &undo,
                       std::vector<std::string> &notes);

/* The Staging ladder's step buttons (#1022), as data rather than widgets. Both move labels
   by `delta` and record the edit they performed, returning how many parts moved so a caller
   can tell a click that did something from one that did not.

   `partStep` moves ONE part; `rungStep` moves EVERY part carrying one label -- which
   includes parts with no stage role: they are attached to whatever moved, and stranding one
   on another label is a mess even while it changes nothing today (#1023 is where such labels
   stop mattering at all).

   Both clamp to the SAVE range [1, kMaxStage], not to the rungs the build happens to carry:
   a fresh build is ONE rung (every part starts at label 1) and autoStage refuses the shapes
   it cannot read, so separating a stack by hand has to be possible from the ladder alone.
   A step is +/-1 by construction, which is what keeps the ladder dense however long a button
   is held.

   The part step records as TYPED (a held button repeats it, and "where was this part before
   I started nudging it" has one answer); a rung shift records as one bulk entry, because the
   rung IS the decision and taking back half of a group's move is not a thing anyone means.
   A click that moves nothing records nothing: an Undo button that undoes nothing is a lie. */
size_t partStepStage(BuildShip &ship, StagingUndo &undo, int part, int delta);
size_t rungStepStage(BuildShip &ship, StagingUndo &undo, int label, int delta);

/* A content hash of everything computeStaging reads about `ship`, plus the two
   scalars. The Staging window redraws every frame on a build that only changes
   when the player edits it, so its rows are cached against this (#293):
   revalidating is one pass over the parts with no allocation, recomputing is a
   burn simulation (84 us for a one-group ship, 473 us for a 240-group one).
   Only the link ENDPOINTS are hashed: computeStaging resolves a link through
   the part ids and ignores its own id, and an id that appears in no link
   cannot change a row. */
uint64_t stagingSignature(const BuildShip &ship, double g, double exhaust_scale);

// Burnable propellant of the part (kg): capacity of the resources flagged in
// `burn`.
double partPropellantMass(const PartDef &def, const bool *burn);

// Inert mass: PartDef::mass (dry, excludes propellant) plus every resource
// not flagged burnable.
inline double partDryMass(const PartDef &def, const bool *burn) {
    assert(def.capacity.size() == (size_t)ResourceType::Num);  // the loop indexes kResources
    double m = def.mass;
    for(size_t r = 0; r < def.capacity.size(); r++) {
        if(!resourceHasMass((ResourceType)r)) { continue; }
        if(burn[r]) { continue; }                        // burnable: sp.fuel
        m += (double)def.capacity[r];
    }
    return m;
}
