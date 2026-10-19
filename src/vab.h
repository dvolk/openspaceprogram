// vab.h -- the VAB editor's interaction layer: physics-free picking of the
// build tree, the ghost preview pose, and placing a part. All in the build
// ship's frame S (Game::vab poses).
#pragma once

#include "game.h"   // Game (vab BuildShip, camera)
#include "pick.h"   // PickRay / PickBodyHit / castRay

class btCollisionObject;
class btCollisionShape;

// Cached convex hull for a catalog part.
btCollisionShape *vabPartHull(const PartDef *def);
btCollisionObject *vabPartObject(const PartDef *def);

// Nearest build part under a pixel. hit is in the build frame S.
bool pickVabPart(Game &g, int px, int py, int &partIdx, PickBodyHit &hit);

glm::dvec3 vabNodePos(const Game &g, int partIdx, int nodeIdx);

// Build-frame (S) point -> window pixel.
bool vabProject(const Game &g, const glm::dvec3 &pS, double &px, double &py);

// Where a named build part can be CLICKED, for --ui-focus. Returns empty on
// success (and a pixel whose real pick lands on that part), or the reason it
// could not be aimed: wrong scene, no such part, behind the camera, or hidden
// from this angle -- which is the case a test must answer by orbiting.
std::string vabAimAtPart(Game &g, const std::string &id, double &sx, double &sy);

// Nearest STACK node to the mouse within `thresholdPx`; -1 if none.
// Occupied ports are skipped (falls through to surface attach).
int pickVabNode(Game &g, int px, int py, int partIdx, double thresholdPx);

// Reset the hover + ghost state.
void vabClearHover(Game &g);

// Per-frame: hover part + node, and the ghost pose at the target.
void vabUpdateHover(Game &g, int px, int py);

// Place the armed palette part at the current hover target. -1 on failure.
int vabPlace(Game &g);

// Q/E: spin the ghost or selected part about its attach axis. No-op if neither.
void vabRotate(Game &g, double deltaDeg);

// Link-mode click: first hovered part = SOURCE, second = DESTINATION. Empty
// space is ignored; a self-link, a duplicate direction, a source a delete
// already removed, and a link that would close a fuel loop all refuse.
void vabLinkClick(Game &g);

// Delete/X: remove the selected part and its subtree. Root refuses.
void vabDeleteSelected(Game &g);

// Del/X: detach the selected subtree into Subassemblies. A lone part is
// deleted instead (the palette already has it). Root refuses.
void vabDetachSelected(Game &g);

// `name` is the ship's identity ("racer"), not a path.
void vabSave(Game &g, const char *name);

// REPLACES the current build; leaves it untouched on failure. Lookup by name
// (data dir's ships/ wins over stock).
bool vabLoad(Game &g, const char *name);

// LAUNCH: place the tree on the home body's pad, take control, switch to Flight.
void vabLaunch(Game &g);

/* --- seat assignments (the seat window) ------------------------------------
   Right-click a capsule in the VAB to open a window with one dropdown per seat,
   listing the kerbals standing on the pad (VabState::seatAssigns, game.h). */

// RMB click in the VAB: open the seat window for the capsule under the cursor.
void vabPickSeat(Game &g, int px, int py);

// Open the seat window for build part `partIdx`. Refuses a part with no seats.
void vabOpenSeatWindow(Game &g, int partIdx, int px, int py);

// Put `profileId` in that seat, or empty it (profileId 0). False if the part or
// slot is not a seat of the current build.
bool vabAssignSeat(Game &g, const std::string &partId, int slot,
                   uint64_t profileId);

// Drop assignments the build no longer has a seat for (part deleted, subtree
// detached, slot out of range). Every frame from vabUpdate.
void vabPruneSeats(Game &g);

// "" if every assignment can be honoured at launch; else why to refuse. Checked
// BEFORE the ship exists: a half-launched ship with an unfillable seat is worse
// than a refusal the player can fix in the window.
std::string vabSeatProblem(const Game &g);

// The window itself (gameui.cpp), drawn while the Vab scene is live.
void drawVabSeatWindow(Game &g);

void vabAimCamera(Game &g);

// Register the headless VAB hooks at boot, before the main loop. Most join
// the pre-scene phase (a launch flips the scene to Flight); --vab-place joins
// the phase fired inside vabUpdate, after the hover pass that arms the ghost.
void vabInstallHooks(Game &g);
// Per-frame step while the Vab scene is live. The sim keeps ticking around it
// (scene.cpp vabUpdateLive: vabUpdate then tick).
void vabUpdate(Game &g);

// What a left click DOES in the editor. Separate from vabUpdate because it
// must run after ImGui::Render, where WantCaptureMouse describes THIS frame
// (see the comment at the definition). Only the VAB consumes the LMB edge.
void vabClickAfterUI(Game &g);

// Scene transitions. The Vab is a LIVE scene (sim keeps coasting). Open/Close
// are the player-facing pair (push/pop the scene stack); Enter/Exit are the
// scene table's lifecycle hooks.
void vabOpen(Game &g);
void vabClose(Game &g);
void vabEnter(Game &g);
void vabExit(Game &g);
