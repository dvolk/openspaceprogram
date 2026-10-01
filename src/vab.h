// vab.h -- the VAB editor's interaction layer: physics-free picking of the
// build tree, the ghost preview pose, and placing a part. All in the build
// ship's frame S (Game::vab poses).
#pragma once

#include "game.h"   // Game (vab BuildShip, camera)
#include "pick.h"   // PickRay / PickBodyHit / castRay

class btCollisionObject;
class btCollisionShape;

// Cached convex hull for a catalog part (physics-free pick geometry).
btCollisionShape *vabPartHull(const PartDef *def);
btCollisionObject *vabPartObject(const PartDef *def);

// Nearest build part under a pixel. hit is in the build frame S.
bool pickVabPart(Game &g, int px, int py, int &partIdx, PickBodyHit &hit);

// Build-frame (S) position of node `nodeIdx` on build part `partIdx`.
glm::dvec3 vabNodePos(const Game &g, int partIdx, int nodeIdx);

// Build-frame (S) point -> window pixel (pickRay unprojection's inverse).
bool vabProject(const Game &g, const glm::dvec3 &pS, double &px, double &py);

// Nearest STACK node to the mouse within `thresholdPx`; -1 if none.
// Occupied ports are skipped (falls through to surface attach).
int pickVabNode(Game &g, int px, int py, int partIdx, double thresholdPx);

// Reset the hover + ghost state.
void vabClearHover(Game &g);

// Per-frame: hover part + node, ghost pose for the armed part at the target.
void vabUpdateHover(Game &g, int px, int py);

// Place the armed palette part at the current hover target. -1 on failure.
int vabPlace(Game &g);

// Q/E: spin the ghost or selected part about its attach axis. No-op if neither.
void vabRotate(Game &g, double deltaDeg);

// Link-mode click: first hovered part = SOURCE, second = DESTINATION.
// Refuses self-link or duplicate; empty space is ignored.
void vabLinkClick(Game &g);

// Delete/X: remove the selected part and its subtree. Root refuses.
void vabDeleteSelected(Game &g);

// Del/X: detach the selected subtree into Subassemblies. A lone part is
// deleted instead (the palette already has it). Root refuses.
void vabDetachSelected(Game &g);

// Write the build tree as the named ship def in the data dir's ships/.
// `name` is the ship's identity ("racer"), not a path.
void vabSave(Game &g, const char *name);

// Load a ship def into the VAB, REPLACING the current build. Lookup by name
// (data dir's ships/ wins over stock). True on success; current build is
// left untouched on failure.
bool vabLoad(Game &g, const char *name);

// LAUNCH: place the tree on the home body's pad, take control, switch to Flight.
void vabLaunch(Game &g);

// Aim the editor's orbit camera at the build tree.
void vabAimCamera(Game &g);

// Per-frame step while the Vab scene is live (the sim does not tick).
// vabFireHooks runs FIRST (a launch flips the scene to Flight).
void vabFireHooks(Game &g);
void vabUpdate(Game &g);

// Scene transitions. The Vab is a LIVE scene (sim keeps coasting).
// vabOpen / vabClose are the player-facing pair (push/pop the scene stack).
// vabEnter / vabExit are the scene table's lifecycle hooks.
void vabOpen(Game &g);
void vabClose(Game &g);
void vabEnter(Game &g);
void vabExit(Game &g);
