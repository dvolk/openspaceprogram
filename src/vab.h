// vab.h -- the VAB editor's interaction layer: physics-free picking of the
// build tree (parts and their attach nodes), the ghost preview pose, and
// placing a part into the tree. All in the build ship's frame S (Game::vab
// poses); the render frame is S shifted by -Game::vab_center.
#pragma once

#include "game.h"   // Game (vab BuildShip, camera)
#include "pick.h"   // PickRay / PickBodyHit / castRay

class btCollisionObject;
class btCollisionShape;

// Cached convex hull (and a wrapper collision object) for a catalog part,
// built once per PartDef from its mesh -- the physics-free pick geometry
// (the build tree has hull shapes but no rigid bodies).
btCollisionShape *vabPartHull(const PartDef *def);
btCollisionObject *vabPartObject(const PartDef *def);

// Nearest build part under window pixel (px,py). hit is in the build frame
// S (the Game::vab poses), per the pickRay contract. false = miss.
bool pickVabPart(Game &g, int px, int py, int &partIdx, PickBodyHit &hit);

// Build-frame (S) position of node `nodeIdx` on build part `partIdx`.
glm::dvec3 vabNodePos(const Game &g, int partIdx, int nodeIdx);

// Build-frame (S) point -> window pixel (the pickRay unprojection's
// inverse; false behind the camera). The port gizmos in drawVabUI draw
// with it.
bool vabProject(const Game &g, const glm::dvec3 &pS, double &px, double &py);

// Nearest STACK node of build part `partIdx` to the mouse within
// `thresholdPx` (screen space); -1 if none. Stack ports are points, so
// screen-space nearest is the natural grab test. Occupied ports
// (BuildShip::nodeOccupied) are skipped, so hovering one falls through to
// surface attach.
int pickVabNode(Game &g, int px, int py, int partIdx, double thresholdPx);

// Reset the hover + ghost state (no target under the mouse).
void vabClearHover(Game &g);

// Per-frame editor update: hover part + node, and the ghost pose for the
// armed palette part at the hovered target (stack node, or a surface hit on
// the hovered parent when no node is near). Sets Game::vab_hover,
// vab_hoverNode, vab_ghost*.
void vabUpdateHover(Game &g, int px, int py);

// Place the armed palette part at the current hover target (stack node or
// surface hit), appending to Game::vab and re-solving poses. Returns the new
// part index, or -1 if nothing is armed/targeted.
int vabPlace(Game &g);

// Q/E: spin the ghost (while one previews) or the selected part about its
// attach axis by deltaDeg. No ghost, no selection: no-op.
void vabRotate(Game &g, double deltaDeg);

// Link-mode click: the first hovered part becomes the fuel link's SOURCE
// (Game::vab_linkFromId), the second the DESTINATION -- appending the link
// to Game::vab.fuelLinks and leaving link mode. Refuses a self-link or a
// duplicate from->to; a click on empty space is ignored (the mode stays).
void vabLinkClick(Game &g);

// Delete/X: remove the selected part and its subtree. The root refuses
// (toast); success clears the selection + hover.
void vabDeleteSelected(Game &g);

// Del/X (the default "delete"): detach the selected part's subtree into
// the session Subassemblies list instead of destroying it (the root
// refuses). Shift+Del/X remains the destructive vabDeleteSelected.
void vabDetachSelected(Game &g);

// Write the build tree to `path` (save_ship_def); toasts the outcome.
void vabSave(Game &g, const char *path);

// LAUNCH: convert the tree to a ShipDef, place it on the home body's pad
// (with startup-style crew aboard), take control of it, and switch to the
// Flight scene. Empty tree: toast, stay put.
void vabLaunch(Game &g);

// Scene transitions (the main menu's "Go to VAB" / the VAB's "Back to
// game"). The sim FREEZES in the Vab scene (tick is skipped), so entering
// mid-flight is a pause. vabOpen parks the flight camera (in either mode)
// and re-aims the orbit at the build tree; vabClose restores it exactly.
void vabOpen(Game &g);
void vabClose(Game &g);
