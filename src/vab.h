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

// Nearest STACK node of build part `partIdx` to the mouse within
// `thresholdPx` (screen space); -1 if none. Stack ports are points, so
// screen-space nearest is the natural grab test.
int pickVabNode(Game &g, int px, int py, int partIdx, double thresholdPx);

// Per-frame editor update: hover part + node, and the ghost pose for the
// armed palette part at the hovered target (stack node, or a surface hit on
// the hovered parent when no node is near). Sets Game::vab_hover,
// vab_hoverNode, vab_ghost*.
void vabUpdateHover(Game &g, int px, int py);

// Place the armed palette part at the current hover target (stack node or
// surface hit), appending to Game::vab and re-solving poses. Returns the new
// part index, or -1 if nothing is armed/targeted.
int vabPlace(Game &g);
