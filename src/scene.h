#pragma once

/* Where the player is.

   This used to be one `Scene scene` field on Game that four unrelated
   questions were read off -- what simulates, what the 3D pass draws, what the
   UI draws, and what input means -- and a fifth question ("is a game in
   progress?") was inferred from `ship != nullptr` rather than represented at
   all. That is why the game-start menu was not a place but a coincidence of
   three facts, and why every flight window needed its own "No active ship."
   guard.

   Modes are a STACK now. The VAB is an excursion you return from, so it
   remembers what it was pushed on top of instead of hardcoding "back to
   flight", and the camera pose travels with the frame that parked it.

   Only the TOP of the stack updates, draws and receives input; the scenes
   below are suspended. The stack is never empty (popScene refuses at depth 1),
   so curScene() is always valid.

   Design, staging and the evidence for it: reports/ui-scenes2026_09_17. */

#include <cstddef>

#include <SDL3/SDL.h>   // SDL_Scancode, Uint16 (the keys hook)

#include <glm/glm.hpp>

#include "camera.h"    // CameraMode
#include "terrain.h"   // TerrainBody

struct Game;
class TransferPlanner;

enum class SceneId : int {
    Flight,   // the sim: tick + world render + flight widgets
    Vab,      // the editor: no sim; physics-free BuildShip draw + editor widgets
    COUNT
};

// "flight" / "vab" -- for the transition log lines.
const char *sceneName(SceneId id);

// The 3D pass's backdrop: the skybox over a black clear, or the editor's flat
// studio gray.
enum class Backdrop : unsigned char { Sky, Studio };

/* A camera pose, captured so one scene can take the camera over and the
   previous one can get it back exactly. Both camera modes ride in here: orbit
   needs the yaw/pitch/distance and the focus, free needs the explicit pose.

   `focusBody` is the orbit target as a BODY, not an index into
   Game::focusTargets: that list shifts when the "ship" entry is inserted or
   dropped (syncShipFocus), so a saved index would be stale by restore time.
   null = the "ship" entry. */
struct CameraSnapshot {
    CameraMode mode = CAM_ORBIT;
    glm::dvec3 pos, fwd, up;          // the free-mode pose
    double distance = 10.0;           // orbit radius
    double yaw = 0.0, pitch = 0.0;    // orbit angles
    TerrainBody *focusBody = nullptr;
};

/* One entry of the scene stack. `cam` is the pose to hand back when THIS frame
   is popped -- the camera as the scene below left it, captured at push time
   before the incoming scene aims it. `camValid` is false when there was no
   camera to capture, in which case the pop leaves the camera alone. */
struct SceneFrame {
    SceneId id = SceneId::Flight;
    CameraSnapshot cam;
    bool camValid = false;
};

/* What a scene IS: its data, its lifecycle, and its per-frame half. This is
   the whole point of the table -- adding a mode means adding a row and its
   four functions, not finding the places that branch on an enum. */
struct SceneDef {
    const char *name;
    bool sim;             // does the clock advance while this scene is on top
    Backdrop backdrop;
    void (*enter)(Game &);   // pushed on top: the camera is already captured
    void (*exit)(Game &);    // popped, or unwound by enterFlight
    /* The per-frame half. Only the TOP scene's are called; the frames below
       are suspended and neither update nor draw. `update` is the loop's LOGIC
       phase (tick for flight, the editor's mouse step for the VAB); a scene
       with sim == false must still get a redraw, which the loop does with
       `if(!sc.sim) redraw = true` since there is no tick to mark the frame. */
    void (*update)(Game &);
    void (*draw3d)(Game &, TransferPlanner &);   // the world / build-tree pass
    void (*drawUi)(Game &, TransferPlanner &);   // this scene's imgui windows
    void (*keys)(Game &, SDL_Scancode, Uint16 mod, bool repeat);
};

extern const SceneDef kScenes[(size_t)SceneId::COUNT];

// The live scene (the top of the stack). Always valid: the stack is seeded
// with Flight in Game's constructor and popScene refuses at depth 1, so it is
// never empty and back() is never out of range.
const SceneDef &curScene(const Game &g);
SceneId curSceneId(const Game &g);
bool sceneIs(const Game &g, SceneId id);

/* Stack transitions. Each logs one line as an e2e anchor:
       [scene] flight -> vab (push)
   pushScene refuses to push a scene that is already live (a VAB inside a VAB
   is a bug, not a feature); popScene refuses at depth 1, because there is
   nowhere to go back to. */
void pushScene(Game &g, SceneId id);
void popScene(Game &g);
/* Collapse the whole stack to [Flight], discarding every parked camera pose.
   This is what an action that establishes a new game does -- LAUNCH, New Game,
   Load -- because there is nothing meaningful to pop back to: the ship the
   excursion started from is not the ship you are flying now. */
void enterFlight(Game &g);

// Capture / apply a camera pose. Free functions rather than Game methods: they
// are transition mechanics belonging to the stack, and the snapshot lives on
// the frame that took the camera over.
CameraSnapshot captureCamera(const Game &g);
void restoreCamera(Game &g, const CameraSnapshot &s);
