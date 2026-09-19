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
#include "uiwins.h"    // WinSet (the windows a scene owns)

struct Game;
class TransferPlanner;

enum class SceneId : int {
    /* The floor is one of Title, SpaceCenter (a new game, no ship yet) or
       Flight, and the editor / hub excursions are pushed on top of whichever
       is live. */
    Title,    // the game-start screen: no vessel; the world as a backdrop and
              // the title menu as the only chrome. Its own scene rather than
              // "Flight with no ship", which is what made the flight readouts
              // render empty behind the menu and the menu closable with no game
              // to go back to.
    Flight,   // the sim: tick + world render + flight widgets. An active
              // vessel is the invariant here -- see enterTitle for the
              // shipless case.
    Vab,      // the editor: no sim; physics-free BuildShip draw + editor widgets
    SpaceCenter, // the hub: no sim; the planet as a static backdrop and a root
              // menu onward to the VAB / the Tracking Station. A NEW game
              // starts here as the floor (no ship yet); "Resume Flight" is
              // offered only when it was pushed on top of a live flight.
    TrackingStation, // no sim; a full-screen chrome-less orbital map + ship list
              // over the paused world, reached from the Space Center hub.
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
    bool pilot;           // does the live ship take WASD/T/RCS this scene
                          // (tick.cpp gates the control block on this, so a
                          // running sim in a non-pilot scene coasts instead
                          // of being steered by stale keys)
    Backdrop backdrop;
    WinSet wins;             // the windows this scene owns (uiwins.h)
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
   This is what an action that establishes a live flight does -- LAUNCH, or a
   Load of a save with vessels -- because there is nothing meaningful to pop
   back to: the ship the excursion started from is not the ship you are flying
   now. */
void enterFlight(Game &g);
/* Collapse to [Title] the same way. This is where a shipless state goes -- a
   bare boot, a save with no vessels, the (defensive) "nothing left to
   control" arm of remove_ship -- and it is what keeps Flight's "there is an
   active vessel" invariant true instead of merely usual. */
void enterTitle(Game &g);
/* Collapse to [SpaceCenter] the same way: the floor of a NEW game, which has
   no ship yet -- the player goes to the VAB to build and launch the first
   vessel (vabLaunch -> enterFlight). "Resume Flight" is hidden there because
   there is no flight to pop back to. */
void enterSpaceCenter(Game &g);
/* Jump to a scene without collapsing the stack: pop down to the topmost
   instance if it is already on the stack (a "back"), else push it (a
   "forward"). Walks top-down, so a duplicated scene (a double-VAB) resolves
   to the nearest copy, and each pop restores that scene's parked camera. This
   is the "go to" semantic the 1/2/3/4 shortcuts use (enterFlight, by contrast,
   collapses the stack because a launch changed the ship you are flying). */
void goScene(Game &g, SceneId id);
/* True when a game is in charge of the world: the stack floor is not the
   Title. The floor is Title only in the no-game states (a bare boot, --vab
   with no vessel, quitToTitle) and SpaceCenter/Flight otherwise. */
bool gameRunning(const Game &g);

// Capture / apply a camera pose. Free functions rather than Game methods: they
// are transition mechanics belonging to the stack, and the snapshot lives on
// the frame that took the camera over.
CameraSnapshot captureCamera(const Game &g);
void restoreCamera(Game &g, const CameraSnapshot &s);
