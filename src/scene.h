#pragma once

// Scene stack: only the top updates/draws/receives input; the stack is never
// empty (popScene refuses at depth 1), so curScene() is always valid.
// EnterFlight/enterTitle collapse it because the ship you are flying changed.

#include <cstddef>

#include <SDL3/SDL.h>   // SDL_Scancode, Uint16 (the keys hook)

#include <glm/glm.hpp>

#include "camera.h"    // CameraMode
#include "terrain.h"   // TerrainBody
#include "uiwins.h"    // WinSet (the windows a scene owns)

struct Game;

enum class SceneId : int {
    // Floor is Title, SpaceCenter (new game, no ship) or Flight; the editor /
    // hub are excursions pushed on top.
    Title,
    Flight,   // active vessel is the invariant (a shipless game is on SpaceCenter)
    Vab,
    SpaceCenter,
    TrackingStation,
    ResearchLab,
    COUNT
};

// "flight" / "vab" -- for the transition log lines.
const char *sceneName(SceneId id);

enum class Backdrop : unsigned char { Sky, Studio };

// A camera pose captured so one scene can take the camera and the previous one
// gets it back. focusBody is a BODY pointer (focusTargets shifts), not an index;
// null = the "ship" entry.
struct CameraSnapshot {
    CameraMode mode = CAM_ORBIT;
    glm::dvec3 pos, fwd, up;          // the free-mode pose
    double distance = 10.0;           // orbit radius
    double yaw = 0.0, pitch = 0.0;    // orbit angles
    TerrainBody *focusBody = nullptr;
};

// One scene-stack entry. cam is the pose to hand back on pop (captured at push);
// camValid false when there was no camera to capture.
struct SceneFrame {
    SceneId id = SceneId::Flight;
    CameraSnapshot cam;
    bool camValid = false;
};

// What a scene IS: data, lifecycle, per-frame half. A new mode is a row here,
// not enum branches.
struct SceneDef {
    const char *name;
    bool sim;             // does the clock advance while this scene is on top
    bool pilot;           // does the live ship take WASD/T/RCS this scene
                          // (non-pilot scenes coast instead of steering)
    Backdrop backdrop;
    WinSet wins;             // the windows this scene owns (uiwins.h)
    void (*enter)(Game &);   // pushed on top: the camera is already captured
    void (*exit)(Game &);    // popped, or unwound by enterFlight
    // Per-frame half; only the TOP scene's run.
    void (*update)(Game &);
    // Per-frame 3D STATE: camera re-pose, terrain LOD, Game::hud.view, the transfer
    // planner. Every scene runs it, painted or not -- the LOD owns the ground's
    // collision bodies and g.hud.view is the snapshot the UI reads. The VAB skips the
    // LOD: its camera is parked at the build tree (#212).
    void (*refresh)(Game &);
    // The pixels. A scene may opt out (noPaint), but the frame loop itself is
    // unconditional -- nothing decides a frame was stale (#209).
    void (*paint)(Game &);
    void (*drawUi)(Game &);   // this scene's imgui windows
    void (*keys)(Game &, SDL_Scancode, Uint16 mod, bool repeat);
};

extern const SceneDef kScenes[(size_t)SceneId::COUNT];

const SceneDef &curScene(const Game &g);
SceneId curSceneId(const Game &g);
bool sceneIs(const Game &g, SceneId id);

// Stack transitions. Each logs one line as an e2e anchor.
// pushScene refuses an already-live scene; popScene refuses at depth 1.
void pushScene(Game &g, SceneId id);
void popScene(Game &g);
// Collapse to [Flight], discarding parked cameras: a launch or a load
// established a live flight, nothing meaningful to pop back to.
void enterFlight(Game &g);
// Collapse to [Title] -- the no-game front door. A shipless GAME sits on
// SpaceCenter, not here.
void enterTitle(Game &g);
// Collapse to [SpaceCenter] -- the floor of a game with no active vessel (new
// game, or a recovered / shipless game).
void enterSpaceCenter(Game &g);
// "Go to" a scene: pop down if on the stack, else push (the 1/2/3/4 shortcuts).
// Unlike enterFlight, no collapse.
void goScene(Game &g, SceneId id);
// True when a game is in charge (stack floor is not Title).
bool gameRunning(const Game &g);

// Capture / apply a camera pose.
CameraSnapshot captureCamera(const Game &g);
void restoreCamera(Game &g, const CameraSnapshot &s);
