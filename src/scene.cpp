// The scene stack: what the transitions do, and the per-scene data the loop
// reads instead of branching on an enum. See scene.h for why this exists.

#include "scene.h"

#include <cstdio>

#include "events.h"    // flightKeyActions / vabKeyActions
#include "game.h"
#include "gameui.h"    // the flight widget set + drawVabUI
#include "render.h"    // draw3d / drawVab
#include "tick.h"      // tick
#include "transferplanner.h"
#include "vab.h"       // vabEnter / vabExit / vabUpdate

namespace {

// The floor scene needs no lifecycle of its own: it is what the stack is
// seeded with at boot, and arriving back in it is just "the excursion ended".
void flightEnter(Game &) {}
void flightExit(Game &) {}

/* The flight scene's widget set, in draw order: the readout windows (HUD ..
   RESOURCES), then the orbital map, then the user-placed part windows, then
   the main menu and Save/Load on top. This is the list stage 3 turns into a
   per-scene window table. */
void flightDrawUi(Game &g, TransferPlanner &p) {
    drawUIReadouts(g, p);
    drawUIMap(g, p);
    drawPartWindows(g);
    drawMainMenu(g);
    drawSaveLoad(g);
}

// The two draw signatures that do not line up with the table's.
void vabDraw3d(Game &g, TransferPlanner &) { drawVab(g); }
void vabDrawUi(Game &g, TransferPlanner &) { drawVabUI(g); }

}   // namespace

const SceneDef kScenes[(size_t)SceneId::COUNT] = {
    // name      sim    backdrop            enter        exit
    { "flight", true,  Backdrop::Sky,    flightEnter, flightExit,
      tick,       draw3d,    flightDrawUi, flightKeyActions },
    // The editor draws no skybox, so it clears to a flat studio gray.
    { "vab",    false, Backdrop::Studio, vabEnter,    vabExit,
      vabUpdate,  vabDraw3d, vabDrawUi,    vabKeyActions },
};

const char *sceneName(SceneId id) { return kScenes[(size_t)id].name; }

SceneId curSceneId(const Game &g) { return g.sceneStack.back().id; }

const SceneDef &curScene(const Game &g) { return kScenes[(size_t)curSceneId(g)]; }

bool sceneIs(const Game &g, SceneId id) { return curSceneId(g) == id; }

CameraSnapshot captureCamera(const Game &g) {
    CameraSnapshot s;
    if(g.camera == nullptr) { return s; }
    s.mode = g.camera->mode;
    s.pos = g.camera->pos;
    s.fwd = g.camera->forward;
    s.up = g.camera->up;
    s.distance = g.camera->distance;
    s.yaw = g.camera->orbitYaw;
    s.pitch = g.camera->orbitPitch;
    // The focus as a BODY, not an index: focusTargets shifts when the "ship"
    // entry is inserted or dropped, so an index captured now would be stale by
    // restore time. On a --vab boot the list is not built yet (main builds it
    // after the scene entry), so fall back to the default focus.
    s.focusBody =
        (g.focusBody >= 0 && g.focusBody < (int)g.focusTargets.size())
        ? g.focusTargets[g.focusBody].body
        : (g.ship != nullptr ? nullptr : g.home);
    return s;
}

void restoreCamera(Game &g, const CameraSnapshot &s) {
    if(g.camera == nullptr) { return; }
    // The saved body resolves to its CURRENT index (null = the "ship" entry).
    for(int i = 0; i < (int)g.focusTargets.size(); i++) {
        if(g.focusTargets[i].body == s.focusBody) { g.focusBody = i; break; }
    }
    if(s.mode == CAM_FREE) {
        g.camera->setFreePose(s.pos, s.fwd, s.up);
    } else {
        g.camera->mode = CAM_ORBIT;
        g.camera->orbitYaw = s.yaw;
        g.camera->orbitPitch = s.pitch;
        g.camera->distance = s.distance;
        g.camera->Follow(g.focusWorldPos(g.focusBody));
        g.camera->ComputeView();   // sane pos/forward/up immediately
    }
    printf("[cam] restored: %s focus=%s dist=%.1f m\n",
           s.mode == CAM_FREE ? "free" : "orbit",
           s.focusBody ? s.focusBody->name.c_str() : "ship",
           g.camera->distance);
    fflush(stdout);
}

void pushScene(Game &g, SceneId id) {
    if(g.sceneStack.empty()) { return; }   // main seeds the stack before the loop
    if(curSceneId(g) == id) {
        // Not reachable from the UI (a scene's own entry points are not drawn
        // while it is live), so this is a programming-error guard rather than
        // something to tell the player about.
        printf("[scene] already in %s -- push refused\n", sceneName(id));
        fflush(stdout);
        return;
    }
    SceneFrame f;
    f.id = id;
    f.camValid = (g.camera != nullptr);
    const SceneId from = curSceneId(g);
    if(f.camValid) {
        f.cam = captureCamera(g);
        // The park/restore pair is the e2e anchor for a scene handing the
        // camera back: the two lines must agree, or the viewpoint came home
        // wrong -- a silent failure, since nothing crashes.
        printf("[cam] parked: %s focus=%s dist=%.1f m\n",
               f.cam.mode == CAM_FREE ? "free" : "orbit",
               f.cam.focusBody ? f.cam.focusBody->name.c_str() : "ship",
               f.cam.distance);
        fflush(stdout);
    }
    g.sceneStack.push_back(f);
    kScenes[(size_t)id].enter(g);
    printf("[scene] %s -> %s (push)\n", sceneName(from), sceneName(id));
    fflush(stdout);
}

void popScene(Game &g) {
    if(g.sceneStack.size() <= 1) {
        printf("[scene] %s is the floor -- pop refused\n", sceneName(curSceneId(g)));
        fflush(stdout);
        return;
    }
    const SceneFrame f = g.sceneStack.back();
    g.sceneStack.pop_back();
    kScenes[(size_t)f.id].exit(g);
    if(f.camValid) { restoreCamera(g, f.cam); }
    printf("[scene] %s -> %s (pop)\n", sceneName(f.id), sceneName(curSceneId(g)));
    fflush(stdout);
}

void enterFlight(Game &g) {
    if(g.sceneStack.empty()) { return; }
    const SceneId from = curSceneId(g);
    const size_t depth = g.sceneStack.size();
    // Unwind every excursion WITHOUT restoring its camera: the ship being
    // flown now is not the one the excursion started from (a launch just
    // placed a new one and select_ship has aimed the camera at it), so the
    // parked poses are dead weight. Discarding them here is what stops a
    // later "back to game" restoring a viewpoint from before the launch.
    while(g.sceneStack.size() > 1) {
        const SceneId top = g.sceneStack.back().id;
        g.sceneStack.pop_back();
        kScenes[(size_t)top].exit(g);
    }
    g.sceneStack.back().camValid = false;
    if(curSceneId(g) != SceneId::Flight) {
        const SceneId floorId = curSceneId(g);
        kScenes[(size_t)floorId].exit(g);
        g.sceneStack.back().id = SceneId::Flight;
        kScenes[(size_t)SceneId::Flight].enter(g);
    }
    // Logged only when something actually moved, so a redundant call is quiet.
    if(depth > 1 || from != SceneId::Flight) {
        printf("[scene] %s -> flight (enterFlight)\n", sceneName(from));
        fflush(stdout);
    }
}
