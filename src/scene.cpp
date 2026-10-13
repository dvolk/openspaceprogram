// The scene stack: transitions + per-scene data (see scene.h).

#include "scene.h"

#include <cstdio>

#include "events.h"    // flightKeyActions / vabKeyActions
#include "game.h"
#include "gameui.h"    // the flight widget set + drawVabUI
#include "render.h"    // refreshWorld / paintWorld / refreshVab / paintVab
#include "tick.h"      // tick
#include "vab.h"       // vabEnter / vabExit / vabUpdate

namespace {

// Neither floor scene has a lifecycle of its own.
void floorEnter(Game &) {}
void floorExit(Game &) {}

// Flight has no enter of its own: re-centering is the caller's (setBaseScene
// skips enter when the base is already Flight).

// The flight widget set, in draw order.
void flightDrawUi(Game &g) {
    drawUIReadouts(g);
    drawUIMap(g);
    drawPartWindows(g);
    drawSaveLoad(g);
}

// Editor mouse half, then the world tick (time passes while you build). Launch
// is the caller's: vabFireHooks runs first.
void vabUpdateLive(Game &g) {
    vabUpdate(g);
    tick(g);
}
void vabDrawUi(Game &g) {
    drawVabUI(g);
    drawUIReadouts(g);
    drawSaveLoad(g);
}

// drawUIReadouts goes through drawWin, so only this scene's windows appear.
void titleDrawUi(Game &g) {
    drawUIReadouts(g);
    drawTitleMenu(g);
    drawReadme(g);
    drawNewGame(g);
    drawSaveLoad(g);
}

// Aim the orbit camera at the home planet. pushScene parked the flight camera,
// so "Resume Flight" hands it back exactly.
void spaceCenterEnter(Game &g) {
    if(g.camera != nullptr && g.home != nullptr) {
        g.camera->mode = CAM_ORBIT;
        for(int i = 0; i < (int)g.focusTargets.size(); i++) {
            if(g.focusTargets[i].body == g.home) { g.focusBody = i; break; }
        }
        g.camera->Follow(g.focusWorldPos(g.focusBody));
        g.camera->distance = 2.0 * g.home->radius;
        g.camera->ComputeView();   // a sane pose immediately, not next frame
    }
    printf("[spacecenter] entered (live sim)\n");
    fflush(stdout);
    g.toastLog("Space Center");
}

void spaceCenterDrawUi(Game &g) {
    drawSpaceCenterTopBar(g);
    drawSpaceCenterMenu(g);
    drawFlightSummary(g);
    drawUIReadouts(g);
    drawSaveLoad(g);
}

/* A scene that shows no 3D world still runs refreshWorld: the map and the shared
   readouts read the g.view snapshot, and the terrain LOD gives the ground its
   collision bodies -- the state pass is not optional, only the pixels stop (#212). */
void noPaint(Game &) {}

// The map is the scene's identity (force it open); the ship list's "Back"/"Fly"
// are the navigation.
void trackingDrawUi(Game &g) {
    setWinOpen(W_TrackingMap, true);
    drawTrackingMap(g);
    drawTrackingShipList(g);
    drawUIReadouts(g);
    drawSaveLoad(g);
}

// The lab is drawn first -- the Atlas docks right of it.
void researchLabDrawUi(Game &g) {
    setWinOpen(W_ResearchLab, true);
    drawResearchLab(g);
    drawResearchAtlas(g);
    drawUIReadouts(g);
    drawSaveLoad(g);
}

}   // namespace

/* Every scene refreshes every frame; only the scenes that show a world paint it.
   Designated initializers because update / refresh / paint share one signature,
   so a swapped pair would compile silently and paint with no state. */
const SceneDef kScenes[(size_t)SceneId::COUNT] = {
    // Title runs the sim + world so the backdrop body turns behind the menu;
    // its key map is the flight one (vessel keys no-op with no vessel).
    { .name = "title", .sim = true, .pilot = false, .backdrop = Backdrop::Sky,
      .wins = kTitleWins, .enter = floorEnter, .exit = floorExit,
      .update = tick, .refresh = refreshWorld, .paint = paintWorld,
      .drawUi = titleDrawUi, .keys = flightKeyActions },
    { .name = "flight", .sim = true, .pilot = true, .backdrop = Backdrop::Sky,
      .wins = kFlightWins, .enter = floorEnter, .exit = floorExit,
      .update = tick, .refresh = refreshWorld, .paint = paintWorld,
      .drawUi = flightDrawUi, .keys = flightKeyActions },
    // The build tree itself is physics-free.
    { .name = "vab", .sim = true, .pilot = false, .backdrop = Backdrop::Studio,
      .wins = kVabWins, .enter = vabEnter, .exit = vabExit,
      .update = vabUpdateLive, .refresh = refreshVab, .paint = paintVab,
      .drawUi = vabDrawUi, .keys = vabKeyActions },
    // The hub: live view of the running sim, not steered (pilot is false).
    { .name = "spacecenter", .sim = true, .pilot = false,
      .backdrop = Backdrop::Sky, .wins = kSpaceCenterWins,
      .enter = spaceCenterEnter, .exit = floorExit,
      .update = tick, .refresh = refreshWorld, .paint = paintWorld,
      .drawUi = spaceCenterDrawUi, .keys = hubKeyActions },
    // Live sim, no world paint (the map covers it).
    { .name = "tracking", .sim = true, .pilot = false,
      .backdrop = Backdrop::Sky, .wins = kTrackingWins,
      .enter = floorEnter, .exit = floorExit,
      .update = tick, .refresh = refreshWorld, .paint = noPaint,
      .drawUi = trackingDrawUi, .keys = trackingKeyActions },
    { .name = "research", .sim = true, .pilot = false,
      .backdrop = Backdrop::Studio, .wins = kResearchWins,
      .enter = researchLabEnter, .exit = floorExit,
      .update = tick, .refresh = refreshWorld, .paint = noPaint,
      .drawUi = researchLabDrawUi, .keys = labKeyActions },
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
    // entry is inserted or dropped.
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
    } else if(s.mode == CAM_FIRST) {
        // Pose is re-derived from the ship each frame: seed the stored one so an
        // immediate ComputeView is not garbage. Turntable state comes back too --
        // leaving first must return the pre-push orbit framing.
        g.camera->pos = s.pos;
        g.camera->forward = s.fwd;
        g.camera->up = s.up;
        g.camera->orbitYaw = s.yaw;
        g.camera->orbitPitch = s.pitch;
        g.camera->distance = s.distance;
        g.camera->mode = CAM_FIRST;
        g.camera->ComputeView();
    } else {
        g.camera->mode = CAM_ORBIT;
        g.camera->orbitYaw = s.yaw;
        g.camera->orbitPitch = s.pitch;
        g.camera->distance = s.distance;
        g.camera->Follow(g.focusWorldPos(g.focusBody));
        g.camera->ComputeView();   // sane pos/forward/up immediately
    }
    printf("[cam] restored: %s focus=%s dist=%.1f m\n",
           s.mode == CAM_FREE ? "free" : (s.mode == CAM_FIRST ? "first" : "orbit"),
           s.focusBody ? s.focusBody->name.c_str() : "ship",
           g.camera->distance);
    fflush(stdout);
}

void pushScene(Game &g, SceneId id) {
    if(g.sceneStack.empty()) { return; }   // main seeds the stack before the loop
    if(curSceneId(g) == id) {
        // A programming-error guard (not reachable from the UI).
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
        // Park/restore pair is an e2e anchor for the camera handback.
        printf("[cam] parked: %s focus=%s dist=%.1f m\n",
               f.cam.mode == CAM_FREE ? "free"
                   : (f.cam.mode == CAM_FIRST ? "first" : "orbit"),
               f.cam.focusBody ? f.cam.focusBody->name.c_str() : "ship",
               f.cam.distance);
        fflush(stdout);
    }
    g.sceneStack.push_back(f);
    kScenes[(size_t)id].enter(g);
    g.ensure_ui_visible();
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
    g.ensure_ui_visible();
    printf("[scene] %s -> %s (pop)\n", sceneName(f.id), sceneName(curSceneId(g)));
    fflush(stdout);
}

/* Collapse the stack to a single floor scene ("a different game state is now
   in charge"): every excursion is unwound and its parked camera discarded. */
static void setBaseScene(Game &g, SceneId id, const char *verb) {
    if(g.sceneStack.empty()) { return; }
    const SceneId from = curSceneId(g);
    const size_t depth = g.sceneStack.size();
    // Unwind WITHOUT restoring cameras: the ship being flown now is not the
    // one the excursion started from.
    while(g.sceneStack.size() > 1) {
        const SceneId top = g.sceneStack.back().id;
        g.sceneStack.pop_back();
        kScenes[(size_t)top].exit(g);
    }
    g.sceneStack.back().camValid = false;
    if(curSceneId(g) != id) {
        const SceneId floorId = curSceneId(g);
        kScenes[(size_t)floorId].exit(g);
        g.sceneStack.back().id = id;
        kScenes[(size_t)id].enter(g);
    }
    // Logged only when something actually moved, so a redundant call is quiet.
    if(depth > 1 || from != id) {
        g.ensure_ui_visible();
        printf("[scene] %s -> %s (%s)\n", sceneName(from), sceneName(id), verb);
        fflush(stdout);
    }
}

void enterFlight(Game &g) { setBaseScene(g, SceneId::Flight, "enterFlight"); }

void enterTitle(Game &g) { setBaseScene(g, SceneId::Title, "enterTitle"); }

void enterSpaceCenter(Game &g) {
    setBaseScene(g, SceneId::SpaceCenter, "enterSpaceCenter");
}

void goScene(Game &g, SceneId id) {
    if(g.sceneStack.empty()) { return; }
    int top = -1;
    for(int i = (int)g.sceneStack.size() - 1; i >= 0; i--) {
        if(g.sceneStack[i].id == id) { top = i; break; }
    }
    if(top < 0) {
        pushScene(g, id);   // not on the stack: jump forward
        return;
    }
    // Pop down to the topmost instance (popScene restores its camera).
    while((int)g.sceneStack.size() - 1 > top) {
        popScene(g);
    }
}

bool gameRunning(const Game &g) {
    return !g.sceneStack.empty() && g.sceneStack.front().id != SceneId::Title;
}
