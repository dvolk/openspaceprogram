// The scene stack: transitions + per-scene data (see scene.h).

#include "scene.h"

#include <cstdio>

#include "events.h"    // flightKeyActions / vabKeyActions
#include "game.h"
#include "gameui.h"    // the flight widget set + drawVabUI
#include "render.h"    // draw3d / drawVab
#include "tick.h"      // tick
#include "vab.h"       // vabEnter / vabExit / vabUpdate

namespace {

// Neither floor scene has a lifecycle of its own.
void floorEnter(Game &) {}
void floorExit(Game &) {}

// Flight has no enter of its own: re-centering happens at the caller
// (setBaseScene skips enter when the base is already Flight).

// The flight widget set, in draw order.
void flightDrawUi(Game &g) {
    drawUIReadouts(g);
    drawUIMap(g);
    drawPartWindows(g);
    drawSaveLoad(g);
}

// The two draw signatures that do not line up with the table's.
void vabDraw3d(Game &g) { drawVab(g); }
// The VAB's per-frame step: editor mouse half, then the world tick (time
// passes while you build). Launch is the caller's: vabFireHooks runs first.
void vabUpdateLive(Game &g) {
    vabUpdate(g);
    tick(g);
}
// The editor's widgets: build tree + top bar, then the shared windows.
void vabDrawUi(Game &g) {
    drawVabUI(g);
    drawUIReadouts(g);
    drawSaveLoad(g);
}

// Title widgets. drawUIReadouts goes through drawWin, so only the shared
// windows appear -- the flight readouts are not this scene's at all.
void titleDrawUi(Game &g) {
    drawUIReadouts(g);
    drawTitleMenu(g);
    drawReadme(g);
    drawNewGame(g);
    drawSaveLoad(g);
}

// Hub enter: aim the orbit camera at the home planet. pushScene parked the
// flight camera, so "Resume Flight" hands it back exactly.
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
    g.toast("Space Center");
}

// Hub widgets: top bar, root menu, then the shared windows on top.
void spaceCenterDrawUi(Game &g) {
    drawSpaceCenterTopBar(g);
    drawSpaceCenterMenu(g);
    drawFlightSummary(g);
    drawUIReadouts(g);
    drawSaveLoad(g);
}

// Tracking Station renders no 3D world, but MUST refresh g.view (the live
// sim coasts the ships and the map reads the snapshot).
void trackingDraw3d(Game &g) {
    updateShipView(g);
}

// The full-screen map is the scene's identity (force it open), then the
// ship list (whose "Back"/"Fly" are the navigation), then shared windows.
void trackingDrawUi(Game &g) {
    setWinOpen(W_TrackingMap, true);
    drawTrackingMap(g);
    drawTrackingShipList(g);
    drawUIReadouts(g);
    drawSaveLoad(g);
}

// Research Lab draws no 3D: the studio clear is its whole backdrop.
void labDraw3d(Game &) {}

// Research Lab widgets: its single window (Root), then shared windows.
void researchLabDrawUi(Game &g) {
    setWinOpen(W_ResearchLab, true);
    drawResearchLab(g);
    drawUIReadouts(g);
    drawSaveLoad(g);
}

}   // namespace

const SceneDef kScenes[(size_t)SceneId::COUNT] = {
    // name      sim    pilot backdrop     wins          enter      exit
    // Title runs the sim + world so the backdrop body turns behind the menu;
    // its key map is the flight one (vessel keys no-op with no vessel).
    { "title",  true,  false, Backdrop::Sky,    kTitleWins,
      floorEnter,  floorExit,
      tick,        draw3d,    titleDrawUi,  flightKeyActions },
    { "flight", true,  true,  Backdrop::Sky,    kFlightWins,
      floorEnter,  floorExit,
      tick,        draw3d,    flightDrawUi, flightKeyActions },
    // The editor: flat studio clear; live sim (time passes while you build);
    // the build tree itself is physics-free.
    { "vab",    true, false, Backdrop::Studio, kVabWins,
      vabEnter,    vabExit,
      vabUpdateLive, vabDraw3d, vabDrawUi, vabKeyActions },
    // The hub: live view of the running sim, not steered (pilot is false).
    { "spacecenter", true, false, Backdrop::Sky, kSpaceCenterWins,
      spaceCenterEnter, floorExit,
      tick, draw3d, spaceCenterDrawUi, hubKeyActions },
    // Tracking Station: live sim, no world draw (the map covers it).
    { "tracking", true, false, Backdrop::Sky, kTrackingWins,
      floorEnter, floorExit,
      tick, trackingDraw3d, trackingDrawUi, trackingKeyActions },
    // Research Lab: studio backdrop, live sim behind it.
    { "research", true, false, Backdrop::Studio, kResearchWins,
      researchLabEnter, floorExit,
      tick, labDraw3d, researchLabDrawUi, labKeyActions },
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
               f.cam.mode == CAM_FREE ? "free" : "orbit",
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
