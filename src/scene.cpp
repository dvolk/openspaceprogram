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

// Neither floor scene has a lifecycle of its own: it is what the stack is
// seeded with at boot, and arriving back in it is just "the excursion ended".
void floorEnter(Game &) {}
void floorExit(Game &) {}

/* The flight scene has no enter of its own: re-centering onto the active ship
   happens at the caller (vabLaunch / select_ship on a switch, the Fly button,
   load_game), not here. An enter hook could not do it reliably -- setBaseScene
   skips the enter when the base is already Flight (the Fly case). "Resume
   Flight" (popScene) restores the parked camera instead. */

/* The flight scene's widget set, in draw order: the readout windows (HUD ..
   RESOURCES), then the orbital map, then the user-placed part windows, then
   the main menu and Save/Load on top. This is the list stage 3 turns into a
   per-scene window table. */
void flightDrawUi(Game &g, TransferPlanner &p) {
    drawUIReadouts(g, p);
    drawUIMap(g, p);
    drawPartWindows(g);
    drawSaveLoad(g);
}

// The two draw signatures that do not line up with the table's.
void vabDraw3d(Game &g, TransferPlanner &) { drawVab(g); }
/* The VAB's per-frame step: the editor's mouse half (hover/ghost/place), then
   the world tick -- the VAB is a live scene (time passes while you build and
   the warp keys pause/accelerate it); the build tree itself is physics-free
   and static. The launch case is the caller's: vabFireHooks runs before
   sc.update and a launch collapses the stack to Flight, so this is only
   reached on a frame where no launch flipped the scene. */
void vabUpdateLive(Game &g) {
    vabUpdate(g);
    tick(g);
}
/* The editor's widgets: the build tree + top bar, then the shared windows
   (the Settings / Controls / Save-Load bodies draw here; winInScene keeps every
   flight readout out). The VAB has no menu of its own: its top bar's "Back"
   button + Esc walk up the tree to the hub. */
void vabDrawUi(Game &g, TransferPlanner &p) {
    drawVabUI(g);
    drawUIReadouts(g, p);
    drawSaveLoad(g);
}

/* The title screen's widgets. drawUIReadouts is shared with flight and draws
   every window it owns -- but each call goes through drawWin, which checks the
   live scene's set, so only the shared ones (Settings, Controls, Save/Load)
   actually appear here. The flight readouts are not "hidden because there is
   no ship", they are not this scene's windows at all. */
void titleDrawUi(Game &g, TransferPlanner &p) {
    drawUIReadouts(g, p);
    drawTitleMenu(g);
    drawSaveLoad(g);
}

/* The Space Center hub's enter: aim the orbit camera at the home planet for
   the menu backdrop. pushScene already parked the flight camera onto the
   frame, so "Resume Flight" (popScene) hands it back exactly. The hub is a
   live scene: the sim keeps running (the planet turns, the ship coasts) but
   is not steered here (SceneDef::pilot is false). */
void spaceCenterEnter(Game &g) {
    if(g.camera != nullptr && g.home != nullptr) {
        g.camera->mode = CAM_ORBIT;
        for(int i = 0; i < (int)g.focusTargets.size(); i++) {
            if(g.focusTargets[i].body == g.home) { g.focusBody = i; break; }
        }
        g.camera->Follow(g.focusWorldPos(g.focusBody));
        g.camera->distance = 3.0 * g.home->radius;
        g.camera->ComputeView();   // a sane pose immediately, not next frame
    }
    printf("[spacecenter] entered (live sim)\n");
    fflush(stdout);
    g.toast("Space Center");
    // A fresh hub view: drop any armed "Return to title" confirm from a
    // previous visit -- arming used to leak across excursions (leave the hub
    // to the VAB/tracking, come back still one click from discarding the game).
    g.returnTitleArmed = false;
}

/* The hub's widgets: its root menu, then the shared menu windows (Settings /
   Controls / Save-Load, opened from the menu) on top of it. */
void spaceCenterDrawUi(Game &g, TransferPlanner &p) {
    drawSpaceCenterMenu(g);
    drawUIReadouts(g, p);
    drawSaveLoad(g);
}

/* The Tracking Station renders no 3D world (its map covers the viewport), but
   it MUST refresh the ship snapshot (g.view) the map reads: the live sim coasts
   the ships, so the snapshot has to track them (updateShipView, render.cpp) --
   this is the render-phase half of "the world advances", the logic half being
   tick. The loop still clears to the Sky backdrop (black) under the opaque map. */
void trackingDraw3d(Game &g, TransferPlanner &) {
    updateShipView(g);
}

/* The Tracking Station: the full-screen map is the scene's identity, so force it
   open (Root), then overlay the ship list (whose "Back" + "Fly" buttons are the
   navigation). Both are this scene's own copies of the flight windows; the map
   fills the view and the list sits beside it. The shared windows + Save/Load
   come last, on top. The Tracking Station has no menu of its own: Esc / "Back"
   walk up the tree to the hub. */
void trackingDrawUi(Game &g, TransferPlanner &p) {
    setWinOpen(W_TrackingMap, true);
    drawTrackingMap(g, p);
    drawTrackingShipList(g);
    drawUIReadouts(g, p);
    drawSaveLoad(g);
}

}   // namespace

const SceneDef kScenes[(size_t)SceneId::COUNT] = {
    // name      sim    pilot backdrop     wins          enter      exit
    /* The title screen runs the sim and draws the world: with no vessel that
       is O(bodies) and it is what makes the backdrop alive -- the home planet
       turns behind the menu instead of hanging frozen. Its key map is the
       flight one, so the orbit/map camera keys still work for looking around
       the system; the vessel keys no-op with nothing to control. */
    { "title",  true,  false, Backdrop::Sky,    kTitleWins,
      floorEnter,  floorExit,
      tick,        draw3d,    titleDrawUi,  flightKeyActions },
    { "flight", true,  true,  Backdrop::Sky,    kFlightWins,
      floorEnter,  floorExit,
      tick,        draw3d,    flightDrawUi, flightKeyActions },
    /* The editor draws no skybox, so it clears to a flat studio gray. It is a
       live scene: time passes while you build (the world coasts) and the warp
       keys pause/accelerate it; the build tree itself is physics-free. */
    { "vab",    true, false, Backdrop::Studio, kVabWins,
      vabEnter,    vabExit,
      vabUpdateLive, vabDraw3d, vabDrawUi, vabKeyActions },
    /* The hub: a live view of the running sim (the planet turns, the active
       ship coasts) with the root menu on top. tick advances the world and
       draw3d draws it; the ship is not steered here (SceneDef::pilot is
       false, so held keys are inert). */
    { "spacecenter", true, false, Backdrop::Sky, kSpaceCenterWins,
      spaceCenterEnter, floorExit,
      tick, draw3d, spaceCenterDrawUi, hubKeyActions },
    // The Tracking Station: a live view of the running sim, reached from the
    // hub. No world draw (the full-screen map covers the viewport) but tick
    // advances the world and trackingDraw3d refreshes the map's snapshot. Esc
    // pops back to the hub (trackingKeyActions).
    { "tracking", true, false, Backdrop::Sky, kTrackingWins,
      floorEnter, floorExit,
      tick, trackingDraw3d, trackingDrawUi, trackingKeyActions },
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

/* Collapse the stack to a single floor scene. Shared by enterFlight and
   enterTitle: both mean "a different game state is now in charge", so every
   excursion above the floor is unwound and its parked camera discarded. */
static void setBaseScene(Game &g, SceneId id, const char *verb) {
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
    // Pop down to the topmost instance (popScene restores its camera as we go).
    while((int)g.sceneStack.size() - 1 > top) {
        popScene(g);
    }
}

bool gameRunning(const Game &g) {
    return !g.sceneStack.empty() && g.sceneStack.front().id != SceneId::Title;
}
