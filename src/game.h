// game.h -- the running game: the long-lived subsystems (borrowed from
// main) plus the runtime state -- the cameras, the clock, the active ship,
// the input/UI flags, the orbit-camera focus targets, the UI window
// registry -- and the control transitions (select/remove a ship, enter
// rails warp, toggle the windows).
//
// main() creates the subsystems, builds the Game once they exist, and then
// drives everything through it: the event dispatch (events.cpp) and the
// logic/render sections of the main loop read and write the state here, so
// there is a single source of truth for it. Game owns none of the
// subsystems (main still creates and deletes them); it only borrows them.
// The small runtime state (clock, selection, flags, focus, UI registry)
// is owned by Game.
#pragma once

#include <SDL2/SDL.h>   // Uint32

#include <vector>

#include "camera.h"   // Camera, OrbitCamera, FreeCamera
#include "cli.h"      // GameArgs
#include "display.h"  // Renderer
#include "postfx.h"   // PostFX
#include "ships.h"    // Ships
#include "system.h"   // System
#include "terrain.h"  // TerrainBody
#include "ui.h"       // ui::Options
#include "vehicle.h"  // Vehicle

// Rails warp threshold: at this accel (and above) nobody is integrated --
// every ship coasts on rails (or sits frozen on the ground) and the Bullet
// world is not stepped, so a tick costs O(ships). Below it the active ship
// is in the physics world. Shared by the event dispatch, the logic tick
// and the startup clamp (was a local const in main).
static const int kRailsWarp = 10000;

// The active camera (was a local `enum CameraMode` in main).
enum CameraMode { CAM_ORBIT, CAM_FREE };

struct Game {
    // --- borrowed subsystems (main creates + deletes) ---------------------
    Renderer &display;
    PostFX *postfx;
    Ships &ships;
    System &sys;
    TerrainBody *sun;
    TerrainBody *home;
    GameArgs &args;
    Uint32 sim_win_id;
    Uint32 loop_start_ms = 0;   // set once the main loop is about to start

    // --- cameras -----------------------------------------------------------
    OrbitCamera *orbitCam = nullptr;
    FreeCamera *freeCam = nullptr;
    Camera *camera = nullptr;   // the active one
    int camMode = CAM_ORBIT;
    int cam_speed = 1;

    // --- the clock ----------------------------------------------------------
    int time_accel = 0;

    // --- input / selection state -------------------------------------------
    bool running = true;
    bool rmbCam = false;            // RMB held over 3D: camera look
    bool poly_mode = false;         // F11 wireframe
    bool screenshot_requested = false;
    int activeIdx = 0;             // the player-controlled ship
    Vehicle *ship = nullptr;       // always ships[activeIdx]

    // --- orbit camera focus targets (the ship + every body; G cycles) ------
    struct FocusTarget { const char *name; TerrainBody *body; };
    std::vector<FocusTarget> focusTargets;
    int focusBody = 0;             // index into focusTargets
    int numFocusTargets = 0;

    // --- UI window registry (the TAB toggle + the main-menu button) --------
    struct UiWin { const char *name; const char *label; ui::Options opts; };
    std::vector<UiWin> ui_windows;
    ui::Options o_hud;
    bool ui_visible = true;

    // --- control transitions (events + the SHIPS window + selftest) --------
    // World (ship-frame) position of a focus target, to point the orbit
    // camera at it.
    glm::dvec3 focusWorldPos(int i) const;
    // TAB: toggle the info windows (the main menu's "Toggle windows" too).
    void toggle_windows();
    // Take control of a ship (release + park the current one, recenter the
    // orbit camera, drop rails warp).
    void select_ship(int idx);
    // Enter rails warp (park every ship); false + keeps the accel if any
    // ship is not rail-eligible.
    bool enter_rails_warp();
    // Remove a ship + its bookkeeping (refuses the last one; hands control
    // off if the active one is removed).
    void remove_ship(int idx);

    Game(Renderer &display, PostFX *postfx, Ships &ships, System &sys,
         TerrainBody *sun, TerrainBody *home, GameArgs &args,
         Uint32 sim_win_id)
        : display(display), postfx(postfx), ships(ships), sys(sys),
          sun(sun), home(home), args(args), sim_win_id(sim_win_id) {}
};
