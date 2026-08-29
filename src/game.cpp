// game.cpp -- the control transitions of the running game (declared in
// game.h): switching / removing the active ship, entering rails warp, the
// UI window toggle and the orbit-camera focus resolution. These were
// lambdas in main(); they touch the fleet (Ships) and the runtime state
// (Game), so they live with the state.
#include "game.h"

#include <cstdio>
#include <string>

glm::dvec3 Game::focusWorldPos(int i) const {
    if (focusTargets[i].body == nullptr) {
        return ship->get_center_of_mass();
    }
    return focusTargets[i].body->frame->GetPositionRelTo(ship->frame);
}

void Game::toggle_windows() {
    ui_visible = !ui_visible;
    for(auto &w : ui_windows) {
        ui::SetOpen(w.name, ui_visible && w.opts.default_open);
    }
    ui::SetOpen("HUD", ui_visible && o_hud.default_open);
}

/* Switch the active (controlled) ship. The ship being left is released:
   throttle zeroed, armed thrust + rotation commands cleared, and it parks
   on rails (coasting or frozen) if it can. The ship being taken re-enters
   physics. Taking control during rails warp drops the warp to 1000 -- the
   active ship is now being integrated. The orbit camera recenters on the
   ship being taken. */
void Game::select_ship(int idx) {
    if(idx < 0 || idx >= (int)ships.size() || idx == activeIdx) { return; }
    ships[activeIdx]->releaseControl();
    ships[activeIdx]->goOnRails();
    activeIdx = idx;
    ships[activeIdx]->leaveRails();
    ship = ships[activeIdx];
    if(time_accel >= kRailsWarp) { time_accel = 1000; }
    focusBody = 0;   // back to the "ship" focus target
    if(camMode == CAM_ORBIT) {
        orbitCam->Follow(ship->get_center_of_mass());
        orbitCam->distance = 50.0;
    }
    printf("Active ship %d of %d: %s\n",
           activeIdx + 1, (int)ships.size(), ship->name.c_str());
}

/* Enter rails warp: park every ship (flying ones coast on their conic,
   grounded ones freeze on the ground). Refuses -- and keeps the current
   accel -- if any ship is not rail-eligible, e.g. a suborbital descent in
   progress. */
bool Game::enter_rails_warp() {
    for(auto *s : ships) {
        if(!s->canRail()) {
            printf("Rails warp refused: '%s' is neither in free fall nor "
                   "grounded (warp stays %d)\n", s->name.c_str(), time_accel);
            return false;
        }
    }
    for(auto *s : ships) { s->goOnRails(); }
    return true;
}

/* Remove a ship + its bookkeeping. The Vehicle dtor detaches the welds
   and unregisters the bodies (skipped when the ship is already parked on
   rails), so this is safe in any state. Refuses to remove the last ship.
   If the removed ship was active, control hands off to the next ship in
   the list (or the last one). */
void Game::remove_ship(int idx) {
    if(idx < 0 || idx >= (int)ships.size()) { return; }
    if(ships.size() <= 1) {
        printf("Refusing to remove the last ship\n");
        return;
    }
    Vehicle *v = ships[idx];
    const bool wasActive = (idx == activeIdx);
    const std::string removedName = v->name;
    if(wasActive) { v->releaseControl(); }

    ships.erase_ship((size_t)idx);   // erase the 4 fleet vectors + delete v

    if(wasActive) {
        activeIdx = (idx >= (int)ships.size()) ? (int)ships.size() - 1 : idx;
        ships[activeIdx]->leaveRails();
        ship = ships[activeIdx];
        if(time_accel >= kRailsWarp) { time_accel = 1000; }
        focusBody = 0;
        if(camMode == CAM_ORBIT) {
            orbitCam->Follow(ship->get_center_of_mass());
            orbitCam->distance = 50.0;
        }
        printf("Removed '%s'; active ship %d of %d: %s\n",
               removedName.c_str(), activeIdx + 1, (int)ships.size(),
               ship->name.c_str());
    } else {
        if(idx < activeIdx) { activeIdx--; }
        printf("Removed '%s' (active unchanged: %s)\n",
               removedName.c_str(), ship->name.c_str());
    }
}
