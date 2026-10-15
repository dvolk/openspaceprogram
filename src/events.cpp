// events.cpp -- input-event dispatch (see events.h).
#include "events.h"

#include <cstdio>
#include <cstdlib>   // std::abs

#include <GL/glew.h>   // glPolygonMode + GL constants

#include "eva.h"        // Kerbal
#include "siminput.h"   // SimKeyPress, SimMouseAction
#include "uiinput.h"    // emit_ui_input
#include "gldebug.h"    // check_gl_error()
#include "uiwins.h"     // winOpen / setWinOpen
#include "vab.h"        // vabRotate / vabDeleteSelected

#include "../middleware/imgui/imgui.h"
#include "../middleware/imgui/backends/imgui_impl_sdl3.h"

void emit_sim_events(Game &g) {
    /* --sim-press: synthetic keys that fell due, down-then-up, polled in the
       same frame so one-shot actions fire on time. */
    if(!g.args.sim_presses.empty()) {
        const Uint32 now = SDL_GetTicks() - g.loop_start_ms;
        auto push_key = [&](SDL_EventType type, const SimKeyPress &p) {
            SDL_Event kev = {0};
            kev.type = type;
            kev.key.windowID = g.sim_win_id;
            kev.key.down = (type == SDL_EVENT_KEY_DOWN);
            kev.key.repeat = false;
            kev.key.key = p.key;
            kev.key.scancode = p.sc;
            SDL_PushEvent(&kev);
        };
        for(auto &p : g.args.sim_presses) {
            if(!p.down_sent && now >= p.down_ms) {
                push_key(SDL_EVENT_KEY_DOWN, p);
                p.down_sent = true;
                /* Hold for at least this frame: a short press whose down and up
                   fall due together has an empty held window and never fires. */
                continue;
            }
            if(p.down_sent && !p.up_sent && now >= p.up_ms) {
                push_key(SDL_EVENT_KEY_UP, p);
                p.up_sent = true;
            }
        }
    }

    /* --sim-mouse: synthetic mouse events in the order each gesture needs.
       Motion carries the delta from the previous simulated position. */
    if(!g.args.sim_mouse_actions.empty()) {
        const Uint32 now = SDL_GetTicks() - g.loop_start_ms;
        auto push_motion = [&](int x, int y) {
            SDL_Event mev = {0};
            mev.type = SDL_EVENT_MOUSE_MOTION;
            mev.motion.windowID = g.sim_win_id;
            mev.motion.which = 0;
            mev.motion.x = x;
            mev.motion.y = y;
            mev.motion.xrel = x - g.args.sim_mouse_x;
            mev.motion.yrel = y - g.args.sim_mouse_y;
            mev.motion.state = 0;
            SDL_PushEvent(&mev);
            g.args.sim_mouse_x = x;
            g.args.sim_mouse_y = y;
        };
        auto push_btn = [&](SDL_EventType type, int button, int x, int y) {
            SDL_Event bev = {0};
            bev.type = type;
            bev.button.windowID = g.sim_win_id;
            bev.button.which = 0;
            bev.button.button = (Uint8)button;
            bev.button.down = (type == SDL_EVENT_MOUSE_BUTTON_DOWN);
            bev.button.x = x;
            bev.button.y = y;
            SDL_PushEvent(&bev);
        };
        for(auto &a : g.args.sim_mouse_actions) {
            if(!a.started && now >= a.time_ms) {
                if(a.button == 4 || a.button == 5) {
                    // SDL3 wheel events carry the scroll amount in x/y
                    SDL_Event wev = {0};
                    wev.type = SDL_EVENT_MOUSE_WHEEL;
                    wev.wheel.windowID = g.sim_win_id;
                    wev.wheel.which = 0;
                    wev.wheel.x = 0.0f;
                    wev.wheel.y = (a.button == 4) ? 1 : -1;
                    wev.wheel.mouse_x = a.x;
                    wev.wheel.mouse_y = a.y;
                    // The imgui backend takes the mouse position from motion
                    // events only -- park the cursor first.
                    push_motion(a.x, a.y);
                    SDL_PushEvent(&wev);
                    a.started = true;
                    a.released = true;   // complete: skip the release path
                } else if(a.button != 0 && a.up_ms > a.time_ms) {
                    // drag: press, then move
                    push_btn(SDL_EVENT_MOUSE_BUTTON_DOWN, a.button, a.x, a.y);
                    push_motion(a.x, a.y);
                } else if(a.button != 0) {
                    // click: move, press, release (same frame)
                    push_motion(a.x, a.y);
                    push_btn(SDL_EVENT_MOUSE_BUTTON_DOWN, a.button, a.x, a.y);
                    push_btn(SDL_EVENT_MOUSE_BUTTON_UP, a.button, a.x, a.y);
                    a.released = true;
                } else {
                    push_motion(a.x, a.y);
                }
                a.started = true;
            }
            if(a.button != 0 && a.started && !a.released
               && now >= a.up_ms) {
                push_btn(SDL_EVENT_MOUSE_BUTTON_UP, a.button, a.x, a.y);
                a.released = true;
            }
        }
    }

    /* --sim-mode: scripted display-mode changes; poll_events' SIZE_CHANGED
       finishes the resize. */
    if(!g.args.sim_mode_changes.empty()) {
        const Uint32 now = SDL_GetTicks() - g.loop_start_ms;
        for(auto &m : g.args.sim_mode_changes) {
            if(!m.done && now >= m.at_ms) {
                m.done = true;
                g.args.window_mode = m.mode;
                g.args.screen_width = m.width;
                g.args.screen_height = m.height;
                g.display.setWindowMode(m.mode, m.width, m.height);
            }
        }
    }

    /* --ui-click / --ui-list. The hook in uiinput.cpp, not this call, does the
       work: it must run inside imgui's NewFrame, after the SDL3 backend has
       queued the live cursor. */
    emit_ui_input(g);
}

/* Mutually exclusive and held until toggled again, exactly like the Autopilot
   window's buttons. Gated on isEva() because tick.cpp applies slewRequest only
   to ships -- a toast there would promise a slew that never happens. Free cam
   is NOT a reason to refuse: the ship still simulates. */
static void toggleSlew(Game &g, SlewMode m, const char *name) {
    Vehicle *ship = g.ship;
    if(ship == nullptr || ship->isEva()) { return; }
    const bool engaged = (ship->slewRequest == m);
    ship->setSlewRequest(engaged ? SlewNone : m);
    printf("Autopilot: %s %s\n", name, engaged ? "off" : "engaged");
    g.toast("Autopilot: %s %s", name, engaged ? "off" : "engaged");
}

/* Time-warp slots are scene-neutral: dispatched in poll_events. */
void flightKeyActions(Game &g, SDL_Scancode ksc, Uint16 kmod, bool repeat) {
    if(slotFired(Slot::CamSpeedUp, ksc, kmod, g.binds)) {
        if(g.cam_speed < 10000000) {
            g.cam_speed *= 4;
        }
    }
    if(slotFired(Slot::CamSpeedDown, ksc, kmod, g.binds)) {
        if(g.cam_speed > 1) {
            g.cam_speed /= 4;
        }
    }
    if(slotFired(Slot::ToggleCamMode, ksc, kmod, g.binds) && !repeat) {
        // Zero the cam shake first: the transitions sample the live pose.
        g.shake_off = glm::dvec3(0.0);
        g.shake_ang = glm::dvec3(0.0);
        const bool first_ok = g.ship && !g.ship->isEva();
        if(first_ok) {
            // Ship: C is orbit <-> first person. From free, enter first.
            if(g.camera->mode == CAM_FIRST) {
                g.camera->toOrbit(g.focusWorldPos(g.focusBody));
                printf("Camera: orbiting %s (C = first person, Shift-C = free)\n",
                       g.focusTargets[g.focusBody].name);
                g.toast("Camera: orbiting %s", g.focusTargets[g.focusBody].name);
            } else {
                g.camera->toFirst();
                printf("Camera: first person (C = orbit, Shift-C = free)\n");
                g.toast("Camera: first person");
            }
        } else {
            // Kerbal / no ship: C is orbit <-> free (no first person).
            if(g.camera->mode == CAM_ORBIT) {
                g.camera->toFree();
                g.toast("Camera: free");
            } else {
                g.camera->toOrbit(g.focusWorldPos(g.focusBody));
                printf("Camera: orbiting %s (G = switch body, C = free)\n",
                       g.focusTargets[g.focusBody].name);
                g.toast("Camera: orbiting %s", g.focusTargets[g.focusBody].name);
            }
        }
    }
    if(slotFired(Slot::ToggleFreeCam, ksc, kmod, g.binds) && !repeat) {
        g.shake_off = glm::dvec3(0.0);
        g.shake_ang = glm::dvec3(0.0);
        if(g.camera->mode == CAM_FREE) {
            g.camera->toOrbit(g.focusWorldPos(g.focusBody));
            printf("Camera: orbiting %s\n", g.focusTargets[g.focusBody].name);
            g.toast("Camera: orbiting %s", g.focusTargets[g.focusBody].name);
        } else {
            g.camera->toFree();
            printf("Camera: free (Shift-C = orbit)\n");
            g.toast("Camera: free flight (Shift-C to leave)");
        }
    }
    if(slotFired(Slot::CycleTarget, ksc, kmod, g.binds) && !repeat) {
        // Cycle the orbit camera's target body.
        if(g.camera->mode == CAM_ORBIT) {
            g.focusBody = (g.focusBody + 1) % (int)g.focusTargets.size();
            g.camera->Follow(g.focusWorldPos(g.focusBody));
            double d = (g.focusTargets[g.focusBody].body == nullptr)
                ? 50.0
                : (double)g.focusTargets[g.focusBody].body->radius * 3.0;
            g.camera->distance = d;
            printf("Orbit camera targeting %s\n", g.focusTargets[g.focusBody].name);
            g.toast("Camera target: %s", g.focusTargets[g.focusBody].name);
        } else if(g.camera->mode == CAM_FIRST) {
            printf("In first person; press C for the orbit view first.\n");
            g.toast("In first person; press C for the orbit view");
        } else {
            printf("In free flight; press C or Shift-C to leave free cam.\n");
            g.toast("In free flight; press C or Shift-C to leave");
        }
    }
    if(slotFired(Slot::NextShip, ksc, kmod, g.binds)) {
        /* Next selectable ship, wrapping. One-shot. Every vessel the walk
           returns is flyable (a seated kerbal is a seat on its capsule Part,
           not a vehicle -- part.h); a walking kerbal still cycles. */
        if(!repeat) {
            std::vector<Vehicle *> all = collectVehicles(g.sys);
            if(all.size() > 1) {
                int cur = -1;
                for(size_t i = 0; i < all.size(); i++) {
                    if(all[i] == g.ship) { cur = (int)i; break; }
                }
                if(cur >= 0) {
                    const int n = (int)all.size();
                    for(int step = 1; step < n; step++) {
                        Vehicle *v = all[(cur + step) % n];
                        g.select_ship(v);
                        break;
                    }
                }
            }
        }
    }
    if(slotFired(Slot::ToggleEva, ksc, kmod, g.binds)) {
        if(!repeat) {
            g.toggle_eva();
        }
    }
    if(slotFired(Slot::Space, ksc, kmod, g.binds)) {
        // EVA: space is the jump key (edge-armed; evaArmCommands consumes).
        if(!repeat && g.ship && g.ship->isEva()) {
            static_cast<Kerbal *>(g.ship)->jumpPressed = true;
        }
        // No staging from free cam or with time stopped.
        if(!repeat && g.ship && g.camera->mode != CAM_FREE && g.time_accel > 0
           && !g.ship->isEva()) {
            g.stage();
        }
    }
    if(slotFired(Slot::Undock, ksc, kmod, g.binds)) {
        if(!repeat && g.ship && g.camera->mode != CAM_FREE && g.time_accel > 0
           && !g.ship->isEva()) {
            g.undock();
        }
    }
    if(slotFired(Slot::Porkchop, ksc, kmod, g.binds)) {
        // The render pass runs the expensive grid.
        if(!repeat) {
            g.porkchop_compute_requested = true;
        }
    }
    if(slotFired(Slot::SurfaceMap, ksc, kmod, g.binds)) {
        if(!repeat) {
            g.surfmap_compute_requested = true;
        }
    }
    /* F1 / F2: diagnostic overlays. Gated on the scene owning the window. */
    if(slotFired(Slot::DebugInfo, ksc, kmod, g.binds)) {
        if(!repeat && winInScene(g, W_Debug)) {
            setWinOpen(W_Debug, !winOpen(W_Debug));
        }
    }
    if(slotFired(Slot::Telemetry, ksc, kmod, g.binds)) {
        if(!repeat && winInScene(g, W_Telemetry)) {
            setWinOpen(W_Telemetry, !winOpen(W_Telemetry));
        }
    }
    if(slotFired(Slot::ResetWindows, ksc, kmod, g.binds)) {
        ui::ResetGui();
    }
    if(slotFired(Slot::Menu, ksc, kmod, g.binds) && !repeat) {
        // The title shares this slot but is already at the top: gate on Flight.
        if(sceneIs(g, SceneId::Flight)) {
            pushScene(g, SceneId::SpaceCenter);
        }
    }
    /* One-key attitude-to-velocity: Z holds prograde, B holds retrograde. The
       Autopilot window's toggles on keys -- opening a window to point the ship
       mid-burn is the friction that makes transfers unusable. */
    if(slotFired(Slot::SlewPrograde, ksc, kmod, g.binds) && !repeat) {
        // Same words as the window's buttons, so the toast reads identically.
        toggleSlew(g, SlewPrograde, "Prograde");
    }
    if(slotFired(Slot::SlewRetrograde, ksc, kmod, g.binds) && !repeat) {
        toggleSlew(g, SlewRetrograde, "Retrograde");
    }
    // The latch keeps engines lit with the key released; a plain thrust press
    // takes manual control back.
    if(slotFired(Slot::ThrustLatch, ksc, kmod, g.binds)) {
        if(!repeat) {
            g.thrust_latched = !g.thrust_latched;
            printf("Thrust latch: %s\n", g.thrust_latched ? "engaged" : "off");
            g.toast("Thrust latch %s", g.thrust_latched ? "engaged" : "off");
        }
    } else if(slotFired(Slot::Thrust, ksc, kmod, g.binds)
              && !repeat && g.thrust_latched) {
        g.thrust_latched = false;
        printf("Thrust latch: off (manual thrust)\n");
        g.toast("Thrust latch off");
    }
}

/* Fixed scancodes (editor-local). An imgui text field owns the keyboard. */
void vabKeyActions(Game &g, SDL_Scancode ksc, Uint16 kmod, bool repeat) {
    if(ImGui::GetIO().WantCaptureKeyboard) { return; }
    if(ksc == SDL_SCANCODE_Q) { vabRotate(g, -5.0); }
    if(ksc == SDL_SCANCODE_E) { vabRotate(g, +5.0); }
    if((ksc == SDL_SCANCODE_DELETE || ksc == SDL_SCANCODE_X) && !repeat) {
        /* Del DETACHES the selected subtree (Shift+Del truly deletes). A
           lone part is not a subassembly, so Del deletes it too. */
        if((kmod & SDL_KMOD_SHIFT) || g.vab.linkSel >= 0) { vabDeleteSelected(g); }
        else { vabDetachSelected(g); }
    }
    if(ksc == SDL_SCANCODE_ESCAPE && !repeat) {
        if(g.vab.linkMode) {
            g.vab.linkMode = false;   // the first Esc leaves link mode ...
            g.vab.linkFromId.clear();
        } else if(!g.vab.armed.empty() || g.vab.armedAsm >= 0) {
            // ... the next disarms whatever is armed ...
            g.vab.armed.clear();
            g.vab.armedAsm = -1;
            g.vab.ghostRoll = 0.0;
        } else {
            // ... then walks up the tree (vabClose logs + toasts like "Back").
            vabClose(g);
        }
    }
}

/* Same Esc slot as flight's, so rebinding moves both. */
void trackingKeyActions(Game &g, SDL_Scancode ksc, Uint16 kmod, bool repeat) {
    if(slotFired(Slot::Menu, ksc, kmod, g.binds) && !repeat) {
        popScene(g);   // the hub is the frame below
    }
}

void labKeyActions(Game &g, SDL_Scancode ksc, Uint16 kmod, bool repeat) {
    if(slotFired(Slot::Menu, ksc, kmod, g.binds) && !repeat) {
        popScene(g);   // the hub is the frame below
    }
}

/* Esc pops to the flight below, or is a no-op when the hub is the floor: it
   never exits to the title. Esc first dismisses an open Flight Summary. */
void hubKeyActions(Game &g, SDL_Scancode ksc, Uint16 kmod, bool repeat) {
    if(ImGui::GetIO().WantCaptureKeyboard) { return; }
    if(ksc == SDL_SCANCODE_ESCAPE && !repeat) {
        if(winOpen(W_FlightSummary)) {
            setWinOpen(W_FlightSummary, false);
            return;
        }
        if(g.sceneStack.size() > 1) {
            popScene(g);
        }
    }
}

void poll_events(Game &g) {
    SDL_Event ev;

    /* Modality is decided ONCE per batch: answerQuestion() runs the card's
       callback synchronously inside this loop, so re-reading g.question per
       event would let a second event in the same batch reach the scene under a
       card that is still up. */
    const bool modal = (g.question != nullptr);

    while (SDL_PollEvent(&ev)) {
        ImGui_ImplSDL3_ProcessEvent(&ev);
        if (ev.type == SDL_EVENT_QUIT) {
            g.running = false;
        }

        if (ev.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED) {
            g.display.onResize(ev.window.data1, ev.window.data2);
            check_gl_error();

            g.postfx->Resize(ev.window.data1, ev.window.data2);
            check_gl_error();

            /* A minimized window reports 0x0 and PostFX::Resize keeps its old
               targets for that; the camera must do the same. w/h == 0 makes the
               aspect inf or NaN, which builds a degenerate projection and
               poisons the frustum angles the terrain cull reads -- the cull
               silently stops culling until a real size arrives. */
            if(ev.window.data1 > 0 && ev.window.data2 > 0) {
                g.camera->setAspect((float)ev.window.data1 / (float)ev.window.data2);
                // the terrain LOD (screen-px budget) reads the live one
                g.camera->setViewport(ev.window.data1, ev.window.data2);
            }
            check_gl_error();
        }
        /* A pending question card is modal (#234). imgui still receives every
           event -- the scrim drawn under the card (drawQuestion) is what stops
           clicks reaching the windows behind -- but the game's OWN key and
           mouse handling stands down. Otherwise ESC popped a scene under a live
           card and its callback then ran from a different scene (its [g]
           captures outliving the scene that asked). */
        if(modal) {
            /* Re-read g.question, do not trust the batch latch for the pointer:
               answerQuestion() destroys the card (and a callback may open the
               NEXT one), so a second ESC in the same batch dereferenced a null
               unique_ptr -- two taps of ESC on a confirm card is a segfault. The
               batch stays modal either way: swallowing the press beats letting it
               reach the scene. */
            if(ev.type == SDL_EVENT_KEY_DOWN && !ev.key.repeat
               && ev.key.scancode == SDL_SCANCODE_ESCAPE
               && g.question != nullptr) {
                // ESC = the card's last choice, "Cancel" in every card.
                const int n = (int)g.question->labels.size();
                if(n > 0) { g.answerQuestion(n - 1); }
            }
            /* An RMB drag that started before the card must not stay armed: its
               release is swallowed here and g.rmbCam is cleared nowhere else, so
               the camera would follow the mouse with no button held. */
            g.rmbCam = false;
            continue;
        }
        if(ev.type == SDL_EVENT_KEY_DOWN && g.rebind_capture_slot >= 0) {
            // Bare modifiers keep capturing; a combo binds on its non-modifier key.
            const SDL_Scancode sc = ev.key.scancode;
            const bool modKey = (sc == SDL_SCANCODE_LSHIFT)
                || (sc == SDL_SCANCODE_RSHIFT) || (sc == SDL_SCANCODE_LCTRL)
                || (sc == SDL_SCANCODE_RCTRL)  || (sc == SDL_SCANCODE_LALT)
                || (sc == SDL_SCANCODE_RALT);
            if(sc != SDL_SCANCODE_UNKNOWN && !modKey) {
                const Uint16 mods = ev.key.mod & KMOD_RELEVANT;
                std::vector<KeyBind> &v =
                    g.binds.perSlot[(size_t)g.rebind_capture_slot];
                v.clear();
                v.push_back(KeyBind{sc, mods});
                g.rebind_capture_slot = -1;   // captured: back to idle
            }
        } else if(ev.type == SDL_EVENT_KEY_DOWN) {
            const SDL_Scancode ksc = ev.key.scancode;
            const Uint16 kmod = ev.key.mod;
            /* Scene-neutral slots: these work in flight AND in the editor. */
            if(slotFired(Slot::Screenshot, ksc, kmod, g.binds)) {
                g.screenshot_requested = true;
            }
            if(slotFired(Slot::Wireframe, ksc, kmod, g.binds)) {
                if(g.poly_mode == false) {
                    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                    g.poly_mode = true;
                } else {
                    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                    g.poly_mode = false;
                }
            }
            /* The orbital-plane overlay is a render toggle like wireframe, but
               the moment it is wanted is mid-burn -- so it flips the SAME flag
               the Settings checkbox writes (Game::draw_orbit_plane), never a
               second one that could disagree with the window. Scene-neutral:
               arming it before a plane change is the point, and a flag with no
               ship under it just sits there. */
            if(slotFired(Slot::ToggleOrbitPlane, ksc, kmod, g.binds)) {
                if(!ev.key.repeat) {
                    g.draw_orbit_plane = !g.draw_orbit_plane;
                    g.toast("Orbital plane %s",
                            g.draw_orbit_plane ? "on" : "off");
                }
            }
            if(slotFired(Slot::ToggleWindows, ksc, kmod, g.binds)) {
                if(!ev.key.repeat) {
                    g.toggle_windows();
                }
            }
            /* F5/F9 are scene-neutral. gameRunning gates the no-game states: a
               title-screen F5 would mint a phantom game dir. */
            if(!ev.key.repeat && gameRunning(g)) {
                if(slotFired(Slot::Quicksave, ksc, kmod, g.binds)) {
                    g.quicksave();
                }
                if(slotFired(Slot::Quickload, ksc, kmod, g.binds)) {
                    g.quickload();
                }
            }
            /* Time warp is a GLOBAL clock, so pause/resume is reachable anywhere. */
            if(slotFired(Slot::WarpUp, ksc, kmod, g.binds)) {
                // Crossing into rails warp (>= kRailsWarp) requires every ship
                // to be rail-eligible; otherwise the step is refused.
                const int next = (g.time_accel == 0) ? 1 : g.time_accel * 10;
                if(next > kMaxWarp) {
                    g.toast("Max warp reached (%dx)", kMaxWarp);
                } else if(next < kRailsWarp || g.enter_rails_warp()) {
                    // enter_rails_warp toasted the refusal reason itself.
                    g.time_accel = next;
                    if(next >= kRailsWarp) {
                        g.toast("Time accel: %dx (rails)", next);
                    } else {
                        g.toast("Time accel: %dx", next);
                    }
                }
            }
            if(slotFired(Slot::WarpDown, ksc, kmod, g.binds)) {
                if(g.time_accel > 1) {
                    const bool leaving_rails_warp =
                        (g.time_accel >= kRailsWarp) && (g.time_accel / 10 < kRailsWarp);
                    g.time_accel /= 10;
                    if(leaving_rails_warp && g.ship != nullptr) {
                        // Only the active ship re-enters physics; idle ones stay parked.
                        g.ship->leaveRails();
                    }
                    g.toast("Time accel: %dx", g.time_accel);
                }
                else if(g.time_accel == 1) {
                    g.time_accel = 0;
                    g.toast("Time accel: paused");
                }
            }

            /* Scene-switch shortcuts (1/2/3/4). Gated three ways: gameRunning,
               !WantCaptureKeyboard (digits are typed in text fields), and GoFlight
               needs an active ship + syncShipFocus (enterFlight skips its enter
               when the base is already Flight). */
            if(!ev.key.repeat && !ImGui::GetIO().WantCaptureKeyboard
                    && gameRunning(g)) {
                if(slotFired(Slot::GoSpaceCenter, ksc, kmod, g.binds)) {
                    goScene(g, SceneId::SpaceCenter);
                }
                if(g.ship != nullptr && slotFired(Slot::GoFlight, ksc, kmod, g.binds)) {
                    enterFlight(g);
                    g.syncShipFocus();
                }
                if(slotFired(Slot::GoTracking, ksc, kmod, g.binds)) {
                    goScene(g, SceneId::TrackingStation);
                }
                if(slotFired(Slot::GoVab, ksc, kmod, g.binds)) {
                    goScene(g, SceneId::Vab);
                }
            }

            // The live scene owns the key map.
            curScene(g).keys(g, ksc, kmod, ev.key.repeat);
        }
        if(ev.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
            // RMB over 3D (not over a UI window) moves the camera.
            if(ev.button.button == SDL_BUTTON_RIGHT &&
               !ImGui::GetIO().WantCaptureMouse) {
                g.rmbCam = true;
                g.rmbDownX = ev.button.x;
                g.rmbDownY = ev.button.y;
                g.rmbDownMs = SDL_GetTicks();
                g.rmbMoved = 0;
            }
        }
        if(ev.type == SDL_EVENT_MOUSE_BUTTON_UP) {
            if(ev.button.button == SDL_BUTTON_RIGHT) {
                g.rmbCam = false;
                // A short, still RMB press is a CLICK (pick the part); a moved
                // one was the camera drag. Flight only: the VAB's RMB orbits.
                if(sceneIs(g, SceneId::Flight)
                   && !ImGui::GetIO().WantCaptureMouse
                   && g.rmbMoved < kPickClickPx
                   && SDL_GetTicks() - g.rmbDownMs < kPickClickMs) {
                    pickAt(g, ev.button.x, ev.button.y);
                }
            }
        }
        if(ev.type == SDL_EVENT_MOUSE_MOTION) {
            if(g.rmbCam && !ImGui::GetIO().WantCaptureMouse) {
                g.rmbMoved += std::abs(ev.motion.xrel)
                            + std::abs(ev.motion.yrel);
                g.camera->RotateY(-ev.motion.xrel / 200.0f);
                g.camera->Pitch(ev.motion.yrel / 200.0f);
            }
        }
        if(ev.type == SDL_EVENT_MOUSE_WHEEL) {
            if(!ImGui::GetIO().WantCaptureMouse) {
                g.camera->wheel(ev.wheel.y);
            }
        }
    }
}
