// events.cpp -- the game's input-event dispatch (declared in events.h).
//
// This was the "EVENTS" section inside main's loop: the synthetic (sim)
// input emission, then the SDL_PollEvent dispatch (the keybinds, the RMB
// camera-look and the wheel). Every state access goes through Game, so the
// section moved out of main() as-is (main locals -> g members). The LOGIC
// section (thrust / rotation commands) and the RENDER section stay in main.
#include "events.h"

#include <cstdio>
#include <cstdlib>   // std::abs (the RMB click-vs-drag motion total)

#include <GL/glew.h>   // glPolygonMode (F11 wireframe) + GL constants

#include "eva.h"        // Kerbal (the space-key jump edge)
#include "siminput.h"   // SimKeyPress, SimMouseAction
#include "gldebug.h"    // check_gl_error()

#include "../middleware/imgui/imgui.h"
#include "../middleware/imgui/backends/imgui_impl_sdl2.h"

void emit_sim_events(Game &g) {
    /* --sim-press: emit the synthetic key events that fell due this
       frame, in down-then-up order per press. They are polled below in
       the same frame, so one-shot actions fire in the frame the press
       is due. */
    if(!g.args.sim_presses.empty()) {
        const Uint32 now = SDL_GetTicks() - g.loop_start_ms;
        auto push_key = [&](SDL_EventType type, const SimKeyPress &p) {
            SDL_Event kev = {0};
            kev.type = type;
            kev.key.windowID = g.sim_win_id;
            kev.key.state = (type == SDL_KEYDOWN) ? SDL_PRESSED : SDL_RELEASED;
            kev.key.repeat = 0;
            kev.key.keysym.sym = p.key;
            kev.key.keysym.scancode = p.sc;
            SDL_PushEvent(&kev);
        };
        for(auto &p : g.args.sim_presses) {
            if(!p.down_sent && now >= p.down_ms) {
                push_key(SDL_KEYDOWN, p);
                p.down_sent = true;
            }
            if(p.down_sent && !p.up_sent && now >= p.up_ms) {
                push_key(SDL_KEYUP, p);
                p.up_sent = true;
            }
        }
    }

    /* --sim-mouse: emit the synthetic mouse events that fell due this
       frame, in the order each gesture needs. A drag (button + held)
       presses the button BEFORE moving so the camera-look handler
       (gated on rmbCam) sees the button down first; a click moves the
       cursor into place then presses + releases in place; BTN==0 just
       repositions. The motion carries the delta from the previous
       simulated position (args.sim_mouse_x/y), which the camera consumes. */
    if(!g.args.sim_mouse_actions.empty()) {
        const Uint32 now = SDL_GetTicks() - g.loop_start_ms;
        auto push_motion = [&](int x, int y) {
            SDL_Event mev = {0};
            mev.type = SDL_MOUSEMOTION;
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
            bev.button.state = (type == SDL_MOUSEBUTTONDOWN) ? SDL_PRESSED
                                                             : SDL_RELEASED;
            bev.button.x = x;
            bev.button.y = y;
            SDL_PushEvent(&bev);
        };
        for(auto &a : g.args.sim_mouse_actions) {
            if(!a.started && now >= a.time_ms) {
                if(a.button != 0 && a.up_ms > a.time_ms) {
                    // drag: press, then move (release comes at up_ms)
                    push_btn(SDL_MOUSEBUTTONDOWN, a.button, a.x, a.y);
                    push_motion(a.x, a.y);
                } else if(a.button != 0) {
                    // click: move into place, press, release (same frame)
                    push_motion(a.x, a.y);
                    push_btn(SDL_MOUSEBUTTONDOWN, a.button, a.x, a.y);
                    push_btn(SDL_MOUSEBUTTONUP, a.button, a.x, a.y);
                    a.released = true;
                } else {
                    // move only (no button)
                    push_motion(a.x, a.y);
                }
                a.started = true;
            }
            // release a held button at up_ms
            if(a.button != 0 && a.started && !a.released
               && now >= a.up_ms) {
                push_btn(SDL_MOUSEBUTTONUP, a.button, a.x, a.y);
                a.released = true;
            }
        }
    }

    /* --sim-mode: the scripted display-mode changes that fell due this
       frame (the same Renderer::setWindowMode path the Settings
       dropdowns use; the SIZE_CHANGED event in poll_events finishes the
       resize). */
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
}

void poll_events(Game &g) {
    SDL_Event ev;

    while (SDL_PollEvent(&ev)) {
        ImGui_ImplSDL2_ProcessEvent(&ev);
        if (ev.type == SDL_QUIT) {
            g.running = false;
        }

        if (ev.type == SDL_WINDOWEVENT) {
            if(ev.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                g.display.onResize(ev.window.data1, ev.window.data2);
                check_gl_error();

                g.postfx->Resize(ev.window.data1, ev.window.data2);
                check_gl_error();

                g.camera->setAspect((float)ev.window.data1 / (float)ev.window.data2);
                // the terrain LOD (screen-px budget) reads the live one
                g.camera->setViewport(ev.window.data1, ev.window.data2);
                check_gl_error();
            }
        }
        if(ev.type == SDL_KEYDOWN) {
            if(ev.key.keysym.sym == SDLK_PERIOD) {
                // Warp up one step (10x), capped at 100000 (ladder top).
                // Crossing into rails warp (>= kRailsWarp, i.e. accel > 10)
                // requires every ship to be rail-eligible: the active ship
                // coasts (or freezes on the ground) and the physics world
                // stops stepping; if any ship is not eligible the step is
                // refused and the current warp stays.
                const int next = (g.time_accel == 0) ? 1 : g.time_accel * 10;
                if(next > 100000) {
                    g.toast("Max warp reached");
                } else if(next < kRailsWarp || g.enter_rails_warp()) {
                    // enter_rails_warp toasted the refusal reason itself.
                    g.time_accel = next;
                    if(next >= kRailsWarp) {
                        printf("Rails warp: time accel %d (ships on rails)\n",
                               next);
                        g.toast("Time accel: %dx (rails)", next);
                    } else {
                        g.toast("Time accel: %dx", next);
                    }
                }
            }
            if(ev.key.keysym.sym == SDLK_COMMA) {
                if(g.time_accel > 1) {
                    const bool leaving_rails_warp =
                        (g.time_accel >= kRailsWarp) && (g.time_accel / 10 < kRailsWarp);
                    g.time_accel /= 10;
                    if(leaving_rails_warp) {
                        // dropped out of rails warp: the active ship
                        // re-enters physics (idle ships stay parked)
                        g.ship->leaveRails();
                        printf("Rails warp: exited, time accel %d\n", g.time_accel);
                    }
                    g.toast("Time accel: %dx", g.time_accel);
                }
                else if(g.time_accel == 1) {
                    g.time_accel = 0;
                    g.toast("Time accel: paused");
                }
            }
            if(ev.key.keysym.sym == SDLK_l) {
                if(g.cam_speed < 10000000) {
                    g.cam_speed *= 4;
                }
            }
            if(ev.key.keysym.sym == SDLK_k) {
                if(g.cam_speed > 1) {
                    g.cam_speed /= 4;
                }
            }
            if(ev.key.keysym.sym == SDLK_c) {
                // Toggle between the body-orbit camera and free flight.
                if(g.camera->mode == CAM_ORBIT) {
                    g.camera->toFree();
                } else {
                    g.camera->toOrbit(g.focusWorldPos(g.focusBody));
                    printf("Camera: orbiting %s (G = switch body, C = free)\n",
                           g.focusTargets[g.focusBody].name);
                }
            }
            if(ev.key.keysym.sym == SDLK_g) {
                // Cycle the orbit camera's target body.
                if(g.camera->mode == CAM_ORBIT) {
                    g.focusBody = (g.focusBody + 1) % g.numFocusTargets;
                    g.camera->Follow(g.focusWorldPos(g.focusBody));
                    double d = (g.focusTargets[g.focusBody].body == nullptr)
                        ? 50.0
                        : (double)g.focusTargets[g.focusBody].body->radius * 3.0;
                    g.camera->distance = d;
                    printf("Orbit camera targeting %s\n",
                           g.focusTargets[g.focusBody].name);
                } else {
                    printf("In free flight; press C to go to orbit, then G to switch body.\n");
                }
            }
            if(ev.key.keysym.sym == SDLK_TAB) {
                // toggle the info windows (one-shot; auto-repeat would
                // just keep flipping)
                if(!ev.key.repeat) {
                    g.toggle_windows();
                }
            }
            if(ev.key.keysym.sym == SDLK_F6) {
                // advance to the next selectable ship in the fleet, wrapping
                // around (one-shot; auto-repeat would keep cycling). Crew
                // characters aboard a capsule are skipped: they are not
                // controllable (EVA them from the capsule window first).
                if(!ev.key.repeat) {
                    std::vector<Vehicle *> all = collectVehicles(g.sys);
                    if(all.size() > 1) {
                        // the active ship's position in the canonical order
                        int cur = -1;
                        for(size_t i = 0; i < all.size(); i++) {
                            if(all[i] == g.ship) { cur = (int)i; break; }
                        }
                        if(cur >= 0) {
                            const int n = (int)all.size();
                            for(int step = 1; step < n; step++) {
                                Vehicle *v = all[(cur + step) % n];
                                if(v->isCrewAboard()) { continue; }
                                g.select_ship(v);
                                break;
                            }
                        }
                    }
                }
            }
            if(ev.key.keysym.sym == SDLK_v) {
                // toggle EVA: spawn/re-select the kerbal, or hand control
                // back to the ship (game.cpp). One-shot.
                if(!ev.key.repeat) {
                    g.toggle_eva();
                }
            }
            if(ev.key.keysym.sym == SDLK_SPACE) {
                // EVA: space is the jump key -- the KEYDOWN edge arms it
                // (evaArmCommands consumes the request on the next tick;
                // an event edge, because a quick tap can end before any
                // tick polls the key state).
                if(!ev.key.repeat && g.ship->isEva()) {
                    static_cast<Kerbal *>(g.ship)->jumpPressed = true;
                }
                // separate the active stage (one-shot; auto-repeat would
                // keep dropping stages). Only while flying a ship with
                // time running (a paused separation would leave the
                // survivors frozen mid-air).
                if(!ev.key.repeat && g.camera->mode == CAM_ORBIT && g.time_accel > 0
                   && !g.ship->isEva()) {
                    // staging needs the parts in the physics world:
                    // wake a ship parked on rails first
                    if(g.ship->onRails) {
                        g.ship->leaveRails();
                        if(g.time_accel >= kRailsWarp) {
                            g.time_accel = 1;
                            g.toast("Staging: left the rails, warp 1x");
                        }
                    }
                    // Refuse to drop a stage that still carries a crewed
                    // capsule (the crew's `aboard` state would dangle); EVA
                    // the crew out first. Check the parts that WOULD be
                    // dropped (the decoupler's child side), not all parts on
                    // the stage -- a sibling branch sharing the stage is fine.
                    bool crewOnStage = false;
                    for(Part *p : g.ship->droppedPartsAtStage(g.ship->activeStage())) {
                        if(p->def == nullptr || p->def->crew_capacity <= 0) { continue; }
                        for(size_t i = 0; i < g.ship->parts.size(); i++) {
                            if(g.ship->parts[i] == p && !partCrew(g.ship, i).empty()) {
                                crewOnStage = true;
                            }
                        }
                    }
                    if(crewOnStage) {
                        // the crew is locked to the vessel: staging the
                        // capsule would strand them (their `aboard` dangles)
                        printf("Stage: refused -- crew aboard the capsule (EVA them first)\n");
                        g.toast("Cannot stage -- EVA the capsule's crew out first");
                    } else {
                        int dropped = g.ship->separateStage(g.ship->activeStage());
                        g.ship->advanceStage();
                        if(dropped > 0) {
                            printf("Stage: dropped %d part(s); now on stage %d of %d\n",
                                   dropped, g.ship->activeStage(), g.ship->numStages());
                        } else {
                            printf("Stage: nothing left to separate\n");
                        }
                    }
                }
            }
            if(ev.key.keysym.sym == SDLK_F12) {
                g.screenshot_requested = true;
            }
            if(ev.key.keysym.sym == SDLK_p) {
                // Compute the porkchop plot for the current transfer target
                // (one-shot; auto-repeat would just recompute it). The render
                // pass consumes the flag and runs the (expensive) grid.
                if(!ev.key.repeat) {
                    g.porkchop_compute_requested = true;
                }
            }
            if(ev.key.keysym.sym == SDLK_m) {
                // Compute the surface map (one-shot, same pattern as P).
                if(!ev.key.repeat) {
                    g.surfmap_compute_requested = true;
                }
            }
            if(ev.key.keysym.sym == SDLK_F11) {
                if(g.poly_mode == false) {
                    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                    g.poly_mode = true;
                } else {
                    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                    g.poly_mode = false;
                }
            }
            if(ev.key.keysym.sym == SDLK_F10) {
                // Reset the window layout to defaults (same as the
                // main menu's "Reset windows" button).
                ui::ResetGui();
            }
            if(ev.key.keysym.sym == SDLK_ESCAPE) {
                // Toggle the main menu.
                ui::SetOpen("Main Menu", !ui::IsOpen("Main Menu"));
            }
        }
        if(ev.type == SDL_MOUSEBUTTONDOWN) {
            // holding RMB over 3D (not over a UI window) moves the camera.
            if(ev.button.button == SDL_BUTTON_RIGHT &&
               !ImGui::GetIO().WantCaptureMouse) {
                g.rmbCam = true;
                g.rmbDownX = ev.button.x;
                g.rmbDownY = ev.button.y;
                g.rmbDownMs = SDL_GetTicks();
                g.rmbMoved = 0;
            }
        }
        if(ev.type == SDL_MOUSEBUTTONUP) {
            if(ev.button.button == SDL_BUTTON_RIGHT) {
                g.rmbCam = false;
                // A short, still RMB press over the 3D view is a CLICK
                // (pick the part under the cursor); a moved one was the
                // camera drag. The camera already got its (sub-threshold)
                // look for a jittery click -- at 6 px that is <1 deg.
                if(!ImGui::GetIO().WantCaptureMouse
                   && g.rmbMoved < kPickClickPx
                   && SDL_GetTicks() - g.rmbDownMs < kPickClickMs) {
                    pickAt(g, ev.button.x, ev.button.y);
                }
            }
        }
        if(ev.type == SDL_MOUSEMOTION) {
            if(g.rmbCam && !ImGui::GetIO().WantCaptureMouse) {
                g.rmbMoved += std::abs(ev.motion.xrel)
                            + std::abs(ev.motion.yrel);
                g.camera->RotateY(-ev.motion.xrel / 200.0f);
                g.camera->Pitch(ev.motion.yrel / 200.0f);
            }
        }
        if(ev.type == SDL_MOUSEWHEEL) {
            // Zoom when the wheel is not scrolling a UI window.
            if(!ImGui::GetIO().WantCaptureMouse) {
                g.camera->wheel(ev.wheel.y);
            }
        }
    }
}
