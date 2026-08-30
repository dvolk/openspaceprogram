// gameui.cpp -- the ImGui UI pass (declared in gameui.h).
//
// This was the ImGui section of main's loop: the readout windows (HUD,
// the window list, Settings, TRANSFER, Game Debug Info, ORBITAL,
// TELEMETRY, SURFACE, SHIPS, VESSEL, SHIP PARTS, Controls, Autopilot,
// RESOURCES), the orbital map and the fixed main menu. They moved out
// as-is: main's locals became Game members (aliased in each function so
// the bodies read the same), and the per-frame state the readouts show
// is the ShipView snapshot the 3D pass (render.cpp) computes. Drawn in
// main in that order, keeping the original window order / z-order.
#include "gameui.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>

#include "calendar.h"    // CalTime (the HUD + Game Debug Info clocks)
#include "version.h"     // VERSION (the main menu)
#include "physics.h"     // GetAngVelocity (the VESSEL window)
#include "siminput.h"    // fmt_time (the TRANSFER window)
#include "orbitsample.h" // OrbitSampleCache + open-arc sampling (the map)
#include "orbitmap.h"    // OrbitMap + contrastingColor (the map)

#include "../middleware/imgui/imgui.h"
#include "../middleware/implot/implot.h"   // the TELEMETRY plots

// GLM's gtx extensions (glm::angle in ORBITAL) hard-error without this.
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/vector_angle.hpp>

void drawUIReadouts(Game &g, TransferPlanner &planner) {
    // The window bodies are verbatim from main's ImGui pass; their locals
    // are Game members (aliased so the bodies read the same).
    Vehicle *ship = g.ship;
    Ships &ships = g.ships;
    System &sys = g.sys;
    GameArgs &args = g.args;
    Camera *camera = g.camera;
    int &activeIdx = g.activeIdx;
    int &time_accel = g.time_accel;
    int &cam_speed = g.cam_speed;
    double &time = g.time;
    int &ui_style = g.ui_style;
    float &window_rounding = g.window_rounding;
    float &ui_alpha = g.ui_alpha;
    float &ui_scale = g.ui_scale;
    // The DPI slider edits this; "Apply DPI" commits it to ui_scale.
    // (1.0f matches the default ui_scale; only the Apply button changes
    // ui_scale, so they can't drift apart after that.)
    static float dpi_pending = 1.0f;
    // The Settings window writes these; the 3D pass (render.cpp) reads.
    bool &physics_debug_drawing = g.physics_debug_drawing;
    bool &world_drawing = g.world_drawing;
    bool &draw_starfield = g.draw_starfield;
    bool &draw_skylines = g.draw_skylines;

    // The per-frame state the 3D pass computed (render.cpp).
    ShipView &view = g.view;
    glm::dvec3 &pos = view.pos;
    glm::dvec3 &vel = view.vel;
    OrbitElements &o = view.o;
    double &distance = view.distance;
    double &speed = view.speed;
    glm::dvec3 &surf_vel = view.surf_vel;
    glm::dvec3 &facing_dir = view.facing_dir;
    glm::dvec3 &vel_dir = view.vel_dir;
    TimeSeries &energy_series = view.energy_series;
    TimeSeries &angmom_series = view.angmom_series;
    double &ver_speed = view.ver_speed;
    double &hor_speed2 = view.hor_speed2;
    double &latitude = view.latitude;
    double &longitude = view.longitude;
    double &pitch = view.pitch;
    double &roll = view.roll;
    double &yaw = view.heading;

    // The transfer planner's state (the TRANSFER window's readouts + inputs).
    std::vector<TransferPlanner::XferTarget> &xferTargets = planner.xferTargets;
    int &xfer_target = planner.xfer_target;
    bool &xfer_auto = planner.xfer_auto;
    float &xfer_tof_log = planner.xfer_tof_log;
    auto &xfer = planner.xfer;

    /* Top bar: one fixed window (no move, no resize, re-placed every
       frame so it tracks the viewport). Row 1: speed + altitude
       (big font) — orbital (ASL + orbital speed) when in the
       inertial frame or above 30km ASL, else surface (terrain
       altitude + ground speed) in the rotating frame.
       Row 2: Kerbin clock (regular font, centered). */
    ui::Window("HUD", g.o_hud, [&] {
        const double asl = distance - ship->m_parent->radius;
        const double agl = distance - ship->m_parent->GetTerrainHeight(glm::normalize(pos));
        const bool surface_mode = ship->frame->isRotFrame() && asl < 30000.0;
        const double alt = surface_mode ? agl : asl;
        const double spd = surface_mode ? glm::length(surf_vel) : speed;
        ImGui::PushFont(g.bigger);
        ImGui::Text("%06dm/s   %08dm", (int)spd, (int)alt);
        ImGui::PopFont();
        if(sys.home && sys.home->cal.valid()) {
            CalTime ct = sys.home->cal.at(time);
            char line[64];
            if(ct.has_year) {
                // CalTime only exposes month + day-of-month, so the
                // day-of-year is day + the days in the earlier months.
                int doy = ct.day;
                for(int m = 0; m < ct.month - 1; m++) {
                    doy += sys.home->cal.month_days[m];
                }
                snprintf(line, sizeof(line),
                         "Year %04d   Day %d/%d   %02d:%02d:%02d",
                         ct.year, doy, sys.home->cal.days_per_year,
                         ct.hh, ct.mm, ct.ss);
            } else {
                snprintf(line, sizeof(line),
                         "Day %d   %02d:%02d:%02d",
                         ct.day, ct.hh, ct.mm, ct.ss);
            }
            ImGui::SetCursorPosX((ImGui::GetWindowWidth() - ImGui::CalcTextSize(line).x) * 0.5f);
            ImGui::TextUnformatted(line);
        }
    });

    /* Window list (single source of truth: the ui_windows table) plus the
       Top-HUD group switch. */
    ui::Window("Windows", g.o_menu, [&] {
        ImGui::Spacing();
        for(auto &w : g.ui_windows) {
            bool open = ui::IsOpen(w.name);
            if(ImGui::Checkbox(w.label, &open)) {
                ui::SetOpen(w.name, open);
            }
        }
        bool hud = ui::IsOpen("HUD");
        if(ImGui::Checkbox("Top HUD", &hud)) {
            ui::SetOpen("HUD", hud);
        }
    });

    // Settings: the render/physics debug toggles (moved out of
    // Game Debug Info, which is now read-only diagnostics).
    ui::Window("Settings", g.o_settings, [&] {
        ImGui::Checkbox("Physics debug draw", &physics_debug_drawing);
        ImGui::Checkbox("World draw", &world_drawing);
        ImGui::Checkbox("Starfield", &draw_starfield);
        ImGui::Checkbox("Reference circles", &draw_skylines);
        if(ImGui::Combo("UI style", &ui_style, "Dark\0Light\0Classic\0")) {
            g.apply_ui_style();
        }
        // Control scheme: how the pitch/yaw stick maps to world axes.
        // Screen-aligned: W/S/A/D move the nose toward the screen
        // top/bottom/left/right (KSP style); heading-aligned: the
        // axes follow the ship itself (pitch about the ship's right
        // wing, yaw about its belly). Roll (Q/E) is about the nose
        // in both. Read live by Vehicle::applyRotationForce each tick.
        if(ImGui::Combo("Controls", &args.control_scheme,
                        "Screen-aligned\0Heading-aligned\0")) {
            g.toast("Controls: %s",
                    args.control_scheme ? "heading-aligned" : "screen-aligned");
        }
        if(ImGui::SliderFloat("Window rounding", &window_rounding,
                             0.0f, 50.0f, "%.0f")) {
            g.apply_ui_style();
        }
        if(ImGui::SliderFloat("Window transparency", &ui_alpha,
                             0.2f, 1.0f, "%.2f")) {
            g.apply_ui_style();
        }
        // The slider only edits the pending value; the Apply button
        // commits it. (Applying live while dragging would move this
        // window out from under the cursor, so the drag would land on
        // the wrong value.)
        ImGui::SliderFloat("DPI scale", &dpi_pending, 0.5f, 3.0f, "%.2fx");
        ImGui::BeginDisabled(dpi_pending == ui_scale);
        if(ImGui::Button("Apply DPI")) {
            ui_scale = dpi_pending;
            g.apply_ui_style();
            // TODO(dpi): the windows don't re-fit / re-place at the new
            // scale (fonts + padding scale, sizes stay put). A relayout
            // attempt DID re-fit them, but with STALE text metrics: the
            // first re-layout after a FontScaleDpi change sizes windows
            // with the PREVIOUS size's font advances (2x overflows, 1x
            // leaves slack); a second re-layout (F10 / "Reset windows")
            // fits correctly. Suspect the imgui 1.92 per-size font bakes
            // (ImFont::GetFontBaked / ImFontBaked::IndexAdvanceX).
        }
        ImGui::EndDisabled();
        if(ImGui::SliderFloat("FOV", &args.camFovDeg, 10.0f, 120.0f, "%.0f°")) {
            const float f = (float)glm::radians(args.camFovDeg);
            camera->setFov(f);
        }
        // Terrain LOD: a patch subdivides while it projects wider than
        // args.terrain_px (read live by GeoPatch::Update). The slider is
        // a 6-step detail level, right = finer: 1024px is the coarsest
        // and fastest to generate (the default), 256px the visual sweet
        // spot, 32px ~= 1px per mesh edge (slowest on software GL).
        {
            const int terrain_px_table[] = { 1024, 512, 256, 128, 64, 32 };
            const int nlevels = 6;
            int terrain_level = 0, best = -1;
            for (int i = 0; i < nlevels; i++) {
                const int d = std::abs(args.terrain_px - terrain_px_table[i]);
                if (best < 0 || d < best) { best = d; terrain_level = i; }
            }
            char terrain_label[48];
            snprintf(terrain_label, sizeof(terrain_label),
                     "Terrain detail (%d px)", terrain_px_table[terrain_level]);
            if(ImGui::SliderInt(terrain_label, &terrain_level, 0, nlevels - 1)) {
                args.terrain_px = terrain_px_table[terrain_level];
            }
        }
        // Test knob: scales the exhaust velocity of every engine (thrust
        // and delta-v scale by it, the fuel burn does not). Synced to the
        // ships each tick; takes effect within one physics step.
        ImGui::SliderFloat("Exhaust scale (test)", &args.exhaust_scale,
                           0.5f, 5.0f, "%.2fx");
        ImGui::Spacing();
        if(ImGui::Button("Back", ImVec2(240.0f, 0.0f))) {
            ui::SetOpen("Settings", false);
        }
    });

    // Transfer planner: parent->child body transfers (with capture)
    // and same-body ship intercepts. The solution is computed in the
    // render pass (xfer), so this window is pure readout + inputs.
    ui::Window("Transfer", g.o_transfer, [&] {
        if(xferTargets.empty()) {
            ImGui::Text("No transfer targets: no child bodies or ships here.");
            return;
        }
        const char *cur = (xfer_target >= 0)
                         ? xferTargets[xfer_target].name : "none";
        if(ImGui::BeginCombo("Target", cur)) {
            for(int i = 0; i < (int)xferTargets.size(); i++) {
                if(ImGui::Selectable(xferTargets[i].name,
                                     i == xfer_target)) {
                    xfer_target = i;
                }
            }
            ImGui::EndCombo();
        }
        if(xfer_target < 0) {
            ImGui::Text("Select a target body or ship.");
            return;
        }
        const bool isShip = xferTargets[xfer_target].ship != nullptr;
        if(isShip) {
            ImGui::TextDisabled("ship target: intercept only, no capture burn");
        }
        ImGui::Checkbox("Auto ToF (min dv)", &xfer_auto);
        if(!xfer_auto) {
            ImGui::SliderFloat("log10(ToF s)", &xfer_tof_log,
                               1.8, 7.5, "%.2f");
            ImGui::Text("ToF: %s",
                        fmt_time(std::pow(10.0, xfer_tof_log)).c_str());
        }
        if(!xfer.valid) {
            ImGui::Text("No transfer solution for this target / ToF.");
            return;
        }
        const TransferSolution &sol = xfer.sol;
        ImGui::Text("dv depart:  %08.1f m/s", sol.dv_departure);
        if(!isShip) {
            ImGui::Text("dv capture: %08.1f m/s @ %.0f km",
                        sol.dv_capture, sol.r_cap / 1000.0);
            if(sol.capture_orbit_period > 0.0) {
                ImGui::Text("capture P:  %s",
                            fmt_time(sol.capture_orbit_period).c_str());
            }
        }
        ImGui::Text("total dv:   %08.1f m/s", sol.total_dv);
        ImGui::Text("ToF:        %s", fmt_time(sol.tof).c_str());
        ImGui::Text("v_inf:      %08.1f m/s", sol.v_inf);
        if(sol.transfer_semi_major > 0.0) {
            ImGui::Text("transfer:   ellipse  a=%.6g m  e=%.3f",
                        sol.transfer_semi_major, sol.transfer_ecc);
        } else if(sol.transfer_semi_major < 0.0) {
            ImGui::Text("transfer:   hyperbolic  a=%.6g m  e=%.3f",
                        sol.transfer_semi_major, sol.transfer_ecc);
        } else {
            ImGui::Text("transfer:   parabolic  e=%.3f",
                        sol.transfer_ecc);
        }
    });

    ui::Window("Game Debug Info", g.o_debug, [&] {
        ImGui::Text("Time: %f", time);
        if(sys.home && sys.home->cal.valid()) {
            CalTime ct = sys.home->cal.at(time);
            if(ct.has_year) {
                ImGui::Text("Clock:  Yr %d  Mo %d  Day %d  %02d:%02d:%02d  (%s time)",
                            ct.year, ct.month, ct.day, ct.hh, ct.mm, ct.ss,
                            sys.home->name.c_str());
            } else {
                ImGui::Text("Clock:  Day %d  %02d:%02d:%02d  (%s time)",
                            ct.day, ct.hh, ct.mm, ct.ss,
                            sys.home->name.c_str());
            }
        }
        // Local date + time on the body the ship is currently in,
        // when it's not the home planet (e.g. the Moon's own day).
        TerrainBody *local_body = (ship && ship->frame)
                                ? ship->frame->body : nullptr;
        if(local_body && local_body != sys.home &&
           local_body->cal.valid()) {
            CalTime lt = local_body->cal.at(time);
            if(lt.has_year) {
                ImGui::Text("Local:  %s  Yr %d  Mo %d  Day %d  %02d:%02d:%02d",
                            local_body->name.c_str(),
                            lt.year, lt.month, lt.day,
                            lt.hh, lt.mm, lt.ss);
            } else {
                ImGui::Text("Local:  %s  Day %d  %02d:%02d:%02d",
                            local_body->name.c_str(),
                            lt.day, lt.hh, lt.mm, lt.ss);
            }
        }
        ImGui::Text("Patches: %d", ship->m_parent->CountPatches());
        ImGui::Text("Cam speed: %d", cam_speed);
        ImGui::Text("Time Accel: %d%s", time_accel,
                    time_accel >= kRailsWarp ? " (rails)" : "");
        ImGui::Text("Camera altitude: %0.f",
                    glm::length(camera->GetPos()) - ship->m_parent->GetTerrainHeight(glm::normalize(camera->GetPos())));
        ImGui::Text("Camera ASL: %0.f", glm::length(camera->GetPos()) - ship->m_parent->radius);
        ImGui::Text("Camera Pos: %.0f %.0f %0.f", camera->GetPos().x, camera->GetPos().y, camera->GetPos().z);
        ImGui::Text("Cam forward: %.2f %.2f %.2f",
                    camera->forward.x, camera->forward.y, camera->forward.z);
        if(ImGui::Button("Print camera pose (CLI args)")) {
            // Copy-paste the printed line after ./osp to relaunch at this view.
            // pos/forward/up are world / ship-frame coordinates.
            printf("Camera pose:\n");
            printf("--free-cam-pos %.9g %.9g %.9g "
                   "--free-cam-fwd %.9g %.9g %.9g "
                   "--free-cam-up %.9g %.9g %.9g "
                   "--fov %.0f\n",
                   camera->pos.x, camera->pos.y, camera->pos.z,
                   camera->forward.x, camera->forward.y, camera->forward.z,
                   camera->up.x, camera->up.y, camera->up.z,
                   args.camFovDeg);
        }
        ImGui::Text("Home distance: %f",
                    glm::length(ship->GetPositionRelTo(ship->controller,
                                                        ships.homeOf(activeIdx)->frame)));
        ImGui::Text("Pos: %.3fkm", distance / 1000);
        ImGui::Text("xyz(%0.f, %0.f, %0.f)", pos.x, pos.y, pos.z);
        ImGui::Text("Vel: %.3fm/s", speed);
        ImGui::Text("xyz(%0.f, %0.f, %0.f)", vel.x, vel.y, vel.z);
    });

    // Labels are abbreviated to <= 3 chars and right-padded to the
    // same width so the values start at a tidy column.
    ui::Window("Orbital", g.o_orbit, [&] {
        ImGui::Text("Vel: %.1fm/s", speed);
        ImGui::Text("Alt: %.1fm", distance);
        /* Every line below is always present; "-" = the quantity
           does not exist for this orbit class. Escape trajectories
           have no apoapsis and no period; a near-circular orbit
           (apsides within 10 km) has no apsis line, so the
           countdowns to one are numerically meaningless. */
        const bool circular = o.ecc < 1.0
                            && (o.apoapsis - o.periapsis) < 10e3;
        if(o.ecc < 1.0) { ImGui::Text("ApA: %.1fm", o.apoapsis); }
        else { ImGui::Text("ApA: -"); }
        if(o.ecc < 1.0 && !circular) { ImGui::Text("ApT: %.1fs", o.time_to_apo); }
        else { ImGui::Text("ApT: -"); }
        ImGui::Text("PeA: %.1fm", o.periapsis);
        if(!circular && o.time_to_peri >= 0.0) { ImGui::Text("PeT: %.1fs", o.time_to_peri); }
        else { ImGui::Text("PeT: -"); }
        if(o.period > 0.0) { ImGui::Text("  T: %.1fs", o.period); }
        else { ImGui::Text("  T: -"); }
        ImGui::Text("Inc: %.2f", glm::degrees(o.inclination));
        ImGui::Text("Ecc: %f", o.ecc);
        ImGui::Text("SMa: %.1fm", o.semi_major);
        ImGui::Text("LAN: %.2f", glm::degrees(o.raan));
        ImGui::Text("LPe: %.2f", glm::degrees(o.arg_periapsis));
        double prograde_angle = glm::angle(facing_dir, vel_dir);
        double retrograde_angle = glm::angle(facing_dir, - vel_dir);
        ImGui::Text("Prg: %.2f", glm::degrees(prograde_angle));
        ImGui::Text("Rtg: %.2f", glm::degrees(retrograde_angle));
        ImGui::Text("Eng: %.2f J", o.energy);
    });

    // Initial size comes from o_telemetry.initial_size (the plots
    // need real estate; content-fit would clip them).
    ui::Window("Telemetry", g.o_telemetry, [&] {
        // Two separate plots: e (~1e5) and |h| (~1e11) differ by
        // ~6 orders of magnitude, so sharing one axis would flatten
        // e to a line and hide the drift we're looking for.
        if(energy_series.count > 1) {
            const int n = energy_series.stage();
            ImPlot::SetNextAxesToFit();  // live auto-fit, both axes
            if(ImPlot::BeginPlot("specific orbital energy (J/kg)")) {
                ImPlot::SetupAxis(ImAxis_X1, "t (s)");
                ImPlot::PlotLine("e", energy_series.t_arr(), energy_series.v_arr(), n);
                ImPlot::EndPlot();
            }
        }
        if(angmom_series.count > 1) {
            const int n = angmom_series.stage();
            ImPlot::SetNextAxesToFit();  // live auto-fit, both axes
            if(ImPlot::BeginPlot("angular momentum (m^2/s)")) {
                ImPlot::SetupAxis(ImAxis_X1, "t (s)");
                ImPlot::PlotLine("|h|", angmom_series.t_arr(), angmom_series.v_arr(), n);
                ImPlot::EndPlot();
            }
        }
    });

    // Labels right-padded to 3 chars, same as ORBITAL.
    ui::Window("Surface", g.o_surface, [&] {
        ImGui::Text("Alt: %.1fm", distance - ship->m_parent->GetTerrainHeight(glm::normalize(pos)));
        ImGui::Text("ASL: %.1fm", distance - ship->m_parent->radius);
        ImGui::Text(" Vs: %.2fm/s", ver_speed);
        ImGui::Text(" Hs: %.2fm/s", hor_speed2);
        ImGui::Text("Lat: %.4f", glm::degrees(latitude));
        ImGui::Text("Lon: %.4f", glm::degrees(longitude));
        ImGui::Text(" Pt: %.2f", glm::degrees(pitch));
        ImGui::Text("  R: %.2f", glm::degrees(roll));
        ImGui::Text("Hdg: %.2f", glm::degrees(yaw));
    });

    ui::Window("Ship List", g.o_ships, [&] {
    // Buttons (natural width) + SameLine, the same pattern as the
    // map controls: a full-width Selectable in this auto-resize window
    // would swallow the line and push the "x" off it (or collapse the
    // window), so each name is its own sized button. The active ship
    // is highlighted with a pushed color.
    for(size_t i = 0; i < ships.size(); i++) {
        const bool active = ((int)i == activeIdx);
        ImGui::PushID((int)i);
        if(active) {
            ImGui::PushStyleColor(ImGuiCol_Button,
                                 ImVec4(0.30f, 0.45f, 0.70f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                                 ImVec4(0.35f, 0.50f, 0.75f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                                 ImVec4(0.40f, 0.55f, 0.80f, 1.0f));
        }
        if(ImGui::Button(ships[i]->name.c_str())) {
            g.select_ship((int)i);
        }
        if(active) {
            ImGui::PopStyleColor(3);
        }
        ImGui::SameLine();
        if(ImGui::SmallButton("x")) {
            g.remove_ship((int)i);
            i = ships.size();   // the list shrank; stop iterating
        }
        ImGui::PopID();
    }
    ImGui::Separator();
    if(ImGui::Button("Spawn a copy of the active ship")) {
        if(!ship->defPath.empty()) {
            ships.spawn_ship(ship->defPath, "", ships.homeOf(activeIdx),
                             ships.scenarioOf(activeIdx), sys);
        } else {
            printf("Spawn: active ship has no def (test ship)\n");
        }
    }
    ImGui::Text("click name - select    x - remove");
    });

    ui::Window("Vessel Info", g.o_vessel, [&] {
        ImGui::Text("Ship: %s", ship->name.c_str());
        ImGui::Text("Stage: %d / %d  (SPACE to drop)",
                    ship->activeStage(), ship->numStages());
        ImGui::Text("Reference frame: %s", ship->frame->name.c_str());
        ImGui::Text("Reference frame type: %s", ship->frame->isRotFrame() ? "Rotational" : "Inertial");
        ImGui::Text("Mass: %.3fkg", ship->getMass());
        ImGui::Text("Delta-v: %.1fm/s", ship->getDeltaV());
        ImGui::Text("Thrust Util: %.0f%%", ship->thruster_util * 100);
        ImGui::Text("Thrust: %.2fN", ship->getThrust());
        ImGui::Text("Current TWR: %.2f/%.2f", ship->getTWR(), ship->getFullThrustTWR());
        ImGui::Text("Max TWR: %.2f", ship->getMaxTWR());
        ImGui::Text("Wheel torque: %.0fN m", ship->GetWheelTorque());
        ImGui::Text("Angular rate: %.2fdeg/s",
                    glm::degrees(glm::length(GetAngVelocity(ship->controller))));
    });
    ui::Window("Ship Parts", g.o_parts, [&] {
        int i = 0;
        for(auto&& part : ship->parts) {
            ImGui::Text("Part #%d  (stage %d)", i, ship->partStages[i]);
            ImGui::Separator();
            ImGui::Text("Name: %s", ship->partDefs[i]->name.c_str());
            ImGui::Text("Mass: %.3fkg", part->mass);
            ImGui::Text("Hydrogen: %.3fkg/%.3fkg",
                        ship->partResources[i].current[(int)ResourceType::Hydrogen],
                        ship->partResources[i].capacity[(int)ResourceType::Hydrogen]);
            ImGui::Text("LOX: %.3fkg/%.3fkg",
                        ship->partResources[i].current[(int)ResourceType::LOX],
                        ship->partResources[i].capacity[(int)ResourceType::LOX]);
            ImGui::Spacing();
            i++;
        }
    });

    ui::Window("Controls", g.o_controls, [&] {
        ImGui::Text("Game");
        ImGui::Separator();
        ImGui::Text("p - toggle wireframe mode");
        ImGui::Text(", - decrease time acceleration");
        ImGui::Text(". - increase time acceleration");
        ImGui::Text("k - decrease camera speed");
        ImGui::Text("l - increase camera speed");
        ImGui::Text("c - switch mode: orbit (flying) <-> free (exploring)");
        ImGui::Text("g - orbit mode: cycle target (ship/sun/planet/moon)");
        ImGui::Text("tab - toggle windows");
        ImGui::Text("f6 - next ship (fleet)");
        ImGui::Text("SHIPS window - select a ship, spawn a copy, remove one (x)");
        ImGui::Text("f10 - reset windows");
        ImGui::Text("esc - main menu");
        ImGui::Text("mouse - UI (hold RMB over 3D to look, both modes)");
        ImGui::Text("RMB (orbital map) - cycle window -> bare map -> no window");
        ImGui::Text("wheel - zoom (orbit mode)");
        ImGui::Spacing();
        ImGui::Text("Orbit mode (flying the ship)");
        ImGui::Separator();
        ImGui::Text("w/s - pitch up/down");
        ImGui::Text("a/d - yaw left/right");
        ImGui::Text("q/e - roll left/right (about the nose)");
        ImGui::Text("pitch/yaw axes: Settings -> Controls");
        ImGui::Text("i - fire ship engines");
        ImGui::Text("x - kill rotation");
        ImGui::Text("b - align prograde");
        ImGui::Text("n - align retrograde");
        ImGui::Text("r/f - throttle up/down");
        ImGui::Text("SPACE - separate the active stage");
        ImGui::Spacing();
        ImGui::Text("Free mode (exploring)");
        ImGui::Separator();
        ImGui::Text("w/s - forward/back");
        ImGui::Text("a/d - strafe");
        ImGui::Text("q/e - roll");
        ImGui::Text("shift/ctrl - up/down");
        ImGui::Spacing();
        if(ImGui::Button("Back", ImVec2(240.0f, 0.0f))) {
            ui::SetOpen("Controls", false);
        }
    });

    ui::Window("Autopilot", g.o_autopilot, [&] {
        ImGui::Button("Prograde");
        ImGui::Button("Retrograde");
        ImGui::Button("Radial-in");
        ImGui::Button("Radial-out");
        ImGui::Button("Normal");
        ImGui::Button("Anti-normal");
    });

    ui::Window("Resources", g.o_resources, [&] {
        // aggregate across the active ship's parts (any ship layout)
        float h_cur = 0, h_cap = 0, l_cur = 0, l_cap = 0;
        for(size_t i = 0; i < ship->partResources.size(); i++) {
            h_cur += ship->partResources[i].current[(int)ResourceType::Hydrogen];
            h_cap += ship->partResources[i].capacity[(int)ResourceType::Hydrogen];
            l_cur += ship->partResources[i].current[(int)ResourceType::LOX];
            l_cap += ship->partResources[i].capacity[(int)ResourceType::LOX];
        }
        const float hydrogen_frac = (h_cap > 0) ? h_cur / h_cap : 0.0f;
        const float lox_frac = (l_cap > 0) ? l_cur / l_cap : 0.0f;

        ImGui::ProgressBar(hydrogen_frac, ImVec2(-1, 0), "Hydrogen");
        ImGui::ProgressBar(lox_frac, ImVec2(-1, 0), "LOX");
        ImGui::ProgressBar(0.13, ImVec2(-1, 0), "Hydrazine");
        ImGui::ProgressBar(0.45, ImVec2(-1, 0), "Electric charge");
        ImGui::ProgressBar(0.75, ImVec2(-1, 0), "Oxygen");
        ImGui::ProgressBar(0.83, ImVec2(-1, 0), "Water");
        ImGui::ProgressBar(0.94, ImVec2(-1, 0), "Food");
    });
}

// The orbital map: right-clicking the window cycles its chrome (full
// window -> bare map -> no window), the map square draws the focus
// body's neighborhood (child-body orbits, SOI rings, the ship's
// trajectory + apside markers, the other ships, the transfer conic),
// and the controls below it edit the map state on the game.
void drawUIMap(Game &g, TransferPlanner &planner) {
    Vehicle *ship = g.ship;
    Ships &ships = g.ships;
    OrbitElements &o = g.view.o;
    double &mu = g.view.mu;
    glm::dvec3 &orbit_pos = g.view.orbit_pos;
    glm::dvec3 &orbit_vel = g.view.orbit_vel;
    std::vector<TerrainBody *> &planets = g.sys.bodies;
    float &map_scale = g.map_scale;
    int &map_plane = g.map_plane;
    ImVec2 &map_pan = g.map_pan;
    bool &map_show_soi = g.map_show_soi;
    bool &map_show_vel = g.map_show_vel;
    int &map_mode = g.map_mode;
    std::vector<TransferPlanner::XferTarget> &xferTargets = planner.xferTargets;
    int &xfer_target = planner.xfer_target;
    auto &xfer = planner.xfer;
    // Cached orbit samplings, one entry per orbiting object (keyed on its
    // pointer -- the ship for the ship's orbit, a body for a child's; a given
    // ship always has exactly one entry, overwritten on each sample). Reuse
    // is only trusted while the orbiting object is on a fixed Keplerian conic:
    // the ship passes its onRails flag, terrain bodies are always on theirs.
    // See OrbitSampleCache.
    static std::map<const void *, OrbitSampleCache> orbit_caches;

    // Mode 2 strips the window chrome entirely (see map_mode): the
    // window is invisible but still hit-tested, so the map below
    // keeps pan/zoom and the right-click cycle.
    if(map_mode == 2) {
        g.o_map.flags |= ImGuiWindowFlags_NoDecoration |
                            ImGuiWindowFlags_NoBackground;
    } else {
        g.o_map.flags = 0;
    }
    ui::Window("Orbital Map", g.o_map, [&] {
        // Right-click anywhere in the window cycles the chrome:
        // full window -> bare map -> no window -> full window.
        // Over the map this is safe: imgui owns the mouse here, so
        // the RMB camera orbit (gated on !WantCaptureMouse) never
        // fires.
        if(ImGui::IsWindowHovered() &&
           ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
            map_mode = (map_mode + 1) % 3;
        }
        const float kMapSize = 360.0f;   // the map square (layout below)
        // The ship's trajectory around the focus: a closed ellipse
        // (a coasting Kepler orbit) or, when the ship is escaping or
        // flying by (ecc >= 1 -- e.g. right after switching SOI to a
        // body you are approaching), an open hyperbolic/parabolic arc.
        // Both draw the same way (a projected polyline); only the
        // sampling differs. Top-down view in the focus's inertial
        // frame, so the trajectory's true 3D orientation shows through
        // the projection.
        const int N = 64;
        const bool closed = (o.ecc < 1.0);
        std::vector<glm::dvec3> traj_pts;
        if(closed) {
            // Sampled through a per-ship cache, trusted only while the
            // ship is on rails (coasting on its Keplerian conic). Off
            // rails -- Bullet-integrated, or right after a burn /
            // staging / SOI switch / crash (all of which clear onRails)
            // -- the orbit is moving, so re-sample every frame. See
            // OrbitSampleCache.
            traj_pts = orbit_caches[(const void *)ship].sample(
                orbit_pos, orbit_vel, mu, N, ship->onRails);
        } else {
            // Open trajectory: an arc around periapsis, truncated where
            // it would run off to infinity. r_cap is the current view
            // extent (the map square's width in world units) so the
            // curve reaches the edge of the view, but never smaller
            // than a few periapsis radii or the ship's current radius
            // (so the ship itself lies on the arc).
            const double r_cap = std::max<double>(
                kMapSize * (double)map_scale,
                std::max(4.0 * o.periapsis, o.distance));
            traj_pts = sampleOpenTrajectory(orbit_pos, orbit_vel, mu, N, r_cap);
        }
    
        // Periapsis (both cases) and apoapsis (closed only). A closed
        // orbit propagates to each apsis (exact); an open arc has no
        // apoapsis, and its periapsis point is radius o.periapsis
        // along the eccentricity vector (which points to periapsis) --
        // no propagation needed.
        glm::dvec3 peri_p, apo_p, tmp;
        bool have_peri = false, have_apo = false;
        if(closed) {
            if(o.time_to_peri > 0.0) {
                propagateKepler(orbit_pos, orbit_vel, mu, o.time_to_peri, peri_p, tmp);
                have_peri = true;
            }
            if(o.time_to_apo > 0.0) {
                propagateKepler(orbit_pos, orbit_vel, mu, o.time_to_apo, apo_p, tmp);
                have_apo = true;
            }
        } else {
            const glm::dvec3 h = glm::cross(orbit_pos, orbit_vel);
            const double hl = glm::length(h);
            if(hl > 1e-9) {
                const glm::dvec3 evec =
                    glm::cross(orbit_vel, h)/mu - orbit_pos/o.distance;
                const double el = glm::length(evec);
                if(el > 1e-9) {
                    peri_p = (o.periapsis / el) * evec;
                    have_peri = true;
                }
            }
        }
    
        // The focus body (the ship's parent) and the map plane.
        // The plane is a normal in the focus's inertial frame;
        // OrbitMap derives an in-plane basis from it. All three
        // candidates live in that frame:
        //   equatorial = the focus's reference plane (normal +Y);
        //   ecliptic   = the system reference plane (root XZ) expressed
        //                in the focus's frame;
        //   orbital    = the ship's own orbital plane (h = r x v).
        TerrainBody *focus = ship->m_parent;
        glm::dvec3 plane_n(0.0, 1.0, 0.0);
        if(map_plane == 1) {
            plane_n = glm::transpose(focus->frame->root_orient) *
                      glm::dvec3(0.0, 1.0, 0.0);
        } else if(map_plane == 2) {
            const glm::dvec3 h = glm::cross(orbit_pos, orbit_vel);
            const double hl = glm::length(h);
            if(hl > 1e-9) { plane_n = h / hl; }
        }
    
        // The map is a kMapSize square at the top of the window; the
        // focus (parent body) sits at its center plus the pan offset;
        // the controls go below. (kMapSize is defined at the top of
        // the block, where the open-trajectory radius cap uses it.)
        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        const float center_x = p0.x + kMapSize * 0.5f;
        const float center_y = p0.y + kMapSize * 0.5f;
    
        // Reserve the map square with an invisible button. It captures
        // the mouse, so a left-drag over the map pans the map instead
        // of moving the window (imgui otherwise treats a drag on the
        // window background as a window move). Wheel-zoom and drag-pan
        // both apply only while the mouse is over this square.
        ImGui::InvisibleButton("##mapnav", ImVec2(kMapSize, kMapSize));
        const bool over_map = ImGui::IsItemHovered();
        const ImGuiIO &g_io = ImGui::GetIO();
        if(over_map && g_io.MouseWheel != 0.0f) {
            // Wheel zooms to the cursor (the world point under the
            // mouse stays put). Reversed per preference: wheel UP zooms
            // IN (scale = meters/pixel goes down), wheel OUT zooms out.
            const float factor = (g_io.MouseWheel > 0.0f) ? 0.8f : 1.25f;
            const float old_scale = map_scale;
            float new_scale = old_scale * factor;
            // Clamp to the same range the Scale slider spans (10^3..10^9.5).
            const float min_scale = 1000.0f;
            const float max_scale = powf(10.0f, 9.5f);
            if(new_scale < min_scale) { new_scale = min_scale; }
            if(new_scale > max_scale) { new_scale = max_scale; }
            const ImVec2 mouse = ImGui::GetMousePos();
            const float u = mouse.x - (center_x + map_pan.x);
            const float v = mouse.y - (center_y + map_pan.y);
            map_pan.x = (mouse.x - u * old_scale / new_scale) - center_x;
            map_pan.y = (mouse.y - v * old_scale / new_scale) - center_y;
            map_scale = new_scale;
        }
        if(ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            map_pan.x += g_io.MouseDelta.x;
            map_pan.y += g_io.MouseDelta.y;
        }
    
        OrbitMap map;
        map.cx = center_x + map_pan.x;
        map.cy = center_y + map_pan.y;
        map.scale = map_scale;
        map.setPlane(plane_n);
    
        // KSP-inspired palette (P4): your orbit is green, the transfer
        // is blue, other bodies are gray. The focus body, ship dot and
        // labels use a near-black/white ink that contrasts with the
        // current style's window background, so they stay readable in
        // both the light and dark themes. The selected transfer target
        // is highlighted brighter than the other children.
        const ImVec4 bg = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
        const ImU32 ink       = contrastingColor(bg);
        const ImU32 col_ship  = ImGui::GetColorU32(ImVec4(0.20f, 0.80f, 0.40f, 1.0f));
        const ImU32 col_apsis = col_ship;  // periapsis / apoapsis: part of your orbit
        const ImU32 col_xfer  = ImGui::GetColorU32(ImVec4(0.35f, 0.55f, 1.00f, 1.0f));
        const ImU32 col_vessel = ImGui::GetColorU32(ImVec4(1.00f, 0.62f, 0.22f, 1.0f));
        const ImU32 col_child = ImGui::GetColorU32(ImVec4(0.55f, 0.55f, 0.55f, 1.0f));
        const ImU32 col_body  = ink;
        const ImU32 col_sel   = ImGui::GetColorU32(ImVec4(0.90f, 0.90f, 0.90f, 1.0f));
        const ImU32 soi_col   = ImGui::GetColorU32(ImVec4(0.50f, 0.50f, 0.50f, 0.30f));
        ImDrawList *dl = ImGui::GetWindowDrawList();
        const ImVec2 focus_px = map.px(glm::dvec3(0.0, 0.0, 0.0));
    
        // The body selected in the TRANSFER window (a child of the
        // focus), highlighted on the map; nullptr for a ship target or
        // no selection.
        TerrainBody *sel_body = nullptr;
        if(xfer_target >= 0 && xfer_target < (int)xferTargets.size() &&
           xferTargets[xfer_target].body) {
            sel_body = xferTargets[xfer_target].body;
        }
    
        // A body's sphere-of-influence ring, faint. Skipped when
        // sub-pixel or far off-view (a huge circle is both useless and
        // expensive to tessellate).
        auto draw_soi = [&](const glm::dvec3 &center, double soi_m) {
            if(!map_show_soi || soi_m <= 0.0) { return; }
            const float r_px = (float)(soi_m / map_scale);
            if(r_px < 1.0f || r_px > 4000.0f) { return; }
            map.drawRing(dl, center, soi_m, soi_col, 1.0f);
        };
    
        // The orbits of the bodies orbiting the focus (the ship's
        // parent) -- moons around a planet, planets around the star --
        // in the same view/scale/plane. Each child's state is taken in
        // the focus's inertial frame; its ellipse is sampled through
        // the same cache as the ship's orbit (a child never burns, so
        // its points are propagated once). Drawn first, so the ship's
        // orbit sits on top.
        for(auto *b : planets) {
            if(!(b->frame && b->frame->parent == focus->frame)) continue;
            const double mu_c = b->frame->parent_mu;
            if(mu_c <= 0.0) continue;
            const glm::dvec3 cpos = b->frame->GetPositionRelTo(focus->frame);
            const glm::dvec3 cvel = b->frame->GetVelocityRelTo(focus->frame);
            const std::vector<glm::dvec3> &cpts =
                orbit_caches[(const void *)b].sample(cpos, cvel, mu_c, N);
            if(cpts.empty()) continue;
            const bool selected = (b == sel_body);
            const ImU32 ccol = selected ? col_sel : col_child;
            map.drawOrbit(dl, cpts, ccol, selected ? 2.0f : 1.0f);
            const ImVec2 cpx = map.px(cpos);
            dl->AddCircleFilled(cpx, selected ? 5.0f : 3.0f, ccol);
            if(selected) { dl->AddCircle(cpx, 8.0f, ccol, 0, 1.0f); }
            dl->AddText(ImVec2(cpx.x + 4.0f, cpx.y - 12.0f), ink,
                        b->name.c_str());
            draw_soi(cpos, b->frame->soi);
        }
        // The focus body's own SOI -- the boundary of the current
        // gravitational regime the ship is inside.
        draw_soi(glm::dvec3(0.0, 0.0, 0.0), focus->frame->soi);
    
        // closed=true for the ellipse (it is a closed loop); false for
        // the open arc (a chord would otherwise close it).
        map.drawOrbit(dl, traj_pts, col_ship, 1.0f, closed);
        map.drawBody(dl, ship->m_parent->radius, col_body);
        // The ship: a bright dot (you are here) with a green ring, on
        // the line from the focus.
        const ImVec2 ship_px = map.px(orbit_pos);
        dl->AddLine(focus_px, ship_px, ink, 1.0f);
        dl->AddCircleFilled(ship_px, 5.0f, ink);
        dl->AddCircle(ship_px, 8.0f, col_ship, 0, 1.5f);
        // Prograde (velocity) arrow, along the ship's velocity.
        if(map_show_vel) {
            map.drawArrow(dl, orbit_pos, orbit_vel, 24.0f, col_ship, 1.5f);
        }
        // Apside markers are only meaningful for a non-circular orbit;
        // an open arc has periapsis but no apoapsis.
        if(o.ecc > 1e-3) {
            if(have_peri) { map.drawDot(dl, peri_p, 4.0f, col_apsis); }
            if(have_apo)  { map.drawDot(dl, apo_p,  4.0f, col_apsis); }
        }
    
        // Every other ship in this body: its orbit (when closed) plus
        // a dot + label at its current position, so the whole traffic
        // pattern shows, not just you and the target. The player's own
        // ship is already drawn above in green (skipped here). Ships
        // on an escape trajectory (ecc >= 1) have no closed orbit to
        // draw -- sample() returns empty -- so only their position
        // marker shows.
        {
            Frame *inertial = ship->frame->getNonRotFrame();
            for(auto *s : ships) {
                if(s == ship || !s->frame || s->frame->body != focus) {
                    continue;
                }
                Frame *tsf = s->frame;
                const glm::dvec3 tcom = s->get_center_of_mass();
                const glm::dmat3 O = tsf->GetOrientRelTo(inertial);
                const glm::dvec3 r2 = O * tcom + tsf->GetPositionRelTo(inertial);
                const glm::dvec3 v2 = O * (s->GetVel()
                                          + tsf->GetStasisVelocity(tcom))
                                    + tsf->GetVelocityRelTo(inertial);
                const std::vector<glm::dvec3> &tpts =
                    orbit_caches[(const void *)s].sample(
                        r2, v2, mu, N, s->onRails);
                if(!tpts.empty()) {
                    map.drawOrbit(dl, tpts, col_vessel, 1.0f);
                }
                const ImVec2 tpx = map.px(r2);
                dl->AddCircleFilled(tpx, 3.0f, col_vessel);
                dl->AddText(ImVec2(tpx.x + 4.0f, tpx.y - 11.0f),
                            col_vessel, s->name.c_str());
            }
        }
    
        // P3: the transfer conic to the selected target (planner has a
        // valid solution). It is a Kepler orbit under the focus's mu,
        // starting at the ship (r1 = orbit_pos) with velocity
        // sol.v_departure and propagated over sol.tof -- the same
        // frame as the rest of the map, so it projects through the
        // same plane. The arc's end is the arrival / intercept point
        // (where the ship meets the target at t + tof); the departure
        // point is the ship dot already drawn above.
        if(xfer.valid) {
            const TransferSolution &sol = xfer.sol;
            std::vector<glm::dvec3> xfer_pts;
            xfer_pts.reserve(N + 1);
            for(int i = 0; i <= N; i++) {
                glm::dvec3 p, v;
                propagateKepler(orbit_pos, sol.v_departure, mu,
                                sol.tof * i / N, p, v);
                xfer_pts.push_back(p);
            }
            map.drawOrbit(dl, xfer_pts, col_xfer, 1.5f, /*closed=*/false);
            const glm::dvec3 &arrival = xfer_pts.back();
            map.drawDot(dl, arrival, 4.0f, col_xfer);
            char xfer_label[96];
            snprintf(xfer_label, sizeof(xfer_label), "%s  %.0f m/s",
                     xferTargets[xfer_target].name, sol.total_dv);
            const ImVec2 apx = map.px(arrival);
            dl->AddText(ImVec2(apx.x + 5.0f, apx.y + 4.0f), col_xfer,
                        xfer_label);
        }
    
        // (The invisible map-nav button above already reserved the map
        // square, so the controls land below it.)
        // Bare-map (mode 1) and chrome-less (mode 2) draws only the
        // map -- the controls below are skipped in both.
        if(map_mode != 0) {
            return;
        }
        // Which plane to project onto (see the plane_n selection above).
        // "Orbital" aligns the view with the ship's orbit, so a polar
        // orbit reads as a full ellipse instead of collapsing to a line.
        static const char *kPlanes[] = { "Equatorial", "Ecliptic", "Orbital" };
        ImGui::Combo("Map plane", &map_plane, kPlanes, 3);
        // The map spans ~8 orders of magnitude (a ~70 km low orbit up
        // to a ~90,000 Mm interplanetary orbit), so the scale is edited
        // on a log10 axis -- a linear slider couldn't reach the moons.
        // Wheel-zoom / drag-pan (above) edit the same values; this is
        // the coarse control, and "Reset view" restores the default.
        {
            float log_scale = log10f(map_scale);
            if(ImGui::SliderFloat("Scale", &log_scale, 3.0f, 9.5f, "%.1f")) {
                map_scale = powf(10.0f, log_scale);
            }
            ImGui::SameLine();
            ImGui::Text("%.0f m/px", (double)map_scale);
            ImGui::SameLine();
            if(ImGui::Button("Reset view")) {
                map_pan = ImVec2(0.0f, 0.0f);
                map_scale = 6000.0f;
            }
        }
        ImGui::Checkbox("SOI rings", &map_show_soi);
        ImGui::SameLine();
        ImGui::Checkbox("Velocity", &map_show_vel);
        ImGui::Text("nu %.2f   E %.2f   inc %.2f deg",
                    o.true_anomaly, o.ecc_anomaly,
                    o.inclination * 180.0 / M_PI);
        // Legend: a compact color key (one line).
        ImGui::Spacing();
        auto legend = [&](const char *label, ImU32 col, bool dot) {
            const ImVec2 p = ImGui::GetCursorScreenPos();
            const float s = 10.0f;
            if(dot) {
                dl->AddCircleFilled(ImVec2(p.x + 5.0f, p.y + 8.0f), 4.0f, col);
            } else {
                dl->AddRectFilled(ImVec2(p.x, p.y + 3.0f),
                                 ImVec2(p.x + s, p.y + 13.0f), col);
            }
            ImGui::Dummy(ImVec2(s, 16.0f));
            ImGui::SameLine();
            ImGui::TextUnformatted(label);
        };
        legend("your orbit", col_ship, false);
        ImGui::SameLine();
        legend("other ships", col_vessel, false);
        ImGui::SameLine();
        legend("transfer", col_xfer, false);
        ImGui::SameLine();
        legend("other bodies", col_child, false);
        ImGui::SameLine();
        legend("apsides", col_apsis, true);
    });
}

void drawToasts(Game &g) {
    const double now = SDL_GetTicks() * 0.001;
    std::vector<const ToastMsg *> live;
    for(const ToastMsg &t : g.toasts) {
        if(now - t.born < kToastLife) { live.push_back(&t); }
    }
    if(live.empty()) { return; }
    if(live.size() > (size_t)kToastVisible) {
        live.erase(live.begin(), live.end() - kToastVisible);
    }

    const ImGuiViewport *vp = ImGui::GetMainViewport();
    const float cx = vp->WorkPos.x + vp->WorkSize.x * 0.5f;
    const float cy = vp->WorkPos.y + vp->WorkSize.y * 0.5f;

    ImGui::PushFont(g.bigger);
    const float line_h = ImGui::GetTextLineHeight();
    const float step = line_h * 1.3f;   // line height + a breath of spacing
    ImDrawList *dl = ImGui::GetForegroundDrawList();
    const float shadow = g.ui_scale;   // DPI-aware shadow offset
    const int n = (int)live.size();
    for(int i = 0; i < n; i++) {
        const ToastMsg &t = *live[i];
        const double age = now - t.born;
        // Fade in over 0.25 s, fade out over the last 0.6 s of the life.
        const double a = std::min(1.0, std::min(age / 0.25,
                                                (kToastLife - age) / 0.6));
        const ImVec2 ts = ImGui::CalcTextSize(t.text.c_str());
        // The DISPLAYED block is centered (one line = exactly center);
        // oldest on top, newest at the bottom.
        const float x = cx - ts.x * 0.5f;
        const float y = cy + (i - (n - 1) / 2.0f) * step - ts.y * 0.5f;
        // A soft shadow pass so the text reads over bright terrain too.
        dl->AddText(ImVec2(x + shadow, y + shadow),
                    IM_COL32(0, 0, 0, (int)(180.0 * a)), t.text.c_str());
        dl->AddText(ImVec2(x, y),
                    IM_COL32(255, 255, 255, (int)(255.0 * a)), t.text.c_str());
    }
    ImGui::PopFont();
}

void drawMainMenu(Game &g) {
    bool &running = g.running;

    // Main menu: Esc toggles it. Fixed, so it stays centered and
    // tracks viewport resizes. Drawn last so it sits on top.
    //
    // Every item is a fixed-width button: the window is
    // AlwaysAutoResize, and imgui measures its size from the PREVIOUS
    // frame's content, so a Text item placed by hand (centered against
    // the window width) feeds back into the measurement and the fit
    // converges over several frames on first open. A button's width is
    // explicit and imgui centers its label (ButtonTextAlign), so the
    // layout is settled from the first visible frame.
    ui::Window("Main Menu", g.o_mainmenu, [&] {
        // One width for the whole column: the widest label (the title,
        // in the bigger font, plus its frame padding so the clipped
        // label fits). Every button fills the content width, so the
        // centered labels read as a centered menu.
        ImGui::PushFont(g.bigger);
        const float bw = ImMax(240.0f,
                               ImGui::CalcTextSize("Open Space Program").x
                               + ImGui::GetStyle().FramePadding.x * 2.0f);
        ImGui::PopFont();
        // A button with alpha-0 colors: reads as plain text (no hover
        // highlight either) but keeps the button's stable layout.
        const ImVec4 invisible = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
        auto text_button = [&](const char *label) {
            ImGui::PushStyleColor(ImGuiCol_Button, invisible);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, invisible);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, invisible);
            ImGui::Button(label, ImVec2(bw, 0.0f));
            ImGui::PopStyleColor(3);
        };
        ImGui::PushFont(g.bigger);
        text_button("Open Space Program");
        if(ImGui::Button("Back to game", ImVec2(bw, 0.0f))) {
            ui::SetOpen("Main Menu", false);
        }
        if(ImGui::Button("Toggle windows", ImVec2(bw, 0.0f))) {
            g.toggle_windows();
        }
        if(ImGui::Button("Reset windows", ImVec2(bw, 0.0f))) {
            ui::ResetGui();
        }
        // Toggles (not just open): these two are not in the TAB list, so
        // the menu is another way to close them (besides their X / Back).
        if(ImGui::Button("Settings", ImVec2(bw, 0.0f))) {
            ui::SetOpen("Settings", !ui::IsOpen("Settings"));
        }
        if(ImGui::Button("Controls", ImVec2(bw, 0.0f))) {
            ui::SetOpen("Controls", !ui::IsOpen("Controls"));
        }
        if(ImGui::Button("Quit game", ImVec2(bw, 0.0f))) {
            running = false;
        }
        ImGui::PopFont();
        // The build's git version (src/version.h, `make version`).
        text_button(VERSION);
    });
}
