// gameui.cpp -- the ImGui UI pass (declared in gameui.h).
//
// This was the ImGui section of main's loop: the readout windows (HUD,
// the window list, Settings, TRANSFER, Game Debug Info, ORBITAL,
// TELEMETRY, SURFACE, SHIPS, VESSEL, SHIP PARTS, Controls, Autopilot,
// RESOURCES) and the fixed main menu. They moved out as-is: main's locals
// became Game members (aliased below so the bodies read the same), and
// the per-frame state they show is the ShipView snapshot the 3D pass
// (render.cpp) computes. The orbital map -- the last window, extracted
// separately -- draws between the readouts and the main menu, keeping the
// original window order / z-order.
#include "gameui.h"

#include <cmath>
#include <cstdio>

#include "calendar.h"    // CalTime (the HUD + Game Debug Info clocks)
#include "physics.h"     // GetAngVelocity (the VESSEL window)
#include "siminput.h"    // fmt_time (the TRANSFER window)

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
    OrbitCamera *orbitCam = g.orbitCam;
    FreeCamera *freeCam = g.freeCam;
    int &activeIdx = g.activeIdx;
    int &time_accel = g.time_accel;
    int &cam_speed = g.cam_speed;
    double &time = g.time;
    int &ui_style = g.ui_style;
    float &window_rounding = g.window_rounding;
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
            if(ui_style == 0) ImGui::StyleColorsDark();
            else if(ui_style == 1) ImGui::StyleColorsLight();
            else ImGui::StyleColorsClassic();
        }
        if(ImGui::SliderFloat("Window rounding", &window_rounding,
                             0.0f, 50.0f, "%.0f")) {
            ImGui::GetStyle().WindowRounding = window_rounding;
        }
        if(ImGui::SliderFloat("FOV", &args.camFovDeg, 10.0f, 120.0f, "%.0f°")) {
            const float f = (float)glm::radians(args.camFovDeg);
            orbitCam->setFov(f);
            freeCam->setFov(f);
        }
    });

    // Transfer planner: parent->child body transfers (with capture)
    // and same-body ship intercepts. The solution is computed in the
    // render pass (xfer), so this window is pure readout + inputs.
    ui::Window("TRANSFER", g.o_transfer, [&] {
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
    ui::Window("ORBITAL", g.o_orbit, [&] {
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
    ui::Window("TELEMETRY", g.o_telemetry, [&] {
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
    ui::Window("SURFACE", g.o_surface, [&] {
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

    ui::Window("SHIPS", g.o_ships, [&] {
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

    ui::Window("VESSEL", g.o_vessel, [&] {
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
    ui::Window("SHIP PARTS", g.o_parts, [&] {
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
        ImGui::Text("q/e - roll left/right");
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
    });

    ui::Window("Autopilot", g.o_autopilot, [&] {
        ImGui::Button("Prograde");
        ImGui::Button("Retrograde");
        ImGui::Button("Radial-in");
        ImGui::Button("Radial-out");
        ImGui::Button("Normal");
        ImGui::Button("Anti-normal");
    });

    ui::Window("RESOURCES", g.o_resources, [&] {
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

void drawMainMenu(Game &g) {
    bool &running = g.running;

    // Main menu: Esc toggles it. Fixed, so it stays centered and
    // tracks viewport resizes. Drawn last so it sits on top.
    // Buttons have an explicit width: -1 (full width) inside an
    // auto-resizing window collapses the window to a sliver.
    ui::Window("Main Menu", g.o_mainmenu, [&] {
        ImGui::PushFont(g.bigger);
        if(ImGui::Button("Toggle windows", ImVec2(240.0f, 0.0f))) {
            g.toggle_windows();
        }
        if(ImGui::Button("Reset windows", ImVec2(240.0f, 0.0f))) {
            ui::ResetGui();
        }
        if(ImGui::Button("Settings", ImVec2(240.0f, 0.0f))) {
            ui::SetOpen("Settings", true);
        }
        if(ImGui::Button("Controls", ImVec2(240.0f, 0.0f))) {
            ui::SetOpen("Controls", true);
        }
        if(ImGui::Button("Quit game", ImVec2(240.0f, 0.0f))) {
            running = false;
        }
        ImGui::PopFont();
    });
}
