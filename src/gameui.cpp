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
#include <climits>
#include <cmath>
#include <cstdio>
#include <map>
#include <vector>

#include "calendar.h"    // CalTime (the HUD + Game Debug Info clocks)
#include "version.h"     // VERSION (the main menu)
#include "physics.h"     // GetAngVelocity (the VESSEL window)
#include "siminput.h"    // fmt_time (the TRANSFER window)
#include "orbitsample.h" // OrbitSampleCache + open-arc sampling (the map)
#include "orbitmap.h"    // OrbitMap + contrastingColor (the map)
#include "surfmap.h"     // the lon/lat <-> pixel math + surfmapCompute
#include "texture.h"     // make_texture_r8 (the Porkchop heatmap + Surface Map)

#include "../middleware/imgui/imgui.h"
#include "../middleware/implot/implot.h"   // the TELEMETRY plots

// GLM's gtx extensions (glm::angle in ORBITAL) hard-error without this.
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/vector_angle.hpp>

namespace {
/* Viridis (matplotlib's default scientific colormap), 11 anchor stops
   linearly interpolated: perceptually uniform, colorblind-safe, reads as
   a smooth "cold -> hot" dv scale. t in [0,1] -> 0xAARRGGBB. */
const unsigned char kViridis[11][3] = {
    { 68,   1,  84}, { 72,  40, 120}, { 62,  74, 137}, { 49, 104, 142},
    { 38, 130, 142}, { 33, 145, 140}, { 31, 160, 136}, { 53, 183, 121},
    {110, 206,  88}, {181, 222,  43}, {253, 231,  37}};
unsigned int ramp_color(float t) {
    if(t < 0.0f) { t = 0.0f; }
    if(t > 1.0f) { t = 1.0f; }
    const float f = t * 10.0f;
    const int i = f < 10.0f ? (int)f : 9;
    const float u = f - (float)i;
    const unsigned char r =
        (unsigned char)(kViridis[i][0] * (1.0f - u) + kViridis[i + 1][0] * u);
    const unsigned char g =
        (unsigned char)(kViridis[i][1] * (1.0f - u) + kViridis[i + 1][1] * u);
    const unsigned char b =
        (unsigned char)(kViridis[i][2] * (1.0f - u) + kViridis[i + 1][2] * u);
    return 0xff000000u | (unsigned int)b << 16 | (unsigned int)g << 8 | r;
}
} // namespace

// Cached orbit samplings, one entry per orbiting object (keyed on its
// pointer -- the ship for the ship's orbit, a body for a child's; a given
// ship always has exactly one entry, overwritten on each sample). Reuse
// is only trusted while the orbiting object is on a fixed Keplerian conic:
// the ship passes its onRails flag, terrain bodies are always on theirs.
// See OrbitSampleCache. File scope so the Orbital Map and the Surface Map
// share one cache (the same orbit sampled once, drawn on both).
static std::map<const void *, OrbitSampleCache> orbit_caches;

// Format a sim-clock time (s) on the home body's calendar, the same
// "Year ... Day d/N ... HH:MM:SS" the top bar (HUD) shows, so a planned
// departure time (a porkchop "Send best") can be read off against it.
// Empty when there is no home calendar or t < 0.
static std::string fmt_cal_time(const Calendar &cal, double t) {
    if(!cal.valid() || t < 0.0) { return std::string(); }
    const CalTime ct = cal.at(t);
    char line[64];
    if(ct.has_year) {
        // CalTime only exposes month + day-of-month, so the day-of-year is
        // day + the days in the earlier months.
        int doy = ct.day;
        for(int m = 0; m < ct.month - 1; m++) { doy += cal.month_days[m]; }
        snprintf(line, sizeof(line), "Year %04d   Day %d/%d   %02d:%02d:%02d",
                 ct.year, doy, cal.days_per_year, ct.hh, ct.mm, ct.ss);
    } else {
        snprintf(line, sizeof(line), "Day %d   %02d:%02d:%02d",
                 ct.day, ct.hh, ct.mm, ct.ss);
    }
    return line;
}

// --- Telemetry window: a 2x2 grid of plots, each with a dropdown to pick
// which time series to show. The series are the two conserved 2-body
// constants (per-frame, sim-time x-axis) and the five per-frame timings
// (main.cpp pushes them every frame; wall-clock x-axis). ---
struct TeleSeriesDef { const char *name; const char *yaxis; };
static const TeleSeriesDef kSeries[7] = {
    {"specific orbital energy", "J/kg"},
    {"angular momentum", "m^2/s"},
    {"frame: events", "ms"},
    {"frame: logic", "ms"},
    {"frame: jobs", "ms"},
    {"frame: render", "ms"},
    {"frame: present", "ms"},
};
static const int kNumSeries = 7;

// Resolve a series index (0..6) to its ring buffer. 0-1 live on the active
// ship's view (render.cpp samples them); 2-6 are the per-frame timings on
// Game (main.cpp samples them).
static TimeSeries *telemetry_series(Game &g, int idx) {
    switch(idx) {
        case 0: return &g.view.energy_series;
        case 1: return &g.view.angmom_series;
        case 2: return &g.perf_events;
        case 3: return &g.perf_logic;
        case 4: return &g.perf_jobs;
        case 5: return &g.perf_render;
        case 6: return &g.perf_present;
        default: return nullptr;
    }
}

// One grid cell: a full-width dropdown to pick the series, then the plot
// filling the rest of the cell.
static void draw_telemetry_cell(Game &g, int idx) {
    const char *items[kNumSeries];
    for(int i = 0; i < kNumSeries; i++) { items[i] = kSeries[i].name; }
    ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
    ImGui::Combo("##series", &g.telemetry_sel[idx], items, kNumSeries);
    ImGui::PopItemWidth();

    const int sel = g.telemetry_sel[idx];
    if(sel < 0 || sel >= kNumSeries) { return; }
    TimeSeries *s = telemetry_series(g, sel);
    if(s == nullptr || s->count < 2) {
        ImGui::TextDisabled("(no data)");
        return;
    }
    const int n = s->stage();
    ImPlot::SetNextAxesToFit();
    if(ImPlot::BeginPlot(kSeries[sel].name)) {
        ImPlot::SetupAxis(ImAxis_X1, "t (s)");
        ImPlot::SetupAxis(ImAxis_Y1, kSeries[sel].yaxis);
        ImPlot::PlotLine(kSeries[sel].name, s->t_arr(), s->v_arr(), n);
        ImPlot::EndPlot();
    }
}

void drawUIReadouts(Game &g, TransferPlanner &planner) {
    // The window bodies are verbatim from main's ImGui pass; their locals
    // are Game members (aliased so the bodies read the same).
    Vehicle *ship = g.ship;
    Ships &ships = g.ships;
    System &sys = g.sys;
    GameArgs &args = g.args;
    Camera *camera = g.camera;
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
    // The Settings window writes these; tick.cpp reads (inverts a manual
    // attitude axis away from the baked-in default).
    bool &flip_pitch = g.flip_pitch;
    bool &flip_yaw = g.flip_yaw;
    bool &flip_roll = g.flip_roll;

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
    auto &pc = planner.pc;   // the porkchop grid (Porkchop window)

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
        if(sys.home) {
            const std::string line = fmt_cal_time(sys.home->cal, time);
            if(!line.empty()) {
                ImGui::SetCursorPosX((ImGui::GetWindowWidth()
                    - ImGui::CalcTextSize(line.c_str()).x) * 0.5f);
                ImGui::TextUnformatted(line.c_str());
            }
        }
    });

    /* Window list (single source of truth: the ui_windows table; entries
       with in_windows_list=false, e.g. the Porkchop, are toggled from
       their parent window instead) plus the Top-HUD group switch. */
    ui::Window("Windows", g.o_menu, [&] {
        ImGui::Spacing();
        for(auto &w : g.ui_windows) {
            if(!w.in_windows_list) { continue; }
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
        // Display: the window mode + the resolution it runs at. Both
        // apply immediately (Renderer::setWindowMode; the SIZE_CHANGED
        // event in events.cpp finishes the resize: the viewport, postfx,
        // the camera aspect). "fullscreen" runs at the display's native
        // mode, so the resolution is off there (it applies to the other
        // three modes).
        {
            static const char *const mode_names[] =
                {"windowed", "borderless", "fullscreen", "exclusive"};
            int wm = (int)args.window_mode;
            if(ImGui::Combo("Window mode", &wm,
                            "windowed\0borderless\0fullscreen\0exclusive\0")) {
                args.window_mode = static_cast<WindowMode>(wm);
                g.display.setWindowMode(args.window_mode,
                                        args.screen_width, args.screen_height);
                g.toast("Window mode: %s", mode_names[wm]);
            }
            // Resolution: the display's supported modes -- one entry per
            // width x height x refresh rate -- (+ the current one). The
            // selection is an exact WxH match (among those, the refresh
            // closest to the display's current one), falling back to the
            // nearest WxH (the WM may have clamped it out of the list).
            const std::vector<Resolution> modes = g.display.displayModes();
            if(!modes.empty()) {
                const int cur_refresh = g.display.currentRefresh();
                int sel_exact = -1, best_exact = INT_MAX;
                int sel_any = 0, best_any = INT_MAX;
                for(size_t i = 0; i < modes.size(); i++) {
                    const int size_d =
                        std::abs(modes[i].width - args.screen_width)
                        + std::abs(modes[i].height - args.screen_height);
                    if(size_d == 0) {
                        const int rr =
                            std::abs(modes[i].refresh - cur_refresh);
                        if(rr < best_exact) { best_exact = rr; sel_exact = (int)i; }
                    } else if(size_d < best_any) {
                        best_any = size_d;
                        sel_any = (int)i;
                    }
                }
                int sel = (sel_exact >= 0) ? sel_exact : sel_any;
                std::string items;
                char buf[48];
                for(size_t i = 0; i < modes.size(); i++) {
                    // Same WxH at several refresh rates: the Hz is what
                    // tells the entries apart (0 = unknown, omit it).
                    snprintf(buf, sizeof(buf),
                             modes[i].refresh > 0 ? "%dx%d @ %dHz" : "%dx%d",
                             modes[i].width, modes[i].height,
                             modes[i].refresh);
                    items += buf;
                    items += '\0';
                }
                items += '\0';
                const bool native_fs =
                    (args.window_mode == WindowMode::Fullscreen);
                ImGui::BeginDisabled(native_fs);
                if(ImGui::Combo("Display mode", &sel, items.c_str())) {
                    args.screen_width = modes[sel].width;
                    args.screen_height = modes[sel].height;
                    if(!native_fs) {
                        g.display.setWindowMode(args.window_mode,
                                                args.screen_width,
                                                args.screen_height);
                    }
                }
                ImGui::EndDisabled();
            }
            if(args.window_mode == WindowMode::Fullscreen) {
                ImGui::TextDisabled("(fullscreen: the display's native "
                                    "resolution)");
            }
            ImGui::Separator();
        }
        ImGui::Checkbox("Physics debug draw", &physics_debug_drawing);
        ImGui::Checkbox("World draw", &world_drawing);
        ImGui::Checkbox("Starfield", &draw_starfield);
        ImGui::Checkbox("Reference circles", &draw_skylines);
        // Post-processing: one checkbox per effect (the passes run in this
        // order, i.e. PostFX::Available() order); gamma also gets a
        // strength slider. A toggle takes effect from the next frame.
        ImGui::Separator();
        ImGui::Text("Post-processing");
        for(const std::string &fx : PostFX::Available()) {
            const char *label = fx.c_str();
            if(fx == "crt") label = "CRT (retro tube)";
            else if(fx == "grain") label = "Film grain";
            else if(fx == "cas") label = "CAS sharpen";
            else if(fx == "gamma") label = "Gamma";
            bool on = g.postfx->IsEnabled(fx);
            if(ImGui::Checkbox(label, &on)) {
                g.postfx->SetEnabled(fx, on);
            }
            if(fx == "gamma" && on) {
                float gamma = g.postfx->GetParam("gamma");
                ImGui::Indent();
                if(ImGui::SliderFloat("gamma (1.0 = neutral)", &gamma,
                                      0.25f, 3.0f, "%.2f")) {
                    g.postfx->SetParam("gamma", gamma);
                }
                ImGui::Unindent();
            }
        }
        if(ImGui::Combo("UI style", &ui_style, "Dark\0Light\0Classic\0")) {
            g.apply_ui_style();
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
        // Controls: invert a manual attitude axis away from the default.
        // The default baseline already bakes in the preferred orientation
        // (viewed from the front, yaw + roll are pre-flipped to respond in
        // your screen direction; pitch is not mirrored), so all three are
        // off by default. tick.cpp applies the sign each tick.
        ImGui::Separator();
        ImGui::Text("Controls (W/S pitch, A/D yaw, Q/E roll)");
        ImGui::Checkbox("Flip pitch (W/S)", &flip_pitch);
        ImGui::Checkbox("Flip yaw (A/D)", &flip_yaw);
        ImGui::Checkbox("Flip roll (Q/E)", &flip_roll);
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
        // The Porkchop window (the launch-window heatmap) hangs off this
        // window rather than the Windows list: pick a target, then open
        // the plot for it.
        bool pc_open = ui::IsOpen("Porkchop");
        if(ImGui::Checkbox("Porkchop", &pc_open)) {
            ui::SetOpen("Porkchop", pc_open);
        }
        if(xfer_target < 0) {
            ImGui::Text("Select a target body or ship.");
            return;
        }
        const bool isShip = xferTargets[xfer_target].ship != nullptr;
        if(isShip) {
            ImGui::TextDisabled("ship target: intercept only, no capture burn");
        }
        // A porkchop "Send best" plan: count down to the departure instant.
        // At zero the live "depart now" solution below IS the best cell, so
        // that is when you burn. The countdown is plain seconds; the
        // departure time itself is on the home calendar (matching the top
        // bar). "Clear plan" drops the plan and restores the ToF mode the
        // user had before sending.
        if(planner.xfer_from_porkchop && planner.xfer_t_dep > 0.0) {
            ImGui::Separator();
            const double tleft = planner.xfer_t_dep - time;
            if(tleft > 0.0) {
                ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f),
                                   "departure in: %.0f s", tleft);
            } else {
                ImGui::TextColored(ImVec4(0.45f, 1.0f, 0.45f, 1.0f),
                                   "DEPARTURE NOW -- burn");
            }
            if(sys.home) {
                const std::string dt = fmt_cal_time(sys.home->cal,
                                                    planner.xfer_t_dep);
                if(!dt.empty()) {
                    ImGui::Text("departure:    %s", dt.c_str());
                }
            }
            if(ImGui::Button("Clear plan")) {
                planner.clearPorkchopPlan();
            }
            ImGui::Separator();
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

    /* Porkchop plot: the 2-D launch-window map (total dv over departure
       delay x time of flight). The grid is computed on demand -- the button
       or the P key -- and cached until the next compute (the MechJeb model),
       so the window is cheap to leave open. */
    ui::Window("Porkchop", g.o_porkchop, [&] {
        if(xferTargets.empty()) {
            ImGui::Text("No transfer targets: no child bodies or ships here.");
            return;
        }
        if(xfer_target < 0) {
            ImGui::Text("Select a target body or ship (Transfer window).");
            return;
        }
        const char *tn = xferTargets[xfer_target].name;

        // On-demand compute (same trigger as the P key). The grid sweep
        // runs on the background worker (g.jobs), so this is a one-shot
        // button, not a per-frame re-sweep, and it never stalls the frame.
        // While a sweep is in flight the button is disabled and the last
        // grid stays on screen (the new one replaces it when the job lands).
        const bool pc_busy = planner.pc_in_flight > 0;
        if(pc_busy) { ImGui::BeginDisabled(); }
        if(ImGui::Button("Compute  (P)")) {
            planner.porkchopCompute();
        }
        if(pc_busy) { ImGui::EndDisabled(); }
        ImGui::SameLine();
        ImGui::Text("target: %s", tn);
        if(pc_busy) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.4f, 1.0f),
                               "   sweeping ...");
        }
        ImGui::TextDisabled("grid %d x %d   (size: --porkchop-n)",
                            g.args.porkchop_n, g.args.porkchop_n);
        if(pc_busy) {
            ImGui::TextDisabled("(the last grid stays shown until the new one "
                                "lands)");
        }

        // Axis ranges. Off = the auto range (departure: 0 .. one target
        // period, covers every relative phase; ToF: 60 s .. three target
        // periods). On = the two sliders, in seconds; press Compute (or P)
        // to re-sweep the grid over them.
        const float kRangeSliderMax = 604800.0f; // 7 days
        if(ImGui::Checkbox("Departure range", &planner.pcCustomDep)
           && planner.pcCustomDep) {
            // Just enabled: seed from the last sweep's range (or 0 .. 1 day
            // if there isn't one yet).
            planner.pcDepLo = pc.valid ? (float)pc.t_dep_lo : 0.0f;
            planner.pcDepHi = pc.valid
                ? (pc.t_dep_hi < kRangeSliderMax ? (float)pc.t_dep_hi
                                                 : kRangeSliderMax)
                : 86400.0f;
        }
        if(planner.pcCustomDep) {
            ImGui::SliderFloat("start (dep)", &planner.pcDepLo, 0.0f,
                               kRangeSliderMax, "%.0f s");
            ImGui::SliderFloat("end (dep)", &planner.pcDepHi, 0.0f,
                               kRangeSliderMax, "%.0f s");
        }
        if(ImGui::Checkbox("ToF range", &planner.pcCustomTof)
           && planner.pcCustomTof) {
            // Just enabled: seed from the last sweep's range (or 60 s ..
            // 1 day if there isn't one yet).
            planner.pcTofLo = pc.valid ? (float)pc.tof_lo : 60.0f;
            planner.pcTofHi = pc.valid
                ? (pc.tof_hi < kRangeSliderMax ? (float)pc.tof_hi
                                               : kRangeSliderMax)
                : 86400.0f;
        }
        if(planner.pcCustomTof) {
            ImGui::SliderFloat("start (ToF)", &planner.pcTofLo, 60.0f,
                               kRangeSliderMax, "%.0f s");
            ImGui::SliderFloat("end (ToF)", &planner.pcTofHi, 60.0f,
                               kRangeSliderMax, "%.0f s");
        }
        if(planner.pcCustomDep || planner.pcCustomTof) {
            ImGui::TextDisabled("(press Compute / P to re-sweep)");
        }

        if(!pc.valid) {
            if(pc_busy) {
                // A sweep is in flight (the grid isn't on screen yet): show
                // the working state instead of the "no window" message.
                ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.4f, 1.0f),
                                   "Sweeping the grid ...");
                ImGui::TextDisabled("The result appears here when it lands.");
            } else {
                ImGui::Text("No launch window in the swept range for this target.");
                ImGui::TextDisabled("Press Compute (or P) to sweep the grid.");
            }
            return;
        }

        // The best cell (argmin over the valid cells).
        ImGui::Text("min dv:      %08.1f m/s", pc.dv_min);
        ImGui::Text("depart in:   %s", fmt_time(pc.t_dep_min).c_str());
        ImGui::Text("time of flt: %s", fmt_time(pc.tof_min).c_str());
        // Apply the best cell to the Transfer planner: pin the ToF (manual
        // mode) and record the ABSOLUTE departure time (compute moment + the
        // best cell's delay). The Transfer window then counts down to it, and
        // at that instant the live "depart now" solution IS the best cell
        // (Kepler propagation composes), so you burn then. Disabled while a
        // new sweep is in flight: pc would still be the PREVIOUS grid, and a
        // plan from it is stale (wait for the sweep to land).
        if(pc_busy) { ImGui::BeginDisabled(); }
        if(ImGui::Button("Send best to Transfer")) {
            // Remember the user's ToF mode so "Clear plan" restores it.
            planner.xfer_prev_auto = planner.xfer_auto;
            planner.xfer_prev_tof_log = planner.xfer_tof_log;
            planner.xfer_auto = false;
            planner.xfer_tof_log = (float)std::log10(pc.tof_min);
            planner.xfer_t_dep = planner.pc_computed_at + pc.t_dep_min;
            planner.xfer_plan_target = xfer_target;
            planner.xfer_from_porkchop = true;
            ui::SetOpen("Transfer", true);
        }
        if(pc_busy) { ImGui::EndDisabled(); }

        // The heatmap: total dv over (departure delay x, time of flight y).
        // Storage is ToF-major (rows = ToF, cols = departure). Drawn as a
        // texture (not ImPlot::PlotHeatmap): that one indexes its color
        // LUT with the raw cell value, so the no-solution (NaN) cells in
        // any launch-window map read out of bounds and assert. NaN cells
        // are a distinct gray here.
        // Color scale: [dv_min, dv_hi]. dv_hi is the robust max (95th
        // percentile) from the grid -- the absolute max is deliberately
        // excluded, because the dv surface has a narrow unphysical spike at
        // the shortest ToFs (hundreds of km/s, < 1% of cells) that would
        // stretch the scale and compress the whole launch window to purple.
        double lo = pc.dv_min;
        double hi = (pc.dv_hi > lo) ? pc.dv_hi : lo;
        const int w = pc.n_dep, h = pc.n_tof;
        std::vector<unsigned char> px((size_t)w * h * 4);
        for(int j = 0; j < h; j++) {
            for(int i = 0; i < w; i++) {
                const double dv = pc.total_dv[(size_t)j * w + i];
                unsigned char *p = &px[((size_t)j * w + i) * 4];
                if(std::isnan(dv)) {
                    p[0] = p[1] = p[2] = 80;   // gray = no solution
                } else {
                    const float t = (float)((dv - lo) / (hi - lo));
                    const unsigned int c = ramp_color(t);
                    p[0] = (unsigned char)(c & 0xff);
                    p[1] = (unsigned char)((c >> 8) & 0xff);
                    p[2] = (unsigned char)((c >> 16) & 0xff);
                }
                p[3] = 255;
            }
        }
        // One texture, re-uploaded on each compute (and when --porkchop-n
        // changes the size). 40 x 40 x 4 B is trivial.
        static Texture *pc_tex = nullptr;
        static int tex_w = 0, tex_h = 0;
        if(!pc_tex || tex_w != w || tex_h != h) {
            if(pc_tex) { delete pc_tex; }
            pc_tex = make_texture_r8(w, h, px.data());
            tex_w = w;
            tex_h = h;
        } else {
            upload_texture_r8(pc_tex, w, h, px.data());
        }
        // The color bar: a 1 x 64 viridis strip, lo at the bottom.
        static Texture *bar_tex = nullptr;
        if(!bar_tex) {
            unsigned char bar[64 * 4];
            for(int i = 0; i < 64; i++) {
                const unsigned int c = ramp_color((float)i / 63.0f);
                bar[i * 4 + 0] = (unsigned char)(c & 0xff);
                bar[i * 4 + 1] = (unsigned char)((c >> 8) & 0xff);
                bar[i * 4 + 2] = (unsigned char)((c >> 16) & 0xff);
                bar[i * 4 + 3] = 255;
            }
            bar_tex = make_texture_r8(1, 64, bar);
        }
        // Fixed display size (the window auto-fits around it). The grid
        // resolution (pc_n) only changes how many cells map onto this size.
        const float img_sz = 420.0f;
        ImGui::Image((ImTextureID)(std::intptr_t)pc_tex->id,
                     ImVec2(img_sz, img_sz), ImVec2(0, 1), ImVec2(1, 0));
        ImGui::SameLine();
        // Colorbar: hi (max dv) at the top, lo (min dv) at the bottom, with
        // the value labels at each end (aligned to the bar, not the heatmap).
        ImGui::BeginGroup();
            ImGui::Text("%.0f", hi);
            ImGui::Image((ImTextureID)(std::intptr_t)bar_tex->id,
                         ImVec2(16.0f,
                                std::max(20.0f, img_sz -
                                         ImGui::GetTextLineHeight() * 2.0f)),
                         ImVec2(0, 1), ImVec2(1, 0));
            ImGui::Text("%.0f", lo);
        ImGui::EndGroup();
        ImGui::TextDisabled("x: departure delay  %.0f .. %.0f s (min %s)",
                            pc.t_dep_lo, pc.t_dep_hi,
                            fmt_time(pc.t_dep_min).c_str());
        ImGui::TextDisabled("y: time of flight   %.0f .. %.0f s (min %s)",
                            pc.tof_lo, pc.tof_hi, fmt_time(pc.tof_min).c_str());
        ImGui::TextDisabled("bar: dv in m/s (top = max)   gray: no solution");
    });

    /* Surface Map: the chosen body's surface as an equirectangular 2-D
       map (north up, lon 0 at the left edge), the ship's position +
       orbit overlaid, and -- optionally -- the terminator (day/night)
       baked in. The pixel buffer is computed on demand -- the button or
       the M key -- on the background worker, and cached until the next
       compute (the same pattern as the Porkchop), so the window is
       cheap to leave open and a sweep never stalls the frame. */
    ui::Window("Surface Map", g.o_surfmap, [&] {
        // Body to map: item 0 = "active ship's body" (surfmap_body =
        // nullptr, so the map follows the ship's SOI); the rest are
        // sys.bodies in order (the star maps itself, fully lit).
        std::vector<std::string> sm_names;
        sm_names.push_back(ship && ship->m_parent
                              ? "active ship's body (" + ship->m_parent->name + ")"
                              : "active ship's body");
        int sm_sel = 0;
        for(auto *b : sys.bodies) {
            sm_names.push_back(b->name);
            if(g.surfmap_body == b) { sm_sel = (int)sm_names.size() - 1; }
        }
        std::vector<const char *> sm_items;
        for(auto &n : sm_names) { sm_items.push_back(n.c_str()); }
        ImGui::Combo("Body", &sm_sel, sm_items.data(), (int)sm_items.size());
        g.surfmap_body = (sm_sel > 0 && sm_sel <= (int)sys.bodies.size())
            ? sys.bodies[sm_sel - 1]
            : nullptr;

        TerrainBody *sm_body = g.surfmap_body
            ? g.surfmap_body
            : (ship && ship->m_parent ? ship->m_parent : sys.home);
        if(sm_body == nullptr) {
            ImGui::Text("No body to map (no ship, no system bodies).");
            return;
        }

        // Auto-compute when there is no map yet, or it was computed for a
        // different body (the combo pick / the ship's SOI changed). Not
        // while a sweep is in flight: the last one lands for the body it
        // was posted for, and this frame re-requests if it is still stale.
        if(g.surfmap_in_flight == 0 &&
           (!g.surfmap_valid || g.surfmap_body_name != sm_body->name)) {
            surfmapCompute(g);
        }

        // The sweep runs on the background worker (g.jobs), like the
        // Porkchop grid: while one is in flight the buttons wait and the
        // last map stays on screen (the new one replaces it when it lands).
        // Read AFTER the auto-compute above so a just-posted job (first
        // open / a body switch) already shows the "mapping ..." state.
        const bool sm_busy = g.surfmap_in_flight > 0;

        if(sm_busy) { ImGui::BeginDisabled(); }
        if(ImGui::Button("Refresh  (M)")) {
            surfmapCompute(g);
        }
        if(sm_busy) { ImGui::EndDisabled(); }
        if(sm_busy) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.4f, 1.0f),
                               "   mapping ...");
        }
        bool shade = g.surfmap_shade;
        if(sm_busy) { ImGui::BeginDisabled(); }
        if(ImGui::Checkbox("Sun shading", &shade)) {
            g.surfmap_shade = shade;
            surfmapCompute(g);   // re-bake with the new terminator
        }
        if(sm_busy) { ImGui::EndDisabled(); }
        ImGui::TextDisabled("map %dx%d   (size: --surfmap-n)",
                            g.surfmap_w, g.surfmap_h);
        if(sm_busy) {
            ImGui::TextDisabled("(the last map stays shown until the new one "
                                "lands)");
        }

        if(!g.surfmap_valid || g.surfmap_px.empty()) {
            if(sm_busy) {
                ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.4f, 1.0f),
                                   "Mapping the surface ...");
                ImGui::TextDisabled("The map appears here when it lands.");
            } else {
                ImGui::Text("No map yet: press Refresh (or M).");
            }
            return;
        }

        // One texture, re-uploaded when the map is recomputed (or the
        // size changes). LINEAR filtering: a smooth map upscaled over the
        // window (unlike the Porkchop's discrete heatmap cells).
        static Texture *sm_tex = nullptr;
        static int sm_tex_w = 0, sm_tex_h = 0, sm_tex_rev = -1;
        if(g.surfmap_rev != sm_tex_rev) {
            if(!sm_tex || sm_tex_w != g.surfmap_w || sm_tex_h != g.surfmap_h) {
                if(sm_tex) { delete sm_tex; }
                sm_tex = make_texture_r8(g.surfmap_w, g.surfmap_h,
                                        g.surfmap_px.data(), /*linear=*/true);
                sm_tex_w = g.surfmap_w;
                sm_tex_h = g.surfmap_h;
            } else {
                upload_texture_r8(sm_tex, g.surfmap_w, g.surfmap_h,
                                  g.surfmap_px.data());
            }
            sm_tex_rev = g.surfmap_rev;
        }

        // Fixed display size (2:1); the map resolution only sets how many
        // texels land on this area. The buffer's row 0 is the north pole,
        // and GL row 0 is uv (0,0) = the drawn rect's top-left, so the
        // default (0,0)-(1,1) uv draws it unflipped (the Porkchop heatmap
        // flips, its row 0 being the axis minimum).
        const float sm_img_w = 480.0f;
        const float sm_img_h = 240.0f;
        const ImVec2 sm_p0 = ImGui::GetCursorScreenPos();
        ImGui::Image((ImTextureID)(std::intptr_t)sm_tex->id,
                     ImVec2(sm_img_w, sm_img_h));

        // The overlay (graticule, orbit, apsides, ship dot) on the map
        // rect. (lon, lat) -> pixels: lon 0..2pi left -> right, lat +pi/2
        // (north, buffer row 0) top -> -pi/2 bottom.
        ImDrawList *dl = ImGui::GetWindowDrawList();
        const ImVec4 sm_bg = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
        const ImU32 sm_ink = contrastingColor(sm_bg);
        const ImU32 sm_ship =
            ImGui::GetColorU32(ImVec4(0.20f, 0.80f, 0.40f, 1.0f));
        auto map_px = [&](double lon, double lat) {
            const double u = lon / (2.0 * M_PI) * (double)sm_img_w;
            const double v = (M_PI * 0.5 - lat) / M_PI * (double)sm_img_h;
            return ImVec2(sm_p0.x + (float)u, sm_p0.y + (float)v);
        };

        // Graticule: lat -60..60 every 30 (equator brighter), lon every
        // 45 -- faint, under the orbit line.
        const ImU32 grat_faint =
            ImGui::GetColorU32(ImVec4(1.0f, 1.0f, 1.0f, 0.12f));
        const ImU32 grat_eq =
            ImGui::GetColorU32(ImVec4(1.0f, 1.0f, 1.0f, 0.25f));
        for(int deg = -60; deg <= 60; deg += 30) {
            const ImVec2 a = map_px(0.0, (double)deg * M_PI / 180.0);
            const ImVec2 b = map_px(2.0 * M_PI, (double)deg * M_PI / 180.0);
            dl->AddLine(a, b, deg == 0 ? grat_eq : grat_faint, 1.0f);
        }
        for(int deg = 0; deg < 360; deg += 45) {
            const double lon = (double)deg * M_PI / 180.0;
            const ImVec2 a = map_px(lon, M_PI * 0.5);
            const ImVec2 b = map_px(lon, -M_PI * 0.5);
            dl->AddLine(a, b, grat_faint, 1.0f);
        }

        // The ship's orbit around the mapped body -- only when the ship is
        // orbiting it (a conic about a different body has no meaning here;
        // the caption below notes it).
        if(ship && sm_body == ship->m_parent && g.view.mu > 0.0) {
            const double &mu = g.view.mu;
            const glm::dvec3 &orbit_pos = g.view.orbit_pos;
            const glm::dvec3 &orbit_vel = g.view.orbit_vel;
            const int N = 128;
            const bool closed = (o.ecc < 1.0);
            std::vector<glm::dvec3> pts;
            if(closed) {
                // Per-ship cache (shared with the Orbital Map), trusted
                // only while the ship coasts on its Keplerian conic.
                pts = orbit_caches[(const void *)ship].sample(
                    orbit_pos, orbit_vel, mu, N, ship->onRails);
            } else {
                // Open arc: the map spans the whole body, so cap the arc
                // a couple of ship radii beyond the current radius.
                const double r_cap =
                    std::max(4.0 * o.periapsis, o.distance) * 2.0;
                pts = sampleOpenTrajectory(orbit_pos, orbit_vel, mu, N,
                                           r_cap);
            }
            if(!pts.empty()) {
                // The points live in the ship's non-rotating frame (the
                // mapped body's inertial frame, sm_body == m_parent above);
                // the map's pixel directions live in the body's ROTATING
                // frame (the surface's own frame). Rigid-transform each
                // point into that frame -- the orbit's GROUND TRACK over
                // the surface. An orbit inclined to the spin axis (Kerbin's
                // is tilted ~23 deg) precesses as the planet spins; that
                // sweep is the ship's true path over the rotating surface,
                // which is what a surface map for landing shows.
                Frame *inertial = ship->frame->getNonRotFrame();
                Frame *rot = sm_body->frame->getRotFrame();
                const glm::dmat3 O = inertial->GetOrientRelTo(rot);
                const glm::dvec3 P = inertial->GetPositionRelTo(rot);
                // Point (inertial frame) -> (lon, lat, pixel). out = false
                // if degenerate.
                auto to_px = [&](const glm::dvec3 &p, ImVec2 &px) -> bool {
                    const glm::dvec3 pr = O * p + P;
                    const double l = glm::length(pr);
                    if(l < 1e-9) { return false; }
                    double lon, lat;
                    surfmapLonLat(pr / l, lon, lat);
                    px = map_px(lon, lat);
                    return true;
                };
                // The polyline, broken at the antimeridian (the map's
                // left and right edges are the SAME meridian; surfmapWraps
                // detects the >half-turn jump between consecutive lons).
                double prev_lon = -1.0e300;
                std::vector<ImVec2> seg;
                for(size_t i = 0; i < pts.size(); i++) {
                    const glm::dvec3 pr = O * pts[i] + P;
                    const double l = glm::length(pr);
                    if(l < 1e-9) { continue; }
                    double lon, lat;
                    surfmapLonLat(pr / l, lon, lat);
                    if(prev_lon > -1.0e299 && surfmapWraps(prev_lon, lon)) {
                        if(seg.size() >= 2) {
                            dl->AddPolyline(seg.data(), (int)seg.size(),
                                            sm_ship, 1.0f);
                        }
                        seg.clear();
                    }
                    seg.push_back(map_px(lon, lat));
                    prev_lon = lon;
                }
                if(seg.size() >= 2) {
                    dl->AddPolyline(seg.data(), (int)seg.size(),
                                    sm_ship, 1.0f);
                }
                // Apsides (closed orbit, non-circular): propagate to each
                // (exact, same as the Orbital Map) and drop a dot.
                if(closed && o.ecc > 1e-3) {
                    glm::dvec3 ap_p, tmp;
                    ImVec2 apx;
                    if(o.time_to_peri > 0.0) {
                        propagateKepler(orbit_pos, orbit_vel, mu,
                                       o.time_to_peri, ap_p, tmp);
                        if(to_px(ap_p, apx)) {
                            dl->AddCircleFilled(apx, 4.0f, sm_ship);
                        }
                    }
                    if(o.time_to_apo > 0.0) {
                        propagateKepler(orbit_pos, orbit_vel, mu,
                                       o.time_to_apo, ap_p, tmp);
                        if(to_px(ap_p, apx)) {
                            dl->AddCircleFilled(apx, 4.0f, sm_ship);
                        }
                    }
                }
            }
        }

        // The ship's position: a bright dot (you are here) with a green
        // ring, the same mark as the Orbital Map. Only on the ship's own
        // body -- on another body the ship is far away, and a dot at its
        // bearing would read as a surface position it isn't (the caption
        // below notes the SOI). The dot is the SUB-SATELLITE point: the
        // ship's COM rigidly transformed into the body's ROTATING frame
        // (the surface's frame) -- the same (lon, lat) the HUD reports
        // from render.cpp's surf_pos, so it stays glued to "what surface
        // I'm over" as the planet spins, right on the ground track. NOT
        // ship->frame's origin: two frames of the same body share an
        // origin, so the origin offset is 0 and the dot would vanish
        // exactly on the ship's own body.
        if(ship && sm_body == ship->m_parent) {
            Frame *rot = sm_body->frame->getRotFrame();
            const glm::dvec3 com = ship->get_center_of_mass();
            const glm::dvec3 sp = ship->frame->GetOrientRelTo(rot) * com
                                + ship->frame->GetPositionRelTo(rot);
            const double sl = glm::length(sp);
            if(sl > 1e-9) {
                double lon, lat;
                surfmapLonLat(sp / sl, lon, lat);
                const ImVec2 p = map_px(lon, lat);
                // A dot crossing the antimeridian (within the ring
                // radius of an edge) gets a twin on the other, so the two
                // halves complement instead of both dots showing in full.
                // Clip to the map image so neither spills into the window
                // margin: what's visible is exactly the part of one circle
                // inside the map, split across the two edges.
                const bool near_left  = (p.x - sm_p0.x) < 8.0f;
                const bool near_right = (sm_p0.x + sm_img_w - p.x) < 8.0f;
                dl->PushClipRect(sm_p0,
                                 ImVec2(sm_p0.x + sm_img_w,
                                        sm_p0.y + sm_img_h),
                                 true);
                dl->AddCircleFilled(p, 5.0f, sm_ink);
                dl->AddCircle(p, 8.0f, sm_ship, 0, 1.5f);
                if(near_left || near_right) {
                    const ImVec2 p2(near_left ? p.x + sm_img_w : p.x - sm_img_w,
                                    p.y);
                    dl->AddCircleFilled(p2, 5.0f, sm_ink);
                    dl->AddCircle(p2, 8.0f, sm_ship, 0, 1.5f);
                }
                dl->PopClipRect();
            }
        }

        if(ship && ship->m_parent && sm_body != ship->m_parent) {
            ImGui::TextDisabled("ship is in %s's SOI -- its orbit (about "
                                "%s) is not shown on %s's map",
                                ship->m_parent->name.c_str(),
                                ship->m_parent->name.c_str(),
                                sm_body->name.c_str());
        }
        ImGui::TextDisabled("equirectangular: lon 0 at the left edge, "
                            "north up   computed at t=%.1fs",
                            g.surfmap_computed_at);
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
                    glm::length(ship->GetPositionRelTo(ship->controller->body,
                                                        ship->home->frame)));
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

    // Initial size comes from o_telemetry.initial_size (a 2x2 grid of plots
    // needs real estate; content-fit would clip them).
    ui::Window("Telemetry", g.o_telemetry, [&] {
        // A 2x2 grid of plots. Each cell is a child region with a dropdown to
        // pick which series to show, then the plot. Positioned explicitly
        // (SetCursorPos) so the grid stays a clean 2x2 regardless of how the
        // child cursors settle.
        const ImVec2 avail = ImGui::GetContentRegionAvail();
        const float gap = ImGui::GetStyle().ItemSpacing.x;
        const float cw = (avail.x - gap) * 0.5f;
        const float ch = (avail.y - gap) * 0.5f;
        const ImVec2 origin = ImGui::GetCursorPos();
        for(int r = 0; r < 2; r++) {
            for(int c = 0; c < 2; c++) {
                const int idx = r * 2 + c;
                ImGui::SetCursorPos(ImVec2(origin.x + c * (cw + gap),
                                            origin.y + r * (ch + gap)));
                char id[16];
                snprintf(id, sizeof(id), "##tc%d", idx);
                ImGui::BeginChild(id, ImVec2(cw, ch));
                draw_telemetry_cell(g, idx);
                ImGui::EndChild();
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
    std::vector<Vehicle *> all = collectVehicles(sys);
    bool removed = false;
    for(size_t i = 0; i < all.size() && !removed; i++) {
        Vehicle *v = all[i];
        const bool active = (v == ship);
        ImGui::PushID((void*)v);
        if(v->isCrewAboard()) {
            // a crew character aboard a capsule: it is in the fleet but not
            // a controllable ship (no select/remove -- it lives in its
            // capsule; EVA it from the capsule window to make it free)
            ImGui::Text("%s (aboard)", v->name.c_str());
        } else {
            if(active) {
                ImGui::PushStyleColor(ImGuiCol_Button,
                                     ImVec4(0.30f, 0.45f, 0.70f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                                     ImVec4(0.35f, 0.50f, 0.75f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                                     ImVec4(0.40f, 0.55f, 0.80f, 1.0f));
            }
            if(ImGui::Button(v->name.c_str())) {
                g.select_ship(v);
            }
            if(active) {
                ImGui::PopStyleColor(3);
            }
            ImGui::SameLine();
            if(ImGui::SmallButton("x")) {
                g.remove_ship(v);
                removed = true;   // the ship was deleted; stop iterating
            }
        }
        ImGui::PopID();
    }
    ImGui::Separator();
    if(ImGui::Button("Spawn a copy of the active ship")) {
        if(!ship->defPath.empty()) {
            ships.spawn_ship(ship->defPath, "", ship->home, ship->scenario, sys);
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
                    glm::degrees(glm::length(GetAngVelocity(ship->controller->body))));
    });
    ui::Window("Ship Parts", g.o_parts, [&] {
        int i = 0;
        for(Part *p : ship->parts) {
            ImGui::Text("Part #%d  (stage %d)", i, p->stage);
            ImGui::Separator();
            ImGui::Text("Name: %s", p->def->name.c_str());
            ImGui::Text("Mass: %.3fkg", p->body->mass);
            ImGui::Text("Hydrogen: %.3fkg/%.3fkg",
                        p->resources.current[(int)ResourceType::Hydrogen],
                        p->resources.capacity[(int)ResourceType::Hydrogen]);
            ImGui::Text("LOX: %.3fkg/%.3fkg",
                        p->resources.current[(int)ResourceType::LOX],
                        p->resources.capacity[(int)ResourceType::LOX]);
            ImGui::Text("Hydrazine: %.3fkg/%.3fkg",
                        p->resources.current[(int)ResourceType::Hydrazine],
                        p->resources.capacity[(int)ResourceType::Hydrazine]);
            ImGui::Spacing();
            i++;
        }
    });

    ui::Window("Controls", g.o_controls, [&] {
        ImGui::Text("Game");
        ImGui::Separator();
        ImGui::Text("p - compute porkchop plot (Porkchop window)");
        ImGui::Text("m - refresh surface map (Surface Map window)");
        ImGui::Text("f11 - toggle wireframe mode");
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
        ImGui::Text("RMB click over 3D - pick a part (opens its window)");
        ImGui::Text("RMB (orbital map) - cycle window -> bare map -> no window");
        ImGui::Text("wheel - zoom (orbit mode)");
        ImGui::Spacing();
        ImGui::Text("Orbit mode (flying the ship)");
        ImGui::Separator();
        ImGui::Text("w/s - pitch up/down");
        ImGui::Text("a/d - yaw left/right");
        ImGui::Text("q/e - roll left/right (about the nose)");
        ImGui::Text("flip pitch/yaw/roll: Settings -> Controls");
        ImGui::Text("i - fire ship engines");
        ImGui::Text("x - kill rotation");
        ImGui::Text("Autopilot window - pro/retrograde + radial / normal slew");
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
        if(ship == nullptr) { return; }
        // Toggle the autopilot: click a mode to engage it -- the nose slews
        // toward the target and holds there -- and click it again to release.
        // The modes are mutually exclusive, like a navball; the engaged one
        // is highlighted (the same blue as the active ship in Ship List).
        auto toggle = [&](SlewMode m, const char *label) {
            const bool engaged = (ship->slewRequest == m);
            if(engaged) {
                ImGui::PushStyleColor(ImGuiCol_Button,
                                     ImVec4(0.30f, 0.45f, 0.70f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                                     ImVec4(0.35f, 0.50f, 0.75f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                                     ImVec4(0.40f, 0.55f, 0.80f, 1.0f));
            }
            if(ImGui::Button(label)) {
                ship->setSlewRequest(engaged ? SlewNone : m);
            }
            if(engaged) {
                ImGui::PopStyleColor(3);
            }
        };
        toggle(SlewPrograde, "Prograde");
        toggle(SlewRetrograde, "Retrograde");
        toggle(SlewRadialOut, "Radial-out");
        toggle(SlewRadialIn, "Radial-in");
        toggle(SlewNormal, "Normal");
        toggle(SlewAntiNormal, "Anti-normal");
        ImGui::Spacing();
        toggle(SlewKillRot, "Kill rotation");
    });

    ui::Window("Resources", g.o_resources, [&] {
        // aggregate across the active ship's parts (any ship layout); a
        // resource no tank carries (cap 0) reads as empty (0)
        auto frac = [&](ResourceType r) {
            float cur = 0, cap = 0;
            for(Part *p : ship->parts) {
                cur += p->resources.current[(int)r];
                cap += p->resources.capacity[(int)r];
            }
            return (cap > 0) ? cur / cap : 0.0f;
        };

        ImGui::ProgressBar(frac(ResourceType::Hydrogen), ImVec2(-1, 0), "Hydrogen");
        ImGui::ProgressBar(frac(ResourceType::LOX), ImVec2(-1, 0), "LOX");
        ImGui::ProgressBar(frac(ResourceType::Hydrazine), ImVec2(-1, 0), "Hydrazine");
        ImGui::ProgressBar(frac(ResourceType::EC), ImVec2(-1, 0), "Electric charge");
        ImGui::ProgressBar(frac(ResourceType::Oxygen), ImVec2(-1, 0), "Oxygen");
        ImGui::ProgressBar(frac(ResourceType::Water), ImVec2(-1, 0), "Water");
        ImGui::ProgressBar(frac(ResourceType::Food), ImVec2(-1, 0), "Food");
    });
}

/* The open part windows: one plain imgui window per part the player
   right-clicked in the 3D view (g.part_sels, opened by pickAt). Plain
   Begin/End -- these are user-placed popups, NOT slot-layout windows,
   and several may be open at once (e.g. two tanks for a fuel transfer).
   Closing the window (X) drops the entry; staging that drops the part
   makes it stale (the entry is dropped, not re-pointed -- the index now
   names a different part). */
void drawPartWindows(Game &g) {
    for(int i = (int)g.part_sels.size() - 1; i >= 0; i--) {
        const size_t idx = (size_t)i;
        PartSel &sel = g.part_sels[idx];
        Vehicle *ship = sel.ship;
        if(sel.part >= ship->parts.size()) {
            g.part_sels.erase(g.part_sels.begin() + idx);
            continue;
        }
        const size_t part = sel.part;
        const PartDef *def = ship->parts[part]->def;
        const Body *partBody = ship->parts[part]->body;

        char name[160];
        snprintf(name, sizeof(name), "Part: %s #%zu",
                 ship->name.c_str(), part);
        if(!sel.placed) {
            // Cascade the popups so several open ones don't fully
            // overlap; after that the user places them freely.
            const ImGuiViewport *vp = ImGui::GetMainViewport();
            ImGui::SetNextWindowPos(
                ImVec2(vp->WorkPos.x + 8.0f + 28.0f * (float)idx,
                       vp->WorkPos.y + 8.0f + 28.0f * (float)idx),
                ImGuiCond_Appearing);
            sel.placed = true;
        }
        bool open = true;
        if(ImGui::Begin(name, &open, ImGuiWindowFlags_NoSavedSettings)) {
            ImGui::Text("Ship: %s", ship->name.c_str());
            ImGui::Text("Part #%zu  (stage %d)", part,
                        ship->parts[part]->stage);
            ImGui::Separator();
            ImGui::Text("Name: %s", def->name.c_str());
            if(!def->type.empty()) {
                ImGui::Text("Type: %s", def->type.c_str());
            }
            ImGui::Text("Mass: %.3fkg", partBody->mass);
            ImGui::Text("Size: %.1fm dia x %.1fm",
                        def->radius * 2.0, def->height);
            if(def->torque > 0.0) {
                ImGui::Text("Torque: %.0fN m (reaction wheel)", def->torque);
            }
            if(def->fuel_rate > 0.0 && def->exhaust_velocity > 0.0) {
                ImGui::Text("Thrust: %.0fN (%.1fkg/s @ %.0fm/s)",
                            def->fullThrust(), def->fuel_rate,
                            def->exhaust_velocity);
            }
            static const char *resNames[(int)ResourceType::Num] = {
                "Hydrogen", "LOX", "EC", "Oxygen", "Water", "Food", "Hydrazine"
            };
            for(int r = 0; r < (int)ResourceType::Num; r++) {
                if(def->capacity[(size_t)r] <= 0.0f) { continue; }
                ImGui::Text("%s: %.1fkg/%.1fkg", resNames[r],
                            ship->parts[part]->resources.current[r],
                            ship->parts[part]->resources.capacity[r]);
            }
            // --- crew (this part is a capsule: holds EVA characters) --------
            // Aboard crew get an EVA button (takes them out, game.cpp); a
            // free kerbal within boarding range (<= 10 m of the capsule)
            // gets a Board button (puts them in). The transitions move the
            // kerbal's mass onto/off the capsule and park/restore its body.
            if(def->crew_capacity > 0) {
                ImGui::Separator();
                std::vector<Kerbal *> aboard = partCrew(ship, part);
                ImGui::Text("Crew: %d / %d", (int)aboard.size(), def->crew_capacity);
                for(size_t ci = 0; ci < aboard.size(); ci++) {
                    Kerbal *k = aboard[ci];
                    ImGui::PushID(k);
                    ImGui::Text("  %s", k->name.c_str());
                    if(ImGui::SmallButton("EVA")) {
                        g.kerbalEVA(k);
                    }
                    ImGui::PopID();
                }
                // free kerbals in boarding range: a Board button each
                const glm::dvec3 capCom = GetPosition(ship->parts[part]->body);
                bool anyInRange = false;
                for(Kerbal *k : freeKerbals(g.sys)) {
                    const double dist =
                        glm::length(k->get_center_of_mass() - capCom);
                    if(dist > 10.0) { continue; }
                    anyInRange = true;
                    const bool full = ((int)aboard.size() >= def->crew_capacity);
                    ImGui::PushID(k);
                    ImGui::Text("  %s (%.1f m)%s", k->name.c_str(), dist,
                                full ? "  (capsule full)" : "");
                    if(ImGui::SmallButton("Board")) {
                        g.kerbalBoard(k, ship, part);
                    }
                    ImGui::PopID();
                }
                if(!anyInRange) {
                    ImGui::Text("  (no one in range to board)");
                }
            }
            ImGui::Separator();
            ImGui::Text("Picked at: (%.0f, %.0f, %.0f)",
                        sel.point.x, sel.point.y, sel.point.z);
        }
        ImGui::End();
        if(!open) {
            g.part_sels.erase(g.part_sels.begin() + idx);
        }
    }
}

// The orbital map: right-clicking the window cycles its chrome (full
// window -> bare map -> no window), the map square draws the focus
// body's neighborhood (child-body orbits, SOI rings, the ship's
// trajectory + apside markers, the other ships, the transfer conic),
// and the controls below it edit the map state on the game.
void drawUIMap(Game &g, TransferPlanner &planner) {
    Vehicle *ship = g.ship;
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
    // (orbit_caches, the per-orbit sampling cache, is file-scope --
    // shared with the Surface Map's orbit overlay.)

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
    
        // Every body's orbit (around its own parent) -- planets around the
        // star, moons around their planets -- projected into the focus's
        // frame, not just the focus's children. Each body's ellipse is
        // sampled in its PARENT's inertial frame (where the Kepler conic is
        // defined and its elements are constant while coasting -- the cache
        // is keyed on them, so a coasting body propagates once) and then
        // rotated/translated into the focus's frame at draw time (the parent
        // moves relative to the focus, so that transform is per-frame). The
        // star (no parent) and orbits whose parent SoI is under 10 px are
        // skipped (LOD -- their moons would be an unresolvable smudge).
        // Drawn before the ship's orbit, so the ship sits on top.
        for(auto *b : planets) {
            Frame *parent = (b->frame && b->frame->parent) ? b->frame->parent : nullptr;
            const double mu_c = b->frame ? b->frame->parent_mu : 0.0;
            if(!parent || mu_c <= 0.0) continue;                // star / non-orbiting
            if(parent->soi / (double)map_scale < 10.0) continue; // LOD: orbit < 10 px
            const glm::dvec3 cpos_p = b->frame->GetPositionRelTo(parent); // parent's frame
            const glm::dvec3 cvel_p = b->frame->GetVelocityRelTo(parent);
            const std::vector<glm::dvec3> &cpts_p =
                orbit_caches[(const void *)b].sample(cpos_p, cvel_p, mu_c, N);
            if(cpts_p.empty()) continue;
            const glm::dmat3 O = parent->GetOrientRelTo(focus->frame); // parent -> focus
            const glm::dvec3 P = parent->GetPositionRelTo(focus->frame);
            const glm::dvec3 cpos_f = b->frame->GetPositionRelTo(focus->frame); // body, focus frame
            // Draw the orbit starting and ending at the body so the line passes
            // exactly through its marker. The equal-mean-anomaly samples don't
            // include the body's position, so the chord near it otherwise visibly
            // misses the marker when zoomed in. The body lies on the orbit between
            // two consecutive samples: find the nearest sample (k) and its CLOSER
            // neighbour (k-1 or k+1) -- those two bracket the body -- then walk the
            // samples from that bracket all the way around to k and prepend the
            // body, so both chords touching the body are the short bracketing ones.
            // (Walking forward from k unconditionally ends at the wrong neighbour
            // when the body sits just past k, so the closing chord skips a sample
            // and jumps across the orbit -- a faint out-of-order line that appears
            // and disappears as the body crosses sample boundaries.)
            const size_t n = cpts_p.size();
            size_t k = 0;
            double best_d = 1e300;
            for(size_t i = 0; i < n; i++) {
                const double d = glm::length(O * cpts_p[i] + P - cpos_f);
                if(d < best_d) { best_d = d; k = i; }
            }
            const double d_km1 = glm::length(O * cpts_p[(k + n - 1) % n] + P - cpos_f);
            const double d_kp1 = glm::length(O * cpts_p[(k + 1) % n] + P - cpos_f);
            const size_t start = (d_kp1 < d_km1) ? (k + 1) % n : k;
            std::vector<glm::dvec3> cpts;
            cpts.reserve(n + 1);
            cpts.push_back(cpos_f);
            for(size_t j = 0; j < n; j++) {
                cpts.push_back(O * cpts_p[(start + j) % n] + P);
            }
            const bool selected = (b == sel_body);
            const ImU32 ccol = selected ? col_sel : col_child;
            map.drawOrbit(dl, cpts, ccol, selected ? 2.0f : 1.0f);
            const ImVec2 cpx = map.px(cpos_f);
            dl->AddCircleFilled(cpx, selected ? 5.0f : 3.0f, ccol);
            if(selected) { dl->AddCircle(cpx, 8.0f, ccol, 0, 1.0f); }
            dl->AddText(ImVec2(cpx.x + 4.0f, cpx.y - 12.0f), ink,
                        b->name.c_str());
            draw_soi(cpos_f, b->frame->soi);
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
            for(auto *s : focus->ships) {
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
        // Toggles (not just open): a quick way to open or close these
        // (besides their X / Back).
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
