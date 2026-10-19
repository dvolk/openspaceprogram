// gameui.cpp -- the ImGui UI pass (declared in gameui.h).
#include "gameui.h"

#include <algorithm>
#include <cassert>
#include <climits>
#include <cmath>
#include <numbers>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <vector>

#include "calendar.h"    // CalTime
#include "constants.h"   // kSurfaceModeAlt (the HUD's surface/orbital flip), map zoom bounds
#include "version.h"     // VERSION
#include "physics.h"     // GetAngVelocity
#include "siminput.h"    // the --sim-press / --sim-mouse queues
#include "uiinput.h"     // uiCombo: a Combo --ui-click can otherwise not name (#251)
#include "fmt.h"         // fmt_dist / fmt_time
#include "orbitsample.h" // OrbitSampleCache + open-arc sampling
#include "orbitmap.h"    // OrbitMap + contrastingColor
#include "equirect.h"    // equirectLonLat / equirectDir
#include "surfmap.h"     // surfmapShade / surfmapWraps + surfmapCompute
#include "texture.h"     // make_texture_r8
#include "vab.h"         // the editor ops (drawVabUI)
#include "render.h"      // updateShipView (re-take a stale g.hud.view snapshot)
#include "staging.h"     // computeStaging
#include "shipdef.h"     // list_vab_ship_defs
#include "resdir.h"      // resdir::path
#include "settings.h"    // kTerrainPxMin/Max (the Settings ladder's bounds)
#include "system.h"      // list_systems
#include "save.h"        // save_game / load_game / list_saves / delete_save
#include "datadir.h"     // the saves/ directory's location

#include "../middleware/imgui/imgui.h"
#include "../middleware/implot/implot.h"

// GLM's gtx extensions (glm::angle in ORBITAL) hard-error without this.
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/vector_angle.hpp>

namespace {
/* Viridis colormap: 11 anchor stops, linearly interpolated. t in [0,1]
   -> 0xAARRGGBB. */
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

// Cached orbit samplings keyed on the object's pointer; valid only
// while it is on a fixed Keplerian conic. See OrbitSampleCache.
static std::map<const void *, OrbitSampleCache> orbit_caches;

// Map content rect (screen px) for view culling.
struct MapViewRect {
    float l, t, r, b;
    bool contains(float x, float y, float margin) const {
        return x >= l - margin && x <= r + margin &&
               y >= t - margin && y <= b + margin;
    }
    // Conservative: does a disc (cx,cy) of radius r_px intersect the rect?
    bool hitsDisc(float cx, float cy, float r_px) const {
        if(r_px < 0.0f) { return cx >= l && cx <= r && cy >= t && cy <= b; }
        return cx + r_px >= l && cx - r_px <= r &&
               cy + r_px >= t && cy - r_px <= b;
    }
};

// The equal-anomaly samples exclude the body, so start the polyline at the
// nearest sample and walk around: else the chord near the marker misses it.
static void drawOrbitThroughBody(ImDrawList *dl, const OrbitMap &map,
                                 const std::vector<glm::dvec3> &cpts_p,
                                 const glm::dmat3 &O, const glm::dvec3 &P,
                                 const glm::dvec3 &cpos_p,
                                 ImU32 col, float thickness) {
    const size_t n = cpts_p.size();
    if(n < 2) { return; }
    size_t k = 0;
    double best = 1e300;
    for(size_t i = 0; i < n; i++) {
        const glm::dvec3 d = cpts_p[i] - cpos_p;
        const double d2 = glm::dot(d, d);
        if(d2 < best) { best = d2; k = i; }
    }
    const glm::dvec3 dkm = cpts_p[(k + n - 1) % n] - cpos_p;
    const glm::dvec3 dkp = cpts_p[(k + 1) % n] - cpos_p;
    const size_t start = (glm::dot(dkp, dkp) < glm::dot(dkm, dkm))
                             ? (k + 1) % n
                             : k;
    // Reused across bodies: one buffer for the whole map pass.
    static thread_local std::vector<glm::dvec3> cpts;
    cpts.clear();
    cpts.reserve(n + 1);
    cpts.push_back(cpos_p);
    for(size_t j = 0; j < n; j++) { cpts.push_back(cpts_p[(start + j) % n]); }
    // Transform parent -> focus in one pass; index 0 is the body, then the walk.
    static thread_local std::vector<glm::dvec3> cpts_f;
    cpts_f.clear();
    cpts_f.reserve(n + 1);
    for(const glm::dvec3 &p : cpts) { cpts_f.push_back(O * p + P); }
    map.drawOrbit(dl, cpts_f, col, thickness, /*closed=*/true, /*start=*/0);
}

// Sampled from each rail's IMMUTABLE epoch state (not the live rail -- that is
// what makes the cache hit), then rotated/translated into the focus's frame.
// LOD skips parents under 10 px SoI and orbits whose apoapsis disc misses the view.
static void drawSystemBodyOrbits(Game &g, TerrainBody *focus,
                                 const OrbitMap &map, ImDrawList *dl,
                                 const MapViewRect &view, double map_scale,
                                 TerrainBody *sel_body,
                                 ImU32 col_child, ImU32 col_sel, ImU32 ink,
                                 ImU32 soi_col) {
    std::vector<TerrainBody *> &planets = g.sys.bodies;
    // The star has no parent, so the loop below skips it: draw its disk + label here.
    if(TerrainBody *sun = g.sys.root; sun && sun != focus && sun->frame) {
        const glm::dvec3 sun_f = sun->frame->GetPositionRelTo(focus->frame);
        const ImVec2 sun_px = map.px(sun_f);
        const float sun_r_px = map.bodyRadiusPx(sun->radius, 4.0f);
        if(view.hitsDisc(sun_px.x, sun_px.y, sun_r_px)) {
            const ImU32 col_sun =
                ImGui::GetColorU32(ImVec4(1.0f, 0.85f, 0.45f, 1.0f));
            map.drawBody(dl, sun_f, sun->radius, col_sun, 4.0f);
            if(view.contains(sun_px.x, sun_px.y, sun_r_px + 16.0f)) {
                dl->AddText(ImVec2(sun_px.x + 6.0f, sun_px.y - sun_r_px - 12.0f),
                            ink, sun->name.c_str());
            }
        }
    }
    for(auto *b : planets) {
        Frame *parent = (b->frame && b->frame->parent) ? b->frame->parent : nullptr;
        const double mu_c = b->frame ? b->frame->parent_mu : 0.0;
        if(!parent || mu_c <= 0.0) continue;                // star / non-orbiting
        if(parent->soi / map_scale < 10.0) continue;        // LOD: orbit < 10 px
        // The rail epoch is the fixed conic in the LOCAL plane; `orient` carries the
        // tilt into the parent.
        const glm::dmat3 &Rloc = b->frame->orient;
        const glm::dvec3 epos_p = Rloc * b->frame->orbit_pos0;
        const glm::dvec3 evel_p = Rloc * b->frame->orbit_vel0;
        // Cheap size for cull + sample-count LOD, before any Kepler work.
        double sma = 0.0, apo = -1.0;
        orbitConicSize(epos_p, evel_p, mu_c, sma, apo);
        const glm::dmat3 O = parent->GetOrientRelTo(focus->frame); // parent -> focus
        const glm::dvec3 P = parent->GetPositionRelTo(focus->frame);
        // The ellipse sits within apoapsis of the parent, so a miss on that disc skips
        // the body.
        const glm::dvec3 cpos_p = b->frame->GetPositionRelTo(parent); // parent frame
        const glm::dvec3 cpos_f = O * cpos_p + P;
        const ImVec2 parent_px = map.px(P);
        const ImVec2 body_px = map.px(cpos_f);
        const float apo_px = (apo > 0.0) ? (float)(apo / map_scale) : 1e9f;
        const bool selected = (b == sel_body);
        if(!selected && !view.hitsDisc(parent_px.x, parent_px.y, apo_px)) {
            continue;
        }
        const int N = orbitSamplesForRadius(apo_px);
        const std::vector<glm::dvec3> &cpts_p =
            orbit_caches[(const void *)b].sample(epos_p, evel_p, mu_c, N);
        if(cpts_p.empty()) continue;
        const ImU32 ccol = selected ? col_sel : col_child;
        drawOrbitThroughBody(dl, map, cpts_p, O, P, cpos_p, ccol,
                             selected ? 2.0f : 1.0f);
        // Disk at the body's TRUE radius, floored so it stays visible zoomed out.
        const float min_r = selected ? 5.0f : 3.0f;
        const float body_r_px = map.bodyRadiusPx(b->radius, min_r);
        map.drawBody(dl, cpos_f, b->radius, ccol, min_r);
        if(selected) {
            dl->AddCircle(body_px, body_r_px + 4.0f, ccol, 0, 1.0f);
        }
        if(view.contains(body_px.x, body_px.y, body_r_px + 16.0f)) {
            const float label_dx = 6.0f, label_gap = 12.0f;
            dl->AddText(ImVec2(body_px.x + label_dx,
                               body_px.y - body_r_px - label_gap),
                        ink, b->name.c_str());
        }
        if(b->frame->soi > 0.0) {
            const float soi_px = (float)(b->frame->soi / map_scale);
            if(soi_px >= 1.0f && soi_px <= 4000.0f) {
                map.drawRing(dl, cpos_f, b->frame->soi, soi_col, 1.0f);
            }
        }
    }
}

// Sim-clock calendar formatting (fmt_cal_time / fmt_cal_compact /
// fmt_cal_duration) lives in calendar.h.

// Debris belts (JSON root "belts") as flat annuli, Tracking map only.
static const ImVec4 kBeltCols[] = {
    ImVec4(0.62f, 0.55f, 0.45f, 0.16f),
    ImVec4(0.55f, 0.63f, 0.75f, 0.12f),
};
constexpr size_t kBeltColsN = sizeof(kBeltCols) / sizeof(kBeltCols[0]);

// A FLAT ring in the star's equatorial plane, so it must be sampled in world
// space and projected -- OrbitMap::drawRing draws a screen-space circle, correct
// only for spheres.
static void drawDebrisBelts(Game &g, TerrainBody *focus, const OrbitMap &map,
                            ImDrawList *dl, const MapViewRect &view,
                            double map_scale) {
    if(g.sys.belts.empty()) { return; }
    TerrainBody *sun = g.sys.root;
    if(!sun || !sun->frame) { return; }
    const glm::dvec3 sun_f = sun->frame->GetPositionRelTo(focus->frame);
    // The belt plane's normal in the focus's inertial frame: the star's pole (#173).
    const glm::dvec3 n = sun->frame->spinAxisRelTo(focus->frame);
    // Any orthonormal pair spanning the belt plane.
    const glm::dvec3 ref = (std::abs(n.y) < 0.9) ? glm::dvec3(0.0, 1.0, 0.0)
                                                 : glm::dvec3(1.0, 0.0, 0.0);
    const glm::dvec3 u = glm::normalize(glm::cross(n, ref));
    const glm::dvec3 v = glm::cross(n, u);
    constexpr int N = 96;
    // Reused across bands and frames.
    thread_local std::vector<ImVec2> pi, po;
    pi.reserve(N); po.reserve(N);
    for(size_t bi = 0; bi < g.sys.belts.size(); bi++) {
        const BeltParams &band = g.sys.belts[bi];
        const ImVec4 &col = kBeltCols[bi % kBeltColsN];
        if((band.outer - band.inner) / map_scale < 1.0) { continue; }  // sub-pixel
        pi.clear(); po.clear();
        float l = 1e30f, t = 1e30f, r = -1e30f, b = -1e30f;
        for(int i = 0; i < N; i++) {
            const double a = 2.0 * std::numbers::pi * (double)i / (double)N;
            const glm::dvec3 dir = std::cos(a) * u + std::sin(a) * v;
            const glm::dvec2 qi = map.project(sun_f + band.inner * dir);
            const glm::dvec2 qo = map.project(sun_f + band.outer * dir);
            pi.push_back(ImVec2((float)qi.x, (float)qi.y));
            po.push_back(ImVec2((float)qo.x, (float)qo.y));
            // The outer loop's bbox bounds the whole annulus.
            l = std::min(l, (float)qo.x); r = std::max(r, (float)qo.x);
            t = std::min(t, (float)qo.y); b = std::max(b, (float)qo.y);
        }
        if(r < view.l || l > view.r || b < view.t || t > view.b) { continue; }
        // Quad list between the two projected loops: imgui has no annulus primitive,
        // and a mid-loop stroke would not foreshorten.
        const ImU32 fill = ImGui::GetColorU32(col);
        const ImVec2 uv(0.0f, 0.0f);
        dl->PrimReserve(6 * N, 6 * N);
        for(int i = 0; i < N; i++) {
            const int j = (i + 1) % N;
            dl->PrimVtx(pi[i], uv, fill);
            dl->PrimVtx(po[j], uv, fill);
            dl->PrimVtx(pi[j], uv, fill);
            dl->PrimVtx(pi[i], uv, fill);
            dl->PrimVtx(po[i], uv, fill);
            dl->PrimVtx(po[j], uv, fill);
        }
        ImVec4 rim = col;
        rim.w = std::min(1.0f, rim.w * 2.8f);
        const ImU32 rim_col = ImGui::GetColorU32(rim);
        dl->AddPolyline(pi.data(), N, rim_col, 1.0f, ImDrawFlags_Closed);
        dl->AddPolyline(po.data(), N, rim_col, 1.0f, ImDrawFlags_Closed);
    }
}

// --- Telemetry window ---
struct TeleSeriesDef { const char *name; const char *yaxis; };
static const TeleSeriesDef kSeries[8] = {
    {"specific orbital energy", "J/kg"},
    {"angular momentum", "m^2/s"},
    {"frame: events", "ms"},
    {"frame: logic", "ms"},
    {"frame: jobs", "ms"},
    {"frame: refresh", "ms"},
    {"frame: render", "ms"},
    {"frame: present", "ms"},
};
static const int kNumSeries = 8;

// Series 0-1 live on the active ship's view; 2-7 are Game's per-frame timings
// (#212).
static TimeSeries *telemetry_series(Game &g, int idx) {
    switch(idx) {
        case 0: return &g.hud.view.energy_series;
        case 1: return &g.hud.view.angmom_series;
        case 2: return &g.hud.perf_events;
        case 3: return &g.hud.perf_logic;
        case 4: return &g.hud.perf_jobs;
        case 5: return &g.hud.perf_refresh;
        case 6: return &g.hud.perf_render;
        case 7: return &g.hud.perf_present;
        default: return nullptr;
    }
}

// One grid cell: a full-width dropdown to pick the series, then the plot.
static void draw_telemetry_cell(Game &g, int idx) {
    const char *items[kNumSeries];
    for(int i = 0; i < kNumSeries; i++) { items[i] = kSeries[i].name; }
    ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
    /* One label per cell, and the cells are ##tc0..##tc3: four combos in one
       frame all called "##series". A bare --ui-click "##series" is therefore
       ambiguous by construction and the harness says so; a case has to name the
       window it means. */
    uiCombo("##series", &g.hud.telemetry_sel[idx], items, kNumSeries);
    ImGui::PopItemWidth();

    const int sel = g.hud.telemetry_sel[idx];
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

/* g.hud.view is the state pass's snapshot of the ship that pass saw. The imgui pass
   can switch the active ship mid-pass, so later windows read another ship's
   snapshot -- or none at all, whose zeros sent the HUD along
   glm::normalize(pos) = NaN (#235). Re-take on a stamp mismatch. */
static void syncShipView(Game &g) {
    if(g.ship != nullptr && g.hud.view.ship != g.ship) { updateShipView(g); }
}

// The flight windows below assume an active vessel: ui_remove_vehicle refuses
// the last.
void drawUIReadouts(Game &g) {
    // The window bodies' locals alias Game members.
    TransferPlanner &planner = g.xferPlanner;
    Vehicle *ship = g.ship;
    System &sys = g.sys;
    std::vector<Vehicle *> &vehicles = g.vehicles;
    GameArgs &args = g.args;
    Camera *camera = g.camera;
    int &time_accel = g.time_accel;
    int &cam_speed = g.cam_speed;
    double &time = g.time;
    int &ui_style = g.ui_style;
    float &window_rounding = g.window_rounding;
    float &ui_alpha = g.ui_alpha;
    float &ui_scale = g.ui_scale;
    float &sfx_volume = g.sfx_volume;
    float &music_volume = g.music_volume;
    // The DPI slider edits this; "Apply DPI" commits it to ui_scale.
    static float dpi_pending = 1.0f;
    // The Settings window writes these; the 3D pass (render.cpp) reads.
    bool &physics_debug_drawing = g.physics_debug_drawing;
    bool &world_drawing = g.world_drawing;
    bool &draw_starfield = g.draw_starfield;
    bool &draw_ref_circles = g.draw_ref_circles;
    bool &draw_orbit_plane = g.draw_orbit_plane;
    float &sky_dim = g.sky_dim;
    float &sky_dim_cone = g.sky_dim_cone;
    // The Settings window writes these; tick.cpp reads.
    bool &flip_pitch = g.flip_pitch;
    bool &flip_yaw = g.flip_yaw;
    bool &flip_roll = g.flip_roll;

    // The per-frame state the 3D pass computed (render.cpp).
    syncShipView(g);
    ShipView &view = g.hud.view;
    glm::dvec3 &pos = view.pos;
    /* The conic's velocity in this frame: the DEBUG window prints it beside the
       orbital speed and energy, all three of which come from the same state. In
       the rotating surface frame view.vel is the ground-relative one and matches
       none of them. */
    glm::dvec3 &marker_vel = view.marker_vel;
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

    // The transfer planner's state (the TARGET window's readouts + inputs).
    std::vector<TransferPlanner::XferTarget> &xferTargets = planner.xferTargets;
    int &xfer_target = planner.xfer_target;
    bool &xfer_compute = planner.xfer_compute;
    bool &xfer_auto = planner.xfer_auto;
    float &xfer_tof_log = planner.xfer_tof_log;
    auto &xfer = planner.xfer;
    auto &pc = planner.pc;   // the porkchop grid (Porkchop window)

    /* Top bar: one fixed window, re-placed every frame so it tracks the viewport.
       Row 1 (big font) is orbital above 30 km ASL or in the inertial frame, else
       surface; row 2 the clock. */
    drawWin(g, W_Hud, [&] {
        if(ship) {
            const double asl = distance
                - ((double)ship->m_parent->radius
                   + (double)ship->m_parent->surface.sea_level);
            const bool surface_mode = ship->frame->isRotFrame()
                                   && asl < kSurfaceModeAlt;
            // AGL is a full-detail terrain sample: ask only in the mode that shows it.
            const double alt = surface_mode
                ? distance
                  - (double)ship->m_parent->GetTerrainHeight(glm::normalize(pos))
                : asl;
            const double spd = surface_mode ? glm::length(surf_vel) : speed;
            ImGui::PushFont(g.bigger);
            // fmt_dist, not a fixed unit: at Oort distances a meter count is 16 digits wide
            // and the old (int) cast was UB.
            char spd_s[32], alt_s[32];
            ImGui::Text("%s/s   %s", fmt_dist(spd, spd_s, sizeof spd_s),
                        fmt_dist(alt, alt_s, sizeof alt_s));
            ImGui::PopFont();
        }
        if(sys.home) {
            char line[64];
            if(fmt_cal_time(sys.home->cal, time, line, sizeof line)) {
                ImGui::SetCursorPosX((ImGui::GetWindowWidth()
                    - ImGui::CalcTextSize(line).x) * 0.5f);
                ImGui::TextUnformatted(line);
            }
        }
    });

    /* Window list: straight from the uiwins.cpp table, so it can never offer a
       toggle for a window not drawn here. inList=false entries are toggled from
       their parent; Root/Chrome are not rows. */
    drawWin(g, W_Windows, [&] {
        ImGui::Spacing();
        const WinSet &set = curScene(g).wins;
        for(size_t i = 0; i < set.n; i++) {
            const Win w = set.ids[i];
            const WinDef &wd = kWins[w];
            if(!wd.inList) { continue; }
            bool open = winOpen(w);
            if(ImGui::Checkbox(wd.label, &open)) {
                setWinOpen(w, open);
            }
        }
    });

    // Settings: the render/physics debug toggles.
    drawWin(g, W_Settings, [&] {
        {
            static const char *const mode_names[] =
                {"windowed", "borderless", "fullscreen", "exclusive"};
            /* displayModes() mallocs, dedups (O(n^2)) and sorts the whole array: refresh at
               ~1 Hz while the window is open, not per frame. */
            static const double kModeRefreshSec = 1.0;
            static double nextModeRefresh = 0.0;
            static std::vector<Resolution> modes;
            static int cur_refresh = 0;
            static std::string items;
            auto refreshModes = [&] {
                nextModeRefresh = ImGui::GetTime() + kModeRefreshSec;
                modes = g.display.displayModes();
                cur_refresh = g.display.currentRefresh();
                items.clear();
                char buf[48];
                for(size_t i = 0; i < modes.size(); i++) {
                    // The Hz tells same-WxH entries apart (0 = unknown, omit it).
                    snprintf(buf, sizeof(buf),
                             modes[i].refresh > 0 ? "%dx%d @ %dHz" : "%dx%d",
                             modes[i].width, modes[i].height,
                             modes[i].refresh);
                    items += buf;
                    items += '\0';
                }
                items += '\0';
            };
            int wm = (int)args.window_mode;
            if(uiCombo("Window mode", &wm,
                            "windowed\0borderless\0fullscreen\0exclusive\0")) {
                args.window_mode = static_cast<WindowMode>(wm);
                g.display.setWindowMode(args.window_mode,
                                        args.screen_width, args.screen_height);
                g.toast("Window mode: %s", mode_names[wm]);
            }
            if(ImGui::GetTime() >= nextModeRefresh) {
                refreshModes();
            }
            // Exact WxH match, preferring the refresh closest to the display's current,
            // else nearest WxH.
            if(!modes.empty()) {
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
                const bool native_fs =
                    (args.window_mode == WindowMode::Fullscreen);
                ImGui::BeginDisabled(native_fs);
                if(uiCombo("Display mode", &sel, items.c_str())) {
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
            // MSAA is fixed at window creation (the GLX visual is chosen then): this sets
            // the LAUNCH value, effective on restart.
            {
                static const int aa_values[] = {0, 2, 4, 8};
                static const char *const aa_names[] = {"none", "2x", "4x",
                                                       "8x"};
                const int n_aa = 4;
                int sel = 0, best = INT_MAX;
                for(int i = 0; i < n_aa; i++) {
                    const int d = std::abs(aa_values[i] - args.msaa_samples);
                    if(d < best) { best = d; sel = i; }
                }
                // Split literals: in C, "\02" is the OCTAL escape for 0x02, not a
                // null + '2' -- one literal would list 2x/4x as missing-glyph boxes.
                if(uiCombo("Antialiasing", &sel,
                                "none\0" "2x\0" "4x\0" "8x\0")) {
                    args.msaa_samples = aa_values[sel];
                    g.toast("Antialiasing: %s (applies on restart)",
                            aa_names[sel]);
                }
            }
            ImGui::Separator();
        }
        ImGui::Checkbox("Physics debug draw", &physics_debug_drawing);
        ImGui::Checkbox("World draw", &world_drawing);
        ImGui::Checkbox("Starfield", &draw_starfield);
        ImGui::Checkbox("Reference circles", &draw_ref_circles);
        // render.cpp drawOrbitPlane: the orbit plane (cyan), the orbit map's
        // reference plane (yellow) and a tick where they cross (white), so Inc
        // reads as an angle and the plane-change burn direction is on screen.
        // F3 (Slot::ToggleOrbitPlane) mirrors this from anywhere, mid-burn.
        ImGui::Checkbox("Orbital plane", &draw_orbit_plane);
        // Prototype (render.cpp skyGain): how far stars fade while the sun is in view
        // and the camera is in sunlight.
        ImGui::SliderFloat("Sky dim", &sky_dim, 0.0f, 1.0f, "%.2f");
        // Half-angle of the "sun in view" cone (same units as --sky-dim-cone).
        ImGui::SliderFloat("Sky dim cone", &sky_dim_cone, 1.0f, 179.0f, "%.0f");
        // Post-processing: one checkbox per effect (the passes run in this order),
        // plus a slider per parameter.
        ImGui::Separator();
        ImGui::Text("Post-processing");
        for(const std::string &fx : PostFX::Available()) {
            const char *label = fx.c_str();
            if(fx == "crt") label = "CRT (retro tube)";
            else if(fx == "grain") label = "Film grain";
            else if(fx == "cas") label = "CAS sharpen";
            else if(fx == "color") {
                label = "Color (gamma, brightness, black level, saturation)";
            }
            bool on = g.postfx->IsEnabled(fx);
            if(ImGui::Checkbox(label, &on)) {
                g.postfx->SetEnabled(fx, on);
            }
            if(!on) { continue; }
            const std::vector<FXParam> params = PostFX::Params(fx);
            if(params.empty()) { continue; }
            for(const FXParam &p : params) {
                // Underscored uniform name -> natural-language label.
                char label[64];
                snprintf(label, sizeof(label), "%s", p.name);
                for(char *c = label; *c; c++) { if(*c == '_') *c = ' '; }
                float v = g.postfx->GetParam(fx, p.name);
                if(ImGui::SliderFloat(label, &v, p.min, p.max, "%.2f")) {
                    g.postfx->SetParam(fx, p.name, v);
                }
            }
        }
        // Audio: master levels apply live and persist with "Save"; a no-op while
        // audio is disabled.
        ImGui::Separator();
        ImGui::Text("Audio");
        if(ImGui::SliderFloat("SFX volume", &sfx_volume, 0.0f, 1.0f, "%.2f")) {
            g.audio.setSfxVolume(sfx_volume);
        }
        if(ImGui::SliderFloat("Music volume", &music_volume, 0.0f, 1.0f,
                             "%.2f")) {
            g.audio.setMusicVolume(music_volume);
        }
        if(uiCombo("UI style", &ui_style, "Dark\0Light\0Classic\0")) {
            g.apply_ui_style();
        }
        if(ImGui::SliderFloat("Window rounding", &window_rounding,
                             0.0f, 50.0f, "%.0f")) {
            g.apply_ui_style();
        }
        if(ImGui::SliderFloat("Window opacity", &ui_alpha,
                             0.2f, 1.0f, "%.2f")) {
            g.apply_ui_style();
        }
        // Applying live while dragging would move this window out from under the cursor.
        ImGui::SliderFloat("DPI scale", &dpi_pending, 0.5f, 3.0f, "%.2fx");
        ImGui::BeginDisabled(dpi_pending == ui_scale);
        if(ImGui::Button("Apply DPI")) {
            ui_scale = dpi_pending;
            g.apply_ui_style();
            // TODO(dpi): windows don't re-fit / re-place at the new scale. A relayout attempt
            // refit them, but the first pass used the PREVIOUS size's font advances; suspect the
            // imgui 1.92 per-size font bakes (ImFont::GetFontBaked).
        }
        ImGui::EndDisabled();
        if(ImGui::SliderFloat("FOV", &args.camFovDeg, 10.0f, 120.0f, "%.0f°")) {
            const float f = (float)glm::radians(args.camFovDeg);
            camera->setFov(f);
        }
        // Terrain LOD: a patch subdivides while it projects wider than args.terrain_px
        // [real screen px]. 64/32 are gone: the budget is per patch, so they explode the
        // patch count and the async builder never reaches max_depth -- where the collision
        // meshes live (terrain.cpp).
        {
            constexpr int terrain_px_table[] = { 1024, 512, 256, 128 };
            constexpr int nlevels = 4;
            static_assert(terrain_px_table[0] <= kTerrainPxMax
                          && terrain_px_table[nlevels - 1] >= kTerrainPxMin,
                          "Settings terrain ladder outside the --terrain-px range");
            int terrain_level = 0, best = -1;
            for (int i = 0; i < nlevels; i++) {
                const int d = std::abs(args.terrain_px - terrain_px_table[i]);
                if (best < 0 || d < best) { best = d; terrain_level = i; }
            }
            if(ImGui::SliderInt("Terrain detail", &terrain_level, 0, nlevels - 1)) {
                args.terrain_px = terrain_px_table[terrain_level];
            }
        }
        // Camera shake: the chase cam rumbles with the crew's felt acceleration
        // (thrust + aero over mass, gravity excluded).
        ImGui::SliderFloat("Camera shake", &args.cam_shake,
                           0.0f, 3.0f, "%.1fx");
        // Controls: invert a manual attitude axis away from the default (see tick.cpp).
        ImGui::Separator();
        ImGui::Text("Controls (W/S pitch, A/D yaw, Q/E roll)");
        ImGui::Checkbox("Flip pitch (W/S)", &flip_pitch);
        ImGui::Checkbox("Flip yaw (A/D)", &flip_yaw);
        ImGui::Checkbox("Flip roll (Q/E)", &flip_roll);
        ImGui::Spacing();
        if(ImGui::Button("Save settings", ImVec2(240.0f, 0.0f))) {
            if(g.save_settings()) {
                g.toast("Settings saved (settings.json)");
            } else {
                g.toast("Could not write settings.json");
            }
        }
        if(ImGui::Button("Back", ImVec2(240.0f, 0.0f))) {
            setWinOpen(W_Settings, false);
        }
    });

    // Target selection + transfer planner: parent->child transfers (with
    // capture) and same-body ship intercepts. The solution is computed in the
    // 3D pass, so this window is readout + inputs -- and only when the user
    // asked for a plan at all.
    drawWin(g, W_Target, [&] {
        if(xferTargets.empty()) {
            ImGui::Text("No transfer targets: no child bodies or ships here.");
            return;
        }
        const char *cur = (xfer_target >= 0)
                         ? xferTargets[xfer_target].name : "none";
        if(uiBeginCombo("Target", cur)) {
            // "none" is a real entry so the target can be cleared back to -1 from here.
            if(ImGui::Selectable("none", xfer_target == -1)) {
                planner.selectTarget(std::string(), false);
            }
            for(int i = 0; i < (int)xferTargets.size(); i++) {
                if(ImGui::Selectable(xferTargets[i].name,
                                     i == xfer_target)) {
                    // By identity, and the row's KIND with it: a body and a
                    // vessel can share a name, and the list shows both rows.
                    planner.selectTarget(xferTargets[i].name,
                                         xferTargets[i].ship != nullptr);
                }
            }
            ImGui::EndCombo();
        }
        /* Two checkboxes, one row: both are switches about the plan rather than
           readouts of it. The window is pinned at 420 x 328 (uiwins.cpp), so the
           row this frees is scroll headroom, not height -- with a porkchop plan
           armed the content ends within a few px of the bottom edge either way,
           and the next line added here puts the scrollbar back. The Porkchop
           window hangs off this window, not the Windows list. */
        bool pc_open = winOpen(W_Porkchop);
        if(ImGui::Checkbox("Porkchop", &pc_open)) {
            setWinOpen(W_Porkchop, pc_open);
        }
        /* A target is worth selecting even with no plan: its orbit is on the
           map, and its plane is one of the map's planes. Off, nothing solves,
           so the HUD burn marker, the map arc and everything below go quiet
           together. Turning it off drops an armed porkchop plan -- the countdown
           and the burn direction both need a live solution -- and "Send best"
           turns planning back on. It rides above the "no target yet" return
           because it is a mode: the setting you want is the one you leave in
           place before you pick what to plan a burn to. */
        ImGui::SameLine();
        if(ImGui::Checkbox("Compute transfer", &xfer_compute) && !xfer_compute) {
            planner.clearPorkchopPlan();
        }
        if(xfer_target < 0) {
            ImGui::Text("Select a target body or ship.");
            return;
        }
        if(!xfer_compute) { return; }
        const bool isShip = xferTargets[xfer_target].ship != nullptr;
        if(isShip) {
            ImGui::TextDisabled("ship target: intercept only, no capture burn");
        }
        // A porkchop "Send best" plan: count down to the departure instant.
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
                char dt[64];
                if(fmt_cal_time(sys.home->cal, planner.xfer_t_dep,
                                dt, sizeof dt)) {
                    ImGui::Text("departure:    %s", dt);
                }
            }
            if(ImGui::Button("Clear plan")) {
                planner.clearPorkchopPlan();
            }
            ImGui::Separator();
        }
        if(ImGui::Checkbox("Auto ToF (min dv)", &xfer_auto) && !xfer_auto
           && xfer.valid) {
            // Seed manual mode from what auto just recommended. The stored
            // default is log10(3600) -- one hour -- which solves for no planet,
            // so switching to manual used to blank the window.
            xfer_tof_log = (float)std::log10(xfer.sol.tof);
        }
        if(!xfer_auto) {
            // The human ToF rides on the slider row: the solution block below
            // already prints a ToF line, and echoing it here as a second row
            // read as the window showing two different answers.
            /* The band is the target's own ToF range and must NOT follow the
               current value: imgui maps the pointer to an absolute fraction of the
               band on every held frame, so a band whose end IS the value decays it
               toward the floor while the button is held, and can never raise it.
               The ceiling is the porkchop range sliders' own end (one definition,
               transferplanner.h), so no grid cell can arm a plan the slider cannot
               show; the floor is the 60 s every sweep starts at. A value outside
               the band can only come from switching to a shorter-period target, and
               the grab sitting at the end says so before a touch corrects it. */
            ImGui::PushItemWidth(150.0f);
            ImGui::SliderFloat("log10(ToF s)", &xfer_tof_log,
                               1.7782f,   // log10(60 s)
                               (float)std::log10(planner.xfer_tof_ceiling()),
                               "%.2f");
            ImGui::SameLine();
            char tof_s[40];
            ImGui::TextDisabled("%s", fmt_time(std::pow(10.0, xfer_tof_log),
                                               tof_s, sizeof tof_s));
            ImGui::PopItemWidth();
        }
        if(!xfer.valid) {
            ImGui::Text("No transfer solution for this target / ToF.");
            return;
        }
        const TransferSolution &sol = xfer.sol;
        char dist_s[32], time_s[40];
        ImGui::Text("dv depart:  %08.1f m/s", sol.dv_departure);
        if(!isShip) {
            ImGui::Text("dv capture: %08.1f m/s @ %s",
                        sol.dv_capture, fmt_dist(sol.r_cap, dist_s, sizeof dist_s));
            if(sol.capture_orbit_period > 0.0) {
                ImGui::Text("capture P:  %s",
                            fmt_time(sol.capture_orbit_period, time_s, sizeof time_s));
            }
        }
        ImGui::Text("total dv:   %08.1f m/s", sol.total_dv);
        // The revolution count rides on the ToF line because that is the
        // number it explains: a ToF that looks absurd next to the target's
        // own orbit is a phasing leg, not a bug (#136).
        ImGui::Text("ToF:        %s (revolutions: %d)",
                    fmt_time(sol.tof, time_s, sizeof time_s), sol.revolutions);
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

    /* Porkchop plot: the 2-D launch-window map (total dv over departure delay x time
       of flight), computed on demand and cached. */
    drawWin(g, W_Porkchop, [&] {
        if(xferTargets.empty()) {
            ImGui::Text("No transfer targets: no child bodies or ships here.");
            return;
        }
        if(xfer_target < 0) {
            ImGui::Text("Select a target body or ship (Target window).");
            return;
        }
        const char *tn = xferTargets[xfer_target].name;

        // The grid sweep runs on the background worker (g.jobs), so this is a one-shot
        // button, not a per-frame re-sweep.
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
        // The grid ON SCREEN, not the configured size: --porkchop-bench leaves the largest
        // swept grid live, so the two can disagree.
        ImGui::TextDisabled("grid %d x %d   (size: --porkchop-n / --porkchop-bench)",
                            pc.valid ? pc.n_dep : g.args.porkchop_n,
                            pc.valid ? pc.n_tof : g.args.porkchop_n);
        if(pc_busy) {
            ImGui::TextDisabled("(the last grid stays shown until the new one "
                                "lands)");
        }

        // Axis ranges. Off = auto (departure 0..one target period; ToF 60 s..
        // three). On = the sliders, whose ends scale with the target: a fixed
        // ceiling cannot express a Mars window, let alone a 1-revolution
        // phasing leg, and it silently clamped the seeded range to a week.
        // No readout here: the x:/y: lines under the plot already give the
        // range actually swept, and this window has no rows to spare at 720p.
        const double period = planner.pc_period > 0.0 ? planner.pc_period : 86400.0;
        // 3x/9x the auto end, but never below the week the sliders used to
        // stop at: a moon's three periods are hours, and a player exploring
        // past them should not find the handle shorter than before.
        const float dep_max = (float)std::max(3.0 * period, 604800.0);
        // The same ceiling the Target window's manual ToF slider offers, so a cell
        // inside these sliders is always a cell that slider can display.
        const float tof_max = (float)planner.xfer_tof_ceiling();
        if(ImGui::Checkbox("Departure range", &planner.pcCustomDep)
           && planner.pcCustomDep) {
            // Just enabled: start from what the last sweep actually used.
            planner.pcDepLo = pc.valid ? (float)pc.t_dep_lo : 0.0f;
            planner.pcDepHi = pc.valid ? (float)pc.t_dep_hi : (float)period;
        }
        if(planner.pcCustomDep) {
            // The slider prints the raw seconds (drag with it, ctrl+click to
            // type); the human time goes beside it, since 1.8e+08 s means
            // nothing to anyone planning a launch.
            ImGui::PushItemWidth(150.0f);
            ImGui::SliderFloat("start (dep)", &planner.pcDepLo, 0.0f,
                               dep_max, "%.3g s");
            ImGui::SameLine();
            { char b[40]; ImGui::TextDisabled("%s", fmt_time(planner.pcDepLo, b, sizeof b)); }
            ImGui::SliderFloat("end (dep)", &planner.pcDepHi, 0.0f,
                               dep_max, "%.3g s");
            ImGui::SameLine();
            { char b[40]; ImGui::TextDisabled("%s", fmt_time(planner.pcDepHi, b, sizeof b)); }
            ImGui::PopItemWidth();
        }
        if(ImGui::Checkbox("ToF range", &planner.pcCustomTof)
           && planner.pcCustomTof) {
            // Just enabled: start from what the last sweep actually used.
            planner.pcTofLo = pc.valid ? (float)pc.tof_lo : 60.0f;
            planner.pcTofHi = pc.valid ? (float)pc.tof_hi : (float)(3.0 * period);
        }
        if(planner.pcCustomTof) {
            ImGui::PushItemWidth(150.0f);
            ImGui::SliderFloat("start (ToF)", &planner.pcTofLo, 60.0f,
                               tof_max, "%.3g s");
            ImGui::SameLine();
            { char b[40]; ImGui::TextDisabled("%s", fmt_time(planner.pcTofLo, b, sizeof b)); }
            ImGui::SliderFloat("end (ToF)", &planner.pcTofHi, 60.0f,
                               tof_max, "%.3g s");
            ImGui::SameLine();
            { char b[40]; ImGui::TextDisabled("%s", fmt_time(planner.pcTofHi, b, sizeof b)); }
            ImGui::PopItemWidth();
        }
        if(planner.pcCustomDep || planner.pcCustomTof) {
            ImGui::TextDisabled("(press Compute / P to re-sweep)");
        }

        if(!pc.valid) {
            if(pc_busy) {
                // A sweep is in flight: show the working state, not "no window".
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
        char pc_t[40];
        ImGui::Text("min dv:      %08.1f m/s", pc.dv_min);
        ImGui::Text("depart in:   %s", fmt_time(pc.t_dep_min, pc_t, sizeof pc_t));
        ImGui::Text("time of flt: %s", fmt_time(pc.tof_min, pc_t, sizeof pc_t));
        // Send the best cell to the Target planner: pin the ToF and record the ABSOLUTE
        // departure time. Disabled during a sweep: pc is then the PREVIOUS grid.
        if(pc_busy) { ImGui::BeginDisabled(); }
        if(ImGui::Button("Send best to Target")) {
            // Remember the user's ToF mode so "Clear plan" restores it.
            planner.xfer_prev_auto = planner.xfer_auto;
            planner.xfer_prev_tof_log = planner.xfer_tof_log;
            planner.xfer_auto = false;
            planner.xfer_tof_log = (float)std::log10(pc.tof_min);
            planner.xfer_t_dep = planner.pc_computed_at + pc.t_dep_min;
            // By identity: the index is only this frame's address for the
            // target, and the plan outlives reshuffles of the list.
            planner.xfer_plan_target = planner.xfer_sel;
            planner.xfer_from_porkchop = true;
            // A plan IS a live solution: the countdown and the burn direction
            // need one, so arming it turns planning back on.
            planner.xfer_compute = true;
            setWinOpen(W_Target, true);
        }
        if(pc_busy) { ImGui::EndDisabled(); }

        // Heatmap: total dv over (departure delay x, time of flight y), stored ToF-major
        // (rows = ToF, cols = departure). Drawn as a texture, not ImPlot::PlotHeatmap:
        // that indexes its color LUT with the raw cell value, so the no-solution (NaN)
        // cells read out of bounds and assert.
        // Color scale [dv_min, dv_hi]: dv_hi is the 95th percentile, not the absolute max
        // -- the dv surface has a narrow unphysical spike at the shortest ToFs.
        double lo = pc.dv_min;
        double hi = (pc.dv_hi > lo) ? pc.dv_hi : lo;
        const int w = pc.n_dep, h = pc.n_tof;
        // A CACHE of pc: px and the texture only change when a sweep lands, so gate both
        // on pc_rev. px keeps its peak capacity, so the largest grid ever swept stays
        // resident.
        static Texture *pc_tex = nullptr;
        static std::vector<unsigned char> px;   // persists across frames
        static int tex_w = 0, tex_h = 0, tex_rev = -1;
        // One read of the stamp: tex_rev must name the pixels built below.
        const int rev = planner.pc_rev;
        if(rev != tex_rev) {
            px.resize((size_t)w * h * 4);
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
            if(!pc_tex || tex_w != w || tex_h != h) {
                if(pc_tex) { delete pc_tex; }
                pc_tex = make_texture_r8(w, h, px.data());
                tex_w = w;
                tex_h = h;
            } else {
                upload_texture_r8(pc_tex, w, h, px.data());
            }
            tex_rev = rev;
        }
        // The fill indexes total_dv as w x h and the Image derefs the texture: both must
        // match the grid on screen.
        assert(pc.total_dv.size() == (size_t)w * h && pc_tex
               && tex_w == w && tex_h == h);
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
        // Fixed display size (the window auto-fits around it).
        const float img_sz = 420.0f;
        const ImVec2 img_tl = ImGui::GetCursorScreenPos();
        ImGui::Image((ImTextureID)(std::intptr_t)pc_tex->id,
                     ImVec2(img_sz, img_sz), ImVec2(0, 1), ImVec2(1, 0));
        /* Overlays on the heatmap. Data -> pixel: x is departure delay from the
           instant the grid was POSTED (pc_computed_at), y is ToF, and the uv
           pair above flips the texture, so tof_lo is at the BOTTOM of the quad.
           Both spans are guarded because the range sliders can be dragged flat
           (lo == hi), which would divide by zero. */
        const double dep_span = pc.t_dep_hi - pc.t_dep_lo;
        const double tof_span = pc.tof_hi - pc.tof_lo;
        if(dep_span > 0.0 && tof_span > 0.0) {
            auto plot_px = [&](double dep, double tof) {
                return ImVec2(img_tl.x + (float)((dep - pc.t_dep_lo) / dep_span) * img_sz,
                              img_tl.y + img_sz
                                     - (float)((tof - pc.tof_lo) / tof_span) * img_sz);
            };
            ImDrawList *dl = ImGui::GetWindowDrawList();
            /* The range sliders set the next sweep's window, and a sweep is
               n x n Lambert solves on the worker: draw the crop box before it is
               paid for. Only lines inside the swept range -- outside it there is
               nothing on this image to crop. They run 2 px past the plot at both
               ends: a line that stops exactly at the border is hard to tell from
               the border once the heatmap's colours reach it, and the stubs say
               "this belongs to the axes, not the picture" without reaching into
               the widgets above and below. */
            const ImU32 pending_col = IM_COL32(255, 255, 255, 230);
            const float over = 2.0f;
            auto vline = [&](double dep) {
                if(dep < pc.t_dep_lo || dep > pc.t_dep_hi) { return; }
                const float x = plot_px(dep, pc.tof_lo).x;
                dl->AddLine(ImVec2(x, img_tl.y - over),
                            ImVec2(x, img_tl.y + img_sz + over), pending_col, 1.0f);
            };
            auto hline = [&](double tof) {
                if(tof < pc.tof_lo || tof > pc.tof_hi) { return; }
                const float y = plot_px(pc.t_dep_lo, tof).y;
                dl->AddLine(ImVec2(img_tl.x - over, y),
                            ImVec2(img_tl.x + img_sz + over, y), pending_col, 1.0f);
            };
            if(planner.pcCustomDep) {
                vline(planner.pcDepLo);
                vline(planner.pcDepHi);
            }
            if(planner.pcCustomTof) {
                hline(planner.pcTofLo);
                hline(planner.pcTofHi);
            }
            /* The armed plan: the Target window counts down to this departure
               and the axis is delay from the sweep, so subtract the stamp the
               grid was posted at. Its ToF is the pin "Send best" set -- or
               whatever auto recommends, if Auto came back on -- and a plan is
               only ever armed with planning on, so the fallback is a solve that
               found nothing. Off the swept range, nothing draws: the plot shows
               what was swept, and the Target window still carries the plan. */
            if(planner.xfer_from_porkchop && planner.xfer_t_dep > 0.0) {
                const double dep = planner.xfer_t_dep - planner.pc_computed_at;
                const double tof = xfer.valid ? xfer.sol.tof
                                             : std::pow(10.0, xfer_tof_log);
                if(dep >= pc.t_dep_lo && dep <= pc.t_dep_hi
                   && tof >= pc.tof_lo && tof <= pc.tof_hi) {
                    const ImVec2 c = plot_px(dep, tof);
                    const ImU32 plan_col = IM_COL32(255, 255, 255, 255);
                    dl->AddLine(ImVec2(c.x - 9.0f, c.y), ImVec2(c.x + 9.0f, c.y), plan_col, 1.5f);
                    dl->AddLine(ImVec2(c.x, c.y - 9.0f), ImVec2(c.x, c.y + 9.0f), plan_col, 1.5f);
                    dl->AddCircle(c, 5.0f, plan_col, 0, 1.5f);
                }
            }
        }
        ImGui::SameLine();
        // Colorbar: hi at the top, lo at the bottom, labels aligned to the bar.
        ImGui::BeginGroup();
            ImGui::Text("%.0f", hi);
            ImGui::Image((ImTextureID)(std::intptr_t)bar_tex->id,
                         ImVec2(16.0f,
                                std::max(20.0f, img_sz -
                                         ImGui::GetTextLineHeight() * 2.0f)),
                         ImVec2(0, 1), ImVec2(1, 0));
            ImGui::Text("%.0f", lo);
        ImGui::EndGroup();
        char pc_min[40];
        ImGui::TextDisabled("x: departure delay  %.0f .. %.0f s (min %s)",
                            pc.t_dep_lo, pc.t_dep_hi,
                            fmt_time(pc.t_dep_min, pc_min, sizeof pc_min));
        ImGui::TextDisabled("y: time of flight   %.0f .. %.0f s (min %s)",
                            pc.tof_lo, pc.tof_hi, fmt_time(pc.tof_min, pc_min, sizeof pc_min));
        ImGui::TextDisabled("bar: dv in m/s (top = max)   gray: no solution");
    });

    /* Surface Map: the chosen body's surface as an equirectangular 2-D map with the
       ship's position + orbit overlaid. The pixel buffer is computed on demand on the
       background worker and cached. */
    drawWin(g, W_SurfaceMap, [&] {
        // Body to map: item 0 = "active ship's body" (surfmap.body == nullptr, so the
        // map follows the ship).
        // This list is rebuilt every frame the window is open, so the vector is
        // static (reserved once) and the item-0 label is a static buffer: the only
        // long string here was that label, and the other 18 push_backs each grew
        // two vectors from scratch. sm_active is static too so sm_items never
        // outlives the bytes it points at, even though Combo consumes it in place.
        static std::vector<const char *> sm_items;
        static char sm_active[160];
        if(ship && ship->m_parent) {
            snprintf(sm_active, sizeof(sm_active), "active ship's body (%s)",
                     ship->m_parent->name.c_str());
        } else {
            snprintf(sm_active, sizeof(sm_active), "active ship's body");
        }
        int sm_sel = 0;
        sm_items.clear();
        sm_items.reserve(sys.bodies.size() + 1);
        sm_items.push_back(sm_active);
        for(auto *b : sys.bodies) {
            sm_items.push_back(b->name.c_str());
            if(g.surfmap.body == b) { sm_sel = (int)sm_items.size() - 1; }
        }
        uiCombo("Body", &sm_sel, sm_items.data(), (int)sm_items.size());
        g.surfmap.body = (sm_sel > 0 && sm_sel <= (int)sys.bodies.size())
            ? sys.bodies[sm_sel - 1]
            : nullptr;

        TerrainBody *sm_body = g.surfmap.body
            ? g.surfmap.body
            : (ship && ship->m_parent ? ship->m_parent : sys.home);
        if(sm_body == nullptr) {
            ImGui::Text("No body to map (no ship, no system bodies).");
            return;
        }

        // Pan/zoom view of the map (the orbital map's idiom): sm_scale = 1 fits the
        // viewport, >1 zooms in; sm_pan offsets the image's top-left from the
        // viewport's. Persists across body switches; Reset view clears it.
        static ImVec2 sm_pan = ImVec2(0.0f, 0.0f);
        static float sm_scale = 1.0f;

        // Auto-compute when there is no map yet or it was computed for a different body.
        // Not mid-sweep: the last one lands for the body it was posted for.
        if(g.surfmap.in_flight == 0 &&
           (!g.surfmap.valid || g.surfmap.body_name != sm_body->name)) {
            surfmapCompute(g);
        }

        // Read AFTER the auto-compute above so a just-posted job shows the state.
        const bool sm_busy = g.surfmap.in_flight > 0;

        if(sm_busy) { ImGui::BeginDisabled(); }
        if(ImGui::Button("Refresh  (M)")) {
            surfmapCompute(g);
        }
        if(sm_busy) { ImGui::EndDisabled(); }
        ImGui::SameLine();
        if(ImGui::Button("Reset view")) {
            sm_pan = ImVec2(0.0f, 0.0f);
            sm_scale = 1.0f;
        }
        // Overlay-only: the terminator is pre-baked as a separate texture, so
        // toggling it is a free show/hide (no re-bake, no need to disable mid-sweep).
        bool shade = g.surfmap.shade;
        if(ImGui::Checkbox("Sun shading", &shade)) { g.surfmap.shade = shade; }
        ImGui::SameLine();
        bool sea = g.surfmap.sea;
        if(sm_busy) { ImGui::BeginDisabled(); }
        if(ImGui::Checkbox("Ocean", &sea)) {
            g.surfmap.sea = sea;
            surfmapCompute(g);   // re-bake with / without the sea
        }
        if(sm_busy) { ImGui::EndDisabled(); }
        ImGui::SameLine();
        // Overlay-only: no re-bake, so no need to disable it mid-sweep.
        bool sml = g.surfmap.site_labels;
        if(ImGui::Checkbox("Site labels", &sml)) { g.surfmap.site_labels = sml; }
        if(sm_busy) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.4f, 1.0f),
                               "   mapping ...");
        }

        if(!g.surfmap.valid || g.surfmap.px.empty()) {
            if(sm_busy) {
                ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.4f, 1.0f),
                                   "Mapping the surface ...");
                ImGui::TextDisabled("The map appears here when it lands.");
            } else {
                ImGui::Text("No map yet: press Refresh (or M).");
            }
            return;
        }

        // LINEAR filtering: a smooth map, unlike the Porkchop's discrete cells.
        // Base albedo + terminator overlay are published together (one rev).
        static Texture *sm_tex = nullptr;
        static int sm_tex_w = 0, sm_tex_h = 0, sm_tex_rev = -1;
        if(g.surfmap.rev != sm_tex_rev) {
            if(!sm_tex || sm_tex_w != g.surfmap.w || sm_tex_h != g.surfmap.h) {
                if(sm_tex) { delete sm_tex; }
                sm_tex = make_texture_r8(g.surfmap.w, g.surfmap.h,
                                        g.surfmap.px.data(), /*linear=*/true);
                sm_tex_w = g.surfmap.w;
                sm_tex_h = g.surfmap.h;
            } else {
                upload_texture_r8(sm_tex, g.surfmap.w, g.surfmap.h,
                                  g.surfmap.px.data());
            }
            sm_tex_rev = g.surfmap.rev;
        }
        static Texture *sm_shade_tex = nullptr;
        static int sm_shade_tex_w = 0, sm_shade_tex_h = 0, sm_shade_tex_rev = -1;
        if(g.surfmap.rev != sm_shade_tex_rev) {
            if(!sm_shade_tex || sm_shade_tex_w != g.surfmap.w
               || sm_shade_tex_h != g.surfmap.h) {
                if(sm_shade_tex) { delete sm_shade_tex; }
                sm_shade_tex = make_texture_r8(g.surfmap.w, g.surfmap.h,
                                              g.surfmap.shade_px.data(),
                                              /*linear=*/true);
                sm_shade_tex_w = g.surfmap.w;
                sm_shade_tex_h = g.surfmap.h;
            } else {
                upload_texture_r8(sm_shade_tex, g.surfmap.w, g.surfmap.h,
                                  g.surfmap.shade_px.data());
            }
            sm_shade_tex_rev = g.surfmap.rev;
        }

        // The map fills the window's content width (2:1 equirectangular). Buffer row 0 is
        // the north pole and GL row 0 the drawn rect's top, so the default uv draws it
        // unflipped (the Porkchop heatmap flips).
        // ImMax: this imgui's GetContentRegionAvail is not clamped to 0.
        const float sm_img_w = ImMax(0.0f, ImGui::GetContentRegionAvail().x);
        const float sm_img_h = sm_img_w * 0.5f;
        const ImVec2 sm_p0 = ImGui::GetCursorScreenPos();
        // The viewport is the scale-1 image rect; the zoomed image is clipped to it.
        const ImVec2 sm_vp_max = ImVec2(sm_p0.x + sm_img_w, sm_p0.y + sm_img_h);
        const ImVec2 sm_img_min = ImVec2(sm_p0.x + sm_pan.x, sm_p0.y + sm_pan.y);
        const ImVec2 sm_img_max = ImVec2(sm_img_min.x + sm_img_w * sm_scale,
                                        sm_img_min.y + sm_img_h * sm_scale);

        ImDrawList *dl = ImGui::GetWindowDrawList();
        dl->PushClipRect(sm_p0, sm_vp_max, true);
        dl->AddImage((ImTextureID)(std::intptr_t)sm_tex->id,
                     sm_img_min, sm_img_max);
        // Terminator overlay: black with alpha = (1 - shade); the default
        // src-alpha-over blend multiplies the albedo by shade. Same zoom transform.
        if(g.surfmap.shade && sm_shade_tex) {
            dl->AddImage((ImTextureID)(std::intptr_t)sm_shade_tex->id,
                         sm_img_min, sm_img_max);
        }
        dl->PopClipRect();

        // An invisible button sized to the VIEWPORT (not the zoomed image) captures the
        // mouse, so a left-drag pans the map instead of moving the window.
        ImGui::InvisibleButton("##smnav", ImVec2(sm_img_w, sm_img_h));
        // Hover state of the button (must be read while it is the last item).
        const bool sm_hover = ImGui::IsItemHovered();
        const ImGuiIO &sm_io = ImGui::GetIO();
        if(sm_hover && sm_io.MouseWheel != 0.0f) {
            // Wheel zooms to the cursor (the map point under the mouse stays put).
            const float factor = (sm_io.MouseWheel > 0.0f) ? 0.8f : 1.25f;
            float new_scale = sm_scale * factor;
            if(new_scale < 1.0f) { new_scale = 1.0f; }
            if(new_scale > 8.0f) { new_scale = 8.0f; }
            const ImVec2 mouse = ImGui::GetMousePos();
            const float u = (mouse.x - sm_img_min.x) / sm_scale;
            const float v = (mouse.y - sm_img_min.y) / sm_scale;
            sm_pan.x = (mouse.x - u * new_scale) - sm_p0.x;
            sm_pan.y = (mouse.y - v * new_scale) - sm_p0.y;
            sm_scale = new_scale;
        }
        if(ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            sm_pan.x += sm_io.MouseDelta.x;
            sm_pan.y += sm_io.MouseDelta.y;
        }

        // map_px scales by the IMAGE size rather than equirectPixel's (h-1) texture rows
        // -- deliberate: continuous screen coords keep the ship dot and the hover inverse
        // consistent (they disagree with the colour sample by at most half a map row near
        // the poles). Zoom/pan fold in here so every overlay tracks the view.
        const ImVec4 sm_bg = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
        const ImU32 sm_ink = contrastingColor(sm_bg);
        const ImU32 sm_ship =
            ImGui::GetColorU32(ImVec4(0.20f, 0.80f, 0.40f, 1.0f));
        auto map_px = [&](double lon, double lat) {
            const double u = lon / (2.0 * std::numbers::pi) * (double)sm_img_w * sm_scale;
            const double v = (std::numbers::pi * 0.5 - lat) / std::numbers::pi * (double)sm_img_h * sm_scale;
            return ImVec2(sm_img_min.x + (float)u, sm_img_min.y + (float)v);
        };

        // Graticule + orbit extend beyond the viewport when zoomed: clip them.
        dl->PushClipRect(sm_p0, sm_vp_max, true);

        // Graticule: faint, under the orbit line.
        const ImU32 grat_faint =
            ImGui::GetColorU32(ImVec4(1.0f, 1.0f, 1.0f, 0.12f));
        const ImU32 grat_eq =
            ImGui::GetColorU32(ImVec4(1.0f, 1.0f, 1.0f, 0.25f));
        for(int deg = -60; deg <= 60; deg += 30) {
            const ImVec2 a = map_px(0.0, (double)deg * std::numbers::pi / 180.0);
            const ImVec2 b = map_px(2.0 * std::numbers::pi, (double)deg * std::numbers::pi / 180.0);
            dl->AddLine(a, b, deg == 0 ? grat_eq : grat_faint, 1.0f);
        }
        for(int deg = 0; deg < 360; deg += 45) {
            const double lon = (double)deg * std::numbers::pi / 180.0;
            const ImVec2 a = map_px(lon, std::numbers::pi * 0.5);
            const ImVec2 b = map_px(lon, -std::numbers::pi * 0.5);
            dl->AddLine(a, b, grat_faint, 1.0f);
        }

        // The ship's orbit, only when the ship orbits this body (a conic about another
        // body is meaningless here).
        if(ship && sm_body == ship->m_parent && g.hud.view.mu > 0.0) {
            const double &mu = g.hud.view.mu;
            const glm::dvec3 &orbit_pos = g.hud.view.orbit_pos;
            const glm::dvec3 &orbit_vel = g.hud.view.orbit_vel;
            // N matches the Orbital Map: the cache is keyed on N and both maps draw the same
            // ship in one frame.
            const int N = 64;
            const bool closed = (o.ecc < 1.0);
            std::vector<glm::dvec3> pts_local;
            const std::vector<glm::dvec3> *pts = nullptr;
            if(closed) {
                // Per-ship cache (shared with the Orbital Map), trusted only while on rails.
                pts = &orbit_caches[(const void *)ship].sample(
                    orbit_pos, orbit_vel, mu, N, ship->onRails);
            } else {
                // Open arc: the map spans the whole body, so cap it a couple of ship radii out.
                const double r_cap =
                    std::max(4.0 * o.periapsis, o.distance) * 2.0;
                pts_local = sampleOpenTrajectory(orbit_pos, orbit_vel, mu, N,
                                                 r_cap);
                pts = &pts_local;
            }
            if(pts && !pts->empty()) {
                // The points live in the ship's non-rotating frame, the map's pixels in the body's
                // ROTATING frame: transforming each point gives the orbit's GROUND TRACK.
                Frame *inertial = ship->frame->getNonRotFrame();
                Frame *rot = sm_body->frame->getRotFrame();
                const glm::dmat3 O = inertial->GetOrientRelTo(rot);
                const glm::dvec3 P = inertial->GetPositionRelTo(rot);
                // Point (inertial frame) -> (lon, lat, pixel); false if degenerate.
                auto to_px = [&](const glm::dvec3 &p, ImVec2 &px) -> bool {
                    const glm::dvec3 pr = O * p + P;
                    const double l = glm::length(pr);
                    if(l < 1e-9) { return false; }
                    double lon, lat;
                    equirectLonLat(pr / l, lon, lat);
                    px = map_px(lon, lat);
                    return true;
                };
                // Broken at the antimeridian: the map's edges are the SAME meridian; surfmapWraps
                // detects the >half-turn jump.
                double prev_lon = -1.0e300;
                // Reused buffer (as drawDebrisBelts' pi/po): this redraws
                // every frame, and a 64-point orbit grows a fresh vector each time.
                static std::vector<ImVec2> seg;
                seg.clear();
                seg.reserve(pts->size());
                for(size_t i = 0; i < pts->size(); i++) {
                    const glm::dvec3 pr = O * (*pts)[i] + P;
                    const double l = glm::length(pr);
                    if(l < 1e-9) { continue; }
                    double lon, lat;
                    equirectLonLat(pr / l, lon, lat);
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
                // Apsides (closed, non-circular): propagate to each exactly, as in the Orbital Map.
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

        dl->PopClipRect();

        /* StaticBuilding::dir is a unit up in the body's ROTATING frame -- the very frame
           the map's pixels are baked in (equirect.h) -- so a site's pixel is equirectLonLat
           straight, no transform. */
        struct SiteMark { ImVec2 px; const std::string *name; };
        // Reused: one entry per VAB pad on the mapped body, rebuilt every frame
        // the window is open. The name pointers are into the body's pads, which
        // outlive the frame.
        static std::vector<SiteMark> site_marks;
        site_marks.clear();
        site_marks.reserve(sm_body->pads.size());
        for(const StaticBuilding *p : sm_body->pads) {
            if(!p->hasFlag("VAB_LAUNCH_PAD")) { continue; }
            double lon, lat;
            equirectLonLat(p->dir, lon, lat);
            site_marks.push_back({ map_px(lon, lat), &p->display_name });
        }
        if(!site_marks.empty()) {
            const ImU32 site_fill =
                ImGui::GetColorU32(ImVec4(0.95f, 0.72f, 0.20f, 1.0f));
            const ImU32 site_edge =
                ImGui::GetColorU32(ImVec4(0.0f, 0.0f, 0.0f, 0.65f));
            const ImU32 site_text =
                ImGui::GetColorU32(ImVec4(1.0f, 0.92f, 0.65f, 1.0f));
            const float sr = 5.0f;
            const float sm_img_w_s = sm_img_w * sm_scale;
            dl->PushClipRect(sm_p0, sm_vp_max, true);
            for(const SiteMark &m : site_marks) {
                const ImVec2 a(m.px.x, m.px.y - sr);
                const ImVec2 b(m.px.x - sr, m.px.y + sr * 0.8f);
                const ImVec2 c(m.px.x + sr, m.px.y + sr * 0.8f);
                dl->AddTriangleFilled(a, b, c, site_fill);
                dl->AddTriangle(a, b, c, site_edge, 1.0f);
                /* A site within a glyph of an edge is half clipped, and the map's edges are the
                   SAME meridian, so the other half belongs on the far side. The seam is at the
                   zoomed image's left/right edges (sm_img_min/max). */
                if(m.px.x - sm_img_min.x < sr || sm_img_max.x - m.px.x < sr) {
                    const ImVec2 q(m.px.x < sm_img_min.x + sm_img_w_s * 0.5f
                                       ? m.px.x + sm_img_w_s : m.px.x - sm_img_w_s,
                                   m.px.y);
                    dl->AddTriangleFilled(ImVec2(q.x, q.y - sr),
                                          ImVec2(q.x - sr, q.y + sr * 0.8f),
                                          ImVec2(q.x + sr, q.y + sr * 0.8f),
                                          site_fill);
                    dl->AddTriangle(ImVec2(q.x, q.y - sr),
                                    ImVec2(q.x - sr, q.y + sr * 0.8f),
                                    ImVec2(q.x + sr, q.y + sr * 0.8f),
                                    site_edge, 1.0f);
                }
                if(!g.surfmap.site_labels) { continue; }
                const char *nm = m.name->c_str();
                const ImVec2 ts = ImGui::CalcTextSize(nm);
                // Right of the glyph; left of it rather than off the map.
                float tx = m.px.x + sr + 3.0f;
                if(tx + ts.x > sm_img_max.x) {
                    tx = m.px.x - sr - 3.0f - ts.x;
                }
                const float ty = m.px.y - ts.y * 0.5f;
                // AddText has no outline: a dark copy a pixel down-right keeps the label readable.
                dl->AddText(ImVec2(tx + 1.0f, ty + 1.0f), site_edge, nm);
                dl->AddText(ImVec2(tx, ty), site_text, nm);
            }
            dl->PopClipRect();
        }

        // The dot is the SUB-SATELLITE point -- the ship's COM in the body's ROTATING frame.
        // NOT ship->frame's origin: two frames of the same body share an origin, so the offset
        // is 0 and the dot would vanish on its own body.
        if(ship && sm_body == ship->m_parent) {
            Frame *rot = sm_body->frame->getRotFrame();
            const glm::dvec3 com = ship->get_center_of_mass();
            const glm::dvec3 sp = ship->frame->GetOrientRelTo(rot) * com
                                + ship->frame->GetPositionRelTo(rot);
            const double sl = glm::length(sp);
            if(sl > 1e-9) {
                double lon, lat;
                equirectLonLat(sp / sl, lon, lat);
                const ImVec2 p = map_px(lon, lat);
                // Crossing the antimeridian gets a twin on the other side; clip so neither spills.
                // The seam is at the zoomed image's left/right edges (sm_img_min/max).
                const bool near_left  = (p.x - sm_img_min.x) < 8.0f;
                const bool near_right = (sm_img_max.x - p.x) < 8.0f;
                dl->PushClipRect(sm_p0, sm_vp_max, true);
                dl->AddCircleFilled(p, 5.0f, sm_ink);
                dl->AddCircle(p, 8.0f, sm_ship, 0, 1.5f);
                if(near_left || near_right) {
                    const ImVec2 p2(near_left ? p.x + sm_img_w * sm_scale
                                             : p.x - sm_img_w * sm_scale,
                                    p.y);
                    dl->AddCircleFilled(p2, 5.0f, sm_ink);
                    dl->AddCircle(p2, 8.0f, sm_ship, 0, 1.5f);
                }
                dl->PopClipRect();
            }
        }

        // The lat/lon/elev readout is always on screen: this window auto-resizes, so a
        // line that appears and disappears would jitter its height. Off the map (no
        // hover, or panned past the edge) it reads dashes.
        const ImVec2 sm_mouse = ImGui::GetMousePos();
        const double sm_u = (sm_mouse.x - sm_img_min.x) / (sm_img_w * sm_scale);
        const double sm_v = (sm_mouse.y - sm_img_min.y) / (sm_img_h * sm_scale);
        const bool sm_on_map = sm_hover && sm_u >= 0.0 && sm_u <= 1.0
                            && sm_v >= 0.0 && sm_v <= 1.0;
        if(sm_on_map) {
            // The hovered pixel inverts map_px. The unit direction feeds
            // GetTerrainHeight straight -- the map is baked in the same rotating frame
            // (equirect.h). "elev" is above SEA LEVEL, like the Surface window's ASL.
            const double lon = sm_u * 2.0 * std::numbers::pi;
            const double lat = std::numbers::pi * 0.5 - sm_v * std::numbers::pi;
            const glm::vec3 dir = equirectDir(lon, lat);
            char elev_s[32];
            // The nearest site under the cursor, so a marker is still identifiable with the
            // labels box off. One suffix on the same line, never a second line.
            char site_s[64] = "";
            const SiteMark *near = nullptr;
            float best = 14.0f * 14.0f;
            for(const SiteMark &m : site_marks) {
                const float dx = m.px.x - sm_mouse.x;
                const float dy = m.px.y - sm_mouse.y;
                const float d2 = dx * dx + dy * dy;
                if(d2 < best) { best = d2; near = &m; }
            }
            if(near != nullptr) {
                snprintf(site_s, sizeof(site_s), "  site: %s",
                         near->name->c_str());
            }
            ImGui::TextDisabled("cursor: lat %+.1f  lon %.1f  elev %s%s",
                                glm::degrees(lat), glm::degrees(lon),
                                fmt_dist((double)sm_body->GetTerrainHeight(dir)
                                         - (double)sm_body->radius
                                         - (double)sm_body->surface.sea_level,
                                         elev_s, sizeof elev_s),
                                site_s);
        } else {
            ImGui::TextDisabled("cursor: lat --  lon --  elev --");
        }
    });

    drawWin(g, W_Debug, [&] {
        ImGui::Text("Time: %f", time);
        if(sys.home && sys.home->cal.valid()) {
            CalTime ct = sys.home->cal.at(time);
            if(ct.civil) {
                ImGui::Text("Clock:  %04d-%02d-%02d  %02d:%02d:%02d UTC",
                            ct.year, ct.month, ct.day, ct.hh, ct.mm, ct.ss);
            } else if(ct.has_year) {
                ImGui::Text("Clock:  Yr %d  Mo %d  Day %d  %02d:%02d:%02d  (%s time)",
                            ct.year, ct.month, ct.day, ct.hh, ct.mm, ct.ss,
                            sys.home->name.c_str());
            } else {
                ImGui::Text("Clock:  Day %d  %02d:%02d:%02d  (%s time)",
                            ct.day, ct.hh, ct.mm, ct.ss,
                            sys.home->name.c_str());
            }
            // Solar time at surface lon 0 (#202): the subsolar point is local noon, so lon 0
            // reads 12h - lon/15 (east-positive lon, #200). Geometric, not a mean-time table.
            if(g.sun && g.sun != sys.home && sys.home->frame) {
                const glm::dvec3 to_sun = g.sun->frame->root_pos
                                        - sys.home->frame->root_pos;
                const double sl = glm::length(to_sun);
                if(sl > 1e-9) {
                    const glm::dmat3 to_rot = glm::transpose(
                        sys.home->frame->getRotFrame()->root_orient);
                    const glm::dvec3 dir = to_rot * (to_sun / sl);
                    double lon = 0.0, lat = 0.0;
                    equirectLonLat(dir, lon, lat);
                    double sod = 12.0 - glm::degrees(lon) / 15.0;
                    while(sod < 0.0) { sod += 24.0; }
                    while(sod >= 24.0) { sod -= 24.0; }
                    int ssec = (int)std::lround(sod * 3600.0);
                    if(ssec >= 86400) { ssec = 86399; }
                    ImGui::Text("Solar:  %02d:%02d:%02d  (at lon 0)",
                                ssec / 3600, (ssec / 60) % 60, ssec % 60);
                }
            }
        }
        // Local date + time on the body the ship is in, when not the home planet.
        TerrainBody *local_body = (ship && ship->frame)
                                ? ship->frame->body : nullptr;
        if(local_body && local_body != sys.home &&
           local_body->cal.valid()) {
            CalTime lt = local_body->cal.at(time);
            if(lt.civil) {
                ImGui::Text("Local:  %s  %04d-%02d-%02d  %02d:%02d:%02d",
                            local_body->name.c_str(),
                            lt.year, lt.month, lt.day, lt.hh, lt.mm, lt.ss);
            } else if(lt.has_year) {
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
        ImGui::Text("Camera ASL: %0.f", glm::length(camera->GetPos())
                    - (ship->m_parent->radius + ship->m_parent->surface.sea_level));
        ImGui::Text("Camera Pos: %.0f %.0f %0.f", camera->GetPos().x, camera->GetPos().y, camera->GetPos().z);
        ImGui::Text("Cam forward: %.2f %.2f %.2f",
                    camera->forward.x, camera->forward.y, camera->forward.z);
        if(ImGui::Button("Print camera pose (CLI args)")) {
            // Copy-paste the printed line after ./osp to relaunch at this view.
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
        char home_d[32], pos_d[32];
        ImGui::Text("Home distance: %s",
                    fmt_dist(glm::length(ship->GetPositionRelTo(ship->controller,
                                                       ship->home->frame)),
                             home_d, sizeof home_d));
        ImGui::Text("Pos: %s", fmt_dist(distance, pos_d, sizeof pos_d));
        ImGui::Text("xyz(%0.f, %0.f, %0.f)", pos.x, pos.y, pos.z);
        ImGui::Text("Vel: %.3fm/s", speed);
        ImGui::Text("xyz(%0.f, %0.f, %0.f)",
                    marker_vel.x, marker_vel.y, marker_vel.z);
        /* Specific orbital energy, J/kg -- the parent's mu is part of the number, so name
           the body (it jumps at an SOI handoff). Redundant with Orbit Info's SMa
           (energy = -mu/2a), hence debug-only: as raw state it is a live integrator check,
           though drag and the per-part gravity-gradient exchange move it too. */
        ImGui::Text("Orb energy (%s): %.6g J/kg", ship->m_parent->name.c_str(),
                    o.energy);

        // --- power balance (the electrical system) -------------------------
        double gen = 0.0, constDraw = 0.0, charge = 0.0, capacity = 0.0;
        ship->getPower(&gen, &constDraw, &charge, &capacity);
        if(gen > 0.0 || constDraw > 0.0 || capacity > 0.0) {
            // Active draw: the wheels, only while commanding (same condition powerTick
            // uses).
            bool wheelsActive = (ship->stick[0] != 0.0f ||
                                 ship->stick[1] != 0.0f ||
                                 ship->stick[2] != 0.0f)
                || ship->slew != SlewNone;
            double activeDraw = 0.0;
            if(wheelsActive) {
                for(Part *p : ship->parts) {
                    if(p->isWheel()) { activeDraw += p->powerDraw(); }
                }
            }
            const double net = gen - constDraw - activeDraw;

            ImGui::Separator();
            ImGui::Text("Power");
            if(ship->powered_) {
                ImGui::TextColored(ImVec4(0.30f, 0.85f, 0.30f, 1.0f),
                                  "  Status:    POWERED");
            } else {
                ImGui::TextColored(ImVec4(0.90f, 0.30f, 0.30f, 1.0f),
                                  "  Status:    NO POWER (uncontrolled)");
            }
            ImGui::Text("  Net:       %+.1f W   (%s)",
                        net, net >= 0.0 ? "charging" : "draining");
            ImGui::Text("  Charge:    %.1f / %.1f Wh", charge, capacity);
            ImGui::Text("  Generation:  %.1f W", gen);
            ImGui::Text("  Const draw:  %.1f W   (life support)", constDraw);
            ImGui::Text("  Active:      %.1f W   (wheels, while commanding)",
                        activeDraw);
        }
    });

    // Labels abbreviated to <= 3 chars and right-padded so values line up.
    drawWin(g, W_Orbital, [&] {
        char dist_s[32];   // one buffer, reused line by line
        ImGui::Text("Bod: %s (%c)", ship->m_parent->name.c_str(),
                    ship->frame->isRotFrame() ? 'R' : 'I');
        ImGui::Text("Vel: %.1fm/s", speed);
        /* R is a RADIUS from the focus, not an altitude; ApA/PeA are apsis RADII for the
           same reason. The SURFACE window has the real altitude (calling these "Alt" was
           #171's sibling complaint). */
        ImGui::Text("  R: %s", fmt_dist(distance, dist_s, sizeof dist_s));
        /* Every line below is always present; "-" = the quantity does not exist for
           this orbit class. */
        const bool circular = o.ecc < 1.0
                            && (o.apoapsis - o.periapsis) < 10e3;
        if(o.ecc < 1.0) { ImGui::Text("ApA: %s", fmt_dist(o.apoapsis, dist_s, sizeof dist_s)); }
        else { ImGui::Text("ApA: -"); }
        if(o.ecc < 1.0 && !circular) { ImGui::Text("ApT: %.1fs", o.time_to_apo); }
        else { ImGui::Text("ApT: -"); }
        ImGui::Text("PeA: %s", fmt_dist(o.periapsis, dist_s, sizeof dist_s));
        if(!circular && o.time_to_peri >= 0.0) { ImGui::Text("PeT: %.1fs", o.time_to_peri); }
        else { ImGui::Text("PeT: -"); }
        if(o.period > 0.0) { ImGui::Text("  T: %.1fs", o.period); }
        else { ImGui::Text("  T: -"); }
        /* Plane angles, measured in the plane the orbit map is showing -- the label names it,
           because the number is meaningless without it (#171). In the Orbital view the plane
           IS the orbit: Inc reads 0 by construction, and LAN/LPe dash (no node, no chosen
           zero longitude). */
        ImGui::Text("Inc: %.2f (%s)", glm::degrees(view.plane.inc),
                    refPlaneName(view.ref_plane.mode));
        ImGui::Text("Ecc: %f", o.ecc);
        ImGui::Text("SMa: %s", fmt_dist(o.semi_major, dist_s, sizeof dist_s));
        // LAN needs both a node line and a chosen zero longitude: the Target-orbit
        // slot has the former and not the latter.
        if(view.plane.node_ok && view.plane.lon_ok) { ImGui::Text("LAN: %.2f", glm::degrees(view.plane.lan)); }
        else { ImGui::Text("LAN: -"); }
        if(view.plane.lpe_ok) { ImGui::Text("LPe: %.2f", glm::degrees(view.plane.lpe)); }
        else { ImGui::Text("LPe: -"); }
        double prograde_angle = glm::angle(facing_dir, vel_dir);
        double retrograde_angle = glm::angle(facing_dir, - vel_dir);
        ImGui::Text("Prg: %.2f", glm::degrees(prograde_angle));
        ImGui::Text("Rtg: %.2f", glm::degrees(retrograde_angle));
    });

    // Initial size comes from o_telemetry.initial_size: content-fit would clip a 2x2 grid.
    drawWin(g, W_Telemetry, [&] {
        // Cells positioned explicitly (SetCursorPos) so the grid stays a clean 2x2.
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
    drawWin(g, W_Surface, [&] {
        char dist_s[32];
        const TerrainBody *b = ship->m_parent;
        const double altAsl = distance - ((double)b->radius
                                          + (double)b->surface.sea_level);
        // Bme/Sit: the science identity of this pose (game.h poseSituation). "-" when there
        // is no biome to name.
        const PoseSituation ps = poseSituation(
            b, glm::normalize(glm::vec3(view.surf_pos)), altAsl, ship->isGrounded());
        ImGui::Text("Bme: %s", ps.biome != Biome::None
                                   ? capitalizeFirst(biomeName(ps.biome)).c_str()
                                   : "-");
        ImGui::Text("Sit: %s", capitalizeFirst(situationName(ps.situation)).c_str());
        ImGui::Text("Alt: %s", fmt_dist(distance - b->GetTerrainHeight(glm::normalize(pos)), dist_s, sizeof dist_s));
        ImGui::Text("ASL: %s", fmt_dist(altAsl, dist_s, sizeof dist_s));
        ImGui::Text(" Vs: %.2fm/s", ver_speed);
        ImGui::Text(" Hs: %.2fm/s", hor_speed2);
        ImGui::Text("Lat: %.4f", glm::degrees(latitude));
        ImGui::Text("Lon: %.4f", glm::degrees(longitude));
        ImGui::Text(" Pt: %.2f", glm::degrees(pitch));
        ImGui::Text("  R: %.2f", glm::degrees(roll));
        ImGui::Text("Hdg: %.2f", glm::degrees(yaw));
        ImGui::Text("Acc: %.1fm/s2", ship->feltAccel());
    });

    /* --info-log: the quantities ORBITAL and SURFACE display plus the energy the
       DEBUG window shows, raw SI (the instrument-log convention in fmt.h that e2e
       CHECK parses). Independent of whether the windows are open. */
    if(args.info_log && ship) {
        const Uint32 now_ms = SDL_GetTicks();
        if(g.logDue(kGateInfo, now_ms, g.orbit_log_interval_ms)) {
            const TerrainBody *b = ship->m_parent;
            const double altAsl = distance - ((double)b->radius
                                          + (double)b->surface.sea_level);
            const PoseSituation ps = poseSituation(
                b, glm::normalize(glm::vec3(view.surf_pos)), altAsl,
                ship->isGrounded());
            const char *bme = (ps.biome != Biome::None)
                ? biomeName(ps.biome) : "-";
            const double prograde_angle = glm::angle(facing_dir, vel_dir);
            const double retrograde_angle = glm::angle(facing_dir, -vel_dir);
            // lan/lpe in the map's plane, labelled with it (#171).
            char lan_s[32], lpe_s[32];
            if(view.plane.node_ok && view.plane.lon_ok) { snprintf(lan_s, sizeof lan_s, "%.6g deg", glm::degrees(view.plane.lan)); }
            else { snprintf(lan_s, sizeof lan_s, "-"); }
            if(view.plane.lpe_ok) { snprintf(lpe_s, sizeof lpe_s, "%.6g deg", glm::degrees(view.plane.lpe)); }
            else { snprintf(lpe_s, sizeof lpe_s, "-"); }
            // r/apo_r/peri_r are RADII from the focus, not altitudes.
            printf("[orbinfo] t=%.1fs body=\"%s\" vel=%.6g m/s r=%.6g m "
                   "apo_r=%.6g m apo_t=%.6g s peri_r=%.6g m peri_t=%.6g s "
                   "period=%.6g s inc=%.6g deg ecc=%.6g sma=%.6g m "
                   "plane=%s lan=%s lpe=%s prg=%.6g deg rtg=%.6g deg "
                   "energy=%.6g J/kg\n",
                   time, b->name.c_str(), speed, distance,
                   o.apoapsis, o.time_to_apo, o.periapsis, o.time_to_peri,
                   o.period, glm::degrees(view.plane.inc), o.ecc, o.semi_major,
                   refPlaneName(view.ref_plane.mode), lan_s, lpe_s,
                   glm::degrees(prograde_angle),
                   glm::degrees(retrograde_angle),
                   o.energy);
            printf("[surfinfo] t=%.1fs "
                   "alt_agl=%.6g m alt_asl=%.6g m "
                   "vs=%.6g m/s hs=%.6g m/s lat=%.8g deg lon=%.8g deg "
                   "pitch=%.6g deg roll=%.6g deg hdg=%.6g deg "
                   "acc=%.6g m/s2 body=\"%s\" bme=\"%s\" sit=\"%s\"\n",
                   time,
                   distance - b->GetTerrainHeight(glm::normalize(pos)),
                   altAsl,
                   ver_speed, hor_speed2,
                   glm::degrees(latitude), glm::degrees(longitude),
                   glm::degrees(pitch), glm::degrees(roll), glm::degrees(yaw),
                   ship->feltAccel(), b->name.c_str(), bme,
                   situationName(ps.situation));
            fflush(stdout);
        }
    }

    drawWin(g, W_ShipList, [&] {
    // Buttons (natural width) + SameLine: a full-width Selectable in this auto-resize
    // window would push the "x" off the line.
    /* Walked by index because a row's "x" erases from the live list. The loop
       stops the moment a row really goes away (see `removed` below), so no index
       is reused after the erase. */
    bool removed = false;
    for(size_t i = 0; i < vehicles.size() && !removed; i++) {
        Vehicle *v = vehicles[i];
        const bool active = (v == ship);
        ImGui::PushID((void*)v);
        /* Every row is a controllable vessel: a seated kerbal is a seat on its
           capsule's Part (part.h), not a vehicle. */
        if(active) {
            ImGui::PushStyleColor(ImGuiCol_Button,
                                 ImVec4(0.30f, 0.45f, 0.70f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                                 ImVec4(0.35f, 0.50f, 0.75f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                                 ImVec4(0.40f, 0.55f, 0.80f, 1.0f));
        }
        if(ImGui::Button(v->name.c_str())) {
            g.select_vehicle(v);
        }
        if(active) {
            ImGui::PopStyleColor(3);
        }
        // A walking kerbal is selectable but not deletable
        // (ui_remove_vehicle refuses it): no "x".
        if(!v->isEva()) {
            ImGui::SameLine();
            // The "##rm_<name>" suffix is invisible but makes the row addressable by --ui-click.
            // Names only ever grow by decoupler labels and a " #N" dedup suffix,
            // so this never truncates; if it somehow did, two rows would share an
            // imgui id and --ui-click would report the label as ambiguous.
            char rm_id[256];
            snprintf(rm_id, sizeof(rm_id), "x##rm_%s", v->name.c_str());
            if(ImGui::SmallButton(rm_id)) {
                /* Stop iterating only when a row really went away. A refusal
                   (a capsule with its own crew aboard still gets an "x")
                   toasts and leaves the list intact, so breaking there
                   would drop the rest of the list for one frame. */
                removed = g.ui_remove_vehicle(v);
            }
        }
        ImGui::PopID();
    }
    ImGui::Separator();
    ImGui::Text("click name - select    x - remove (not crew)");
    });

    drawWin(g, W_VesselInfo, [&] {
        ImGui::Text("Ship: %s", ship->name.c_str());
        ImGui::Text("Stage: %d / %d  (SPACE to drop)",
                    ship->activeStage(), ship->numStages());
        ImGui::Text("Mass: %.3fkg", ship->getMass());
        ImGui::Text("Delta-v: %.1fm/s", ship->getDeltaV());
        /* "Throttle", not "Thrust Util": the engine command value R/F ramp (0..1), not a
           utilisation ratio. */
        ImGui::Text("Throttle: %.0f%%", ship->thruster_util * 100);
        ImGui::Text("Thrust: %.2fN", ship->getThrust());
        ImGui::Text("Current TWR: %.2f/%.2f", ship->getTWR(), ship->getFullThrustTWR());
        ImGui::Text("Max TWR: %.2f", ship->getMaxTWR());
        ImGui::Text("Wheel torque: %.0fN m", ship->GetWheelTorque());
        ImGui::Text("Angular rate: %.2fdeg/s",
                    glm::degrees(glm::length(ship->partAngVel(ship->controller))));
    });
    drawWin(g, W_Controls, [&] {
        ImGui::Text("Click rebind, then press a key (or Shift/Ctrl/Alt + key) to bind it.");
        ImGui::Spacing();
        char buf[80];
        const char *groupNames[(int)SlotGroup::GROUP_COUNT] = {
            "Game (one-shot)",
            "Flight (piloting)",
            "Camera (free mode)",
            "EVA (the kerbal)",
        };
        for(int gi = 0; gi < (int)SlotGroup::GROUP_COUNT; gi++) {
            const SlotGroup grp = (SlotGroup)gi;
            ImGui::Text("%s", groupNames[gi]);
            ImGui::Separator();
            for(size_t i = 0; i < (size_t)Slot::SLOT_COUNT; i++) {
                if(slotGroup((Slot)i) != grp) { continue; }
                // Unique per-row ID: the "rebind"/"clear" labels repeat on every row.
                ImGui::PushID((int)i);
                const std::vector<KeyBind> &v = g.binds.perSlot[i];
                ImGui::AlignTextToFramePadding();
                ImGui::Text("%s", slotLabel((Slot)i));
                ImGui::SameLine(215.0f);
                snprintf(buf, sizeof buf, "%s",
                         v.empty() ? "(unbound)" : bindLabel(v[0]).c_str());
                ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.85f, 1.0f), "%s", buf);
                ImGui::SameLine(365.0f);
                const bool capturing = (g.rebind_capture_slot == (int)i);
                if(capturing) {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 0.6f, 0.2f, 1.0f));
                    if(ImGui::Button("press a key...", ImVec2(120.0f, 0.0f))) {
                        g.rebind_capture_slot = -1;   // click again to cancel
                    }
                    ImGui::PopStyleColor();
                } else {
                    if(ImGui::Button("rebind", ImVec2(120.0f, 0.0f))) {
                        g.rebind_capture_slot = (int)i;
                    }
                }
                ImGui::SameLine(0.0f, 6.0f);
                if(ImGui::Button("clear", ImVec2(60.0f, 0.0f))) {
                    g.binds.perSlot[i].clear();
                }
                ImGui::PopID();
            }
        }
        ImGui::Spacing();
        if(ImGui::Button("Reset all to defaults", ImVec2(240.0f, 0.0f))) {
            g.binds.resetDefaults();
            g.rebind_capture_slot = -1;
        }
        ImGui::SameLine();
        if(ImGui::Button("Save", ImVec2(90.0f, 0.0f))) {
            if(g.save_settings()) {
                g.toast("Settings saved (settings.json)");
            } else {
                g.toast("Could not write settings.json");
            }
        }
        ImGui::Spacing();
        if(ImGui::Button("Back", ImVec2(240.0f, 0.0f))) {
            setWinOpen(W_Controls, false);
            g.rebind_capture_slot = -1;
        }
    });

    drawWin(g, W_Autopilot, [&] {
        // Click a mode to engage, click again to release; the modes are mutually exclusive.
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
                /* Toast it: an engaged autopilot keeps turning the ship with no key held, so it must
                   be visible outside this window (Z/B engage the same state). */
                g.toast("Autopilot: %s %s", label, engaged ? "off" : "engaged");
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

    drawWin(g, W_Resources, [&] {
        // Aggregate across the active ship's parts; only types with capacity show.
        static const char *resNames[(int)ResourceType::Num] = {
            "Hydrogen", "LOX", "Electric charge", "Oxygen", "Water", "Food",
            "Hydrazine", "Jet fuel"
        };
        // Display order: fuels first, then life support (independent of enum order).
        static const ResourceType resOrder[] = {
            ResourceType::Hydrogen, ResourceType::LOX, ResourceType::JetFuel,
            ResourceType::Hydrazine, ResourceType::EC, ResourceType::Oxygen,
            ResourceType::Water, ResourceType::Food
        };
        for(ResourceType r : resOrder) {
            float cur = 0, cap = 0;
            for(Part *p : ship->parts) {
                cur += p->resources.current[(int)r];
                cap += p->resources.capacity[(int)r];
            }
            if(cap <= 0) { continue; }
            ImGui::ProgressBar(cur / cap, ImVec2(-1, 0), resNames[(int)r]);
        }
    });
}

/* One plain imgui window per part right-clicked in the 3D view (g.part_sels, opened
   by pickAt). Plain Begin/End -- user-placed popups, NOT slot-layout windows. Staging
   that drops the part leaves the entry stale (dropped, not re-pointed). */
void drawPartWindows(Game &g) {
    std::vector<Vehicle *> &vehicles = g.vehicles;
    for(int i = (int)g.part_sels.size() - 1; i >= 0; i--) {
        const size_t idx = (size_t)i;
        PartSel &live = g.part_sels[idx];
        Vehicle *ship = live.ship;
        if(live.part >= ship->parts.size()) {
            g.part_sels.erase(g.part_sels.begin() + idx);
            continue;
        }
        const size_t part = live.part;
        const PartDef *def = ship->parts[part]->def;

        // Catalog display_name; falls back to the machine id for older catalogs.
        const char *disp = def->label().c_str();

        /* The "##suffix" is a hidden ImGui window id, unique per (ship, part).
           It used to carry the Vehicle pointer, so a part window's registered
           name differed on every run: --ui-list could not be diffed and
           --ui-click had to fall back to a bare "Board" label, which any other
           window could satisfy instead. The visible prefix names the ship, and
           every path that SPAWNS a ship dedups its name (dedupVehicleName,
           vehicle.cpp:418), so the part index alone is unique.

           What is NOT enforced is name uniqueness on LOAD: the loader assigns
           the saved name verbatim (save.cpp:696) with no dedup, even though the
           same layer rejects duplicate roster names and part uids. A
           hand-authored save with two ships sharing a name (the
           save_roster_named fixture, case 212) therefore merges their part
           windows into one -- Begin(name) hashes the name, so the second ship's
           body lands inside the first window. Unreachable in play, since the
           game never writes such a save; #315. */
        char name[256];
        snprintf(name, sizeof(name), "%s > %s##%zu",
                 ship->name.c_str(), disp, part);
        /* Placement, every frame rather than once. Two reasons it cannot be a
           one-off:
             - the window's size is unknown until imgui has finished auto-fitting
               it (AutoFitFrames counts down from 2), so the first frames place
               against an estimate;
             - the content CHANGES after that: a Board row replaces "(no one in
               reach to board or store)", a Store row appears under it, the suit
               window grows a line per finding it carries. The window tracks its
               content (AlwaysAutoResize at the Begin below), so growing is
               exactly what it does -- and a window that was clear of its
               neighbour can grow into it, or off the bottom edge.
           Measured at 1080p: a 1-seat capsule window is ~284x300, a 6-seat one
           ~284x480, a materials pod ~280x210 -- so the estimate is the middle
           case, not the worst case; the real size takes over as soon as it is
           known, which is what keeps a 480-tall window safe without shoving
           every small window away from the mouse.
           A window whose Pos is not where we asked for it is the player's own
           (we only ever ask for askX/askY), and we stop moving it. */
        ImGuiWindow *win = ImGui::FindWindowByName(name);
        const bool sized = win != nullptr && win->AutoFitFramesX <= 0
                        && win->AutoFitFramesY <= 0;
        if(sized) {
            live.winW = win->Size.x;
            live.winH = win->Size.y;
        }
        const float estw = 300.0f, esth = 300.0f, gap = 12.0f, step = 96.0f;
        const float resw = live.winW > 0 ? live.winW : estw;
        const float resh = live.winH > 0 ? live.winH : esth;
        const ImGuiViewport *vp = ImGui::GetMainViewport();
        const float xlo = (float)vp->WorkPos.x, ylo = (float)vp->WorkPos.y;
        const float xhiW = xlo + (float)vp->WorkSize.x;
        const float yhiW = ylo + (float)vp->WorkSize.y;
        /* Adjacent parts pick a few tens of pixels apart, so a second part
           window lands almost exactly on the first and buries its buttons --
           the player cannot see them, let alone press them. Try the asked-for
           spot, then steps away from it, and take the first that clears every
           other part window. Each other window is measured against where it
           WILL be this frame (the spot we asked for, not last frame's Pos): two
           windows resolving in the same frame must not step into each other's
           move, which is how a one-pass layout oscillates. */
        struct Rect { float x, y, w, h; };
        /* Collected once per window per frame, not inside the candidate loop: a
           user-moved window's rect costs a name rebuild plus a
           FindWindowByName, and there are 12 candidates to test. Reused buffer --
           this pass runs every frame. */
        static std::vector<Rect> others;
        others.clear();
        for(const PartSel &other : g.part_sels) {
            if(&other == &live || !other.placed
               || other.part >= other.ship->parts.size()) { continue; }
            Rect r = {other.askX, other.askY,
                      other.winW > 0 ? other.winW : estw,
                      other.winH > 0 ? other.winH : esth};
            if(other.userMoved) {
                char oname[256];
                snprintf(oname, sizeof(oname), "%s > %s##%zu",
                         other.ship->name.c_str(),
                         other.ship->parts[other.part]->def->label().c_str(),
                         other.part);
                const ImGuiWindow *w = ImGui::FindWindowByName(oname);
                if(w == nullptr) { continue; }
                r = {w->Pos.x, w->Pos.y, w->Size.x, w->Size.y};
            }
            others.push_back(r);
        }
        auto clashes = [&](float cx, float cy, float cw, float ch) {
            for(const Rect &o : others) {
                if(cx < o.x + o.w && cx + cw > o.x && cy < o.y + o.h && cy + ch > o.y) {
                    return true;
                }
            }
            return false;
        };
        // Right first (the screen is wide and a part window is tall), then down,
        // then left. Falls through on the last candidate if none is clear.
        static const int kx[] = {0, 1, 2, 3, 4, 5, -1, -2, 0, 0, 0, 0};
        static const int ky[] = {0, 0, 0, 0, 0, 0,  0,  0, 1, 2, 3,-1};
        auto place = [&](float x0, float y0, float cw, float ch,
                         float &ox, float &oy) {
            /* A viewport smaller than the window leaves an empty clamp range
               (hi < lo is UB, and silently yields a negative coordinate), so keep
               the range non-empty: such a window just sits at the work origin. */
            const float xhi = std::max(xlo, xhiW - cw);
            const float yhi = std::max(ylo, yhiW - ch);
            ox = x0; oy = y0;
            for(int t = 0; t < 12; t++) {
                ox = std::clamp(x0 + (float)kx[t] * step, xlo, xhi);
                oy = std::clamp(y0 + (float)ky[t] * step, ylo, yhi);
                if(!clashes(ox, oy, cw, ch)) { return; }
            }
        };
        if(!live.placed) {
            // Open near the mouse (the part was picked there), flipping sides when there is no room.
            float x = (float)live.mx + gap;
            float y = (float)live.my + gap;
            if(x + resw > xhiW) { x = (float)live.mx - gap - resw; }
            if(y + resh > yhiW) { y = (float)live.my - gap - resh; }
            float px, py;
            place(std::max(x, xlo), std::max(y, ylo), resw, resh, px, py);
            ImGui::SetNextWindowPos(ImVec2(px, py), ImGuiCond_Appearing);
            live.askX = px;
            live.askY = py;
            live.placed = true;
        } else if(win != nullptr) {
            if(std::abs(win->Pos.x - live.askX) > 0.5f
               || std::abs(win->Pos.y - live.askY) > 0.5f) {
                live.userMoved = true;   // the player put it there; leave it be
            } else if(sized && (clashes(win->Pos.x, win->Pos.y, win->Size.x, win->Size.y)
                                || win->Pos.x + win->Size.x > xhiW
                                || win->Pos.y + win->Size.y > yhiW)) {
                // It grew into a neighbour, or hangs off the viewport: move it.
                float px, py;
                place(win->Pos.x, win->Pos.y, win->Size.x, win->Size.y, px, py);
                ImGui::SetNextWindowPos(ImVec2(px, py), ImGuiCond_Always);
                live.askX = px;
                live.askY = py;
            }
        }
        /* From here the body can fire a button that deletes a vehicle, and
           dropVehicleRefs -> dropPartWindowsFor erase_from part_sels: a reference
           into the vector would then alias whatever shifted into this slot, and
           the rest of the body (sel.point, and the next iteration's index) would
           read the wrong window. The layout fields above are the only fields
           this loop writes, and they are all written before this copy, so the
           body works from a copy. */
        const PartSel sel = live;
        bool open = true;
            /* A successful pickup may have dropped an EARLIER part_sels entry, shifting what
               this loop indexes. */
            bool worldChanged = false;
        /* AlwaysAutoResize is not cosmetic. Without it imgui auto-fits a window
           for the two frames after it opens and then FREEZES the size
           (AutoFitFrames is set once, at creation -- imgui.cpp:6746). The content
           here outgrows that first fit: a Board row for a kerbal who walks into
           reach, a Store row under it, one line per finding a suit carries. The
           overflow is not a scrollbar the player could drag -- imgui does not
           register an item that fails its clip test, so a button pushed past the
           frozen bottom simply stops existing: --ui-click reports it as "no such
           item drawn", and a player never sees it at all. Measured: the 6-seat
           capsule window freezes at 284x479 with 443 of content; six carried
           findings make the content 551 and the bottom button undrawable.
           The cost is no resize grip on these popups (imgui disables it for
           auto-resizing windows); the layout pass above is what positions them. */
        if(ImGui::Begin(name, &open, ImGuiWindowFlags_NoSavedSettings
                                   | ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("Ship: %s", ship->name.c_str());
            ImGui::Text("Part #%zu  (stage %d)", part,
                        ship->parts[part]->stage);
            ImGui::Separator();
            if(!def->display_name.empty()) {
                ImGui::Text("ID: %s", def->name.c_str());
            }
            if(!def->type.empty()) {
                ImGui::Text("Type: %s", def->type.c_str());
            }
            /* effectiveMass: a capsule with crew shows body + crew, matching the ship HUD. */
            ImGui::Text("Mass: %.3fkg", ship->parts[part]->effectiveMass());
            ImGui::Text("Size: %.1fm dia x %.1fm",
                        def->radius * 2.0, def->height);
            if(def->torque > 0.0) {
                ImGui::Text("Torque: %.0fN m (reaction wheel)", def->torque);
            }
            if(def->jet) {
                ImGui::Text("Jet: %.0fN static, air-breathing (%.1fkg/s jet fuel, intake %.2fm^2, %.0fm/s exhaust)",
                            def->jet_fan_thrust, def->propellant_rate[(int)ResourceType::JetFuel],
                            def->jet_intake_area, def->exhaust_velocity);
            } else if(def->totalPropellantRate() > 0.0 && def->exhaust_velocity > 0.0) {
                ImGui::Text("Thrust: %.0fN (%.1fkg/s @ %.0fm/s)",
                            def->fullThrust(), def->totalPropellantRate(),
                            def->exhaust_velocity);
            }
            static const char *resNames[(int)ResourceType::Num] = {
                "Hydrogen", "LOX", "EC", "Oxygen", "Water", "Food", "Hydrazine", "Jet fuel"
            };
            // EC is stored energy (watt-hours), not a substance: a battery drains under load
            // without losing mass.
            static const char *resUnits[(int)ResourceType::Num] = {
                "kg", "kg", "Wh", "kg", "kg", "kg", "kg", "kg"
            };
            for(int r = 0; r < (int)ResourceType::Num; r++) {
                if(def->capacity[(size_t)r] <= 0.0f) { continue; }
                ImGui::Text("%s: %.1f%s/%.1f%s", resNames[r],
                            ship->parts[part]->resources.current[r],
                            resUnits[r],
                            ship->parts[part]->resources.capacity[r],
                            resUnits[r]);
            }
            // --- docking port ------------------------------------------------
            // Docking is intent-driven (Game::updateDocking): the active ship docks only with
            // BOTH an armed port of its own AND a targeted port on another ship. The dock
            // consumes both, so undocking cannot immediately re-dock.
            if(def->docking_port) {
                ImGui::Separator();
                Vehicle *act = g.ship;
                if(act != nullptr && !act->isEva()) {
                    if(act != ship) {
                        // On another ship: make it the dock target (the other ship's half of the intent).
                        if(act->dockTargetPort == ship->parts[part]) {
                            ImGui::Text("Docking target of %s", act->name.c_str());
                            if(ImGui::SmallButton("Clear docking target")) {
                                act->dockTargetShip = nullptr;
                                act->dockTargetPort = nullptr;
                            }
                        } else if(ImGui::SmallButton("Target for docking")) {
                            act->dockTargetShip = ship;
                            act->dockTargetPort = ship->parts[part];
                            g.toast("Docking target: %s", ship->name.c_str());
                        }
                    } else {
                        // On the ACTIVE ship: arm it as the port that does the mating.
                        if(act->dockArmPort == ship->parts[part]) {
                            ImGui::Text("Armed port of %s", act->name.c_str());
                            if(ImGui::SmallButton("Disarm port")) {
                                act->dockArmPort = nullptr;
                            }
                        } else if(ImGui::SmallButton("Arm for docking")) {
                            act->dockArmPort = ship->parts[part];
                            g.toast("Armed port %zu", part);
                        }
                    }
                } else {
                    ImGui::Text("(switch to a controllable ship to use this port)");
                }
            }
            // --- crew (this part is a capsule) ---------------------------------
            // A seat gets an EVA button; a walking kerbal in boarding range gets Board.
            if(def->crew_capacity > 0) {
                ImGui::Separator();
                /* A seat is this capsule's occupant (part.h): a suit Part parked
                   in it, not a Kerbal vehicle. The snapshot of suits is safe
                   to keep iterating after a button fires -- an EVA hands the
                   suit to the kerbal it just minted, so it stays alive. */
                std::vector<Part *> seats;
                for(Part *suit : ship->parts[part]->seats) {
                    if(suit != nullptr) { seats.push_back(suit); }
                }
                ImGui::Text("Crew: %d / %d", (int)seats.size(), def->crew_capacity);
                /* Iterate the SLOTS, not the compacted list: the button ids below
                   are slot numbers, so a row keeps its id when a middle seat
                   empties. */
                char btn[32];   // slot ids only ("Exp##exp12"); name ids use std::string below
                for(size_t slot = 0; slot < ship->parts[part]->seats.size(); slot++) {
                    Part *suit = ship->parts[part]->seats[slot];
                    if(suit == nullptr) { continue; }
                    ImGui::PushID(suit);
                    ImGui::Text("  %s", seatName(suit).c_str());
                    /* The ## id is not decoration: PushID(suit) already gives each
                       row its own imgui id, but the test engine registers the
                       LABEL, so N seats registered N identical "EVA"s and no seat's
                       button could be addressed at all (#326). The suffix is hidden
                       on screen, and it names the slot a player reads top-down. */
                    snprintf(btn, sizeof(btn), "EVA##eva%zu", slot);
                    if(ImGui::SmallButton(btn)) {
                        g.kerbalEVA(suit);
                    }
                    ImGui::SameLine();
                    // Per-crew experiment: the "pick a kerbal" path (no roles yet).
                    snprintf(btn, sizeof(btn), "Exp##exp%zu", slot);
                    if(ImGui::SmallButton(btn)) {
                        g.runSeatExperiment(suit);
                    }
                    // What this kerbal is carrying (unrecovered); a repeat still banks, just less.
                    if(!suit->experiments.empty()) {
                        ImGui::Indent();
                        for(const Experiment &he : suit->experiments) {
                            const bool repeat =
                                holdsExperiment(g.science.recovered, he);
                            ImGui::TextDisabled("  %s%s",
                                                experimentName(he).c_str(),
                                                repeat ? "  (repeat)" : "");
                        }
                        ImGui::Unindent();
                    }
                    ImGui::PopID();
                }
                // Capsule-level "Run Experiment": first seat (stable for
                // --sim-press).
                if(!seats.empty()) {
                    if(ImGui::SmallButton("Run Experiment")) {
                        g.runSeatExperiment(seats.front());
                    }
                }
                // Free kerbals in boarding reach: a Board button each, plus Store when one holds a
                // finding.
                bool anyInRange = false;
                // Reused buffer: this popup redraws every frame. Not reentrant --
                // the buttons below must not redraw this window, or the walk
                // refills the vector being iterated.
                static std::vector<Kerbal *> free_ks;
                freeKerbalsInto(vehicles, free_ks);
                for(Kerbal *k : free_ks) {
                    // Reach decides what is LISTED; canBoard / canMoveExperiment
                    // decide what is OFFERED -- the same gates the actions ask, so
                    // a button cannot offer what an action would refuse.
                    if(!g.kerbalInRange(k, ship->parts[part])) { continue; }
                    anyInRange = true;
                    // dist is for the on-screen readout only (the gate above did the math).
                    const glm::dvec3 capCom =
                        ship->GetPositionRelTo(ship->parts[part], k->frame);
                    const double dist =
                        glm::length(k->get_center_of_mass() - capCom);
                    /* The annotation names ONE reason, so it cannot go stale when
                       canBoard grows a condition. The gate below is the general
                       predicate; this is just the label. */
                    const bool full = ((int)ship->parts[part]->seatCount()
                                       >= def->crew_capacity);
                    const bool boardable = g.canBoard(k, ship, part);
                    ImGui::PushID(k);
                    ImGui::Text("  %s (%.1f m)%s", k->name.c_str(), dist,
                                full ? "  (capsule full)" : "");
                    ImGui::BeginDisabled(!boardable);
                    /* Same reason as the seat rows: PushID(k) makes the imgui id
                       unique but the test engine registers the LABEL, so two
                       kerbals in reach drew two indistinguishable "Board"s
                       (#326). The hidden id names the kerbal, which also lets a
                       case pin WHO it let in. */
                    /* std::string rather than a fixed buffer: the roster puts no
                       limit on a name (crew.cpp:64), and a truncated hidden id
                       would put two walkers back under one label. */
                    const std::string board = "Board##board_" + k->name;
                    const bool boardClicked = ImGui::SmallButton(board.c_str());
                    ImGui::EndDisabled();
                    if(boardClicked) {
                        g.kerbalBoard(k, ship, part);
                        /* Boarding DELETES the walking vehicle: what survives is
                           its suit Part, now parked in the capsule as a seat
                           (part.h). So `k` is dangling from here on and nothing
                           below may read it -- the Store block right after this
                           reads k->parts, which is exactly the use-after-free
                           the ASan report named. free_ks still holds the pointer,
                           but freeKerbalsInto refills it before anything reads
                           it again. The delete also drops part-window entries for
                           the walker, which shifts this loop's vector, so stop
                           the frame here and let the windows redraw. */
                        worldChanged = true;
                        ImGui::PopID();
                        break;
                    }
                    /* Store (the dance deposit). Offered whenever this capsule can
                       take findings at all: hiding the button on a refusal would
                       leave a player holding a sample with no visible way to
                       deposit it, so the refusal is a greyed button plus the
                       reason. Inside the canReceive branch the only way
                       canMoveExperiment can still refuse is canHold -- reach was
                       established by the loop above -- so the note is exact. */
                    if(!k->parts.empty()) {
                        const std::vector<Experiment> &held =
                            k->parts[0]->experiments;
                        if(!held.empty()) {
                            ImGui::Indent();
                            for(const Experiment &he : held) {
                                const bool repeat =
                                    holdsExperiment(g.science.recovered, he);
                                ImGui::TextDisabled("  %s%s",
                                                    experimentName(he).c_str(),
                                                    repeat ? "  (repeat)" : "");
                            }
                            ImGui::Unindent();
                            if(ship->parts[part]->canReceive()) {
                                const bool storable
                                    = g.canMoveExperiment(k->parts[0],
                                                          ship->parts[part], k);
                                if(!storable) {
                                    ImGui::TextDisabled("  (this capsule already"
                                                        " holds that finding)");
                                }
                                ImGui::BeginDisabled(!storable);
                                /* Addressable by WHO deposits: two walkers in
                                   reach draw two of these, and #323 asks for the
                                   name in the visible label too. The hidden id
                                   fixes the test half without deciding that copy
                                   question. */
                                const std::string store
                                    = "Store  (into capsule)##store_" + k->name;
                                const bool storeClicked
                                    = ImGui::SmallButton(store.c_str());
                                ImGui::EndDisabled();
                                if(storeClicked) {
                                    g.moveExperiment(k->parts[0],
                                                     ship->parts[part], 0);
                                }
                            }
                        }
                    }
                    ImGui::PopID();
                }
                if(!anyInRange) {
                    ImGui::Text("  (no one in reach to board or store)");
                }
            }
            // --- science (this part is a kerbal suit) --------------------------
            // v1 only kerbals run/hold experiments. The capsule's buttons cover an ABOARD
            // kerbal; this is the free-EVA path.
            if(ship->isEva() && part == 0) {
                Part *suit = ship->parts[part];
                ImGui::Separator();
                if(ImGui::SmallButton("Run Experiment")) {
                    g.runExperiment(static_cast<Kerbal *>(ship));
                }
                ImGui::Text("Experiments: %d", (int)suit->experiments.size());
                for(const Experiment &e : suit->experiments) {
                    ImGui::Text("  %s", experimentName(e).c_str());
                }
            }
            // --- science instrument (this part runs + holds an experiment) ------
            if(!def->experiment_family.empty()) {
                Part *pod = ship->parts[part];
                ImGui::Separator();
                /* The first seat on this ship (part.h): a seated kerbal is a suit
                   Part parked in a capsule, and the provenance a pod records is a
                   NAME, not a pointer. */
                std::vector<Part *> seats = vehicleSeats(ship);
                std::string runner = seats.empty() ? "" : seatName(seats.front());
                if(!runner.empty()) {
                    // ##id: keep this distinct from the capsule's "Run Experiment".
                    if(ImGui::SmallButton("Run Experiment##pod_run")) {
                        g.runPodExperiment(pod, runner);
                    }
                } else {
                    ImGui::TextDisabled("Run Experiment  (needs a crew aboard)");
                }
                ImGui::Text("Experiments: %d", (int)pod->experiments.size());
                for(const Experiment &e : pod->experiments) {
                    const bool repeat = holdsExperiment(g.science.recovered, e);
                    ImGui::Text("  %s%s", experimentName(e).c_str(),
                                repeat ? "  (repeat)" : "");
                }
                // Take: a WALKING kerbal in reach moves the pod's finding onto their suit (the
                // courier), to then store into a capsule. A seated one can't reach out.
                if(!pod->experiments.empty()) {
                    bool anyTake = false;
                    // Reused buffer + stack label (see the Board loop above).
                    static std::vector<Kerbal *> take_ks;
                    freeKerbalsInto(vehicles, take_ks);
                    for(Kerbal *k : take_ks) {
                        // A successful take empties the pod; the next iteration's
                        // canHold(pod->experiments[0]) would read an empty vector.
                        if(pod->experiments.empty()) { break; }
                        // Guarded here, not inside the gate: canMoveExperiment
                        // cannot protect a caller that already indexed.
                        if(k->parts.empty()
                           || !g.canMoveExperiment(pod, k->parts[0], k)) {
                            continue;
                        }
                        anyTake = true;
                        ImGui::PushID(k);
                        char takeLabel[192];
                        snprintf(takeLabel, sizeof(takeLabel),
                                 "Take  (to %s)##pod_take", k->name.c_str());
                        if(ImGui::SmallButton(takeLabel)) {
                            g.moveExperiment(pod, k->parts[0], 0);
                        }
                        ImGui::PopID();
                    }
                    if(!anyTake) {
                        ImGui::TextDisabled("Take  (needs a kerbal on EVA, in reach)");
                    }
                }
            }
            // --- inventory (this part is a container) --------------------------
            // Contained items get Drop (they leave into a free 1-part ship, Game::dropItem);
            // free item ships in pickup range get Pick up.
            if(def->inventory_capacity > 0) {
                Part *cont = ship->parts[part];
                ImGui::Separator();
                ImGui::Text("Inventory: %d / %d",
                            (int)cont->contents.size(),
                            def->inventory_capacity);
                if(cont->contents.empty()) {
                    ImGui::Text("  (empty)");
                }
                /* Snapshot: dropItem erases from contents mid-loop, and
                   a vector erase invalidates the range-for's iterator. The
                   buffer is reused -- this popup redraws every frame. */
                static std::vector<Part *> items;
                items = cont->contents;
                for(size_t ii = 0; ii < items.size(); ii++) {
                    Part *item = items[ii];
                    const char *in = item->def->label().c_str();
                    ImGui::PushID(item);
                    ImGui::Text("  %s  (%.1f kg)", in,
                                item->effectiveMass());
                    /* The hidden id is the row number, for the same reason as the
                       seat rows (#326): PushID(item) gives each row its own imgui
                       id, but the test engine registers the LABEL, so two stowed
                       parts of one kind drew two indistinguishable "Drop"s. The
                       index is what a player counts down the list, and unlike
                       item->uid it is the same on every run. */
                    char drop_id[32];
                    snprintf(drop_id, sizeof(drop_id), "Drop##drop_%zu", ii);
                    if(ImGui::SmallButton(drop_id)) {
                        g.dropItem(item);
                    }
                    ImGui::PopID();
                }
                // Free item ships in range: a Pick up button each (a crewed ship is not stowable).
                bool anyInRange = false;
                /* Walked by index because a successful "Pick up" deletes a
                   vehicle, and a delete erases it from the live list. The loop
                   stops there rather than continuing over a shifted index: the
                   window redraws next frame anyway, which is what `worldChanged`
                   is for. */
                bool stowed = false;
                for(size_t ni = 0; ni < vehicles.size() && !stowed; ni++) {
                    Vehicle *v = vehicles[ni];
                    if(v->isEva()) { continue; }
                    if(v->parts.size() != 1) { continue; }
                    if(v->parts[0]->container != nullptr) { continue; }
                    // A crewed ship is not stowable: its capsule holds a seat.
                    bool crewed = false;
                    for(Part *p : v->parts) {
                        if(p->seatCount() > 0) { crewed = true; break; }
                    }
                    if(crewed) { continue; }
                    if(v == ship) { continue; }
                    /* distanceTo: COM-to-COM in the common root frame -- a raw coordinate difference
                       would mix the two frames. */
                    const double dist = ship->distanceTo(v);
                    if(dist > 10.0) { continue; }
                    anyInRange = true;
                    ImGui::PushID(v);
                    ImGui::Text("  %s (%.1f m, %.1f kg)", v->name.c_str(),
                                dist, v->getMass());
                    /* Named by the cargo ship, same reason as "Drop" above: two
                       free items in range drew two indistinguishable "Pick up"s.
                       Ship names are deduped on spawn (dedupVehicleName), and a
                       std::string keeps a long name from truncating onto another. */
                    const std::string pick = "Pick up##pick_" + v->name;
                    if(ImGui::SmallButton(pick.c_str())) {
                        // Picking up the ship you are flying leaves orbit-view -- hand the player back to
                        // the carrier.
                        const bool wasFlying = (g.ship == v);
                        if(g.pickUpItem(v, cont)) {
                            if(wasFlying) { g.select_vehicle(cont->owner); }
                            worldChanged = true;
                            stowed = true;
                        }
                    }
                    ImGui::PopID();
                }
                if(!anyInRange) {
                    ImGui::Text("  (no cargo in range)");
                }
            }
            ImGui::Separator();
            ImGui::Text("Picked at: (%.0f, %.0f, %.0f)",
                        sel.point.x, sel.point.y, sel.point.z);
        }
        ImGui::End();
        if(!open) {
            /* Erase by identity, not by idx: a successful pickup may have removed an earlier
               entry and shifted this one off idx. */
            for(auto it = g.part_sels.begin(); it != g.part_sels.end(); ++it) {
                if(it->ship == ship && it->part == part) {
                    g.part_sels.erase(it);
                    break;
                }
            }
        }
        if(worldChanged) { break; }
    }
}

/* The focus frame's +X, passed to OrbitMap::setPlane as the screen-x reference: all
   three combo slots pin to it so switching the plane TILTS rather than rotates
   (#173, #185). */
static const glm::dvec3 kFocusEast(1.0, 0.0, 0.0);

/* The orbit map's plane for one combo slot: a normal in the focus's inertial frame
   plus the screen-x reference (zero length = let OrbitMap derive the basis). Both maps
   and --map-dump go through here, so the slots cannot drift apart (#173). The normals
   come from uiRefPlane -- the same call the Inc/LAN/LPe readout and the 3D plane
   overlay build -- so the picture and the numbers cannot drift either; only the
   screen-x pin is the map's own. */
static void mapPlaneBasis(Frame *focusFrame, const glm::dvec3 &orbit_pos,
                          const glm::dvec3 &orbit_vel, bool have_ship, int mode,
                          const glm::dvec3 &target_hat,
                          glm::dvec3 &plane_n, glm::dvec3 &plane_x) {
    const glm::dvec3 h = glm::cross(orbit_pos, orbit_vel);
    const double hl = glm::length(h);
    if(mode == kRefOrbit && !have_ship) {
        // No conic to take a plane from: the rail plane, basis left to OrbitMap.
        plane_n = glm::dvec3(0.0, 1.0, 0.0);
        plane_x = glm::dvec3(0.0, 0.0, 0.0);
        return;
    }
    /* A radial state's h is arithmetic noise, and a zero normal would send
       OrbitMap::setPlane off its canonical branch: read the rail plane there,
       as this function always has. */
    plane_n = uiRefPlane(focusFrame,
                         hl > 1e-9 ? h / hl : glm::dvec3(0.0, 1.0, 0.0),
                         target_hat, mode).n_hat;
    /* Screen-x pinned to the focus frame's +X in every slot (#185). Unpinned, a
       focus far off the system plane pushed setPlane off its canonical branch
       onto the node line and this slot's screen-east disagreed; the Orbital slot
       has the same problem as its normal approaches the rail normal. The AN/DN
       markers are world-space points projected through px(), so they do not
       need the screen-x axis to point at the node. This is a SCREEN
       choice, not the readout's zero longitude: LAN measures from
       equator_orient * +X, where the pole leans (uiRefPlane). */
    plane_x = kFocusEast;
}

/* The map's AN/DN markers: where the drawn orbit crosses the plane the map is
   showing, so they move with the Map plane combo. Green ring = ascending (the
   orbit leaves the far side of the plane there), red = descending. Rings, not
   filled dots: the apside markers are dots on the same curve and the two must
   not read as one glyph. The hues live in constants.h because the 3D plane
   overlay marks the same two points and must not disagree. orbit.h's
   orbitNodePoints shares the HUD's node gate, so a marker here and a dashed
   LAN there cannot disagree. */
static const ImVec4 kColNodeAn(kNodeAnRgb[0], kNodeAnRgb[1], kNodeAnRgb[2], 1.0f);
static const ImVec4 kColNodeDn(kNodeDnRgb[0], kNodeDnRgb[1], kNodeDnRgb[2], 1.0f);

static void drawMapNodes(OrbitMap &map, ImDrawList *dl, const OrbitElements &o,
                         double mu) {
    /* map.n rather than the caller's plane_n: setPlane normalized it, and the
       helper's sin(inc) floor only means what the HUD's gate means for a unit
       normal. Uninitialized points stay zero, so a missed bit cannot draw a
       marker at the focus. */
    glm::dvec3 an(0.0), dn(0.0);
    const int got = orbitNodePoints(o, map.n, mu, an, dn);
    if(got & kNodeAn) {
        dl->AddCircle(map.px(an), 4.5f, ImGui::GetColorU32(kColNodeAn), 0, 1.5f);
    }
    if(got & kNodeDn) {
        dl->AddCircle(map.px(dn), 4.5f, ImGui::GetColorU32(kColNodeDn), 0, 1.5f);
    }
}

// The orbital map: right-clicking cycles its chrome (full window -> bare map -> no
// window).
void drawUIMap(Game &g) {
    // flightDrawUi draws the map AFTER the windows, and the Ship List is one: a switch
    // there leaves the conic describing the previous vessel.
    syncShipView(g);
    TransferPlanner &planner = g.xferPlanner;
    Vehicle *ship = g.ship;
    std::vector<Vehicle *> &vehicles = g.vehicles;
    OrbitElements &o = g.hud.view.o;
    double &mu = g.hud.view.mu;
    glm::dvec3 &orbit_pos = g.hud.view.orbit_pos;
    glm::dvec3 &orbit_vel = g.hud.view.orbit_vel;
    float &map_scale = g.map.scale;
    int &map_plane = g.map.plane;
    ImVec2 &map_pan = g.map.pan;
    int &map_mode = g.map.mode;
    std::vector<TransferPlanner::XferTarget> &xferTargets = planner.xferTargets;
    int &xfer_target = planner.xfer_target;
    auto &xfer = planner.xfer;

    // Mode 2 strips the chrome: the window is invisible but still hit-tested, so the map
    // keeps pan/zoom and the right-click cycle. The table's options are const, hence a
    // per-frame copy.
    ui::Options mapOpts = kWins[W_OrbitalMap].opts;
    if(map_mode == 2) {
        mapOpts.flags |= ImGuiWindowFlags_NoDecoration |
                         ImGuiWindowFlags_NoBackground;
    }
    drawWin(g, W_OrbitalMap, mapOpts, [&] {
        // Right-click cycles the chrome. Over the map this is safe: imgui owns the mouse,
        // so the RMB camera orbit (gated on !WantCaptureMouse) never fires.
        if(ImGui::IsWindowHovered() &&
           ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
            map_mode = (map_mode + 1) % 3;
        }
        const float avail_w = ImGui::GetContentRegionAvail().x;
        const float avail_h = ImGui::GetContentRegionAvail().y;
        const float isp = ImGui::GetStyle().ItemSpacing.y;
        const float legend_h = std::max(16.0f, ImGui::GetTextLineHeight());
        // Controls sit ABOVE the map so they stay visible when the window is short.
        const float controls_h = (map_mode == 0)
            ? (legend_h + isp)
              + (ImGui::GetFrameHeight() + isp)
            : 0.0f;
        const float map_w = std::max(0.0f, avail_w);
        const float map_h = std::max(0.0f, avail_h - controls_h);
        // The ship's trajectory: a closed Kepler ellipse, or an open arc when escaping
        // (ecc >= 1).
        const bool closed = (o.ecc < 1.0);
        // A reference into the cache (closed) or a local (open) -- never a copy every frame.
        std::vector<glm::dvec3> traj_local;
        const std::vector<glm::dvec3> *traj_pts = nullptr;
        if(closed) {
            // Per-ship cache, trusted only while the ship is ON RAILS; off rails the orbit is
            // moving, so re-sample every frame. Fixed N (not the body-orbit LOD count): the
            // Surface Map shares this entry.
            const int N = 64;
            traj_pts = &orbit_caches[(const void *)ship].sample(
                orbit_pos, orbit_vel, mu, N, ship->onRails);
        } else {
            // Open trajectory: an arc around periapsis, truncated where it would run to infinity.
            // r_cap is the view extent, never below a few periapsis radii or the current radius.
            const double r_cap = std::max<double>(
                std::max(map_w, map_h) * (double)map_scale,
                std::max(4.0 * o.periapsis, o.distance));
            const int N = 64;
            traj_local = sampleOpenTrajectory(orbit_pos, orbit_vel, mu, N, r_cap);
            traj_pts = &traj_local;
        }

        // Periapsis (both cases) and apoapsis (closed only): a closed orbit propagates to
        // each exactly.
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
    
        // The focus body and the map plane -- a normal in the focus's inertial frame;
        // OrbitMap derives an in-plane basis from it.
        TerrainBody *focus = ship->m_parent;
        glm::dvec3 plane_n, plane_x;
        mapPlaneBasis(focus->frame, orbit_pos, orbit_vel, true, map_plane,
                      g.hud.view.target_h_hat, plane_n, plane_x);

        // KSP-inspired palette: your orbit green, the transfer blue, other bodies gray; the
        // focus, ship dot and labels use ink contrasting the window background.
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
        // The near-body shell ring (science + surface-frame boundary), gray like the SOI.
        const ImU32 shell_col = ImGui::GetColorU32(ImVec4(0.50f, 0.50f, 0.50f, 0.35f));
        // Atmosphere top: a desaturated-blue disk behind the body, so the rim marks where
        // the air ends.
        const ImU32 col_atmo = ImGui::GetColorU32(ImVec4(0.35f, 0.50f, 0.66f, 0.20f));
        ImDrawList *dl = ImGui::GetWindowDrawList();

        // Controls (mode 0 only), pinned ABOVE the map so they stay on a short window.
        if(map_mode == 0) {
            enum Swatch : unsigned char { kRect, kDot, kRing };
            auto legend = [&](const char *label, ImU32 col, Swatch kind) {
                const ImVec2 p = ImGui::GetCursorScreenPos();
                const float s = 10.0f;
                if(kind == kDot) {
                    dl->AddCircleFilled(ImVec2(p.x + 5.0f, p.y + 8.0f), 4.0f, col);
                } else if(kind == kRing) {
                    // Same glyph as the map's AN/DN markers, so the legend is not
                    // describing a shape the map never draws.
                    dl->AddCircle(ImVec2(p.x + 5.0f, p.y + 8.0f), 4.0f, col, 0, 1.5f);
                } else {
                    dl->AddRectFilled(ImVec2(p.x, p.y + 3.0f),
                                     ImVec2(p.x + s, p.y + 13.0f), col);
                }
                ImGui::Dummy(ImVec2(s, 16.0f));
                ImGui::SameLine();
                ImGui::TextUnformatted(label);
            };
            legend("you", col_ship, kRect);
            ImGui::SameLine();
            legend("ships", col_vessel, kRect);
            ImGui::SameLine();
            legend("xfer", col_xfer, kRect);
            ImGui::SameLine();
            legend("bodies", col_child, kRect);
            ImGui::SameLine();
            legend("apsides", col_apsis, kDot);
            ImGui::SameLine();
            legend("AN", ImGui::GetColorU32(kColNodeAn), kRing);
            ImGui::SameLine();
            legend("DN", ImGui::GetColorU32(kColNodeDn), kRing);
            // Map plane (half-width) + reset view.
            /* The fourth slot orients the map into the TARGET window's target
               orbit, so the map's dihedral is the plane change to it. It needs a
               target, so picking one without a target is refused rather than
               silently re-orienting the map. And the widget shows the plane the
               map is ACTUALLY in (RefPlane::mode): when a target disappears under
               the slot -- "none" in the TARGET window, or a vessel target that
               lands, escapes or leaves the SOI -- the readout falls back to the
               ecliptic and says "ecl", so the combo must not keep claiming
               "Target orbit". The choice itself survives, and comes back with the
               target. */
            static const char *kPlanes[] = { "Equatorial", "Ecliptic", "Orbital",
                                             "Target orbit" };
            ImGui::PushItemWidth(avail_w * 0.5f);
            int plane_sel = g.hud.view.ref_plane.mode;
            if(uiCombo("Map plane", &plane_sel, kPlanes, 4)) {
                if(plane_sel == kRefTarget && !g.hud.view.target_plane_ok) {
                    g.toast("No transfer target: pick one in the TARGET window");
                } else {
                    map_plane = plane_sel;
                }
            }
            ImGui::PopItemWidth();
            ImGui::SameLine();
            if(ImGui::Button("Reset view")) {
                map_pan = ImVec2(0.0f, 0.0f);
                map_scale = kMapDefaultScale;
            }
        }

        // The focus (parent body) sits at the map's centre plus the pan offset.
        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        const float center_x = p0.x + map_w * 0.5f;
        const float center_y = p0.y + map_h * 0.5f;

        // An invisible button sized to the map: it captures the mouse, so a left-drag pans
        // the map instead of moving the window.
        ImGui::InvisibleButton("##mapnav", ImVec2(map_w, map_h));
        const bool over_map = ImGui::IsItemHovered();
        const ImGuiIO &g_io = ImGui::GetIO();
        if(over_map && g_io.MouseWheel != 0.0f) {
            // Wheel zooms to the cursor (the world point under the mouse stays put).
            const float factor = (g_io.MouseWheel > 0.0f) ? 0.8f : 1.25f;
            const float old_scale = map_scale;
            float new_scale = old_scale * factor;
            if(new_scale < kMapMinScale) { new_scale = kMapMinScale; }
            if(new_scale > kMapMaxScale) { new_scale = kMapMaxScale; }
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

        OrbitMap proj;
        proj.cx = center_x + map_pan.x;
        proj.cy = center_y + map_pan.y;
        proj.scale = map_scale;
        proj.setPlane(plane_n, plane_x);
        const ImVec2 focus_px = proj.px(glm::dvec3(0.0, 0.0, 0.0));

        // The body selected in the TARGET window, highlighted on the map; nullptr for a
        // ship target.
        TerrainBody *sel_body = nullptr;
        if(xfer_target >= 0 && xfer_target < (int)xferTargets.size() &&
           xferTargets[xfer_target].body) {
            sel_body = xferTargets[xfer_target].body;
        }

        // SOI ring, faint. Skipped when sub-pixel or far off-view (a huge circle is useless
        // and expensive).
        auto draw_soi = [&](const glm::dvec3 &center, double soi_m, ImU32 col) {
            if(soi_m <= 0.0) { return; }
            const float r_px = (float)(soi_m / map_scale);
            if(r_px < 1.0f || r_px > 4000.0f) { return; }
            proj.drawRing(dl, center, soi_m, col, 1.0f);
        };

        // Drawn before the ship's orbit so the ship sits on top.
        {
            const MapViewRect view{p0.x, p0.y, p0.x + map_w, p0.y + map_h};
            drawSystemBodyOrbits(g, focus, proj, dl, view, map_scale,
                                 sel_body, col_child, col_sel, ink,
                                 soi_col);
        }
        // The focus body's own SOI -- the gravitational regime the ship is inside.
        draw_soi(glm::dvec3(0.0, 0.0, 0.0), focus->frame->soi, soi_col);
        // The near-body (rotating-frame) shell -- drives the science orbit cut and the
        // surface-frame flip.
        draw_soi(glm::dvec3(0.0, 0.0, 0.0), focus->rot_frame->soi, shell_col);
    
        // The atmosphere top (top() in meters above sea level) as a blue disk under the
        // orbit line, with the same min-pixel floor as the body disk.
        const double atmo_top = focus->surface.atmosphere.top();
        if(atmo_top > 0.0) {
            proj.drawBody(dl, glm::dvec3(0.0, 0.0, 0.0),
                         focus->radius + atmo_top, col_atmo, 3.0f);
        }
        // closed=true for the ellipse; false for the open arc (a chord would close it).
        proj.drawOrbit(dl, *traj_pts, col_ship, 1.0f, closed);
        proj.drawBody(dl, glm::dvec3(0.0, 0.0, 0.0), focus->radius,
                     col_body, 3.0f);
        const ImVec2 ship_px = proj.px(orbit_pos);
        dl->AddLine(focus_px, ship_px, ink, 1.0f);
        dl->AddCircleFilled(ship_px, 5.0f, ink);
        dl->AddCircle(ship_px, 8.0f, col_ship, 0, 1.5f);
        // Prograde (velocity) arrow.
        proj.drawArrow(dl, orbit_pos, orbit_vel, 24.0f, col_ship, 1.5f);
        // Apside markers only mean anything for a non-circular orbit.
        if(o.ecc > 1e-3) {
            if(have_peri) { proj.drawDot(dl, peri_p, 4.0f, col_apsis); }
            if(have_apo)  { proj.drawDot(dl, apo_p,  4.0f, col_apsis); }
        }
        // AN/DN against the plane this map is showing: always drawn when the
        // orbit is inclined to it, so picking "Target orbit" and reading where
        // the two curves cross is a glance, not a calculation.
        drawMapNodes(proj, dl, o, mu);

        // Every other ship in this body: its orbit (when closed) plus a dot + label. An escaping
        // ship has no closed orbit -- sample() returns empty -- so only its marker shows.
        {
            Frame *inertial = ship->frame->getNonRotFrame();
            for(auto *s : vehicles) {
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
                const int N = 64;
                const std::vector<glm::dvec3> &tpts =
                    orbit_caches[(const void *)s].sample(
                        r2, v2, mu, N, s->onRails);
                if(!tpts.empty()) {
                    proj.drawOrbit(dl, tpts, col_vessel, 1.0f);
                }
                const ImVec2 tpx = proj.px(r2);
                dl->AddCircleFilled(tpx, 3.0f, col_vessel);
                // Label offset from the marker (px): +x right, -y up.
                const float label_dx = 6.0f, label_dy = 14.0f;
                dl->AddText(ImVec2(tpx.x + label_dx, tpx.y - label_dy),
                            col_vessel, s->name.c_str());
            }
        }

        // The solution belongs to the target the solve USED: xfer.valid lags the TARGET
        // window by a frame -- clearing the combo there is not re-validated until the next
        // planner.update(), so pairing xfer.sol with the live xfer_target read xferTargets[-1]
        // when "none" was clicked.
        if(xfer.valid && xfer_target == xfer.target &&
           xfer.target >= 0 && xfer.target < (int)xferTargets.size()) {
            const TransferSolution &sol = xfer.sol;
            // Even-in-anomaly (not uniform-in-time) so the leg draws with an even outline.
            std::vector<glm::dvec3> xfer_pts =
                sampleTransferArc(orbit_pos, sol.v_departure, mu, sol.tof, 64);
            proj.drawOrbit(dl, xfer_pts, col_xfer, 1.5f, /*closed=*/false);
            const glm::dvec3 &arrival = xfer_pts.back();
            proj.drawDot(dl, arrival, 4.0f, col_xfer);
            char xfer_label[96];
            snprintf(xfer_label, sizeof(xfer_label), "%s  %.0f m/s",
                     xferTargets[xfer.target].name, sol.total_dv);
            const ImVec2 apx = proj.px(arrival);
            dl->AddText(ImVec2(apx.x + 5.0f, apx.y + 4.0f), col_xfer,
                        xfer_label);
        }
    });
}

/* The modal scrim: a full-viewport window drawn UNDER the card so a pending question
   cannot be clicked through (#234). It carries no widgets, but imgui hit-tests it.

   NOT flagged NoBringToFrontOnFocus: that flag makes CreateNewWindow push_front() the
   window (imgui.cpp:6791) -- the BACK of the display order. */
static void drawQuestionScrim() {
    const ImGuiViewport *vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->Pos, ImGuiCond_Always);
    ImGui::SetNextWindowSize(vp->Size, ImGuiCond_Always);
    ImGui::Begin("##question_scrim", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize
               | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar
               | ImGuiWindowFlags_NoSavedSettings
               | ImGuiWindowFlags_NoBackground);
    ImGui::GetWindowDrawList()->AddRectFilled(
        vp->Pos, ImVec2(vp->Pos.x + vp->Size.x, vp->Pos.y + vp->Size.y),
        IM_COL32(0, 0, 0, 110));
    ImGui::End();
}

/* The question card (Game::ask). Deliberately NOT in the window table (uiwins.h): a
   card must show in every scene and must not vanish on TAB, and the table's three gates
   -- scene membership, TAB hiding, default_open -- are each a way for a dialog to
   silently never draw. Called after the scene's UI pass and before drawToasts. */
void drawQuestion(Game &g) {
    if(!g.question) { return; }
    drawQuestionScrim();
    static const ui::Options opts = []{
        ui::Options o;
        o.slot = ui::Slot::Center;
        o.fixed = true;              // no move, no resize; re-placed each frame
        o.fixed_width = 34.0f;       // font/DPI-relative; height fits content
        return o;
    }();

    int choice = -1;
    const bool drew = ui::Window("Question", opts, [&]{
        ImGui::TextWrapped("%s", g.question->text.c_str());
        ImGui::Spacing();
        const int n = (int)g.question->labels.size();
        if(n == 0) { return; }
        // Equal-width buttons across the card; fixed width keeps a long label from
        // overflowing its button.
        const float gap = ImGui::GetStyle().ItemSpacing.x;
        const float bw = (ImGui::GetContentRegionAvail().x - gap * (n - 1)) / n;
        for(int i = 0; i < n; i++) {
            if(i > 0) { ImGui::SameLine(); }
            if(ImGui::Button(g.question->labels[i].c_str(), ImVec2(bw, 0.0f))) {
                choice = i;
            }
        }
    });
    assert(drew && "question pending but the card did not draw");
    /* Keep the card in front of the scrim: clicking the scrim focuses it, and FocusWindow
       moves the focused window to the display front. */
    if(ImGuiWindow *card = ImGui::FindWindowByName("Question")) {
        ImGui::BringWindowToDisplayFront(card);
    }
    // Answered after End(): answerQuestion destroys g.question, which the body above is
    // still reading.
    if(choice >= 0) { g.answerQuestion(choice); }
}

void drawToasts(Game &g) {
    /* A frame about to be screenshotted draws no toasts: they land dead centre, so a burst
       of F12 presses photographs the previous shot's message. The queue is untouched, so an
       in-flight toast returns next frame. */
    if(g.screenshot_requested) { return; }
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
        // Oldest on top, newest at the bottom.
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

/* The main menu: one shared shell for every scene. Items are fixed-width buttons: the
   window is AlwaysAutoResize and imgui measures from the PREVIOUS frame's content, so
   a hand-centered Text item feeds back into the measurement.
   `isRoot` scenes force the menu open every frame -- TAB, "Reset windows" or a stray
   SetOpen must not leave a scene with no UI. */
static void drawMenuWindow(Game &g, Win win, bool isRoot, const char *heading,
                           void (*nav)(Game &, float),
                           void (*navBottom)(Game &, float) = nullptr) {
    if(isRoot) { setWinOpen(win, true); }
    drawWin(g, win, [&] {
        bool &running = g.running;
        // One width for the whole column: the widest label (the title, in the bigger font)
        // plus frame padding.
        ImGui::PushFont(g.bigger);
        const float bw = ImMax(240.0f,
                               ImGui::CalcTextSize("Open Space Program").x
                               + ImGui::GetStyle().FramePadding.x * 2.0f);
        ImGui::PopFont();
        // Alpha-0 button colors: reads as plain text but keeps the button's stable layout.
        const ImVec4 invisible = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
        auto text_button = [&](const char *label) {
            ImGui::PushStyleColor(ImGuiCol_Button, invisible);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, invisible);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, invisible);
            ImGui::Button(label, ImVec2(bw, 0.0f));
            ImGui::PopStyleColor(3);
        };
        ImGui::PushFont(g.bigger);
        text_button(heading);
        if(nav) { nav(g, bw); }
        if(ImGui::Button("Save/Load", ImVec2(bw, 0.0f))) {
            setWinOpen(W_SaveLoad, !winOpen(W_SaveLoad));
        }
        if(ImGui::Button("Settings", ImVec2(bw, 0.0f))) {
            setWinOpen(W_Settings, !winOpen(W_Settings));
        }
        if(ImGui::Button("Controls", ImVec2(bw, 0.0f))) {
            setWinOpen(W_Controls, !winOpen(W_Controls));
        }
        // The scene's exits (hub: "Return to title"), just above the app's own.
        if(navBottom) { navBottom(g, bw); }
        /* Quitting discards everything since the last manual save (no autosave, no
           save-on-quit), so a live game is asked first; the title screen keeps one click. */
        if(ImGui::Button("Quit game", ImVec2(bw, 0.0f))) {
            if(!gameRunning(g)) {
                running = false;
            } else {
                g.ask("Quit the game?\n\nEverything since your last save is lost.",
                      {"Quit", "Stay"},
                      [&g](int choice) {
                    if(choice == 0) { g.running = false; }
                });
            }
        }
        ImGui::PopFont();
        // The build's git version (src/version.h, `make version`), grayed as a footer.
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.55f, 0.55f, 0.55f, 1.0f));
        text_button(VERSION);
        ImGui::PopStyleColor();
    });
}

/* The per-scene nav blocks. Each closes its own menu before the transition it starts. */
static void navTitle(Game &g, float bw) {
    /* "last" means the newest save by WRITE time, not the newest game dir
       (save.h:latestSave). This menu is Root -- open every frame -- so the saves/ scan
       refreshes at ~1 Hz. Cached as a path, never a pointer: the slot can be deleted
       between frames. */
    static const double kRescanSec = 1.0;
    static double nextRescan = 0.0;
    static std::string last;
    if(ImGui::GetTime() >= nextRescan) {
        nextRescan = ImGui::GetTime() + kRescanSec;
        last = latestSave(datadir::saves());
    }
    // Above "New Game": resuming beats starting over when there is a save.
    if(!last.empty() && ImGui::Button("Load last save", ImVec2(bw, 0.0f))) {
        const size_t slash = last.find('/');   // latestSave: "<game>/<slot>"
        const std::string game = gameDirName(last.substr(0, slash));
        const std::string slot = last.substr(slash + 1);
        /* Deferred: the load tears the fleet down immediately, and this window
           is Root -- the rest of the frame's widgets would read the new world
           (or nothing) under a click that has not finished drawing (#304).
           `last` rescans at 1 Hz, so the slot is captured, not looked up again. */
        const std::string dir = datadir::saves() + "/" + last;
        g.requestWorldOp([&g, dir, game, slot] {
            if(g.loadFrom(dir)) {
                g.toast("Loaded %s / %s", game.c_str(), slot.c_str());
            }
        });
    }
    // Opens the New Game setup sheet (system + difficulty), not an immediate start.
    if(ImGui::Button("New Game", ImVec2(bw, 0.0f))) {
        setWinOpen(W_NewGame, true);
    }
    if(ImGui::Button("Readme", ImVec2(bw, 0.0f))) {
        setWinOpen(W_Readme, !winOpen(W_Readme));
    }
}
static void navSpaceCenter(Game &g, float bw) {
    if(ImGui::Button("VAB", ImVec2(bw, 0.0f))) {
        setWinOpen(W_SpaceCenterMenu, false);
        vabOpen(g);
    }
    // Open even with no ship yet (a new game before the first launch).
    if(ImGui::Button("Tracking Station", ImVec2(bw, 0.0f))) {
        setWinOpen(W_SpaceCenterMenu, false);
        pushScene(g, SceneId::TrackingStation);
    }
    // The Research Lab: archive of recovered experiments, another excursion.
    if(ImGui::Button("Research Lab", ImVec2(bw, 0.0f))) {
        setWinOpen(W_SpaceCenterMenu, false);
        pushScene(g, SceneId::ResearchLab);
    }
    // Offered only when the hub sits ON TOP of a live flight; when the hub IS the floor
    // there is nothing to pop to.
    if(g.ship != nullptr && ImGui::Button("Resume Flight", ImVec2(bw, 0.0f))) {
        setWinOpen(W_SpaceCenterMenu, false);
        popScene(g);
    }
    // Recover Vessel ends the flight successfully. Not offered for a free EVA kerbal;
    // recoverActive also refuses unless the ship is grounded on the home body.
    if(g.ship != nullptr && !g.ship->isEva()
       && ImGui::Button("Recover Vessel", ImVec2(bw, 0.0f))) {
        g.recoverActive();
    }
    // "Return to title" lives in navSpaceCenterExit. Esc in the hub never exits to the
    // title -- it only pops to the flight (hubKeyActions).
    ImGui::Spacing();
}

/* The hub's bottom nav row: the exits. The confirm lives on the card, not on the
   button -- an armed first click overflowed the button. */
static void navSpaceCenterExit(Game &g, float bw) {
    if(ImGui::Button("Return to title", ImVec2(bw, 0.0f))) {
        g.ask("Return to the title screen?\n\nThe current flight is left behind.",
              {"Return", "Stay"},
              [&g](int choice) {
            if(choice != 0) { return; }
            setWinOpen(W_SpaceCenterMenu, false);
            /* The card answers inside drawQuestion, i.e. after this frame's
               refresh and paint -- so the teardown it starts waits for the top
               of the next frame, where a hook's would run (#304). */
            g.requestWorldOp([&g] { g.resetToTitle(); });
        });
    }
}

// The two menus: one shell, one heading + nav block each, both Root.
void drawTitleMenu(Game &g) {
    drawMenuWindow(g, W_TitleMenu, true, "Open Space Program", navTitle);
}

void drawSpaceCenterMenu(Game &g) {
    drawMenuWindow(g, W_SpaceCenterMenu, true, "Space Center", navSpaceCenter,
                   navSpaceCenterExit);
}

/* The hub's top bar: home calendar clock, science recovered, vessels out (free
   kerbals included). Read-only. */
void drawSpaceCenterTopBar(Game &g) {
    drawWin(g, W_SpaceCenterTopBar, [&] {
        const Calendar &cal = g.sys.home ? g.sys.home->cal : Calendar{};
        char stamp[64];
        if(fmt_cal_time(cal, g.time, stamp, sizeof stamp)) {
            ImGui::TextUnformatted(stamp);
        } else {
            ImGui::Text("t=%.0f s", g.time);
        }
        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();
        ImGui::Text("Science: %d", g.science.score);
        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();
        ImGui::Text("Vessels: %d", (int)g.vehicles.size());
    });
}

// A save-slot name is a single directory under a game dir: whitelist to letters,
// digits, - _ . so it can never carry a path separator or be a dot-name.
static bool safeSlotName(const std::string &n) {
    if(n.empty() || n == "." || n == "..") { return false; }
    for(unsigned char c : n) {
        bool alnum = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                     (c >= '0' && c <= '9');
        if(!alnum && c != '-' && c != '_' && c != '.') { return false; }
    }
    return true;
}

// A game name is the tail of a game dir name under saves/ (<stamp>-<name>). Blocklist
// instead of the slot whitelist: forbid only what breaks a dir name.
static bool safeGameName(const std::string &n) {
    if(n.empty() || n == "." || n == "..") { return false; }
    for(unsigned char c : n) {
        if(c < 0x20) { return false; }
        switch(c) {
            case '/': case '\\': case ':': case '*': case '?':
            case '"': case '<': case '>': case '|':
                return false;
        }
    }
    return true;
}

/* The New Game sheet's difficulty band. Narrower than the range --exhaust-scale
   and the save loader allow (0.5-5): past these the game stops being a game, and
   the CLI + hand-edited saves stay the door to the extremes. */
static constexpr float kExhaustUiMin = 0.8f;
static constexpr float kExhaustUiMax = 2.0f;

/* The New Game setup sheet: the game's name, which star system to load, and the
   exhaust-velocity scale (difficulty). The system list is a directory scan of
   res/systems, cached and re-read only when its mtime changes. */
void drawNewGame(Game &g) {
    // Selection + the scanned list persist across frames (and a close/reopen, so the last
    // pick sticks).
    static std::vector<std::string> systems;
    static bool scanned = false;
    static std::filesystem::file_time_type dirMtime;
    static int sysSel = 0;
    // The game's display name: the dir under saves/ it mints is <stamp>-<name>.
    static char nameBuf[256] = "game1";
    // The slider's draft value, committed to args.exhaust_scale only by Start: bound live,
    // it would survive Cancel / X and leak into settings.json via "Save settings".
    static float exhaustSel = 1.0f;
    {
        std::error_code ec;
        const auto mtime = std::filesystem::last_write_time(
            resdir::path("res/systems"), ec);
        if(!ec && (!scanned || mtime != dirMtime)) {
            // Keep an in-progress pick across a rescan; only the FIRST scan seeds from the
            // running system (so Start is a no-op swap).
            const bool first = !scanned;
            std::string keep;
            if(!first && sysSel >= 0 && sysSel < (int)systems.size()) {
                keep = systems[(size_t)sysSel];
            }
            systems = list_systems(resdir::path("res/systems"));
            dirMtime = mtime;
            scanned = true;
            sysSel = 0;
            if(first) {
                const std::string &cur = g.systemPath.empty()
                                             ? g.args.system_file
                                             : g.systemPath;
                const size_t slash = cur.find_last_of('/');
                const std::string file =
                    (slash == std::string::npos) ? cur : cur.substr(slash + 1);
                keep = (file.size() > 5
                        && file.compare(file.size() - 5, 5, ".json") == 0)
                           ? file.substr(0, file.size() - 5)
                           : file;
            }
            for(size_t i = 0; i < systems.size(); i++) {
                if(systems[i] == keep) { sysSel = (int)i; break; }
            }
        }
    }
    if(sysSel >= (int)systems.size()) { sysSel = (int)systems.size() - 1; }
    if(sysSel < 0) { sysSel = 0; }

    drawWin(g, W_NewGame, [&] {
        // Re-seed the draft difficulty on every open, so Cancel / X discard the slider
        // edits. Clamped to the sheet's band: a --exhaust-scale 5 boot would
        // otherwise park the knob at 2.0 while Start committed 5.
        if(ImGui::IsWindowAppearing()) {
            exhaustSel = std::clamp(g.args.exhaust_scale, kExhaustUiMin, kExhaustUiMax);
        }
        ImGui::TextWrapped(
            "Name the game, choose a star system and engine performance, "
            "then start.");
        ImGui::Spacing();

        ImGui::Text("Game name");
        ImGui::SetNextItemWidth(220);
        ImGui::InputText("##newgame_name", nameBuf, sizeof(nameBuf));
        if(!safeGameName(nameBuf)) {
            ImGui::TextDisabled("(no / \\ : * ? \" < > | or control chars)");
        }
        ImGui::Spacing();

        ImGui::Text("System");
        ImGui::SetNextItemWidth(-1.0f);
        if(systems.empty()) {
            ImGui::TextDisabled("(none in res/systems)");
        } else if(uiBeginCombo("##newgame_system",
                                    systems[(size_t)sysSel].c_str())) {
            for(size_t i = 0; i < systems.size(); i++) {
                if(ImGui::Selectable(systems[i].c_str(), (int)i == sysSel)) {
                    sysSel = (int)i;
                }
            }
            ImGui::EndCombo();
        }

        ImGui::Spacing();
        ImGui::Text("Exhaust velocity scale (difficulty)");
        // Draft until Start (see exhaustSel above). Lower is harder (less thrust/delta-v).
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::SliderFloat("##newgame_exhaust", &exhaustSel,
                           kExhaustUiMin, kExhaustUiMax, "%.2fx");
        ImGui::TextDisabled("0.8x harder  ·  1.0x stock  ·  2.0x easier");
        ImGui::TextDisabled("Stored in the save file.");

        ImGui::Spacing();
        if(ImGui::Button("Start", ImVec2(120.0f, 0.0f))) {
            if(systems.empty()) {
                g.toast("No systems in res/systems");
            } else if(!safeGameName(nameBuf)) {
                g.toast("Give the game a name (spaces are fine)");
            } else {
                const std::string path =
                    "res/systems/" + systems[(size_t)sysSel] + ".json";
                /* Deferred (#304): startNewGame switches system and tears the
                   world down. nameBuf, systems and sysSel are all sheet-local
                   statics, so they would still EXIST when this runs -- they
                   would just hold whatever the next frame's draw left in them,
                   which is why the name and the scale are copied out. */
                const std::string gameName = nameBuf;
                const float exhaust = exhaustSel;   // static: copy, do not capture
                g.requestWorldOp([&g, gameName, path, exhaust] {
                    if(g.startNewGame(gameName, path, exhaust)) {
                        setWinOpen(W_NewGame, false);
                    }
                });
            }
        }
        ImGui::SameLine();
        if(ImGui::Button("Cancel", ImVec2(120.0f, 0.0f))) {
            setWinOpen(W_NewGame, false);
        }
    });
}

/* The Flight Summary (W_FlightSummary), opened by the hub's "Recover Vessel": the
   recovered vessel, the flight duration, and the SoI enter/leave journal. Transient;
   Space-Center-only. */
void drawFlightSummary(Game &g) {
    drawWin(g, W_FlightSummary, [&] {
        const Game::FlightSummary &fs = g.flightSummary;
        ImGui::PushFont(g.bigger);
        ImGui::TextUnformatted("Flight complete!");
        ImGui::PopFont();
        ImGui::Spacing();
        ImGui::TextWrapped(
            "Congratulations — a successful flight. "
            "The vessel and its crew are home.");
        ImGui::Spacing();
        ImGui::Text("Vessel: %s", fs.shipName.c_str());
        const Calendar &cal = g.sys.home ? g.sys.home->cal : Calendar{};
        const double dt = fs.end_t - fs.log.start_t;
        char dur[32];
        fmt_cal_duration(cal, dt, dur, sizeof dur);
        ImGui::Text("Duration: %s", dur);
        char stamp[64];
        if(fmt_cal_time(cal, fs.log.start_t, stamp, sizeof stamp)) {
            ImGui::Text("Started:  %s", stamp);
            if(g.sys.home) {
                ImGui::SameLine();
                ImGui::TextDisabled("(%s)", g.sys.home->name.c_str());
            }
        }
        if(fmt_cal_time(cal, fs.end_t, stamp, sizeof stamp)) {
            ImGui::Text("Recovered: %s", stamp);
        }
        if(fs.scienceGained > 0) {
            ImGui::Spacing();
            ImGui::Text("Science: +%d", fs.scienceGained);
            if(!fs.newExperiments.empty()) {
                ImGui::Indent();
                for(const Experiment &e : fs.newExperiments) {
                    ImGui::Text("new:   %s", experimentName(e).c_str());
                }
                ImGui::Unindent();
            }
            if(fs.repeatScience > 0) {
                // Re-farmed experiments: scored down by diminishing returns.
                ImGui::TextDisabled("%d from repeats (already recovered)",
                                    fs.repeatScience);
            }
            ImGui::Text("Total science: %d", g.science.score);
        }
        ImGui::Spacing();
        if(fs.log.events.empty()) {
            ImGui::TextDisabled("No recorded events.");
        } else {
            ImGui::TextUnformatted("Events");
            ImGui::Indent();
            for(const FlightEvent &e : fs.log.events) {
                char ts[48];
                if(!fmt_cal_compact(cal, e.t, ts, sizeof ts)) {
                    snprintf(ts, sizeof ts, "t=%.0f", e.t);
                }
                ImGui::Text("%s  %s %s", ts,
                            e.enter ? "entered" : "left", e.body.c_str());
            }
            ImGui::Unindent();
        }
        ImGui::Spacing();
        if(ImGui::Button("OK", ImVec2(120.0f, 0.0f))) {
            setWinOpen(W_FlightSummary, false);
        }
    });
}

/* The title screen's README panel: the readme as plain text (no markdown renderer, so
   HTML image tags drop and heading / bold markers strip). First readable candidate wins. */
static const std::vector<std::string> &readmeLines() {
    static const std::vector<std::string> lines = [] {
        namespace fs = std::filesystem;
        const std::string root = resdir::root();
        const std::string cands[] = {
            root + "release/README.md",
            root + "README.md",
            root + "../doc/README.md",
        };
        std::string text;
        for(const std::string &p : cands) {
            std::ifstream f(p);
            if(!f) { continue; }
            std::ostringstream ss;
            ss << f.rdbuf();
            if(!ss.str().empty()) {
                text = ss.str();
                break;
            }
        }
        if(text.empty()) { text = "(README.md not found)\n"; }

        std::vector<std::string> out;
        std::istringstream in(text);
        std::string line;
        while(std::getline(in, line)) {
            if(!line.empty() && line.back() == '\r') { line.pop_back(); }
            // HTML image tags are not renderable here; drop the line.
            if(line.compare(0, 4, "<img") == 0) { continue; }
            size_t i = 0;
            while(i < line.size() && line[i] == '#') { i++; }
            if(i > 0 && i < line.size() && line[i] == ' ') {
                line.erase(0, i + 1);
            }
            for(size_t b; (b = line.find("**")) != std::string::npos; ) {
                line.erase(b, 2);
            }
            out.push_back(line);
        }
        return out;
    }();
    return lines;
}

void drawReadme(Game &g) {
    drawWin(g, W_Readme, [&] {
        // Scroll box: the readme is longer than a fitted panel.
        ImGui::BeginChild("readme_body", ImVec2(0.0f, 0.0f));
        for(const std::string &line : readmeLines()) {
            if(line.empty()) {
                ImGui::Spacing();
            } else {
                ImGui::TextWrapped("%s", line.c_str());
            }
        }
        ImGui::EndChild();
    });
}

void drawSaveLoad(Game &g) {
    // The slot name and the selections persist (static): so the player does not retype,
    // and Load/Delete act on the chosen row.
    static char nameBuf[256] = "save1";
    static int gameSel = 0;    // index into the games list
    static int slotSel = 0;    // index into the selected game's slots
    static int prevGameSel = -2;   // a game switch re-picks the first slot
    /* Both lists are directory scans: refresh at ~1 Hz while the window is open. A save
       or delete resets the timer, so your own action shows on the next frame. */
    static const double kRescanSec = 1.0;
    static double nextRescan = 0.0;
    static std::vector<GameEntry> games;
    static std::vector<std::string> slots;

    drawWin(g, W_SaveLoad, [&] {
        // Clamp BOTH bounds: a frame with an empty list parks the index at -1, and a later
        // `list[-1]` is an out-of-bounds read.
        const bool due = ImGui::GetTime() >= nextRescan;
        if(due) {
            nextRescan = ImGui::GetTime() + kRescanSec;
            games = list_games(datadir::saves());
        }
        if(gameSel < 0 || gameSel >= (int)games.size()) {
            gameSel = (int)games.size() - 1;
        }
        if(due || gameSel != prevGameSel) {
            slots.clear();
            if(gameSel >= 0) {
                slots = list_saves(datadir::saves() + "/" + games[gameSel].dirName);
            }
        }
        if(gameSel != prevGameSel) { slotSel = 0; prevGameSel = gameSel; }
        if(slotSel < 0 || slotSel >= (int)slots.size()) {
            slotSel = (int)slots.size() - 1;
        }

        // --- save-as: capture the live ships + crew + clock ----------------
        ImGui::TextWrapped(
            "Save the current game (%s) -- its ships, crew and clock -- "
            "into a slot of it.", g.gameName.c_str());
        ImGui::SetNextItemWidth(220);
        ImGui::InputText("##newslot", nameBuf, sizeof(nameBuf));
        ImGui::SameLine();
        // quicksave-NN is the F5 pool: a hand-saved slot under that name would join the
        // rotation and a wrap could clobber it.
        const bool nameOk = safeSlotName(nameBuf) && quicksaveNN(nameBuf) < 0;
        if(ImGui::Button("Save##saveload") && nameOk) {
            const std::string gamedir = g.ensureGameDir();
            if(gamedir.empty()) {
                g.toast("Save failed: no game directory");
            } else {
                const std::string dir = gamedir + "/" + nameBuf;
                try {
                    save_game(g, dir);
                    g.toast("Saved to %s/%s", g.gameName.c_str(), nameBuf);
                    nextRescan = 0.0;   // the new slot joins the list next frame
                } catch(const std::exception &e) {
                    g.toast("Save failed: %s", e.what());
                }
            }
        }
        if(!nameOk) {
            ImGui::SameLine();
            if(!safeSlotName(nameBuf)) {
                ImGui::TextDisabled("(name: letters, digits, - _ . only)");
            } else {
                ImGui::TextDisabled("(quicksave-NN is reserved for F5)");
            }
        }

        ImGui::Separator();

        // --- load / delete an existing game's slot --------------------------
        ImGui::Text("Existing games and their saves:");
        if(games.empty()) {
            ImGui::TextDisabled("(none yet)");
        } else {
            // Two half-width columns (games | slots of the selected game).
            const float full = ImGui::GetContentRegionAvail().x;
            const float w = (full - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
            ImGui::BeginChild("##games", ImVec2(w, 180.0f));
            for(size_t i = 0; i < games.size(); i++) {
                if(ImGui::Selectable(games[i].name.c_str(), (int)i == gameSel)) {
                    gameSel = (int)i;
                }
            }
            ImGui::EndChild();
            ImGui::SameLine();
            ImGui::BeginChild("##slots", ImVec2(w, 180.0f));
            if(slots.empty()) {
                ImGui::TextDisabled("(no saves in this game)");
            }
            for(size_t i = 0; i < slots.size(); i++) {
                if(ImGui::Selectable(slots[i].c_str(), (int)i == slotSel)) {
                    slotSel = (int)i;
                }
            }
            ImGui::EndChild();
            ImGui::Spacing();
            if(ImGui::Button("Load##saveload") && gameSel >= 0 && slotSel >= 0) {
                const std::string dir = datadir::saves() + "/" +
                                       games[gameSel].dirName + "/" + slots[slotSel];
                const std::string gameName = games[gameSel].name;
                const std::string slotName = slots[slotSel];
                /* Deferred to the top of the next frame (#304): the load deletes
                   the fleet immediately, and this sheet is drawn last in every
                   scene only by convention. loadFrom does the load, the scene
                   decision and the failure toast, and adopts the game's
                   identity so the next save lands here. */
                g.requestWorldOp([&g, dir, gameName, slotName] {
                    if(g.loadFrom(dir)) {
                        g.toast("Loaded %s/%s", gameName.c_str(), slotName.c_str());
                        setWinOpen(W_SaveLoad, false);
                    }
                });
            }
            ImGui::SameLine();
            if(ImGui::Button("Delete##saveload") && gameSel >= 0 && slotSel >= 0) {
                const std::string dir = datadir::saves() + "/" +
                                       games[gameSel].dirName + "/" + slots[slotSel];
                try {
                    delete_save(dir, datadir::saves());
                    g.toast("Deleted %s/%s", games[gameSel].name.c_str(),
                            slots[slotSel].c_str());
                    slotSel = 0;   // the slot list shrank under the selection
                    nextRescan = 0.0;   // and the deleted row goes away next frame
                } catch(const std::exception &e) {
                    g.toast("Delete failed: %s", e.what());
                }
            }
        }
    });
}

/* The VAB seat window: right-click a capsule in the 3D view (vabPickSeat,
   events.cpp) and one dropdown per seat appears, listing the kerbals standing
   on the pad. Choosing here is what makes a launch fill NAMED seats instead of
   drawing people off the pad FIFO (VabState::seatAssigns, game.h). Plain
   Begin/End, placed once with ImGuiCond_Appearing -- unlike the flight part
   windows, which now measure their size and re-place every frame (drawPartWindows);
   this one still stacks on itself when a capsule has several seats (#324).
   A user-placed popup, not a slot window.

   These dropdowns go through uiBeginCombo, so --ui-list and --ui-click can name
   them (#251 is closed); --vab-assign stays as the headless route only because
   the VAB's 3D view cannot be driven without it (#311). */
void drawVabSeatWindow(Game &g) {
    if(g.vab.seatPartId.empty()) { return; }
    const BuildPart *bp = nullptr;
    for(const BuildPart &p : g.vab.build.parts) {
        if(p.id == g.vab.seatPartId) { bp = &p; break; }
    }
    if(bp == nullptr || bp->def == nullptr || bp->def->crew_capacity <= 0) {
        g.vab.seatPartId.clear();   // the part was deleted or detached
        return;
    }

    char name[256];
    snprintf(name, sizeof(name), "Crew##%s", bp->id.c_str());
    if(!g.vab.seatPlaced) {
        // Near the click, flipping sides when there is no room (as drawPartWindows).
        const ImGuiViewport *vp = ImGui::GetMainViewport();
        const float maxw = 300.0f, maxh = 260.0f, gap = 12.0f;
        float x = (float)g.vab.seatMx + gap;
        float y = (float)g.vab.seatMy + gap;
        if(x + maxw > vp->WorkPos.x + vp->WorkSize.x) {
            x = (float)g.vab.seatMx - gap - maxw;
        }
        if(y + maxh > vp->WorkPos.y + vp->WorkSize.y) {
            y = (float)g.vab.seatMy - gap - maxh;
        }
        x = std::max(x, (float)vp->WorkPos.x);
        y = std::max(y, (float)vp->WorkPos.y);
        ImGui::SetNextWindowPos(ImVec2(x, y), ImGuiCond_Appearing);
        g.vab.seatPlaced = true;
    }

    StaticBuilding *pad = homeCrewPad(g.sys);
    bool open = true;
    if(ImGui::Begin(name, &open, ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::Text("%s  (%s)", bp->def->label().c_str(), bp->id.c_str());
        ImGui::Text("%d kerbal(s) at the launch pad",
                    pad != nullptr ? (int)pad->crew.size() : 0);
        ImGui::Separator();
        for(int slot = 0; slot < bp->def->crew_capacity; slot++) {
            uint64_t cur = 0;
            for(const VabState::SeatAssign &a : g.vab.seatAssigns) {
                if(a.partId == bp->id && a.slot == slot) { cur = a.profileId; }
            }
            const KerbalProfile *cp = cur != 0 ? g.roster.find(cur) : nullptr;
            char preview[128];
            snprintf(preview, sizeof(preview), "%s",
                     cp != nullptr ? cp->name.c_str() : "Empty");
            char lbl[64];
            snprintf(lbl, sizeof(lbl), "Seat %d##s%d", slot + 1, slot);
            ImGui::SetNextItemWidth(-1.0f);
            if(uiBeginCombo(lbl, preview)) {
                if(ImGui::Selectable("Empty", cur == 0)) {
                    vabAssignSeat(g, bp->id, slot, 0);
                }
                if(pad != nullptr) {
                    for(const Part *suit : pad->crew) {
                        /* A save predating the roster leaves a suit anonymous
                           (seat_profile 0, save.cpp). The plan is keyed by
                           roster id, so there is nothing to name here -- and
                           offering id 0 would collide with the empty-seat
                           marker, which is also 0. */
                        if(suit->kerbal == nullptr || suit->kerbal->id == 0) {
                            continue;
                        }
                        const KerbalProfile &kp = *suit->kerbal;
                        /* Already in another seat of THIS build: naming one
                           person twice makes the launch refuse, so grey it out
                           instead of offering the refusal. */
                        bool seated = false;
                        for(const VabState::SeatAssign &a : g.vab.seatAssigns) {
                            if(a.profileId == kp.id
                               && !(a.partId == bp->id && a.slot == slot)) {
                                seated = true;
                                break;
                            }
                        }
                        char item[160];
                        snprintf(item, sizeof(item), "%s%s", kp.name.c_str(),
                                 seated ? "  (seated)" : "");
                        if(seated) { ImGui::BeginDisabled(); }
                        if(ImGui::Selectable(item, kp.id == cur)) {
                            vabAssignSeat(g, bp->id, slot, kp.id);
                        }
                        if(seated) { ImGui::EndDisabled(); }
                    }
                }
                ImGui::EndCombo();
            }
        }
    }
    ImGui::End();
    if(!open) { g.vab.seatPartId.clear(); }
}

void drawVabUI(Game &g) {
    /* TAB. The top bar is a registered window and drawWin already suppresses it, but the
       two panels below are raw ImGui::Begin calls -- not in the window table -- so this is
       their only gate. */
    if(!g.ui_visible) { return; }

    int hoveredLink = -1;   // the fuel-link list row under the mouse (overlay reads it)

    /* Free-port gizmos: a green dot at every unconsumed stack node, projected to window
       pixels (projectPixel). Drawn on the foreground layer so the far-side ports stay
       discoverable. */
    if(g.camera != nullptr) {
        ImDrawList *dl = ImGui::GetForegroundDrawList();
        for(size_t i = 0; i < g.vab.build.parts.size(); i++) {
            const BuildPart &bp = g.vab.build.parts[i];
            if(bp.def == nullptr) { continue; }
            for(size_t k = 0; k < bp.def->nodes.size(); k++) {
                const Node &n = bp.def->nodes[k];
                if(n.surface) { continue; }
                if(g.vab.build.nodeOccupied((int)i, n.id)) { continue; }
                double px = 0, py = 0;
                if(!projectPixel(g, vabNodePos(g, (int)i, (int)k), px, py)) { continue; }
                const ImVec2 c((float)px, (float)py);
                dl->AddCircleFilled(c, 4.0f, IM_COL32(90, 230, 140, 210));
                dl->AddCircle(c, 4.0f, IM_COL32(15, 60, 30, 255), 0, 1.5f);
            }
        }
    }

    // The save NAME (not a path -- .json and where the file lives are vabSave's business).
    static char saveName[128] = {0};
    if(saveName[0] == 0) {
        snprintf(saveName, sizeof(saveName), "%s",
                 g.vab.build.name.empty() ? "untitled" : g.vab.build.name.c_str());
    }

    // The load picker: ship NAMES (stock + the player's data-dir ships/, data dir wins).
    // Testships filtered out. Cached, re-read on mtime change.
    static std::vector<std::string> shipNames;
    static bool shipsScanned = false;   // a real mtime could be the epoch; don't rely on that
    static std::filesystem::file_time_type stockMtime, userMtime;
    {
        std::error_code ec;
        const std::string stockDir = resdir::path("res/ships");
        const std::string userDir = datadir::ships();
        const auto sm = std::filesystem::last_write_time(stockDir, ec);
        const std::filesystem::file_time_type stock =
            ec ? std::filesystem::file_time_type{} : sm;
        const auto um = std::filesystem::last_write_time(userDir, ec);
        const std::filesystem::file_time_type user =
            ec ? std::filesystem::file_time_type{} : um;
        if(!shipsScanned || stock != stockMtime || user != userMtime) {
            shipNames = list_vab_ship_defs(stockDir);
            for(const std::string &s : list_vab_ship_defs(userDir)) {
                if(std::find(shipNames.begin(), shipNames.end(), s) == shipNames.end()) {
                    shipNames.push_back(s);
                }
            }
            std::sort(shipNames.begin(), shipNames.end());
            stockMtime = stock;
            userMtime = user;
            shipsScanned = true;
        }
    }
    static int loadSel = 0;
    if(loadSel >= (int)shipNames.size()) { loadSel = (int)shipNames.size() - 1; }
    if(loadSel < 0) { loadSel = 0; }   // an empty list clamps to -1 above; keep valid

    /* Top bar: a fixed top-center window mirroring the HUD, in two lines: Back to game /
       save name / Save / ship picker + Load, then the launch body + scenario dropdowns and
       LAUNCH. */
    drawWin(g, W_VabTopBar, [&] {
        if(ImGui::Button("Back to game##vab")) { vabClose(g); }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(140);
        ImGui::InputText("##savename", saveName, sizeof(saveName));
        ImGui::SameLine();
        if(ImGui::Button("Save")) { vabSave(g, saveName); }
        // Load: replace the build with a named ship and point the save name at it, so a
        // following Save round-trips it.
        if(!shipNames.empty()) {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(160);
            const char *cur = shipNames[(size_t)loadSel].c_str();
            if(uiBeginCombo("##vabload", cur)) {
                for(size_t i = 0; i < shipNames.size(); i++) {
                    if(ImGui::Selectable(shipNames[i].c_str(), (int)i == loadSel)) {
                        loadSel = (int)i;
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::SameLine();
            if(ImGui::Button("Load")) {
                const std::string &nm = shipNames[(size_t)loadSel];
                if(vabLoad(g, nm.c_str())) {
                    snprintf(saveName, sizeof(saveName), "%s", nm.c_str());
                }
            }
        } else {
            ImGui::SameLine();
            ImGui::TextDisabled("(no ships)");
        }

        // Line 2: where + how to launch (vabLaunch resolves both). Pad scenarios also pick a
        // launch SITE.
        ImGui::SetNextItemWidth(160);
        if(uiBeginCombo("##vabbody", g.vab.bodyName.c_str())) {
            for(size_t i = 0; i < g.sys.bodies.size(); i++) {
                const char *nm = g.sys.bodies[i]->name.c_str();
                if(ImGui::Selectable(nm, g.vab.bodyName == nm)) {
                    g.vab.bodyName = nm;
                    // A site id is only valid on one body; drop a stale pick.
                    g.vab.siteName.clear();
                }
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(160);
        if(uiBeginCombo("##vabscn", g.vab.scenarioName.c_str())) {
            for(size_t i = 0; i < scenario_count(); i++) {
                const char *nm = scenario_name_at(i);
                /* Dev beds are hidden without --dev-scenarios. A dev bed armed
                   at boot by --vab-scenario keeps its label on the combo but
                   cannot be re-picked: switching away drops it for good. */
                if(!g.args.dev_scenarios && scenario_dev_only_at(i)) { continue; }
                if(ImGui::Selectable(nm, g.vab.scenarioName == nm)) { g.vab.scenarioName = nm; }
            }
            ImGui::EndCombo();
        }
        // Launch site: only meaningful for pad scenarios. Body-filtered.
        {
            const ScenarioDef *scn = nullptr;
            try { scn = scenario_by_name(g.vab.scenarioName.empty() ? "pad"
                                         : g.vab.scenarioName); }
            catch(...) { scn = nullptr; }
            if(scn != nullptr && scn->on_pad) {
                TerrainBody *hb = g.home;
                if(!g.vab.bodyName.empty()) {
                    if(TerrainBody *b = g.sys.find(g.vab.bodyName)) { hb = b; }
                }
                std::string siteLabel = g.vab.siteName.empty() ? "(default)" : g.vab.siteName;
                if(hb != nullptr) {
                    for(auto *p : hb->pads) {
                        if(p->id == g.vab.siteName) { siteLabel = p->display_name; break; }
                    }
                }
                ImGui::SameLine();
                ImGui::SetNextItemWidth(160);
                if(uiBeginCombo("##vabsite", siteLabel.c_str())) {
                    if(hb != nullptr) {
                        for(auto *p : hb->pads) {
                            if(!p->hasFlag("VAB_LAUNCH_PAD")) { continue; }
                            // Unique id: two pads may share a display name.
                            ImGui::PushID(p->id.c_str());
                            if(ImGui::Selectable(p->display_name.c_str(),
                                                 g.vab.siteName == p->id)) {
                                g.vab.siteName = p->id;
                            }
                            ImGui::PopID();
                        }
                    }
                    ImGui::EndCombo();
                }
            }
        }
        ImGui::SameLine();
        if(ImGui::Button("LAUNCH")) { vabLaunch(g); }
    });

    ImGui::SetNextWindowPos(ImVec2(8, 8), ImGuiCond_Once);
    // Resizable: default size at creation, then the user owns the size.
    ImGui::SetNextWindowSize(ImVec2(340, 410), ImGuiCond_FirstUseEver);
    ImGui::Begin("VAB", nullptr);
    ImGui::Text("VAB -- %s (%d parts)", g.vab.build.name.c_str(),
                (int)g.vab.build.parts.size());
    // Parts are selected by clicking them in the 3D view; the panel below acts on that
    // selection.
    if(g.vab.selected >= 0 && (size_t)g.vab.selected < g.vab.build.parts.size()) {
        ImGui::Separator();
        const int sel = g.vab.selected;
        BuildPart &bp = g.vab.build.parts[(size_t)sel];
        const char *selName = bp.def != nullptr ? bp.def->label().c_str() : "?";
        ImGui::Text("selected: %s (%s)", bp.id.c_str(), selName);
        if(sel == 0) {
            ImGui::TextDisabled("root -- cannot rotate or delete");
        } else {
            const bool srf = (bp.attach == AttachMode::Surface);
            if(ImGui::Button("-5##rot")) { g.vab.build.rotatePart(sel, -5.0); }
            ImGui::SameLine();
            if(ImGui::Button("+5##rot")) { g.vab.build.rotatePart(sel, +5.0); }
            ImGui::SameLine();
            ImGui::Text("%s roll: %.0f deg", srf ? "surface" : "stack",
                        srf ? bp.roll : bp.angle);
            int st = bp.stage;
            ImGui::SetNextItemWidth(60);
            if(ImGui::InputInt("stage##sel", &st, 1, 1)) {
                if(st < 1) { st = 1; }
                bp.stage = st;
            }
            // The vab ops swap the parts vector (the bp reference dies with it).
            if(ImGui::Button("Detach subtree")) { vabDetachSelected(g); }
            ImGui::SameLine();
            if(ImGui::Button("Delete##sel")) { vabDeleteSelected(g); }
        }
    }
    ImGui::Separator();
    /* Fuel links: virtual from->to edges (no pose, no stage), added with a two-click 3D
       pick. */
    if(ImGui::Button(g.vab.linkMode ? "Cancel fuel link" : "Add fuel link")) {
        g.vab.linkMode = !g.vab.linkMode;
        g.vab.linkFromId.clear();
    }
    if(g.vab.linkMode) {
        if(g.vab.linkFromId.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f),
                               "click the SOURCE part (fuel flows out of it)");
        } else {
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f),
                               "%s feeds ... click the DESTINATION",
                               g.vab.linkFromId.c_str());
        }
    }
    for(size_t i = 0; i < g.vab.build.fuelLinks.size(); i++) {
        const BuildShip::FuelLink &fl = g.vab.build.fuelLinks[i];
        char label[256];
        snprintf(label, sizeof(label), "%s -> %s##fl%d", fl.from.c_str(),
                 fl.to.c_str(), (int)i);
        const bool sel = (g.vab.linkSel == (int)i);
        if(ImGui::Selectable(label, sel)) {
            g.vab.linkSel = sel ? -1 : (int)i;
            g.vab.selected = -1;
        }
        if(ImGui::IsItemHovered()) { hoveredLink = (int)i; }
    }
    if(g.vab.linkSel >= 0 && (size_t)g.vab.linkSel < g.vab.build.fuelLinks.size()) {
        if(ImGui::Button("Delete link")) { vabDeleteSelected(g); }
    }
    ImGui::Separator();
    /* Subassemblies: multi-part subtrees detached instead of deleted (Del); a lone part
       just deletes. Arming one places COPIES of the whole tree. */
    ImGui::Text("Subassemblies");
    int dropAsm = -1;
    for(size_t i = 0; i < g.vab.subassemblies.size(); i++) {
        const VabState::Subassembly &sa = g.vab.subassemblies[i];
        char label[256];
        snprintf(label, sizeof(label), "%s (%d parts)##asm%zu", sa.name.c_str(),
                 (int)sa.ship.parts.size(), i);
        const bool armed = (g.vab.armedAsm == (int)i);
        if(ImGui::Selectable(label, armed)) {
            if(armed) {
                g.vab.armedAsm = -1;
            } else {
                g.vab.armedAsm = (int)i;
                g.vab.armed.clear();      // exclusive with a catalog part
                g.vab.ghostRoll = 0.0;
            }
        }
        ImGui::SameLine();
        char xl[32];
        snprintf(xl, sizeof(xl), "x##asm%zu", i);
        if(ImGui::SmallButton(xl)) { dropAsm = (int)i; }
    }
    if(dropAsm >= 0) {   // erase AFTER the loop (indices drive the widgets)
        g.vab.subassemblies.erase(g.vab.subassemblies.begin() + dropAsm);
        if(g.vab.armedAsm == dropAsm) { g.vab.armedAsm = -1; }
        else if(g.vab.armedAsm > dropAsm) { g.vab.armedAsm--; }
    }
    if(g.vab.subassemblies.empty()) {
        ImGui::TextDisabled("select a part with children -> Detach (Del)");
    } else if(g.vab.armedAsm >= 0) {
        ImGui::TextColored(ImVec4(0.6f, 1.0f, 0.6f, 1.0f),
                           "placing copies -- hover a port/surface, LMB; Esc stops");
    }
    ImGui::End();

    // Palette: arm a catalog part, then hover the ship and LMB to place it (snaps to the
    // hovered stack port, or surface-attaches). Hover/ghost targeting comes from the 3D
    // pick, so list hover must not overwrite g.vab.hover.
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - 290, 8),
                            ImGuiCond_Once);
    // Resizable: default size at creation, then the user owns it. A negative child height
    // is an offset from the bottom edge. The list is reserved at its max so the window
    // needs no scrollbar.
    ImGui::SetNextWindowSize(ImVec2(280, 660), ImGuiCond_FirstUseEver);
    ImGui::Begin("Palette", nullptr);
    const float below = ImGui::GetStyle().ItemSpacing.y * 9.0f + 1.0f
        + ImGui::GetTextLineHeight()
        + ImGui::GetFrameHeight() * 3.0f
        + ImGui::GetTextLineHeight() * 4.0f;
    ImGui::BeginChild("palette_parts", ImVec2(0.0f, -below), true);
    for(size_t i = 0; i < g.parts.parts.size(); i++) {
        const PartDef &pd = g.parts.parts[i];
        if(pd.fuel_link) { continue; }
        /* A kerbal is a person, not a component: it belongs to a seat or to the
           roster, never to a build. Offering it lets a player root a ship on one
           and reach a state the spawner refuses (a Kerbal vehicle is one part). */
        if(pd.type == "kerbal") { continue; }
        const bool armed = (g.vab.armed == pd.name);
        // Display name (falling back to the machine id); the ##id keeps the ImGui id unique.
        char lbl[256];
        snprintf(lbl, sizeof(lbl), "%s##%s",
                 pd.label().c_str(),
                 pd.name.c_str());
        if(ImGui::Selectable(lbl, armed)) {
            g.vab.armed = armed ? std::string("") : pd.name;
            g.vab.armedAsm = -1;     // exclusive with a subassembly
            g.vab.ghostRoll = 0.0;   // a fresh part starts unrolled
        }
    }
    ImGui::EndChild();
    ImGui::Separator();
    // Radial symmetry for SURFACE placing: N copies ringing the hovered parent's long
    // axis. Stack ports are singletons, so it does not apply there.
    ImGui::Text("Symmetry");
    for(int n = 1; n <= 8; n++) {
        if(n > 1) { ImGui::SameLine(0, 3); }
        char lbl[12];
        snprintf(lbl, sizeof(lbl), "%d##sym", n);
        if(ImGui::Selectable(lbl, g.vab.symmetry == n, 0, ImVec2(21, 0))) {
            g.vab.symmetry = n;
        }
    }
    ImGui::Checkbox("Snap distance 10cm (Alt bypasses)", &g.vab.snapLen);
    ImGui::Checkbox("Snap angle 10deg (Alt bypasses)", &g.vab.snapAng);
    const bool asmArmed = g.vab.armedAsm >= 0
        && (size_t)g.vab.armedAsm < g.vab.subassemblies.size();
    if(asmArmed || !g.vab.armed.empty()) {
        if(asmArmed) {
            ImGui::Text("armed: %s",
                        g.vab.subassemblies[(size_t)g.vab.armedAsm].name.c_str());
        } else {
            // g.vab.armed holds the catalog id; show its display name.
            const PartDef *ad = g.parts.find(g.vab.armed);
            ImGui::Text("armed: %s",
                        ad != nullptr ? ad->label().c_str()
                                      : g.vab.armed.c_str());
        }
        if(g.vab.ghostValid) {
            ImGui::Text("roll: %.0f deg (Q/E)", g.vab.ghostRollUsed);
            if(g.vab.symmetry > 1) {
                if(g.vab.ghostSurface) {
                    ImGui::Text("placing x%d around the parent axis",
                                1 + (int)g.vab.ghostClones.size());
                } else {
                    ImGui::TextDisabled("stack port: symmetry n/a");
                }
            }
        }
        if(g.vab.build.parts.empty()) {
            ImGui::TextDisabled("LMB: anchor the ROOT at the origin");
        } else {
            ImGui::TextDisabled("hover a port/surface, LMB to place");
        }
    } else {
        ImGui::TextDisabled("pick a part to arm");
    }
    ImGui::End();

    // The seat window, if a capsule was right-clicked (vabPickSeat).
    drawVabSeatWindow(g);

    /* Staging table: one row per stage period (flight order), with vacuum delta-v and TWR
       against the SURFACE GRAVITY OF THE BODY THIS BUILD LAUNCHES FROM (fuel links
       honoured). The gravity follows the top bar's launch-body dropdown, not home: the
       min-TWR "cannot lift off" verdict belongs to the launch body. */
    drawWin(g, W_Staging, [&] {
        TerrainBody *lbody = g.sys.home;
        if(!g.vab.bodyName.empty()) {
            if(TerrainBody *b = g.sys.find(g.vab.bodyName)) { lbody = b; }
        }
        const double gLaunch = (lbody != nullptr) ? lbody->g : 9.81;
        ImGui::Text("TWR on %s  (g = %.2f m/s^2)",
                    lbody != nullptr ? lbody->name.c_str() : "?",
                    gLaunch);
        if(g.vab.build.parts.empty()) {
            ImGui::TextDisabled("empty build -- place a part");
            return;
        }
        /* Recompute only when the build changed (#293): this redraws every
           frame, and the burn simulation is the most expensive thing in the
           editor. The signature hashes exactly what computeStaging reads --
           part ids, defs, parents, stages, fuel-link endpoints, launch g,
           exhaust scale -- so a place, delete, detach, re-parent, a stage
           number or a link misses the cache and refreshes the rows on the
           spot, while an edit that cannot move a number (a rotate, which only
           touches angle and roll) correctly does not. */
        const uint64_t sig = stagingSignature(g.vab.build, gLaunch, g.args.exhaust_scale);
        if(g.vab.stagingSig != sig) {
            g.vab.stagingSig = sig;
            g.vab.stagingRows = computeStaging(g.vab.build, gLaunch, g.args.exhaust_scale);
        }
        const std::vector<StageRow> &rows = g.vab.stagingRows;
        if(rows.empty()) {
            ImGui::TextDisabled("nothing to stage");
            return;
        }
        if(ImGui::BeginTable("staging_table", 5,
                             ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
                             | ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableSetupColumn("Stage", ImGuiTableColumnFlags_WidthFixed, 48.0f);
            ImGui::TableSetupColumn("dv m/s", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("min TWR", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("max TWR", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("mass kg", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();
            double totalDv = 0.0;
            for(size_t i = 0; i < rows.size(); i++) {
                const StageRow &r = rows[i];
                totalDv += r.deltaV;
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("%d%s", r.stage, r.drops ? "" : " *");
                ImGui::TableNextColumn();
                ImGui::Text("%.0f", r.deltaV);
                ImGui::TableNextColumn();
                if(r.minTWR > 0.0) {
                    // Red when the stack cannot lift off the pad.
                    if(r.minTWR < 1.0) {
                        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.35f, 1.0f),
                                           "%.2f", r.minTWR);
                    } else {
                        ImGui::Text("%.2f", r.minTWR);
                    }
                } else {
                    ImGui::TextDisabled("--");
                }
                ImGui::TableNextColumn();
                if(r.maxTWR > 0.0) {
                    ImGui::Text("%.2f", r.maxTWR);
                } else {
                    ImGui::TextDisabled("--");
                }
                ImGui::TableNextColumn();
                ImGui::Text("%.0f -> %.0f", r.massStart, r.massEnd);
            }
            ImGui::EndTable();
            ImGui::Text("total dv: %.0f m/s", totalDv);
            ImGui::TextDisabled("* final burn (no separation)");
        }
    });

    /* Fuel-link overlay lines (drawn last, foreground layer: always on top, no depth
       test). Flow reads from a dimmed source half, a bright destination half, an arrowhead. */
    if(g.camera != nullptr && !g.vab.build.fuelLinks.empty()) {
        ImDrawList *dl = ImGui::GetForegroundDrawList();
        for(size_t i = 0; i < g.vab.build.fuelLinks.size(); i++) {
            const BuildShip::FuelLink &fl = g.vab.build.fuelLinks[i];
            const BuildPart *from = nullptr, *to = nullptr;
            for(size_t k = 0; k < g.vab.build.parts.size(); k++) {
                if(g.vab.build.parts[k].id == fl.from) { from = &g.vab.build.parts[k]; }
                if(g.vab.build.parts[k].id == fl.to)   { to = &g.vab.build.parts[k]; }
            }
            if(from == nullptr || to == nullptr) { continue; }
            double ax = 0, ay = 0, bx = 0, by = 0;
            if(!projectPixel(g, from->localPos, ax, ay)) { continue; }
            if(!projectPixel(g, to->localPos, bx, by)) { continue; }
            const ImVec2 A((float)ax, (float)ay), B((float)bx, (float)by);
            const ImVec2 mid((A.x + B.x) * 0.5f, (A.y + B.y) * 0.5f);
            const float len = sqrtf((B.x - A.x) * (B.x - A.x)
                                    + (B.y - A.y) * (B.y - A.y));
            const bool hl = (g.vab.linkSel == (int)i) || (hoveredLink == (int)i);
            const ImU32 dimC = hl ? IM_COL32(120, 200, 255, 130)
                                  : IM_COL32(255, 190, 60, 80);
            const ImU32 litC = hl ? IM_COL32(150, 215, 255, 255)
                                  : IM_COL32(255, 200, 60, 220);
            const float th = hl ? 3.0f : 2.0f;
            dl->AddLine(A, mid, dimC, th);
            dl->AddLine(mid, B, litC, th);
            if(len > 24.0f) {
                const ImVec2 d((B.x - A.x) / len, (B.y - A.y) / len);
                const ImVec2 p(-d.y, d.x);
                dl->AddTriangleFilled(
                    ImVec2(mid.x + d.x * 10.0f, mid.y + d.y * 10.0f),
                    ImVec2(mid.x - d.x * 2.0f + p.x * 6.0f,
                           mid.y - d.y * 2.0f + p.y * 6.0f),
                    ImVec2(mid.x - d.x * 2.0f - p.x * 6.0f,
                           mid.y - d.y * 2.0f - p.y * 6.0f),
                    litC);
            }
        }
    }
}

// ---- Tracking Station windows --------------------------------------------
// Copies of the flight Ship List and Orbital Map, each renamed to its own window id so
// the Tracking versions can diverge.

void drawTrackingShipList(Game &g) {
    Vehicle *ship = g.ship;
    std::vector<Vehicle *> &vehicles = g.vehicles;
    drawWin(g, W_TrackingShipList, [&] {
    // Back to the hub (the frame below); Esc does the same (trackingKeyActions).
    if(ImGui::Button("Back to Space Center")) {
        popScene(g);
    }
    ImGui::Separator();
    /* Walked by index because a row's "x" erases from the live list. The loop
       stops the moment a row really goes away (see `removed` below), so no index
       is reused after the erase. */
    bool removed = false;
    for(size_t i = 0; i < vehicles.size() && !removed; i++) {
        Vehicle *v = vehicles[i];
        const bool active = (v == ship);
        ImGui::PushID((void*)v);
        /* Every row is a controllable vessel: a seated kerbal is a seat on its
           capsule's Part (part.h), not a vehicle. */
        if(active) {
            ImGui::PushStyleColor(ImGuiCol_Button,
                                 ImVec4(0.30f, 0.45f, 0.70f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                                 ImVec4(0.35f, 0.50f, 0.75f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                                 ImVec4(0.40f, 0.55f, 0.80f, 1.0f));
        }
        if(ImGui::Button(v->name.c_str())) {
            g.select_vehicle(v);
        }
        if(active) {
            ImGui::PopStyleColor(3);
        }
        if(!v->isEva()) {
            ImGui::SameLine();
            char rm_id[256];
            snprintf(rm_id, sizeof(rm_id), "x##rm_%s", v->name.c_str());
            if(ImGui::SmallButton(rm_id)) {
                /* Only a real removal stops the list (and hides "Fly"):
                   a refusal toasts and leaves the row in place. */
                removed = g.ui_remove_vehicle(v);
            }
        }
        if(active && !removed) {
            // Fly: only drawn for the active ship, so enterFlight just collapses the stack to the
            // live flight (syncShipFocus does the re-centering).
            ImGui::SameLine();
            if(ImGui::SmallButton("Fly")) {
                enterFlight(g);
                g.syncShipFocus();
            }
        }
        ImGui::PopID();
    }
    ImGui::Separator();
    ImGui::Text("click name - select    x - remove (not crew)");
    });
}

void drawTrackingMap(Game &g) {
    TransferPlanner &planner = g.xferPlanner;
    Vehicle *ship = g.ship;
    OrbitElements &o = g.hud.view.o;
    double &mu = g.hud.view.mu;
    glm::dvec3 &orbit_pos = g.hud.view.orbit_pos;
    glm::dvec3 &orbit_vel = g.hud.view.orbit_vel;
    float &map_scale = g.map.scale;
    int &map_plane = g.map.plane;
    ImVec2 &map_pan = g.map.pan;
    std::vector<TransferPlanner::XferTarget> &xferTargets = planner.xferTargets;
    int &xfer_target = planner.xfer_target;

    /* Full-screen and chrome-less: the map IS the Tracking Station view (no controls --
       they would overflow the auto-fit window off-screen). Diverged from drawUIMap on
       purpose: this one never touches the shared g.map.mode. */
    const ImGuiViewport *tvp = ImGui::GetMainViewport();
    // Edge-to-edge: cancel the slot's margin and zero the window padding.
    const ImVec2 vmarg = ui::Manager::Get().margin;
    ui::Options mapOpts;
    mapOpts.slot = ui::Slot::TopLeft;
    mapOpts.offset = ImVec2(-vmarg.x, -vmarg.y);
    mapOpts.fixed = true;   // re-placed every frame; not movable or resizable
    mapOpts.default_open = true;
    mapOpts.flags = ImGuiWindowFlags_NoDecoration |
                    ImGuiWindowFlags_NoBringToFrontOnFocus;
    // Width and height INDEPENDENTLY (not a square), so a wide screen is covered edge to
    // edge.
    const float mapW = tvp->WorkSize.x;
    const float mapH = tvp->WorkSize.y;
    // Zero padding + border and an opaque BLACK canvas: the Tracking map is its own
    // backdrop and meets the loop's black Sky clear, so the auto-fit window's shortfall
    // shows no seam. Pushed before drawWin so contrastingColor(WindowBg) picks light ink.
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 1.0f));
    drawWin(g, W_TrackingMap, mapOpts, [&] {
        // The focus body: the ship's parent when there is one, else home (a new game before
        // its first launch). Ship-specific bits are absent then.
        TerrainBody *focus = ship ? ship->m_parent : g.home;

        // Empty with no ship.
        bool closed = false;
        std::vector<glm::dvec3> traj_local;
        const std::vector<glm::dvec3> *traj_pts = nullptr;
        glm::dvec3 peri_p, apo_p, tmp;
        bool have_peri = false, have_apo = false;
        if(ship) {
            closed = (o.ecc < 1.0);
            if(closed) {
                // Per-ship cache, trusted only while on rails; off rails re-sample every frame.
                // Fixed N: shared with the Surface Map.
                const int N = 64;
                traj_pts = &orbit_caches[(const void *)ship].sample(
                    orbit_pos, orbit_vel, mu, N, ship->onRails);
            } else {
                const double r_cap = std::max<double>(
                    (double)std::max(mapW, mapH) * map_scale,
                    std::max(4.0 * o.periapsis, o.distance));
                const int N = 64;
                traj_local = sampleOpenTrajectory(orbit_pos, orbit_vel, mu, N, r_cap);
                traj_pts = &traj_local;
            }
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
        }

        // The map plane: a normal in the focus's inertial frame. "Orbital" needs a ship, so
        // without one it stays on the default (rail) plane; "Target orbit" needs one too, and
        // without a ship the view's stale target normal must not leak into the basis.
        glm::dvec3 plane_n, plane_x;
        mapPlaneBasis(focus->frame, orbit_pos, orbit_vel, ship != nullptr,
                      map_plane, ship ? g.hud.view.target_h_hat : glm::dvec3(0.0),
                      plane_n, plane_x);

        // The map fills the window (the whole viewport); the focus sits at its centre plus
        // the pan offset.
        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        const float center_x = p0.x + mapW * 0.5f;
        const float center_y = p0.y + mapH * 0.5f;

        ImGui::InvisibleButton("##mapnav", ImVec2(mapW, mapH));
        const bool over_map = ImGui::IsItemHovered();
        const ImGuiIO &g_io = ImGui::GetIO();
        if(over_map && g_io.MouseWheel != 0.0f) {
            // Wheel zooms to the cursor; wheel UP zooms IN (scale = m/px goes down).
            const float factor = (g_io.MouseWheel > 0.0f) ? 0.8f : 1.25f;
            const float old_scale = map_scale;
            float new_scale = old_scale * factor;
            if(new_scale < kMapMinScale) { new_scale = kMapMinScale; }
            if(new_scale > kMapMaxScale) { new_scale = kMapMaxScale; }
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

        OrbitMap proj;
        proj.cx = center_x + map_pan.x;
        proj.cy = center_y + map_pan.y;
        proj.scale = map_scale;
        proj.setPlane(plane_n, plane_x);

        const ImVec4 bg = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
        const ImU32 ink       = contrastingColor(bg);
        const ImU32 col_ship  = ImGui::GetColorU32(ImVec4(0.20f, 0.80f, 0.40f, 1.0f));
        const ImU32 col_apsis = col_ship;  // periapsis / apoapsis: part of your orbit
        const ImU32 col_vessel = ImGui::GetColorU32(ImVec4(1.00f, 0.62f, 0.22f, 1.0f));
        const ImU32 col_child = ImGui::GetColorU32(ImVec4(0.55f, 0.55f, 0.55f, 1.0f));
        const ImU32 col_body  = ink;
        const ImU32 col_sel   = ImGui::GetColorU32(ImVec4(0.90f, 0.90f, 0.90f, 1.0f));
        const ImU32 soi_col   = ImGui::GetColorU32(ImVec4(0.50f, 0.50f, 0.50f, 0.30f));
        // The near-body shell ring (science + surface-frame boundary), gray like the SOI.
        const ImU32 shell_col = ImGui::GetColorU32(ImVec4(0.50f, 0.50f, 0.50f, 0.35f));
        const ImU32 col_atmo = ImGui::GetColorU32(ImVec4(0.35f, 0.50f, 0.66f, 0.20f));
        ImDrawList *dl = ImGui::GetWindowDrawList();
        const ImVec2 focus_px = proj.px(glm::dvec3(0.0, 0.0, 0.0));

        TerrainBody *sel_body = nullptr;
        if(xfer_target >= 0 && xfer_target < (int)xferTargets.size() &&
           xferTargets[xfer_target].body) {
            sel_body = xferTargets[xfer_target].body;
        }

        // SOI ring, faint. Skipped when sub-pixel or far off-view.
        auto draw_soi = [&](const glm::dvec3 &center, double soi_m, ImU32 col) {
            if(soi_m <= 0.0) { return; }
            const float r_px = (float)(soi_m / map_scale);
            if(r_px < 1.0f || r_px > 4000.0f) { return; }
            proj.drawRing(dl, center, soi_m, col, 1.0f);
        };

        // Drawn before the ship's orbit so the ship sits on top.
        {
            const MapViewRect view{p0.x, p0.y, p0.x + mapW, p0.y + mapH};
            // Belts go down first: orbits, bodies and labels should read through them.
            drawDebrisBelts(g, focus, proj, dl, view, map_scale);
            drawSystemBodyOrbits(g, focus, proj, dl, view, map_scale,
                                 sel_body, col_child, col_sel, ink,
                                 soi_col);
        }
        // The focus body's own SOI -- the gravitational regime in play.
        draw_soi(glm::dvec3(0.0, 0.0, 0.0), focus->frame->soi, soi_col);
        draw_soi(glm::dvec3(0.0, 0.0, 0.0), focus->rot_frame->soi, shell_col);

        const double atmo_top = focus->surface.atmosphere.top();
        if(atmo_top > 0.0) {
            proj.drawBody(dl, glm::dvec3(0.0, 0.0, 0.0),
                         focus->radius + atmo_top, col_atmo, 3.0f);
        }
        proj.drawBody(dl, glm::dvec3(0.0, 0.0, 0.0), focus->radius, col_body, 3.0f);

        // The ship itself: its orbit (open arc uses closed=false so a chord does not close
        // it), a dot with a green ring, the prograde arrow and the apside markers.
        if(ship) {
            proj.drawOrbit(dl, *traj_pts, col_ship, 1.0f, closed);
            const ImVec2 ship_px = proj.px(orbit_pos);
            dl->AddLine(focus_px, ship_px, ink, 1.0f);
            dl->AddCircleFilled(ship_px, 5.0f, ink);
            dl->AddCircle(ship_px, 8.0f, col_ship, 0, 1.5f);
            // Prograde (velocity) arrow.
            proj.drawArrow(dl, orbit_pos, orbit_vel, 24.0f, col_ship, 1.5f);
            // Apside markers only mean anything for a non-circular orbit.
            if(o.ecc > 1e-3) {
                if(have_peri) { proj.drawDot(dl, peri_p, 4.0f, col_apsis); }
                if(have_apo)  { proj.drawDot(dl, apo_p,  4.0f, col_apsis); }
            }
            drawMapNodes (proj, dl, o, mu);

            // Every ship in the system -- any SOI: a Kepler conic around its OWN central body,
            // projected into the focus's frame. An escaping ship has no closed orbit -- sample()
            // returns empty -- so only its marker shows.
            for(auto *s : g.vehicles) {
                if(s == ship || !s->frame) { continue; }
                Frame *inertial = s->frame->getNonRotFrame();
                const double mu_c = inertial ? inertial->body->mu : 0.0;
                if(!inertial || mu_c <= 0.0) { continue; }
                // The ship's state in its central body's inertial frame.
                const glm::dvec3 tcom = s->get_center_of_mass();
                const glm::dmat3 Oc = s->frame->GetOrientRelTo(inertial);
                const glm::dvec3 r2 = Oc * tcom
                                     + s->frame->GetPositionRelTo(inertial);
                const glm::dvec3 v2 = Oc * (s->GetVel()
                                           + s->frame->GetStasisVelocity(tcom))
                                     + s->frame->GetVelocityRelTo(inertial);
                const int N = 64;
                const std::vector<glm::dvec3> &tpts_c =
                    orbit_caches[(const void *)s].sample(
                        r2, v2, mu_c, N, s->onRails);
                // Into the focus's frame (central body -> focus).
                const glm::dmat3 O = inertial->GetOrientRelTo(focus->frame);
                const glm::dvec3 P = inertial->GetPositionRelTo(focus->frame);
                if(!tpts_c.empty()) {
                    static thread_local std::vector<glm::dvec3> tpts_f;
                    tpts_f.clear();
                    tpts_f.reserve(tpts_c.size());
                    for(const glm::dvec3 &pt : tpts_c) {
                        tpts_f.push_back(O * pt + P);
                    }
                    proj.drawOrbit(dl, tpts_f, col_vessel, 1.0f);
                }
                const ImVec2 tpx = proj.px(O * r2 + P);
                dl->AddCircleFilled(tpx, 3.0f, col_vessel);
                // Label offset from the marker (px): +x right, -y up.
                const float label_dx = 6.0f, label_dy = 14.0f;
                dl->AddText(ImVec2(tpx.x + label_dx, tpx.y - label_dy),
                            col_vessel, s->name.c_str());
            }
        }

        if(!ship) {
            ImGui::Text("No ships yet -- launch one from the VAB.");
        }

    });
    ImGui::PopStyleColor();   // the black map background
    ImGui::PopStyleVar(2);    // WindowBorderSize + WindowPadding
}

// ---- Research Lab windows -------------------------------------------------
// The scene's single window: the science score + the FULL collection log, one row per
// bank. Read-only -- recoverActive grows the log.
// Rows are PRE-BUILT, not rebuilt per frame (#87): researchLabEnter fills g.labRows,
// and the render self-heals if g.science.version differs from g.labRowsVersion.

static void buildAtlasRows(Game &g);

void researchLabEnter(Game &g) {
    const Calendar &cal = g.sys.home ? g.sys.home->cal : Calendar{};
    g.labRows = labEntries(g.science.recovered, cal);
    g.labRowsVersion = g.science.version;
    buildAtlasRows(g);
}

void drawResearchLab(Game &g) {
    drawWin(g, W_ResearchLab, [&] {
        // Back to the hub (the frame below); Esc does the same (labKeyActions).
        if(ImGui::Button("Back to Space Center")) {
            popScene(g);
        }
        // A checkbox rather than the window's X alone: closing it leaves no other way back in.
        bool atlas_on = winOpen(W_ResearchAtlas);
        if(ImGui::Checkbox("System Atlas", &atlas_on)) {
            setWinOpen(W_ResearchAtlas, atlas_on);
        }
        ImGui::Separator();
        ImGui::Text("Science: %d", g.science.score);
        ImGui::Separator();
        if(g.science.recovered.empty()) {
            ImGui::TextDisabled("No recovered experiment data yet.");
            ImGui::TextDisabled(
                "Run experiments aboard a crewed ship and recover it to "
                "archive them here.");
        } else {
            // Self-heal: rebuild if the career log changed since the rows were built.
            if(g.labRowsVersion != g.science.version) {
                const Calendar &cal =
                    g.sys.home ? g.sys.home->cal : Calendar{};
                g.labRows = labEntries(g.science.recovered, cal);
                g.labRowsVersion = g.science.version;
            }
            // g.labRows is ready (built on entry, or just healed above). The archive outgrows the
            // window, so the child scrolls.
            ImGui::BeginChild("##recovered", ImVec2(0.0f, 0.0f));
            for(const LabEntry &r : g.labRows) {
                ImGui::TextUnformatted(r.line1.c_str());
                if(!r.line2.empty()) {
                    ImGui::TextDisabled("%s", r.line2.c_str());
                }
            }
            ImGui::EndChild();
        }
    });
}

// ---- System Atlas (Research Lab) -----------------------------------------
// The system as a tree, one row per body: name (indented by depth), research VALUE
// tier, APPROACH dv, % DISCOVERED. Selecting a body adds its DOSSIER. --atlas-dump
// prints the same rows.
// Rows are PRE-BUILT, not computed per frame (the Lab's #87 pattern): buildAtlasRows
// fills g.atlasRows on scene entry, both self-heal on g.science.version.
namespace {
// A (family, situation) study-slot -- the identity key for the coverage count.
// (Named AtlasSlot: ui::Slot is a different thing in scope.)
struct AtlasSlot {
    std::string type;
    int situation = 0;
    bool operator<(const AtlasSlot &o) const {
        if(type != o.type) { return type < o.type; }
        return situation < o.situation;
    }
};

// Total study-slots across the family table: one slot per situation each family can
// run in.
size_t atlasTotalSlots() {
    size_t total = 0;
    for(const ExperimentDef &d : experimentDefs()) {
        total += (size_t)d.valid_in.size();
    }
    return (total > 0) ? total : 1;   // a missing table must not divide by 0
}

/* The dossier: the system JSON read back in friendly units, so the lab can
   sanity-check the flight readouts -- a line disagreeing with what the game flies means
   the DERIVATION is wrong. The orbit is inverted from orb_ang_speed (Kepler III)
   exactly as the loader places the body, with e from the epoch rail's angular momentum. */
void atlasDossier(const TerrainBody &b, std::vector<AtlasFact> &f) {
    char v[64], w[64];
    auto line = [&](const char *label, const char *value) {
        f.push_back(AtlasFact{ label, value, false });
    };
    auto head = [&](const char *title) {
        f.push_back(AtlasFact{ title, std::string(), true });
    };

    head("body");
    line("type", b.isStar() ? "star"
               : (b.type == BodyType::Moon ? "moon" : "planet"));
    line("radius", fmt_dist((double)b.radius, v, sizeof v));
    line("mass", fmt_mass((double)b.mass, v, sizeof v));
    snprintf(v, sizeof v, "%.2f m/s2", b.g);
    line("surface gravity", v);
    // Both at the mean radius under the body's own mu: what a launch must reach, an
    // arrival burn must shed.
    line("escape velocity",
         fmt_speed(std::sqrt(2.0 * b.mu / (double)b.radius), v, sizeof v));
    line("surface orbit",
         fmt_speed(std::sqrt(b.mu / (double)b.radius), v, sizeof v));
    // solar_day_seconds: the measured solar day (#201), not the civil 86400 s day (#202).
    line("day length", b.cal.solar_day_seconds > 0.0
                         ? fmt_time(b.cal.solar_day_seconds, v, sizeof v)
                         : "none (no spin)");
    if(b.rot_frame != nullptr) {
        // The pole IS the spin frame's +Y, and initial_orient carries the authored tilt
        // (tilt_azimuth turns the lean's direction, leaving its angle, so col1.y alone
        // gives the tilt).
        const double pole = glm::clamp(b.rot_frame->initial_orient[1].y,
                                       -1.0, 1.0);
        line("axial tilt", fmt_deg(std::acos(pole), v, sizeof v));
        line("surface shell", fmt_dist(b.rot_frame->soi, v, sizeof v));
    }
    if(b.frame != nullptr) {
        // The star's inertial frame carries the authored universe bound, not an SoI (no
        // parent whose tide could dominate).
        line(b.frame->parent == nullptr ? "universe bound" : "SoI",
             fmt_dist(b.frame->soi, v, sizeof v));
    }

    head("surface");
    if(b.isStar()) { line("ground", "none (a star)"); }
    else if(b.surface.bands) { line("ground", "none (gas giant)"); }
    else { line("ground", "solid"); }
    if(!b.isStar() && !b.surface.bands) {
        if(b.surface.has_sea) {
            snprintf(v, sizeof v, "yes, %+.0f m, %s sea",
                     (double)b.surface.sea_level,
                     oceanModeName(b.surface.ocean_mode));
            line("seas", v);
            // The shell the loader actually BUILT, not the authored mode: a flat-sea body never
            // has one, and a mesh body has none until its heavy phase lands. This is the fact the
            // renderer uses, so e2e can pin it.
            if(b.ocean != nullptr) {
                line("ocean shell", fmt_dist((double)b.ocean_radius,
                                             v, sizeof v));
            }
        } else { line("seas", "none"); }
        // The authored relief; the MEASURED highest ground only once the heavy phase has
        // sampled the heightfield.
        line("relief", fmt_dist((double)b.surface.amplitude, v, sizeof v));
        if(b.ready) {
            line("highest ground",
                 fmt_dist((double)b.surface.max_height, v, sizeof v));
        }
    }
    const AtmosphereParams &at = b.surface.atmosphere;
    if(at.enabled) {
        line("atmosphere top", fmt_dist(at.top(), v, sizeof v));
        if(at.scale_height > 0.0) {
            line("scale height", fmt_dist(at.scale_height, v, sizeof v));
        }
        // The block can exist for the drawn rim alone, with no drag air behind it (the two
        // halves are independent).
        if(at.sea_level_density > 0.0) {
            snprintf(v, sizeof v, "%.3f kg/m3", at.sea_level_density);
            line("air density", v);
        } else {
            line("air density", "none (rim only)");
        }
    } else { line("atmosphere", "none"); }
    for(const RingParams &r : b.surface.rings) {
        line(r.name.empty() ? "ring band" : r.name.c_str(),
             (std::string(fmt_dist(r.inner, v, sizeof v)) + " - "
              + fmt_dist(r.outer, w, sizeof w)).c_str());
    }

    head("orbit");
    const Frame *fr = b.frame;
    if(fr == nullptr) { return; }
    if(fr->parent == nullptr || fr->parent->body == nullptr) {
        line("about", "nothing (the star)");
        return;
    }
    const TerrainBody *par = fr->parent->body;
    line("about", par->name.c_str());
    if(fr->orb_ang_speed <= 0.0 || fr->parent_mu <= 0.0) {
        line("orbit", "fixed offset (not orbiting)");
        return;
    }
    const double n = fr->orb_ang_speed;
    const double a = std::cbrt(fr->parent_mu / (n * n));
    line("semi-major axis", fmt_dist(a, v, sizeof v));
    // p = h^2/mu = a(1-e^2), read off the epoch rail state.
    const double h = glm::length(glm::cross(fr->orbit_pos0, fr->orbit_vel0));
    const double e = std::sqrt(std::max(0.0, 1.0 - (h * h / fr->parent_mu) / a));
    snprintf(v, sizeof v, "%.4f", e);
    line("eccentricity", v);
    // From the parent's centre (what ORBITAL's PeA/ApA show) and above its mean sea level.
    const double surf = (double)par->radius + (double)par->surface.sea_level;
    line("periapsis", fmt_dist(a * (1.0 - e), v, sizeof v));
    line("periapsis alt", fmt_dist(a * (1.0 - e) - surf, v, sizeof v));
    if(e < 1.0) {
        line("apoapsis", fmt_dist(a * (1.0 + e), v, sizeof v));
        line("apoapsis alt", fmt_dist(a * (1.0 + e) - surf, v, sizeof v));
    }
    line("period", fmt_time(2.0 * std::numbers::pi / n, v, sizeof v));
    /* Plane tilt in three references, because for a moon of an inclined planet they differ:
       Bop authors orb_incl 15 deg against JOOL's plane, itself 1.30 deg off the system
       plane. (ecl) is the SYSTEM plane -- what the flight readout and the map's Ecliptic
       view measure in. (parent) is what "incl_ref": "orbit" authors against; (eq.) is the
       parent's EQUATOR. */
    const glm::dvec3 nrm = fr->orient * glm::dvec3(0.0, 1.0, 0.0);   // parent axes
    const double tilt_parent =
        std::acos(glm::clamp(fr->orient[1].y, -1.0, 1.0));
    // root_orient carries the whole parent chain, so this is the same normal in the system
    // frame -- and rails never spin their own frame, so it is constant in time.
    const glm::dvec3 nrm_root =
        glm::normalize(fr->root_orient * glm::dvec3(0.0, 1.0, 0.0));
    const double tilt_ecl = std::acos(glm::clamp(nrm_root.y, -1.0, 1.0));
    line("plane tilt (ecl)", fmt_deg(tilt_ecl, v, sizeof v));
    if(std::fabs(tilt_parent - tilt_ecl) > 1e-4) {
        line("plane tilt (parent)", fmt_deg(tilt_parent, v, sizeof v));
    }
    if(par->rot_frame != nullptr) {
        const glm::dvec3 paxis = glm::normalize(
            par->rot_frame->initial_orient * glm::dvec3(0.0, 1.0, 0.0));
        const double tilt_eq =
            std::acos(glm::clamp(glm::dot(paxis, nrm), -1.0, 1.0));
        if(std::fabs(tilt_eq - tilt_ecl) > 1e-4) {
            line("plane tilt (eq.)", fmt_deg(tilt_eq, v, sizeof v));
        }
    }
    /* Node and periapsis longitude in the SYSTEM plane (RefPlane{}: +Y normal, +X zero
       longitude) -- the reference the flight readout uses on Ecliptic. Dashed when
       undefined: an orbit IN the plane has no node, a near-circular one no periapsis
       direction. The authored node lives in the PARENT's plane, so it joins the list
       whenever it differs. */
    const PlaneAngles pa = orbitPlaneAngles(
        computeOrbitElements(fr->root_orient * fr->orbit_pos0,
                             fr->root_orient * fr->orbit_vel0,
                             fr->parent_mu),
        RefPlane{});
    line("node (ecl)", pa.node_ok ? fmt_deg(pa.lan, v, sizeof v) : "-");
    const PlaneAngles pp = orbitPlaneAngles(
        computeOrbitElements(fr->orient * fr->orbit_pos0,
                             fr->orient * fr->orbit_vel0,
                             fr->parent_mu),
        RefPlane{});   // parent axes: +Y is the parent's rail-plane normal
    const double lan_gap = pa.node_ok && pp.node_ok
        ? std::fabs(std::remainder(pa.lan - pp.lan, 2.0 * std::numbers::pi)) : 1.0;
    if(pp.node_ok && lan_gap > 1e-4) {
        line("node (parent)", fmt_deg(pp.lan, v, sizeof v));
    }
    line("periapsis lon (ecl)", pa.lpe_ok ? fmt_deg(pa.lpe, v, sizeof v) : "-");
}

// Walk the frame tree from `b`, one row per body (tree order). A body's inertial frame
// has TWO kinds of children: its OWN spin frame (rotating -- skipping it is what keeps
// this from recursing forever) and the inertial frames of the bodies that ORBIT it.
// `seen` guards a cyclic orbit pair in the data (#129).
void atlasWalk(const System &sys, const TerrainBody *b, int depth,
               const std::map<std::string, std::set<AtlasSlot>> &covered,
               size_t total, std::vector<AtlasRow> &out,
               std::set<const TerrainBody *> &seen) {
    if(!seen.insert(b).second) { return; }   // already shown: stop the cycle
    AtlasRow r;
    r.rawName = b->name;   // the plain name, for the detail-pane selection key
    std::string name;
    name.append((size_t)depth * 3, ' ');
    name += b->name;
    if(sys.home == b) { name += "  (home)"; }
    r.name = std::move(name);
    r.valueWord = valueWord(b->science_mult);
    r.valueExact = b->science_mult;
    // 0 = where you start (home) or the reference (the star): no approach.
    r.dv = (b->transfer_dv <= 0.0) ? 0
                                   : (long)std::lround(b->transfer_dv / 50.0) * 50;
    auto it = covered.find(b->name);
    const size_t cov = (it != covered.end()) ? it->second.size() : 0;
    r.discovered = (int)std::min<size_t>(100, (size_t)std::lround(100.0 * cov / total));
    atlasDossier(*b, r.facts);
    out.push_back(std::move(r));

    if(b->frame == nullptr) { return; }
    for(const Frame *child : b->frame->children) {
        if(child == nullptr || child->body == nullptr) { continue; }
        if(child->rotating) { continue; }   // b's own spin frame, not a moon
        atlasWalk(sys, child->body, depth + 1, covered, total, out, seen);
    }
}
}   // namespace

static void buildAtlasRows(Game &g) {
    const System &sys = g.sys;
    g.atlasRows.clear();
    g.atlasRowsVersion = g.science.version;
    if(sys.root == nullptr) { return; }

    // Covered study-slots per body: recovered findings only, counted per family+situation.
    // BIOMES are NOT separate slots here (that needs the heavy phase's max height), so a
    // biome-rich body reads LOWER than its true surveyed fraction -- the % is a floor.
    std::map<std::string, std::set<AtlasSlot>> covered;
    for(const Experiment &e : g.science.recovered) {
        if(defFor(e.type) == nullptr) { continue; }   // unknown family: skip
        covered[e.body].emplace(AtlasSlot{ e.type, (int)e.situation });
    }
    const size_t total = atlasTotalSlots();

    std::set<const TerrainBody *> seen;   // cycle guard (#129)
    atlasWalk(sys, sys.root, 0, covered, total, g.atlasRows, seen);
}

void drawResearchAtlas(Game &g) {
    drawWin(g, W_ResearchAtlas, [&] {
        const System &sys = g.sys;
        if(sys.root == nullptr) {
            ImGui::TextDisabled("No system loaded.");
            return;
        }
        // Self-heal: rebuild if the career log changed since the rows were built.
        if(g.atlasRowsVersion != g.science.version) {
            buildAtlasRows(g);
        }
        if(g.atlasRows.empty()) {
            ImGui::TextDisabled("No bodies in this system.");
            return;
        }

        // Selection = the raw body name (a string, not an index, so it survives the rows being
        // rebuilt). Default to home, else the first body.
        static std::string selected;
        auto findRow = [&](const std::string &n) -> const AtlasRow * {
            for(const AtlasRow &r : g.atlasRows) {
                if(r.rawName == n) { return &r; }
            }
            return nullptr;
        };
        if(findRow(selected) == nullptr) {
            const AtlasRow *home = sys.home ? findRow(sys.home->name) : nullptr;
            selected = (home != nullptr) ? home->rawName
                                         : g.atlasRows.front().rawName;
        }

        // The Δ, × and — in the rows below render in the bundled
        // DejaVuSansMono; a --font override lacking them would show tofu.
        ImGui::TextDisabled("select a body:");
        ImGui::Separator();

        // Tree list on the left (narrow 1/3), dossier on the right. Two side-by-side children
        // rather than a table -- a table cell cannot fill the height.
        const float availW = ImGui::GetContentRegionAvail().x;
        const float listW =
            std::min(ImGui::GetFontSize() * 16.0f, availW * 0.30f);
        ImGui::BeginChild("##atlasList", ImVec2(listW, 0.0f), false);
        for(const AtlasRow &r : g.atlasRows) {
            const bool sel = (r.rawName == selected);
            if(ImGui::Selectable(r.name.c_str(), sel)) {
                selected = r.rawName;
            }
        }
        ImGui::EndChild();

        ImGui::SameLine(0.0f, ImGui::GetStyle().ItemSpacing.x);

        ImGui::BeginChild("##atlasDetail", ImVec2(0.0f, 0.0f), false);
        const AtlasRow *row = findRow(selected);
        if(row == nullptr) {
            ImGui::TextDisabled("(not in this system)");
        } else {
            ImGui::Text("%s", row->rawName.c_str());
            ImGui::Separator();
            if(ImGui::BeginTable("##atlasDetailRows", 2,
                                 ImGuiTableFlags_BordersInner)) {
                ImGui::TableSetupColumn("label",
                                        ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("value",
                                        ImGuiTableColumnFlags_WidthStretch);
                // research weight (the exact × is a tooltip on the word)
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted("research weight");
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(row->valueWord.c_str());
                ImGui::SetItemTooltip("×%.1f of home", row->valueExact);
                // approach Δv (0 = home / the star: no approach)
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted("approach Δv");
                ImGui::TableNextColumn();
                if(row->dv <= 0) { ImGui::TextDisabled("\xe2\x80\x94"); }  // "—"
                else { ImGui::Text("%ld m/s", row->dv); }
                // science found (% of the body's study-situations covered)
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted("science found");
                ImGui::TableNextColumn();
                ImGui::Text("%d%%", row->discovered);
                // Pre-built strings, so this window and --atlas-dump print the same text.
                for(const AtlasFact &fa : row->facts) {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    if(fa.header) {
                        ImGui::TextDisabled("%s", fa.label.c_str());
                        continue;
                    }
                    ImGui::TextUnformatted(fa.label.c_str());
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted(fa.value.c_str());
                }
                ImGui::EndTable();
            }
        }
        ImGui::EndChild();
    });
}

/* --atlas-dump MS: every Atlas row (tree order + dossier) to stdout, for reading the
   system in a terminal and for e2e to pin. Prints the SAME pre-built strings the window
   draws. */
void dumpAtlas(Game &g) {
    buildAtlasRows(g);
    for(const AtlasRow &r : g.atlasRows) {
        printf("[atlas] %s weight=%s approach_dv=%ld discovered=%d%%\n",
               r.rawName.c_str(), r.valueWord.c_str(), r.dv, r.discovered);
        for(const AtlasFact &fa : r.facts) {
            if(fa.header) { printf("[atlas]   [%s]\n", fa.label.c_str()); }
            else { printf("[atlas]   %-20s %s\n", fa.label.c_str(),
                                            fa.value.c_str()); }
        }
    }
    fflush(stdout);
}

/* --map-dump MS: the basis each map plane slot produces for the active ship. The map is
   pure drawing, so a flipped e2 is invisible to every check but an eyeball (#181). */
void dumpMapBasis(Game &g) {
    Vehicle *ship = g.ship;
    if(!ship || !ship->m_parent || !ship->m_parent->frame) {
        // Say so: a silent no-op reads as "the map has no basis to report".
        printf("[mapdump] no active ship, nothing to dump\n");
        fflush(stdout);
        return;
    }
    Frame *focus_frame = ship->m_parent->frame;
    /* Four slots, but the target slot only exists while a transfer target is
       picked: printing the ecliptic fallback under a "tgt" label is the exact lie
       uiRefPlane refuses to print, so drop the row. */
    static const int kSlots[] = { kRefEquator, kRefEcliptic, kRefOrbit,
                                  kRefTarget };
    glm::dvec3 target_hat(0.0);
    const bool target_ok = g.xferPlanner.targetPlaneNormal(focus_frame,
                                                           target_hat);
    for(const int slot : kSlots) {
        if(slot == kRefTarget && !target_ok) { continue; }
        glm::dvec3 plane_n, plane_x;
        mapPlaneBasis(focus_frame, g.hud.view.orbit_pos, g.hud.view.orbit_vel, true,
                      slot, target_hat, plane_n, plane_x);
        OrbitMap m;
        m.setPlane(plane_n, plane_x);
        /* A prograde body at +e1 moves along n x e1, which must read -e2 on
           screen in every slot (#181). +1 here means that slot draws the sweep
           the other way from the others. */
        const double sweep = glm::dot(glm::cross(m.n, m.e1), m.e2);
        /* The AN/DN markers the map draws for this slot (orbit.h
           orbitNodePoints), or "-" when the slot has no crossing to mark --
           which for the Orbital slot is always: an orbit does not cross its own
           plane. Pinning which slots get markers is the only way to catch the
           map being handed the wrong plane, since every slot's picture still
           looks plausible. */
        glm::dvec3 an(0.0), dn(0.0);
        const int nodes = orbitNodePoints(g.hud.view.o, m.n, g.hud.view.mu, an, dn);
        char an_s[64] = "-", dn_s[64] = "-";
        if(nodes & kNodeAn) {
            snprintf(an_s, sizeof an_s, "(%.0f,%.0f,%.0f)", an.x, an.y, an.z);
        }
        if(nodes & kNodeDn) {
            snprintf(dn_s, sizeof dn_s, "(%.0f,%.0f,%.0f)", dn.x, dn.y, dn.z);
        }
        // Sim time, like [orbitlog] / [orbinfo], so a CHECK filtering on t= means the same
        // instant.
        printf("[mapdump] t=%.1fs slot=%s n=(%.6f,%.6f,%.6f) e1=(%.6f,%.6f,%.6f) "
               "e2=(%.6f,%.6f,%.6f) sweep=%+d an=%s dn=%s\n",
               g.time, refPlaneName(slot),
               m.n.x, m.n.y, m.n.z, m.e1.x, m.e1.y, m.e1.z,
               m.e2.x, m.e2.y, m.e2.z, sweep > 0.0 ? 1 : -1, an_s, dn_s);
    }
    fflush(stdout);
}

