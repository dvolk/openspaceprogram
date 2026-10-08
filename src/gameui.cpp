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
#include "fmt.h"         // fmt_dist / fmt_time
#include "orbitsample.h" // OrbitSampleCache + open-arc sampling
#include "orbitmap.h"    // OrbitMap + contrastingColor
#include "equirect.h"    // equirectLonLat / equirectDir
#include "surfmap.h"     // surfmapShade / surfmapWraps + surfmapCompute
#include "texture.h"     // make_texture_r8
#include "vab.h"         // the editor ops (drawVabUI)
#include "staging.h"     // computeStaging
#include "shipdef.h"     // list_vab_ship_defs
#include "resdir.h"      // resdir::path
#include "system.h"      // list_systems
#include "save.h"        // save_game / load_game / list_saves / delete_save
#include "datadir.h"     // the saves/ directory's location

#include "../middleware/imgui/imgui.h"
#include "../middleware/implot/implot.h"

// GLM's gtx extensions (glm::angle in ORBITAL) hard-error without this.
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/vector_angle.hpp>

namespace {
/* Viridis (matplotlib's default scientific colormap), 11 anchor stops
   linearly interpolated: perceptually uniform, colorblind-safe. t in [0,1]
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

// Cached orbit samplings, one entry per orbiting object (keyed on its
// pointer). Reuse is only trusted while the orbiting object is on a fixed
// Keplerian conic. File scope so the Orbital Map and the Surface Map
// share one cache. See OrbitSampleCache.
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

// Stroke one body's orbit so the polyline starts and ends at the body
// marker: the equal-anomaly samples do not include the body, and the chord
// near it otherwise visibly misses the marker when zoomed in. Find the
// nearest sample (k) and its CLOSER neighbour, then walk from that bracket
// all the way around to k and prepend the body (walking forward from k
// unconditionally ends at the wrong neighbour when the body sits just past
// k).
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
    // Transform parent -> focus in one pass. Keep the prepend-body order:
    // index 0 is the body, then the rotated walk.
    static thread_local std::vector<glm::dvec3> cpts_f;
    cpts_f.clear();
    cpts_f.reserve(n + 1);
    for(const glm::dvec3 &p : cpts) { cpts_f.push_back(O * p + P); }
    map.drawOrbit(dl, cpts_f, col, thickness, /*closed=*/true, /*start=*/0);
}

// Every body's orbit (around its own parent) projected into the focus's
// frame. Each body's ellipse is sampled from its rail's IMMUTABLE epoch
// state in the parent frame (the cache key is bit-stable) and then
// rotated/translated into the focus's frame at draw time. Orbits whose
// parent SoI is under 10 px are skipped (LOD); so is any orbit whose
// apoapsis disc misses the view rect. The star has no parent and so no
// orbit, but it is drawn as the system's anchor -- see below.
static void drawSystemBodyOrbits(Game &g, TerrainBody *focus,
                                 const OrbitMap &map, ImDrawList *dl,
                                 const MapViewRect &view, double map_scale,
                                 TerrainBody *sel_body,
                                 ImU32 col_child, ImU32 col_sel, ImU32 ink,
                                 ImU32 soi_col) {
    std::vector<TerrainBody *> &planets = g.sys.bodies;
    // The star: skipped by the loop below (no parent, no conic), so draw its
    // disk + label first -- everything else orbits it, and a map without it
    // has no obvious centre. When the star IS the focus the caller already
    // draws it at the centre.
    // Warm rather than the bodies' ink: it is the light source, and at
    // system scale it is the one marker you want to pick out instantly.
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
        // The rail epoch is the fixed conic in the LOCAL orbital plane;
        // `orient` carries the plane tilt into the parent frame. Sampling
        // this -- not the live rail state -- is what makes the cache hit.
        const glm::dmat3 &Rloc = b->frame->orient;
        const glm::dvec3 epos_p = Rloc * b->frame->orbit_pos0;
        const glm::dvec3 evel_p = Rloc * b->frame->orbit_vel0;
        // Cheap size for cull + sample-count LOD, before any Kepler work.
        double sma = 0.0, apo = -1.0;
        orbitConicSize(epos_p, evel_p, mu_c, sma, apo);
        const glm::dmat3 O = parent->GetOrientRelTo(focus->frame); // parent -> focus
        const glm::dvec3 P = parent->GetPositionRelTo(focus->frame);
        // Body + parent on the map (focus's frame). The ellipse sits within
        // apoapsis of the parent, so a miss on that disc skips the whole body.
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
        // A disk at the body's TRUE radius (floored so it stays visible when
        // zoomed out). Label only when the body is a real disk or selected --
        // 200 moon names on a wide system are unreadable.
        const float min_r = selected ? 5.0f : 3.0f;
        const float body_r_px = map.bodyRadiusPx(b->radius, min_r);
        map.drawBody(dl, cpos_f, b->radius, ccol, min_r);
        if(selected) {
            dl->AddCircle(body_px, body_r_px + 4.0f, ccol, 0, 1.0f);
        }
        // Label whenever the orbit is drawn and the marker is near the view
        // -- the name is how you tell one moon's ellipse from another.
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

// Format a sim-clock time (s) on the home body's calendar lives in
// calendar.h (fmt_cal_time / fmt_cal_compact / fmt_cal_duration).

// The debris belts (JSON root "belts", parsed in src/system.cpp) as flat
// annuli, drawn on the Tracking map only. Geometry is data; the colours are
// still code-side, cycling by array position -- rock warm, ice cool, both
// low-alpha so a band reads as a populated region rather than another orbit.
static const ImVec4 kBeltCols[] = {
    ImVec4(0.62f, 0.55f, 0.45f, 0.16f),
    ImVec4(0.55f, 0.63f, 0.75f, 0.12f),
};
constexpr size_t kBeltColsN = sizeof(kBeltCols) / sizeof(kBeltCols[0]);

// A belt is a FLAT ring in the star's equatorial plane, so it has to be
// sampled in world space and projected: OrbitMap::drawRing draws a
// screen-space circle, which is only correct for spheres (the SOI rings).
// Sampling is what makes the band foreshorten when the map shows a plane
// other than the star's equator (Ecliptic / Orbital) instead of always
// reading top-down. The plane is assumed, not authored: a belt that needed
// its own inclination would need a field for it here.
static void drawDebrisBelts(Game &g, TerrainBody *focus, const OrbitMap &map,
                            ImDrawList *dl, const MapViewRect &view,
                            double map_scale) {
    if(g.sys.belts.empty()) { return; }
    TerrainBody *sun = g.sys.root;
    if(!sun || !sun->frame) { return; }
    const glm::dvec3 sun_f = sun->frame->GetPositionRelTo(focus->frame);
    // The belt plane's normal, in the focus's inertial frame (the same space
    // map.setPlane() and the positions use): the star's pole. Spin is about the
    // pole, so the rotating frame's orientation gives it at any spin angle. No
    // shipped system tilts its star, so this lands on the system plane -- a
    // convention, documented on Frame::spinAxisRelTo (#173).
    const glm::dvec3 n = sun->frame->spinAxisRelTo(focus->frame);
    // Any orthonormal pair spanning the belt plane.
    const glm::dvec3 ref = (std::abs(n.y) < 0.9) ? glm::dvec3(0.0, 1.0, 0.0)
                                                 : glm::dvec3(1.0, 0.0, 0.0);
    const glm::dvec3 u = glm::normalize(glm::cross(n, ref));
    const glm::dvec3 v = glm::cross(n, u);
    constexpr int N = 96;
    // Reused across bands and frames, like OrbitMap::drawOrbit's buffer.
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
        // Fill: a quad list between the two projected loops. ImGui has no
        // annulus primitive, and stroking the mid-loop at a constant
        // thickness would ignore the foreshortening we are drawing for.
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

// --- Telemetry window: a 2x2 grid of plots, each with a dropdown to pick
// which time series to show.
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
// ship's view; 2-6 are the per-frame timings on Game.
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

// One grid cell: a full-width dropdown to pick the series, then the plot.
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

/* The flight windows below assume there IS an active vessel, and they are
   right to: Flight is only the live scene when there is one. A shipless boot
   lands on the title and a shipless load on the Space Center hub; remove_ship
   refuses the last vessel; LAUNCH / New Game create one before enterFlight.
   The per-window "No active ship." guards went with the state they covered. */
void drawUIReadouts(Game &g) {
    // The window bodies' locals are Game members (aliased so the bodies
    // read the same).
    TransferPlanner &planner = g.xferPlanner;
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
    float &sfx_volume = g.sfx_volume;
    float &music_volume = g.music_volume;
    // The DPI slider edits this; "Apply DPI" commits it to ui_scale.
    static float dpi_pending = 1.0f;
    // The Settings window writes these; the 3D pass (render.cpp) reads.
    bool &physics_debug_drawing = g.physics_debug_drawing;
    bool &world_drawing = g.world_drawing;
    bool &draw_starfield = g.draw_starfield;
    bool &draw_skylines = g.draw_skylines;
    float &sky_dim = g.sky_dim;
    float &sky_dim_cone = g.sky_dim_cone;
    // The Settings window writes these; tick.cpp reads.
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
       (big font) -- orbital (ASL + orbital speed) when in the
       inertial frame or above 30km ASL, else surface (terrain
       altitude + ground speed) in the rotating frame.
       Row 2: Kerbin clock (regular font, centered). */
    drawWin(g, W_Hud, [&] {
        if(ship) {
            const double asl = distance
                - ((double)ship->m_parent->radius
                   + (double)ship->m_parent->surface.sea_level);
            const double agl = distance - ship->m_parent->GetTerrainHeight(glm::normalize(pos));
            const bool surface_mode = ship->frame->isRotFrame()
                                   && asl < kSurfaceModeAlt;
            const double alt = surface_mode ? agl : asl;
            const double spd = surface_mode ? glm::length(surf_vel) : speed;
            ImGui::PushFont(g.bigger);
            /* fmt_dist, not a fixed unit: at oort distances a meter count
               is 16 digits wide and the old (int) cast was UB (INT_MIN). */
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

    /* Window list: the LIVE scene's own windows, straight from the table in
       uiwins.cpp -- so the panel can never offer a toggle for a window that
       is not drawn here. Entries with inList=false are toggled from their
       parent window instead, and Root/Chrome are not rows at all (a panel
       that could close itself is a dead end). */
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
        // Display: the window mode + the resolution it runs at. Both apply
        // immediately (the SIZE_CHANGED event in events.cpp finishes the
        // resize). "fullscreen" runs at the display's native mode, so the
        // resolution is off there.
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
            // Resolution: the display's supported modes (+ the current one).
            // The selection is an exact WxH match (among those, the refresh
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
            // Antialiasing: the window's MSAA sample count is fixed at
            // creation (the GLX visual is chosen then), so picking a new
            // value here sets the launch value -- it takes effect on
            // restart. The selection tracks args.msaa_samples so the
            // dropdown always reflects your pick (mirroring the count the
            // current window runs at left it stuck on the old value).
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
                // Split literals: in C, "\02" is the OCTAL escape for
                // 0x02 (STX), not a null + '2' -- a single literal here
                // would list the 2x/4x entries as missing-glyph boxes.
                if(ImGui::Combo("Antialiasing", &sel,
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
        ImGui::Checkbox("Reference circles", &draw_skylines);
        // Prototype (render.cpp skyGain): how far the stars fade while the sun
        // is in view and the camera is in sunlight. 1 = no fade. Live, so the
        // look can be judged without a restart.
        ImGui::SliderFloat("Sky dim", &sky_dim, 0.0f, 1.0f, "%.2f");
        // Half-angle of the "sun in view" cone (same units as --sky-dim-cone).
        ImGui::SliderFloat("Sky dim cone", &sky_dim_cone, 1.0f, 179.0f, "%.0f");
        // Post-processing: one checkbox per effect (the passes run in this
        // order); an effect that exposes parameters also gets a slider per
        // parameter (range + neutral value from the effect's definition).
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
                // Underscored uniform name -> natural-language label
                // (black_level -> "black level").
                char label[64];
                snprintf(label, sizeof(label), "%s", p.name);
                for(char *c = label; *c; c++) { if(*c == '_') *c = ' '; }
                float v = g.postfx->GetParam(fx, p.name);
                if(ImGui::SliderFloat(label, &v, p.min, p.max, "%.2f")) {
                    g.postfx->SetParam(fx, p.name, v);
                }
            }
        }
        // Audio: the master levels apply live and persist with "Save".
        // No-op while audio is disabled (headless) -- the values still save.
        // (0..1 like "Window opacity" -- a %.0f%% label would only
        // ever print "0%" or "1%".)
        ImGui::Separator();
        ImGui::Text("Audio");
        if(ImGui::SliderFloat("SFX volume", &sfx_volume, 0.0f, 1.0f, "%.2f")) {
            g.audio.setSfxVolume(sfx_volume);
        }
        if(ImGui::SliderFloat("Music volume", &music_volume, 0.0f, 1.0f,
                             "%.2f")) {
            g.audio.setMusicVolume(music_volume);
        }
        if(ImGui::Combo("UI style", &ui_style, "Dark\0Light\0Classic\0")) {
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
        // The slider only edits the pending value; the Apply button
        // commits it (applying live while dragging would move this window
        // out from under the cursor).
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
        // args.terrain_px [real screen px] (read live by GeoPatch::Update).
        // The slider is a 6-step detail level, right = finer: 512px is the
        // default. The 64/32 steps are there for poking at the LOD, not for
        // playing: the budget is per patch, so they ask for tens of thousands
        // of patches and the single async builder never catches up.
        {
            const int terrain_px_table[] = { 1024, 512, 256, 128, 64, 32 };
            const int nlevels = 6;
            int terrain_level = 0, best = -1;
            for (int i = 0; i < nlevels; i++) {
                const int d = std::abs(args.terrain_px - terrain_px_table[i]);
                if (best < 0 || d < best) { best = d; terrain_level = i; }
            }
            if(ImGui::SliderInt("Terrain detail", &terrain_level, 0, nlevels - 1)) {
                args.terrain_px = terrain_px_table[terrain_level];
            }
        }
        // Difficulty lives on the New Game sheet + save.json (and
        // --exhaust-scale for tests). No Settings slider: it is per-game.
        // Camera shake: the chase cam rumbles with the crew's felt
        // acceleration (thrust + aero over mass, gravity excluded).
        ImGui::SliderFloat("Camera shake", &args.cam_shake,
                           0.0f, 3.0f, "%.1fx");
        // Controls: invert a manual attitude axis away from the default
        // (see tick.cpp). All three are off by default.
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

    // Transfer planner: parent->child body transfers (with capture) and
    // same-body ship intercepts. The solution is computed in the 3D pass
    // (xfer), so this window is pure readout + inputs.
    drawWin(g, W_Transfer, [&] {
        if(xferTargets.empty()) {
            ImGui::Text("No transfer targets: no child bodies or ships here.");
            return;
        }
        const char *cur = (xfer_target >= 0)
                         ? xferTargets[xfer_target].name : "none";
        if(ImGui::BeginCombo("Target", cur)) {
            // "none" is a real dropdown entry (not just the closed label)
            // so the target can be cleared back to -1 from here.
            if(ImGui::Selectable("none", xfer_target == -1)) {
                xfer_target = -1;
            }
            for(int i = 0; i < (int)xferTargets.size(); i++) {
                if(ImGui::Selectable(xferTargets[i].name,
                                     i == xfer_target)) {
                    xfer_target = i;
                }
            }
            ImGui::EndCombo();
        }
        // The Porkchop window hangs off this window rather than the Windows
        // list: pick a target, then open the plot for it.
        bool pc_open = winOpen(W_Porkchop);
        if(ImGui::Checkbox("Porkchop", &pc_open)) {
            setWinOpen(W_Porkchop, pc_open);
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
        // At zero the live "depart now" solution below IS the best cell.
        // "Clear plan" drops the plan and restores the ToF mode the user
        // had before sending.
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
        ImGui::Checkbox("Auto ToF (min dv)", &xfer_auto);
        if(!xfer_auto) {
            ImGui::SliderFloat("log10(ToF s)", &xfer_tof_log,
                               1.8, 7.5, "%.2f");
            char tof_s[40];
            ImGui::Text("ToF: %s", fmt_time(std::pow(10.0, xfer_tof_log), tof_s, sizeof tof_s));
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
        ImGui::Text("ToF:        %s", fmt_time(sol.tof, time_s, sizeof time_s));
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
       delay x time of flight). Computed on demand and cached until the
       next compute, so the window is cheap to leave open. */
    drawWin(g, W_Porkchop, [&] {
        if(xferTargets.empty()) {
            ImGui::Text("No transfer targets: no child bodies or ships here.");
            return;
        }
        if(xfer_target < 0) {
            ImGui::Text("Select a target body or ship (Transfer window).");
            return;
        }
        const char *tn = xferTargets[xfer_target].name;

        // On-demand compute (same trigger as the P key). The grid sweep runs
        // on the background worker (g.jobs), so this is a one-shot button,
        // not a per-frame re-sweep. While a sweep is in flight the button is
        // disabled and the last grid stays on screen.
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
        // The grid ON SCREEN, not the configured size: --porkchop-bench leaves
        // the largest swept grid as the live plot, so the two can disagree.
        ImGui::TextDisabled("grid %d x %d   (size: --porkchop-n / --porkchop-bench)",
                            pc.valid ? pc.n_dep : g.args.porkchop_n,
                            pc.valid ? pc.n_tof : g.args.porkchop_n);
        if(pc_busy) {
            ImGui::TextDisabled("(the last grid stays shown until the new one "
                                "lands)");
        }

        // Axis ranges. Off = the auto range (departure: 0 .. one target
        // period; ToF: 60 s .. three target periods). On = the two sliders,
        // in seconds; press Compute (or P) to re-sweep over them.
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
        char pc_t[40];
        ImGui::Text("min dv:      %08.1f m/s", pc.dv_min);
        ImGui::Text("depart in:   %s", fmt_time(pc.t_dep_min, pc_t, sizeof pc_t));
        ImGui::Text("time of flt: %s", fmt_time(pc.tof_min, pc_t, sizeof pc_t));
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
            setWinOpen(W_Transfer, true);
        }
        if(pc_busy) { ImGui::EndDisabled(); }

        // The heatmap: total dv over (departure delay x, time of flight y).
        // Storage is ToF-major (rows = ToF, cols = departure). Drawn as a
        // texture (not ImPlot::PlotHeatmap): that one indexes its color LUT
        // with the raw cell value, so the no-solution (NaN) cells read out
        // of bounds and assert. NaN cells are a distinct gray here.
        // Color scale: [dv_min, dv_hi]. dv_hi is the robust max (95th
        // percentile) -- the absolute max is deliberately excluded, because
        // the dv surface has a narrow unphysical spike at the shortest ToFs
        // that would stretch the scale and compress the whole launch window
        // to purple.
        double lo = pc.dv_min;
        double hi = (pc.dv_hi > lo) ? pc.dv_hi : lo;
        const int w = pc.n_dep, h = pc.n_tof;
        // The heatmap is a CACHE of pc: px and the texture only change when a
        // sweep lands, so gate both on pc_rev (bumped where pc is published).
        // Rebuilding per frame -- the old behaviour, whatever the window's
        // "cheap to leave open" comment claimed -- is a full RGBA fill plus a
        // GL upload every frame for a plot that only moves when you press P:
        // 256 KB at the default 256x256, tens of MB at the --porkchop-n /
        // --porkchop-bench caps. px keeps its peak capacity, so the largest
        // grid ever swept also stays resident.
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
        // The fill above indexes total_dv as w x h and the Image below derefs
        // the texture: both must match the grid that is on screen.
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
        char pc_min[40];
        ImGui::TextDisabled("x: departure delay  %.0f .. %.0f s (min %s)",
                            pc.t_dep_lo, pc.t_dep_hi,
                            fmt_time(pc.t_dep_min, pc_min, sizeof pc_min));
        ImGui::TextDisabled("y: time of flight   %.0f .. %.0f s (min %s)",
                            pc.tof_lo, pc.tof_hi, fmt_time(pc.tof_min, pc_min, sizeof pc_min));
        ImGui::TextDisabled("bar: dv in m/s (top = max)   gray: no solution");
    });

    /* Surface Map: the chosen body's surface as an equirectangular 2-D map,
       the ship's position + orbit overlaid, and optionally the terminator.
       The pixel buffer is computed on demand on the background worker and
       cached until the next compute (same pattern as the Porkchop). */
    drawWin(g, W_SurfaceMap, [&] {
        // Body to map: item 0 = "active ship's body" (surfmap_body =
        // nullptr, so the map follows the ship's SOI); the rest are
        // sys.bodies in order.
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
        // different body. Not while a sweep is in flight (the last one lands
        // for the body it was posted for).
        if(g.surfmap_in_flight == 0 &&
           (!g.surfmap_valid || g.surfmap_body_name != sm_body->name)) {
            surfmapCompute(g);
        }

        // The sweep runs on the background worker (g.jobs), like the
        // Porkchop grid. Read AFTER the auto-compute above so a just-posted
        // job already shows the "mapping ..." state.
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
        ImGui::SameLine();
        bool sea = g.surfmap_sea;
        if(sm_busy) { ImGui::BeginDisabled(); }
        if(ImGui::Checkbox("Ocean", &sea)) {
            g.surfmap_sea = sea;
            surfmapCompute(g);   // re-bake with / without the sea
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

        // The map fills the window's content width (2:1 equirectangular).
        // The buffer's row 0 is the north pole, and GL row 0 is uv (0,0)
        // = the drawn rect's top-left, so the default (0,0)-(1,1) uv
        // draws it unflipped (the Porkchop heatmap flips, its row 0
        // being the axis minimum).
        // ImMax: this imgui's GetContentRegionAvail is not clamped to 0.
        const float sm_img_w = ImMax(0.0f, ImGui::GetContentRegionAvail().x);
        const float sm_img_h = sm_img_w * 0.5f;
        const ImVec2 sm_p0 = ImGui::GetCursorScreenPos();
        ImGui::Image((ImTextureID)(std::intptr_t)sm_tex->id,
                     ImVec2(sm_img_w, sm_img_h));
        // Hover state of the Image (must be read while it is the last
        // item): the caption below shows the terrain height under the
        // cursor while the mouse is over the map.
        const bool sm_hover = ImGui::IsItemHovered();

        // The overlay (graticule, orbit, apsides, ship dot) on the map
        // rect. Screen (u, v) from (lon, lat): lon 0..2pi left -> right,
        // lat +pi/2 top -> -pi/2 bottom. Scales by the IMAGE size (image-
        // edge poles) rather than equirectPixel's (h-1) texture rows --
        // deliberate: continuous screen coords for drawing, so the ship
        // dot and the hover inverse stay consistent with each other (they
        // disagree with the colour sample by at most half a map row near
        // the poles).
        ImDrawList *dl = ImGui::GetWindowDrawList();
        const ImVec4 sm_bg = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
        const ImU32 sm_ink = contrastingColor(sm_bg);
        const ImU32 sm_ship =
            ImGui::GetColorU32(ImVec4(0.20f, 0.80f, 0.40f, 1.0f));
        auto map_px = [&](double lon, double lat) {
            const double u = lon / (2.0 * std::numbers::pi) * (double)sm_img_w;
            const double v = (std::numbers::pi * 0.5 - lat) / std::numbers::pi * (double)sm_img_h;
            return ImVec2(sm_p0.x + (float)u, sm_p0.y + (float)v);
        };

        // Graticule: lat -60..60 every 30 (equator brighter), lon every
        // 45 -- faint, under the orbit line.
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

        // The ship's orbit around the mapped body -- only when the ship is
        // orbiting it (a conic about a different body has no meaning here).
        if(ship && sm_body == ship->m_parent && g.view.mu > 0.0) {
            const double &mu = g.view.mu;
            const glm::dvec3 &orbit_pos = g.view.orbit_pos;
            const glm::dvec3 &orbit_vel = g.view.orbit_vel;
            // N matches the Orbital Map's ship-orbit count: the cache is
            // keyed on N, and both maps draw the same ship in one frame.
            const int N = 64;
            const bool closed = (o.ecc < 1.0);
            std::vector<glm::dvec3> pts_local;
            const std::vector<glm::dvec3> *pts = nullptr;
            if(closed) {
                // Per-ship cache (shared with the Orbital Map), trusted
                // only while the ship coasts on its Keplerian conic.
                pts = &orbit_caches[(const void *)ship].sample(
                    orbit_pos, orbit_vel, mu, N, ship->onRails);
            } else {
                // Open arc: the map spans the whole body, so cap the arc
                // a couple of ship radii beyond the current radius.
                const double r_cap =
                    std::max(4.0 * o.periapsis, o.distance) * 2.0;
                pts_local = sampleOpenTrajectory(orbit_pos, orbit_vel, mu, N,
                                                 r_cap);
                pts = &pts_local;
            }
            if(pts && !pts->empty()) {
                // The points live in the ship's non-rotating frame; the
                // map's pixel directions live in the body's ROTATING frame.
                // Rigid-transform each point into that frame -- the orbit's
                // GROUND TRACK over the surface.
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
                    equirectLonLat(pr / l, lon, lat);
                    px = map_px(lon, lat);
                    return true;
                };
                // The polyline, broken at the antimeridian (the map's
                // left and right edges are the SAME meridian; surfmapWraps
                // detects the >half-turn jump between consecutive lons).
                double prev_lon = -1.0e300;
                std::vector<ImVec2> seg;
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
        // body. The dot is the SUB-SATELLITE point: the ship's COM rigidly
        // transformed into the body's ROTATING frame (the surface's frame)
        // -- the same (lon, lat) the HUD reports, so it stays glued to
        // "what surface I'm over". NOT ship->frame's origin: two frames of
        // the same body share an origin, so the origin offset is 0 and the
        // dot would vanish exactly on the ship's own body.
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
                // A dot crossing the antimeridian (within the ring
                // radius of an edge) gets a twin on the other. Clip to the
                // map image so neither spills into the window margin.
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

        if(sm_hover) {
            // The hovered pixel inverts map_px. The unit direction feeds
            // GetTerrainHeight straight -- the map is baked in the same
            // rotating frame (equirect.h), so no transform. "elev" is
            // above SEA LEVEL, like the Surface window's ASL.
            const ImVec2 sm_mouse = ImGui::GetMousePos();
            const double u = (sm_mouse.x - sm_p0.x) / sm_img_w;
            const double v = (sm_mouse.y - sm_p0.y) / sm_img_h;
            const double lon = u * 2.0 * std::numbers::pi;
            const double lat = std::numbers::pi * 0.5 - v * std::numbers::pi;
            const glm::vec3 dir = equirectDir(lon, lat);
            char elev_s[32];
            ImGui::TextDisabled("cursor: lat %+.1f  lon %.1f  elev %s",
                                glm::degrees(lat), glm::degrees(lon),
                                fmt_dist((double)sm_body->GetTerrainHeight(dir)
                                         - (double)sm_body->radius
                                         - (double)sm_body->surface.sea_level,
                                         elev_s, sizeof elev_s));
        }
        if(ship && ship->m_parent && sm_body != ship->m_parent) {
            ImGui::TextDisabled("ship is in %s's SOI -- its orbit (about "
                                "%s) is not shown on %s's map",
                                ship->m_parent->name.c_str(),
                                ship->m_parent->name.c_str(),
                                sm_body->name.c_str());
        }
        ImGui::TextDisabled("equirectangular: lon 0 at the left edge "
                            "(+X), north up   computed at t=%.1fs",
                            g.surfmap_computed_at);
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
            // Mean solar time at surface lon 0 (#202): the subsolar point's
            // equirect lon is the hour angle, so LMT = lon/15 deg. Makes
            // --start-time's "pad in daylight" checkable against the clock.
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
                    // Subsolar lon is where it is noon; lon 0's solar time
                    // is behind by lon hours (east is earlier / negative HA).
                    double sod = 12.0 - glm::degrees(lon) / 15.0;
                    while(sod < 0.0) { sod += 24.0; }
                    while(sod >= 24.0) { sod -= 24.0; }
                    ImGui::Text("Solar:  %02d:%02d:%02d  (mean, at lon 0)",
                                (int)sod, (int)(60.0 * (sod - (int)sod)),
                                (int)(3600.0 * (sod - (int)sod)) % 60);
                }
            }
        }
        // Local date + time on the body the ship is currently in,
        // when it's not the home planet (e.g. the Moon's own day).
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
        char home_d[32], pos_d[32];
        ImGui::Text("Home distance: %s",
                    fmt_dist(glm::length(ship->GetPositionRelTo(ship->controller,
                                                       ship->home->frame)),
                             home_d, sizeof home_d));
        ImGui::Text("Pos: %s", fmt_dist(distance, pos_d, sizeof pos_d));
        ImGui::Text("xyz(%0.f, %0.f, %0.f)", pos.x, pos.y, pos.z);
        ImGui::Text("Vel: %.3fm/s", speed);
        ImGui::Text("xyz(%0.f, %0.f, %0.f)", vel.x, vel.y, vel.z);

        // --- power balance (the electrical system) -------------------------
        // The same resolution powerTick runs each substep, shown live.
        // Only shown for a ship that actually has an EC system.
        double gen = 0.0, constDraw = 0.0, charge = 0.0, capacity = 0.0;
        ship->getPower(&gen, &constDraw, &charge, &capacity);
        if(gen > 0.0 || constDraw > 0.0 || capacity > 0.0) {
            // active draw: the wheels, only while commanding (the same
            // condition powerTick uses).
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

    // Labels are abbreviated to <= 3 chars and right-padded to the
    // same width so the values start at a tidy column.
    drawWin(g, W_Orbital, [&] {
        char dist_s[32];   // one buffer, reused line by line
        ImGui::Text("Bod: %s (%c)", ship->m_parent->name.c_str(),
                    ship->frame->isRotFrame() ? 'R' : 'I');
        ImGui::Text("Vel: %.1fm/s", speed);
        /* R is a RADIUS from the focus, not an altitude: an 85 km orbit
           around Kerbin reads 685 km here. ApA/PeA are apsis RADII for the
           same reason. The SURFACE window has the real altitude (and
           --info-log derives alt_asl); calling these "Alt" was #171's
           sibling complaint. */
        ImGui::Text("  R: %s", fmt_dist(distance, dist_s, sizeof dist_s));
        /* Every line below is always present; "-" = the quantity
           does not exist for this orbit class (escape trajectories have no
           apoapsis/period; a near-circular orbit has no apsis line). */
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
        /* Plane angles, measured in the plane the orbit map is showing -- the
           label names it, because the number is meaningless without it (#171).
           In the map's Orbital view the plane IS the orbit: Inc reads 0 by
           construction, and there is no node and no chosen zero longitude, so
           LAN and LPe both dash rather than print a confident random number. */
        ImGui::Text("Inc: %.2f (%s)", glm::degrees(view.plane.inc),
                    refPlaneName(g.map_plane));
        ImGui::Text("Ecc: %f", o.ecc);
        ImGui::Text("SMa: %s", fmt_dist(o.semi_major, dist_s, sizeof dist_s));
        if(view.plane.node_ok) { ImGui::Text("LAN: %.2f", glm::degrees(view.plane.lan)); }
        else { ImGui::Text("LAN: -"); }
        if(view.plane.lpe_ok) { ImGui::Text("LPe: %.2f", glm::degrees(view.plane.lpe)); }
        else { ImGui::Text("LPe: -"); }
        double prograde_angle = glm::angle(facing_dir, vel_dir);
        double retrograde_angle = glm::angle(facing_dir, - vel_dir);
        ImGui::Text("Prg: %.2f", glm::degrees(prograde_angle));
        ImGui::Text("Rtg: %.2f", glm::degrees(retrograde_angle));
        ImGui::Text("Eng: %.2f J", o.energy);
    });

    // Initial size comes from o_telemetry.initial_size (a 2x2 grid of plots
    // needs real estate; content-fit would clip them).
    drawWin(g, W_Telemetry, [&] {
        // A 2x2 grid of plots. Each cell is a child region with a dropdown to
        // pick which series to show, then the plot. Positioned explicitly
        // (SetCursorPos) so the grid stays a clean 2x2.
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
        // Bme/Sit: the science identity of this pose (game.h poseSituation).
        // Biome is "-" when there is none to name (star, banded giant, or
        // terrain still building).
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

    /* --info-log: the same quantities ORBITAL and SURFACE display, raw SI
       (the instrument-log convention in fmt.h: e2e CHECK parses them).
       Independent of whether the windows are open -- the values come from
       the same ShipView snapshot the draw bodies read. */
    if(args.info_log && ship) {
        const Uint32 now_ms = SDL_GetTicks();
        if(now_ms - g.info_log_last_ms >= g.orbit_log_interval_ms) {
            g.info_log_last_ms = now_ms;
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
            if(view.plane.node_ok) { snprintf(lan_s, sizeof lan_s, "%.6g deg", glm::degrees(view.plane.lan)); }
            else { snprintf(lan_s, sizeof lan_s, "-"); }
            if(view.plane.lpe_ok) { snprintf(lpe_s, sizeof lpe_s, "%.6g deg", glm::degrees(view.plane.lpe)); }
            else { snprintf(lpe_s, sizeof lpe_s, "-"); }
            // r/apo_r/peri_r are RADII from the focus, not altitudes; the
            // [surfinfo] line below carries the real alt_asl / alt_agl.
            printf("[orbinfo] t=%.1fs body=\"%s\" vel=%.6g m/s r=%.6g m "
                   "apo_r=%.6g m apo_t=%.6g s peri_r=%.6g m peri_t=%.6g s "
                   "period=%.6g s inc=%.6g deg ecc=%.6g sma=%.6g m "
                   "plane=%s lan=%s lpe=%s prg=%.6g deg rtg=%.6g deg "
                   "energy=%.6g J/kg\n",
                   time, b->name.c_str(), speed, distance,
                   o.apoapsis, o.time_to_apo, o.periapsis, o.time_to_peri,
                   o.period, glm::degrees(view.plane.inc), o.ecc, o.semi_major,
                   refPlaneName(g.map_plane), lan_s, lpe_s,
                   glm::degrees(prograde_angle),
                   glm::degrees(retrograde_angle),
                   o.energy);
            printf("[surfinfo] t=%.1fs "
                   "alt_agl=%.6g m alt_asl=%.6g m "
                   "vs=%.6g m/s hs=%.6g m/s lat=%.6g deg lon=%.6g deg "
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
    // Buttons (natural width) + SameLine: a full-width Selectable in this
    // auto-resize window would swallow the line and push the "x" off it.
    std::vector<Vehicle *> all = collectVehicles(sys);
    bool removed = false;
    for(size_t i = 0; i < all.size() && !removed; i++) {
        Vehicle *v = all[i];
        const bool active = (v == ship);
        ImGui::PushID((void*)v);
        if(v->isCrewAboard()) {
            // a crew character aboard a capsule: in the fleet but not a
            // controllable ship (EVA it from the capsule window to make it free)
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
            // A crew member is selectable but not deletable -- remove_ship
            // refuses it -- so it gets no "x" to click.
            if(!v->isEva()) {
                ImGui::SameLine();
                if(ImGui::SmallButton("x")) {
                    g.remove_ship(v);
                    removed = true;   // the ship was deleted; stop iterating
                }
            }
        }
        ImGui::PopID();
    }
    ImGui::Separator();
    if(ImGui::Button("Spawn a copy of the active ship")) {
        if(ship == nullptr) {
            g.toast("Spawn: no active ship");
        } else if(!ship->defPath.empty()) {
            ships.spawn_ship(ship->defPath, "", ship->home, ship->scenario, sys, g.time);
        } else {
            printf("Spawn: active ship has no def (test ship)\n");
        }
    }
    ImGui::Text("click name - select    x - remove (not crew)");
    });

    drawWin(g, W_VesselInfo, [&] {
        ImGui::Text("Ship: %s", ship->name.c_str());
        ImGui::Text("Stage: %d / %d  (SPACE to drop)",
                    ship->activeStage(), ship->numStages());
        ImGui::Text("Mass: %.3fkg", ship->getMass());
        ImGui::Text("Delta-v: %.1fm/s", ship->getDeltaV());
        ImGui::Text("Thrust Util: %.0f%%", ship->thruster_util * 100);
        ImGui::Text("Thrust: %.2fN", ship->getThrust());
        ImGui::Text("Current TWR: %.2f/%.2f", ship->getTWR(), ship->getFullThrustTWR());
        ImGui::Text("Max TWR: %.2f", ship->getMaxTWR());
        ImGui::Text("Wheel torque: %.0fN m", ship->GetWheelTorque());
        ImGui::Text("Angular rate: %.2fdeg/s",
                    glm::degrees(glm::length(ship->partAngVel(ship->controller))));
    });
    drawWin(g, W_Controls, [&] {
        // Interactive rebind: click "rebind", then press a key (or a
        // Shift/Ctrl/Alt combo) to bind it. "clear" unbinds, "Reset all"
        // restores the default map, "Save" writes settings.json.
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
                // Unique per-row ID: the button labels ("rebind"/"clear")
                // repeat on every row, so without this all rows' buttons
                // share one window ID.
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
        // Toggle the autopilot: click a mode to engage it and click it again
        // to release. The modes are mutually exclusive, like a navball.
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

    drawWin(g, W_Resources, [&] {
        // aggregate across the active ship's parts (any ship layout); only
        // the resource types the ship has capacity for are shown
        static const char *resNames[(int)ResourceType::Num] = {
            "Hydrogen", "LOX", "Electric charge", "Oxygen", "Water", "Food",
            "Hydrazine", "Jet fuel"
        };
        // display order: fuels first, then life support (independent of the
        // enum order)
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

/* The open part windows: one plain imgui window per part the player
   right-clicked in the 3D view (g.part_sels, opened by pickAt). Plain
   Begin/End -- these are user-placed popups, NOT slot-layout windows.
   Closing the window (X) drops the entry; staging that drops the part
   makes it stale (the entry is dropped, not re-pointed). */
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

        // the human-readable part name (catalog display_name); fall back to
        // the machine id for catalogs that predate the field
        const char *disp = def->display_name.empty() ? def->name.c_str()
                                                     : def->display_name.c_str();

        // window title "<ship> > <part>". The ##suffix is a hidden ImGui
        // window id, unique per (ship, part).
        char name[256];
        snprintf(name, sizeof(name), "%s > %s##%p#%zu",
                 ship->name.c_str(), disp, (const void *)ship, part);
        if(!sel.placed) {
            // Open near the mouse: the part was picked there, so the
            // window appears just down-right of the pointer, flipping to
            // up-left when there is no room on the default side.
            const ImGuiViewport *vp = ImGui::GetMainViewport();
            const float maxw = 320.0f, maxh = 340.0f, gap = 12.0f;
            float x = (float)sel.mx + gap;
            float y = (float)sel.my + gap;
            if(x + maxw > vp->WorkPos.x + vp->WorkSize.x) {
                x = (float)sel.mx - gap - maxw;
            }
            if(y + maxh > vp->WorkPos.y + vp->WorkSize.y) {
                y = (float)sel.my - gap - maxh;
            }
            x = std::max(x, (float)vp->WorkPos.x);
            y = std::max(y, (float)vp->WorkPos.y);
            ImGui::SetNextWindowPos(ImVec2(x, y), ImGuiCond_Appearing);
            sel.placed = true;
        }
        bool open = true;
            /* a successful pickup may have dropped an EARLIER part_sels entry
               (the item ship's own window, via dropPartWindowsFor), shifting
               the entries this loop is indexing -- see the break at the end */
            bool worldChanged = false;
        if(ImGui::Begin(name, &open, ImGuiWindowFlags_NoSavedSettings)) {
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
            /* phase 3: effectiveMass -- a capsule with crew shows the total
               (body + crew), matching the ship HUD (getMass). */
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
            // EC is stored energy (watt-hours), not a substance (kg) -- a
            // battery's charge drains under load without losing mass.
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
            // Docking is intent-driven (Game::updateDocking): the active ship
            // docks only when it has BOTH an armed port of its own AND a
            // targeted port on another ship. The dock consumes both -- so
            // undocking cannot immediately re-dock.
            if(def->docking_port) {
                ImGui::Separator();
                Vehicle *act = g.ship;
                if(act != nullptr && !act->isEva()) {
                    if(act != ship) {
                        // This port is on another ship: make it the dock
                        // target (the other ship's half of the intent).
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
                        // This port is on the ACTIVE ship: arm it as the port
                        // that does the mating (this ship's half of the intent).
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
            // --- crew (this part is a capsule: holds EVA characters) --------
            // Aboard crew get an EVA button; a free kerbal within boarding
            // range gets a Board button. The transitions move the kerbal's
            // mass onto/off the capsule and park/restore its body.
            if(def->crew_capacity > 0) {
                ImGui::Separator();
                std::vector<Kerbal *> aboard = partCrew(ship->parts[part]);
                ImGui::Text("Crew: %d / %d", (int)aboard.size(), def->crew_capacity);
                for(size_t ci = 0; ci < aboard.size(); ci++) {
                    Kerbal *k = aboard[ci];
                    ImGui::PushID(k);
                    ImGui::Text("  %s", k->name.c_str());
                    if(ImGui::SmallButton("EVA")) {
                        g.kerbalEVA(k);
                    }
                    ImGui::SameLine();
                    // Per-crew experiment: the "pick a kerbal" path (no roles
                    // yet, so any of them can run the same observation).
                    if(ImGui::SmallButton("Exp")) {
                        g.runExperiment(k);
                    }
                    // What this kerbal is carrying (unrecovered): the count,
                    // and the names. A repeat (already in the career) is
                    // flagged -- it still banks, just less.
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
                        }
                    }
                    ImGui::PopID();
                }
                // Capsule-level "Run Experiment": first aboard crew (stable
                // for --sim-press).
                if(!aboard.empty()) {
                    if(ImGui::SmallButton("Run Experiment")) {
                        g.runExperiment(aboard.front());
                    }
                }
                // free kerbals in boarding reach: a Board button each, and --
                // the take/store dance -- a Store button when one holds a
                // finding and this capsule can receive.
                bool anyInRange = false;
                for(Kerbal *k : freeKerbals(g.sys)) {
                    // Board/Store gate: the same kerbalInRange the Take button
                    // and the headless hooks use -- one reach rule, one function.
                    if(!g.kerbalInRange(k, ship->parts[part])) { continue; }
                    anyInRange = true;
                    // dist is for the on-screen readout only (the gate above
                    // already did the same COM math).
                    const glm::dvec3 capCom =
                        ship->GetPositionRelTo(ship->parts[part], k->frame);
                    const double dist =
                        glm::length(k->get_center_of_mass() - capCom);
                    const bool full = ((int)aboard.size() >= def->crew_capacity);
                    ImGui::PushID(k);
                    ImGui::Text("  %s (%.1f m)%s", k->name.c_str(), dist,
                                full ? "  (capsule full)" : "");
                    if(ImGui::SmallButton("Board")) {
                        g.kerbalBoard(k, ship, part);
                    }
                    // Store (the dance deposit): this FREE kerbal is in
                    // reach; if they carry a finding and the capsule can
                    // receive, offer to deposit it. moveExperiment re-checks
                    // canHold (a Container is unlimited per family).
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
                                if(ImGui::SmallButton("Store  (into capsule)")) {
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
            // --- science (this part is a kerbal suit: holds experiments) ---
            // v1 only kerbals run/hold experiments (on their suit Part). The
            // capsule's Exp / Run Experiment buttons cover an ABOARD kerbal;
            // this is the free-EVA (or picked suit) path.
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
            // --- science instrument (this part runs + holds an experiment) ---
            // A science pod: an aboard kerbal runs the pod's experiment, which
            // lands on the pod and is recovered with the ship. No crew aboard
            // -> there is no one to run it with.
            if(!def->experiment_family.empty()) {
                Part *pod = ship->parts[part];
                ImGui::Separator();
                Kerbal *runner = nullptr;
                for(Vehicle *kv : ship->crew) {
                    Kerbal *k = static_cast<Kerbal *>(kv);
                    if(!k->parts.empty()) { runner = k; break; }
                }
                if(runner != nullptr) {
                    // ##id: a capsule def could one day carry both crew and
                    // a science family -- keep this button's ID distinct from
                    // the capsule section's "Run Experiment".
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
                // Take: a FREE (EVA) kerbal in reach of the pod moves its
                // finding onto their suit (the courier), to then store into a
                // capsule. Being free + in reach is the gate -- an aboard
                // kerbal can't reach out. moveExperiment re-checks the suit's
                // canHold (a Courier holds 1 per family).
                if(!pod->experiments.empty()) {
                    bool anyTake = false;
                    for(Kerbal *k : freeKerbals(g.sys)) {
                        // A successful take above empties the pod; the next
                        // iteration's canHold(pod->experiments[0]) would read
                        // an empty vector. Stop once it's spent.
                        if(pod->experiments.empty()) { break; }
                        if(!g.kerbalInRange(k, pod)) { continue; }
                        if(k->parts.empty() ||
                           !k->parts[0]->canHold(pod->experiments[0])) {
                            continue;
                        }
                        anyTake = true;
                        ImGui::PushID(k);
                        const std::string takeLabel =
                            "Take  (to " + k->name + ")##pod_take";
                        if(ImGui::SmallButton(takeLabel.c_str())) {
                            g.moveExperiment(pod, k->parts[0], 0);
                        }
                        ImGui::PopID();
                    }
                    if(!anyTake) {
                        ImGui::TextDisabled("Take  (needs a kerbal on EVA, in reach)");
                    }
                }
            }
            // --- inventory (this part is a container: holds items) --------
            // Contained items get a Drop button (it leaves into a free 1-part
            // ship, Game::dropItem). Free item ships within pickup range
            // (<= 10 m of the container, like boarding) get a Pick up button.
            if(def->inventory_capacity > 0) {
                Part *cont = ship->parts[part];
                ImGui::Separator();
                ImGui::Text("Inventory: %d / %d",
                            (int)cont->ownedContents.size(),
                            def->inventory_capacity);
                if(cont->ownedContents.empty()) {
                    ImGui::Text("  (empty)");
                }
                /* Snapshot: dropItem erases from ownedContents mid-loop, and
                   a vector erase invalidates the range-for's iterator. */
                std::vector<Part *> items = cont->ownedContents;
                for(Part *item : items) {
                    const char *in = item->def->display_name.empty()
                        ? item->def->name.c_str()
                        : item->def->display_name.c_str();
                    ImGui::PushID(item);
                    ImGui::Text("  %s  (%.1f kg)", in,
                                item->effectiveMass());
                    if(ImGui::SmallButton("Drop")) {
                        g.dropItem(item);
                    }
                    ImGui::PopID();
                }
                // free item ships in range: a Pick up button each (a 1-part
                // ship with a free part; kerbals board, not cargo; a crewed
                // ship is not stowable)
                bool anyInRange = false;
                for(Vehicle *v : collectVehicles(g.sys)) {
                    if(v->isEva()) { continue; }
                    if(v->parts.size() != 1) { continue; }
                    if(v->parts[0]->container != nullptr) { continue; }
                    if(!v->crew.empty()) { continue; }
                    if(v == ship) { continue; }
                    /* distanceTo: COM-to-COM in the common root frame
                       (updateProximity's idiom) -- a raw coordinate
                       difference would mix the two ships' frames */
                    const double dist = ship->distanceTo(v);
                    if(dist > 10.0) { continue; }
                    anyInRange = true;
                    ImGui::PushID(v);
                    ImGui::Text("  %s (%.1f m, %.1f kg)", v->name.c_str(),
                                dist, v->getMass());
                    if(ImGui::SmallButton("Pick up")) {
                        // picking up the ship you are flying leaves
                        // orbit-view (pickUpItem drops control) -- hand the
                        // player back to the carrier
                        const bool wasFlying = (g.ship == v);
                        if(g.pickUpItem(v, cont)) {
                            if(wasFlying) { g.select_ship(cont->owner); }
                            worldChanged = true;
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
            /* erase by identity, not by idx: a successful pickup may have
               removed an earlier entry (the item ship's window) and shifted
               this one off idx */
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

/* The focus frame's +X, passed to OrbitMap::setPlane as the screen-x reference.
   All three combo slots pin to it, so switching the plane combo tilts the
   picture rather than rotating it (#173, #185) -- on a focus whose rail is
   tilted off the system plane, leaving the Ecliptic slot to derive its basis
   from the node line put its screen-east 119 deg from the other two on Iapetus
   and 180 deg on Triton. */
static const glm::dvec3 kFocusEast(1.0, 0.0, 0.0);

/* The orbit map's plane for one combo slot: a normal in the focus's inertial
   frame, plus the screen-x reference (zero length = let OrbitMap derive the
   basis). Both maps and --map-dump go through here, so the three slots have one
   spelling rather than three that can drift -- #173 was exactly such a drift
   (the Equatorial slot was drawing the rail plane, not the equator). */
static void mapPlaneBasis(Frame *focusFrame, const glm::dvec3 &orbit_pos,
                          const glm::dvec3 &orbit_vel, bool have_ship, int mode,
                          glm::dvec3 &plane_n, glm::dvec3 &plane_x) {
    plane_n = glm::dvec3(0.0, 1.0, 0.0);
    plane_x = glm::dvec3(0.0, 0.0, 0.0);   // zero: OrbitMap derives the basis
    if(mode == kRefEquator) {
        // The focus's EQUATOR, not its rail plane (#173): (0,1,0) is the rail
        // normal and sits off the pole by exactly the axial tilt.
        plane_n = focusFrame->spinAxisRelTo(focusFrame);
        // Screen-x = the focus frame's +X, the same "east" the Ecliptic view's
        // canonical basis uses, so flipping the plane combo tilts the picture
        // instead of rotating it (~150 deg on Kerbin). This is a SCREEN choice,
        // not the readout's zero longitude: LAN measures from equator_orient *
        // +X, where the pole leans (uiRefPlane). Nothing draws a node marker,
        // so the two never visibly disagree -- do not "align" them without
        // adding one.
        plane_x = kFocusEast;
    } else if(mode == kRefEcliptic) {
        plane_n = glm::transpose(focusFrame->root_orient) * glm::dvec3(0.0, 1.0, 0.0);
        /* Pinned like the other two slots. Unpinned, a focus whose rail sits
           more than 8.13 deg off the system plane pushed setPlane off its
           canonical branch onto the node line, so this slot's screen-east
           disagreed with the other two's by 119 deg on Iapetus, 125 on Pluto,
           180 on Triton -- switching the combo rotated the picture instead of
           tilting it. It also drops the canonical branch's fixed e2, which is
           only in plane when the normal is exactly +-Y (on Venus it sits 3.4
           deg out of the drawn plane). */
        plane_x = kFocusEast;
    } else if(mode == kRefOrbit && have_ship) {
        const glm::dvec3 h = glm::cross(orbit_pos, orbit_vel);
        const double hl = glm::length(h);
        if(hl > 1e-9) { plane_n = h / hl; }
        /* Pin screen-x to the same focus +X the Equatorial slot uses (#185).
           Without it this slot derives its basis from the node line, which
           degenerates as h approaches the rail normal: the derived basis
           approaches (-Z, +X) while the near-polar canonical branch hands over
           (+X, +Z), so a ship pitching through a near-equatorial inclination
           watched its whole map turn a quarter turn at 8.13 deg (measured:
           tmp/t185jump.cpp). With +X pinned the two agree to 0.1 deg across
           that boundary, and the basis stays continuous for every h the game
           can produce -- the one exception is h exactly along the focus +-X, a
           single point, which setPlane guards against dividing by zero.
           An equatorial bed's h IS the focus pole, so its Equatorial and
           Orbital slots share a normal and now draw pixel-identical pictures:
           the map does not move when you switch between them. The ORBITAL
           readout's LAN/LPe do go blank there, though -- that plane has no zero
           longitude to measure from (uiRefPlane's has_zero), and that is a
           measurement fact, not this screen choice. */
        plane_x = kFocusEast;
    }
}

// The orbital map: right-clicking the window cycles its chrome (full
// window -> bare map -> no window), the map square draws the focus
// body's neighborhood (child-body orbits, SOI rings, the ship's
// trajectory + apside markers, the other ships, the transfer conic),
// and the controls above it edit the map state on the game.
void drawUIMap(Game &g) {
    TransferPlanner &planner = g.xferPlanner;
    Vehicle *ship = g.ship;
    OrbitElements &o = g.view.o;
    double &mu = g.view.mu;
    glm::dvec3 &orbit_pos = g.view.orbit_pos;
    glm::dvec3 &orbit_vel = g.view.orbit_vel;
    float &map_scale = g.map_scale;
    int &map_plane = g.map_plane;
    ImVec2 &map_pan = g.map_pan;
    int &map_mode = g.map_mode;
    std::vector<TransferPlanner::XferTarget> &xferTargets = planner.xferTargets;
    int &xfer_target = planner.xfer_target;
    auto &xfer = planner.xfer;
    // (orbit_caches, the per-orbit sampling cache, is file-scope --
    // shared with the Surface Map's orbit overlay.)

    // Mode 2 strips the window chrome entirely (see map_mode): the
    // window is invisible but still hit-tested, so the map below
    // keeps pan/zoom and the right-click cycle. The table's options
    // are const, so this one window draws from a per-frame copy.
    ui::Options mapOpts = kWins[W_OrbitalMap].opts;
    if(map_mode == 2) {
        mapOpts.flags |= ImGuiWindowFlags_NoDecoration |
                         ImGuiWindowFlags_NoBackground;
    }
    drawWin(g, W_OrbitalMap, mapOpts, [&] {
        // Right-click anywhere in the window cycles the chrome:
        // full window -> bare map -> no window -> full window.
        // Over the map this is safe: imgui owns the mouse here, so
        // the RMB camera orbit (gated on !WantCaptureMouse) never
        // fires.
        if(ImGui::IsWindowHovered() &&
           ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
            map_mode = (map_mode + 1) % 3;
        }
        // The map fills the window: the whole content width, and the
        // remaining height after the controls below (mode 0 only).
        const float avail_w = ImGui::GetContentRegionAvail().x;
        const float avail_h = ImGui::GetContentRegionAvail().y;
        // The controls block's height (mode 0 only), from the same style
        // the rows below use.
        const float isp = ImGui::GetStyle().ItemSpacing.y;
        const float legend_h = std::max(16.0f, ImGui::GetTextLineHeight());
        // Controls sit ABOVE the map (legend row + plane row), so they
        // stay visible even when the window is short; the map fills rest.
        const float controls_h = (map_mode == 0)
            ? (legend_h + isp)
              + (ImGui::GetFrameHeight() + isp)
            : 0.0f;
        const float map_w = std::max(0.0f, avail_w);
        const float map_h = std::max(0.0f, avail_h - controls_h);
        // The ship's trajectory around the focus: a closed ellipse (a
        // coasting Kepler orbit) or, when the ship is escaping or flying
        // by (ecc >= 1), an open hyperbolic/parabolic arc. Both draw the
        // same way (a projected polyline); only the sampling differs.
        const bool closed = (o.ecc < 1.0);
        // A reference into the cache (closed) or a local (open) -- never a
        // copy of the cache's point list every frame.
        std::vector<glm::dvec3> traj_local;
        const std::vector<glm::dvec3> *traj_pts = nullptr;
        if(closed) {
            // Sampled through a per-ship cache, trusted only while the
            // ship is on rails (coasting on its Keplerian conic). Off
            // rails -- Bullet-integrated, or right after a burn /
            // staging / SOI switch / crash -- the orbit is moving, so
            // re-sample every frame. See OrbitSampleCache. Fixed N (not
            // the body-orbit LOD count): the Surface Map shares this
            // cache entry and both may draw the ship in one frame.
            const int N = 64;
            traj_pts = &orbit_caches[(const void *)ship].sample(
                orbit_pos, orbit_vel, mu, N, ship->onRails);
        } else {
            // Open trajectory: an arc around periapsis, truncated where
            // it would run off to infinity. r_cap is the current view
            // extent (the map's larger dimension in world units) so the
            // curve reaches the edge of the view, but never smaller
            // than a few periapsis radii or the ship's current radius.
            const double r_cap = std::max<double>(
                std::max(map_w, map_h) * (double)map_scale,
                std::max(4.0 * o.periapsis, o.distance));
            const int N = 64;
            traj_local = sampleOpenTrajectory(orbit_pos, orbit_vel, mu, N, r_cap);
            traj_pts = &traj_local;
        }
    
        // Periapsis (both cases) and apoapsis (closed only). A closed
        // orbit propagates to each apsis (exact); an open arc has no
        // apoapsis, and its periapsis point is radius o.periapsis
        // along the eccentricity vector (which points to periapsis).
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
        // OrbitMap derives an in-plane basis from it.
        TerrainBody *focus = ship->m_parent;
        glm::dvec3 plane_n, plane_x;
        mapPlaneBasis(focus->frame, orbit_pos, orbit_vel, true, map_plane,
                      plane_n, plane_x);

        // KSP-inspired palette (P4): your orbit is green, the transfer
        // is blue, other bodies are gray. The focus body, ship dot and
        // labels use a near-black/white ink that contrasts with the
        // current style's window background. The selected transfer target
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
        // The near-body shell ring (science + surface-frame boundary),
        // gray like the SOI ring.
        const ImU32 shell_col = ImGui::GetColorU32(ImVec4(0.50f, 0.50f, 0.50f, 0.35f));
        // Atmosphere top: a desaturated-blue disk behind the body, so the
        // rim marks where the air ends (top() = 0 for airless bodies).
        const ImU32 col_atmo = ImGui::GetColorU32(ImVec4(0.35f, 0.50f, 0.66f, 0.20f));
        ImDrawList *dl = ImGui::GetWindowDrawList();

        // Controls (mode 0 only), pinned ABOVE the map so they stay
        // visible on a short window. Row 1: the color legend. Row 2:
        // the map plane (half-width) + a reset-view button.
        if(map_mode == 0) {
            // Legend: a compact color key (one line), abbreviated so it
            // fits a narrow window.
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
            legend("you", col_ship, false);
            ImGui::SameLine();
            legend("ships", col_vessel, false);
            ImGui::SameLine();
            legend("xfer", col_xfer, false);
            ImGui::SameLine();
            legend("bodies", col_child, false);
            ImGui::SameLine();
            legend("apsides", col_apsis, true);
            // Map plane (half-width) + reset view.
            static const char *kPlanes[] = { "Equatorial", "Ecliptic", "Orbital" };
            ImGui::PushItemWidth(avail_w * 0.5f);
            ImGui::Combo("Map plane", &map_plane, kPlanes, 3);
            ImGui::PopItemWidth();
            ImGui::SameLine();
            if(ImGui::Button("Reset view")) {
                map_pan = ImVec2(0.0f, 0.0f);
                map_scale = kMapDefaultScale;
            }
        }

        // The map fills the window below the controls (map_w x map_h,
        // defined at the top of the block); the focus (parent body)
        // sits at its center plus the pan offset.
        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        const float center_x = p0.x + map_w * 0.5f;
        const float center_y = p0.y + map_h * 0.5f;

        // Reserve the map area with an invisible button, sized to the
        // window (map_w x map_h). It captures the mouse, so a left-drag
        // over the map pans the map instead of moving the window. Wheel-zoom
        // and drag-pan both apply only while the mouse is over the map.
        ImGui::InvisibleButton("##mapnav", ImVec2(map_w, map_h));
        const bool over_map = ImGui::IsItemHovered();
        const ImGuiIO &g_io = ImGui::GetIO();
        if(over_map && g_io.MouseWheel != 0.0f) {
            // Wheel zooms to the cursor (the world point under the
            // mouse stays put). Reversed per preference: wheel UP zooms
            // IN (scale = meters/pixel goes down), wheel OUT zooms out.
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

        OrbitMap map;
        map.cx = center_x + map_pan.x;
        map.cy = center_y + map_pan.y;
        map.scale = map_scale;
        map.setPlane(plane_n, plane_x);
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
        auto draw_soi = [&](const glm::dvec3 &center, double soi_m, ImU32 col) {
            if(soi_m <= 0.0) { return; }
            const float r_px = (float)(soi_m / map_scale);
            if(r_px < 1.0f || r_px > 4000.0f) { return; }
            map.drawRing(dl, center, soi_m, col, 1.0f);
        };

        // Every body's orbit (around its own parent), projected into the
        // focus's frame -- see drawSystemBodyOrbits. Drawn before the
        // ship's orbit, so the ship sits on top.
        {
            const MapViewRect view{p0.x, p0.y, p0.x + map_w, p0.y + map_h};
            drawSystemBodyOrbits(g, focus, map, dl, view, map_scale,
                                 sel_body, col_child, col_sel, ink,
                                 soi_col);
        }
        // The focus body's own SOI -- the boundary of the current
        // gravitational regime the ship is inside.
        draw_soi(glm::dvec3(0.0, 0.0, 0.0), focus->frame->soi, soi_col);
        // The near-body (rotating-frame) shell -- the boundary that drives
        // the science orbit cut and the surface-frame flip. The lambda's
        // LOD hides it until zoomed in enough for it to matter.
        draw_soi(glm::dvec3(0.0, 0.0, 0.0), focus->rot_frame->soi, shell_col);
    
        // The atmosphere top (radius + top(), top() in meters above
        // sea level) as a desaturated-blue disk, under the orbit line.
        // Same min-pixel floor as the body disk, so the body never
        // pokes through when the atmosphere is sub-pixel.
        const double atmo_top = focus->surface.atmosphere.top();
        if(atmo_top > 0.0) {
            map.drawBody(dl, glm::dvec3(0.0, 0.0, 0.0),
                         focus->radius + atmo_top, col_atmo, 3.0f);
        }
        // closed=true for the ellipse (it is a closed loop); false for
        // the open arc (a chord would otherwise close it).
        map.drawOrbit(dl, *traj_pts, col_ship, 1.0f, closed);
        // The focus body's disk at the centre, with the same visibility
        // floor as the looped bodies.
        map.drawBody(dl, glm::dvec3(0.0, 0.0, 0.0), focus->radius,
                     col_body, 3.0f);
        // The ship: a bright dot (you are here) with a green ring, on
        // the line from the focus.
        const ImVec2 ship_px = map.px(orbit_pos);
        dl->AddLine(focus_px, ship_px, ink, 1.0f);
        dl->AddCircleFilled(ship_px, 5.0f, ink);
        dl->AddCircle(ship_px, 8.0f, col_ship, 0, 1.5f);
        // Prograde (velocity) arrow, along the ship's velocity.
        map.drawArrow(dl, orbit_pos, orbit_vel, 24.0f, col_ship, 1.5f);
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
                const int N = 64;
                const std::vector<glm::dvec3> &tpts =
                    orbit_caches[(const void *)s].sample(
                        r2, v2, mu, N, s->onRails);
                if(!tpts.empty()) {
                    map.drawOrbit(dl, tpts, col_vessel, 1.0f);
                }
                const ImVec2 tpx = map.px(r2);
                dl->AddCircleFilled(tpx, 3.0f, col_vessel);
                // Label offset from the marker (px): +x right, -y up.
                // Raise label_dx / label_dy to push names further out.
                const float label_dx = 6.0f, label_dy = 14.0f;
                dl->AddText(ImVec2(tpx.x + label_dx, tpx.y - label_dy),
                            col_vessel, s->name.c_str());
            }
        }

        // P3: the transfer conic to the selected target (planner has a
        // valid solution). It is a Kepler orbit under the focus's mu,
        // starting at the ship with velocity sol.v_departure and
        // propagated over sol.tof. The arc's end is the arrival point.
        // The solution belongs to the target the solve USED: xfer.valid
        // lags the TRANSFER window (drawn earlier in this same UI pass)
        // by a frame -- clearing/moving the combo there is not re-validated
        // until the next planner.update(), so pairing xfer.sol with the
        // live xfer_target read xferTargets[-1] when "none" was clicked.
        if(xfer.valid && xfer_target == xfer.target &&
           xfer.target >= 0 && xfer.target < (int)xferTargets.size()) {
            const TransferSolution &sol = xfer.sol;
            // Even-in-anomaly (not uniform-in-time) so the leg draws with an
            // even outline, like the closed orbits (see sampleTransferArc).
            std::vector<glm::dvec3> xfer_pts =
                sampleTransferArc(orbit_pos, sol.v_departure, mu, sol.tof, 64);
            map.drawOrbit(dl, xfer_pts, col_xfer, 1.5f, /*closed=*/false);
            const glm::dvec3 &arrival = xfer_pts.back();
            map.drawDot(dl, arrival, 4.0f, col_xfer);
            char xfer_label[96];
            snprintf(xfer_label, sizeof(xfer_label), "%s  %.0f m/s",
                     xferTargets[xfer.target].name, sol.total_dv);
            const ImVec2 apx = map.px(arrival);
            dl->AddText(ImVec2(apx.x + 5.0f, apx.y + 4.0f), col_xfer,
                        xfer_label);
        }
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

/* The main menu, one shared shell for every scene: the heading, the
   scene's navigation block (nav), and the standard items every menu shares
   (Save/Load, Settings, Controls, Quit game) + the version footer. Only the
   heading and nav differ per scene.

   Every item is a fixed-width button: the window is AlwaysAutoResize, and
   imgui measures its size from the PREVIOUS frame's content, so a Text item
   placed by hand (centered against the window width) feeds back into the
   measurement and the fit converges over several frames on first open. A
   button's width is explicit and imgui centers its label, so the layout is
   settled from the first visible frame.

   `isRoot` marks the scenes whose menu IS the scene (the title screen, the
   Space Center hub): forced open every frame, so TAB, "Reset windows" and any
   stray SetOpen cannot leave the scene with no UI at all. ui::Options::closable
   = false is NOT enough on its own -- it only hides the X button. The other
   menus are Transient overlays: closable, and each nav item closes its menu
   before the transition it starts (push / pop / enterTitle do not close
   windows).

   Quit to title is a NAV item, not a shared one: every scene has it except
   the title screen itself. `navBottom` is the same hook drawn LOWER -- between
   the shared toggles and "Quit game" -- for a scene's exits.

   Deliberately NOT in the menu -- each of these has a key: "Toggle windows" is
   TAB, "Reset windows" is F10, Game Debug Info is F1 and Telemetry is F2.
   They are not Windows-panel rows either (the panel lists the flight readouts
   you arrange; these are overlays you flip on). */
static void drawMenuWindow(Game &g, Win win, bool isRoot, const char *heading,
                           void (*nav)(Game &, float),
                           void (*navBottom)(Game &, float) = nullptr) {
    if(isRoot) { setWinOpen(win, true); }
    drawWin(g, win, [&] {
        bool &running = g.running;
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
        text_button(heading);
        if(nav) { nav(g, bw); }
        if(ImGui::Button("Save/Load", ImVec2(bw, 0.0f))) {
            setWinOpen(W_SaveLoad, !winOpen(W_SaveLoad));
        }
        // Toggles (not just open): a quick way to open or close these
        // (besides their X / Back).
        if(ImGui::Button("Settings", ImVec2(bw, 0.0f))) {
            setWinOpen(W_Settings, !winOpen(W_Settings));
        }
        if(ImGui::Button("Controls", ImVec2(bw, 0.0f))) {
            setWinOpen(W_Controls, !winOpen(W_Controls));
        }
        // The scene's exits (hub: "Return to title"), just above the app's own.
        if(navBottom) { navBottom(g, bw); }
        if(ImGui::Button("Quit game", ImVec2(bw, 0.0f))) {
            running = false;
        }
        ImGui::PopFont();
        // The build's git version (src/version.h, `make version`), grayed
        // out so it reads as a footer, not a menu item.
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.55f, 0.55f, 0.55f, 1.0f));
        text_button(VERSION);
        ImGui::PopStyleColor();
    });
}

/* The per-scene navigation blocks, drawn between the heading and the shared
   items. Each closes its own menu before the transition it starts. */
static void navTitle(Game &g, float bw) {
    // Opens the New Game setup sheet (system + difficulty) rather than
    // starting immediately; there is no game to go back to yet.
    if(ImGui::Button("New Game", ImVec2(bw, 0.0f))) {
        setWinOpen(W_NewGame, true);
    }
    // Toggle the README panel (left of this menu). Open by default; this
    // brings it back after an X / close.
    if(ImGui::Button("Readme", ImVec2(bw, 0.0f))) {
        setWinOpen(W_Readme, !winOpen(W_Readme));
    }
}
static void navSpaceCenter(Game &g, float bw) {
    // Push the editor on top of the hub; the VAB's "Back to game" pops back
    // here. (The hub's menu is Root -- re-opened every frame -- so the closes
    // below are a formality.)
    if(ImGui::Button("VAB", ImVec2(bw, 0.0f))) {
        setWinOpen(W_SpaceCenterMenu, false);
        vabOpen(g);
    }
    // The Tracking Station is another excursion on top of the hub; its "Back"
    // pops back here. Open even with no ship yet (a new game before the first
    // launch).
    if(ImGui::Button("Tracking Station", ImVec2(bw, 0.0f))) {
        setWinOpen(W_SpaceCenterMenu, false);
        pushScene(g, SceneId::TrackingStation);
    }
    // The Research Lab: the archive of recovered experiments, another
    // excursion on top of the hub. Open even before the first recovery.
    if(ImGui::Button("Research Lab", ImVec2(bw, 0.0f))) {
        setWinOpen(W_SpaceCenterMenu, false);
        pushScene(g, SceneId::ResearchLab);
    }
    // "Resume Flight" pops back to the flight below -- offered only when the
    // hub sits ON TOP of a live flight. When the hub IS the floor there is
    // nothing to pop back to, so the button is hidden.
    if(g.ship != nullptr && ImGui::Button("Resume Flight", ImVec2(bw, 0.0f))) {
        setWinOpen(W_SpaceCenterMenu, false);
        popScene(g);
    }
    // Recover Vessel ends the flight successfully: the active ship is deleted,
    // the player is left at the hub with no active vessel, and the Flight
    // Summary window opens. Offered only when a ship is active, and not for a
    // free EVA kerbal. No arm/confirm: this discards a flight you already
    // left, not the game. recoverActive also refuses unless the ship is
    // grounded on the home body -- the toast explains (or --recover-anywhere
    // lifts it).
    if(g.ship != nullptr && !g.ship->isEva()
       && ImGui::Button("Recover Vessel", ImVec2(bw, 0.0f))) {
        g.recoverActive();
    }
    // Return to title lives in navSpaceCenterExit, drawn by the shell just
    // above "Quit game". Esc in the hub never exits to the title -- it only
    // pops to the flight below (hubKeyActions).
    // A visual break between the scene's nav (above) and the shared items
    // (Save/Load .. Quit) the shell draws below.
    ImGui::Spacing();
}

/* The hub's bottom nav row: the exits, drawn by the shell between the shared
   toggles and "Quit game". */
static void navSpaceCenterExit(Game &g, float bw) {
    if(ImGui::Button("Return to title", ImVec2(bw, 0.0f))) {
        setWinOpen(W_SpaceCenterMenu, false);
        g.quitToTitle();
    }
}

// The two menus: one shell, one heading + navigation block each, both Root --
// the title screen and the Space Center hub ARE their menus (forced open every
// frame). The other scenes have no menu of their own: Esc walks up the tree
// and the hub is the only in-game menu.
void drawTitleMenu(Game &g) {
    drawMenuWindow(g, W_TitleMenu, true, "Open Space Program", navTitle);
}

void drawSpaceCenterMenu(Game &g) {
    drawMenuWindow(g, W_SpaceCenterMenu, true, "Space Center", navSpaceCenter,
                   navSpaceCenterExit);
}

/* The hub's top bar: the career state the hub is the right place to show --
   the home calendar clock, the science recovered so far, and how many vessels
   are out there (free kerbals included). Read-only. */
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
        ImGui::Text("Vessels: %d", (int)collectVehicles(g.sys).size());
    });
}

// A save-slot name is a single directory under a game dir. Whitelist to
// letters, digits, - _ . so a name can never carry a path separator or be a
// dot-name. (Defense-in-depth: delete_save also guards its base; this is the
// user-facing slot picker, so it stays inside saves/.)
static bool safeSlotName(const std::string &n) {
    if(n.empty() || n == "." || n == "..") { return false; }
    for(unsigned char c : n) {
        bool alnum = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                     (c >= '0' && c <= '9');
        if(!alnum && c != '-' && c != '_' && c != '.') { return false; }
    }
    return true;
}

// A game name is the tail of a game dir name under saves/ (<stamp>-<name>).
// Blocklist instead of the slot whitelist: forbid only what breaks a dir name
// -- path separators, the Windows-reserved set, control chars.
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

/* The New Game setup sheet: the game's name, which star system to load, and
   the exhaust-velocity scale (difficulty). Start switches system if needed,
   applies the scale and begins the game.

   The system list is a directory scan of res/systems, cached and re-read
   only when the directory's mtime changes (same gate as the VAB Load
   picker's res/ships list). */
void drawNewGame(Game &g) {
    // Selection + the scanned list persist across frames (and across a
    // close/reopen, so the last pick sticks).
    static std::vector<std::string> systems;
    static bool scanned = false;
    static std::filesystem::file_time_type dirMtime;
    static int sysSel = 0;
    // The game's display name (persistent across frames / close-reopen):
    // the dir under saves/ it mints is <stamp>-<name>.
    static char nameBuf[256] = "game1";
    // The slider's draft value, committed to args.exhaust_scale only by
    // Start. Bound live to args instead, it would survive Cancel / X and
    // leak into settings.json via "Save settings".
    static float exhaustSel = 1.0f;
    {
        std::error_code ec;
        const auto mtime = std::filesystem::last_write_time(
            resdir::path("res/systems"), ec);
        if(!ec && (!scanned || mtime != dirMtime)) {
            // Keep an in-progress pick across a rescan; only the FIRST scan
            // seeds from the running system (so Start is a no-op swap).
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
        // Re-seed the draft difficulty on every open, so Cancel / X (and a
        // close via the window chrome) discard the slider edits.
        if(ImGui::IsWindowAppearing()) {
            exhaustSel = g.args.exhaust_scale;
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
        } else if(ImGui::BeginCombo("##newgame_system",
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
        // Draft until Start (see exhaustSel above). Lower is harder (less
        // thrust + delta-v for the same fuel).
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::SliderFloat("##newgame_exhaust", &exhaustSel,
                           0.5f, 5.0f, "%.2fx");
        ImGui::TextDisabled("0.5x harder  ·  1.0x stock  ·  5.0x easier");
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
                if(g.startNewGame(nameBuf, path, exhaustSel)) {
                    setWinOpen(W_NewGame, false);
                }
            }
        }
        ImGui::SameLine();
        if(ImGui::Button("Cancel", ImVec2(120.0f, 0.0f))) {
            setWinOpen(W_NewGame, false);
        }
    });
}

/* The Flight Summary window (W_FlightSummary), opened by the hub's
   "Recover Vessel" (recoverActive). Shows the recovered vessel, the
   flight duration on the home calendar, and the SoI enter/leave journal.
   Transient like New Game. Space-Center-only. */
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

/* The title screen's README panel (left of the menu). Loads the
   player-facing readme once and shows it as plain text -- no markdown
   renderer, just the lines with HTML image tags dropped and the
   heading / bold markers stripped so it reads as text in a scroll box.

   Which file: release/README.md in the source tree (root README.md is
   the build-from-source doc), else README.md beside the assets (the
   name the tarball / AppImage package it as), else the AppImage's
   usr/share/doc/. First hit wins; a miss shows a placeholder rather
   than an empty window. */
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
            // Strip ATX heading markers ("# ", "## ", ...).
            size_t i = 0;
            while(i < line.size() && line[i] == '#') { i++; }
            if(i > 0 && i < line.size() && line[i] == ' ') {
                line.erase(0, i + 1);
            }
            // Strip **bold** markers (the release readme uses them for
            // the key names and list labels).
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
        // Scroll box: the readme is longer than a fitted panel, and the
        // window size is then free to stay user-resizable.
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
    // The slot name to save into, and the selected game + slot, are all
    // persistent (static): the name so the player does not retype it, and the
    // selections so the Load/Delete button acts on the row the player chose.
    static char nameBuf[256] = "save1";
    static int gameSel = 0;    // index into the games list
    static int slotSel = 0;    // index into the selected game's slots
    static int prevGameSel = -2;   // a game switch re-picks the first slot

    drawWin(g, W_SaveLoad, [&] {
        // The lists are directory scans, so read them only while the window
        // is open, and clamp the selections if the lists grew or shrank (a
        // save / delete). Clamp BOTH bounds: a frame with an empty list parks
        // the index at -1, and a later `list[-1]` is an out-of-bounds read.
        std::vector<GameEntry> games = list_games(datadir::saves());
        if(gameSel < 0 || gameSel >= (int)games.size()) {
            gameSel = (int)games.size() - 1;
        }
        std::vector<std::string> slots;
        if(gameSel >= 0) {
            slots = list_saves(datadir::saves() + "/" + games[gameSel].dirName);
        }
        if(gameSel != prevGameSel) { slotSel = 0; prevGameSel = gameSel; }
        if(slotSel < 0 || slotSel >= (int)slots.size()) {
            slotSel = (int)slots.size() - 1;
        }

        // --- save-as: capture the live fleet + crew + clock ----------------
        ImGui::TextWrapped(
            "Save the current game (%s) -- the live fleet, crew and clock -- "
            "into a slot of it.", g.gameName.c_str());
        ImGui::SetNextItemWidth(220);
        ImGui::InputText("##newslot", nameBuf, sizeof(nameBuf));
        ImGui::SameLine();
        // quicksave-NN is the F5 pool: a hand-saved slot under that name
        // would join the rotation and a wrap could silently clobber it.
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
            // Two half-width columns (games | slots of the selected game);
            // split the content region, leaving room for the item spacing.
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
                // loadFrom does the load, the scene decision and the failure
                // toast. It also adopts the game's identity (load_game), so
                // the next save lands in THIS game's dir.
                if(g.loadFrom(dir)) {
                    g.toast("Loaded %s/%s", games[gameSel].name.c_str(),
                            slots[slotSel].c_str());
                    setWinOpen(W_SaveLoad, false);
                }
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
                } catch(const std::exception &e) {
                    g.toast("Delete failed: %s", e.what());
                }
            }
        }
    });
}

void drawVabUI(Game &g) {
    /* TAB. The top bar is a registered window and drawWin already suppresses
       it, but the two panels below are raw ImGui::Begin calls -- they predate
       the window table and are not in it -- so this is the only gate they
       have. Bringing them into the table would let this go too. */
    if(!g.ui_visible) { return; }

    int hoveredLink = -1;   // the fuel-link list row under the mouse (the
                            // overlay lines below read it for the highlight)

    /* Free-port gizmos: a green dot at every unconsumed stack node,
       projected to window pixels (vabProject). Screen-space like KSP's
       port markers, and drawn on the foreground layer (no depth test) so
       the far-side ports stay discoverable. */
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
                if(!vabProject(g, vabNodePos(g, (int)i, (int)k), px, py)) { continue; }
                const ImVec2 c((float)px, (float)py);
                dl->AddCircleFilled(c, 4.0f, IM_COL32(90, 230, 140, 210));
                dl->AddCircle(c, 4.0f, IM_COL32(15, 60, 30, 255), 0, 1.5f);
            }
        }
    }

    // the save NAME (not a path -- .json / where the file lives are vabSave's
    // business). Seeded once from the build's name, editable.
    static char saveName[128] = {0};
    if(saveName[0] == 0) {
        snprintf(saveName, sizeof(saveName), "%s",
                 g.vab.build.name.empty() ? "untitled" : g.vab.build.name.c_str());
    }

    // the load picker: ship NAMES (stock res/ships + the player's data-dir
    // ships/; the data dir wins on a collision). Testships are filtered out.
    // Cached and re-read when either directory's mtime changes.
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

    /* Top bar: a fixed top-center window (no titlebar, not movable, not
       resizable) mirroring the HUD, in two lines:
         line 1 -- Back to game, the save-name input, Save, the ship picker
                   + Load (load a ship into the build, replacing it)
         line 2 -- the launch body + scenario dropdowns, then LAUNCH */
    drawWin(g, W_VabTopBar, [&] {
        // line 1: back to the game / save the build / load a ship
        if(ImGui::Button("Back to game##vab")) { vabClose(g); }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(140);
        ImGui::InputText("##savename", saveName, sizeof(saveName));
        ImGui::SameLine();
        if(ImGui::Button("Save")) { vabSave(g, saveName); }
        // Load: pick a ship by name, replace the build with it, and point the
        // save name at the same one so a following Save round-trips it.
        if(!shipNames.empty()) {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(160);
            const char *cur = shipNames[(size_t)loadSel].c_str();
            if(ImGui::BeginCombo("##vabload", cur)) {
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

        // line 2: where + how to launch (vabLaunch resolves both), then LAUNCH
        ImGui::SetNextItemWidth(160);
        if(ImGui::BeginCombo("##vabbody", g.vab.bodyName.c_str())) {
            for(size_t i = 0; i < g.sys.bodies.size(); i++) {
                const char *nm = g.sys.bodies[i]->name.c_str();
                if(ImGui::Selectable(nm, g.vab.bodyName == nm)) { g.vab.bodyName = nm; }
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(160);
        if(ImGui::BeginCombo("##vabscn", g.vab.scenarioName.c_str())) {
            for(size_t i = 0; i < scenario_count(); i++) {
                const char *nm = scenario_name_at(i);
                if(ImGui::Selectable(nm, g.vab.scenarioName == nm)) { g.vab.scenarioName = nm; }
            }
            ImGui::EndCombo();
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
    // Parts are selected by clicking them in the 3D view (vab.cpp LMB pick);
    // the panel below acts on that selection.
    if(g.vab.selected >= 0 && (size_t)g.vab.selected < g.vab.build.parts.size()) {
        ImGui::Separator();
        const int sel = g.vab.selected;
        BuildPart &bp = g.vab.build.parts[(size_t)sel];
        const char *selName = (bp.def != nullptr && !bp.def->display_name.empty())
            ? bp.def->display_name.c_str()
            : (bp.def != nullptr ? bp.def->name.c_str() : "?");
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
            // last: the vab ops swap the parts vector (the bp reference
            // dies with it)
            if(ImGui::Button("Detach subtree")) { vabDetachSelected(g); }
            ImGui::SameLine();
            if(ImGui::Button("Delete##sel")) { vabDeleteSelected(g); }
        }
    }
    ImGui::Separator();
    /* Fuel links: virtual from->to edges (no pose, no stage). Added with a
       two-click 3D pick, drawn as overlay lines (end of this function),
       managed from this list. */
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
    /* Subassemblies: multi-part subtrees detached instead of deleted (Del);
       a lone part just deletes. Arming one places COPIES of the whole tree;
       the entry survives placing -- copy & paste. */
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

    // Palette: arm a catalog part, then hover the ship and LMB to place it
    // (snaps to the hovered stack port, or surface-attaches at the hover
    // point). Hover/ghost targeting comes from the 3D pick (vab.cpp), not
    // from this list, so list hover must not overwrite g.vab.hover.
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - 290, 8),
                            ImGuiCond_Once);
    // Resizable: default size at creation, then the user owns the size.
    // The part list takes the top and resizes with the window: a negative
    // child height is an offset from the bottom edge, leaving room for the
    // symmetry/snap/status block below. Reserved at its max (armed + ghost +
    // symmetry>1), so the window's own content never needs a scrollbar.
    ImGui::SetNextWindowSize(ImVec2(280, 660), ImGuiCond_FirstUseEver);
    ImGui::Begin("Palette", nullptr);
    const float below = ImGui::GetStyle().ItemSpacing.y * 9.0f + 1.0f
        + ImGui::GetTextLineHeight()
        + ImGui::GetFrameHeight() * 3.0f
        + ImGui::GetTextLineHeight() * 4.0f;
    ImGui::BeginChild("palette_parts", ImVec2(0.0f, -below), true);
    for(size_t i = 0; i < g.ships.catalog().parts.size(); i++) {
        const PartDef &pd = g.ships.catalog().parts[i];
        if(pd.fuel_link) { continue; }
        const bool armed = (g.vab.armed == pd.name);
        // The row shows the human-readable display name (falling back to the
        // machine id). The ##id keeps the ImGui id unique per part.
        char lbl[256];
        snprintf(lbl, sizeof(lbl), "%s##%s",
                 pd.display_name.empty() ? pd.name.c_str() : pd.display_name.c_str(),
                 pd.name.c_str());
        if(ImGui::Selectable(lbl, armed)) {
            g.vab.armed = armed ? std::string("") : pd.name;
            g.vab.armedAsm = -1;     // exclusive with a subassembly
            g.vab.ghostRoll = 0.0;   // a fresh part starts unrolled
        }
    }
    ImGui::EndChild();
    ImGui::Separator();
    // Radial symmetry for SURFACE placing: N evenly-spaced copies ringing
    // the hovered parent's own long axis (1 = single part). Stack ports are
    // singletons, so symmetry does not apply there.
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
            // g.vab.armed holds the catalog id; show its display name like
            // the palette rows do.
            const PartDef *ad = g.ships.catalog().find(g.vab.armed);
            ImGui::Text("armed: %s",
                        (ad != nullptr && !ad->display_name.empty())
                        ? ad->display_name.c_str()
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

    /* Staging table: one row per stage period (flight order -- the first
       burn at the top), with vacuum delta-v and TWR against the home
       body's surface gravity. Fuel links are honoured. Recomputed every
       frame -- the build is small and this is pure math. */
    drawWin(g, W_Staging, [&] {
        const double gHome = (g.sys.home != nullptr) ? g.sys.home->g : 9.81;
        ImGui::Text("TWR on %s  (g = %.2f m/s^2)",
                    g.sys.home != nullptr ? g.sys.home->name.c_str() : "?",
                    gHome);
        if(g.vab.build.parts.empty()) {
            ImGui::TextDisabled("empty build -- place a part");
            return;
        }
        const std::vector<StageRow> rows =
            computeStaging(g.vab.build, gHome, g.args.exhaust_scale);
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

    /* Fuel-link overlay lines (drawn last, foreground layer: always on top,
       no depth test). Source centre -> destination centre, with the flow
       direction shown three ways: the source half dimmed, the destination
       half bright, and an arrowhead at the midpoint. */
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
            if(!vabProject(g, from->localPos, ax, ay)) { continue; }
            if(!vabProject(g, to->localPos, bx, by)) { continue; }
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
// Copies of the flight Ship List and Orbital Map windows, each renamed to its
// own window id so the Tracking Station versions can diverge from the flight
// ones without touching them.

void drawTrackingShipList(Game &g) {
    Vehicle *ship = g.ship;
    Ships &ships = g.ships;
    System &sys = g.sys;
    drawWin(g, W_TrackingShipList, [&] {
    // Back to the hub (the frame below); Esc does the same (trackingKeyActions).
    if(ImGui::Button("Back to Space Center")) {
        popScene(g);
    }
    ImGui::Separator();
    // Buttons (natural width) + SameLine: a full-width Selectable in this
    // auto-resize window would swallow the line and push the "x" off it.
    std::vector<Vehicle *> all = collectVehicles(sys);
    bool removed = false;
    for(size_t i = 0; i < all.size() && !removed; i++) {
        Vehicle *v = all[i];
        const bool active = (v == ship);
        ImGui::PushID((void*)v);
        if(v->isCrewAboard()) {
            // a crew character aboard a capsule: in the fleet but not a
            // controllable ship (EVA it from the capsule window to make it free)
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
            // A crew member is selectable but not deletable -- remove_ship
            // refuses it -- so it gets no "x" to click.
            if(!v->isEva()) {
                ImGui::SameLine();
                if(ImGui::SmallButton("x")) {
                    g.remove_ship(v);
                    removed = true;   // the ship was deleted; stop iterating
                }
            }
            if(active && !removed) {
                // Fly: back to the cockpit of this (already-active) ship --
                // the map is a view, this is the way back into it. Fly is
                // only drawn for the active ship, so enterFlight just
                // collapses the stack to the live flight. It skips its enter
                // when the base is already Flight, so syncShipFocus does the
                // re-centering.
                ImGui::SameLine();
                if(ImGui::SmallButton("Fly")) {
                    enterFlight(g);
                    g.syncShipFocus();
                }
            }
        }
        ImGui::PopID();
    }
    ImGui::Separator();
    if(ImGui::Button("Spawn a copy of the active ship")) {
        if(ship == nullptr) {
            g.toast("Spawn: no active ship");
        } else if(!ship->defPath.empty()) {
            ships.spawn_ship(ship->defPath, "", ship->home, ship->scenario, sys, g.time);
        } else {
            printf("Spawn: active ship has no def (test ship)\n");
        }
    }
    ImGui::Text("click name - select    x - remove (not crew)");
    });
}

void drawTrackingMap(Game &g) {
    TransferPlanner &planner = g.xferPlanner;
    Vehicle *ship = g.ship;
    OrbitElements &o = g.view.o;
    double &mu = g.view.mu;
    glm::dvec3 &orbit_pos = g.view.orbit_pos;
    glm::dvec3 &orbit_vel = g.view.orbit_vel;
    std::vector<TerrainBody *> &planets = g.sys.bodies;
    float &map_scale = g.map_scale;
    int &map_plane = g.map_plane;
    ImVec2 &map_pan = g.map_pan;
    std::vector<TransferPlanner::XferTarget> &xferTargets = planner.xferTargets;
    int &xfer_target = planner.xfer_target;

    /* Full-screen and chrome-less: the map IS the Tracking Station view.
       No controls below (they would overflow the auto-fit window off-screen);
       pan/zoom is the mouse wheel/drag, as in the flight map. Diverged from
       drawUIMap on purpose: the flight map keeps its resizable window and
       the right-click chrome cycle, this one is always full-screen and never
       touches the shared g.map_mode. */
    const ImGuiViewport *tvp = ImGui::GetMainViewport();
    // Edge-to-edge: cancel the slot's margin and zero the window padding so
    // the map content starts at the viewport origin.
    const ImVec2 vmarg = ui::Manager::Get().margin;
    ui::Options mapOpts;
    mapOpts.slot = ui::Slot::TopLeft;
    mapOpts.offset = ImVec2(-vmarg.x, -vmarg.y);
    mapOpts.fixed = true;   // re-placed every frame; not movable or resizable
    mapOpts.default_open = true;
    mapOpts.flags = ImGuiWindowFlags_NoDecoration |
                    ImGuiWindowFlags_NoBringToFrontOnFocus;
    // The map fills the whole work area, width and height INDEPENDENTLY (not a
    // square), so a wide screen is covered edge to edge rather than letterboxed.
    const float mapW = tvp->WorkSize.x;
    const float mapH = tvp->WorkSize.y;
    // Zero padding + zero border (edge-to-edge, no 1px line) and an opaque BLACK
    // canvas -- the Tracking map is its own black backdrop, not the theme's
    // window colour. The black also meets the loop's black Sky clear, so the
    // auto-fit window's few-px shortfall at the bottom/right shows no seam.
    // Pushed before drawWin so the body's contrastingColor(WindowBg) picks a
    // light ink for the labels on black.
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 1.0f));
    drawWin(g, W_TrackingMap, mapOpts, [&] {
        // The focus body: the ship's parent when there is one, else the home
        // body (a new game before its first launch has no ship). Everything
        // ship-specific is guarded on `ship` and simply absent then.
        TerrainBody *focus = ship ? ship->m_parent : g.home;

        // The ship's trajectory around the focus: a closed ellipse (a coasting
        // Kepler orbit) or, when the ship is escaping or flying by (ecc >= 1),
        // an open hyperbolic/parabolic arc. Empty with no ship.
        bool closed = false;
        std::vector<glm::dvec3> traj_local;
        const std::vector<glm::dvec3> *traj_pts = nullptr;
        glm::dvec3 peri_p, apo_p, tmp;
        bool have_peri = false, have_apo = false;
        if(ship) {
            closed = (o.ecc < 1.0);
            if(closed) {
                // Sampled through a per-ship cache, trusted only while the
                // ship is on rails. Off rails the orbit is moving, so
                // re-sample every frame. See OrbitSampleCache. Fixed N:
                // shared with the Surface Map cache entry.
                const int N = 64;
                traj_pts = &orbit_caches[(const void *)ship].sample(
                    orbit_pos, orbit_vel, mu, N, ship->onRails);
            } else {
                // Open trajectory: an arc around periapsis, truncated where
                // it would run off to infinity. r_cap is the current view
                // extent so the curve reaches the edge of the view, but never
                // smaller than a few periapsis radii or the ship's current
                // radius.
                const double r_cap = std::max<double>(
                    (double)std::max(mapW, mapH) * map_scale,
                    std::max(4.0 * o.periapsis, o.distance));
                const int N = 64;
                traj_local = sampleOpenTrajectory(orbit_pos, orbit_vel, mu, N, r_cap);
                traj_pts = &traj_local;
            }
            // Periapsis (both cases) and apoapsis (closed only). A closed orbit
            // propagates to each apsis (exact); an open arc has no apoapsis,
            // and its periapsis point is radius o.periapsis along the
            // eccentricity vector.
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

        // The map plane: a normal in the focus's inertial frame; OrbitMap
        // derives an in-plane basis from it. "Orbital" needs a ship, so
        // without one it stays on the default (rail) plane.
        glm::dvec3 plane_n, plane_x;
        mapPlaneBasis(focus->frame, orbit_pos, orbit_vel, ship != nullptr,
                      map_plane, plane_n, plane_x);
    
        // The map fills the window (the whole viewport here); the focus sits
        // at its center plus the pan offset. (mapW/mapH are defined at the
        // top of the function.)
        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        const float center_x = p0.x + mapW * 0.5f;
        const float center_y = p0.y + mapH * 0.5f;

        // Reserve the map area with an invisible button. It captures the mouse,
        // so a left-drag over the map pans it instead of moving the window
        // (imgui otherwise treats a drag on the window background as a move).
        // Wheel-zoom and drag-pan both apply only while the mouse is over it.
        ImGui::InvisibleButton("##mapnav", ImVec2(mapW, mapH));
        const bool over_map = ImGui::IsItemHovered();
        const ImGuiIO &g_io = ImGui::GetIO();
        if(over_map && g_io.MouseWheel != 0.0f) {
            // Wheel zooms to the cursor (the world point under the
            // mouse stays put). Reversed per preference: wheel UP zooms
            // IN (scale = meters/pixel goes down), wheel OUT zooms out.
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

        OrbitMap map;
        map.cx = center_x + map_pan.x;
        map.cy = center_y + map_pan.y;
        map.scale = map_scale;
        map.setPlane(plane_n, plane_x);

        // KSP-inspired palette (P4): your orbit is green, the transfer
        // is blue, other bodies are gray. The focus body, ship dot and
        // labels use a near-black/white ink that contrasts with the
        // current style's window background. The selected transfer target
        // is highlighted brighter than the other children.
        const ImVec4 bg = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
        const ImU32 ink       = contrastingColor(bg);
        const ImU32 col_ship  = ImGui::GetColorU32(ImVec4(0.20f, 0.80f, 0.40f, 1.0f));
        const ImU32 col_apsis = col_ship;  // periapsis / apoapsis: part of your orbit
        const ImU32 col_vessel = ImGui::GetColorU32(ImVec4(1.00f, 0.62f, 0.22f, 1.0f));
        const ImU32 col_child = ImGui::GetColorU32(ImVec4(0.55f, 0.55f, 0.55f, 1.0f));
        const ImU32 col_body  = ink;
        const ImU32 col_sel   = ImGui::GetColorU32(ImVec4(0.90f, 0.90f, 0.90f, 1.0f));
        const ImU32 soi_col   = ImGui::GetColorU32(ImVec4(0.50f, 0.50f, 0.50f, 0.30f));
        // The near-body shell ring (science + surface-frame boundary),
        // gray like the SOI ring.
        const ImU32 shell_col = ImGui::GetColorU32(ImVec4(0.50f, 0.50f, 0.50f, 0.35f));
        // Atmosphere top: a desaturated-blue disk behind the body, so the
        // rim marks where the air ends (top() = 0 for airless bodies).
        const ImU32 col_atmo = ImGui::GetColorU32(ImVec4(0.35f, 0.50f, 0.66f, 0.20f));
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
        auto draw_soi = [&](const glm::dvec3 &center, double soi_m, ImU32 col) {
            if(soi_m <= 0.0) { return; }
            const float r_px = (float)(soi_m / map_scale);
            if(r_px < 1.0f || r_px > 4000.0f) { return; }
            map.drawRing(dl, center, soi_m, col, 1.0f);
        };

        // Every body's orbit (around its own parent), projected into the
        // focus's frame -- see drawSystemBodyOrbits. Drawn before the
        // ship's orbit, so the ship sits on top.
        {
            const MapViewRect view{p0.x, p0.y, p0.x + mapW, p0.y + mapH};
            // The belts go down first: they are regions, so orbits, bodies
            // and labels should all read through them.
            drawDebrisBelts(g, focus, map, dl, view, map_scale);
            drawSystemBodyOrbits(g, focus, map, dl, view, map_scale,
                                 sel_body, col_child, col_sel, ink,
                                 soi_col);
        }
        // The focus body's own SOI -- the boundary of the current
        // gravitational regime (with a ship: the one it is inside; without:
        // home's).
        draw_soi(glm::dvec3(0.0, 0.0, 0.0), focus->frame->soi, soi_col);
        // The near-body (rotating-frame) shell -- the boundary that drives
        // the science orbit cut and the surface-frame flip. The lambda's
        // LOD hides it until zoomed in enough for it to matter.
        draw_soi(glm::dvec3(0.0, 0.0, 0.0), focus->rot_frame->soi, shell_col);

        // The atmosphere top (radius + top(), top() in meters above
        // sea level) as a desaturated-blue disk, then the body on top so
        // the visible rim is the air. Same min-pixel floor, so the body
        // never pokes through when the atmosphere is sub-pixel.
        const double atmo_top = focus->surface.atmosphere.top();
        if(atmo_top > 0.0) {
            map.drawBody(dl, glm::dvec3(0.0, 0.0, 0.0),
                         focus->radius + atmo_top, col_atmo, 3.0f);
        }
        // The focus body's disk at the centre (home when there is no ship),
        // with the same visibility floor as the looped bodies.
        map.drawBody(dl, glm::dvec3(0.0, 0.0, 0.0), focus->radius, col_body, 3.0f);

        // The ship itself, absent with no ship: its orbit (closed loop, or an
        // open arc -- closed=false so a chord does not close it), a bright dot
        // (you are here) with a green ring on the line from the focus, the
        // prograde arrow and the apside markers.
        if(ship) {
            map.drawOrbit(dl, *traj_pts, col_ship, 1.0f, closed);
            const ImVec2 ship_px = map.px(orbit_pos);
            dl->AddLine(focus_px, ship_px, ink, 1.0f);
            dl->AddCircleFilled(ship_px, 5.0f, ink);
            dl->AddCircle(ship_px, 8.0f, col_ship, 0, 1.5f);
            // Prograde (velocity) arrow, along the ship's velocity.
            map.drawArrow(dl, orbit_pos, orbit_vel, 24.0f, col_ship, 1.5f);
            // Apside markers are only meaningful for a non-circular orbit; an
            // open arc has periapsis but no apoapsis.
            if(o.ecc > 1e-3) {
                if(have_peri) { map.drawDot(dl, peri_p, 4.0f, col_apsis); }
                if(have_apo)  { map.drawDot(dl, apo_p,  4.0f, col_apsis); }
            }

            // Every ship in the system -- any SOI, not just the focus's: its
            // orbit (a Kepler conic around its OWN central body) projected
            // into the focus's frame, plus a dot + label at its position.
            // The player's own ship is already drawn above in green (skipped
            // here). Each ship's conic is sampled in its central body's
            // inertial frame, then the whole ellipse is rotated/translated
            // into the focus's frame. A ship on an escape trajectory has no
            // closed orbit to draw -- sample() returns empty -- so only its
            // marker shows.
            for(auto *b : planets) {
                for(auto *s : b->ships) {
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
                        map.drawOrbit(dl, tpts_f, col_vessel, 1.0f);
                    }
                    const ImVec2 tpx = map.px(O * r2 + P);
                    dl->AddCircleFilled(tpx, 3.0f, col_vessel);
                    // Label offset from the marker (px): +x right, -y up.
                    // Raise label_dx / label_dy to push names further out.
                    const float label_dx = 6.0f, label_dy = 14.0f;
                    dl->AddText(ImVec2(tpx.x + label_dx, tpx.y - label_dy),
                                col_vessel, s->name.c_str());
                }
            }
        }

        // No ship yet: the map shows the system around home, but there is no
        // orbit to track -- say so (the ship list beside it is already empty).
        if(!ship) {
            ImGui::Text("No ships yet -- launch one from the VAB.");
        }

    });
    ImGui::PopStyleColor();   // the black map background
    ImGui::PopStyleVar(2);    // WindowBorderSize + WindowPadding
}

// ---- Research Lab windows -------------------------------------------------
// The scene's single window: the science score + the FULL collection log, one
// row per bank in bank order. Read-only -- recoverActive grows the log.
//
// The rows are PRE-BUILT, not rebuilt per frame (issue #87): researchLabEnter
// fills g.labRows via labEntries, and the render walks them. The render also
// self-heals -- if g.science.version differs from g.labRowsVersion, it rebuilds
// first (a Load that commits while this scene is open changes the log; a
// refused one leaves the rows, and the career, untouched).

// Defined with the Atlas (below); declared here so scene entry can build the
// Atlas rows alongside the Lab's.
static void buildAtlasRows(Game &g);

void researchLabEnter(Game &g) {
    const Calendar &cal = g.sys.home ? g.sys.home->cal : Calendar{};
    g.labRows = labEntries(g.science.recovered, cal);
    g.labRowsVersion = g.science.version;
    // The System Atlas rows too (it's open by default in the lab scene).
    buildAtlasRows(g);
}

void drawResearchLab(Game &g) {
    drawWin(g, W_ResearchLab, [&] {
        // Back to the hub (the frame below); Esc does the same (labKeyActions).
        if(ImGui::Button("Back to Space Center")) {
            popScene(g);
        }
        // Toggle the System Atlas (its own window, docked right of the lab).
        // A checkbox rather than relying on the window's X alone: closing it
        // leaves no other way back in the lab, so this is the reopen path.
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
            // Self-heal: rebuild the rows if the career log changed since they
            // were built (a Load from this scene is the one way it can).
            if(g.labRowsVersion != g.science.version) {
                const Calendar &cal =
                    g.sys.home ? g.sys.home->cal : Calendar{};
                g.labRows = labEntries(g.science.recovered, cal);
                g.labRowsVersion = g.science.version;
            }
            // g.labRows is ready (built on entry, or just healed above), so
            // this is a plain walk. The archive outgrows the window as the
            // career grows, so the child fills the remaining height and scrolls.
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
// The system as a tree: the star at the root, planets under it, moons under
// their planet (recursively -- a moon of a moon nests one level deeper).
// One row per body: name (indented by depth), its research VALUE (a plain-
// english tier for "findings worth ×N of home" -- the exact × is a tooltip),
// the APPROACH Δv (the cost to get there, rounded to 50; "—" = home/star),
// and how much of the body's science has been DISCOVERED. Selecting a body
// adds its DOSSIER: the physical and orbital numbers (radius, mass, gravity,
// escape velocity, day length, tilt, air, and the orbit's a/e/apsides/period/
// tilt) -- the system JSON read back in friendly units, so sanity-checking
// what the flight windows show does not need the data file open. --atlas-dump
// prints the same rows to stdout.
//
// The rows are PRE-BUILT, not computed per frame (the Lab's issue #87 pattern):
// buildAtlasRows fills g.atlasRows on scene entry, drawResearchAtlas walks
// them, and both self-heal if g.science.version has moved since (a recovery
// while the lab is open, or a Load from it, grows the log).
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

// Total study-slots across the family table: each family contributes one slot
// per situation it can run in. Uniform denominator for every body.
size_t atlasTotalSlots() {
    size_t total = 0;
    for(const ExperimentDef &d : experimentDefs()) {
        total += (size_t)d.valid_in.size();
    }
    return (total > 0) ? total : 1;   // a missing table must not divide by 0
}

/* The dossier: what the Atlas knows about a body without opening the system
   JSON, so the lab can sanity-check the flight readouts. Every line is either
   authored there (radius / mass / g / the atmosphere block) or derived from it
   (mu, escape velocity, the orbit from the mean angular rate), so a line that
   disagrees with what the game flies means the derivation is wrong, not the
   data. Two derivations worth naming:
   - The orbit is inverted from orb_ang_speed (Kepler III) exactly as the
     loader places the body, and e comes from the epoch rail state's angular
     momentum -- these are the rails' own numbers, not the JSON's.
   - "plane tilt" is stated against three named references -- the system
     plane (ecl), the parent's orbital plane (parent) and the parent's
     equator (eq.) -- because for a moon of an inclined planet those are
     three different numbers, and "incl_ref" in the system JSON picks which
     one the authored orb_incl means. See the block that computes them. */
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
    // Both at the mean radius under the body's own mu: what a launch must
    // reach, and what an arrival burn must shed.
    line("escape velocity",
         fmt_speed(std::sqrt(2.0 * b.mu / (double)b.radius), v, sizeof v));
    line("surface orbit",
         fmt_speed(std::sqrt(b.mu / (double)b.radius), v, sizeof v));
    // solar_day_seconds: the measured solar day (#201). Civil clocks run
    // 86400 s days (#202) -- show the physical day here, not the civil one.
    line("day length", b.cal.solar_day_seconds > 0.0
                         ? fmt_time(b.cal.solar_day_seconds, v, sizeof v)
                         : "none (no spin)");
    if(b.rot_frame != nullptr) {
        // The pole IS the spin frame's +Y, and initial_orient carries the
        // authored tilt (tilt_azimuth turns the lean's direction, leaving its
        // angle, so col1.y alone gives the tilt).
        const double pole = glm::clamp(b.rot_frame->initial_orient[1].y,
                                       -1.0, 1.0);
        line("axial tilt", fmt_deg(std::acos(pole), v, sizeof v));
        line("surface shell", fmt_dist(b.rot_frame->soi, v, sizeof v));
    }
    if(b.frame != nullptr) {
        // The star's inertial frame carries the authored universe bound, not
        // an SoI (it has no parent whose tide could dominate).
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
            // The shell the loader actually BUILT, not the authored mode: a
            // flat-sea body never has one, and a mesh body has none until its
            // heavy phase lands. This is the fact the renderer uses, so the
            // e2e cases can pin the mode's effect rather than its spelling.
            if(b.ocean != nullptr) {
                line("ocean shell", fmt_dist((double)b.ocean_radius,
                                             v, sizeof v));
            }
        } else { line("seas", "none"); }
        // The authored relief; the MEASURED highest ground only once the
        // heavy phase has sampled the heightfield (max_height starts at 1 m).
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
        // The block can exist for the drawn rim alone, with no drag air
        // behind it (AtmosphereParams: the two halves are independent).
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
    // From the parent's centre (what ORBITAL's PeA/ApA show) and above its
    // mean sea level (what a periapsis-raising burn is measured in).
    const double surf = (double)par->radius + (double)par->surface.sea_level;
    line("periapsis", fmt_dist(a * (1.0 - e), v, sizeof v));
    line("periapsis alt", fmt_dist(a * (1.0 - e) - surf, v, sizeof v));
    if(e < 1.0) {
        line("apoapsis", fmt_dist(a * (1.0 + e), v, sizeof v));
        line("apoapsis alt", fmt_dist(a * (1.0 + e) - surf, v, sizeof v));
    }
    line("period", fmt_time(2.0 * std::numbers::pi / n, v, sizeof v));
    /* Plane tilt in three references, because for a moon of an inclined
       planet they are three different numbers: Bop authors orb_incl 15 deg
       against JOOL's plane, which itself sits 1.30 deg off the system plane.
       (ecl) is the SYSTEM plane -- the same one the flight readout's "ecl"
       and the map's Ecliptic view measure in, and the only one that means the
       same thing at every depth of the tree. (parent) is the plane the body's
       own rail is authored against when "incl_ref": "orbit"; (eq.) is the
       parent's EQUATOR, which is what "incl_ref": "equator" states. A line is
       dropped only when it repeats the (ecl) one. */
    const glm::dvec3 nrm = fr->orient * glm::dvec3(0.0, 1.0, 0.0);   // parent axes
    const double tilt_parent =
        std::acos(glm::clamp(fr->orient[1].y, -1.0, 1.0));
    // root_orient carries the whole parent chain, so this is the same normal
    // in the system frame -- and rails never spin their own frame, so it is
    // constant in time.
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
    /* Node and periapsis longitude in the system plane -- the reference the
       flight readout uses when the map is on Ecliptic, so the dossier and the
       HUD can be compared (RefPlane{} is the system plane: +Y normal, +X zero
       longitude). Dashed when undefined: an orbit lying IN the system plane
       has no node, and a near-circular one has no periapsis direction to
       point at. The authored node, though, lives in the PARENT's plane, and
       for a moon of an inclined planet that is a different number again: Bop
       authors lon_asc_node 10.0 and reads 61.22 in the system plane. So the
       parent-plane node joins the list whenever it differs. */
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

// Walk the frame tree from `b`, appending one row per body (in tree order).
// A body's inertial frame has TWO kinds of children: its OWN spin frame
// (rotating -- skipping it is what keeps this from recursing forever) and
// the inertial frames of the bodies that ORBIT it. `seen` also guards a
// cyclic orbit pair in the data (issue #129): a bad system then stops
// re-expanding an already-shown body instead of crashing the lab.
void atlasWalk(const System &sys, const TerrainBody *b, int depth,
               const std::map<std::string, std::set<AtlasSlot>> &covered,
               size_t total, std::vector<AtlasRow> &out,
               std::set<const TerrainBody *> &seen) {
    if(!seen.insert(b).second) { return; }   // already shown: stop the cycle
    AtlasRow r;
    r.rawName = b->name;   // the plain name, for the detail-pane selection key
    // Indent: a fixed 3 spaces per level (plain ASCII, reads as a tree
    // without relying on box-drawing glyphs, fine in any font).
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

// Build g.atlasRows from the current system + career log, and stamp the
// science version they were built from (for the self-heal check).
static void buildAtlasRows(Game &g) {
    const System &sys = g.sys;
    g.atlasRows.clear();
    g.atlasRowsVersion = g.science.version;
    if(sys.root == nullptr) { return; }

    // Which study-slots have been covered on each body. Recovered findings
    // only (the bank), counted per family+situation. NOTE (the limitation the
    // caption under the table states): a body's BIOMES are NOT separate slots
    // here -- that needs the heavy phase's max height, so a biome-rich body
    // reads LOWER than its true surveyed fraction. The % is a floor.
    std::map<std::string, std::set<AtlasSlot>> covered;
    for(const Experiment &e : g.science.recovered) {
        if(defFor(e.type) == nullptr) { continue; }   // unknown family: skip
        covered[e.body].emplace(AtlasSlot{ e.type, (int)e.situation });
    }
    const size_t total = atlasTotalSlots();

    std::set<const TerrainBody *> seen;   // cycle guard (issue #129)
    atlasWalk(sys, sys.root, 0, covered, total, g.atlasRows, seen);
}

void drawResearchAtlas(Game &g) {
    drawWin(g, W_ResearchAtlas, [&] {
        const System &sys = g.sys;
        if(sys.root == nullptr) {
            ImGui::TextDisabled("No system loaded.");
            return;
        }
        // Self-heal: rebuild the rows if the career log changed since they
        // were built (a recovery while the lab was open, or a Load from it).
        if(g.atlasRowsVersion != g.science.version) {
            buildAtlasRows(g);
        }
        if(g.atlasRows.empty()) {
            ImGui::TextDisabled("No bodies in this system.");
            return;
        }

        // Selection = the raw body name (a string, not an index, so it
        // survives the row set being rebuilt or the system changing). Default
        // to home, else the first body; keep it while it stays valid.
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

        // Split the window: the tree list on the left, the selected body's
        // dossier on the right. Two side-by-side children (not a table -- a
        // table cell can't fill the height independently); each scrolls.
        // The list is the narrow 1/3; the dossier gets the rest.
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
                // The dossier: the body's physical + orbital numbers, in
                // sections. Pre-built strings, so this window and --atlas-dump
                // print the same text (one derivation, two renderings).
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

/* --atlas-dump MS: every Atlas row (tree order + dossier) to stdout, for
   reading the system in a terminal and for e2e to pin. Prints the SAME
   pre-built strings the window draws -- one derivation, two renderings, so
   the terminal readout cannot drift from the lab. */
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

/* --map-dump MS: print the basis each map plane slot produces for the active
   ship. The map is pure drawing, so nothing else in the test stack can see a
   flipped e2 -- #181 was exactly that, invisible to every check but an
   eyeball. Reads the same mapPlaneBasis the two maps call, so the dump cannot
   disagree with the picture. */
void dumpMapBasis(Game &g) {
    Vehicle *ship = g.ship;
    if(!ship || !ship->m_parent || !ship->m_parent->frame) {
        // Say so: a silent no-op reads as "the map has no basis to report".
        printf("[mapdump] no active ship, nothing to dump\n");
        fflush(stdout);
        return;
    }
    Frame *focus_frame = ship->m_parent->frame;
    static const int kSlots[] = { kRefEquator, kRefEcliptic, kRefOrbit };
    for(const int slot : kSlots) {
        glm::dvec3 plane_n, plane_x;
        mapPlaneBasis(focus_frame, g.view.orbit_pos, g.view.orbit_vel, true,
                      slot, plane_n, plane_x);
        OrbitMap m;
        m.setPlane(plane_n, plane_x);
        /* A prograde body at +e1 moves along n x e1, which must read -e2 on
           screen in every slot (#181). +1 here means that slot draws the sweep
           the other way from the others. */
        const double sweep = glm::dot(glm::cross(m.n, m.e1), m.e2);
        // Sim time, like [orbitlog] / [orbinfo]: a CHECK that filters on t=
        // must mean the same instant across the logs.
        printf("[mapdump] t=%.1fs slot=%s n=(%.6f,%.6f,%.6f) e1=(%.6f,%.6f,%.6f) "
               "e2=(%.6f,%.6f,%.6f) sweep=%+d\n",
               g.time, refPlaneName(slot),
               m.n.x, m.n.y, m.n.z, m.e1.x, m.e1.y, m.e1.z,
               m.e2.x, m.e2.y, m.e2.z, sweep > 0.0 ? 1 : -1);
    }
    fflush(stdout);
}

