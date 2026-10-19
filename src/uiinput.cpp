// uiinput.cpp -- see uiinput.h.
//
// With IMGUI_ENABLE_TEST_ENGINE defined (the Makefile defines it) imgui
// reports every drawn item's id, bounding box and label through the
// IMGUI_TEST_ENGINE_ITEM_* hooks. Implementing those hooks here, instead of
// linking the Test Engine, is what makes a widget addressable by name rather
// than by window pixel.

#include "uiinput.h"

#include <imgui.h>
#include <imgui_internal.h>   // ImGuiContext, ImGuiWindow, ImRect, the hook decls

#include <SDL3/SDL.h>

#include <cfloat>
#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

#include "game.h"
#include "siminput.h"   // UiClick

namespace {

struct UiItem {
    ImGuiID id;
    ImRect rect;      // the item's rect, in viewport coordinates
    ImRect visible;   // ... clipped to its window's clip rect when drawn
    std::string window;
    std::string label;
    /* A BeginDisabled() item still registers, still gets a rect, and still
       becomes HoveredId when pointed at (imgui.cpp:5078 -- "when disabled we'll
       return false but still set HoveredId"). So a click aimed at it looks clean
       and activates nothing. Record it and refuse. */
    bool disabled = false;
};

// The items of the imgui pass that just finished, and of the one in flight;
// the frame hook rotates the pair before imgui starts the next pass.
std::vector<UiItem> g_done;
std::vector<UiItem> g_filling;

// The click currently between press and release, and the item it was aimed
// at (0 = none). imgui needs the press and release on different frames, so
// only one is ever in flight.
UiClick *g_press = nullptr;
ImGuiID g_press_id = 0;
float g_press_x = 0.0f;
float g_press_y = 0.0f;

// Park the cursor on the frame after a release (see uiInputFrame).
bool g_park_next = false;

// Wall clock allowed for a window to open and settle before a --ui-click
// gives up.
const Uint32 kUiClickRetryMs = 2000;

// Wrap-safe form of `now >= at + kUiClickRetryMs` for a wrapping Uint32 clock.
bool pastDeadline(Uint32 now, Uint32 at) {
    return now >= at && now - at >= kUiClickRetryMs;
}

/* Compare a --ui-click path segment against a registered label EXACTLY.
   imgui's "##id" / "###id" disambiguators are part of the registered label
   and --ui-list prints them, so the dump is the source of truth. Stripping
   them would collapse every suffixed row to one unaddressable name; a wrong
   guess then fails loudly instead of hitting a lookalike. */
bool exactEq(std::string_view a, std::string_view b) {
    return a == b;
}

/* The items matching `path`. A qualified "Window/Label" is tried at every '/'
   in the path, rightmost first -- window names and labels both contain
   slashes, so no single split rule works. Otherwise the whole path is the
   label, in any window. */
void findMatches(const std::string &path, std::vector<const UiItem *> &out) {
    out.clear();
    for(size_t slash = path.size(); slash-- > 0; ) {
        if(path[slash] != '/') { continue; }
        const std::string_view win(path.data(), slash);
        const std::string_view lab(path.data() + slash + 1, path.size() - slash - 1);
        for(const UiItem &it : g_done) {
            if(exactEq(it.window, win) && exactEq(it.label, lab)) {
                out.push_back(&it);
            }
        }
        if(!out.empty()) { return; }
    }
    for(const UiItem &it : g_done) {
        if(exactEq(it.label, path)) { out.push_back(&it); }
    }
}

const char *labelOf(ImGuiID id) {
    for(const UiItem &it : g_done) {
        if(it.id == id && !it.label.empty()) { return it.label.c_str(); }
    }
    return nullptr;
}

// The live registry, for --ui-list: what can I click here, and what is it
// called?
void dumpUiItems(Uint32 now_ms) {
    printf("[uilist] loop=%.2fs items=%zu\n", now_ms / 1000.0, g_done.size());
    for(const UiItem &it : g_done) {
        if(it.label.empty()) { continue; }   // window title bars, decorations
        // A clipped item's rect is not clickable: say so, rather than have
        // --ui-click refuse a rect the dump advertised as live.
        const bool clipped = it.visible.Min.x > it.visible.Max.x
                          || it.visible.Min.y > it.visible.Max.y;
        printf("[uilist] window=\"%s\" label=\"%s\" rect=%.0f,%.0f-%.0f,%.0f%s%s\n",
               it.window.c_str(), it.label.c_str(),
               it.rect.Min.x, it.rect.Min.y, it.rect.Max.x, it.rect.Max.y,
               clipped ? " clipped" : "", it.disabled ? " disabled" : "");
    }
    fflush(stdout);
}

void failClick(const UiClick &c, Uint32 now_ms, const std::string &why) {
    printf("[uiclick] loop=%.2fs path=\"%s\" FAILED %s\n",
           now_ms / 1000.0, c.path.c_str(), why.c_str());
    fflush(stdout);
}

// The NewFramePre hook this module installs (0 = not installed).
ImGuiID g_hook = 0;

/* One imgui frame's worth of work: rotate the registry, serve --ui-list, and
   advance the click state machine.

   A NewFramePre hook rather than the game loop because the SDL3 backend
   queues mouse-position events of its own inside NewFrame, and events are
   consumed in queue order -- NewFramePre runs ahead of them, so ours land
   last. */
void uiInputFrame(ImGuiContext *ctx, ImGuiContextHook *hook) {
    Game &g = *static_cast<Game *>(hook->UserData);
    ImGuiIO &io = ctx->IO;

    // The pass that just finished is the map we aim at.
    g_done.swap(g_filling);
    g_filling.clear();

    const Uint32 now = SDL_GetTicks() - g.loop_start_ms;

    if(g.args.ui_list_ms >= 0 && now >= (Uint32)g.args.ui_list_ms) {
        dumpUiItems(now);
        g.args.ui_list_ms = -1;
    }

    /* Park the cursor a frame AFTER a release, never on the release frame
       itself: ButtonBehavior only reports pressed if the item is still
       hovered when the button comes up. */
    if(g_park_next) {
        io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
        g_park_next = false;
    }

    if(g_press != nullptr) {
        UiClick &c = *g_press;
        /* HoveredId here is last frame's (NewFrame rotates it after this
           hook), so it is what we aimed at on the press frame. If it is not,
           the window moved or another widget covers the point -- and imgui
           will activate whatever IS there, so say so rather than log a clean
           click. */
        if(ctx->HoveredId != g_press_id) {
            const char *lab = labelOf(ctx->HoveredId);
            failClick(c, now, lab != nullptr
                ? std::string("MISMATCH hovered \"") + lab
                  + "\" at release (the item moved or is covered)"
                : "MISMATCH nothing was hovered at release");
        } else if(ctx->HoveredIdIsDisabled) {
            // It turned disabled between the pass we aimed at and the press.
            failClick(c, now, "disabled at release (the gate closed between the"
                              " frame we aimed at and the frame we pressed)");
        }
        // Re-assert the position: the backend may have queued its own since.
        io.AddMousePosEvent(g_press_x, g_press_y);
        io.AddMouseButtonEvent(0, false);
        g_press = nullptr;
        g_press_id = 0;
        c.up_sent = true;
        c.done = true;
        g_park_next = true;
        return;
    }

    std::vector<const UiItem *> hits;
    for(UiClick &c : g.args.ui_clicks) {
        if(c.done || c.down_sent || now < c.at_ms) { continue; }
        findMatches(c.path, hits);
        if(hits.empty()) {
            if(pastDeadline(now, c.at_ms)) {
                failClick(c, now, "not found (no such item drawn within 2.0s;"
                                  " --ui-list dumps what is)");
                c.done = true;
            }
            continue;
        }
        if(hits.size() > 1) {
            /* Distinct candidates, in draw order: repeats of the same
               window/label pair are one problem, not many. */
            std::vector<std::string> cands;
            for(const UiItem *it : hits) {
                const std::string pair = it->window + "/" + it->label;
                bool seen = false;
                for(const std::string &s : cands) { if(s == pair) { seen = true; break; } }
                if(!seen && cands.size() < 4) { cands.push_back(pair); }
            }
            std::string why = "ambiguous (" + std::to_string(hits.size()) + " matches";
            if(cands.size() > 1) {
                why += ":";
                for(const std::string &s : cands) { why += " \"" + s + "\""; }
                why += ") -- qualify it as Window/Label";
            } else {
                why += ", all in \"" + cands[0] + "\") -- the labels are"
                       " identical; give the widgets distinct ## ids";
            }
            failClick(c, now, why);
            c.done = true;
            continue;
        }
        /* A --sim-mouse gesture may be holding the button; imgui filters a
           duplicate press, so pressing now would log a click that never
           reaches imgui. */
        if(io.MouseDown[0]) {
            if(pastDeadline(now, c.at_ms)) {
                failClick(c, now, "blocked (the left button is already down --"
                                  " a --sim-mouse gesture overlaps this click)");
                c.done = true;
            }
            continue;
        }
        const UiItem &hit = *hits[0];
        /* A disabled widget takes the click and does nothing with it, so a case
           that clicks one reads as a clean press and fails three EXPECTs later.
           Refuse here, where the reason is known. */
        if(hit.disabled) {
            failClick(c, now, "disabled (the widget is greyed out -- the gate"
                              " that greys it is what this case should assert)");
            c.done = true;
            continue;
        }
        /* ItemAdd fires before imgui's clip early-out, so an item scrolled out
           of its window still registers -- with a rect that would land the
           click on whatever is drawn there instead. */
        if(hit.visible.GetArea() <= 0.0f) {
            failClick(c, now, "clipped (the item is scrolled out of view)");
            c.done = true;
            continue;
        }
        const float cx = (hit.visible.Min.x + hit.visible.Max.x) * 0.5f;
        const float cy = (hit.visible.Min.y + hit.visible.Max.y) * 0.5f;
        if(cx < 0.0f || cy < 0.0f || cx > io.DisplaySize.x || cy > io.DisplaySize.y) {
            failClick(c, now, "off screen (the item is outside the viewport)");
            c.done = true;
            continue;
        }
        io.AddMousePosEvent(cx, cy);
        io.AddMouseButtonEvent(0, true);
        c.down_sent = true;
        g_press = &c;
        g_press_id = hit.id;
        g_press_x = cx;
        g_press_y = cy;
        printf("[uiclick] loop=%.2fs path=\"%s\" window=\"%s\" label=\"%s\" "
               "at=%.0f,%.0f pressed\n",
               now / 1000.0, c.path.c_str(), hit.window.c_str(),
               hit.label.c_str(), cx, cy);
        fflush(stdout);
        break;   // released next frame; no second press until then
    }
}

}  // namespace

void emit_ui_input(Game &g) {
    ImGuiContext *ctx = ImGui::GetCurrentContext();
    if(ctx == nullptr) { return; }

    // Armed only while work remains: the registry allocates per item per
    // frame, and a finished test should not pay it.
    bool pending = g.args.ui_list_ms >= 0;
    for(const UiClick &c : g.args.ui_clicks) {
        if(!c.done) { pending = true; break; }
    }
    if(g_press != nullptr) { pending = true; }
    ctx->TestEngineHookItems = pending;
    if(!pending || g_hook != 0) { return; }

    ImGuiContextHook hook;
    hook.Type = ImGuiContextHookType_NewFramePre;
    hook.Callback = uiInputFrame;
    hook.UserData = &g;
    g_hook = ImGui::AddContextHook(ctx, &hook);
}

// The four symbols imgui's test-engine macros reference; all four must link
// when imgui is built with IMGUI_ENABLE_TEST_ENGINE, including Log, which we
// never call.
void ImGuiTestEngineHook_ItemAdd(ImGuiContext *ctx, ImGuiID id, const ImRect &bb,
                                 const ImGuiLastItemData *item_data) {
    if(ctx == nullptr || ctx->CurrentWindow == nullptr) { return; }
    ImGuiWindow *w = ctx->CurrentWindow;
    /* Begin() registers the window itself (id == w->ID, labelled with the
       window's own name) and its title bar. Neither is a widget, and the
       window-root entry would shadow -- and so ambiguise -- a button sharing
       its window's name. */
    if(id == 0 || id == w->ID || id == w->MoveId) { return; }
    UiItem it;
    it.id = id;
    // bb is the NAV rect, which spans a whole table row or tree node for some
    // widgets; the drawn rect is what ButtonBehavior hit-tests.
    it.rect = item_data != nullptr ? item_data->Rect : bb;
    it.visible = it.rect;
    it.visible.ClipWith(w->ClipRect);
    it.window = w->Name ? w->Name : "";
    it.disabled = item_data != nullptr
               && (item_data->ItemFlags & ImGuiItemFlags_Disabled) != 0;
    g_filling.push_back(std::move(it));
}

void ImGuiTestEngineHook_ItemInfo(ImGuiContext *ctx, ImGuiID id, const char *label,
                                  ImGuiItemStatusFlags flags) {
    // ItemAdd registered this id earlier in the same pass; fill its label in.
    for(size_t i = g_filling.size(); i-- > 0; ) {
        if(g_filling[i].id == id) {
            if(g_filling[i].label.empty()) { g_filling[i].label = label ? label : ""; }
            return;
        }
    }
}

void ImGuiTestEngineHook_Log(ImGuiContext *ctx, const char *fmt, ...) {}

const char *ImGuiTestEngine_FindItemDebugLabel(ImGuiContext *ctx, ImGuiID id) {
    return labelOf(id);
}
