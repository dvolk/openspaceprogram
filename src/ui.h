// ui.h -- the imgui window wrapper for the game UI.
//
// Handles the fiddly parts of the imgui window API in one place:
//
//   * layout: every window is placed into one of 9 screen slots (corners,
//     middle edges, center; default: center), with an optional pixel
//     offset. A window can also be placed to the right of, or below,
//     another window (right_of / below); the source must be drawn earlier
//     in the same frame.
//   * size: windows fit their content on first layout (or use
//     initial_size / width_ratio when set), and stay user-resizable
//     afterwards.
//   * freedom: the layout is applied once per generation; after that the
//     user can move and resize windows freely.
//   * reset: ui::ResetGui(), or `ui::ResetFlag() = true` from anywhere,
//     bumps the generation, restores every window's default open state and
//     re-applies the whole layout against the CURRENT viewport. That is the
//     recovery path for windows that wandered off screen after an
//     app-window resize.
//   * fixed windows (opts.fixed): not movable, not resizable, re-placed
//     every frame -- so they track viewport resizes on their own.
//
// The lambda form is a plain inlined Begin()/End() pair: zero perf cost
// versus writing them by hand (it's a template, so it's inlined into the
// caller exactly as if you'd written the body inline).

#pragma once

#include <imgui.h>
#include <imgui_internal.h> // ImGuiWindow, FindWindowByName

#include <cstddef>
#include <float.h>
#include <string>
#include <unordered_map>

namespace ui {

// Set to true from anywhere to request a full UI reset; consumed by the
// next Window() call of the frame. (static local: C++11 has no inline
// variables)
inline bool& ResetFlag() { static bool f = false; return f; }
inline void  ResetGui() { ResetFlag() = true; }

// 9 screen slots.
enum class Slot {
    TopLeft, TopCenter, TopRight,
    MiddleLeft, Center, MiddleRight,
    BottomLeft, BottomCenter, BottomRight,
};

// Per-window options. Leave a field at its default for plain behavior.
struct Options {
    Slot slot = Slot::Center;            // where to place the window
    ImVec2 offset = ImVec2(0.0f, 0.0f);  // pixel nudge from the slot anchor

    // Place this window's X to the right of / Y below the named window.
    // The source must be drawn earlier in the frame; if it's closed, the
    // slot placement stands.
    const char* right_of = nullptr;
    const char* below = nullptr;

    // Initial size on layout, both components; (-1, -1) = fit to content
    // (the default).
    ImVec2 initial_size = ImVec2(-1.0f, -1.0f);

    // width = width_ratio * height, where height is the content auto-fit.
    // -1 = off. For content with no meaningful width of its own (e.g.
    // full-width progress bars).
    float width_ratio = -1.0f;

    bool fixed = false;      // no move, no resize; re-placed every frame
    bool closable = true;    // show the X close button
    bool default_open = true; // open state a reset restores

    bool has_bg = false;     // use bg_color for the window background
    ImVec4 bg_color = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);

    ImGuiWindowFlags flags = 0; // extra raw imgui flags (e.g. NoTitleBar)
};

// Per-window state plus the layout generation the state belongs to.
class Manager {
public:
    struct WinState {
        bool open = true;
        bool default_open = true;
        bool fixed = false;          // from options: re-laid-out every frame
        int applied_generation = 0; // layout applied for this generation?
        int wait_frames = 0;        // frames spent waiting for a source rect
        int ratio_frames = 0;       // width-ratio constraint frames left
        int layout_frame = 0;       // imgui frame of the last relayout
        bool has_rect = false;      // measured on screen rect, this frame
        ImVec2 rect_min, rect_max;
    };

    static Manager& Get() { static Manager m; return m; }

    ImVec2 margin = ImVec2(8.0f, 8.0f); // slot distance from viewport edges
    float spacing = 8.0f;               // gap used by right_of / below
    int max_wait = 8;                   // frames to wait for a late source
    int generation = 1;

    // State for a window, created on first use (open = default_open).
    WinState& state(const char* name, const Options& o) {
        auto it = states.find(name);
        if (it == states.end()) {
            WinState st;
            st.open = st.default_open = o.default_open;
            st.fixed = o.fixed;
            it = states.emplace(name, st).first;
        }
        it->second.default_open = o.default_open;
        it->second.fixed = o.fixed;
        return it->second;
    }

    WinState* find(const char* name) {
        auto it = states.find(name);
        return it == states.end() ? nullptr : &it->second;
    }

    // Set the open state without touching default_open (state() would
    // overwrite it with the passed options).
    void set_open(const char* name, bool open) {
        auto it = states.find(name);
        if (it == states.end()) {
            WinState st;
            st.open = open;
            it = states.emplace(name, st).first;
        }
        it->second.open = open;
    }

    // Full reset: new generation, default open states, layout pending for
    // every window. The layout then resolves against the CURRENT viewport,
    // so this recovers windows off a resized app window.
    void reset_now() {
        generation++;
        for (auto& kv : states) {
            WinState& st = kv.second;
            st.open = st.default_open;
            st.applied_generation = 0;
            st.wait_frames = 0;
            st.ratio_frames = 0;
            st.has_rect = false;
        }
        ResetFlag() = false;
    }

    void sync_frame() {
        if (ResetFlag())
            reset_now();
    }

    // Record the on-screen rect after a window's End(), for use as a
    // right_of / below source by later windows.
    void cache_rect(const char* name) {
        WinState* st = find(name);
        if (st == nullptr)
            return;
        ImGuiWindow* w = ImGui::FindWindowByName(name);
        if (w == nullptr)
            return;
        st->rect_min = w->Pos;
        st->rect_max = ImVec2(w->Pos.x + w->Size.x, w->Pos.y + w->Size.y);
        st->has_rect = true;
    }

    // Slot anchor point + the window pivot that pins the window there:
    // e.g. TopLeft = (work_min, pivot 0,0), Center = (work_center, 0.5,0.5).
    static void slot_anchor(Slot slot, const ImVec2& m,
                            ImVec2& pos, ImVec2& pivot) {
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImVec2 min(vp->WorkPos.x + m.x, vp->WorkPos.y + m.y);
        ImVec2 max(vp->WorkPos.x + vp->WorkSize.x - m.x,
                   vp->WorkPos.y + vp->WorkSize.y - m.y);
        if (max.x < min.x) max.x = min.x;
        if (max.y < min.y) max.y = min.y;
        ImVec2 cx(vp->WorkPos.x + vp->WorkSize.x * 0.5f,
                  vp->WorkPos.y + vp->WorkSize.y * 0.5f);

        switch (slot) {
        case Slot::TopLeft:      pos = min;                    pivot = ImVec2(0.0f, 0.0f); break;
        case Slot::TopCenter:    pos = ImVec2(cx.x, min.y);    pivot = ImVec2(0.5f, 0.0f); break;
        case Slot::TopRight:     pos = ImVec2(max.x, min.y);   pivot = ImVec2(1.0f, 0.0f); break;
        case Slot::MiddleLeft:   pos = ImVec2(min.x, cx.y); pivot = ImVec2(0.0f, 0.5f); break;
        case Slot::Center:       pos = cx;   pivot = ImVec2(0.5f, 0.5f); break;
        case Slot::MiddleRight:  pos = ImVec2(max.x, cx.y); pivot = ImVec2(1.0f, 0.5f); break;
        case Slot::BottomLeft:   pos = ImVec2(min.x, max.y); pivot = ImVec2(0.0f, 1.0f); break;
        case Slot::BottomCenter: pos = ImVec2(cx.x, max.y); pivot = ImVec2(0.5f, 1.0f); break;
        case Slot::BottomRight:  pos = max;  pivot = ImVec2(1.0f, 1.0f); break;
        }
    }

    // Resolve this window's layout position. Returns false while waiting
    // for a source window's rect (the source is declared later in the
    // frame); after max_wait frames the slot placement stands.
    bool resolve(const Options& o, WinState& self,
                 ImVec2& pos, ImVec2& pivot) {
        slot_anchor(o.slot, margin, pos, pivot);
        pos.x += o.offset.x;
        pos.y += o.offset.y;
        if (o.right_of != nullptr && !axis_from(o.right_of, self, true, pos, pivot))
            return false;
        if (o.below != nullptr && !axis_from(o.below, self, false, pos, pivot))
            return false;
        return true;
    }

private:
    bool axis_from(const char* src, WinState& self, bool x_axis,
                   ImVec2& pos, ImVec2& pivot) {
        const WinState* s = find(src);
        if (s == nullptr || !s->open)
            return true; // source closed or unknown: slot placement stands
        if (!s->has_rect) {
            if (++self.wait_frames < max_wait)
                return false; // source declared later this frame: wait
            return true;      // it never showed up: slot placement stands
        }
        // The source was re-laid-out this frame and is not fixed: imgui
        // only applies a window's content-fit size on the NEXT frame's
        // Begin, so the rect we just measured is stale (min-size). Wait a
        // frame so the source settles. Fixed windows keep a stable size,
        // so their rect is trustworthy immediately.
        if (s->layout_frame == (int)ImGui::GetFrameCount() && !s->fixed) {
            if (++self.wait_frames < max_wait)
                return false;
            return true;
        }
        if (x_axis) { pos.x = s->rect_max.x + spacing; pivot.x = 0.0f; }
        else        { pos.y = s->rect_max.y + spacing; pivot.y = 0.0f; }
        return true;
    }

    std::unordered_map<std::string, WinState> states;
};

// imgui size-constraint callback for width_ratio windows: width =
// ratio * height (height = content auto-fit). imgui passes no window
// identity to the callback, so the ratio travels in the user-data
// pointer (imgui's documented pattern for scalar callback data).
static void WidthRatioCallback(ImGuiSizeCallbackData* d) {
    const float ratio =
        static_cast<float>(reinterpret_cast<std::size_t>(d->UserData)) / 1000.0f;
    d->DesiredSize.x = d->DesiredSize.y * ratio;
}

// Draw a window with the wrapper's open state and layout. Returns true if
// it was drawn (false if closed, or waiting on a source window's rect).
//
//     ui::Options o;
//     o.slot = ui::Slot::TopLeft;
//     ui::Window("Tools", o, [] {
//         ImGui::Text("Hello");
//     });
template <typename Body>
bool Window(const char* name, const Options& o, Body&& body) {
    Manager& m = Manager::Get();
    m.sync_frame();

    Manager::WinState& st = m.state(name, o);
    if (!st.open) {
        st.has_rect = false;
        return false;
    }

    ImGuiWindowFlags flags = o.flags | ImGuiWindowFlags_NoSavedSettings;
    if (o.fixed)
        flags |= ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                 ImGuiWindowFlags_AlwaysAutoResize;

    // First layout of this generation -- or every frame for fixed windows,
    // which must track viewport resizes and content changes.
    const bool relayout = o.fixed || st.applied_generation != m.generation;

    if (relayout) {
        // Windows closed by default open centered on screen: their slot
        // is where they sit in the default layout, but the first time the
        // user opens one, screen center is the least surprising spot.
        // Fixed windows keep their slot (they re-place every frame anyway).
        Options o2 = o;
        if (!o2.default_open && !o2.fixed) {
            o2.slot = Slot::Center;
            o2.right_of = nullptr;
            o2.below = nullptr;
        }
        ImVec2 pos, pivot;
        if (!m.resolve(o2, st, pos, pivot))
            return false;
        st.wait_frames = 0;
        st.applied_generation = m.generation;
        st.layout_frame = (int)ImGui::GetFrameCount();

        if (o.width_ratio > 0.0f) {
            // width = ratio * content height; the constraint is applied
            // over the next few frames (below) while the one-shot fit
            // settles.
            flags |= ImGuiWindowFlags_AlwaysAutoResize;
            st.ratio_frames = 3;
        } else if (o.initial_size.x > 0.0f && o.initial_size.y > 0.0f) {
            ImGui::SetNextWindowSize(o.initial_size, ImGuiCond_Always);
        } else {
            // One-shot content fit: imgui auto-fits a window for two frames
            // while its size is unknown; the flag re-triggers that on a
            // window that was already sized (reset), then drops off so the
            // user can resize freely from frame two on.
            flags |= ImGuiWindowFlags_AlwaysAutoResize;
        }
        ImGui::SetNextWindowPos(pos, ImGuiCond_Always, pivot);
        ImGui::SetNextWindowCollapsed(false, ImGuiCond_Always);
        ImGui::SetNextWindowScroll(ImVec2(0.0f, 0.0f));
    }

    // The ratio constraint is per-frame (imgui's NextWindowData), so it
    // must be re-issued for each frame while the one-shot fit settles.
    if (o.width_ratio > 0.0f && st.ratio_frames > 0) {
        st.ratio_frames--;
        ImGui::SetNextWindowSizeConstraints(
            ImVec2(0.0f, 0.0f), ImVec2(FLT_MAX, FLT_MAX),
            WidthRatioCallback,
            reinterpret_cast<void*>(
                static_cast<std::size_t>(o.width_ratio * 1000.0f)));
    }

    if (o.has_bg)
        ImGui::PushStyleColor(ImGuiCol_WindowBg, o.bg_color);

    bool* p_open = o.closable ? &st.open : nullptr;
    const bool visible = ImGui::Begin(name, p_open, flags);
    if (visible)
        body();
    ImGui::End();

    if (o.has_bg)
        ImGui::PopStyleColor();

    m.cache_rect(name);
    return visible;
}

// Open-state access (for checkboxes and the like).
inline bool IsOpen(const char* name) {
    const Manager::WinState* st = Manager::Get().find(name);
    return st != nullptr && st->open;
}

inline void SetOpen(const char* name, bool open) {
    Manager::Get().set_open(name, open);
}

} // namespace ui
