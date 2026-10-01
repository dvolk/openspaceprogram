// ui.h -- the imgui window wrapper: slot layout, size, reset.

#pragma once

#include <imgui.h>
#include <imgui_internal.h> // ImGuiWindow, FindWindowByName

#include <cstddef>
#include <float.h>
#include <string>
#include <unordered_map>

namespace ui {

// Set to true from anywhere to request a full UI reset; consumed by the
// next Window() call of the frame.
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

    // Place left of / right of / below the named window. The source must
    // be drawn earlier in the frame; if closed, the slot placement stands.
    const char* left_of = nullptr;
    const char* right_of = nullptr;
    const char* below = nullptr;

    // Initial size on layout; (-1, -1) = fit to content.
    ImVec2 initial_size = ImVec2(-1.0f, -1.0f);

    // Fixed width in font-size units (tracks font/DPI); height auto-fits.
    float fixed_width = -1.0f;

    // Per-frame size-constraint callback (SetNextWindowSizeConstraints).
    ImGuiSizeCallback size_cb = nullptr;
    void *size_cb_data = nullptr;

    bool fixed = false;      // no move, no resize; re-placed every frame
    bool closable = false;   // show the X close button (the menu windows)
    bool default_open = true; // open state a reset restores

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
        int layout_frame = 0;       // imgui frame of the last relayout
        bool has_rect = false;      // measured on screen rect, this frame
        int first_rect_frame = -1;  // imgui frame of the first measured rect
        ImVec2 rect_min, rect_max;
    };

    static Manager& Get() { static Manager m; return m; }

    ImVec2 margin = ImVec2(8.0f, 8.0f); // slot distance from viewport edges
    float spacing = 8.0f;               // gap used by left_of / right_of / below
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

    // Set open state without touching default_open (state() would overwrite it).
    void set_open(const char* name, bool open) {
        auto it = states.find(name);
        if (it == states.end()) {
            WinState st;
            st.open = open;
            it = states.emplace(name, st).first;
        }
        it->second.open = open;
    }

    // Full reset: new generation, default open states, layout pending.
    void reset_now() {
        generation++;
        for (auto& kv : states) {
            WinState& st = kv.second;
            st.open = st.default_open;
            st.applied_generation = 0;
            st.wait_frames = 0;
            st.has_rect = false;
            st.first_rect_frame = -1;
        }
        ResetFlag() = false;
    }

    void sync_frame() {
        if (ResetFlag())
            reset_now();
    }

    // Record the on-screen rect after a window's End(), for use as a
    // left_of / right_of / below source by later windows.
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
        if (st->first_rect_frame < 0)
            st->first_rect_frame = (int)ImGui::GetFrameCount();
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

    // Resolve this window's layout position. False while waiting for a
    // source window's rect; after max_wait frames the slot placement stands.
    bool resolve(const Options& o, WinState& self,
                 ImVec2& pos, ImVec2& pivot) {
        slot_anchor(o.slot, margin, pos, pivot);
        pos.x += o.offset.x;
        pos.y += o.offset.y;
        if (o.left_of != nullptr && !axis_from(o.left_of, self, true, true, pos, pivot))
            return false;
        if (o.right_of != nullptr && !axis_from(o.right_of, self, true, false, pos, pivot))
            return false;
        if (o.below != nullptr && !axis_from(o.below, self, false, false, pos, pivot))
            return false;
        return true;
    }

private:
    bool axis_from(const char* src, WinState& self, bool x_axis, bool left,
                   ImVec2& pos, ImVec2& pivot) {
        const WinState* s = find(src);
        if (s == nullptr || !s->open)
            return true; // source closed or unknown: slot placement stands
        if (!s->has_rect) {
            if (++self.wait_frames < max_wait)
                return false; // source declared later this frame: wait
            return true;      // it never showed up: slot placement stands
        }
        // imgui applies a window's content-fit size only on the NEXT
        // frame's Begin, so a first-appearance rect is not yet real. Wait
        // one frame (also after a re-layout of a non-fixed source).
        if (s->first_rect_frame >= (int)ImGui::GetFrameCount()) {
            if (++self.wait_frames < max_wait)
                return false;
            return true;
        }
        if (s->layout_frame == (int)ImGui::GetFrameCount() && !s->fixed) {
            if (++self.wait_frames < max_wait)
                return false;
            return true;
        }
        if (x_axis) {
            if (left) { pos.x = s->rect_min.x - spacing; pivot.x = 1.0f; }
            else      { pos.x = s->rect_max.x + spacing; pivot.x = 0.0f; }
        } else {
            pos.y = s->rect_max.y + spacing; pivot.y = 0.0f;
        }
        return true;
    }

    std::unordered_map<std::string, WinState> states;
};

// Size-constraint callback for fixed_width windows (width via UserData).
static void FixedWidthCallback(ImGuiSizeCallbackData* d) {
    d->DesiredSize.x =
        static_cast<float>(reinterpret_cast<std::size_t>(d->UserData));
}

// Draw a window with the wrapper's open state and layout. Returns true if drawn.
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
        flags |= ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;
    // fixed / fixed_width windows auto-fit every frame (user cannot resize).
    if (o.fixed || o.fixed_width > 0.0f)
        flags |= ImGuiWindowFlags_AlwaysAutoResize;

    // First layout of this generation -- or every frame for fixed windows.
    const bool relayout = o.fixed || st.applied_generation != m.generation;

    if (relayout) {
        // Windows closed by default open centered (least surprising).
        // Explicit left_of/right_of/below wins; fixed keeps its slot.
        Options o2 = o;
        if (!o2.default_open && !o2.fixed &&
            o2.left_of == nullptr && o2.right_of == nullptr &&
            o2.below == nullptr) {
            o2.slot = Slot::Center;
        }
        ImVec2 pos, pivot;
        if (!m.resolve(o2, st, pos, pivot))
            return false;
        st.wait_frames = 0;
        st.applied_generation = m.generation;
        st.layout_frame = (int)ImGui::GetFrameCount();

        if (o.initial_size.x > 0.0f && o.initial_size.y > 0.0f) {
            ImGui::SetNextWindowSize(o.initial_size, ImGuiCond_Always);
        } else {
            // One-shot content fit: re-triggers AlwaysAutoResize so the
            // user can resize freely from frame two on.
            flags |= ImGuiWindowFlags_AlwaysAutoResize;
        }
        ImGui::SetNextWindowPos(pos, ImGuiCond_Always, pivot);
        ImGui::SetNextWindowCollapsed(false, ImGuiCond_Always);
        ImGui::SetNextWindowScroll(ImVec2(0.0f, 0.0f));
    }

    // Width constraint is per-frame (NextWindowData): height tracks content.
    if (o.fixed_width > 0.0f) {
        // GetFontSize() includes the DPI scale, so width follows "Apply DPI".
        ImGui::SetNextWindowSizeConstraints(
            ImVec2(0.0f, 0.0f), ImVec2(FLT_MAX, FLT_MAX),
            FixedWidthCallback,
            reinterpret_cast<void*>(
                static_cast<std::size_t>(o.fixed_width * ImGui::GetFontSize())));
    } else if (o.size_cb != nullptr) {
        // Applied on initial size and on every user resize (size-constraint pass).
        ImGui::SetNextWindowSizeConstraints(
            ImVec2(0.0f, 0.0f), ImVec2(FLT_MAX, FLT_MAX),
            o.size_cb, o.size_cb_data);
    }

    // Closable windows pass open state to imgui so the X button closes them.
    bool* p_open = o.closable ? &st.open : nullptr;
    const bool visible = ImGui::Begin(name, p_open, flags);
    if (visible)
        body();
    ImGui::End();

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
