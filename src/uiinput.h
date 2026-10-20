// uiinput.h -- synthetic imgui interaction for the e2e tests: click a widget
// by its "Window/Label" path, or wheel-scroll a named window/item, instead of
// by window pixels (see --ui-click / --ui-scroll).

#pragma once

#include <imgui.h>   // ImGuiComboFlags

struct Game;

/* Arm the synthetic imgui input: install the frame hook and the item registry
   while --ui-click / --ui-scroll / --ui-list work remains. Clicks resolve on
   the items the PRECEDING imgui pass drew; scrolls use those items or the live
   window list, and queue through imgui's own input path, so the widget
   activates through normal hit-testing. */
void emit_ui_input(Game &g);

/* imgui never tells the test-engine hooks what a combo is called. BeginCombo
   submits its frame with the 3-arg ItemAdd(total_bb, id, &nav_bb), which emits
   no IMGUI_TEST_ENGINE_ITEM_INFO, so the combo sits in the registry under an
   empty name and --ui-click cannot reach it (#251) -- while a checkbox or a
   button next to it works fine. These wrappers register the label the widget
   already knows. Use them for EVERY combo, hand-rolled ones included, so a new
   combo is addressable by construction rather than by remembering;
   ImGui::EndCombo() closes uiBeginCombo as usual. */
bool uiCombo(const char *label, int *current,
             const char *items_separated_by_zeros);
bool uiCombo(const char *label, int *current, const char *const *items, int count);
bool uiBeginCombo(const char *label, const char *preview_value,
                  ImGuiComboFlags flags = 0);
