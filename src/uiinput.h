// uiinput.h -- synthetic imgui interaction for the e2e tests: click a widget
// by its "Window/Label" path instead of by window pixels (see --ui-click).

#pragma once

struct Game;

/* Arm the synthetic imgui input: install the frame hook and the item registry
   while --ui-click / --ui-list work remains. Clicks resolve on the items the
   PRECEDING imgui pass drew and queue through imgui's own input path, so the
   widget activates through normal hit-testing. */
void emit_ui_input(Game &g);

/* ImGui::Combo registers its frame with NO label: BeginCombo calls the 3-arg
   ItemAdd(total_bb, id, &nav_bb), which emits no test-engine ITEM_INFO, so the
   combo sits in the registry under an empty name and --ui-click cannot reach it
   (#251) -- while a checkbox or a button next to it works fine. These wrappers
   register the label the widget already knows. Use them for every combo so a new
   one is addressable by construction rather than by remembering. */
bool uiCombo(const char *label, int *current,
             const char *items_separated_by_zeros);
bool uiCombo(const char *label, int *current, const char *const *items, int count);
