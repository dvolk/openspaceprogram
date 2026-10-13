// uiinput.h -- synthetic imgui interaction for the e2e tests: click a widget
// by its "Window/Label" path instead of by window pixels (see --ui-click).

#pragma once

struct Game;

/* Arm the synthetic imgui input: install the frame hook and the item registry
   while --ui-click / --ui-list work remains. Clicks resolve on the items the
   PRECEDING imgui pass drew and queue through imgui's own input path, so the
   widget activates through normal hit-testing. */
void emit_ui_input(Game &g);
