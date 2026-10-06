// uiinput.h -- synthetic imgui interaction for the e2e tests: click a widget
// by its "Window/Label" path instead of by window pixels (see --ui-click).

#pragma once

struct Game;

/* Arm the synthetic imgui input: install the imgui frame hook and turn on the
   item registry, while any --ui-click / --ui-list work remains. Called from
   emit_sim_events each loop iteration; the clicks themselves are resolved by
   the hook, on the items the PRECEDING imgui pass drew, and queued through
   imgui's own input queue so the widget activates through normal
   hit-testing. */
void emit_ui_input(Game &g);
