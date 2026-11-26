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

/* A readout row, for the harness as much as for the player: --ui-list can dump it
   and an e2e EXPECT can read the value the player sees (#601). Convert the rows a
   test should be able to read and leave the rest as ImGui::Text, so this stays a
   channel rather than a tax on every caption.
   The contract: one line of text (the dump is one line per item), and the row is
   NOT clickable -- it never matches a --ui-click or --ui-scroll target, so a case
   aimed at a caption fails the way it fails for a missing button. The draw costs
   what ImGui::Text cost; the registry entry costs nothing until the harness arms
   it, and one string copy once it has. */
void uiText(const char *fmt, ...) IM_FMTARGS(1);

/* The same row, drawn in the disabled colour: for a readout that is deliberately quiet (a
   cursor line, a unit label) but still has to be a row a test can read. Without this, the
   only way to get the quiet look is ImGui::TextDisabled, which registers nothing -- and a
   number the harness cannot see is a number that quietly stops being true (#601, #741).
   The colour comes from the style, so this is the SAME colour ImGui itself uses. */
void uiTextDisabled(const char *fmt, ...) IM_FMTARGS(1);

/* A row that needs an id the player must not see. imgui hides the "##" tail of a WIDGET
   label, but TextEx renders what it is given verbatim ("we don't hide text after ## in this
   end-user function", imgui_widgets.cpp:191) -- so uiText("%s##%s", name, id) puts the id on
   screen. This draws `shown` and registers "<shown>##<id>", which is what a Selectable's
   label already looks like to the harness (the Palette's "Capsule (2m)##capsule", #165's
   uniqueness rule), and the registered rect is the DRAWN text, so a clipping check measures
   what is actually on screen. Same rule as uiText: never a click target. */
void uiTextId(const char *shown, const char *id);

/* Which imgui window would take a click at this pixel, or nullptr when the 3D
   view under it would. Mirrors FindHoveredWindowEx's gate -- WasActive, not
   Hidden, no NoMouseInputs, inside OuterRectClipped -- because that is exactly
   what makes io.WantCaptureMouse true, which is what vabClickAfterUI and the
   flight pick refuse a click on. Runs before the frame's imgui pass, so it
   reports the previous frame's layout: the same one-frame lag the VAB hover
   already lives with. */
const char *uiWindowAtPixel(int px, int py);
