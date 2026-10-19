# Driving the UI in a case

The case format (NAME/ARGS/EXPECT/CHECK/...) is documented in `run.py`'s
docstring. This file is about the part that is easy to get wrong: clicking the
game's own widgets instead of using a headless hook.

Discover the address first. `--ui-list MS` prints every clickable item as
`window="..." label="..." rect=... [disabled]`, and `--ui-click MS,"Window/Label"`
matches that pair exactly, imgui's `##` disambiguator included. A path that
guesses at a label fails with `not found`; a path that matches two items fails
as `ambiguous`. `--ui-list` is the source of truth, not the source code.

A click presses in one frame and releases in the next, so a click can never be
same-frame with a hook phase. That is the only reason a hook still exists in a
UI-driven case: 244 needs a detach and a launch in one frame, 243 needs a scan
between two passes, 237 needs a plan the UI refuses to build.

For a 3D pick, `--ui-focus MS,PART_ID` parks the cursor on a pixel whose real
raycast lands on that part, and `--sim-mouse MS,0,CUR,CUR,BTN` clicks there. A
fixed pixel on a multi-part build picks whatever is under it, which is how 233
used to fail. The aim prints `[uifocus] ... -> x,y` or `FAILED <why>`, so every
such case carries `FORBID FAILED`. In the VAB, RMB there opens the part window,
whose title is the catalog label with the build id as its suffix --
`Capsule (3m)##capsule_r1.5h3_1/Seat 1##s0`, not a name you can invent.

A greyed-out row is an assertion, not a mistake. `--ui-click` refuses a disabled
widget, and the refusal line is what the case EXPECTs (236, 237). Pin WHICH
refusal: there is one at aim time and one at release (`FAILED disabled at
release`, uiinput.cpp:185), and the release wording contains the aim-time
substring, so forbid the other one.

An open combo owns the mouse. A refused row does not close the dropdown, so the
next click hits the popup's scrim and reports `MISMATCH nothing was hovered at
release`. Picking a row is the only way out -- click `##Combo_00/Empty` (236,
237). Nothing else closes it: the game binds Escape to a scene action and imgui's
keyboard nav is never enabled.
