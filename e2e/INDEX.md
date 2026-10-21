# E2E index

Generated from each case's `CLAIM` and `KIND` by `python3 e2e/index.py`.
Read this before the case files: a case pins the thing in its claim, and
kind says how much weight the assertions carry.

| kind | meaning | cases |
|---|---|---|
| `invariant` | a behaviour or a player route the game must keep | 248 |
| `golden` | tuned numbers or a content table -- update deliberately when tuning | 30 |
| `harness` | CLI/test-engine plumbing, not game spec | 11 |
| `canary` | only proves the thing boots | 4 |
| | **total** | **293** |

There is no full battery: the runner takes at most 5 cases per
invocation and starts a 10-minute cooldown. Select by number, name or
path (a whole subject is usually too wide), e.g.
`python3 e2e/run.py save/172 262`.

## flight (53)

| case | kind | pins |
|---|---|---|
| `flight/02-staging-basic.txt` | invariant | Staging drops the spent stage and leaves the ship flying under the next one. |
| `flight/03-thrust-ascent.txt` | invariant | A full-throttle ascent gains altitude and speed while the engine burns. |
| `flight/04-orbit-burn.txt` | invariant | A prograde burn raises the apoapsis and turns a coast into an orbit. |
| `flight/05-rails-warp.txt` | invariant | Rails warp advances the conic and reports the orbit at the warp rate. |
| `flight/06-ship-switch.txt` | invariant | F6 cycles the fleet and hands the camera and HUD to the next vessel. |
| `flight/07-spin-regression.txt` | invariant | Radial-test stacks hold attitude with no phantom spin and no NaN. |
| `flight/08-mouse-orbit.txt` | invariant | RMB drag orbits the camera by exactly yaw -dx/200 and pitch +dy/200. |
| `flight/16-transfer-body.txt` | invariant | A body-transfer plan reports a departure burn that reaches the target SOI. |
| `flight/17-transfer-ship.txt` | invariant | A near-co-orbital rendezvous is a nudge, not a launch-class burn. |
| `flight/18-soi-warp.txt` | invariant | Rails warp crosses an SOI boundary and hands the ship to the new body at 1x. |
| `flight/19-exhaust-scale.txt` | golden | The exhaust plume follows throttle and engine scale within its clamp. |
| `flight/20-porkchop.txt` | invariant | The porkchop window creates and re-uploads its heatmap with no GL error. |
| `flight/22-attitude-physics.txt` | invariant | Rigid-body attitude answers the wheel and COM torques the physics asks for. |
| `flight/36-fuel-link-symmetric.txt` | golden | A symmetric fuel link drains both tanks at the same rate. |
| `flight/37-asparagus-drain.txt` | golden | Asparagus staging drains the outer tanks first and drops them in order. |
| `flight/38-asparagus-flight.txt` | golden | The full asparagus ascent stages five times with the expected thrust and drain rates. |
| `flight/39-prox-fly.txt` | invariant | Proximity checks report the right state for a ship flying near another. |
| `flight/40-prox-ground.txt` | invariant | Proximity checks report ground contact for a landed ship. |
| `flight/41-rebind.txt` | invariant | A rebound thrust key burns and the key it replaced no longer does. |
| `flight/42-thrust-latch.txt` | invariant | A thrust latch stays on after the key is released and clears on its own key. |
| `flight/43-ship-rcs.txt` | invariant | Ship RCS translates the vessel and changes its orbital energy. |
| `flight/44-ship-rcs-fuel.txt` | golden | RCS translation burns monopropellant at the expected rate. |
| `flight/45-dock.txt` | invariant | Two ports in range mate into one vessel and the absorbed hull leaves the world once. |
| `flight/46-undock.txt` | invariant | Undock splits the vessel in two and restores both as flyable ships. |
| `flight/47-dock-approach.txt` | invariant | A short approach burn closes the last metres and captures. |
| `flight/48-com-torque.txt` | invariant | Thrust through the COM produces no spurious torque. |
| `flight/49-drag-ascent.txt` | invariant | Drag reduces ascent acceleration and the drag log reports finite forces. |
| `flight/50-cam-shake.txt` | invariant | Camera shake tracks thrust, rises while burning and decays to zero. |
| `flight/51-jet-vtol.txt` | invariant | A jet VTOL lifts off and burns jet fuel. |
| `flight/79-dock-save-load.txt` | invariant | A docked pair survives save/load as one vessel. |
| `flight/80-glider-aoa-swing.txt` | invariant | A glider's AoA swings toward the trimmed angle and settles. |
| `flight/97-nuclear-engine.txt` | golden | A nuclear engine produces its rated thrust and hydrogen rate. |
| `flight/98-atmo-top.txt` | invariant | Above the atmosphere's top boundary density and drag are exactly zero. |
| `flight/99-rails-airless-arc.txt` | invariant | Rails warp accepts a suborbital arc on an airless body and parks it at the ground. |
| `flight/100-warp-ladder.txt` | invariant | Warp steps up and back down the ladder and reports each rate. |
| `flight/101-recover-ship.txt` | invariant | Recover retires the flown vessel exactly once and leaves an empty world. |
| `flight/102-recover-esc-multi.txt` | invariant | ESC after a recovery returns to the hub with the fleet intact. |
| `flight/103-flight-events.txt` | invariant | Flight events (liftoff, staging, atmosphere) are logged once, in order. |
| `flight/106-recover-refused.txt` | invariant | Recover refuses a flying ship, and the button is what is clicked. |
| `flight/107-recover-grounded.txt` | invariant | Recover succeeds for a ship grounded on its home body. |
| `flight/108-recover-wrong-body.txt` | invariant | Recover refuses a ship grounded on someone else's body. |
| `flight/150-chute-deploy.txt` | invariant | A chute arms and deploys only when its deploy condition is met. |
| `flight/151-chute-terminal.txt` | golden | A deployed chute reaches chute terminal velocity, not hull terminal. |
| `flight/152-chute-flip.txt` | golden | A chute below the COM weathervanes the ship nose-down; one above hangs it nose-up. |
| `flight/160-pad-rest-settle.txt` | golden | A pad ship spawns at its resting height and does not free-fall on the first ticks. |
| `flight/161-pad-ships-share-the-deck.txt` | golden | Every pad ship stands on its site deck, not beside or above it. |
| `flight/176-mouse-drag-camera.txt` | invariant | A zero-length RMB press must not open a part window and truncate the next drag. |
| `flight/193-porkchop-range.txt` | golden | The porkchop sliders scale their range with the target's own period. |
| `flight/255-tof-past-the-slider-band.txt` | golden | The ToF slider accepts a value past its old hard-coded band and the plan uses it. |
| `flight/309-rails-atmosphere-refused.txt` | invariant | Rails warp refuses a ship flying in air: the gate is the air, not periapsis. |
| `flight/310-rails-atmosphere-drop.txt` | invariant | Rails warp drops to 1x when a coasting conic reaches the atmosphere. |
| `flight/311-rails-departure-periapsis.txt` | invariant | Rails warp allows a high receding burn whose periapsis is under the ground. |
| `flight/312-soi-warp-max.txt` | invariant | Warp to the system's maximum still crosses an SOI and drops to 1x. |

## crew (24)

| case | kind | pins |
|---|---|---|
| `crew/26-eva-walk.txt` | invariant | WASD walks the kerbal on the surface without tipping it over. |
| `crew/27-eva-jump.txt` | invariant | A jump leaves the ground and lands back on it. |
| `crew/28-eva-rcs.txt` | invariant | Suit RCS thrust draws a puff and clears when the key is released. |
| `crew/29-eva-spawn.txt` | invariant | EVA spawns the roster's lowest-id kerbal at the capsule. |
| `crew/30-eva-rcs-fuel.txt` | golden | Suit RCS burns suit fuel at the expected rate. |
| `crew/31-eva-orbit.txt` | invariant | A kerbal can EVA in orbit and the suit state is kept. |
| `crew/81-crew-noncapsule.txt` | invariant | A kerbal can only be seated in a capsule, not an arbitrary part. |
| `crew/82-crew-overcapacity.txt` | invariant | A capsule refuses more crew than its capacity. |
| `crew/83-inventory-load.txt` | invariant | A loaded kerbal keeps the items in its pockets. |
| `crew/84-inventory-pocket.txt` | invariant | An item in a suit pocket rides the kerbal and can be taken out. |
| `crew/186-eva-walk-reload.txt` | invariant | A reloaded kerbal walks like a live one; foot friction is rebuilt. |
| `crew/187-eva-jump-padwall.txt` | invariant | A jump into the pad wall ends instead of gliding along the ground forever. |
| `crew/188-eva-yaw-reload.txt` | invariant | A spacewalking kerbal's QE yaw offset survives save/load. |
| `crew/207-crew-roster-named.txt` | invariant | A save carries the roster and each seat names its person by id. |
| `crew/208-crew-roster-unknown.txt` | invariant | A seat naming an unknown roster id is a refused load. |
| `crew/209-crew-roster-spent.txt` | invariant | A pool with more seats than people leaves the extra seats empty. |
| `crew/210-crew-roster-refill.txt` | invariant | Recovering a ship returns its crew to the pad and the next launch can re-seat them. |
| `crew/211-crew-name-clash.txt` | invariant | A ship may not take a kerbal's display name. |
| `crew/212-save-name-clash.txt` | invariant | A save with two ships named alike is refused rather than merged. |
| `crew/215-eva-board-round-trip.txt` | invariant | EVA out and board again leaves exactly one vessel and the suit parked as a seat. |
| `crew/224-ui-board-button.txt` | invariant | The part window's Board button returns a walking kerbal to its capsule. |
| `crew/225-startship-kerbal-multidef.txt` | invariant | A kerbal def with more than one part is refused at boot. |
| `crew/226-vab-launch-kerbal-multidef.txt` | invariant | The VAB refuses to launch a multi-part kerbal with a toast. |
| `crew/271-recover-refused-walking.txt` | invariant | Recover is offered for a walking kerbal and refuses without tearing the game down. |

## science (18)

| case | kind | pins |
|---|---|---|
| `science/104-science-experiment.txt` | golden | Running an experiment records a finding and its value. |
| `science/105-science-recover-load.txt` | golden | A recovered finding survives save/load in the career log. |
| `science/111-research-lab.txt` | invariant | The Research Lab shows the science log and score. |
| `science/116-science-landed.txt` | golden | A landed barometer reading scores the landed situation. |
| `science/117-materials-pod.txt` | golden | The materials pod holds several findings before they are couriered. |
| `science/118-materials-pod-dance.txt` | golden | Pod study, EVA take/store and recover score the sample. |
| `science/119-courier-two-findings.txt` | golden | One finding is couriered home and scores; one left on the walker does not. |
| `science/120-barometer.txt` | golden | A low-orbit barometer run records and recovers the low-orbit value. |
| `science/121-barometer-landed.txt` | golden | A landed barometer run scores the landed value. |
| `science/122-barometer-flying-low.txt` | golden | A flying-low barometer run scores the flying-low value. |
| `science/123-barometer-flying-high.txt` | golden | A flying-high barometer run scores the flying-high value. |
| `science/124-science-star-orbit.txt` | golden | A star-orbit experiment scores the space-high situation. |
| `science/125-jool-shell-vacuum.txt` | golden | A Jool-shell experiment classifies the vacuum situation. |
| `science/126-duna-flying-high.txt` | golden | A Duna flying-high run classifies the high band. |
| `science/127-phobos-inertial-orbit.txt` | golden | A Phobos inertial orbit classifies as low orbit. |
| `science/273-courier-pairs-across-the-fleet.txt` | golden | A courier pairs with the instrument beside it, not the fleet's first. |
| `science/284-seat-experiment-names-the-sitter.txt` | invariant | A per-seat Experiment button names the crew member who ran it. |
| `science/285-suit-experiment-from-the-walker.txt` | invariant | A walking kerbal's suit window runs its own experiment. |

## vab (37)

| case | kind | pins |
|---|---|---|
| `vab/25-part-pick.txt` | invariant | RMB in flight opens the part window for the part under the cursor. |
| `vab/52-vab-launch.txt` | invariant | LAUNCH from the VAB places the build on the pad. |
| `vab/56-vab-root-place.txt` | invariant | Clicking the palette then the 3D area roots a part in an empty build. |
| `vab/57-vab-launch-orbit.txt` | invariant | The start-bed dropdown launches the build on an orbit bed. |
| `vab/58-vab-launch-body.txt` | invariant | The body and start-bed dropdowns choose where the build launches. |
| `vab/59-vab-load.txt` | invariant | Load replaces the whole build rather than merging into it. |
| `vab/60-vab-close.txt` | invariant | Back to game parks and restores the camera to its exact flight distance. |
| `vab/77-vab-detach-lone.txt` | invariant | Detaching a leaf by a real pick removes it from the build. |
| `vab/78-vab-detach-asm.txt` | invariant | Detaching a subtree root stashes it and launches only what remains. |
| `vab/132-vab-save-rename.txt` | invariant | Save-as renames the ship identity in the file and on reload. |
| `vab/164-vab-pad-occupied.txt` | invariant | A launch onto an occupied pad asks before placing a second ship. |
| `vab/165-vab-pad-neighbour-clear.txt` | invariant | Occupancy is per pad: a ship on the polar pad does not block the main port. |
| `vab/183-vab-launch-no-ship.txt` | invariant | Launching from a shipless VAB does not sample NaN terrain. |
| `vab/184-vab-pad-launch-no-ship.txt` | invariant | A pad launch from a shipless VAB survives the first frame. |
| `vab/190-startship-bad-site.txt` | invariant | A bad --startship site fails at boot and lists the body's pads. |
| `vab/191-startship-site-on-orbit.txt` | invariant | A pad id on an orbit bed is refused at boot. |
| `vab/192-newgame-exhaust-clamp.txt` | golden | The New Game sheet clamps a seeded exhaust scale into its band. |
| `vab/233-vab-seat-assign.txt` | invariant | Naming a VAB seat flies that person and turns auto-fill off. |
| `vab/234-vab-seat-partial.txt` | invariant | Naming one seat leaves the other seats empty. |
| `vab/235-vab-seat-autofill.txt` | invariant | With no names, every seat is auto-filled from the pad. |
| `vab/236-vab-seat-full.txt` | invariant | Seats filled out of pad order land where the player put them. |
| `vab/238-vab-seat-emptied.txt` | invariant | Choosing Empty for a seat keeps auto-fill from filling it. |
| `vab/239-vab-seat-detached.txt` | invariant | Detaching a named capsule drops its plan and falls back to the pad line. |
| `vab/240-vab-seat-detached-two.txt` | invariant | Detaching one of two named capsules keeps the other's plan. |
| `vab/245-load-resolves-engine-shroud.txt` | invariant | A loaded engine resolves its shroud like a spawned one. |
| `vab/246-vab-launch-fuel-cycle.txt` | invariant | A fuel-link loop makes LAUNCH refuse with a toast, not abort. |
| `vab/247-vab-link-loop-refused.txt` | invariant | A fuel-link click that closes a loop is refused where it is made. |
| `vab/256-vab-link-parent.txt` | invariant | A part parented to a fuel link is refused by the parser. |
| `vab/278-vab-click-places.txt` | invariant | A synthetic click places a part in the VAB. |
| `vab/279-vab-click-through-window.txt` | invariant | A click inside a VAB window does not place a part through it. |
| `vab/283-tall-part-window-stays-on-screen.txt` | invariant | A tall part window re-fits so its lower rows stay on screen. |
| `vab/286-part-window-grows-for-late-rows.txt` | invariant | A part window grows when rows appear after it opened. |
| `vab/287-inventory-drop-row-addressable.txt` | invariant | Each inventory row's Drop button has a distinct addressable label. |
| `vab/288-vab-launch-site-by-name.txt` | invariant | The VAB site dropdown launches on the named pad. |
| `vab/306-vab-part-window-resources.txt` | invariant | A tank's part window reports the resources it carries. |
| `vab/307-vab-clear.txt` | invariant | Clear asks first, empties the workspace, then refuses when already empty. |
| `vab/308-vab-dev-vehicles.txt` | invariant | Dev vehicles appear in the Load picker, below the fold until scrolled. |

## save (49)

| case | kind | pins |
|---|---|---|
| `save/53-save.txt` | invariant | The Save window writes the post-burn state to a named slot. |
| `save/54-save-load.txt` | invariant | A saved ship loads back with the same parts and orbit. |
| `save/55-engine-shroud.txt` | invariant | A loaded engine resolves the shroud its def asks for. |
| `save/64-reload.txt` | invariant | The Save/Load sheet's Load button restores the staged slot. |
| `save/65-reload-refused.txt` | invariant | A reload of a missing or bad slot is refused and the running game survives. |
| `save/115-quicksave.txt` | invariant | Quicksave writes the current state and reloads it. |
| `save/128-boot-load-shipless.txt` | invariant | Boot --load of a shipless save lands on the hub, not the title. |
| `save/129-reload-shipless.txt` | invariant | An in-game reload of a shipless save moves to the hub. |
| `save/130-reload-refused-shipless.txt` | invariant | A refused same-system shipless reload leaves the old game intact. |
| `save/131-boot-load-shipless-cross.txt` | invariant | A cross-system shipless save loads onto the new system's hub. |
| `save/133-reload-refused-science.txt` | invariant | A refused reload does not clobber the running career's science. |
| `save/170-title-load-last-save.txt` | invariant | Load last save picks the most recently written save, not the newest game. |
| `save/171-title-no-save-no-resume.txt` | invariant | With no saves, the title offers no Load last save button. |
| `save/172-reload-refused-bad-format.txt` | invariant | A save declaring another format is refused loudly, not half-parsed. |
| `save/181-resave-round-trip.txt` | invariant | A save/load round trip preserves every field the writer emits. |
| `save/213-load-no-active-ship.txt` | invariant | A save without active_ship picks a vessel deterministically. |
| `save/214-load-active-ship-unknown-slug.txt` | invariant | A save naming a missing active ship is refused. |
| `save/216-load-crew-bad-def.txt` | invariant | A seat whose part is not a capsule is refused at load. |
| `save/217-load-crew-kerbal-cargo.txt` | invariant | A kerbal stowed as cargo in a seat's pocket is refused. |
| `save/218-load-crew-dup-profile.txt` | invariant | Two seats naming one roster id are refused. |
| `save/219-load-deep-containment.txt` | invariant | A containment tree past the depth cap is refused, not asserted. |
| `save/220-load-crew-multidef.txt` | invariant | A crew def that builds several parts is refused. |
| `save/221-load-item-seats.txt` | invariant | A seats array on an inventory item is refused. |
| `save/222-load-crew-top-seats.txt` | invariant | A seats array at a crew entry's top level is refused. |
| `save/223-load-crew-pocket.txt` | invariant | A seated kerbal's pocket tank contributes its mass. |
| `save/227-load-crew-capsule-def.txt` | invariant | A one-part non-kerbal def named as crew is refused. |
| `save/228-crew-pool-load.txt` | invariant | A save's pad cast is rebuilt as written, with no fallback mint. |
| `save/229-crew-pool-badpart.txt` | invariant | A crew-pool entry naming a non-person is refused and the running cast survives. |
| `save/230-crew-pool-dup.txt` | invariant | A roster id worn by both a seat and a pool suit is refused. |
| `save/231-new-game-crew-pool.txt` | invariant | New Game re-mints the pad cast in the new system, and a launch fills named seats from it. |
| `save/232-crew-pool-roster-rollback.txt` | invariant | A refused load rolls the roster back with the fleet. |
| `save/259-focus-list-seeded-once-across-systems.txt` | invariant | A cross-system boot seeds the focus list once, not twice. |
| `save/260-new-game-mints-its-own-game-dir.txt` | invariant | A New Game mints its own saves/ directory. |
| `save/261-refused-load-restores-clock.txt` | invariant | A refused load does not move the running game's clock. |
| `save/262-refuse-dangling-body-before-switch.txt` | invariant | A cross-system save naming a missing body refuses before the switch. |
| `save/263-refuse-dangling-pose-body.txt` | invariant | A same-system ship naming a missing body refuses instead of reloading at home. |
| `save/264-refuse-bad-building-before-switch.txt` | invariant | A system whose buildings cannot be placed is refused before the switch. |
| `save/266-save-leaves-no-temp-files.txt` | invariant | Saving leaves no .tmp files behind; each write is renamed into place. |
| `save/267-dock-intent-survives-save-load.txt` | invariant | A docking intent survives save/load. |
| `save/268-cross-system-unknown-roster-refuses-before-switch.txt` | invariant | A cross-system load with an unknown roster id refuses before the switch. |
| `save/269-cross-system-save-carries-its-own-roster.txt` | invariant | A cross-system save is validated against its own roster. |
| `save/304-load-from-the-save-load-sheet.txt` | invariant | The Save/Load sheet defers the load to the next frame and it succeeds. |
| `save/305-cross-system-deep-containment-refuses-before-switch.txt` | invariant | A cross-system deep containment failure refuses before the switch. |
| `save/314-save-game.txt` | invariant | The Save window mints a game directory and writes into it. |
| `save/315-cross-system-seat-on-non-capsule-refuses-before-switch.txt` | invariant | A cross-system seat-rule violation refuses before the switch. |
| `save/316-load-active-ship-none.txt` | invariant | An empty active_ship means no vessel is flown; an absent key means pick one. |
| `save/317-recover-leaves-nothing-flown.txt` | invariant | The writer records an empty active_ship after the last vessel is recovered. |
| `save/318-reload-refused-active-ship.txt` | invariant | An in-game reload with a bad active_ship refuses and the game survives. |
| `save/319-deep-tree-rolls-back-roster.txt` | invariant | A build-time refusal after a roster swap restores the running roster. |

## ui (45)

| case | kind | pins |
|---|---|---|
| `ui/61-title-boot.txt` | canary | A shipless boot lands on the title with a backdrop. |
| `ui/62-title-newgame.txt` | invariant | New Game from the title opens the hub. |
| `ui/66-quit-title.txt` | invariant | Flight, hub, Return to title asks and lands on the title. |
| `ui/67-vab-quit-title.txt` | invariant | VAB, hub, Return to title asks and lands on the title. |
| `ui/68-tracking-quit-title.txt` | invariant | Tracking, hub, Return to title asks and lands on the title. |
| `ui/69-tracking-back.txt` | invariant | Back to Space Center from Tracking pops to the hub. |
| `ui/70-spacecenter-back.txt` | invariant | The hub's back route leaves the game in a clean state. |
| `ui/71-tracking-esc-back.txt` | invariant | ESC pops the Tracking Station to the hub. |
| `ui/72-tab-reset-scene-entry.txt` | invariant | TAB hides the windows and the scene-entry state resets. |
| `ui/73-esc-scene-pop.txt` | invariant | ESC pops the current scene to the one below. |
| `ui/74-esc-vab-back.txt` | invariant | ESC in the VAB returns to the hub. |
| `ui/75-scene-switch-keys.txt` | invariant | The 1/2/3/4 scene shortcuts switch scenes. |
| `ui/76-scene-switch-title-denied.txt` | invariant | The scene shortcuts are denied on the title screen. |
| `ui/85-title-boot-full.txt` | canary | The full solar system boots to the title with a backdrop. |
| `ui/86-exit-midstream-full.txt` | canary | The game exits cleanly mid-stream on the full system. |
| `ui/87-switch-system.txt` | invariant | A New Game system picker switches systems and mints the game. |
| `ui/88-switch-save-load.txt` | invariant | A save from another system loads and switches. |
| `ui/89-switch-save-bad-system.txt` | invariant | A save naming an unknown system is refused. |
| `ui/90-load-fail-before-system-switch.txt` | invariant | A cross-system save that cannot be built leaves the running career intact. |
| `ui/91-boot-load-cross-system.txt` | invariant | Boot --load from another system switches and lands in flight. |
| `ui/92-title-load-spin.txt` | invariant | Loading a railed save from the title spins the surface correctly. |
| `ui/93-title-load-spin-t0.txt` | invariant | The same load at t0 spins the surface correctly. |
| `ui/94-boot-load-nonhome.txt` | invariant | Boot --load of a non-home-body ship lands in the right frame. |
| `ui/109-surfmap-body-switch.txt` | invariant | The Surface Map's body pin does not survive into a system that lacks it. |
| `ui/110-esc-spacecenter-noop.txt` | invariant | ESC on the hub is a no-op, not a pop to the title. |
| `ui/112-research-esc-back.txt` | invariant | ESC in the Research Lab returns to the hub. |
| `ui/134-ui-click-new-game.txt` | invariant | Clicking New Game then Start through imgui starts a game. |
| `ui/135-ui-click-scene-nav.txt` | invariant | The scene-nav buttons are clickable and switch scenes. |
| `ui/162-remove-dropped-stage.txt` | invariant | The Ship List's x removes a dropped stage's debris. |
| `ui/166-question-stay.txt` | invariant | The question card's Stay answer dismisses it and keeps the game. |
| `ui/167-question-return.txt` | invariant | The question card's affirmative answer routes on the answer frame. |
| `ui/168-question-quit.txt` | invariant | Quit game from a live game asks first. |
| `ui/169-question-quit-title-free.txt` | invariant | Quit game on the title quits in one click. |
| `ui/173-question-modal.txt` | invariant | The question card is modal: a click behind it does nothing. |
| `ui/174-autopilot-keys.txt` | invariant | Z and B toggle prograde and retrograde slew exclusively. |
| `ui/182-question-modal-esc-twice.txt` | invariant | Two ESCs in one batch answer the card once without a crash. |
| `ui/185-midpass-ship-switch.txt` | invariant | A ship switch inside the imgui pass aliases the new vessel safely. |
| `ui/241-target-selection-survives-reshuffle.txt` | invariant | Target selection is an identity and survives a fleet reshuffle. |
| `ui/242-porkchop-plan-survives-reshuffle.txt` | invariant | An armed porkchop plan survives deleting a different ship. |
| `ui/257-remove-ship-with-no-active-vessel.txt` | invariant | Removing a non-active vessel when none is flown does not crash. |
| `ui/275-ui-type-game-name.txt` | invariant | Typing a game name through the widget runs the field's own validation. |
| `ui/276-ui-type-illegal-game-name.txt` | invariant | An illegal game name keeps the sheet open and starts nothing. |
| `ui/277-save-slot-through-the-window.txt` | invariant | The Save/Load window's Save button writes the typed slot. |
| `ui/344-staff-lounge.txt` | invariant | The Staff Lounge opens from the hub, reports where each staff member is, and returns. |
| `ui/345-quit-title-stops-warp.txt` | invariant | Quitting to the title drops the flight's warp, so the title screen is not simulating. |

## render (55)

| case | kind | pins |
|---|---|---|
| `render/09-resolution.txt` | invariant | A resized window keeps the framebuffer and aspect correct. |
| `render/10-borderless.txt` | invariant | Borderless mode creates a fullscreen-desktop window. |
| `render/11-fullscreen.txt` | invariant | Fullscreen mode creates a fullscreen window. |
| `render/12-exclusive.txt` | invariant | Exclusive fullscreen is accepted or cleanly refused. |
| `render/13-frame-cap.txt` | invariant | The frame cap limits the frame rate to the requested value. |
| `render/14-postfx.txt` | invariant | The post-fx stack builds and applies without shader errors. |
| `render/15-postfx-alias.txt` | invariant | Sharpen aliasing stays within its configured bounds. |
| `render/21-display-mode-cycle.txt` | invariant | Cycling display modes round-trips and the window survives. |
| `render/23-surfmap.txt` | invariant | The Surface Map draws the body's shaded surface. |
| `render/24-surfmap-noshade.txt` | invariant | The Surface Map's no-shade mode draws without lighting. |
| `render/32-postfx-gamma-chain.txt` | invariant | A gamma chain applies each stage once. |
| `render/33-antialiasing.txt` | invariant | MSAA is requested and the framebuffer reports the sample count. |
| `render/34-postfx-color.txt` | invariant | Color post-fx applies the configured matrix. |
| `render/95-terrain-lod.txt` | invariant | Terrain LOD reaches the depth that carries collision under the camera. |
| `render/96-terrain-lod-farcam.txt` | invariant | A far camera still gets terrain collision under the ship. |
| `render/138-high-polar-equator.txt` | invariant | The high-polar bed is polar about the spin axis; inc is measured from the equator. |
| `render/139-plane-readout-tilted.txt` | invariant | Inc/LAN/LPe are measured in the map's default equatorial plane on a tilted body. |
| `render/140-plane-readout-untilted.txt` | invariant | On an untilted body the rail plane is the equator and LAN dashes for an in-plane orbit. |
| `render/141-plane-readout-ecliptic.txt` | invariant | The Ecliptic slot measures inc against the system plane. |
| `render/142-plane-readout-orbital.txt` | invariant | The Orbital slot measures against the ship's own plane and dashes LAN/LPe. |
| `render/143-map-plane-basis.txt` | invariant | Each map plane slot draws the basis it claims. |
| `render/144-spawn-equatorial-kerbin.txt` | invariant | The -eq beds spawn in the focus's equator on Kerbin. |
| `render/145-spawn-equatorial-uranus.txt` | invariant | The -eq beds behave on Uranus's 97.77-degree tilt. |
| `render/146-map-plane-basis-venus.txt` | invariant | A retrograde-spin body's map slots do not mirror each other. |
| `render/147-map-plane-basis-equatorial.txt` | invariant | The Orbital and Equatorial slots agree when they describe one plane. |
| `render/148-ocean-flat-earth.txt` | invariant | Earth's sea is painted flat instead of using the sphere shell. |
| `render/149-ocean-mesh-kerbin.txt` | invariant | Kerbin's sea uses the mesh shell with waves and glint. |
| `render/153-idle-occluded.txt` | invariant | An occluded window idles at the idle interval but the sim keeps ticking. |
| `render/154-idle-off.txt` | invariant | With idle off, a hidden window changes nothing and no counter appears. |
| `render/156-idle-rate-floor.txt` | invariant | The idle rate is clamped to physics_hz/8 and startup says so. |
| `render/157-view-live-unpainted.txt` | invariant | The Research Lab still advances the view snapshot without painting 3D. |
| `render/158-tracking-lod-live.txt` | invariant | The Tracking Station's terrain LOD keeps advancing behind the map. |
| `render/159-vab-view-live.txt` | invariant | The VAB's state pass takes a ship snapshot without terrain LOD. |
| `render/163-surfmap-launch-sites.txt` | golden | The Surface Map marks every launch pad on the mapped body. |
| `render/189-terrain-lod-drill.txt` | invariant | The terrain queue builds collision under the camera early. |
| `render/194-orbit-plane-overlay.txt` | invariant | The overlay draws the orbital and reference rings and their node ticks. |
| `render/195-orbit-plane-rot-frame.txt` | invariant | The overlay's reference normal is correct in the rotating frame. |
| `render/196-orbit-plane-coplanar.txt` | invariant | Coplanar planes produce one ring and no node tick. |
| `render/197-orbit-plane-no-orbit.txt` | invariant | A grounded ship gets no plane overlay. |
| `render/198-orbit-plane-target-slot.txt` | invariant | The Target orbit slot uses the target's conic about the focus. |
| `render/199-orbit-plane-target-none.txt` | invariant | With no target the Target orbit slot falls back to the ecliptic and says so. |
| `render/200-orbit-plane-target-ship.txt` | invariant | A vessel target's plane comes from its state through its frame. |
| `render/201-orbit-plane-target-grounded.txt` | invariant | A grounded target has no plane and falls back to the ecliptic. |
| `render/202-target-no-compute.txt` | invariant | Compute transfer off silences the solve but keeps the target's plane. |
| `render/203-map-node-markers.txt` | invariant | The map marks AN/DN per slot where the drawn orbit crosses the shown plane. |
| `render/204-markers-in-orbit-plane.txt` | invariant | Flight markers lie in the plane the overlay draws, including in the surface frame. |
| `render/205-plane-overlay-while-flying.txt` | invariant | A flying suborbital ship still gets the plane overlay. |
| `render/206-orbit-plane-keybind.txt` | invariant | F3 flips the same flag as the Settings checkbox. |
| `render/313-postfx-gamma.txt` | invariant | Gamma post-fx produces the expected output. |
| `render/350-map-fit-earth.txt` | invariant | The orbital map's initial zoom is the ship's apoapsis at 100 px. |
| `render/351-map-fit-small-body.txt` | invariant | The fit also zooms IN for a body far smaller than Kerbin. |
| `render/352-map-fit-follows-focus.txt` | invariant | A focus change refits the map to the new ship's orbit. |
| `render/353-map-fit-landed-body-floor.txt` | invariant | A landed ship opens framed on the body's disk. |
| `render/354-map-fit-open-conic.txt` | invariant | An escaping ship's map opens framed on its current radius. |
| `render/355-map-fit-switch-same-body.txt` | invariant | Switching ships around the same body refits the map to the new orbit. |

## harness (12)

| case | kind | pins |
|---|---|---|
| `harness/01-smoke.txt` | canary | The binary boots, opens a window and exits on --timeout with no GL or crash line. |
| `harness/35-compound-check.txt` | harness | The compound-check self-test reproduces the staged compound. |
| `harness/136-ui-list-title.txt` | harness | The live imgui registry reports the title menu's labels. |
| `harness/137-ui-click-failures.txt` | harness | A bad --ui-click path fails loudly as not-found, not ambiguous, never as a stray click. |
| `harness/155-idle-headless-never.txt` | harness | The offscreen driver never reports the window occluded. |
| `harness/175-sim-press-digit-keys.txt` | harness | Digit keys sent by --sim-press parse as digits, not raw SDL keycodes. |
| `harness/272-ui-click-disabled-widget.txt` | harness | A click on a disabled widget lands but activates nothing. |
| `harness/274-ui-combo-map-plane.txt` | harness | A Combo is addressable by its label and its popup by nesting depth. |
| `harness/280-ui-focus-refuses-hidden-part.txt` | harness | --ui-focus refuses a part no pixel can reach instead of aiming at a neighbour. |
| `harness/281-ui-focus-after-orbit.txt` | harness | --ui-focus lands after the camera is orbited. |
| `harness/282-ui-focus-unknown-part.txt` | harness | --ui-focus says an unknown part is unknown rather than clicking empty space. |
| `harness/343-screenshot.txt` | harness | F12 writes a PNG under the data dir's screenshots/ and toasts the outcome. |
