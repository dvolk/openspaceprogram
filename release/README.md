# OPEN SPACE PROGRAM

A sandbox space sim: build a rocket, launch, visit different
planets, run science experiments, return home to finish the
flight and recover the ship and experiments.

Very early development — expect rough edges.

## GIVING FEEDBACK

If you have feedback please open an issue on github:

https://github.com/dvolk/openspaceprogram

## (SOME) DEFAULT CONTROLS

- W/S - pitch
- A/D - yaw
- Q/E - roll
- T - thrust
- Shift-T - lock/unlock thrust
- Space - stage
- R/F - throttle up/down
- Z - autopilot: hold the nose along your velocity
- B - autopilot: hold the nose against your velocity
- F5 - quicksave
- F9 - quickload
- F12 - screenshot

### Mouse

- Right-drag - orbit the camera around what it is looking at
- Wheel - zoom
- Right-click a part - open that part's window (board a capsule,
  run an experiment, store or pick up cargo)

See Controls for all bindings

## IF IT WILL NOT START

osp.exe runs without a console window, so a startup failure (missing
res/, no OpenGL) can look like the window flashing and vanishing.
Start it from cmd or PowerShell to see what it says. To script a run
in cmd, use `start /wait "" osp.exe ...` — cmd does not wait for it
otherwise.

## WHERE YOUR DATA GOES

Saves, screenshots, user ships and settings are (usually) in:

- Linux: ~/.local/share/openspaceprogram/
- Windows: %APPDATA%\openspaceprogram\
