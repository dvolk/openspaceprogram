# Open Space Program — how to run

An open source space sim (Kerbal-style): build a rocket, launch, orbit,
land. Very early development — expect rough edges.

## Run it

The game finds its assets in the `res/` directory next to the binary, so
you can start it from anywhere:

- **Linux** — from a terminal:

      ./osp-<version>-linux/osp

  (or add that directory to your `PATH` and run `osp`)

- **Windows** — double-click `osp.exe`, or from a terminal:

      osp-<version>-windows\osp.exe

A combined archive (the `linux+windows` one) contains both binaries in
the same directory — use the one for your OS; the other is inert.

## Requirements

- CPU: x86-64 with the x86-64-v2 instruction set (~2010+; SSE4.2)
- Graphics: OpenGL 4.5 (a hardware GPU; it will boot on software
  rendering like Mesa llvmpipe, but slowly)
- The Linux build links the standard desktop libraries (X11, GL,
  audio); the Windows build is fully self-contained

## Basics

- `T` thrust, `Space` stage, `W/S` pitch, `A/D` yaw, `Q/E` roll,
  `R/F` throttle up/down
- `F5` pause, `F12` screenshot (lands in the data directory)
- `--help` lists every command-line option (starting ships, bodies,
  scenarios); `--version` prints the build version

## Where your data goes

Saves, screenshots, and settings live in your per-user data
directory — `~/.local/share/openspaceprogram/` on Linux,
`%APPDATA%\openspaceprogram\` on Windows (override with `--data-dir`).
Nothing in this directory is ever modified by the game.

## Licence

Code: AGPL-3.0; content: CC BY-SA 3.0 (see LICENSE.md).
