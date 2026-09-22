# Phase 2 — release artifacts — plan

Date: 2026-09-22
Status: planning (no code written)
Parent: build-tree.md — its "Phase 2 — CI matrix (stub)" anticipated
tag → artifacts; the user has deferred CI ("no remote CI servers for
the moment"), so this phase does the artifact half, locally, on this
box.
Extends: release-infra2026_09_03 — its artifact shape (binary + res/ +
licenses + run notes + SHA256SUMS) is kept; its CI half is out of scope;
its static-build and Windows phases are already done (build-tree
phases 0 + 1).

## TL;DR

A one-command local release flow that produces upload-ready archives —
**per-OS (native format) and a combined one** (user: "both separate
and combined"), sharing one `res/`:

    dist/
      osp-<ver>-linux.tar.xz          # per-OS, native formats
      osp-<ver>-windows.zip           # (the 09_03 shape)
      osp-<ver>-linux+windows.tar.xz  # combined, one file to upload
      SHA256SUMS                      # all three

`make artifacts` is the fast path (build + package, **no e2e** — the
explicit "testing releases" mode the user asked for). `make release`
is the full path: `make test` + `make e2e` under both OSes first,
refusing to package on failure. No CI; the targets are CI-shaped so a
future runner can call the same ones.

## What already exists (verified 2026-09-22)

- Self-contained release binaries for both OSes (build-tree phase 1):
  linux (static SDL3/middleware; the dynamic closure is a standard
  desktop: X11 family, libGL, audio libs — normal for a Linux game),
  windows **fully static** (D5 of phase1-windows.md).
- Versioning: `make version` writes `src/version.h` from
  `git describe --tags --always --dirty --match "[0-9A-Z]*.[0-9A-Z]*"`.
  A `v0.1.0`-style tag already matches the pattern. **No tags exist
  yet** — the current string is `436a4c1-dirty`.
- The game loads everything from `res/` relative to CWD (28 MB:
  meshes, textures, shaders, `parts.json`, `ships/`, audio, fonts);
  per-user data (saves/screenshots/settings) goes to the OS pref path
  or `--data-dir` — never shipped.
- `make test` + `make e2e` (run.py, linux and wine/.exe modes) to gate
  a release.
- No LICENSE file in the repo (README states GPL-3 code /
  CC-BY-SA 3.0 content + Pioneer credit). Distribution needs the
  license text included (GPL-3 §4) → we add one and ship it.
- Packaging tools: `tar`/`xz`/`sha256sum` present; `zip` installable
  (user OK); python3 stdlib `zipfile` as a fallback.

## Decisions

**D1 — Per-OS archives *and* a combined one** (user: "both separate
and combined"). Each per-OS archive is self-contained (binary + its
own `res/` + licenses + run notes); the combined one shares a single
`res/`:

    osp-<ver>-linux/           osp-<ver>-windows/          osp-<ver>-linux+windows/
      osp                        osp.exe                       osp          (linux)
      res/                       res/                          osp.exe      (windows)
      LICENSE.md                 LICENSE.md                    res/         (shared, 28 MB)
      README.md                  README.md                     LICENSE.md
                                                               README.md

The combined archive puts both binaries at the root: the names don't
collide (`osp` / `osp.exe`), and the game loads `res/` relative to the
CWD — so "run the binary from the archive root" is the only correct
invocation, and there is no `linux/` subdir to `cd` into and get a
missing-res/ failure (user note, 2026-09-22).

- Per-OS formats: `linux.tar.xz`, `windows.zip` (the 09_03 shape —
  zip is the native double-click format on Windows; tar.xz is the
  native one on Linux).
- Combined format: `tar.xz` (extraction is universal: Linux/macOS
  `tar` built in, Windows 10 1803+/11 `tar.exe` built in).
- Sizes: per-OS ≈ 28 MB res/ + 13 MB binary each; combined ≈ 28 MB
  res/ + 26 MB binaries (res/ is mostly already-compressed media, so
  each lands ~40–45 MB).
- Naming: `osp-<ver>-<os>.<ext>`. `<ver>` is exactly the string the
  build embeds (D2). `SHA256SUMS` lists all three archives.

**D2 — Version = the embedded version string.** `make artifacts`
reuses the version target's logic so the archive name always matches
what the game reports (main menu footer). Real releases: tag first
(`vX.Y.Z` matches the existing describe pattern) → clean name. Dirty
tree: allowed but the name carries `-dirty` (honest) and the target
warns. First tag: open question (below).

**D3 — e2e is opt-in, and the default is OFF for packaging.**
User requirement: "make sure there's an option to skip e2e at least
for testing releases since it takes a while."
- `make artifacts` — build both OS releases (incremental if current) +
  stage + archive + SHA256SUMS. No e2e, no unit tests. The testing
  path.
- `make release` — `make test` (linux) → `make e2e` (linux binary) →
  `make e2e` (windows .exe under wine, auto-serial) → `artifacts`.
  Any failure stops the flow with no partial artifact in `dist/`.
  The real-release path.

**D4 — Artifacts land in `dist/`** (the 09_03 plan's directory name),
gitignored, never committed; the user uploads manually. Staging
happens in `tmp/` (scratch, per project convention) and is removed
after a successful package.

**D5 — Add `LICENSE.md`** (committed to the repo, shipped in the
archive): the full GPL-3.0 text (code) + CC-BY-SA 3.0 text (content) +
the Pioneer attribution already in the README. A "how to run"
`README.md` (~30 lines) also ships: per-OS run instructions,
requirements (x86-64-v2 CPU, OpenGL 4.5, ~50 MB), where saves land,
`--help`, license summary, and a note that Windows extraction needs
Win10 1803+ or 7-Zip.

## Steps

| Step | Work | Acceptance |
|---|---|---|
| 2.1 | `LICENSE.md` + the archive `README.md` text (content only) | both files read correctly; licenses complete |
| 2.2 | `artifacts` target: compute version string (reuse the version target's logic, warn on `-dirty`), build linux + windows release, stage the D1 layouts under `tmp/`, package all three archives into `dist/` (zip via python3 `zipfile`/`zip`, tar.xz via tar), `SHA256SUMS`, clean the staging dir | `make artifacts` on a clean tree → the three `dist/osp-<ver>-*` archives + `SHA256SUMS`; no e2e runs (wall-time check) |
| 2.3 | `release` target: `make test` → `make e2e` (linux) → `make e2e` (`.exe`) → `artifacts`, fail-fast, atomic (artifact appears only if all gates pass) | a forced e2e failure blocks the artifact; a green run produces the same `dist/` as 2.2 |
| 2.4 | Fresh-extract verification: extract each archive into a clean dir outside the repo, run `osp --timeout 5` (Xvfb, from the linux and combined layouts) and `osp.exe --timeout 5` (wine, from the windows and combined layouts), confirm both boot to the main loop with res/ loaded and exit cleanly; verify every checksum | both binaries run from every extraction layout without the repo present |
| 2.5 (optional QoL) | `--version` flag printing the embedded VERSION (cli.h already mentions it in a comment; the flag doesn't exist) | `osp --version` / `osp.exe --version` print the version string and exit 0 |

## Out of scope

- macOS — needs Xcode; not buildable on this box. Later phase (the
  archive layout has room: add a `macos/` dir).
- CI — deferred per user; `release`/`artifacts` are CI-shaped
  (idempotent, no interactivity) so a future runner reuses them.
- Steam, exe icon/manifest, resolving `res/` relative to the
  executable (09_03 parking lot) — the shipped README covers "run
  from the extracted directory".

## Risks / gotchas

- `dist/` holds ~3 × 40–45 MB per release (per-OS ×2 + combined).
  Fine for manual upload; if the eventual target has size limits, the
  per-OS archives already cover it. Old releases accumulate in
  `dist/` — expected for a manual flow (D4).
- The linux binary is *desktop-static* (X11/libGL/audio dynamic) —
  fine on any normal desktop, but it is the one non-self-contained
  piece (the .exe is fully static). Worth one line in the shipped
  README.
- `git describe` needs `fetch-depth: 0`-equivalent history — fine
  locally (full clone), only a gotcha if a future CI uses shallow
  checkouts (note for that phase).
- Stale `dist/`: the target overwrites per version; different version
  = different filename, so old archives accumulate — expected for a
  manual-upload flow; `make clean-dist` if we want it (skip for now).

## Decisions with the user (2026-09-22)

1. First tag: **`v0.0.1`** ("not really ready for user testing").
2. **`--version` included** (step 2.5).
3. Combined-archive layout: **both binaries at the root** (no
   `linux/`/`windows/` subdirs — see D1 note).
