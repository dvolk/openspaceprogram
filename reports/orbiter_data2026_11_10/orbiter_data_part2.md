# Orbiter data, part 2: the format is pinned, my part-1 findings had three bugs in them

Same investigation as `orbiter_data.md`, same day, after the Orbiter source turned up in
`../references/orbiter`. **This file supersedes specific claims in that first report.** The first
report is left as written; the tool it now lacks is `otree.py` (also copied to
`staging/orbiter-data/dev/otree.py`, since that is where the data lives).

## Three things part 1 got wrong, and how I know

1. **"the tile magic is `BAP `".** False. It is `'DDS '`. I converted 542,327,876 to hex by hand
   and read `0x20504142`; it is `0x20534444`. Python says so; my arithmetic did not.
2. **"`data_bytes` is a u32 that overflows on Earth -- a silent trap".** False, and it is the kind
   of finding that is extra convincing because it looked like a bug in someone else's writer. The
   field is `dataLength`, a **signed 64-bit at header offset 16**; I read the u32 at offset 12
   (`dataOfs`) and the one at 16 and joined them wrong. Re-measured on three archives:
   `dataOfs + dataLength == member size` exactly (Earth 4,971,312 + 15.433 GB = 15,438,259,865 ✓,
   Moon Surf ✓, Moon Elev ✓). No trap. `otree.py info` prints the check so nobody re-derives it.
3. **"the pixels decode to plausible colours but render streaky; either a swizzle remains or the
   imagery is banded -- needs the Orbiter source".** The source answered the swizzle question in
   about a minute: tiles are ordinary DXT1/DXT3/DXT5, standard block layout, raster at +128. The
   streaky-magenta look was **my decoder**: BC1 endpoints are RGB**565** = 5/**6**/5 bits, and I
   masked green with `&31`. With green at 6 bits the Moon's global mean is exactly neutral
   `(140.1, 140.1, 140.1)` and mare/highland patterns are visible; Titan reads tan
   `(128, 107, 80)`; Earth's sea blocks are blue. The bug is in part 1's `treedec.py` (`&31` on the
   green field) and in the block-layout sweeps that used it -- those measurements are worthless.

So: **the container and the codec are both fully pinned, with nothing open.** The residual
complaint about the imagery is real but smaller, and it is measured below.

## The format, in full, with the source to point at

```
header 48 B   'TX' ver 0x01 00 | u32 hdrsize(48) | u32 flags(1 = deflated) | u32 dataOfs
              | s64 dataLength | u32 ntoc | u32 rootPos1, rootPos2, rootPos3, rootPos4[2]
TOC    ntoc x 32 B { s64 pos (from dataOfs), u32 inflated size, s32 child[4] (-1 = leaf) }
data   one zlib stream per node -- pos/size are enough, the deflated length is not stored,
       so inflate from `pos` with a generous window and stop at the stream end
```
Source: `Utils/texpack/texpack.cpp` (`TOCEntry`, `TreeTOC::Header`), `Src/Orbiter/ZTreeMgr.cpp`
(`TreeFileHeader`, `ReadData` -> `uncompress`).

* Header roots name **five** trees: one whole-globe tile at level 1 (128 px), one at level 2
  (256), one at level 3 (512), and **two level-4 tiles** as the quadtree roots. That is exactly the
  5 unreachable-by-child-links roots I had measured in part 1 and could not explain.
* Addressing (`Utils/tileedit/qt/src/tileblock.h`): **equirectangular, 2:1** -- at level L >= 4 the
  globe is `nLng = 2^(L-3)` columns by `nLat = 2^(L-4)` rows of 512 px tiles, and
  `child index = (ilat&1)*2 + (ilng&1)`. Metres/pixel therefore halves per level (Earth):
  level 9 = 1222 m/px, 11 = 305, 12 = 153, **13 = 76**, 14 = 38, 15 = 19, 16 = 9.5, 17 = 4.8,
  18 = 2.4, 19 = 1.2. Moon Surf tops out at level 11 = 305 m/px.
* Colour tile payload = `'DDS '` + 124-byte DDS header (`dwSize=124`, `dwFlags=0x81007`, height,
  width, linearsize) + raster at **+128**. The fourCC sits 4 bytes earlier than a stock `DDS_HEADER`
  claims, so `otree.py` *searches* the header for `DXT1/3/5` rather than trusting the offset.
  Surf/Mask/Cloud are DXT1 (0.5 B/px) or DXT3/5 (1 B/px); Label tiles are a third thing (`lab`).
* Elevation tile payload = `ELEVFILEHEADER`, 100 B: `'ELE\1'`, hdrsize, `dtype` (-16 = int16,
  0 = flat), `xgrd,ygrd` (259,259), `xpad,ypad` (1,1), then nine doubles:
  `scale, offset, latmin, latmax, lngmin, lngmax, emin, emax, emean`;
  **elevation = raw*scale + offset [m]**, and every tile states **its own lat/lng box in radians**.
  Moon level-1 tile: lat [-90,90] x lng [-180,0], emin **-8125 m**, emax **+9214 m** -- real relief,
  and the box-in-the-header is what makes an offline re-bake easy: you do not have to reproduce
  their addressing at all, you just splat each tile where its own header says it is.
  Source: `Src/Orbiter/elevmgr.cpp` (and `Utils/tileedit/qt/src/elv_io.cpp` for the standalone
  `.elv` variant, whose header is 76 B).

## What the imagery is actually like (measured, with the fixed decoder)

Moon Surf, a level-11 leaf: mean `|dx|` between horizontal neighbours **3.89** vs mean `|dy|`
between vertical neighbours **57.93** -- a **15x** one-scanline stripe. Titan level 3: 5.56 vs
34.10 (6x). So the tiles are geometrically right and the packs have **strong horizontal banding
baked in** (row-mean std is ~20% of total std). Practical consequence: colour from these packs
wants a cheap de-band (2-3 tap vertical blur) during any offline re-bake, or it will show at close
range. Elevation has no such problem: it is a plain int16 grid.

## Licence: the source changes the shape of the question, not the answer

`../references/orbiter` is **MIT** (`LICENSE`, "Copyright (c) 2000-2026 Martin Schweiger", and the
per-file headers on `ZTreeMgr.cpp`/`elevmgr.cpp`). That means the *reader* is free: we can vendor
those structs or write our own with no obligation. It says nothing about the 58 GB of rasters,
which carry no licence anywhere in the zips, and those are the thing we would be copying.

On "if the user downloads it as an optional addition they take responsibility anyway i guess" --
that covers *them obtaining it*, which is fine and is the common way this is done. It does not cover
*us distributing it*: the moment the pack is in an installer, our server, or a repacked subset we
built, the redistribution right has to exist, and an EULA click-through doesn't create one. The
shape that is actually clean is: **we ship nothing** -- OSP reads a directory the player fetched
themselves (there is already a data-dir resolver in `datadir.h`), with the source URL and our
uncertainty shown in the UI. Then the licensing is theirs, the 58 GB is not ours, and the only thing
we owe is honest labelling of what we know about the provenance. Repacking *from* it into our own
assets (the tempting thing) is exactly the step that needs the right, so if we ever want that in
`res/`, it waits for a per-body answer on where each raster came from.

## Tickets

Part 1's issue #753 stands but its body needs correcting (done, in a comment on it). The cheap,
non-blocking piece -- elevation only, no streaming, no licence exposure beyond "player-supplied
directory" -- is filed separately and points here.
