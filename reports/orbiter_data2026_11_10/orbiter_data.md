# What is in `staging/orbiter-data`, and what using it in OSP would cost

Snapshot of an investigation on 2026-11-10. Nothing here is integrated; nothing here is a plan I
started. Supporting code: `tree.py` (container reader), `treedec.py` (BC1 tile decoder).

## 1. What the download actually is

58 GB of zip, from orbiter-mods.com: the **Orbiter "high-resolution global surface" sets from the
2016 beta era** plus the community **MinorBodies** texture set. Every zip carries only two kinds of
file: a `.tree` tile archive under `Textures/<Body>/Archive/`, and (for two bodies only) an Orbiter
body config under `Config/`.

| zip | payload (uncompressed) | what it holds |
|---|---|---|
| `earth-hires-2016-surf.zip` | `Surf.tree` 15.44 GB | Earth surface colour |
| `earth-hires-2016-elev.zip` | `Elev.tree` 17.24 GB, `Elev_mod.tree` 0.52 MB | Earth relief (plus a small override archive) |
| `earth-hires-2016-mask.zip` | `Mask.tree` 2.27 GB | Earth land/sea (specular) mask |
| `earth-hires-2016-cloud.zip` | `Cloud.tree` 0.31 GB | Earth cloud deck |
| `moon-hires-2016-surf.zip` | `Surf.tree` 3.75 GB, `Label.tree` | Moon colour + place-name labels |
| `moon-hires-2016-elev.zip` | `Elev.tree` 11.78 GB | Moon relief |
| `mars-hires-2016-surf.zip` / `-elev.zip` | 4.04 GB / 2.63 GB | Mars colour + relief |
| `Mercury-hires-2016.zip` | Surf 3.74 GB, Elev 0.27 GB, Label, `Mercury.cfg` | Mercury |
| `Titan-hires-2016.zip` | Surf 0.165 GB, Elev 2.9 MB, Cloud, Label, `Titan.cfg` | Titan |
| `MinorBodies.zip` | 0.27 GB | Surf for **Io, Europa, Enceladus, Phobos, Vesta**; Elev for Io, Phobos, Vesta |

**There is no licence, README or attribution file inside any of the eleven zips.** Provenance can
only come from the pages they were downloaded from.

## 2. The container format (cracked, and one trap in it)

A `.tree` is a tile archive:

```
offset 0   header, 48 B: 'T' 'X', u16 version (=1), u32 48, u32 1,
                          u32 index_bytes, u32 data_bytes, u32 0, u32 nrec, u32 0,1,2,3
offset 48  nrec * 32 B node records, depth-first:
              u64  tile offset   (from the END of the index)
              u32  tile size     (UNCOMPRESSED -- see the trap)
              s32  child[4]      (-1 = leaf)
              u32  key           (0 for terrain tiles; a name-hash for Label tiles)
then       one raw zlib stream per node, in that order
```

So it is a **sparse quadtree forest**: each node is one tile, `child[i]` are its four sub-tiles.
Earth's first archive: 155,352 nodes, 5 roots, `depth 0..15`, tile size 128x128 at depth 0,
256x256 at depth 1, and **512x512 for every node below that**. Nodes are independent zlib streams
at known offsets, so random access is cheap (I read single tiles out of a 15 GB member by streaming
only as far as the offset needed: 2.55 GB skipped in 5.6 s).

**The trap:** `data_bytes` is a `u32` and *overflows* on Earth. `index_bytes + data_bytes` is
exactly `filesize mod 2^32` (2,553,357,977 vs a real 15,438,259,865). A reader that trusts the
header's size, or that assumes one archive per file, reads 2.5 GB of a 15.4 GB archive and does not
notice. Use `max(node.offset + node.size) + index_bytes`, or the file size.

Tile payloads (confirmed by reading the bytes, not by guessing):

* image tiles: a 128-byte header — magic `'BAP '`, `u32 header_length (=124)`, `u32 csum`,
  `u32 w`, `u32 h`, `u32 raster_bytes`, ..., `u32 format (=4)`, fourcc `'DXT1'` — then the raster.
  Surf rasters are **0.5 B/px = BC1/DXT1**; Cloud rasters are **1.00 B/px**, i.e. the DXT3/DXT5
  family (not pinned down).
* elevation tiles use a *different* 100-byte header (magic `'EJE\x01'`) and are **259 x 259 of
  int16** (256 + a 3-sample halo). Unresolved: one Phobos tile's body is 14 samples short of
  259x259, and 86% of it was zero, so the quantisation/packing (and the scale-offset fields, which
  read as 1.875 and -16 in that header) still need work.

Resolution ladder: each tree level halves metres-per-pixel. Reading the roots as cube faces,
Earth's bulk (depth 9, 95,050 tiles) is about **35 m/px**, with a few thousand patches diving to
depth 12-15 (1-4 m/px); `MinorBodies` tops out nearer **~1 km/px** on Io. Earth's archive holds
155k tiles whose uncompressed rasters total 20.4 GB against 15.4 GB on disk (zlib ~76%).

## 3. How far the pixel decode got, honestly

* The BC1 path is verified: a hand-built block round-trips correctly through the decoder in
  `treedec.py`, and Earth tile block-adjacency colour correlation is 0.81 in *both* axes, with
  endpoints close (mean |c0-c1| 8%). Colours decode plausibly (blue ocean blocks `(0,49,255)`,
  white polar rows top and bottom of the coarse global tiles, green land).
* **But tiles render streaky.** Measuring neighbour differences inside a 4x4 block:
  horizontal mean |delta| 8.3/255 vs **vertical 37/255**. The in-block 2-bit code correlation matrix
  follows the standard row-major order (so the index mapping is the normal one), and every
  alternative block order I tried is *worse*: macroblock 2x2/4x4/8x8/16x16 (row- and column-major
  inside), Morton/Z-order, transposed block grid, block-grid strides 2..200. Plain row-major wins
  every time, and still leaves the vertical incoherence. So either there is a swizzle I have not
  found, or these particular 2016-era mosaics have banding baked in. This is the one open question
  that decides whether the pixels are directly usable, and it needs the Orbiter source (or the
  thread where these packs were made) rather than more guessing.

## 4. Where it would land in OSP, and what would have to be built

Shape-wise the fit is good: our terrain is already a subdividing quadtree of spherical patches
(`GeoPatch::kids[4]`, `src/terrain.h`) over an equirect (lon,lat) convention (`src/equirect.h`),
which is the same tree shape as `.tree`. What we do *not* have is any streaming:

* Terrain today bakes **vertex colour** from `terrainSurfaceColor()` in `terragen.h`; there is no
  terrain texture sampler in `terrainShader.fs`, and `src/texture.cpp` only ever does whole-image
  `glTexImage2D` — no `glCompressedTexImage2D`, no tile cache, no eviction, no async inflate.
* Our one real heightmap, `res/heightmaps/earth/earth_hm.i16`, is a 2700x1350 HM16 equirect
  (GEBCO 2008 via Blue Marble) — around **2 km/px**, i.e. ~60x coarser than the Orbiter Earth
  elevation, and loaded whole (`Heightmap` in `terragen.h`).

So the cost splits cleanly:

1. **Offline unpacker** (Python, `utils/`): read `.tree`, inflate, decode BC1, and re-bake into our
   own layout. Two options: (a) keep their quadtree addressing and feed tiles straight to patches,
   or (b) resample into a handful of our-equirect levels (far simpler, throws away the deepest
   dives). Plus the elevation path, whose packing is still unpinned. *Order of days*, and most of
   that is writing the format note above down as code and testing it.
2. **Runtime streaming** (the expensive part): a per-patch tile cache with an LRU and a memory
   budget, async inflate on the existing job system, a sampler + level selection in
   `terrainShader.fs`, `glCompressedTexImage2D` for BC1 (128 KB per 512x512 tile on disk *and* on
   the GPU), and an eviction policy that survives orbit. *Order of weeks*, and it is the same work
   we would need for any streamed imagery, Orbiter or our own.
3. **Optional-download plumbing**: a data root outside `res/` (`datadir.h` already resolves a data
   dir), a manifest listing body -> archive -> available depth with hashes, a loader that degrades
   to the procedural terrain when the pack is absent, and — the part the project cares about —
   the system file has to *record* which bodies have an external image set and down to which level,
   otherwise the dossier and the pixels are two authorities for the same ground (#687).
4. **Size**: shipping all of it is silly. Capping the depth (say <= 8, ~70 m/px) and taking
   surface+elevation for a few bodies is the order of **1-3 GB**, not 58. The 58 GB figure is what
   the last two levels of Earth cost.

## 5. The licence question, which is the real blocker

Nothing in the payload states a licence. Independent of the Orbiter-side terms (the app is free but
its ecosystem's add-ons very often say "for use with Orbiter", non-commercial, or no-redistribution),
the *rasters themselves* are the problem: Earth imagery is NASA/GSFC-derived (public domain), the
elevations look like SRTM/GMTED-family DEMs (historically redistribution-restricted), and community
"super resolution" patches in these sets were frequently hand-assembled from commercial imagery
providers whose terms forbid redistribution. Without the source pages there is no way to know which
body's tiles carry which terms.

Recommendation: keep it in `staging/` as private play-test data, do **not** move anything into
`res/`, and if we ever want it as an optional download, record per-body provenance from the download
pages in the manifest before writing a single line of loader.

## 6. A small thing worth taking that has nothing to do with the gigabytes

`Config/Mercury.cfg` and `Config/Titan.cfg` are a third *authored* corpus, and they carry fields we
author by hand and would like cross-checked:

| field | Orbiter | ours (`solar_system.json`) |
|---|---|---|
| Titan radius | 2.575e6 | 2,575,000 |
| Titan sea-level pressure | 146e3 Pa | 143,555.8 Pa |
| Titan gas constant / gamma | 285.92 / 1.3941 | (no `gamma` authored) |
| Titan atmosphere limit / cloud layer | 600 km / 30 km | (no such rows) |
| Mercury radius | 2.440e6 | 2,439,700 |
| Mercury J2 | 0.00006 | (not authored) |
| rotation: obliquity, LAN, sidereal period | all stated for both | we author spin only |

Titan's `AtmGamma = 1.3941` is a gamma for a world where we currently have none, from a source with
a stated derivation. Two config files is not a corpus, but if more `.cfg` files exist in the wider
set (they ship per body in Orbiter), that is a legitimate authored-atmosphere corpus for #625-style
work, independent of any texture licensing.
