# Skybox staging

Candidate skybox faces and the source data they are baked from, kept out of
git (`staging/*/bakes/`, `staging/*/data/`, `staging/*/preview/`) until the
look is settled. Promotion: copy the six faces into `res/skybox/<set>/`
alongside `skybox_ATTRIBUTION.txt` and `skybox_params.json`, commit them,
and name the DIRECTORY in the systems that should use it --
`"skybox": "res/skybox/<set>"`. The loader builds the six
`skybox_{px,nx,py,ny,pz,nz}.png` names itself (src/system.cpp), so a system
can also point straight at a bake here to judge it in game.

## Layout

    bakes/     face sets awaiting a decision, one directory per candidate
    data/      inputs: bsc5.dat, the Deep Star Maps EXRs and their ffmpeg
               .raw decode caches (~1.6 GB, re-downloadable)
    preview/   montages and in-game screenshots of each candidate
    NOTES.md   this file -- tracked, unlike everything above

Tools live in `utils/skybox/`: `skybox_common.py` (cubemap face math, the
rail-frame sky embedding, the alignment pins), `make_skybox.py` (Deep Star
Maps bake), `make_catalog_skybox.py` (BSC5 catalog bake).

## Candidates

Luminance over all six faces, 0-255 scale. For reference, the placeholder
`v1` (`res/skybox/v1/`, six copies of the old `res/textures/skybox.png`)
measures mean 2.72 / p99 85.0 / p99.9 255.0 / max 254, with 0.615 % of
pixels above 128 -- that saturation is what the runtime `--sky-dim` fade was
added to paper over. `old_system.json` and `ksp_system.json` still load it.

| directory | how | mean | p99 | p99.9 | max | >128 |
|---|---|---|---|---|---|---|
| **`dsm_1024_g3_ds2`** | `--size 1024 --gain 3.0 --ds 2` -- **promoted** to `res/skybox/dsm_1024_g3_ds2`, used by the four `solar_system*.json` | 7.53 | 51.5 | 113.1 | 242 | 0.070 % |
| `dsm` | `--size 1024 --gain 1.0 --ds 1` (Deep Star Maps) | 2.27 | 17.8 | 39.2 | 159 | 0.001 % |
| `dsm_2048_g2_ds4` | `--gain 2.0` (defaults: 2048, ds 4) | 4.98 | 34.8 | 69.3 | 217 | 0.006 % |
| `dsm_2048_g2_ds2` | `--gain 2.0 --ds 2` | 4.95 | - | 82.4 | 220 | - |
| `catalog` | `--bake` (BSC5 stars only) | 0.01 | 0.0 | 1.0 | 172 | 0.000 % |
| `catalog_2048` | an earlier catalog bake (2048, predates `make_catalog_skybox.py`; params not recorded) | 9.79 | 91.7 | 127.0 | 239 | 0.087 % |
| `catalog_hybrid_g1` | `--bake --band milkyway_2020_8k --gain 1.0` | 1.28 | 11.4 | 22.4 | 173 | 0.000 % |
| `catalog_hybrid_g2p5` | same, `--gain 2.5` | 4.15 | 28.6 | 52.9 | 233 | 0.001 % |
| `catalog_hybrid_g5` | same, `--gain 5.0` | 8.07 | 53.8 | 94.4 | 253 | 0.022 % |

The promoted set is 1024^2 on purpose: a 2048^2 bake at the same knobs is
visibly equivalent by these numbers (mean 7.53 / p99 51.5 / max 242) but
costs 23.6 MB in git instead of 7.0 MB, and ~76 MB of VRAM instead of ~19.
Re-baking it is `python3 utils/skybox/make_skybox.py --size 1024 --gain 3.0
--ds 2 --out res/skybox/dsm_1024_g3_ds2`, then `--check` against that path
needs the source EXRs.

At gain 3.0 the six pin stars all peak 235..242, so the tone map is close to
clipping them: the star field is bright by authorship, and there is little
headroom left if the renderer ever goes HDR.

Catalog bakes use `--bright 200 --sigma 1.2`: the brightest star in the sky
(Sirius, V -1.46) reads 200 and every other star sits where its V magnitude
puts it. That scale is the point of the catalog route -- an image bake
inherits NASA's exposure, this one states its own.

`tmp/newskybox_gain1.0/` is a byte-identical duplicate of `bakes/dsm`
(same md5 on `skybox_px.png`); left where it is, delete it if you want.

## What is settled

- **Promoted.** `res/skybox/dsm_1024_g3_ds2` is the sky for the four
  `solar_system*.json`; `res/skybox/v1` (six copies of the old placeholder)
  stays on `old_system.json` and `ksp_system.json`. A system JSON names a
  DIRECTORY, so face identity now lives in the file names -- which is why
  `test-py` runs the alignment pins against the COMMITTED faces: a mirrored
  or swapped set fails there rather than showing up as a mystery sky in
  game. (The old axis-keyed object form put identity in the JSON keys
  instead; `tests/test_skybox.cpp` pins that the object and array forms are
  both rejected now.)
- **Alignment.** The catalog's coordinates and the rail embedding agree with
  the NASA-derived faces to within a texel: reading the 25 brightest BSC5
  stars out of an image bake gives median offset 0.00 texel, median peak
  125.5, faintest 93.2 (`make_catalog_skybox.py --cross`). This is the only
  non-circular alignment check available -- a catalog bake pinned with the
  same transform it was placed with proves nothing.
- **`--ds` cost at 2048** (worst over the six pin stars): star rms width
  3.16 / 3.32 / 3.84 texels at ds 1 / 2 / 4. ds 4 is 20 % blurrier and is
  what pushed the old argmax pin past its threshold; ds 2 costs 5 % over
  ds 1.
- **The image bake's stars are ~3.2 texels rms wide** -- the source map's
  own PSF. Catalog stars at `--sigma 1.2` are ~2.6x crisper.

## Open

- A bare catalog sky is too empty (mean 0.01): BSC5 stops at V=6.5, the
  naked-eye sky, with no Milky Way and no unresolved haze. The hybrid keeps
  NASA's band; a statistical faint-star field is the alternative.
- `--sigma` is a filtering choice, not a physical size (seeing ~1 arcmin vs
  5.3 arcmin/texel at 1024). Too small and cube filtering makes stars
  flicker as the sky turns. Needs judging in motion.
- 275 catalog rows have no B-V and currently render white.
- `bakes/dsm` (gain 1.0) still fails the Betelgeuse pin, reading 117.8
  against the pins' floor of 120 -- issue #176. That bake is not committed,
  so `test-py` no longer runs against it; the promoted set reads 235 there.
