# staging/heightmaps/earth

Earth-like elevation for terrain (`surface.heightmap` in the system JSON).

## Status

Promoted: the game asset is `res/heightmaps/earth/` (committed). Source
GeoTIFFs and working previews stay here and are gitignored
(`staging/**/data/`, `staging/**/preview/`); this note is the tracked
decision record.

## Source

NASA Earth Observatory Blue Marble: Next Generation topography + bathymetry
(GEBCO 2008, British Oceanographic Data Centre), 5400x2700 GeoTIFF (~8 km/px):

- https://assets.science.nasa.gov/content/dam/science/esd/eo/images/bmng/topography/gebco_08_rev_elev_5400x2700.tif
- https://assets.science.nasa.gov/content/dam/science/esd/eo/images/bmng/bathymetry/gebco_08_rev_bath_5400x2700.tif

8-bit greyscale, not a float DEM (verified empirically):

- elev: 0 over ocean, land scaled ~0..6400 m (`m = elev * 6400/255`)
- bath: 255 over land, ocean scaled ~-8000..0 m (`m = (bath - 255) * 8000/255`)

Use the GeoTIFFs, never the JPEG twins (lossy ringing on coastlines).

## Bake

```text
python3 utils/heightmaps/gen_earth_hm.py            # -> res/heightmaps/earth/
python3 utils/heightmaps/gen_earth_hm.py --verify   # pin known elevations (test-py)
python3 utils/heightmaps/gen_earth_hm.py --check    # re-bake vs res/ (needs the TIFFs)
```

Output is one self-describing `earth_hm.i16` (see the script header) plus a
preview PNG for eyeballing. Game convention matches `src/surfmap.h`:
lon 0 at the left edge, north up. The bake rolls the NASA map (lon -180
left) accordingly.

## Composition (src/terragen.h)

Heightmap replaces the *continents* FBM only. Fine noise is an additive
residual (`surface.detail_amplitude` metres). No mountain-fold on top --
GEBCO already has the major ranges. Absent key = today's procedural terrain.

Bathymetry is stored signed. `ocean: "flat"` still clamps the floor at sea
level (current Earth), so the depths are ready when the Mesh ocean shell
works again.
