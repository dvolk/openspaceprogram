#!/usr/bin/env python3
"""Probe the smoothness of a dumped porkchop grid.

Adjacent-cell jumps far larger than the local dv scale mean the sampled
surface is not resolved (or the solver is switching conic branch between
neighbouring cells), which is what makes a coarse grid's argmin meaningless.
"""
import math
import sys
from analyze import load


def main(path):
    g = load(path)
    grid, n = g["grid"], g["n"]
    j = round((g["tof_min"] - g["tof_lo"]) / (g["tof_hi"] - g["tof_lo"]) * (len(grid) - 1))
    i = round((g["t_dep_min"] - g["t_dep_lo"]) / (g["t_dep_hi"] - g["t_dep_lo"]) * (n - 1))
    step_dep = (g["t_dep_hi"] - g["t_dep_lo"]) / (n - 1)
    print("== %s  (%dx%d, dep step %.4g s)" % (path, n, len(grid), step_dep))
    print("   argmin cell i=%d j=%d dv=%.1f" % (i, j, grid[j][i]))

    row = grid[j]
    print("   along departure at the argmin ToF (+-%d cells):" % 5)
    for k in range(max(0, i - 5), min(n, i + 6)):
        print("      i=%d t_dep=%.6g s dv=%s%s"
              % (k, g["t_dep_lo"] + k * step_dep,
                 "nan" if math.isnan(row[k]) else "%.1f" % row[k],
                 "  <-- argmin" if k == i else ""))

    # Neighbour-to-neighbour jumps along one departure row and one ToF column.
    jumps = [abs(row[k + 1] - row[k]) for k in range(n - 1)
             if not math.isnan(row[k]) and not math.isnan(row[k + 1])]
    col = [grid[m][i] for m in range(len(grid))]
    cj = [abs(col[m + 1] - col[m]) for m in range(len(col) - 1)
          if not math.isnan(col[m]) and not math.isnan(col[m + 1])]
    for label, js in (("along departure", jumps), ("along ToF", cj)):
        if not js:
            continue
        js_sorted = sorted(js)
        print("   %s: median |d2 dv| = %.1f m/s, 95th = %.1f, max = %.1f"
              % (label, js_sorted[len(js) // 2], js_sorted[int(0.95 * len(js))],
                 js_sorted[-1]))

    # How many cells are cheaper than the coarse grid's best?
    for thr in (1.0, 1.02, 1.05, 1.10):
        c = sum(1 for r in grid for x in r if not math.isnan(x)
                and x <= g["dv_min"] * thr)
        print("   cells <= %.0f%% of this grid's best: %d" % (thr * 100, c))


if __name__ == "__main__":
    for p in sys.argv[1:]:
        main(p)
