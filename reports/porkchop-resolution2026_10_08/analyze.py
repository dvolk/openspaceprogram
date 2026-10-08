#!/usr/bin/env python3
"""Compare porkchop grids of different sizes against the finest one.

Reads the CSVs written by `osp --porkchop-dump` (one file per grid size, the
header comment carrying the axis ranges and the argmin) and reports, for every
coarser grid:

  dv_err      dv_min(n) - dv_min(ref)          how much better the plan gets
  t_dep_err   t_dep_min(n) - t_dep_min(ref)    how different the departure is
  regret      the REFERENCE grid's dv at the departure/ToF the coarse grid
              picked, minus the reference best -- what you would actually pay
              if you flew the plan the coarse plot handed you
  same_basin  whether the coarse argmin sits in the reference argmin's basin

plus, from the reference grid alone, how much of the (t_dep, ToF) domain is
near-optimal and how many separate minima the departure profile has -- i.e. how
many transfers a coarse grid can miss.
"""
import bisect
import math
import os
import sys


def load(path):
    header = None
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                header = line[1:].strip()
                continue
            line = line.strip()
            if not line:
                continue
            rows.append([float(x) for x in line.split(",")])
    if header is None:
        raise SystemExit("%s: no header" % path)
    v = [float(x) for x in header.split(",")]
    # The header gained n_dep,n_tof up front; accept both shapes so the
    # dumps taken before that change still read.
    if len(v) == 9:
        n_hdr, n_tof_hdr, t_dep_lo, t_dep_hi, tof_lo, tof_hi, dv_min, \
            t_dep_min, tof_min = v
    else:
        t_dep_lo, t_dep_hi, tof_lo, tof_hi, dv_min, t_dep_min, tof_min = v
    return {
        "n": len(rows[0]), "t_dep_lo": t_dep_lo, "t_dep_hi": t_dep_hi,
        "tof_lo": tof_lo, "tof_hi": tof_hi, "dv_min": dv_min,
        "t_dep_min": t_dep_min, "tof_min": tof_min, "grid": rows,
    }


def near_best(g, t_dep, tof, radius_cells):
    """Best dv the reference grid offers within `radius_cells` reference cells
    of a point. The nearest-cell value alone is misleading: the departure axis
    is sawtooth, so one cell over can be 5x worse."""
    n = g["n"]
    i = round((t_dep - g["t_dep_lo"]) / (g["t_dep_hi"] - g["t_dep_lo"]) * (n - 1))
    j = round((tof - g["tof_lo"]) / (g["tof_hi"] - g["tof_lo"]) * (len(g["grid"]) - 1))
    best = float("inf")
    for jj in range(max(0, j - radius_cells), min(len(g["grid"]), j + radius_cells + 1)):
        for ii in range(max(0, i - radius_cells), min(n, i + radius_cells + 1)):
            x = g["grid"][jj][ii]
            if not math.isnan(x) and x < best:
                best = x
    return best


def basins(g):
    """Local minima of the min-over-ToF departure profile, with prominence."""
    prof = [min((row[i] for row in g["grid"] if not math.isnan(row[i])),
                default=float("inf"))
            for i in range(g["n"])]
    lo = min(prof)
    peaks, mins = [], []
    for i in range(1, len(prof) - 1):
        if prof[i] <= prof[i - 1] and prof[i] < prof[i + 1]:
            mins.append(i)
        if prof[i] >= prof[i - 1] and prof[i] > prof[i + 1]:
            peaks.append(prof[i])
    # A minimum counts if some neighbouring peak is at least 2% above it.
    strong = [i for i in mins
              if any(p > prof[i] * 1.02 for p in peaks)]
    return prof, lo, strong


def main(d):
    files = sorted(os.listdir(d))
    grids = []
    for f in files:
        if f.endswith(".csv"):
            grids.append(load(os.path.join(d, f)))
    if not grids:
        raise SystemExit("%s: no CSVs" % d)
    grids.sort(key=lambda g: g["n"])
    ref = grids[-1]
    prof, ref_lo, strong = basins(ref)
    allv = [x for row in ref["grid"] for x in row if not math.isnan(x)]
    valid = len(allv)
    near = sum(1 for x in allv if x <= ref["dv_min"] * 1.05)
    sorted_v = sorted(allv)
    step_dep = (ref["t_dep_hi"] - ref["t_dep_lo"]) / max(1, ref["n"] - 1)
    step_tof = (ref["tof_hi"] - ref["tof_lo"]) / max(1, len(ref["grid"]) - 1)

    print("== %s" % d)
    print("   reference %dx%d: dep window %.4g s (step %.4g s), "
          "ToF range %.4g s (step %.4g s)"
          % (ref["n"], len(ref["grid"]), ref["t_dep_hi"], step_dep,
             ref["tof_hi"], step_tof))
    print("   cells within 5%% of best: %d / %d (%.2f%%)"
          % (near, valid, 100.0 * near / max(1, valid)))
    print("   departure profile: %d distinct minima >=2%% deep, best %.1f m/s"
          % (len(strong), ref_lo))
    print("   %-6s %11s %10s %8s %12s %11s %11s %8s"
          % ("n", "dv_min", "dv_err", "dv_err%", "t_dep_min", "t_dep_err",
             "near_best", "rank%"))
    for g in grids:
        dv_err = g["dv_min"] - ref["dv_min"]
        t_err = g["t_dep_min"] - ref["t_dep_min"]
        # One coarse cell, measured in reference cells.
        rad = max(1, ref["n"] // g["n"])
        nb = near_best(ref, g["t_dep_min"], g["tof_min"], rad)
        rank = 100.0 * bisect.bisect_left(sorted_v, g["dv_min"]) / max(1, valid)
        print("   %-6d %11.1f %10.1f %8.2f%% %12.4g %11.4g %11.1f %8.3f"
              % (g["n"], g["dv_min"], dv_err,
                 100.0 * dv_err / ref["dv_min"], g["t_dep_min"], t_err,
                 nb, rank))
    print()


if __name__ == "__main__":
    for d in sys.argv[1:]:
        main(d)
