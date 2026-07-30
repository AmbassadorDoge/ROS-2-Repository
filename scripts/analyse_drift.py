#!/usr/bin/env python3
"""Turn scripted_drive CSVs into the three numbers worth having.

    python3 scripts/analyse_drift.py runs/*.csv

1. Position error per 90 deg of rotation, isolated by comparing paths that cover
   the same translation with different amounts of counted rotation.
2. Effective-track factor: wheel-reported yaw change divided by true yaw change,
   per turn. This is the correction real skid-steer odometry needs, and it is
   currently an unmeasured guess in drivebase_description/urdf/gazebo.xacro.
3. Whether the GPS-corrected global filter beats the local one once turns are
   involved. On straight-line travel it did not.

Deliberately stdlib-only: no numpy in the ROS container by default, and this is
not worth a dependency.
"""

import math
import pathlib
import statistics
import sys


def read_csv(path):
    lines = [
        line.strip() for line in pathlib.Path(path).read_text().splitlines()
        if line.strip() and not line.startswith("#")
    ]
    header = lines[0].split(",")
    rows = []
    for line in lines[1:]:
        parts = line.split(",")
        row = {}
        for key, value in zip(header, parts):
            if key in ("leg_type",):
                row[key] = value
            else:
                try:
                    row[key] = float(value)
                except ValueError:
                    row[key] = math.nan
        rows.append(row)
    path_name = "unknown"
    for line in pathlib.Path(path).read_text().splitlines():
        if line.startswith("# path="):
            path_name = line.split("=", 1)[1].strip()
    return path_name, rows


def error(row, prefix):
    return math.hypot(row[f"{prefix}_x"] - row["true_x"],
                      row[f"{prefix}_y"] - row["true_y"])


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def final_errors(rows):
    """Error over the last settled stretch, so a mid-turn transient is not the
    headline number."""
    tail = [r for r in rows if r["leg_type"] == "pause"][-8:] or rows[-8:]
    local = statistics.median(error(r, "loc") for r in tail)
    globals_ = [error(r, "glob") for r in tail
                if not math.isnan(r["glob_x"])]
    return local, (statistics.median(globals_) if globals_ else math.nan)


def track_factors(rows):
    """One ratio per turn leg: wheel yaw change / true yaw change."""
    factors = []
    current = None
    for row in rows:
        if row["leg_type"] in ("turn", "orient"):
            if current is None or current["leg"] != row["leg"]:
                current = {"leg": row["leg"], "rows": []}
                factors.append(current)
            current["rows"].append(row)
        else:
            current = None
    out = []
    for leg in factors:
        first, last = leg["rows"][0], leg["rows"][-1]
        true_delta = abs(angle_diff(last["true_yaw"], first["true_yaw"]))
        wheel_delta = abs(angle_diff(last["odom_yaw"], first["odom_yaw"]))
        # Skip turns too small to divide meaningfully, and any turn near pi where
        # the wrapped difference is ambiguous.
        if true_delta < 0.3 or true_delta > 2.8 or wheel_delta > 2.8:
            continue
        out.append(wheel_delta / true_delta)
    return out


def main(paths):
    by_path = {}
    all_factors = []

    print(f"{'file':<26} {'path':<12} {'dist':>7} {'rot':>7} "
          f"{'local':>8} {'global':>8} {'rows':>6} {'nan':>5}")
    print("-" * 88)
    for path in paths:
        name, rows = read_csv(path)
        if not rows:
            print(f"{pathlib.Path(path).name:<26} EMPTY")
            continue
        local, glob = final_errors(rows)
        distance = rows[-1]["cum_dist"]
        rotation = math.degrees(rows[-1]["cum_rot"])
        gaps = sum(1 for r in rows if math.isnan(r["loc_x"])
                   or math.isnan(r["true_x"]))
        print(f"{pathlib.Path(path).name:<26} {name:<12} {distance:7.1f} "
              f"{rotation:7.0f} {local:8.3f} "
              f"{glob if not math.isnan(glob) else float('nan'):8.3f} "
              f"{len(rows):6d} {gaps:5d}")
        by_path.setdefault(name, {"local": [], "global": [], "rot": []})
        by_path[name]["local"].append(local)
        if not math.isnan(glob):
            by_path[name]["global"].append(glob)
        by_path[name]["rot"].append(rotation)
        all_factors += track_factors(rows)

    print()
    print("=== median error by path ===")
    summary = {}
    for name, data in sorted(by_path.items()):
        local = statistics.median(data["local"])
        rot = statistics.median(data["rot"])
        glob = statistics.median(data["global"]) if data["global"] else math.nan
        summary[name] = (rot, local, glob)
        print(f"  {name:<12} rotation {rot:6.0f} deg   "
              f"local {local:6.3f} m   global {glob:6.3f} m   "
              f"n={len(data['local'])}")

    # Error per 90 deg, taking the lowest-rotation path as the baseline so that
    # straight-line error is subtracted out.
    if len(summary) >= 2:
        baseline_name = min(summary, key=lambda k: summary[k][0])
        base_rot, base_local, base_global = summary[baseline_name]
        print()
        print(f"=== error attributable to rotation (baseline: {baseline_name}) ===")
        for name, (rot, local, glob) in sorted(summary.items(),
                                               key=lambda kv: kv[1][0]):
            quarters = (rot - base_rot) / 90.0
            if quarters <= 0.01:
                continue
            per_local = (local - base_local) / quarters
            line = (f"  {name:<12} +{rot - base_rot:5.0f} deg "
                    f"({quarters:4.1f} quarter-turns)   "
                    f"local {per_local:+.3f} m per 90 deg")
            if not math.isnan(glob) and not math.isnan(base_global):
                line += f"   global {(glob - base_global) / quarters:+.3f}"
            print(line)

    print()
    print("=== effective-track factor (wheel yaw / true yaw) ===")
    if all_factors:
        print(f"  n={len(all_factors)}  median={statistics.median(all_factors):.3f}  "
              f"mean={statistics.mean(all_factors):.3f}  "
              f"min={min(all_factors):.3f}  max={max(all_factors):.3f}")
        if len(all_factors) > 1:
            print(f"  stdev={statistics.stdev(all_factors):.3f}")
        print(f"  -> wheel_separation for matching odometry: "
              f"{0.430 * statistics.median(all_factors):.3f} m "
              f"(physical track 0.430 m)")
    else:
        print("  no usable turns found")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    main(sys.argv[1:])
