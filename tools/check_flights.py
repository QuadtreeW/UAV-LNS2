"""Audit integer-step Lifelong exports, independently of the planner. Python 3."""
import argparse
from collections import Counter
import csv
from itertools import combinations
import json
from pathlib import Path
import re
import sys


HEADER = re.compile(r"Flight (\d+) aircraft=(\d+) leg=(\d+) release=(\d+) takeoff=(\d+) arrival=(\d+)")
POINT = re.compile(r"\((-?\d+),(-?\d+)\)->")
FIELDS = ("flight_id aircraft_id leg_id release_time takeoff_time arrival_time "
          "origin_row origin_col destination_row destination_col departure_delay path_length retries").split()


def require(ok, message):
    if not ok:
        raise ValueError(message)


def read_map(filename):
    lines = Path(filename).read_text().splitlines()
    if lines and lines[0].startswith("type "):
        require(len(lines) >= 4 and lines[3] == "map", "invalid MovingAI map header")
        require(lines[1].startswith("height ") and lines[2].startswith("width "), "invalid dimensions")
        height, width = int(lines[1][7:]), int(lines[2][6:])
        grid = lines[4:]
    else:
        height, width = map(int, (lines or [""])[0].split(","))
        grid = lines[1:]
    require(width > 0 and height == len(grid) > 0 and all(len(r) == width for r in grid),
            "map dimensions disagree")
    return grid


def read_schedule(filename):
    flights = {}
    legs = set()
    with Path(filename).open(newline="") as stream:
        rows = csv.DictReader(stream)
        require(set(FIELDS + ["status"]) <= set(rows.fieldnames or []), "missing CSV columns")
        for row in rows:
            row.update({k: int(row[k]) for k in FIELDS})
            fid = row["flight_id"]
            key = row["aircraft_id"], row["leg_id"]
            require(fid not in flights and key not in legs, f"duplicate flight or leg: {fid}, {key}")
            release, start, end = (row[k] for k in ("release_time", "takeoff_time", "arrival_time"))
            require(min(fid, *key, release, row["retries"]) >= 0, f"invalid identifiers/time: {fid}")
            if row["status"] == "DEFERRED":
                require((start, end, row["departure_delay"], row["path_length"]) == (-1, -1, -1, 0),
                        f"flight {fid}: invalid DEFERRED metadata")
            else:
                require(row["status"] in {"ACTIVE", "COMPLETED"} and release <= start <= end
                        and row["departure_delay"] == start-release and row["path_length"] == end-start+1,
                        f"flight {fid}: inconsistent schedule")
            flights[fid] = row
            legs.add(key)
    require(flights, "empty schedule")
    return flights


def audit(map_file, schedule_file, path_file, radius, sample_limit=8):
    require(radius >= 0, "radius must be nonnegative")
    grid, flights = read_map(map_file), read_schedule(schedule_file)
    paths = {}
    for line_no, line in enumerate(Path(path_file).read_text().splitlines(), 1):
        head, _, body = line.partition(":")
        match = HEADER.fullmatch(head)
        require(match, f"invalid path header at line {line_no}")
        fid, aircraft, leg, release, start, end = map(int, match.groups())
        require(fid in flights and fid not in paths, f"unknown or repeated path: {fid}")
        row = flights[fid]
        require([aircraft, leg, release, start, end] == [row[k] for k in
                ("aircraft_id", "leg_id", "release_time", "takeoff_time", "arrival_time")],
                f"flight {fid}: path header disagrees with CSV")
        coords = []
        offset = 0
        while offset < len(body):
            point = POINT.match(body, offset)
            require(point, f"flight {fid}: invalid coordinate at character {offset}")
            r, c = map(int, point.groups())
            t = start + len(coords)
            require(0 <= r < len(grid) and 0 <= c < len(grid[0]) and grid[r][c] == ".",
                    f"flight {fid} t={t}: blocked or outside map at {(r,c)}")
            if coords:
                pr, pc = coords[-1]
                require(abs(r-pr) + abs(c-pc) <= 1, f"flight {fid} t={t}: illegal move {coords[-1]} -> {(r,c)}")
            coords.append((r,c))
            offset = point.end()
        require(len(coords) == end-start+1 and coords[0] == (row["origin_row"], row["origin_col"])
                and coords[-1] == (row["destination_row"], row["destination_col"]),
                f"flight {fid}: wrong length or endpoints")
        paths[fid] = coords
    require(set(paths) == {fid for fid, row in flights.items() if row["takeoff_time"] >= 0},
            "path coverage disagrees with admitted flights")
    counts, examples, violating_pairs = Counter(), [], 0
    for (a, pa), (b, pb) in combinations(paths.items(), 2):
        ta, tb = flights[a]["takeoff_time"], flights[b]["takeoff_time"]
        pair_hit = False
        for t in range(max(ta,tb), min(ta+len(pa),tb+len(pb))):
            i, j = t-ta, t-tb
            p, q = pa[i], pb[j]
            kinds = []
            if p == q:
                kinds.append("vertex")
            elif (p[0]-q[0])**2 + (p[1]-q[1])**2 < radius**2:
                kinds.append("radius")
            if i and j and p != q and pa[i-1] == q and pb[j-1] == p:
                kinds.append("swap")
            if kinds:
                pair_hit = True
                counts.update(kinds)
                if len(examples) < sample_limit:
                    examples.append(dict(flights=[a,b], timestep=t, positions=[p,q], types=kinds))
        violating_pairs += pair_hit
    return dict(checked_paths=len(paths), statuses=dict(Counter(r["status"] for r in flights.values())),
                violating_pairs=violating_pairs, pair_steps_by_type=dict(counts), examples=examples)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("map", "schedule", "paths"):
        parser.add_argument("--"+key, required=True, type=Path)
    parser.add_argument("--radius", type=int, required=True, help="minimum distance in grid cells")
    args = parser.parse_args()
    try:
        report = audit(args.map, args.schedule, args.paths, args.radius)
    except (ValueError, OSError, TypeError, csv.Error) as exc:
        sys.exit(f"Invalid export: {exc}")
    print(json.dumps(report, indent=2))
    sys.exit(int(report["violating_pairs"] > 0))
