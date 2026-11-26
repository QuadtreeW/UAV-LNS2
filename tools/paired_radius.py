"""Run paired Yuehai Lifelong experiments; store inputs' resolved settings and all exports."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import time

from check_flights import audit, read_schedule


ROOT = Path(__file__).resolve().parents[1]


def task_signature(flights):
    by_leg = {(r["aircraft_id"], r["leg_id"]): r for r in flights.values()}
    tasks = {}
    for (plane, leg), r in by_leg.items():
        previous = by_leg[plane, leg-1]["arrival_time"] if leg else 0
        tasks[plane, leg] = tuple(r[k] for k in
            ("origin_row", "origin_col", "destination_row", "destination_col")) + (r["release_time"]-previous,)
    return tasks


def run(args):
    args.output.mkdir(parents=True, exist_ok=False)
    with (args.output/"runs.csv").open("w", newline="") as out:
        writer = None
        for seed in range(1, args.seeds + 1):
            for window in (30, 0):
                signatures = []
                for radius in ((1,4) if seed % 2 else (4,1)):
                    case = args.output / f"seed{seed}-release{window}-r{radius}"
                    case.mkdir()
                    schedule, paths = case/"flights.csv", case/"paths.txt"
                    cmd = list(map(str, [args.binary, "-m", args.map, "-a", args.scene, "-k", 20,
                        "--seed", seed, "--lifelong=true", "--pairedTasks=true", "--screen=0",
                        "--simulationTime=2000", "--legsPerAircraft=3", "--restMin=10", "--restMax=30",
                        "--retryInterval=5", "--maxRetries=50", "--maxWait=600", "--maxNoProgress=600",
                        "--planningTime=5", f"--initialReleaseMax={window}", f"--safeRadius={radius}",
                        "-o", schedule, "--outputPaths", paths]))
                    (case/"command.json").write_text(json.dumps(cmd))
                    with (case/"solver.log").open("w") as log:
                        start = time.perf_counter()
                        subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=600)
                        elapsed = time.perf_counter()-start
                    flights = read_schedule(schedule)
                    signatures.append(task_signature(flights))
                    report = audit(args.map, schedule, paths, 4)
                    (case/"target4.json").write_text(json.dumps(report, indent=2))
                    own = report if radius == 4 else audit(args.map, schedule, paths, 1)
                    if own["violating_pairs"]:
                        raise ValueError(f"{case}: violations at planning radius: {own}")
                    admitted = [r for r in flights.values() if r["takeoff_time"] >= 0]
                    stats = report["statuses"]
                    row = dict(seed=seed, release_max=window, radius=radius, planned=60, generated=len(flights),
                               admitted=len(admitted), completed=stats.get("COMPLETED",0), active=stats.get("ACTIVE",0),
                               deferred=stats.get("DEFERRED",0), ungenerated=60-len(flights),
                               target4_pairs=report["violating_pairs"],
                               target4_pair_steps=sum(report["pair_steps_by_type"].values()),
                               delay_steps=sum(r["departure_delay"] for r in admitted),
                               flight_steps=sum(r["path_length"]-1 for r in admitted),
                               retries=sum(r["retries"] for r in flights.values()), wall_s=round(elapsed,4))
                    if writer is None:
                        writer = csv.DictWriter(out, fieldnames=list(row))
                        writer.writeheader()
                    writer.writerow(row)
                    out.flush()
                a,b = signatures
                common = a.keys() & b.keys()
                if any(a[k] != b[k] for k in common) or len([k for k in common if k[1]==0]) != 20:
                    raise ValueError(f"task mismatch: seed {seed}, release window {window}")
                print(f"seed={seed} release_max={window} verified {len(common)} common tasks", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=ROOT/"build/lns")
    parser.add_argument("--map", type=Path, default=ROOT/"yuehai_fix.map")
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="new output directory")
    parser.add_argument("--seeds", type=int, default=10)
    args = parser.parse_args()
    if args.seeds < 1:
        parser.error("--seeds must be positive")
    run(args)
