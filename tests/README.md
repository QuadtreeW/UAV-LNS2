# Lifelong regression cases

From the project root on Ubuntu or WSL, with the usual build dependencies and Python 3 installed, run:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
python3 tests/test_lifelong.py
```

The suite launches the real CLI, writes each run to a fresh temporary directory, and checks both the schedule CSV and the global-time path TXT. Set `LNS_BIN` to test an executable in a different build directory. All cases use seed 7. The exact seeded release/rest samples and the historical trajectory were exercised with the WSL Linux build.

| Case | Input and expected behavior |
| --- | --- |
| Radius boundary | Two straight parallel routes with radius 2. One row apart, the second flight waits until step 2. Two rows apart, both take off at step 0 and follow their direct routes. |
| Arrival occupancy | A reaches B's departure pad at step 1, the same step B is released. With a one-step retry interval B takes off at 2 and arrives at 4. |
| Immutable airborne route | Capture A's published full route at horizon 0. Repeat the identical scenario with enough time for B to fly. Compare every position of A and check the two trajectories at overlapping global steps. B's direct route would enter A's cell at step 3. |
| Ground retry | Reuse the bundled 32×32 benchmark, first three aircraft, two legs each. All six finish. Flight 4 is released at 45, takes off at 47 after one retry, and arrives at 51. Its origin's squared distance to Flight 5 increases from 2 to 10 across the two attempts (threshold 4). |

### Crossing timeline observed in WSL

Coordinates are `(row, column)`. B takes off at its release time and spends one additional step at (1,3) while airborne.

| Global step | 0 | 1 | 2 | 3 | 4 | 5 | 6 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A | (2,0) | (2,1) | (2,2) | **(2,3)** | (2,4) | (2,5) | (2,6) |
| B | on ground | (0,3) | (1,3) | **(1,3)** | (2,3) | (3,3) | (4,3) |

For this case the assertions allow other valid avoidance maneuvers. They require the earlier commitment to be preserved, the new leg to finish, and the overlapping paths to respect vertex and edge-swap constraints at radius 1.

## Independently check an exported run

From the project root, supply the original map, the schedule CSV and the path TXT from the same run:

```bash
python3 tools/check_flights.py --map yuehai_fix.map \
  --schedule results/yuehai_final-lifelong.csv --paths results/yuehai_final.txt --radius 4
python3 -B -m unittest discover -s tests -v
```

The checker uses only Python's standard library. It treats only dots as traversable, matching this planner's map loader. It checks flight IDs, endpoints against the CSV, timing and metadata agreement, four-neighbor moves or waits, obstacles and coverage. For every pair of exported flights it aligns their time spans, includes both arrival steps, compares squared Euclidean distances directly and checks for edge swaps. The JSON report includes up to eight example pair-steps, the number of distinct violating flight pairs, and counts per category. A vertex hit is counted under vertex; a swap can also violate the radius. Exit status is zero for valid discrete paths, and nonzero on a violation or invalid input. Check completion counts separately. ACTIVE paths include their full committed suffix beyond the horizon. The checker evaluates integer samples and edge swaps.

## Paired Yuehai experiment

```bash
python3 -B tools/paired_radius.py --scene yuehai_fix_n20_seed1_distinct.scen --output results/paired-v1
```

Use a new output directory. If the scene is published under another name, pass that path. Scene SHA-256 for the table below: `fad58c189c946e41490777f812875cb30f7fdbcd9dd9440b7133a280a5e85acf`. Map SHA-256: `725295c2f0f0cd215f1ffb1a40004765741b3024c93da28593277e8dd7f92c35`.

The runner uses 20 aircraft, three planned legs each, a 2000-step horizon, seeds 1–10, two initial release windows, and planning radii 1 and 4. Radius 1 permits distinct adjacent cells and keeps vertex and swap constraints. The optional UAV setting `pairedTasks=true` keys each subsequent rest and destination draw by seed, aircraft and leg. It defaults to false. The runner verifies each pair's exported origins, destinations, initial releases and rest durations for all commonly generated legs. All 60 tasks matched in every measured pair. Subsequent releases still follow actual arrivals. Runs are sequential with alternating radius order across seeds. The command records each run's arguments, schedule, paths, solver output and independent radius-4 audit.

| Initial releases | Planning radius | Completed / planned | Active at horizon | Pairs violating R=4 | Violating pair-steps | Ground delay (steps) | Mean summed flight steps/run | Mean wall s/run |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| [0,30] | 1 | 596/600 | 4 | 124 | 705 | 0 | 21444.0 | 0.74 |
| [0,30] | 4 | 596/600 | 4 | 0 | 0 | 10 | 21459.0 | 0.89 |
| All at 0 | 1 | 596/600 | 4 | 101 | 886 | 5 | 21443.5 | 0.63 |
| All at 0 | 4 | 596/600 | 4 | 0 | 0 | 25 | 21457.6 | 0.81 |

Counts and ground delay are totals across ten runs per row. All 600 planned legs per row were generated and admitted, with zero deferred or ungenerated. Flight steps include airborne waits and the full paths of flights active at the horizon. Wall time covers the solver process including startup, preprocessing and export; it excludes the independent audit. It is machine-dependent. Each run passed at its own planning radius. For example, at seed 1 with releases in [0,30] and planning radius 1, flights 3 and 9 occupied (259,348) and (257,351) at step 354. Their squared distance of 13 is below 16. See the [40 individual runs](results/paired-radius.csv).
