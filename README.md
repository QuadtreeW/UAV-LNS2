# UAV-LNS2

**UAV-LNS2** is a UAV-oriented fork of [MAPF-LNS2](https://github.com/Jiaoyang-Li/MAPF-LNS2). It preserves the Large Neighborhood Search (LNS) framework and SIPP-based single-agent planning of the original project, while adding safety-radius constraints, goal disappearance, and event-driven multi-leg flight scheduling for low-altitude UAV operations.

The current implementation provides a deterministic and interpretable planning baseline for future work on UAV scheduling, Lifelong MAPF, and learning-based admission policies.

## UAV Adaptation

Classical MAPF usually prevents two agents from occupying the same vertex at the same timestep or swapping the same edge in opposite directions. It also commonly assumes that all agents start together and remain at their goals forever after arrival. These assumptions do not fully represent UAV operations.

| Classical MAPF assumption | UAV-LNS2 behavior |
| --- | --- |
| Only vertex and edge-swap conflicts are prohibited | A configurable Euclidean safety radius is enforced at integer timesteps |
| Agents remain at their goals forever | A UAV occupies its goal at arrival and leaves the airspace reservation afterward |
| All tasks start at timestep 0 | Each flight leg has its own release, takeoff, and arrival time |
| Each agent performs one task | An aircraft can land, rest, and take off again for another leg |
| All paths are planned as one static batch | New legs are admitted by events while airborne paths remain immutable |

UAV-LNS2 therefore extends both the spatial constraints and the temporal lifecycle of the original planner.

## Main UAV Extensions

### 1. Safety Radius

`safeRadius` specifies the minimum separation in grid cells. If the Euclidean distance between two UAVs at the same integer timestep is strictly smaller than this value, the state is considered unsafe. A distance exactly equal to `safeRadius` is allowed.

For a map resolution of 5 meters per grid cell:

```text
safeRadius=4  ->  allowed distance is at least 20 meters at integer timesteps
```

The PathTable stores only the cells actually occupied by UAVs. The safety neighborhood is expanded once when constraints are queried. The same representation is used by:

- PathTable vertex, edge, and goal queries;
- SIPP safe intervals;
- InitLNS and subsequent LNS planning;
- immutable ACTIVE-flight reservations in Lifelong mode;
- final global conflict validation.

This avoids double expansion of the radius and prevents overlapping safety neighborhoods from overwriting each other in the reservation table.

### 2. Goal Disappearance

In UAV safety mode, a UAV still occupies its destination at the arrival timestep. Starting from the next timestep, it no longer reserves the destination or its safety neighborhood.

This differs from the original MAPF goal-holding behavior and prevents a completed flight from blocking an airport pad indefinitely. Classic behavior remains available when `safeRadius=0`.

### 3. Event-Driven Multi-Leg Scheduling

Lifelong mode maintains a lifecycle for every flight leg:

```text
PENDING -> READY -> ACTIVE -> COMPLETED
                    |
                    +------> DEFERRED
```

The scheduler works as follows:

1. The start and goal pairs in the `.scen` file define the first leg of each aircraft.
2. The initial release time is sampled from `[0, initialReleaseMax]`.
3. When a flight arrives, its actual arrival time becomes known and a rest duration is sampled from `[restMin, restMax]`.
4. The next release time is calculated as `actual arrival time + rest time`, so no arrival time needs to be predicted in advance.
5. A new destination is selected from the union of starts and goals in the `.scen` file, excluding the aircraft's current location.
6. READY flights are ordered deterministically by waiting time, static distance, and flight ID before admission.
7. Paths of airborne ACTIVE flights are immutable hard reservations for every newly admitted leg.
8. If a safe path cannot be found, the aircraft remains on the ground and retries after `retryInterval` timesteps.

This is **event-driven full-leg planning**, not a fixed RHCR rolling window. Every admitted path is planned from its origin to its destination without cutting a flight at a window boundary.

### 4. Output and Validation

After planning, UAV-LNS2 validates all globally overlapping flight legs for:

- vertex conflicts;
- opposite edge conflicts;
- safety-radius conflicts;
- consistency of origins, destinations, path lengths, takeoff times, and arrival times.

The `-o` option writes a flight-schedule CSV containing fields such as:

```text
flight_id, aircraft_id, leg_id, origin, destination,
release_time, takeoff_time, arrival_time, departure_delay,
path_length, retries, status, reason
```

`--outputPaths` writes the complete grid path of every flight leg together with its global release, takeoff, and arrival metadata. A path file is written only after global validation succeeds.

## Build

UAV-LNS2 requires CMake, Boost, and Eigen. Install the dependencies on Ubuntu or WSL:

```bash
sudo apt update
sudo apt install -y build-essential cmake libboost-all-dev libeigen3-dev
```

Build the project:

```bash
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE ..
cmake --build . -j
```

Display all command-line options:

```bash
./lns --help
```

## Input Files

- `-m`: a 2D grid map in MovingAI format;
- `-a`: a scenario file in MovingAI `.scen` format;
- `-k`: the number of agents read from the beginning of the scenario file;
- `--uavConfig`: a UAV-LNS2 configuration file.

Map and scenario formats are documented by the [MovingAI MAPF Benchmarks](https://movingai.com/benchmarks/mapf/index.html).

The UAV configuration file intentionally accepts only options introduced by this fork. Original MAPF-LNS2 parameters such as the map, scenario, number of agents, random seed, solver, and output paths remain command-line arguments. Unknown configuration fields are rejected instead of being silently ignored.

UAV values follow this precedence:

```text
compiled defaults < UAV configuration file < explicit command-line options
```

For example, if the configuration contains `safeRadius=4`, adding `--safeRadius 5` to the command line overrides it for that run only.

## UAV Configuration

The following `yuehai_n20_lifelong.cfg` example configures 20 aircraft with up to three flight legs per aircraft:

```ini
# UAV-LNS2 configuration
uavConfigVersion=1

safeRadius=4
lifelong=true

simulationTime=2000
legsPerAircraft=3
initialReleaseMax=30

restMin=10
restMax=30
retryInterval=5
maxRetries=50
maxWait=600
maxNoProgress=600

planningTime=5
```

| Option | Default | Description |
| --- | ---: | --- |
| `uavConfigVersion` | `1` | UAV configuration schema version; currently only version 1 is supported |
| `safeRadius` | `0` | Safety radius in grid cells; Lifelong mode requires a positive value |
| `lifelong` | `false` | Enables event-driven multi-leg planning |
| `simulationTime` | `2000` | Global simulation horizon in timesteps |
| `legsPerAircraft` | `3` | Maximum number of legs generated for each aircraft, including its first leg |
| `initialReleaseMax` | `0` | Initial release time is sampled from `[0, value]` |
| `restMin` | `10` | Minimum rest time after arrival |
| `restMax` | `30` | Maximum rest time after arrival |
| `retryInterval` | `5` | Ground-delay interval between failed admission attempts |
| `maxRetries` | `100` | Maximum number of planning retries for one leg |
| `maxWait` | `600` | Maximum allowed departure delay before a leg is deferred |
| `maxNoProgress` | `600` | Termination guard when READY flights cannot make progress and no flight is ACTIVE |
| `planningTime` | `10` | Solver time limit in seconds for each admission attempt |

`--seed` remains a command-line option. It controls the generated release times, rest durations, future destinations, and solver randomness, so it should be fixed when reproducing an experiment.

## Test with 20 UAVs

The following example is executed from the `build` directory. It assumes that the map, scenario, and configuration file are stored in the project root:

```bash
mkdir -p ../results

./lns \
  -m ../yuehai_fix.map \
  -a ../yuehai_n20.scen \
  -k 20 \
  --screen 1 \
  --seed 1 \
  --uavConfig ../yuehai_n20_lifelong.cfg \
  -o ../results/yuehai_final \
  --outputPaths=../results/yuehai_final.txt
```

This experiment models a two-dimensional low-altitude operating plane at approximately 50 meters above Yuehai Subdistrict, Shenzhen. It schedules 20 UAVs with asynchronous release times and repeated flights. With three legs per aircraft, the run generates 60 flight legs in total.

The current `seed=1` result is:

```text
UAVLifelong: flights=60, completed=60, active=0, deferred=0, horizon=2000
```

- All 60 flight legs were completed within the simulation horizon.
- The final leg arrived at global timestep 1583.
- No ground delay or planning retry was required in this run.
- Global validation found no integer-timestep vertex, opposite-edge, or safety-radius conflict.

The command produces:

- `../results/yuehai_final-lifelong.csv`: the generated multi-leg flight schedule and terminal status;
- `../results/yuehai_final.txt`: complete flight paths with `release`, `takeoff`, and `arrival` metadata.

It is normal for an individual leg to report `iterations = 1`. Each leg is planned against the immutable reservations of all ACTIVE flights. If the initial SIPP/PP path already satisfies those reservations, no additional LNS repair iteration is needed.

For Lifelong regression tests, the standalone discrete trajectory checker, and the paired radius experiment, see [Tests and evaluation](tests/README.md).

## Classic MAPF Usage

The original single-task workflow remains available when Lifelong mode is disabled:

```bash
./lns \
  -m ../random-32-32-20.map \
  -a ../random-32-32-20-random-1.scen \
  -k 20 \
  -t 300 \
  -o ../results/classic_test \
  --outputPaths=../results/classic_test_paths.txt
```

`--safeRadius N` can also be added without enabling Lifelong mode to apply UAV separation and goal-disappearance behavior to a single MAPF instance.

## Author

UAV-LNS2 is developed and maintained by
[QuadtreeW](https://github.com/QuadtreeW).

The UAV-specific extensions include safety-radius constraints, goal
disappearance, event-driven multi-leg scheduling, immutable airborne
reservations, and UAV configuration support.

## Acknowledgements

UAV-LNS2 is based on MAPF-LNS2 developed by Jiaoyang Li and Zhe Chen and retains its original license. Some rule-based MAPF solvers in the project are based on [PIBT](https://github.com/Kei18/pibt/tree/v1.3). See `license.txt` before using or redistributing this fork.

## References

1. Jiaoyang Li, Zhe Chen, Daniel Harabor, Peter J. Stuckey, and Sven Koenig. *MAPF-LNS2: Fast Repairing for Multi-Agent Path Finding via Large Neighborhood Search*. AAAI, 2022.
2. Jiaoyang Li, Zhe Chen, Daniel Harabor, Peter J. Stuckey, and Sven Koenig. *Anytime Multi-Agent Path Finding via Large Neighborhood Search*. IJCAI, 2021.
