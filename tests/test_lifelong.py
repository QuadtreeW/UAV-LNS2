"""Black-box regressions for the Linux/WSL lns executable (Python standard library)."""
import csv
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
LNS = Path(os.environ.get("LNS_BIN", ROOT / "build" / "lns")).resolve()
HEADER = re.compile(r"Flight (\d+) aircraft=(\d+) leg=(\d+) release=(\d+) takeoff=(\d+) arrival=(\d+)")


class LifelongRegression(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        self.assertTrue(LNS.is_file(), f"Build lns or set LNS_BIN: {LNS}")

    def open_grid(self, rows, cols, legs):
        """legs use (row,col) pairs. MovingAI writes column first."""
        grid = self.directory / "open.map"
        scene = self.directory / "open.scen"
        grid.write_text(f"type octile\nheight {rows}\nwidth {cols}\nmap\n" + ("." * cols + "\n") * rows)
        records = ["version 1"]
        for (r, c), (rr, cc) in legs:
            records.append(f"0\topen.map\t{cols}\t{rows}\t{c}\t{r}\t{cc}\t{rr}\t{abs(rr-r)+abs(cc-c)}")
        scene.write_text("\n".join(records) + "\n")
        return grid, scene

    def simulate(self, inputs, count, **changes):
        grid, scene = inputs
        options = dict(lifelong="true", safeRadius=2, simulationTime=100, legsPerAircraft=1,
                       initialReleaseMax=0, retryInterval=1, planningTime=3)
        options.update(changes)
        with tempfile.TemporaryDirectory(dir=self.directory) as output:
            prefix = Path(output) / "run"
            paths_file = Path(output) / "paths.txt"
            cmd = [str(LNS), "-m", str(grid), "-a", str(scene), "-k", str(count), "--seed", "7",
                   "-o", str(prefix), "--outputPaths", str(paths_file)]
            cmd.extend(f"--{k}={v}" for k, v in options.items())
            run = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            with Path(str(prefix) + "-lifelong.csv").open(newline="") as f:
                records = list(csv.DictReader(f))
            flights = {int(row["flight_id"]): row for row in records}
            self.assertEqual(len(flights), len(records))
            paths = {}
            for line in paths_file.read_text().splitlines():
                header, coords = line.split(":", 1)
                match = HEADER.fullmatch(header)
                self.assertIsNotNone(match, line)
                fid, *meta = map(int, match.groups())
                row = flights[fid]
                self.assertEqual(meta, [int(row[key]) for key in
                                        ("aircraft_id", "leg_id", "release_time", "takeoff_time", "arrival_time")])
                self.assertNotIn(fid, paths)
                paths[fid] = [(int(r), int(c)) for r, c in re.findall(r"\((\d+),(\d+)\)->", coords)]
                self.assertEqual(len(paths[fid]), meta[-1] - meta[-2] + 1)
            self.assertEqual(set(paths), {fid for fid, row in flights.items() if int(row["takeoff_time"]) >= 0})
            return flights, paths

    def completed(self, flights, expected):
        self.assertEqual(len(flights), expected)
        self.assertTrue(all(row["status"] == "COMPLETED" for row in flights.values()), flights)

    def timing(self, row):
        return tuple(int(row[k]) for k in ("release_time", "takeoff_time", "arrival_time", "retries"))

    def test_parallel_lanes_radius_boundary(self):
        for gap in (1, 2):
            with self.subTest(gap=gap):
                inputs = self.open_grid(3, 5, [((0, 0), (0, 4)), ((gap, 0), (gap, 4))])
                flights, paths = self.simulate(inputs, 2)
                self.completed(flights, 2)
                self.assertEqual(self.timing(flights[0]), (0, 0, 4, 0))
                # At t=1 A is at (0,1): diagonal distance sqrt(2) still blocks B in lane 1.
                delay = 2 if gap == 1 else 0
                self.assertEqual(self.timing(flights[1]), (0, delay, delay + 4, delay))
                for fid, lane in ((0, 0), (1, gap)):
                    self.assertEqual(paths[fid], [(lane, c) for c in range(5)])

    def test_pad_occupied_on_arrival_released_next_step(self):
        inputs = self.open_grid(2, 4, [((0, 0), (0, 1)), ((0, 1), (0, 3))])
        flights, paths = self.simulate(inputs, 2, safeRadius=1, initialReleaseMax=5)
        self.completed(flights, 2)
        self.assertEqual(self.timing(flights[0]), (0, 0, 1, 0))
        self.assertEqual(self.timing(flights[1]), (1, 2, 4, 1))
        self.assertEqual(paths[0][-1], paths[1][0])

    def test_airborne_commitment_survives_crossing_request(self):
        inputs = self.open_grid(5, 7, [((2, 0), (2, 6)), ((0, 3), (4, 3))])
        opts = dict(safeRadius=1, initialReleaseMax=5)
        # Stop at A's admission and capture its entire committed path before B is released.
        early, original = self.simulate(inputs, 2, simulationTime=0, **opts)
        self.assertEqual(early[0]["status"], "ACTIVE")
        self.assertEqual(set(original), {0})
        flights, paths = self.simulate(inputs, 2, **opts)
        self.completed(flights, 2)
        self.assertEqual(paths[0], original[0])
        self.assertEqual(self.timing(flights[0]), (0, 0, 6, 0))
        self.assertEqual(paths[0][3], (2, 3))  # B's direct route, released at 1, also reaches here at 3.
        release, takeoff, arrival, _ = self.timing(flights[1])
        self.assertEqual(release, 1)
        self.assertGreater(arrival, release + 4)
        self.assertEqual((paths[1][0], paths[1][-1]), ((0, 3), (4, 3)))
        for t in range(takeoff, min(arrival, 6) + 1):
            j = t - takeoff
            self.assertNotEqual(paths[0][t], paths[1][j], f"vertex conflict at {t}")
            if j > 0:
                self.assertNotEqual((paths[0][t-1], paths[0][t]), (paths[1][j], paths[1][j-1]))

    def test_historical_ground_retry(self):
        inputs = ROOT / "random-32-32-20.map", ROOT / "random-32-32-20-random-1.scen"
        flights, paths = self.simulate(inputs, 3, simulationTime=300, legsPerAircraft=2,
                                      initialReleaseMax=5, restMin=5, restMax=10, retryInterval=2)
        self.completed(flights, 6)
        self.assertEqual(self.timing(flights[4]), (45, 47, 51, 1))
        self.assertEqual(int(flights[4]["departure_delay"]), 2)
        start = paths[4][0]
        self.assertEqual(start, (23, 28))
        takeoff = int(flights[5]["takeoff_time"])
        for t, expected_squared_distance in ((45, 2), (47, 10)):
            other = paths[5][t - takeoff]
            self.assertEqual(sum((a-b)**2 for a, b in zip(start, other)), expected_squared_distance)


if __name__ == "__main__":
    unittest.main(verbosity=2)
