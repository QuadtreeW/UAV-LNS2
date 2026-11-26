"""Exercise the standalone auditor CLI on clean and deliberately corrupted exports."""
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


CHECK = Path(__file__).resolve().parents[1] / "tools/check_flights.py"


class ExportAudit(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        (self.root/"map").write_text("type octile\nheight 4\nwidth 4\nmap\n....\n....\n....\n...@\n")

    def export(self, legs):
        fields = ("flight_id aircraft_id leg_id origin_row origin_col destination_row destination_col "
                  "release_time takeoff_time arrival_time departure_delay path_length retries status reason").split()
        lines = []
        with (self.root/"schedule").open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(fields)
            for fid, (start, path) in enumerate(legs):
                end = start + len(path)-1
                writer.writerow([fid, fid, 0, *path[0], *path[-1], start, start, end, 0, len(path), 0, "COMPLETED", ""])
                head = f"Flight {fid} aircraft={fid} leg=0 release={start} takeoff={start} arrival={end}:"
                lines.append(head + "".join(f"({r},{c})->" for r,c in path))
        (self.root/"paths").write_text("\n".join(lines)+"\n")

    def check(self, radius=2):
        return subprocess.run([sys.executable, str(CHECK), "--map", "map", "--schedule", "schedule",
                               "--paths", "paths", "--radius", str(radius)], cwd=self.root,
                              capture_output=True, text=True, timeout=10)

    def test_altered_path_triggers_exact_evidence(self):
        self.export([(4, [(0,0),(0,1),(0,2)]), (5, [(2,1),(2,2),(2,3)])])
        ok = self.check()
        self.assertEqual(ok.returncode, 0, ok.stderr)  # distance exactly 2
        # Keep endpoints, path length and legal steps intact. Change only B's interior
        # point in a second clean fixture to put it one row from A at global t=6.
        self.export([(4, [(0,0),(0,1),(0,2)]), (5, [(2,2),(2,2),(2,2)])])
        self.assertEqual(self.check().returncode, 0)
        paths = self.root/"paths"
        paths.write_text(paths.read_text().replace("(2,2)->(2,2)->(2,2)", "(2,2)->(1,2)->(2,2)"))
        bad = self.check()
        self.assertEqual(bad.returncode, 1)
        self.assertEqual(json.loads(bad.stdout)["examples"], [
            dict(flights=[0,1], timestep=6, positions=[[0,2],[1,2]], types=["radius"])])

    def test_swap_and_arrival_boundary(self):
        self.export([(0, [(0,0),(0,1)]), (0, [(0,1),(0,0)])])
        self.assertEqual(json.loads(self.check(1).stdout)["pair_steps_by_type"], {"swap":1})
        self.export([(0, [(0,0),(0,1)]), (1, [(0,1),(0,2)])])
        self.assertEqual(json.loads(self.check(1).stdout)["pair_steps_by_type"], {"vertex":1})
        self.export([(0, [(0,0),(0,1)]), (2, [(0,1),(0,2)])])
        self.assertEqual(self.check(1).returncode, 0)

    def test_invalid_export(self):
        for path, message in [([(3,2),(3,3)], "blocked"), ([(0,0),(0,2)], "illegal move")]:
            with self.subTest(message=message):
                self.export([(0, path)])
                result = self.check()
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(message, result.stderr)
        self.export([(0, [(0,0),(0,1)])])
        txt = self.root/"paths"
        clean = txt.read_text()
        for changed, message in [(clean.replace("arrival=1","arrival=2"), "header disagrees"),
                                 (clean.replace("(0,1)->",""), "wrong length"),
                                 (clean.replace("(0,1)", "(1,0)"), "endpoints"),
                                 ("", "path coverage")]:
            with self.subTest(message=message):
                txt.write_text(changed)
                result = self.check()
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(message, result.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=2)
