import unittest
import os
import subprocess
import sys

class TestEmergencyDrills(unittest.TestCase):
    """
    Integration test suite executing deterministic fault-injection emergency drills
    via the unified elevon control binary.
    """

    @classmethod
    def setUpClass(cls):
        cls.repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        cls.elevon_bin = os.path.join(cls.repo_root, "bin", "elevon")
        if not os.path.exists(cls.elevon_bin):
            subprocess.run(["make"], cwd=cls.repo_root, check=True)

    def run_drill(self, scenario_id):
        res = subprocess.run(
            [self.elevon_bin, "drill", "--scenario", str(scenario_id)],
            cwd=self.repo_root,
            capture_output=True,
            text=True
        )
        return res

    def test_drill_1_asymmetric_pitot_freeze(self):
        res = self.run_drill(1)
        self.assertEqual(res.returncode, 0)
        self.assertIn("Drill 1: Simulating Pitot Probe 1 Icing Freeze", res.stdout)
        self.assertIn("ADIRS FDI isolated Probe 1", res.stdout)
        self.assertIn("[PASS]", res.stdout)

    def test_drill_2_high_speed_exceedance(self):
        res = self.run_drill(2)
        self.assertEqual(res.returncode, 0)
        self.assertIn("Drill 2: Simulating High-Speed Dive", res.stdout)
        self.assertIn("High-speed protection active: YES", res.stdout)
        self.assertIn("[PASS]", res.stdout)

    def test_drill_3_tmr_lane_loss(self):
        res = self.run_drill(3)
        self.assertEqual(res.returncode, 0)
        self.assertIn("Drill 3: Simulating Lane B Hardware Power Cut", res.stdout)
        self.assertIn("Lane B isolated (DOWN)", res.stdout)
        self.assertIn("[PASS]", res.stdout)

    def test_cli_afdx_simulation(self):
        res = subprocess.run(
            [self.elevon_bin, "afdx", "--vl", "101", "--count", "4"],
            cwd=self.repo_root,
            capture_output=True,
            text=True
        )
        self.assertEqual(res.returncode, 0)
        self.assertIn("AFDX Summary: 4 transmitted, 4 delivered, 4 redundant duplicates dropped", res.stdout)

    def test_cli_benchmarks(self):
        res = subprocess.run(
            [self.elevon_bin, "bench"],
            cwd=self.repo_root,
            capture_output=True,
            text=True
        )
        self.assertEqual(res.returncode, 0)
        self.assertIn("Mid-Value Selection (MVS) Voting Throughput", res.stdout)
        self.assertIn("Full FCC 100 Hz Minor Cycle Latency", res.stdout)

if __name__ == "__main__":
    unittest.main()
