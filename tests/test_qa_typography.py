import unittest
import os
import subprocess
import sys

class TestQaTypography(unittest.TestCase):
    """
    Quality assurance test ensuring that no em dashes or improper dashes exist
    anywhere in the repository source, comments, or documentation.
    """

    def test_no_em_dashes_in_repository(self):
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        script_path = os.path.join(repo_root, "tools", "lint_no_em_dash.py")
        self.assertTrue(os.path.exists(script_path), "lint_no_em_dash.py script missing")

        res = subprocess.run(
            [sys.executable, script_path],
            cwd=repo_root,
            capture_output=True,
            text=True
        )
        self.assertEqual(
            res.returncode, 0,
            f"Zero em dash invariant violated:\n{res.stderr}\n{res.stdout}"
        )

if __name__ == "__main__":
    unittest.main()
