import os
import sys
import tempfile
import unittest
from pathlib import Path


TESTS_DIR = os.path.dirname(__file__)
PROJECT_ROOT = os.path.abspath(os.path.join(TESTS_DIR, ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.tools.visualize_lerobot_dataset import validate_dataset  # noqa: E402


class LeRobotVisualizerTests(unittest.TestCase):
    def test_sample_v21_dataset_validates(self):
        try:
            import pandas  # noqa: F401
        except ImportError:
            self.skipTest("pandas is required to inspect LeRobot parquet files")

        dataset_root = Path(PROJECT_ROOT) / "datasets" / "local" / "quest3-acone"
        if not dataset_root.exists():
            self.skipTest("sample dataset is not present")

        issues = validate_dataset(dataset_root, "local/quest3-acone", 0)

        self.assertFalse([issue for issue in issues if issue.level == "error"])

    def test_missing_v21_paths_are_errors(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            dataset_root = Path(temp_dir) / "local" / "bad"
            (dataset_root / "meta").mkdir(parents=True)
            (dataset_root / "meta" / "info.json").write_text('{"codebase_version": "v2.1"}', encoding="utf-8")

            issues = validate_dataset(Path(temp_dir), "local/bad", 0)

        self.assertTrue(any("Missing required path" in issue.message for issue in issues))


if __name__ == "__main__":
    unittest.main()
